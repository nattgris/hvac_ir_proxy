/* Panasonic AC remote infrared transceiver/MQTT bridge

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
#include "mqtt.h"
#include "panasonic_frame.h"
#include "panasonic_state.h"

static const char TAG[] = "IR";

/******************************************************/
/*****                SELF TEST:                  *****/
/*Connect RMT_TX_GPIO_NUM with RMT_RX_GPIO_NUM        */
/*TX task will send data with carrier disabled    */
/*RX task will print data it receives.            */
/******************************************************/
#define RMT_RX_ACTIVE_LEVEL  1   /*!< If we connect with a IR receiver, the data is active low */
#define RMT_TX_ACTIVE_LEVEL  1   /*!< If we connect with a IR receiver, the data is active low */
#define RMT_TX_CARRIER_EN    0   /*!< Enable carrier for IR transmitter test with IR led */

#define RMT_TX_CHANNEL    4     /*!< RMT channel for transmitter */
#define RMT_TX_GPIO_NUM  13     /*!< GPIO number for transmitter signal */
#define RMT_RX_CHANNEL    0     /*!< RMT channel for receiver */
#define RMT_RX_GPIO_NUM  14     /*!< GPIO number for receiver */
#define RMT_CLK_DIV      80    /*!< RMT counter clock divider for µs ticks */

#define HEADER_MARK_US    3543          /*!< Panasonic protocol header */
#define HEADER_SPACE_US   1700          /*!< Panasonic protocol header */
#define MARK_US            400          /*!< Panasonic protocol mark */
#define BIT_ONE_SPACE_US  1340          /*!< Panasonic protocol space bit 1 */
#define BIT_ZERO_SPACE_US  470          /*!< Panasonic protocol space bit 0 */
#define IDLE_US          10400          /*!< Panasonic protocol interframe spacing */
#define BIT_MARGIN         150          /*!< Panasonic parse margin time */

#define ITEM_DURATION(d)  (d & 0x7fff)  /*!< Parse duration time from memory register value */
#define RMT_ITEM32_TIMEOUT_US  4000   /*!< RMT receiver timeout value(us) */

enum pana_item {
	PANA_INVALID = -1,
	PANA_BIT_0,
	PANA_BIT_1,
	PANA_HEADER,
	PANA_END
};

static const uint8_t header[] = {0x02, 0x20, 0xE0, 0x04, 0x00, 0x00, 0x00, 0x06};
static void (*receive_cb)(const struct panasonic_command *cmd, void *priv);
static void *receive_priv;
/*
 * @brief Build register value of waveform for one item
 */
static inline void fill_item_level(rmt_item32_t* item, int mark_us, int space_us)
{
	item->level0 = !RMT_TX_ACTIVE_LEVEL;
	item->duration0 = space_us;
	item->level1 = RMT_TX_ACTIVE_LEVEL;
	item->duration1 = mark_us;
}

/*
 * @brief Generate 1st header item; interframe spacing, followed by long mark
 */
static void fill_item_header1(rmt_item32_t* item)
{
	fill_item_level(item, HEADER_MARK_US, IDLE_US);
}

/*
 * @brief Generate 2nd header item; like a normal bit but 4 units of space
 */
static void fill_item_header2(rmt_item32_t* item)
{
	fill_item_level(item, MARK_US, HEADER_SPACE_US);
}

/*
 * @brief Generate data bit 1: 3 units of space before a mark
 */
static void fill_item_bit_one(rmt_item32_t* item)
{
	fill_item_level(item, MARK_US, BIT_ONE_SPACE_US);
}

/*
 * @brief Generate data bit 0: 1 unit of space before a mark
 */
static void fill_item_bit_zero(rmt_item32_t* item)
{
	fill_item_level(item, MARK_US, BIT_ZERO_SPACE_US);
}

/*
 * @brief Generate end item
 */
static void fill_item_end(rmt_item32_t* item)
{
	item->level0 = !RMT_TX_ACTIVE_LEVEL;
	item->duration0 = 0;
	item->level1 = !RMT_TX_ACTIVE_LEVEL;
	item->duration1 = 0;
}

static uint16_t mark_ticks(const rmt_item32_t *item)
{
	return item->level0 == RMT_RX_ACTIVE_LEVEL ? item->duration0 : item->duration1;
}

static uint16_t space_ticks(const rmt_item32_t *item)
{
	return item->level0 == RMT_RX_ACTIVE_LEVEL ? item->duration1 : item->duration0;
}

/*
 * @brief Check whether duration is around target
inline bool in_range(int duration, int target, int margin)
{
	return duration < target + margin && duration > target - margin;
}
 */

/*
 * @brief Decode an item into the corresponding symbol
 */
static enum pana_item decode_item(const rmt_item32_t* item)
{
	uint16_t mark = mark_ticks(item);
	uint16_t space = space_ticks(item);

	if (space == 0) {
		return PANA_END;
	}

	if ((mark > 2700 && space < mark) || (space > 1600 && mark < space)) {
	/*if (in_range(space, HEADER_SPACE_US, BIT_MARGIN)
	 || in_range(mark, HEADER_MARK_US, BIT_MARGIN)) {*/
		return PANA_HEADER;
	}

	if (mark < MARK_US - BIT_MARGIN || mark > MARK_US + BIT_MARGIN) {
		return PANA_INVALID;
	}

	if (space < mark * 2) {
		return PANA_BIT_0;
	} else {
		return PANA_BIT_1;
	}
}

struct panasonic_parser {
	uint8_t data;
	uint8_t *buf;
	size_t bufsize;
	int bitcount;
	size_t bytecount;
	bool in_frame;
};

/*
 * @brief Parse Panasonic data array.
 */
static int panasonic_parse_items(struct panasonic_parser *p, const rmt_item32_t* i)
{
	//ESP_LOGI(TAG, "RMT RCV %5u %u %5u %u", i->duration0, i->level0, i->duration1, i->level1);

	enum pana_item pi = decode_item(i);

	if (pi == PANA_HEADER) {
		//ESP_LOGI(TAG, "RMT RCV START %5u %u %5u %u", i->duration0, i->level0, i->duration1, i->level1);
		p->bitcount = 0;
		p->bytecount = 0;
		p->in_frame = true;
		return 0;
	} else if (pi == PANA_END) {
		//ESP_LOGI(TAG, "RMT RCV END, %d bytes, %d bits", p->bytecount, p->bitcount);
		int ret = p->bitcount == 0 ? p->bytecount : -1;
		p->bitcount = 0;
		p->bytecount = 0;
		p->in_frame = false;
		return ret;
	} else if (pi == PANA_INVALID) {
		//ESP_LOGI(TAG, "RMT RCV INV %5u %u %5u %u", i->duration0, i->level0, i->duration1, i->level1);
		p->in_frame = false;
		return -1;
	} else if (p->in_frame) {
		/* Bit received in frame, shift in data */
		p->data = (p->data >> 1) | (pi == PANA_BIT_1 ? 1 << 7 : 0);

		if (++p->bitcount == 8) {
			p->bitcount = 0;
			if (p->bytecount < p->bufsize) {
				//ESP_LOGI(TAG, "RMT RCV --- %02x", p->data);
				p->buf[p->bytecount] = p->data;
			} else {
				//ESP_LOGI(TAG, "RMT OVF --- %02x", p->data);
				p->in_frame = false;
				return -1;
			}
			p->bytecount++;
		}
	} else {
		//ESP_LOGW(TAG, "RMT Not in frame %5u %u %5u %u", i->duration0, i->level0, i->duration1, i->level1);
	}

	return 0;
}

static void panasonic_transmit_frame(const uint8_t *data, int len)
{
	rmt_item32_t *item = calloc(2 + sizeof(header)*8 + 2 + len * 8 + 1, sizeof(*item));
	int n = 0;

	if (item == NULL) {
		ESP_LOGE(TAG, "Not enough heap");
		return;
	}

	fill_item_header1(&item[n++]);
	fill_item_header2(&item[n++]);

	for (int i = 0; i < sizeof(header); i++) {
		uint8_t d = header[i];
		for (int b = 0; b < 8; b++) {
			if (d & 1) {
				fill_item_bit_one(&item[n++]);
			} else {
				fill_item_bit_zero(&item[n++]);
			}
			d >>= 1;
		}
	}

	fill_item_header1(&item[n++]);
	fill_item_header2(&item[n++]);

	for (int i = 0; i < len; i++) {
		uint8_t d = data[i];
		for (int b = 0; b < 8; b++) {
			if (d & 1) {
				fill_item_bit_one(&item[n++]);
			} else {
				fill_item_bit_zero(&item[n++]);
			}
			d >>= 1;
		}
	}

	fill_item_end(&item[n++]);

	rmt_write_items(RMT_TX_CHANNEL, item, n, true);
	//rmt_fill_tx_items(RMT_TX_CHANNEL, item, n, 0);
	//rmt_tx_start(RMT_TX_CHANNEL, true);
	rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
	//rmt_tx_stop(RMT_TX_CHANNEL);
	free(item);

}

void panasonic_transmit(const struct panasonic_command *cmd)
{
	uint8_t data[19];
	int ret;
	char s[sizeof(data) * 3 + 1];
	size_t len = 0;

	ret = panasonic_build_frame(cmd, data, sizeof(data));
	if (ret < 0) {
		return;
	}

	for (int i = 0; i < ret; i++) {
		len += snprintf(s + len, sizeof(s) - len, "%02x ", data[i]);
	}
	ESP_LOGI(TAG, "XMT %s", s);

	panasonic_transmit_frame(data, ret);
}

/**
 * @brief RMT receiver task.
 *
 */
static void panasonic_rx_task()
{
	int channel = RMT_RX_CHANNEL;
	RingbufHandle_t rb = NULL;
	//get RMT RX ringbuffer
	rmt_get_ringbuf_handle(channel, &rb);
	rmt_rx_start(channel, true);

	uint8_t data[19];
	struct panasonic_parser p = { .buf = data, .bufsize = sizeof(data) };
	struct panasonic_command cmd;

	while(1) {
		size_t rx_size = 0;
		//try to receive data from ringbuffer.
		//RMT driver will push all the data it receives to its ringbuffer.
		//We just need to parse the value and return the spaces of ringbuffer.
		const rmt_item32_t* item = (const rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
		if (item) {
			int ret = 0;
			for (const rmt_item32_t* i = item; rx_size >= sizeof(*i); i++, rx_size -= 4) {
				//parse data value from ringbuffer.
				ret = panasonic_parse_items(&p, i);

				if (ret > 0 && ret <= sizeof(data)) {
					char s[sizeof(data) * 3 + 1];
					size_t len = 0;

					for (int i = 0; i < ret; i++) {
						len += snprintf(s + len, sizeof(s) - len, "%02x ", data[i]);
					}
					ESP_LOGI(TAG, "RCV %s", s);

					ret = panasonic_parse_frame(&cmd, data, ret);
					if (ret > 0) {
						ESP_LOGI(TAG, "Call receive");
						receive_cb(&cmd, receive_priv);
					}
				} else if (ret < 0) {
					ESP_LOGE(TAG, "Error");
				}
			}
			//after parsing the data, return spaces to ringbuffer.
			vRingbufferReturnItem(rb, (void*) item);
		}
	}

	ESP_LOGI(TAG, "Exiting");
	vTaskDelete(NULL);
}

/*
 * @brief RMT transmitter initialization
 */
static void tx_init()
{
	rmt_config_t rmt_tx;
	rmt_tx.channel = RMT_TX_CHANNEL;
	rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
	rmt_tx.mem_block_num = 1;
	rmt_tx.clk_div = RMT_CLK_DIV;
	rmt_tx.tx_config.loop_en = false;
	rmt_tx.tx_config.carrier_duty_percent = 50;
	rmt_tx.tx_config.carrier_freq_hz = 38000;
	rmt_tx.tx_config.carrier_level = RMT_TX_ACTIVE_LEVEL;
	rmt_tx.tx_config.carrier_en = RMT_TX_CARRIER_EN;
	rmt_tx.tx_config.idle_level = !RMT_TX_ACTIVE_LEVEL;
	rmt_tx.tx_config.idle_output_en = true;
	rmt_tx.rmt_mode = RMT_MODE_TX;
	rmt_config(&rmt_tx);
	rmt_driver_install(rmt_tx.channel, 0, 0);
}

/*
 * @brief RMT receiver initialization
 */
static void rx_init()
{
	rmt_config_t rmt_rx;
	rmt_rx.channel = RMT_RX_CHANNEL;
	rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
	rmt_rx.clk_div = RMT_CLK_DIV;
	rmt_rx.mem_block_num = 4;
	rmt_rx.rmt_mode = RMT_MODE_RX;
	rmt_rx.rx_config.filter_en = true;
	rmt_rx.rx_config.filter_ticks_thresh = 255;
	rmt_rx.rx_config.idle_threshold = RMT_ITEM32_TIMEOUT_US;
	rmt_config(&rmt_rx);
	rmt_driver_install(rmt_rx.channel, 4000, 0);
}

void panasonic_ir_init(void (*receiver)(const struct panasonic_command *cmd, void *priv), void *priv)
{
	receive_cb = receiver;
	receive_priv = priv;
	tx_init();
	rx_init();
	xTaskCreate(panasonic_rx_task, "rmt_rx_task", 2048, NULL, 10, NULL);
}
