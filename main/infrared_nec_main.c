/* NEC remote infrared RMT example

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

static const char* NEC_TAG = "NEC";

/******************************************************/
/*****                SELF TEST:                  *****/
/*Connect RMT_TX_GPIO_NUM with RMT_RX_GPIO_NUM        */
/*TX task will send NEC data with carrier disabled    */
/*RX task will print NEC data it receives.            */
/******************************************************/
#define RMT_RX_ACTIVE_LEVEL  0   /*!< If we connect with a IR receiver, the data is active low */
#define RMT_TX_ACTIVE_LEVEL  0   /*!< If we connect with a IR receiver, the data is active low */
#define RMT_TX_CARRIER_EN    0   /*!< Enable carrier for IR transmitter test with IR led */

#define RMT_TX_CHANNEL    1     /*!< RMT channel for transmitter */
#define RMT_TX_GPIO_NUM  14     /*!< GPIO number for transmitter signal */
#define RMT_RX_CHANNEL    0     /*!< RMT channel for receiver */
#define RMT_RX_GPIO_NUM  16     /*!< GPIO number for receiver */
#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define HEADER_MARK_US    3469          /*!< NEC protocol header */
#define HEADER_SPACE_US   1773          /*!< NEC protocol header */
#define BIT_ONE_MARK_US    400          /*!< NEC protocol data bit 1 */
#define BIT_ONE_SPACE_US  1333          /*!< NEC protocol data bit 1 */
#define BIT_ZERO_MARK_US   400          /*!< NEC protocol data bit 0 */
#define BIT_ZERO_SPACE_US  473          /*!< NEC protocol data bit 0 */
#define BIT_END_US         400          /*!< NEC protocol end */
#define BIT_MARGIN         100          /*!< NEC parse margin time */

#define ITEM_DURATION(d)  ((d & 0x7fff)*10/RMT_TICK_10_US)  /*!< Parse duration time from memory register value */
#define RMT_ITEM32_TIMEOUT_US  4000   /*!< RMT receiver timeout value(us) */

/*
 * @brief Build register value of waveform for NEC one data bit
 */
static inline void nec_fill_item_level(rmt_item32_t* item, int mark_us, int space_us)
{
    item->level0 = RMT_TX_ACTIVE_LEVEL;
    item->duration0 = (mark_us) / 10 * RMT_TICK_10_US;
    item->level1 = !RMT_TX_ACTIVE_LEVEL;
    item->duration1 = (space_us) / 10 * RMT_TICK_10_US;
}

/*
 * @brief Generate NEC header value: active 9ms + negative 4.5ms
 */
static void nec_fill_item_header(rmt_item32_t* item)
{
    nec_fill_item_level(item, HEADER_MARK_US, HEADER_SPACE_US);
}

/*
 * @brief Generate NEC data bit 1: positive 0.56ms + negative 1.69ms
 */
static void nec_fill_item_bit_one(rmt_item32_t* item)
{
    nec_fill_item_level(item, BIT_ONE_MARK_US, BIT_ONE_SPACE_US);
}

/*
 * @brief Generate NEC data bit 0: positive 0.56ms + negative 0.56ms
 */
static void nec_fill_item_bit_zero(rmt_item32_t* item)
{
    nec_fill_item_level(item, BIT_ZERO_MARK_US, BIT_ZERO_SPACE_US);
}

/*
 * @brief Generate NEC end signal: positive 0.56ms
 */
static void nec_fill_item_end(rmt_item32_t* item)
{
    nec_fill_item_level(item, BIT_END_US, 0x7fff);
}

static int mark_ticks(const rmt_item32_t *item)
{
    return item->level0 == RMT_RX_ACTIVE_LEVEL ? item->duration0 : item->duration1;
}

static int space_ticks(const rmt_item32_t *item)
{
    return item->level0 == RMT_RX_ACTIVE_LEVEL ? item->duration1 : item->duration0;
}

/*
 * @brief Check whether duration is around target_us
 */
inline bool nec_check_in_range(int duration_ticks, int target_us, int margin_us)
{
    if(( ITEM_DURATION(duration_ticks) < (target_us + margin_us))
        && ( ITEM_DURATION(duration_ticks) > (target_us - margin_us))) {
        return true;
    } else {
        return false;
    }
}

/*
 * @brief Check whether this value represents an NEC header
 */
static bool nec_header_if(const rmt_item32_t* item)
{
    return nec_check_in_range(space_ticks(item), HEADER_SPACE_US, BIT_MARGIN);
}

/*
 * @brief Check whether this value represents an NEC data bit 1
 */
static bool nec_bit_one_if(const rmt_item32_t* item)
{
    return nec_check_in_range(space_ticks(item), BIT_ONE_SPACE_US, BIT_MARGIN);
}

/*
 * @brief Check whether this value represents an NEC data bit 0
 */
static bool nec_bit_zero_if(const rmt_item32_t* item)
{
    return nec_check_in_range(space_ticks(item), BIT_ZERO_SPACE_US, BIT_MARGIN);
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
    //ESP_LOGI(NEC_TAG, "RMT RCV %5u %u %5u %u", i->duration0, i->level0, i->duration1, i->level1);
    if (i == NULL || space_ticks(i) == 0) {
        int ret = p->bitcount == 0 ? p->bytecount : -1;
        p->bitcount = 0;
        p->bytecount = 0;
        p->in_frame = false;
        //ESP_LOGI(NEC_TAG, "RMT RCV END");
        return ret;
    } else if (nec_header_if(i)) {
        p->bitcount = 0;
        p->bytecount = 0;
        p->in_frame = true;
        //ESP_LOGI(NEC_TAG, "RMT RCV HEADER");
    } else if (p->in_frame) {
        bool bit;
        if (nec_bit_one_if(i)) {
            //ESP_LOGI(NEC_TAG, "RMT RCV 1");
            bit = true;
        } else if (nec_bit_zero_if(i)) {
            //ESP_LOGI(NEC_TAG, "RMT RCV 0");
            bit = false;
        } else {
            //ESP_LOGI(NEC_TAG, "RMT RCV INVALID %5u %u %5u %u", i->duration0, i->level0, i->duration1, i->level1);
            p->in_frame = false;
            return -1;
        }

        p->data = (p->data >> 1) | (bit ? 1 << 7 : 0);

        if (++p->bitcount == 8) {
            p->bitcount = 0;
            // p-> data = (p->data >> 4) | (p->data << 4);
            if (p->bytecount < p->bufsize) {
                p->buf[p->bytecount] = p->data;
                //ESP_LOGI(NEC_TAG, "RMT RCV --- %02x", p->data);
            } else {
                ESP_LOGI(NEC_TAG, "RMT OVF --- %02x", p->data);
            }
            p->bytecount++;
        }
    } else {
        ESP_LOGI(NEC_TAG, "RMT Not a header %5u %u %5u %u", i->duration0, i->level0, i->duration1, i->level1);
    }

    return 0;
}

/*
 * @brief Build NEC 32bit waveform.
 */
static int nec_build_items(int channel, rmt_item32_t* item, int item_num, uint16_t addr, uint16_t cmd_data)
{
    return 0;
}

/*
 * @brief RMT transmitter initialization
 */
static void nec_tx_init()
{
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = RMT_TX_CARRIER_EN;
    rmt_tx.tx_config.idle_level = 0;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

/*
 * @brief RMT receiver initialization
 */
static void nec_rx_init()
{
    rmt_config_t rmt_rx;
    rmt_rx.channel = RMT_RX_CHANNEL;
    rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
    rmt_rx.clk_div = RMT_CLK_DIV;
    rmt_rx.mem_block_num = 4;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 255;
    rmt_rx.rx_config.idle_threshold = RMT_ITEM32_TIMEOUT_US / 10 * (RMT_TICK_10_US);
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 4000, 0);
}

/**
 * @brief RMT receiver demo, this task will print each received NEC data.
 *
 */
static void rmt_example_nec_rx_task()
{
    int channel = RMT_RX_CHANNEL;
    nec_rx_init();
    RingbufHandle_t rb = NULL;
    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(channel, &rb);
    rmt_rx_start(channel, 1);

    uint8_t data[19];
    struct panasonic_parser p = { .buf = data, .bufsize = sizeof(data) };

    while(1) {
        size_t rx_size = 0;
        //try to receive data from ringbuffer.
        //RMT driver will push all the data it receives to its ringbuffer.
        //We just need to parse the value and return the spaces of ringbuffer.
        const rmt_item32_t* item = (const rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
        if (item) {
            int ret = 0;
            if (rx_size == 0) {
                ret = panasonic_parse_items(&p, NULL);
            } else for (const rmt_item32_t* i = item; rx_size >= sizeof(*i); i++, rx_size -= 4) {
                //parse data value from ringbuffer.
                ret = panasonic_parse_items(&p, i);
            }

            if (ret > 0 && ret <= sizeof(data)) {
                char s[sizeof(data) * 3 + 1];
                size_t len = 0;

                uint8_t sum = 0;
                for (int i = 0; i < ret - 1; i++) {
                    len += snprintf(s + len, sizeof(s) - len, "%02x ", data[i]);
                    sum += data[i];
                }
                ESP_LOGI(NEC_TAG, "RCV %s %s", s, sum == data[ret-1] ? "OK" : "CSUM ERR");

            } else if (ret < 0) {
                ESP_LOGI(NEC_TAG, "Error");
            }
            //after parsing the data, return spaces to ringbuffer.
            vRingbufferReturnItem(rb, (void*) item);
        }
    }

    ESP_LOGI(NEC_TAG, "Exiting");
    vTaskDelete(NULL);
}

/**
 * @brief RMT transmitter demo, this task will periodically send NEC data. (100 * 32 bits each time.)
 *
 */
static void rmt_example_nec_tx_task()
{
    vTaskDelay(10);
    nec_tx_init();
    esp_log_level_set(NEC_TAG, ESP_LOG_INFO);
    int channel = RMT_TX_CHANNEL;
    uint16_t cmd = 0x0;
    uint16_t addr = 0x11;
    for(;;) {
        ESP_LOGI(NEC_TAG, "RMT TX DATA");
        size_t size = (sizeof(rmt_item32_t) * 0 * 0);
        //each item represent a cycle of waveform.
        rmt_item32_t* item = (rmt_item32_t*) malloc(size);
        int item_num = 0;
        memset((void*) item, 0, size);
        int i, offset = 0;
        while(1) {
            //To build a series of waveforms.
            i = nec_build_items(channel, item + offset, item_num - offset, ((~addr) << 8) | addr, ((~cmd) << 8) |  cmd);
            if(i < 0) {
                break;
            }
            cmd++;
            addr++;
            offset += i;
        }
        //To send data according to the waveform items.
        rmt_write_items(channel, item, item_num, true);
        //Wait until sending is done.
        rmt_wait_tx_done(channel, portMAX_DELAY);
        //before we free the data, make sure sending is already done.
        free(item);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(rmt_example_nec_rx_task, "rmt_nec_rx_task", 2048, NULL, 10, NULL);
    //xTaskCreate(rmt_example_nec_tx_task, "rmt_nec_tx_task", 2048, NULL, 10, NULL);
}
