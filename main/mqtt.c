/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "mqtt.h"

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "mqtt_client.h"

#include "panasonic_state.h"

#define TOPIC_PREFIX "panasonic/"

static const char TAG[] = "MQTT_EXAMPLE";

static char unique_id[13];
static char discovery_topic[50];

static const char discovery_data[] = ""
"{\n"
"  \"~\":\""TOPIC_PREFIX"%s\",\n"
"  \"name\":\"Panasonic HVAC\",\n"
"  \"uniq_id\":\"%s\",\n"
//"  \"avty_t\":\"~/available\",\n"
//"  \"pl_avail\":\"online\",\n"
//"  \"pl_not_avail\":\"offline\",\n"
"  \"mode_cmd_t\":\"~/mode/set\",\n"
"  \"mode_stat_t\":\"~\",\n"
"  \"mode_stat_tpl\":\"{{value_json.mode}}\",\n"
//"  \"modes\":[\"auto\",\"off\",\"cool\",\"heat\",\"dry\",\"fan_only\"],\n"
"  \"temp_cmd_t\":\"~/temperature/set\",\n"
"  \"temp_stat_t\":\"~\",\n"
"  \"temp_stat_tpl\":\"{{value_json.temperature}}\",\n"
"  \"fan_mode_cmd_t\":\"~/fan/set\",\n"
"  \"fan_mode_stat_t\":\"~\",\n"
"  \"fan_mode_stat_tpl\":\"{{value_json.fan}}\",\n"
"  \"fan_modes\":[\"auto\",\"min\",\"low\",\"medium\",\"high\",\"max\"],\n"
"  \"swing_mode_cmd_t\":\"~/swing/set\",\n"
"  \"swing_mode_stat_t\":\"~\",\n"
"  \"swing_mode_stat_tpl\":\"{{value_json.swing}}\",\n"
"  \"swing_modes\":[\"auto\",\"forward\",\"high\",\"middle\",\"low\",\"down\"],\n"
"  \"min_temp\":\"8\",\n"
"  \"max_temp\":\"31\",\n"
//"  \"temp_step\":\"1\",\n"
"  \"dev\":{\n"
"    \"ids\":\"%s\",\n"
"    \"mdl\":\"CS-NE9LKE\",\n"
"    \"sw\":\"%s\"\n"
"  }\n"
"}";
#define DISCOVERY_FORMAT(unique_id) discovery_data, unique_id, unique_id, unique_id, esp_ota_get_app_description()->version

static esp_mqtt_client_handle_t client;

static int string_to_mode(enum mode *mode, const char *s, int len)
{
	if (strncmp("auto", s, len) == 0) {
		*mode = MODE_AUTO;
		return 1;
	} else if (strncmp("cool", s, len) == 0) {
		*mode = MODE_COOL;
		return 1;
	} else if (strncmp("dry", s, len) == 0) {
		*mode = MODE_DRY;
		return 1;
	} else if (strncmp("fan_only", s, len) == 0) {
		*mode = MODE_FAN;
		return 1;
	} else if (strncmp("heat", s, len) == 0) {
		*mode = MODE_HEAT;
		return 1;
	}
	return -1;
}

static int string_to_fan(enum fan *fan, const char *s, int len)
{
	if (strncmp("auto", s, len) == 0) {
		*fan = FAN_AUTO;
		return 1;
	} else if (strncmp("min", s, len) == 0) {
		*fan = FAN_1;
		return 1;
	} else if (strncmp("low", s, len) == 0) {
		*fan = FAN_2;
		return 1;
	} else if (strncmp("medium", s, len) == 0) {
		*fan = FAN_3;
		return 1;
	} else if (strncmp("high", s, len) == 0) {
		*fan = FAN_4;
		return 1;
	} else if (strncmp("max", s, len) == 0) {
		*fan = FAN_5;
		return 1;
	}
	return -1;
}

static int string_to_swing(enum swing *swing, const char *s, int len)
{
	if (strncmp("auto", s, len) == 0) {
		*swing = SWING_AUTO;
		return 1;
	} else if (strncmp("forward", s, len) == 0) {
		*swing = SWING_1;
		return 1;
	} else if (strncmp("high", s, len) == 0) {
		*swing = SWING_2;
		return 1;
	} else if (strncmp("middle", s, len) == 0) {
		*swing = SWING_3;
		return 1;
	} else if (strncmp("low", s, len) == 0) {
		*swing = SWING_4;
		return 1;
	} else if (strncmp("down", s, len) == 0) {
		*swing = SWING_5;
		return 1;
	}
	return -1;
}

static bool ends_with(const char *a, int alen, const char *end)
{
	size_t endlen = strlen(end);

	return alen >= endlen && strncmp(a + (alen - endlen), end, endlen) == 0;
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
	esp_mqtt_client_handle_t client = event->client;
	int msg_id;
	char buf[sizeof(discovery_data) + 12 + 12 + 12 + 32 + 10];

	switch (event->event_id) {
	case MQTT_EVENT_CONNECTED:
		ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
		msg_id = esp_mqtt_client_subscribe(client, TOPIC_PREFIX"restart", 0);
		ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

		snprintf(buf, sizeof(buf), TOPIC_PREFIX"%s/mode/set", unique_id);
		msg_id = esp_mqtt_client_subscribe(client, buf, 0);
		ESP_LOGI(TAG, "subscribed to %s, msg_id=%d", buf, msg_id);

		snprintf(buf, sizeof(buf), TOPIC_PREFIX"%s/temperature/set", unique_id);
		msg_id = esp_mqtt_client_subscribe(client, buf, 0);
		ESP_LOGI(TAG, "subscribed to %s, msg_id=%d", buf, msg_id);

		snprintf(buf, sizeof(buf), TOPIC_PREFIX"%s/fan/set", unique_id);
		msg_id = esp_mqtt_client_subscribe(client, buf, 0);
		ESP_LOGI(TAG, "subscribed to %s, msg_id=%d", buf, msg_id);

		snprintf(buf, sizeof(buf), TOPIC_PREFIX"%s/swing/set", unique_id);
		msg_id = esp_mqtt_client_subscribe(client, buf, 0);
		ESP_LOGI(TAG, "subscribed to %s, msg_id=%d", buf, msg_id);

		snprintf(buf, sizeof(buf), DISCOVERY_FORMAT(unique_id));
		msg_id = esp_mqtt_client_publish(client, discovery_topic, buf, 0, 0, 1);
		ESP_LOGI(TAG, "published to %s, msg_id=%d", discovery_topic, msg_id);
		break;
	case MQTT_EVENT_DISCONNECTED:
		ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
		break;

	case MQTT_EVENT_SUBSCRIBED:
		ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
		break;
	case MQTT_EVENT_UNSUBSCRIBED:
		ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
		break;
	case MQTT_EVENT_PUBLISHED:
		ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
		break;
	case MQTT_EVENT_DATA:
		ESP_LOGI(TAG, "MQTT_EVENT_DATA");
		if (strncmp(event->topic, TOPIC_PREFIX"restart", event->topic_len) == 0) {
			ESP_LOGI(TAG, "Rebooting ...");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			esp_restart();
		} else if (ends_with(event->topic, event->topic_len, "/mode/set")) {
			if (strncmp("off", event->data, event->data_len) == 0) {
				panasonic_set_mode(false, MODE_AUTO);
			} else {
				enum mode mode;
				if (string_to_mode(&mode, event->data, event->data_len) > 0) {
					ESP_LOGI(TAG, "Mode to %d", mode);
					panasonic_set_mode(true, mode);
				} else {
					ESP_LOGI(TAG, "Unknown mode");
				}
			}
		} else if (ends_with(event->topic, event->topic_len, "/temperature/set")) {
			long temp = strtol(event->data, NULL, 10);
			panasonic_set_temperature(temp);
		} else if (ends_with(event->topic, event->topic_len, "/fan/set")) {
			enum fan fan;
			if (string_to_fan(&fan, event->data, event->data_len) > 0) {
				ESP_LOGI(TAG, "Fan to %d", fan);
				panasonic_set_fan(fan);
			} else {
				ESP_LOGI(TAG, "Unknown fan");
			}
		} else if (ends_with(event->topic, event->topic_len, "/swing/set")) {
			enum swing swing;
			if (string_to_swing(&swing, event->data, event->data_len) > 0) {
				ESP_LOGI(TAG, "Swing to %d", swing);
				panasonic_set_swing(swing);
			} else {
				ESP_LOGI(TAG, "Unknown swing");
			}
		} else {
			printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
			printf("DATA=%.*s\r\n", event->data_len, event->data);
		}
		break;
	case MQTT_EVENT_ERROR:
		ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
		break;
	default:
		ESP_LOGI(TAG, "Other event id:%d", event->event_id);
		break;
	}
	return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
	ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
	mqtt_event_handler_cb(event_data);
}

void mqtt_init(const char *device_id)
{
	snprintf(unique_id, sizeof(unique_id), "%s", device_id);
	snprintf(discovery_topic, sizeof(discovery_topic), "homeassistant/climate/%s/config", device_id);

	esp_mqtt_client_config_t mqtt_cfg = {
		.uri = CONFIG_BROKER_URL,
	};
#if CONFIG_BROKER_URL_FROM_STDIN
	char line[128];

	if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
		int count = 0;
		printf("Please enter url of mqtt broker\n");
		while (count < 128) {
			int c = fgetc(stdin);
			if (c == '\n') {
				line[count] = '\0';
				break;
			} else if (c > 0 && c < 127) {
				line[count] = c;
				++count;
			}
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		mqtt_cfg.uri = line;
		printf("Broker url: %s\n", line);
	} else {
		ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
		abort();
	}
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

	client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
	esp_mqtt_client_start(client);
}

int mqtt_pub(const char *suffix, const char *data, int len, int qos, int retain)
{
	char topic[32];
	if (client == NULL) {
		return -1;
	}

	snprintf(topic, sizeof(topic), TOPIC_PREFIX"%s%s", unique_id, suffix);
	return esp_mqtt_client_publish(client, topic, data, len, qos, retain);
}
