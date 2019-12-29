#include "panasonic_state.h"
#include "panasonic_ir.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "mqtt.h"
#include <stdbool.h>

static const char TAG[] = "PANA";

static struct panasonic_command state;
static SemaphoreHandle_t state_mutex;

static const char *mode_to_string(enum mode mode)
{
	switch (mode) {
	case MODE_AUTO:
		return "auto";
	case MODE_COOL:
		return "cool";
	case MODE_DRY:
		return "dry";
	case MODE_FAN:
		return "fan_only";
	case MODE_HEAT:
		return "heat";
	}
	return "invalid";
}

static const char *fan_to_string(enum fan fan)
{
	switch (fan) {
	case FAN_AUTO:
		return "auto";
	case FAN_1:
		return "min";
	case FAN_2:
		return "low";
	case FAN_3:
		return "medium";
	case FAN_4:
		return "high";
	case FAN_5:
		return "max";
	}
	return "invalid";
}

static const char *swing_to_string(enum swing swing)
{
	switch (swing) {
	case SWING_AUTO:
		return "auto";
	case SWING_1:
		return "forward";
	case SWING_2:
		return "high";
	case SWING_3:
		return "middle";
	case SWING_4:
		return "low";
	case SWING_5:
		return "down";
	}

	return "invalid";
}

const char *command_to_string(enum cmd cmd)
{
	switch (cmd) {
	case CMD_E_ION:
		return "E-ion";
	case CMD_PATROL:
		return "Patrol";
	case CMD_QUIET:
		return "Quiet";
	case CMD_POWERFUL:
		return "Powerful";
	case CMD_CHECK:
		return "Check";
	case CMD_SET_AIR_1:
		return "Set_Air_1";
	case CMD_SET_AIR_2:
		return "Set_Air_2";
	case CMD_SET_AIR_3:
		return "Set_Air_3";
	case CMD_AC_RESET:
		return "AC_Reset";
	default:
		return "invalid";
	}
}

static int panasonic_send_mqtt(const struct panasonic_command *cmd, const char *topic, int qos, int retain)
{
	const int maxlen = 100;
	char *s = malloc(maxlen);
	int ret;
	int len;

	if (cmd->cmd == CMD_STATE) {
		len = panasonic_state_to_json(s, maxlen, cmd);
	} else {
		len = snprintf(s, maxlen, "%s", command_to_string(cmd->cmd));
	}

	if (len > 0 && len < maxlen) {
		ESP_LOGI(TAG, "Publish \"%s\"", s);
		ret = mqtt_pub(topic, s, len, qos, retain);
	} else {
		ESP_LOGE(TAG, "Buffer too small, needed %d bytes", len);
		ret = -1;
	}

	free(s);

	return ret;
}

static void panasonic_send_state(void)
{
	panasonic_transmit(&state);
	panasonic_send_mqtt(&state, "panasonic/state", 0, 0);
}

void panasonic_set_state(const struct panasonic_command *cmd)
{
	xSemaphoreTake(state_mutex, portMAX_DELAY);
	state = *cmd;
	panasonic_send_state();
	xSemaphoreGive(state_mutex);
}

void panasonic_set_temperature(int temperature)
{
	xSemaphoreTake(state_mutex, portMAX_DELAY);
	state.temp = temperature < 0 ? 0 : temperature > 31 ? 31 : temperature;
	state.no_time = true;
	panasonic_send_state();
	xSemaphoreGive(state_mutex);
}

void panasonic_set_mode(enum mode mode)
{
	xSemaphoreTake(state_mutex, portMAX_DELAY);
	state.mode = mode;
	state.no_time = true;
	panasonic_send_state();
	xSemaphoreGive(state_mutex);
}

void panasonic_set_power(bool on)
{
	xSemaphoreTake(state_mutex, portMAX_DELAY);
	state.on = on;
	state.no_time = true;
	panasonic_send_state();
	xSemaphoreGive(state_mutex);
}

void panasonic_set_fan(enum fan fan)
{
	xSemaphoreTake(state_mutex, portMAX_DELAY);
	state.fan = fan;
	state.no_time = true;
	panasonic_send_state();
	xSemaphoreGive(state_mutex);
}

void panasonic_set_swing(enum swing swing)
{
	xSemaphoreTake(state_mutex, portMAX_DELAY);
	state.swing = swing;
	state.no_time = true;
	panasonic_send_state();
	xSemaphoreGive(state_mutex);
}

int panasonic_state_to_json(char *str, size_t size, const struct panasonic_command *cmd)
{
	if (cmd->cmd == CMD_STATE) {
		return snprintf(str, size, "{\"mode\":\"%s\",\"temperature\":\"%d\",\"fan\":\"%s\",\"swing\":\"%s\"}",
		                cmd->on ? mode_to_string(cmd->mode) : "off",
		                cmd->temp,
		                fan_to_string(cmd->fan),
		                swing_to_string(cmd->swing));
	}

	return snprintf(str, size, "%s", "");
}

void panasonic_state_init(void)
{
	state_mutex = xSemaphoreCreateMutex();
}
