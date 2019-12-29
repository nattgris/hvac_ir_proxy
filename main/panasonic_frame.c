#include "panasonic_frame.h"
#include "esp_log.h"
#include <string.h>
#include <assert.h>

static const char TAG[] = "PANA";

static const uint8_t header[] = { 0x02, 0x20, 0xE0, 0x04 };

static uint8_t sum(const uint8_t *data, int len)
{
	uint8_t sum = 0;
	for (int i = 0; i < len; i++) {
		sum += data[i];
	}
	return sum;
}

int panasonic_parse_frame(struct panasonic_command *cmd, const uint8_t *data, int len)
{
	if (len != 19 && len != 8) {
		ESP_LOGW(TAG, "Invalid length %d", len);
		return -1;
	}

	if (sum(data, len - 1) != data[len - 1]) {
		ESP_LOGW(TAG, "Invalid checksum");
		return -1;
	}

	if (memcmp(data, header, sizeof(header)) !=0) {
		ESP_LOGW(TAG, "Invalid header");
		return -1;
	}

	if (len == 8 && (data[4] & 0x80) == 0) {
		return 0; /* Header found */
	}

	memset(cmd, 0, sizeof(*cmd));

	cmd->cmd = len == 19 ? CMD_STATE : (data[6] << 8) | data[5];

	switch (cmd->cmd) {
	case CMD_STATE:
		break;
	case CMD_E_ION:
	case CMD_PATROL:
	case CMD_QUIET:
	case CMD_POWERFUL:
	case CMD_CHECK:
	case CMD_SET_AIR_1:
	case CMD_SET_AIR_2:
	case CMD_SET_AIR_3:
	case CMD_AC_RESET:
		return 1;
	default:
		ESP_LOGW(TAG, "Invalid command %d", cmd->cmd);
		return -1;
	}

	assert(len == 19);

	cmd->mode = data[5] >> 4;

	switch (cmd->mode) {
	case MODE_AUTO:
	case MODE_COOL:
	case MODE_DRY:
	case MODE_FAN:
	case MODE_HEAT:
		break;
	default:
		ESP_LOGW(TAG, "Invalid mode %d", cmd->mode);
		return -1;
	}

	cmd->off_timer = (data[5] & 4) != 0;
	cmd->on_timer  = (data[5] & 2) != 0;
	cmd->on        = (data[5] & 1) != 0;

	cmd->temp = (data[6] >> 1) & 0x1F;

	cmd->swing = data[8] & 0x0F;
	cmd->fan = data[8] >> 4;

	switch (cmd->swing) {
	case SWING_AUTO:
	case SWING_1:
	case SWING_2:
	case SWING_3:
	case SWING_4:
	case SWING_5:
		break;
	default:
		ESP_LOGW(TAG, "Invalid swing mode %d", cmd->swing);
		return -1;
	}

	switch (cmd->fan) {
	case FAN_AUTO:
	case FAN_1:
	case FAN_2:
	case FAN_3:
	case FAN_4:
	case FAN_5:
		break;
	default:
		ESP_LOGW(TAG, "Invalid fan mode %d", cmd->fan);
		return -1;
	}

	/* TODO: Support time settings. */

	return 1;
}

int panasonic_build_frame(const struct panasonic_command *cmd, uint8_t *data, size_t size)
{
	if (cmd == NULL || (cmd->cmd == CMD_STATE && size < 19) || size < 8) {
		return -1;
	}

	memcpy(data, header, sizeof(header));

	if (cmd->cmd != CMD_STATE) {
		data[4] = 0x80;
		data[5] = cmd->cmd;
		data[6] = cmd->cmd >> 8;
		data[7] = sum(data, 7);

		return 8;
	}

	bool no_time = cmd->time == 0 || cmd->no_time;
	uint16_t off_time = no_time ? 0x600 : cmd->off_time;
	uint16_t on_time  = no_time ? 0x600 : cmd->on_time;
	uint16_t time     = no_time ? 0 : cmd->time;

	data[4] = 0x00;
	data[5] = (cmd->mode << 4) | (1 << 3) | (cmd->off_timer << 2) | (cmd->on_timer << 1) | cmd->on;
	data[6] = cmd->temp << 1;
	data[7] = 0x80;
	data[8] = (cmd->fan << 4) | cmd->swing;
	data[9] = 0x00;
	data[10] = on_time;
	data[11] = ((off_time & 0x03) << 4) | (1 << 3) | (on_time >> 8);
	data[12] = (1 << 7) | (off_time >> 4);
	data[13] = 0x00;
	data[14] = 0x00;
	data[15] = 0x80 | no_time;
	data[16] = time;
	data[17] = time >> 8;
	data[18] = sum(data, 18);

	return 19;
}
