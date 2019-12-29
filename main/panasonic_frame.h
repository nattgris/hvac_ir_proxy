#ifndef PANASONIC_FRAME_H
#define PANASONIC_FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct panasonic_command {
	enum cmd {
		CMD_STATE     = 0,
		CMD_E_ION     = 0x3361,
		CMD_PATROL    = 0x3363,
		CMD_QUIET     = 0x3381,
		CMD_POWERFUL  = 0x3586,
		CMD_CHECK     = 0x3293,
		CMD_SET_AIR_1 = 0x328D,
		CMD_SET_AIR_2 = 0x328E,
		CMD_SET_AIR_3 = 0x328F,
		CMD_AC_RESET  = 0x9D32,
	} cmd;
	enum mode {
		MODE_AUTO = 0,
		MODE_DRY  = 2,
		MODE_COOL = 3,
		MODE_HEAT = 4,
		MODE_FAN  = 6,
	} mode;
	enum swing {
		SWING_1 = 1, /* H */
		SWING_2 = 2,
		SWING_3 = 3,
		SWING_4 = 4,
		SWING_5 = 5, /* V */
		SWING_AUTO = 0xF,
	} swing;
	enum fan {
		FAN_1 = 3,
		FAN_2 = 4,
		FAN_3 = 5,
		FAN_4 = 6,
		FAN_5 = 7,
		FAN_AUTO = 0xA,
	} fan;
	uint16_t on_time;
	uint16_t off_time;
	uint16_t time;
	uint8_t temp;
	bool on :1;
	bool on_timer :1;
	bool off_timer :1;
	bool no_time :1;
};

int panasonic_parse_frame(struct panasonic_command *cmd, const uint8_t *data, int len);
int panasonic_build_frame(const struct panasonic_command *cmd, uint8_t *data, size_t size);

#endif /* PANASONIC_FRAME_H */
