#ifndef PANASONIC_STATE_H
#define PANASONIC_STATE_H

#include "panasonic_frame.h"
#include <stdbool.h>
#include <stddef.h>

void panasonic_state_init(void);
void panasonic_set_state(const struct panasonic_command *cmd);
void panasonic_set_temperature(int temperature);
void panasonic_set_mode(bool power, enum mode mode);
void panasonic_set_power(bool on);
void panasonic_set_fan(enum fan fan);
void panasonic_set_swing(enum swing swing);
int panasonic_state_to_json(char *str, size_t maxlen, const struct panasonic_command *cmd);

#endif /* PANASONIC_STATE_H */
