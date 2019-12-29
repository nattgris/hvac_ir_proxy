#ifndef PANASONIC_IR_H
#define PANASONIC_IR_H

#include "panasonic_frame.h"

void panasonic_ir_init(void (*receiver)(const struct panasonic_command *cmd, void *priv), void *priv);
void panasonic_transmit(const struct panasonic_command *cmd);

#endif /* PANASONIC_IR_H */
