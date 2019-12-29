#ifndef MQTT_H
#define MQTT_H

void mqtt_init(void);
int mqtt_pub(const char *topic, const char *data, int len, int qos, int retain);

#endif /* MQTT_H */
