#ifndef CAN_TELEMETRY_H
#define CAN_TELEMETRY_H

#include <stdint.h>


int can_telemetry_init(const char* interface_name);

void can_send_engine_data(int socket_fd, uint16_t rpm, uint8_t pedal_position);

void can_send_wheel_speeds(int socket_fd, double fl_kmh, double fr_kmh, double rl_kmh, double rr_kmh);


#endif