#ifndef NETWORK_CORE_H
#define NETWORK_CORE_H

#include <stdint.h>
#include <stdbool.h>


typedef struct {
    uint8_t header;
    float speed;
    float rpm;
} __attribute__((packed)) TelemetryPacket;

typedef struct {
    float target_speed;
    float brake_intensity;
} __attribute__((packed)) ControlPacket;


bool init_network(const char* ip_address, int port);
void send_telemetry(double speed, double rpm);
bool receive_control(ControlPacket* out_packet);
void close_network(void);

#endif



