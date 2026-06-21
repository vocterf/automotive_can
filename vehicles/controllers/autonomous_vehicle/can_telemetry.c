#include "can_telemetry.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int can_telemetry_init(const char *interface_name) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("[CAN INIT ERROR]: Failed to open socket");
        return -1;
    }

    strncpy(ifr.ifr_name, interface_name, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("[CAN INIT ERROR]: ioctl interface lookup failed");
        close(s);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[CAN INIT ERROR]: Socket bind failed");
        close(s);
        return -1;
    }

    printf("[CAN INIT]: Successfully connected to %s\n", interface_name);
    return s;
}

void can_send_engine_data(int socket_fd, uint16_t rpm, uint8_t pedal_position) {
    if (socket_fd < 0) return;

    struct can_frame frame;
    frame.can_id = 0x110;
    frame.can_dlc = 3;

    // Big-Endian (Motorola) bit packing
    frame.data[0] = (rpm >> 8) & 0xFF;
    frame.data[1] = rpm & 0xFF;
    frame.data[2] = pedal_position;

    if (write(socket_fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("[CAN TX ERROR]: Failed to write EngineData");
    }
}

void can_send_wheel_speeds(int socket_fd, double fl_kmh, double fr_kmh, double rl_kmh, double rr_kmh) {
    if (socket_fd < 0) return;

    struct can_frame frame;
    frame.can_id = 0x215;
    frame.can_dlc = 8;

    // Skalowanie wartości fizycznych (f32 * 100.0) do surowego u16
    uint16_t fl_raw = (uint16_t)(fl_kmh * 100.0);
    uint16_t fr_raw = (uint16_t)(fr_kmh * 100.0);
    uint16_t rl_raw = (uint16_t)(rl_kmh * 100.0);
    uint16_t rr_raw = (uint16_t)(rr_kmh * 100.0);

    // Big-Endian (Motorola) bit packing dla wszystkich 4 kół
    frame.data[0] = (fl_raw >> 8) & 0xFF; frame.data[1] = fl_raw & 0xFF;
    frame.data[2] = (fr_raw >> 8) & 0xFF; frame.data[3] = fr_raw & 0xFF;
    frame.data[4] = (rl_raw >> 8) & 0xFF; frame.data[5] = rl_raw & 0xFF;
    frame.data[6] = (rr_raw >> 8) & 0xFF; frame.data[7] = rr_raw & 0xFF;

    if (write(socket_fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("[CAN TX ERROR]: Failed to write AbsWheelSpeeds");
    }
}