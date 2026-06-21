#include "network_core.h"
#include <stdio.h>
#include <winsock2.h>
#include <math.h>

// Globalne, prywatne zmienne modułu sieciowego
static SOCKET udp_socket = INVALID_SOCKET;
static struct sockaddr_in server_addr;

bool init_network(const char* ip_address, int port) {
    WSADATA wsa;
    
    // 1. Inicjalizacja biblioteki Winsock2
    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
        printf("[WINSOCK ERR] Blad inicjalizacji Winsock2.\n");
        return false;
    }

    // 2. Utworzenie surowego gniazda UDP
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket == INVALID_SOCKET) {
        printf("[WINSOCK ERR] Nie mozna utworzyc gniazda UDP.\n");
        WSACleanup();
        return false;
    }

    // 3. Konfiguracja gniazda w tryb NIEBLOKUJĄCY
    u_long mode = 1; // 1 = włącz tryb nieblokujący
    ioctlsocket(udp_socket, FIONBIO, &mode);

    // 4. Definicja adresu IP oraz portu docelowego serwera Rust
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = inet_addr(ip_address);

    printf("[NETWORK] Gniazdo UDP zainicjalizowane na %s:%d (Tryb Nieblokujacy)\n", ip_address, port);
    return true;
}

void send_telemetry(double speed, double rpm) {
    if (udp_socket == INVALID_SOCKET) return;

    TelemetryPacket packet;
    packet.header = 0x5A;
    packet.speed = (float)(isnan(speed) ? 0.0 : speed);
    packet.rpm = (float)(isnan(rpm) ? 0.0 : rpm);

    sendto(udp_socket, (const char*)&packet, sizeof(TelemetryPacket), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
}

bool receive_control(ControlPacket* out_packet) {
    if (udp_socket == INVALID_SOCKET) return false;

    int bytes_received = recvfrom(udp_socket, (char*)out_packet, sizeof(ControlPacket), 0, NULL, NULL);

    if (bytes_received > 0) return true;
    return false;
}

void close_network(void) {
    if (udp_socket != INVALID_SOCKET) {
        closesocket(udp_socket);
        WSACleanup();
        printf("[NETWORK] Zasoby sieciowe zwolnione poprawnie.\n");
    }
}