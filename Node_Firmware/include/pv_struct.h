#include "main.h"

struct espnow {
    String mac_addr;
    String data;
    uint8_t len;
};

struct mqtt_infor {
    String server;
    String user_name;
    String password;
    String device_name;
};

struct wifi_infor {
    String ssid;
    String password;
};

struct uart_msg {
    uint8_t code;
    espnow now_msg;
};