#include "main.h"
#include "pv_espnow.h"

//uart msg code
#define FW_MSG              0x00        //receive
#define GET_PEER            0x01        //receive
#define ADD_PEER            0x02        //send
#define DEL_PEER            0x03        //send
#define SEND_DONE           0x04        //send
#define SEND_ESPNOW         0x05        //send

struct uart_msg {
    uint8_t code;
    espnow now_msg;
};

void parse_uart_msg(uart_msg *msg, String parse_str);