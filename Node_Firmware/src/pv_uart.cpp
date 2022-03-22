#include "pv_uart.h"

void parse_uart_msg(uart_msg *msg, String parse_str) {
    msg->code = parse_str[0];
    msg->now_msg.mac_addr = parse_str.substring(1, 6);
    msg->now_msg.len = parse_str[parse_str.length() - 1];
    msg->now_msg.data = parse_str.substring(7, msg->now_msg.len);
}