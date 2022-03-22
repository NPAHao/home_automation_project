#include "main.h"

//Define macro
#define DHT11_PIN           19
#define D_INPUT_PIN_1       21
// #define D_INPUT_PIN_2       22
// #define D_INPUT_PIN_3       23
// #define D_INPUT_PIN_4       25
#define D_OUTPUT_PIN_1      26
// #define D_OUTPUT_PIN_2      27
// #define D_OUTPUT_PIN_3      32
// #define D_OUTPUT_PIN_4      33
#define TOUCH_PIN_1         34
// #define TOUCH_PIN_2         35
// #define TOUCH_PIN_3         36
// #define TOUCH_PIN_4         39

//uart msg code
#define FW_MSG              0x00        //receive
#define GET_PEER            0x01        //receive
#define ADD_PEER            0x02        //send
#define DEL_PEER            0x03        //send
#define SEND_DONE           0x04        //send
#define SEND_ESPNOW         0x05        //send