//SEND NODE
#include <Arduino.h>
#include <driver/can.h>

#define CAN_TX_PIN                  (gpio_num_t)26
#define CAN_RX_PIN                  (gpio_num_t)25
#define TOUCH_PIN_1                 39
#define TOUCH_PIN_2                 36

#define FLAME_ALERT_CODE            0x01    //    IO
#define LIGHT_CONTROL_CODE          0x02
#define HUMAN_STT_CODE              0x04    //    O
#define DHT11_DATA_CODE             0x05    //    O

typedef enum state
{
  IDLE,
  CAN_FLAME_CONFIRM,
  CAN_LIGHT_CONTROL
} state;

QueueHandle_t     state_Queue;
state             device_State = IDLE;
can_message_t     can_send_Pack ={  .flags = CAN_MSG_FLAG_NONE,
                                    .identifier = 0x40};
can_message_t     can_recv_Pack;

void can_recv_task(void* n);     //Task Pin to core 0
void can_bus_setup();
void IRAM_ATTR on_touch_1();
void IRAM_ATTR on_touch_2();
void process_can_recv_msg_func(uint8_t *data);

void setup() {
  Serial.begin(115200);
  state_Queue = xQueueCreate(10, sizeof(state));
  if(state_Queue == NULL)
    Serial.println("State create fail");
  can_bus_setup();
  pinMode(TOUCH_PIN_1, INPUT);
  pinMode(TOUCH_PIN_2, INPUT);
  attachInterrupt(TOUCH_PIN_1, on_touch_1, RISING);
  attachInterrupt(TOUCH_PIN_2, on_touch_2,RISING);
  if(xTaskCreatePinnedToCore(can_recv_task, "can_recv_mess", 4096, NULL, 0, NULL, 0) != pdPASS) 
    Serial.println("Task create fail");
}

void loop() {
  switch (device_State)
  {
  case IDLE:
    xQueueReceive(state_Queue, &device_State, 10);
    break;
  case CAN_FLAME_CONFIRM:
    can_send_Pack.data[0] = FLAME_ALERT_CODE;
    can_send_Pack.data_length_code = 1;
    can_transmit(&can_send_Pack, portMAX_DELAY);
    device_State = IDLE;
    break;
  case CAN_LIGHT_CONTROL:
    can_send_Pack.data[0] = LIGHT_CONTROL_CODE;
    can_send_Pack.data_length_code = 1;
    can_transmit(&can_send_Pack, portMAX_DELAY);
    device_State = IDLE;
    break;
  }
}

void can_recv_task(void* n)     //Task Pin to core 0
{
  can_message_t pseudo_recv_msg;
  while(true)
  {
    if(can_receive(&pseudo_recv_msg, 10) == ESP_OK)
    {
      process_can_recv_msg_func(pseudo_recv_msg.data);
    }
  }  
}
void can_bus_setup()
{
  can_general_config_t general_config = { .mode = CAN_MODE_NORMAL, 
                                          .tx_io = CAN_TX_PIN, 
                                          .rx_io = CAN_RX_PIN,       
                                          .clkout_io = CAN_IO_UNUSED, 
                                          .bus_off_io = CAN_IO_UNUSED,       
                                          .tx_queue_len = 10, 
                                          .rx_queue_len = 10,                          
                                          .alerts_enabled = CAN_ALERT_NONE,  
                                          .clkout_divider = 0,        
                                          .intr_flags = ESP_INTR_FLAG_LEVEL1};
  can_timing_config_t timing_config = CAN_TIMING_CONFIG_1MBITS();
  can_filter_config_t filter_config = { .acceptance_code = (uint32_t)0x740<<21,
                                        .acceptance_mask = (uint32_t)0x001FFFFF,
                                        .single_filter = true};
  if(can_driver_install(&general_config, &timing_config, &filter_config) != ESP_OK) Serial.println("CANbus setup fail");
  if(can_start() != ESP_OK) Serial.println("CANbus setup fail");
}
void IRAM_ATTR on_touch_1()
{
  state pseudo_state = CAN_FLAME_CONFIRM;
  xQueueSendFromISR(state_Queue, &pseudo_state, NULL);
}
void IRAM_ATTR on_touch_2()
{
  state pseudo_state = CAN_LIGHT_CONTROL;
  xQueueSendFromISR(state_Queue, &pseudo_state, NULL);
}
void process_can_recv_msg_func(uint8_t *data)
{
  switch (data[0])
  {
  case  FLAME_ALERT_CODE:
    Serial.println("Flame detect!");
    break;
  case HUMAN_STT_CODE:
    if(data[1])
      Serial.println("Human is present!");
      else Serial.println("No human!");
    break;
  case DHT11_DATA_CODE:
    Serial.print("Tempature:\t");
    Serial.print(data[1]);
    Serial.print(".");
    Serial.print(data[2]);
    Serial.println("*C");
    Serial.print("Huminity:\t");
    Serial.print(data[3]);
    Serial.print(".");
    Serial.print(data[4]);
    Serial.println("%");    
    break;
  case LIGHT_CONTROL_CODE:
    if(data[1] == 0)
      Serial.println("Manual light off.");
      else Serial.println("Manual light on");
    break;
  }
}