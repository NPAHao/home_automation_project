//Kitchen
//Device MAC_address: 9C:9C:1F:C7:20:8C (Master)
//Slave  MAC_address: 9C:9C:1F:C6:FB:BF
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <driver/can.h>

#define TOGGLE_PIN(n)               digitalWrite(n,digitalRead(n)?0:1)
/**
 * @brief define pin is used
 * 
 */
#define TOUCH_UP_PIN                39
#define TOUCH_DOWN_PIN              36
#define HUMAN_SS_PIN                35
#define AUTO_LIGHT_PIN              34
#define MANUAL_LIGHT_PIN            33
#define DHT11_PIN                   32
#define MANUAL_LIGHT_TOUCH_PIN      27
#define CAN_TX_PIN                  (gpio_num_t)26
#define CAN_RX_PIN                  (gpio_num_t)25
#define BUZZ_PIN                    23
/**
 * @brief define for data field byte[0] of CAN data frame
 * 
 */
#define FLAME_ALERT_CODE            0x01    //    IO
#define LIGHT_CONTROL_CODE          0x02
#define HUMAN_STT_CODE              0x04    //    O
#define DHT11_DATA_CODE             0x05    //    O
/**
 * @brief define for data field byte[2] of CAN data frame
 * 
 */
#define DEVICE_CAN_SEND_ID          0x740
#define DEVICE_CAN_RECV_ID          (uint32_t)(0x40 <<21)
#define DEVICE_CAN_RECV_ID_MASK     (uint32_t)0x001FFFFF

/**
 * @brief type define for enum
 * 
 */
typedef enum state
{
  IDLE,
  HUMAN_ACTIVE,
  HUMAN_INACTIVE,
  FLAME_ALERT,
  SEND_DHT11_MQ2_DATA,
  SEND_LIGHT_STT,
  PROCESS_NOW_RECV_PACK
} state;
typedef struct now_alert_send
{
  uint8_t alert_code;
  bool alert_confirm=false;
} now_alert_send;
typedef struct now_alert_recv
{
  uint8_t alert_code;
  bool alert_flag=false;
} now_alert_recv;
typedef struct dht11_infor
{
  uint8_t humi;
  uint8_t humd;
  uint8_t tempi;
  uint8_t tempd;
} dht11_infor;
/**
 * @brief variable declare
 * 
 */
uint8_t           slave_Address[6] = {0x48,0x3F,0xDA,0x77,0xCB,0x63};
QueueHandle_t     state_Queue;
QueueHandle_t     now_rx_msg_Queue;
now_alert_recv    now_alert_recv_Pack;
now_alert_send    now_send_confirm_Pack;
dht11_infor       dht11_data;
state             device_State = IDLE;
volatile bool     human_stt;
volatile char     bright = 5;
hw_timer_t        *timer = NULL;
can_message_t     can_send_Pack = { .flags = CAN_MSG_FLAG_NONE,
                                    .identifier = DEVICE_CAN_SEND_ID};
can_message_t     can_recv_Pack;
/********************************************************/
/********************************************************/
void can_recv_task(void* n);     //Task Pin to core 0
void timer_setup();
void esp_now_setup();
void can_bus_setup();
void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status);
void recv_callback(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void IRAM_ATTR pir_trigger();
void IRAM_ATTR up_touch();
void IRAM_ATTR down_touch();
void IRAM_ATTR manual_light_touch();
void IRAM_ATTR onTimer();
void process_now_recv_pack_func();
void process_can_recv_pack_func(uint8_t *data);
void human_active_func();
void human_inactive_func();
void flame_alert_func();
void send_dht11_mq2_data_func();
void send_light_stt_func();
void change_bright();
void start_dht11_signal(uint8_t dht11_pin);
bool read_dht11(uint8_t dht11_pin, dht11_infor *temp_hum_data);
/********************************************************/
/********************************************************/
void setup() {
  Serial.begin(115200);
  esp_now_setup();
  timer_setup();
  can_bus_setup();
  state_Queue = xQueueCreate(10,sizeof(state));
  if (state_Queue == NULL) Serial.println("State queue can't create");
  now_rx_msg_Queue = xQueueCreate(10,sizeof(now_alert_recv));
  if (now_rx_msg_Queue == NULL) Serial.println("ESPNOW receive queue can't create");
  pinMode(MANUAL_LIGHT_PIN, OUTPUT);
  digitalWrite(MANUAL_LIGHT_PIN, LOW);
  pinMode(MANUAL_LIGHT_TOUCH_PIN, INPUT);
  pinMode(HUMAN_SS_PIN, INPUT);
  pinMode(TOUCH_UP_PIN, INPUT_PULLDOWN);
  pinMode(TOUCH_DOWN_PIN, INPUT_PULLDOWN);
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(LED_BUILTIN, 0);
  ledcWrite(0, 0);
  // ledcSetup(1,5000,8);
  // ledcSetup(2,5000,8);
  // ledcSetup(3,5000,8);
  // ledcSetup(4,5000,8);
  attachInterrupt(HUMAN_SS_PIN, pir_trigger, CHANGE);
  attachInterrupt(MANUAL_LIGHT_TOUCH_PIN, manual_light_touch, RISING);
  if(xTaskCreatePinnedToCore(can_recv_task,"can_recv_mess",4096,NULL,0,NULL,0) != pdPASS) Serial.println("Task create fail");
  Serial.println("ESP start");
}
// /********************************************************/
void loop() {
  switch (device_State)
  {
  case IDLE:
    xQueueReceive(state_Queue, &device_State,0);
    break;
  case HUMAN_ACTIVE:
    Serial.println("Send human present task");
    human_active_func();
    device_State = IDLE;
    break;
  case HUMAN_INACTIVE:
    Serial.println("Send no human task");
    human_inactive_func();
    device_State = IDLE;
    break;
  case FLAME_ALERT:
    Serial.println("Send flame alert task");
    flame_alert_func();
    device_State = IDLE;
    break;
  case SEND_DHT11_MQ2_DATA:
    Serial.println("Send DHT11 task");
    send_dht11_mq2_data_func();
    device_State = IDLE;
    break;
  case SEND_LIGHT_STT:
    Serial.println("Send manual light status");
    send_light_stt_func();
    device_State = IDLE;
    break;
  case PROCESS_NOW_RECV_PACK:
    Serial.println("Process NOW receive message task");
    process_now_recv_pack_func();
    device_State = IDLE;
    break;
  }
}
/********************************************************/
/********************************************************/
void can_recv_task(void* n)
{
  can_message_t pseudo_recv_msg;
  while(true)
  {
    if(can_receive(&pseudo_recv_msg,10) == ESP_OK)
    {
      process_can_recv_pack_func(pseudo_recv_msg.data);
    }
  }
}
/********************************************************/
void timer_setup()
{
  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000000, true);
  timerAlarmEnable(timer);
}
/********************************************************/
void esp_now_setup()
{
  WiFi.mode(WIFI_STA);
  if(esp_now_init() != ESP_OK) Serial.println("ESPNOW init fail");
  if(esp_now_register_send_cb(send_callback) != ESP_OK) Serial.println("ESPNOW send regist fail");
  if(esp_now_register_recv_cb(recv_callback) != ESP_OK) Serial.println("ESPNOW recv regist fail");
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, slave_Address, 6);
  if(esp_now_add_peer(&peerInfo) == ESP_OK)
    Serial.println("ESPNOW setup done!");
}
/********************************************************/
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
  can_filter_config_t filter_config = { .acceptance_code = DEVICE_CAN_RECV_ID,
                                        .acceptance_mask = DEVICE_CAN_RECV_ID_MASK,
                                        .single_filter = true};
  if(can_driver_install(&general_config, &timing_config, &filter_config) != ESP_OK) Serial.println("CANbus setup fail");
  if(can_start() != ESP_OK) Serial.println("CANbus setup fail");
}
/********************************************************/
void recv_callback(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  state pseudo_state = PROCESS_NOW_RECV_PACK;
  if (memcmp(mac_addr,slave_Address,6) == 0)
  {
    memcpy(&now_alert_recv_Pack,data,sizeof(now_alert_recv));
    xQueueSendFromISR(state_Queue, &pseudo_state, NULL);
  }
}
/********************************************************/
void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if(status!=0)
    esp_now_send(slave_Address,(uint8_t *) &now_send_confirm_Pack,sizeof(now_alert_send));
}
/********************************************************/
void process_now_recv_pack_func()
{
  state pseudo_state;
  if(now_alert_recv_Pack.alert_flag == true)
  {
    if(now_alert_recv_Pack.alert_code == FLAME_ALERT_CODE)
      pseudo_state = FLAME_ALERT;
    xQueueSend(state_Queue,&pseudo_state,10);
  }
}
// /********************************************************/
void process_can_recv_pack_func(uint8_t *data)
{
  switch (data[0])
  {
  case FLAME_ALERT_CODE:
    digitalWrite(BUZZ_PIN, HIGH);
    break;
  case LIGHT_CONTROL_CODE:
    send_light_stt_func();
    break;
  }
}
/********************************************************/
void human_active_func()
{
  change_bright();
  attachInterrupt(TOUCH_UP_PIN,&up_touch,RISING);
  attachInterrupt(TOUCH_DOWN_PIN,&down_touch,RISING);
  can_send_Pack.data_length_code = 2;
  can_send_Pack.data[0] = HUMAN_STT_CODE;
  can_send_Pack.data[1] = human_stt;
  can_transmit(&can_send_Pack,portMAX_DELAY);
}
/********************************************************/
void human_inactive_func()
{
  bright = 0;
  change_bright();
  bright = 5;
  detachInterrupt(TOUCH_UP_PIN);
  detachInterrupt(TOUCH_DOWN_PIN);
  can_send_Pack.data_length_code = 2;
  can_send_Pack.data[0] = HUMAN_STT_CODE;
  can_send_Pack.data[1] = human_stt;
  can_transmit(&can_send_Pack,portMAX_DELAY);
}
/********************************************************/
void flame_alert_func()
{
  digitalWrite(BUZZ_PIN, LOW);
  can_send_Pack.data_length_code = 1;
  can_send_Pack.data[0] = FLAME_ALERT_CODE;
  can_transmit(&can_send_Pack,portMAX_DELAY);
  now_send_confirm_Pack.alert_code = FLAME_ALERT_CODE;
  now_send_confirm_Pack.alert_confirm = true;
  esp_now_send(slave_Address,(uint8_t *) &now_send_confirm_Pack,sizeof(now_alert_send));
}
/********************************************************/
void send_dht11_mq2_data_func()
{
  start_dht11_signal(DHT11_PIN);
  read_dht11(DHT11_PIN,&dht11_data);
  can_send_Pack.data_length_code = 5;
  can_send_Pack.data[0] = DHT11_DATA_CODE;
  can_send_Pack.data[1] = dht11_data.tempi;
  can_send_Pack.data[2] = dht11_data.tempd;
  can_send_Pack.data[3] = dht11_data.humi;
  can_send_Pack.data[4] = dht11_data.humd;
  if(can_transmit(&can_send_Pack,portMAX_DELAY) != ESP_OK) Serial.println("Send DHT11 fail");
}
void send_light_stt_func()
{
  TOGGLE_PIN(MANUAL_LIGHT_PIN);
  can_send_Pack.data_length_code = 2;
  can_send_Pack.data[0] = LIGHT_CONTROL_CODE;
  can_send_Pack.data[1] = (uint8_t)digitalRead(MANUAL_LIGHT_PIN);
  if (can_transmit(&can_send_Pack,portMAX_DELAY) != ESP_OK) Serial.println("Send manual light status fail");  
}
/********************************************************/
void change_bright()
{
  ledcWrite(0,bright*51);
}
/**
 * @brief ISR for up touch button to increase the bright
 * 
 */
void IRAM_ATTR up_touch()
{
  if(bright<5)
    bright++;
  change_bright();
}
/**
 * @brief ISR for down touch button to decrease the bright
 * 
 */
void IRAM_ATTR down_touch()
{
  if(bright>0)
    bright--;
  change_bright();
}
void IRAM_ATTR manual_light_touch()
{
  state pseudo_state = SEND_LIGHT_STT;
  xQueueSendFromISR(state_Queue, &pseudo_state, NULL);
}
/**
 * @brief ISR for human motion detect sensor
 * 
 */
void IRAM_ATTR pir_trigger()
{
  state pseudo_state;
  delay(5);
  if(digitalRead(HUMAN_SS_PIN))
    {
    pseudo_state = HUMAN_ACTIVE;
    human_stt = true;
    }
    else {
      pseudo_state = HUMAN_INACTIVE;
      human_stt = false;
    }
  xQueueSendFromISR(state_Queue, &pseudo_state, NULL);
}
/**
 * @brief ISR for timer interrupt
 * 
 */
void IRAM_ATTR onTimer()
{
  state pseudo_state = SEND_DHT11_MQ2_DATA;
  xQueueSendFromISR(state_Queue, &pseudo_state, NULL);
}
/********************************************************/
void start_dht11_signal(uint8_t dht11_pin){
pinMode(dht11_pin, OUTPUT);
digitalWrite(dht11_pin, LOW); 
delay(18);
digitalWrite(dht11_pin, HIGH);
pinMode(dht11_pin, INPUT);
digitalWrite(dht11_pin, HIGH); 
}
/********************************************************/
bool read_dht11(uint8_t dht11_pin, dht11_infor *temp_hum_data)
{
  uint16_t rawHumidity = 0;
  uint16_t rawTemperature = 0;
  uint8_t checkSum = 0;
  uint16_t data = 0;

  uint8_t humi;
  uint8_t humd;
  uint8_t tempi;
  uint8_t tempd; 

  unsigned long startTime;
  
  for ( int8_t i = -3 ; i < 80; i++ ) {
    byte live;
    startTime = micros();

    do {
      live = (unsigned long)(micros() - startTime);
      if ( live > 90 )
        return false;
    }
    while ( digitalRead(dht11_pin) == (i & 1) ? HIGH : LOW );

    if ( i >= 0 && (i & 1) ) {
      data <<= 1;

      // TON of bit 0 is maximum 30 usecs and of bit 1 is at least 68 usecs.
      if ( live > 30 ) {
        data |= 1; // we got a one
      }
    }

    switch ( i ) {
      case 31:
        rawHumidity = data;
        break;
      case 63:
        rawTemperature = data;
      case 79: 
        checkSum = data;
        data = 0;
        break;
    }
  }

humi = rawHumidity >> 8;
rawHumidity = rawHumidity << 8;
humd = rawHumidity >> 8;

tempi = rawTemperature >> 8;
rawTemperature = rawTemperature << 8;
tempd = rawTemperature >> 8;
//tempd = (byte)rawTemperature;
if((byte)checkSum == (byte)(tempi + tempd + humi + humd))
  {
    temp_hum_data->humi = humi;
    temp_hum_data->humd = humd;
    temp_hum_data->tempi = tempi;
    temp_hum_data->tempd = tempd;
    return true;
  }
  else
    return false;
}