//Include necessary libraries
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <DHT.h>
#include <ArduinoJson.h>

#define DEBUG               1

//Define macro
#define EEPROM_PIN          0
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

//Declare Object
Preferences pref;

//Declare semaphore
SemaphoreHandle_t   touch_1_smp = NULL;
SemaphoreHandle_t   thread_smp = NULL;
SemaphoreHandle_t   save_infor_smp = NULL;
SemaphoreHandle_t   send_smp = NULL;
SemaphoreHandle_t   add_subnode_smp = NULL;

//Task handle
TaskHandle_t    touch_1_task_handle;
TaskHandle_t    d_input_1_task_handle;
TaskHandle_t    espnow_recv_task_handle;
TaskHandle_t    infor_recv_task_handle;
TaskHandle_t    add_subnode_task_handle;
TaskHandle_t    clear_eeprom_task_handle;
TaskHandle_t    setup_gpio_task_handle;
TaskHandle_t    publish_task_handle;

//Queue handle
QueueHandle_t   espnow_queue;
QueueHandle_t   espnow_infor_queue;
QueueHandle_t   subnode_infor_queue;
QueueHandle_t   publish_queue;

//Struct
typedef struct recv_msg
{
    String mac;
    String data;
    int data_len;
} recv_msg;

typedef struct subnode_infor
{
    String type;
    uint8_t interval;
    String topic;
} subnode_infor;

typedef struct node_infor
{
    String  device_name;
    String  mac_addr;
} node_infor;

typedef struct publish_infor
{
    String  topic;
    String  payload;
} publish_infor;

//Variable
String  device_name;
String  mac_addr;
uint8_t broadcast[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
esp_now_peer_info peer;

                            //Normal Function
//complete
void blink_led(int time);

//complete
String convert_touch_time_to_text(uint8_t time);

//complete
void IRAM_ATTR touch_1_isr();
void touch_1_handler(void *pvPara);

//complete
void IRAM_ATTR d_input_1_isr();
void d_input_1_handler(void *pvPara);

//complete
void IRAM_ATTR eeprom_pin_isr();
void clear_eeprom_handler(void *pvPara);

//complete
void espnow_send_isr(const uint8_t *mac_addr, esp_now_send_status_t status);

//uncomplte
void espnow_recv_isr(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void espnow_recv_handler(void *pvPara);

//complete
void espnow_infor_recv_isr(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void espnow_infor_recv_handler(void *pvPara);


                                //Task Define
void add_subnode_task(void *pvPara);
void save_infor(node_infor infor);
void wait_infor_task(void *pvPara);
void publish_task(void *pvPara);
bool get_infor();
void setup_gpio_task(void *pvPara);
void setup_espnow(esp_now_send_cb_t send_cb, esp_now_recv_cb_t recv_cb);
void desetup_espnow();
void setup_eeprom_pin();

void setup() {
    Serial.begin(115200);
    thread_smp = xSemaphoreCreateBinary();
    xTaskCreate(clear_eeprom_handler, "clear eeprom", 2048, NULL, 10, &clear_eeprom_task_handle);
    setup_eeprom_pin();
    if(get_infor() == false) {
        xTaskCreate(wait_infor_task, "wait infor", 8192, NULL, 10, NULL);
    } else {
        xSemaphoreGive(thread_smp);
    }
    xSemaphoreTake(thread_smp, portMAX_DELAY);
    Serial.println(device_name);
    for(uint8_t i = 0; i < 6; i++) {
        Serial.print( *(mac_addr.c_str() + i), HEX );
        Serial.print(" ");
    }
    touch_1_smp = xSemaphoreCreateBinary();
    send_smp = xSemaphoreCreateBinary();
    add_subnode_smp = xSemaphoreCreateBinary();
    subnode_infor_queue = xQueueCreate(1, sizeof(subnode_infor));
    espnow_queue = xQueueCreate(5, sizeof(recv_msg));
    publish_queue = xQueueCreate(5, sizeof(publish_infor));
    xTaskCreate(touch_1_handler, "touch 1", 2048, NULL, 10, &touch_1_task_handle);
    xTaskCreate(d_input_1_handler, "d input 1", 2048, NULL, 10, &d_input_1_task_handle);
    xTaskCreate(espnow_recv_handler, "espnow recv", 2048, NULL, 10, &espnow_recv_task_handle);
    xTaskCreate(add_subnode_task, "add subnode", 2048, NULL, 10, &add_subnode_task_handle);
    xTaskCreate(setup_gpio_task, "setup GPIO", 1024, NULL, 10, &setup_gpio_task_handle);
    xTaskCreate(publish_task, "publish", 2048, NULL, 10, &publish_task_handle);
    setup_espnow(espnow_send_isr, espnow_recv_isr);
    peer.channel = 1;
    peer.encrypt = false;
    memcpy(peer.peer_addr, (uint8_t *)mac_addr.c_str(), 6);
    esp_now_add_peer(&peer);
    vTaskDelete(NULL);
}

void loop(){

}

                            //Normal Function
//complete
void blink_led(int time) {
    pinMode(LED_BUILTIN, OUTPUT);
    for(int i = 0; i < time; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay( 200 / portTICK_PERIOD_MS);
        digitalWrite(LED_BUILTIN, LOW);
        vTaskDelay( 200 / portTICK_PERIOD_MS);
    }
}

//complete
String convert_touch_time_to_text(uint8_t time) {
    String text;
    switch (time)
    {
    case 1:
        text = "single touch";
        break;
    case 2:
        text = "double touch";
        break;
    case 3:
        text = "triple touch";
        break;
    default:
        text = "";
        break;
    }
    return text;
}

//complete
void IRAM_ATTR touch_1_isr() {
    xSemaphoreGiveFromISR(touch_1_smp, NULL);
}
void touch_1_handler(void *pvPara) {
    uint8_t time;
    publish_infor infor;
    infor.topic = device_name + '/' + "touch_1";
    while (1)
    {
        time = 0;
        if( xSemaphoreTake( touch_1_smp , portMAX_DELAY) == pdTRUE ) {
            time++;
            while ( xSemaphoreTake ( touch_1_smp , 500 / portTICK_PERIOD_MS) == pdTRUE) {
                time++;
                if(time == 3) break;
            }
            if( (time == 1) && digitalRead(TOUCH_PIN_1) ) {
                infor.payload = "long touch";
            } else {
                infor.payload = convert_touch_time_to_text(time);
            }
            xQueueSend(publish_queue, &infor, portMAX_DELAY);
        }
    }
}

//complete
void IRAM_ATTR d_input_1_isr() {
    vTaskNotifyGiveFromISR(d_input_1_task_handle, NULL);
}
void d_input_1_handler(void *pvPara) {
    publish_infor infor;
    infor.topic = device_name + '/' + "d_input1";
    infor.payload = "ACTIVE";
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xQueueSend(publish_queue, &infor, portMAX_DELAY);
    }
}

//complete
void IRAM_ATTR eeprom_pin_isr() {
    vTaskNotifyGiveFromISR(clear_eeprom_task_handle, NULL);
}
void clear_eeprom_handler(void *pvPara) {
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        pref.begin("infor", false);
        pref.putString("mac_addr", "");
        pref.putString("name", "");
        pref.end();
        blink_led(5);
        ESP.restart();
    }
}

//complete
void espnow_send_isr(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if((status == ESP_NOW_SEND_SUCCESS) && (memcmp(mac_addr, broadcast, 6) != 0)){
        xSemaphoreGiveFromISR(send_smp, NULL);
    }
}

//uncomplte
void espnow_recv_isr(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    recv_msg msg;
    msg.mac = String((char *)mac_addr);
    msg.data = String((char *)data);
    msg.data_len = data_len;
    xQueueSendFromISR(espnow_queue, &msg, NULL);
}
void espnow_recv_handler(void *pvPara) {
    recv_msg msg;
    DynamicJsonDocument doc(250);
    String from;
    String to;
    String purpose;
    while (1)
    {
        xQueueReceive(espnow_queue, &msg, portMAX_DELAY);
        DeserializationError err = deserializeJson(doc, msg.data);
        if(!err) {
            from = String((const char *)doc["from"]);
            to = String((const char *)doc["to"]);
            purpose = String((const char *)doc["purpose"]);
            if((from == "hub") && (to == "node")) {
                if(purpose == "clear eeprom") {
                    xTaskNotifyGive(clear_eeprom_task_handle);
                } else if(purpose == "add subnode") {
                    subnode_infor infor ;
                    infor.type = String((const char *)doc["type"]);
                    infor.topic = String((const char *)doc["topic"]);
                    infor.interval = (uint8_t)doc["interval"];
                    xQueueSend(subnode_infor_queue, &infor, portMAX_DELAY);
                } else if(purpose == "control") {

                }
            } else if((from == "subnode") && (to == "node")) {
                if(purpose == "send data") {
                    publish_infor infor;
                    infor.topic = device_name + '/' + String((const char*)doc["topic"]);
                    infor.payload = String((const char*)doc["payload"]);
                    xQueueSend(publish_queue, &infor, portMAX_DELAY);
                } else if(purpose == "add success") {
                    xSemaphoreGive(add_subnode_smp);
                }
            }
        }
    }
}

//complete
void espnow_infor_recv_isr(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    recv_msg msg;
    msg.mac = String((char *)mac_addr);
    msg.data = String((char *)data);
    msg.data_len = data_len;
    xQueueSendFromISR(espnow_infor_queue, &msg, NULL);
}
void espnow_infor_recv_handler(void *pvPara) {
    DynamicJsonDocument doc(250);
    String from;
    String to;
    String purpose;
    recv_msg msg;
    while (1)
    {
        xQueueReceive(espnow_infor_queue, &msg, portMAX_DELAY);
        DeserializationError err = deserializeJson(doc, msg.data);
        if(!err) {
            from = String((const char*)doc["from"]);
            to = String((const char*)doc["to"]);
            purpose = String((const char*)doc["purpose"]);
            if((from == "hub") && (to == "node") && (purpose == "add node")) {
                node_infor infor;
                infor.device_name = String((const char*)doc["name"]);
                infor.mac_addr = msg.mac;
                save_infor(infor);
            }
        }
    }
}


                                //Task Define
void add_subnode_task(void *pvPara) {
    subnode_infor infor;
    while (1)
    {
        xQueueReceive(subnode_infor_queue, &infor, portMAX_DELAY);
        DynamicJsonDocument doc(250);
        doc["from"] = "node";
        doc["to"] = "subnode";
        doc["purpose"] = "add subnode";
        doc["type"] = infor.type;
        doc["interval"] = infor.interval;
        doc["topic"] = infor.topic;
        String buffer;
        serializeJson(doc, buffer);
        do {
            esp_now_send(broadcast, (uint8_t *)buffer.c_str(), buffer.length());
        } while (xSemaphoreTake(add_subnode_smp, 100 / portTICK_PERIOD_MS) == pdFALSE);  
    }
}

void save_infor(node_infor infor) {
    pref.begin("infor", false);
    pref.putString("mac_addr", infor.mac_addr);
    pref.putString("name", infor.device_name);
    pref.end();
    xSemaphoreGive(save_infor_smp);
}

void wait_infor_task(void *pvPara) {
    espnow_infor_queue = xQueueCreate(1, sizeof(recv_msg));
    xTaskCreate(espnow_infor_recv_handler,"infor recv", 4096, NULL, 10, &infor_recv_task_handle);
    save_infor_smp = xSemaphoreCreateBinary();
    setup_espnow(espnow_send_isr, espnow_infor_recv_isr);
    xSemaphoreTake(save_infor_smp, portMAX_DELAY);
    esp_now_unregister_recv_cb();
    vQueueDelete(espnow_infor_queue);
    vTaskDelete(infor_recv_task_handle);
    vSemaphoreDelete(save_infor_smp);
    get_infor();
    peer.channel = 1;
    peer.encrypt = false;
    memcpy(peer.peer_addr, (uint8_t *)mac_addr.c_str(), 6);
    esp_now_add_peer(&peer);
    DynamicJsonDocument doc(250);
    String buffer;
    send_smp = xSemaphoreCreateBinary();
    doc["from"] = "node";
    doc["to"] = "hub";
    doc["purpose"] = "add success";
    serializeJson(doc, buffer);
    do {
        esp_now_send((uint8_t *)mac_addr.c_str(), (uint8_t *)buffer.c_str(), buffer.length());
    } while(xSemaphoreTake(send_smp, 100 / portTICK_PERIOD_MS) == pdFALSE);
    esp_now_del_peer((uint8_t *)mac_addr.c_str());
    desetup_espnow();
    vSemaphoreDelete(send_smp);
    xSemaphoreGive(thread_smp);
    vTaskDelete(NULL);
}

void publish_task(void *pvPara) {
    publish_infor infor;
    DynamicJsonDocument doc(250);
    String buffer;
    while (1)
    {
        xQueueReceive(publish_queue, &infor, portMAX_DELAY);
        doc["from"] = "node";
        doc["to"] = "hub";
        doc["purpose"] = "publish";
        doc["topic"] = infor.topic;
        doc["payload"] = infor.payload;
        serializeJson(doc, buffer);
        esp_now_send((uint8_t *)mac_addr.c_str(), (uint8_t *)buffer.c_str(), buffer.length());
    }
}

bool get_infor() {
    pref.begin("infor", true);
    mac_addr = pref.getString("mac_addr", "");
    device_name = pref.getString("name", "");
    pref.end();
    return ((mac_addr != "") && (device_name != ""));
}

void setup_gpio_task(void *pvPara) {
    pinMode(D_OUTPUT_PIN_1, OUTPUT);
    pinMode(TOUCH_PIN_1, INPUT);
    pinMode(D_INPUT_PIN_1, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN_1), touch_1_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(D_INPUT_PIN_1), d_input_1_isr, RISING);
    vTaskDelete(NULL);
}

void setup_espnow(esp_now_send_cb_t send_cb, esp_now_recv_cb_t recv_cb) {
    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect();
    esp_now_init();
    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);
    peer.channel = 1;
    peer.encrypt = false;
    memcpy(peer.peer_addr, broadcast, 6);
    esp_now_add_peer(&peer);
}

void desetup_espnow() {
    esp_now_del_peer(broadcast);
    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();
    esp_now_deinit();
}

void setup_eeprom_pin() {
    pinMode(EEPROM_PIN, INPUT_PULLUP);
    attachInterrupt(EEPROM_PIN, eeprom_pin_isr, FALLING);
}