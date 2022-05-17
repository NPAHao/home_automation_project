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
#define DHT11_PIN           27
#define BUZZER_PIN          13
#define INPUT_PIN_1         26
#define INPUT_PIN_2         25
#define INPUT_PIN_3         33
#define INPUT_PIN_4         32
#define ANALOG_PIN_1        22
#define ANALOG_PIN_2        23
#define OUTPUT_PIN_1        16
#define OUTPUT_PIN_2        17
#define OUTPUT_PIN_3        18
#define OUTPUT_PIN_4        19
#define TOUCH_PIN_1         35
#define TOUCH_PIN_2         34
#define TOUCH_PIN_3         39
#define TOUCH_PIN_4         36

Preferences pref;

SemaphoreHandle_t   touch_1_smp = NULL;
SemaphoreHandle_t   touch_2_smp = NULL;
SemaphoreHandle_t   touch_3_smp = NULL;
SemaphoreHandle_t   touch_4_smp = NULL;

TaskHandle_t    send_confirm_task_handle;
TaskHandle_t    clear_eeprom_task_handle;

QueueHandle_t   delete_node_queue;
QueueHandle_t   add_subnode_queue;
QueueHandle_t   set_input_config_queue;
QueueHandle_t   get_input_config_queue;
QueueHandle_t   input_config_queue;
QueueHandle_t   get_input_state_queue;
QueueHandle_t   input_state_queue;
QueueHandle_t   set_analog_config_queue;
QueueHandle_t   get_analog_config_queue;
QueueHandle_t   analog_config_queue;
QueueHandle_t   get_analog_value_queue;
QueueHandle_t   analog_value_queue;
QueueHandle_t   control_ouput_state_queue;
QueueHandle_t   get_output_state_queue;
QueueHandle_t   output_state_queue;
QueueHandle_t   trigger_buzzer_queue;
QueueHandle_t   restart_queue;
QueueHandle_t   espnow_recv_queue;
QueueHandle_t   espnow_send_queue;

typedef struct espnow_packet
{
    uint8_t mac[ESP_NOW_ETH_ALEN];
    uint8_t data[ESP_NOW_MAX_DATA_LEN];
} recv_msg;
typedef struct input_config 
{
    String mode;
    String int_mode;
} input_config;
typedef struct output
{
    String output;
    String state;
} output;
typedef struct input
{
    String input;
    String state;
} input;

String  node;
uint8_t hub_mac[6];
String s_hub_mac;
uint8_t broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info peer;
bool espnow_send_success = false;
String a_config_1, a_config_2;
input_config d_config_1, d_config_2, d_config_3, d_config_4;

void blink_led(int time, int interval_ms) 
{
    pinMode(LED_BUILTIN, OUTPUT);
    for(int i = 0; i < time; i++) 
    {
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay( interval_ms / portTICK_PERIOD_MS);
        digitalWrite(LED_BUILTIN, LOW);
        vTaskDelay( interval_ms / portTICK_PERIOD_MS);
    }
}
void setup_espnow(esp_now_send_cb_t send_cb, esp_now_recv_cb_t recv_cb) 
{
    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect();
    while(esp_now_init() != ESP_OK) 
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    while(esp_now_register_recv_cb(recv_cb) != ESP_OK) 
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    while(esp_now_register_send_cb(send_cb) != ESP_OK) 
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
void IRAM_ATTR eeprom_pin_isr() 
{
    vTaskNotifyGiveFromISR(clear_eeprom_task_handle, NULL);
}
void IRAM_ATTR touch_1_isr() 
{
    xSemaphoreGiveFromISR(touch_1_smp, NULL);
}
void IRAM_ATTR touch_2_isr() 
{
    xSemaphoreGiveFromISR(touch_2_smp, NULL);
}
void IRAM_ATTR touch_3_isr() 
{
    xSemaphoreGiveFromISR(touch_3_smp, NULL);
}
void IRAM_ATTR touch_4_isr() 
{
    xSemaphoreGiveFromISR(touch_4_smp, NULL);
}
void IRAM_ATTR input_1_isr() 
{
    input infor;
    infor.input = String("input 1");
    infor.state = String(digitalRead(INPUT_PIN_1)?"HIGH":"LOW");
    xQueueSendFromISR(input_state_queue, &infor, NULL);
}
void IRAM_ATTR input_2_isr() 
{
    input infor;
    infor.input = String("input 2");
    infor.state = String(digitalRead(INPUT_PIN_2)?"HIGH":"LOW");
    xQueueSendFromISR(input_state_queue, &infor, NULL);
}
void IRAM_ATTR input_3_isr() 
{
    input infor;
    infor.input = String("input 3");
    infor.state = String(digitalRead(INPUT_PIN_3)?"HIGH":"LOW");
    xQueueSendFromISR(input_state_queue, &infor, NULL);
}
void IRAM_ATTR input_4_isr() 
{
    input infor;
    infor.input = String("input 4");
    infor.state = String(digitalRead(INPUT_PIN_4)?"HIGH":"LOW");
    xQueueSendFromISR(input_state_queue, &infor, NULL);
}

void delete_node_task(void *pvParameters)
{
    delete_node_queue = xQueueCreate(1, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    doc["object"] = String("confirm delete node");
    doc["node"] = node;
    doc["mac"] = WiFi.macAddress();
    espnow_packet packet;
    memcpy(packet.mac, hub_mac, ESP_NOW_ETH_ALEN);
    serializeJson(doc, packet.data);
    while(1)
    {
        espnow_packet trash;
        xQueueReceive(delete_node_queue, &trash, portMAX_DELAY);
        xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        xTaskNotifyGive(clear_eeprom_task_handle);
    }
}

void add_subnode_task(void *pvParameters) 
{
    add_subnode_queue = xQueueCreate(3, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    while (1)
    {
        espnow_packet packet;
        xQueueReceive(add_subnode_queue, &packet, portMAX_DELAY);
        memcpy(packet.mac, broadcast, ESP_NOW_ETH_ALEN);
        xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
    }
}
 
void set_input_config_task(void *pvParameters) 
{
    set_input_config_queue = xQueueCreate(3, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    while (1)
    {
        espnow_packet packet;
        xQueueReceive(set_input_config_queue, &packet, portMAX_DELAY);
        deserializeJson(doc, packet.data);
        if(String((const char*)doc["input"]) == "input 1") 
        {
            d_config_1.mode = String((const char*)doc["mode"]);
            d_config_1.int_mode = String((const char*)doc["int_mode"]);
        } else if(String((const char*)doc["input"]) == "input 2") 
        {
            d_config_2.mode = String((const char*)doc["mode"]);
            d_config_2.int_mode = String((const char*)doc["int_mode"]);
        } else if(String((const char*)doc["input"]) == "input 3") 
        {
            d_config_3.mode = String((const char*)doc["mode"]);
            d_config_3.int_mode = String((const char*)doc["int_mode"]);            
        } else if(String((const char*)doc["input"]) == "input 4") 
        {
            d_config_4.mode = String((const char*)doc["mode"]);
            d_config_4.int_mode = String((const char*)doc["int_mode"]);            
        }
        pref.begin(doc["input"], false);
        pref.putString("mode", String((const char*)doc["mode"]));
        pref.putString("int_mode", String((const char*)doc["int_mode"]));
        pref.end();
        String input = String((const char*)doc["input"]);
        xQueueSend(input_config_queue, &input, portMAX_DELAY);
    }
}

void get_input_config_task(void *pvParameters) 
{
    get_input_config_queue = xQueueCreate(3, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    while (1)
    {
        espnow_packet packet;
        xQueueReceive(get_input_config_queue, &packet, portMAX_DELAY);
        deserializeJson(doc, packet.data);
        String input = String((const char*)doc["input"]);
        xQueueSend(input_config_queue, &input, portMAX_DELAY);
    }
}

void input_config_task(void *pvParameters) 
{
    input_config_queue = xQueueCreate(3, sizeof(String));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    doc["object"] = String("input config");
    while (1)
    {
        String input;
        xQueueReceive(input_config_queue, &input, portMAX_DELAY);
        doc["node"] = node;
        doc["input"] = input;
        if(input == String("input 1")) {
            pref.begin("input 1", true);
        } else if(input == String("input 2")) {
            pref.begin("input 2", true);
        } else if(input == String("input 3")) {
            pref.begin("input 3", true);
        } else if(input == String("input 4")) {
            pref.begin("input 4", true);
        }
        doc["mode"] = pref.getString("mode", "unused");
        doc["int_mode"] = pref.getString("int_mode", "unused");
        pref.end();
        espnow_packet packet;
        memcpy(packet.mac, hub_mac, ESP_NOW_ETH_ALEN);
        serializeJson(doc, packet.data);
        xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
    }
}

void get_input_state_task(void *pvParameters) 
{
    get_input_state_queue = xQueueCreate(3, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    while (1)
    {
        espnow_packet packet;
        xQueueReceive(get_input_state_queue, &packet, portMAX_DELAY);
        deserializeJson(doc, packet.data);
        String input = String((const char*)doc["input"]);
        if(input == String("input 1"))
        {
            input_1_isr();
        } else if(input == String("input 2"))
        {
            input_2_isr();
        } else if(input == String("input 3"))
        {
            input_3_isr();
        } else if(input == String("input 4"))
        {
            input_4_isr();
        }
    }
}

void input_state_task(void *pvParameters) {
    input_state_queue = xQueueCreate(10, sizeof(input));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    doc["object"] = String("input state");
    doc["node"] = node;
    { // get config
        { //config 1
            pref.begin("input 1", true);
            d_config_1.mode = pref.getString("mode", "unused");
            d_config_1.int_mode = pref.getString("int_mode", "unused");
            pref.end();
        }
        { //config 2
            pref.begin("input 2", true);
            d_config_2.mode = pref.getString("mode", "unused");
            d_config_2.int_mode = pref.getString("int_mode", "unused");
            pref.end();
        }
        { //config 3
            pref.begin("input 3", true);
            d_config_3.mode = pref.getString("mode", "unused");
            d_config_3.int_mode = pref.getString("int_mode", "unused");
            pref.end();
        }
        { //config 4
            pref.begin("input 4", true);
            d_config_4.mode = pref.getString("mode", "unused");
            d_config_4.int_mode = pref.getString("int_mode", "unused");
            pref.end();
        }
    }
    { // setup input
        { // input 1
            { //mode
                if(d_config_1.mode == String("unused")) 
                {
                } else if(d_config_1.mode == String("input")) 
                {
                    pinMode(INPUT_PIN_1, INPUT);
                } else if(d_config_1.mode == String("pull up")) 
                {
                    pinMode(INPUT_PIN_1, INPUT_PULLUP);
                } else if(d_config_1.mode == String("pull down")) 
                {
                    pinMode(INPUT_PIN_1, INPUT_PULLDOWN);
                }
            }
            { //interrupt mode
                if(d_config_1.int_mode == String("unused")) 
                {
                } else if(d_config_1.int_mode == String("rising")) 
                {
                    attachInterrupt(INPUT_PIN_1, input_1_isr, RISING);
                } else if(d_config_1.int_mode == String("falling")) 
                {
                    attachInterrupt(INPUT_PIN_1, input_1_isr, FALLING);
                } else if(d_config_1.int_mode == String("change")) 
                {
                    attachInterrupt(INPUT_PIN_1, input_1_isr, CHANGE);
                }
            }
        }
        { // input 2
            { //mode
                if(d_config_2.mode == String("unused")) 
                    {
                } else if(d_config_2.mode == String("input")) 
                {
                    pinMode(INPUT_PIN_2, INPUT);
                } else if(d_config_2.mode == String("pull up")) 
                {
                    pinMode(INPUT_PIN_2, INPUT_PULLUP);
                } else if(d_config_2.mode == String("pull down")) 
                {
                    pinMode(INPUT_PIN_2, INPUT_PULLDOWN);
                }
            }
            { //interrupt mode
                if(d_config_2.int_mode == String("unused")) 
                {
                } else if(d_config_2.int_mode == String("rising")) 
                {
                    attachInterrupt(INPUT_PIN_2, input_2_isr, RISING);
                } else if(d_config_2.int_mode == String("falling")) 
                {
                    attachInterrupt(INPUT_PIN_2, input_2_isr, FALLING);
                } else if(d_config_2.int_mode == String("change")) 
                {
                    attachInterrupt(INPUT_PIN_2, input_2_isr, CHANGE);
                }
            }
        }
        { // input 3
            { //mode
                if(d_config_3.mode == String("unused")) 
                    {
                } else if(d_config_3.mode == String("input")) 
                {
                    pinMode(INPUT_PIN_3, INPUT);
                } else if(d_config_3.mode == String("pull up")) 
                {
                    pinMode(INPUT_PIN_3, INPUT_PULLUP);
                } else if(d_config_3.mode == String("pull down")) 
                {
                    pinMode(INPUT_PIN_3, INPUT_PULLDOWN);
                }
            }
            { //interrupt mode
                if(d_config_3.int_mode == String("unused")) 
                {
                } else if(d_config_3.int_mode == String("rising")) 
                {
                    attachInterrupt(INPUT_PIN_3, input_3_isr, RISING);
                } else if(d_config_3.int_mode == String("falling")) 
                {
                    attachInterrupt(INPUT_PIN_3, input_3_isr, FALLING);
                } else if(d_config_3.int_mode == String("change")) 
                {
                    attachInterrupt(INPUT_PIN_3, input_3_isr, CHANGE);
                }
            }
        }
        { // input 4
            { //mode
                if(d_config_4.mode == String("unused")) 
                {
                } else if(d_config_4.mode == String("input")) 
                {
                    pinMode(INPUT_PIN_4, INPUT);
                } else if(d_config_4.mode == String("pull up")) 
                {
                    pinMode(INPUT_PIN_4, INPUT_PULLUP);
                } else if(d_config_4.mode == String("pull down")) 
                {
                    pinMode(INPUT_PIN_4, INPUT_PULLDOWN);
                }
            }
            { //interrupt mode
                if(d_config_4.int_mode == String("unused")) 
                {
                } else if(d_config_4.int_mode == String("rising")) 
                {
                    attachInterrupt(INPUT_PIN_4, input_4_isr, RISING);
                } else if(d_config_4.int_mode == String("falling")) 
                {
                    attachInterrupt(INPUT_PIN_4, input_4_isr, FALLING);
                } else if(d_config_4.int_mode == String("change")) 
                {
                    attachInterrupt(INPUT_PIN_4, input_4_isr, CHANGE);
                }
            }
        }
    }
    while (1)
    {
        input infor;
        xQueueReceive(input_state_queue, &infor, portMAX_DELAY);
        doc["input"] = infor.input;
        doc["state"] = infor.state;
        espnow_packet packet;
        memcpy(packet.mac, hub_mac, ESP_NOW_ETH_ALEN);
        serializeJson(doc, packet.data);
        if((infor.input == String("input 1")) && (d_config_1.mode != String("unused")))
        {
            xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
        } else if((infor.input == String("input 2")) && (d_config_2.mode != String("unused")))
        {
            xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
        } else if((infor.input == String("input 3")) && (d_config_3.mode != String("unused")))
        {
            xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
        } else if((infor.input == String("input 4")) && (d_config_4.mode != String("unused")))
        {
            xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
        }
    }
}

void set_analog_config_task(void *pvParameters) {
    set_analog_config_queue = xQueueCreate(3, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    while (1)
    {
        espnow_packet packet;
        xQueueReceive(set_analog_config_queue, &packet, portMAX_DELAY);
        deserializeJson(doc, packet.data);
        if(String((const char*)doc["analog"]) == "analog 1") 
        {
            a_config_1 = String((const char*)doc["mode"]);
        } else if(String((const char*)doc["analog"]) == "analog 2") 
        {
            a_config_2 = String((const char*)doc["mode"]);
        }
        pref.begin(doc["analog"], false);
        pref.putString("mode", String((const char*)doc["mode"]));
        pref.end();
        String analog = String((const char*)doc["analog"]);
        xQueueSend(analog_config_queue, &analog, portMAX_DELAY);
    }
}

void get_analog_config_task(void *pvParameters) {
    get_analog_config_queue = xQueueCreate(3, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    while (1)
    {
        espnow_packet packet;
        xQueueReceive(get_analog_config_queue, &packet, portMAX_DELAY);
        deserializeJson(doc, packet.data);
        String analog = String((const char*)doc["analog"]);
        xQueueSend(analog_config_queue, &analog, portMAX_DELAY);
    }
}

void analog_config_task(void *pvParameters) {
    analog_config_queue = xQueueCreate(3, sizeof(String));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    doc["object"] = String("analog config");
    while (1)
    {
        String analog;
        xQueueReceive(input_config_queue, &analog, portMAX_DELAY);
        doc["node"] = node;
        doc["analog"] = analog;
        if(analog == String("analog 1")) {
            pref.begin("analog 1", true);
        } else if(analog == String("analog 2")) {
            pref.begin("analog 2", true);
        }
        doc["mode"] = pref.getString("mode", "unused");
        pref.end();
        espnow_packet packet;
        memcpy(packet.mac, hub_mac, ESP_NOW_ETH_ALEN);
        serializeJson(doc, packet.data);
        xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
    }
}

void get_analog_value_task(void *pvParameters) {
    get_analog_value_queue = xQueueCreate(3, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    while (1)
    {
        espnow_packet packet;
        xQueueReceive(get_analog_value_queue, &packet, portMAX_DELAY);
        deserializeJson(doc, packet.data);
        String analog = String((const char*)doc["analog"]);
        xQueueSend(analog_value_queue, &analog, portMAX_DELAY);
    }
}

void analog_value_task(void *pvParameters) {
    analog_value_queue = xQueueCreate(10, sizeof(String));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    doc["object"] = String("analog state");
    doc["node"] = node;
    { // get config
        { //analog config 1
            pref.begin("analog 1", true);
            a_config_1 = pref.getString("mode", "unused");
            pref.end();
        }
        { //analog config 2
            pref.begin("analog 2", true);
            a_config_2 = pref.getString("mode", "unused");
            pref.end();
        }
    }
    { // setup analog
        { // analog 1
            { //mode
                if(a_config_1 == String("unused")) 
                {
                } else if(a_config_1 == String("used"))
                {
                    pinMode(ANALOG_PIN_1, ANALOG);
                }
            }
        }
        { // analog 2
            { //mode
                if(a_config_2 == String("unused")) 
                {
                } else if(d_config_2.mode == String("used")) 
                {
                    pinMode(ANALOG_PIN_2, ANALOG);
                }
            }
        }
    }
    while (1)
    {
        String analog;
        xQueueReceive(analog_value_queue, &analog, portMAX_DELAY);
        int val = 0;
        if(analog == String("analog 1")) {
            val = analogRead(ANALOG_PIN_1);
        } else if(analog == String("analog 2")) {
            val = analogRead(ANALOG_PIN_2);
        }
        doc["analog"] = analog;
        doc["value"] = String(val);
        espnow_packet packet;
        memcpy(packet.mac, hub_mac, ESP_NOW_ETH_ALEN);
        serializeJson(doc, packet.data);
        if((String((const char*)doc["analog"]) == String("analog 1")) && (a_config_1 != String("unused")))
        {
            xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
        } else if((String((const char*)doc["analog"]) == String("analog 2")) && (a_config_2 != String("unused")))
        {
            xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
        }
    }
}

void control_output_state_task(void *pvParameters) 
{
    control_ouput_state_queue = xQueueCreate(5, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    pinMode(OUTPUT_PIN_1, OUTPUT);
    digitalWrite(OUTPUT_PIN_1, LOW);
    pinMode(OUTPUT_PIN_2, OUTPUT);
    digitalWrite(OUTPUT_PIN_2, LOW);
    pinMode(OUTPUT_PIN_3, OUTPUT);
    digitalWrite(OUTPUT_PIN_3, LOW);
    pinMode(OUTPUT_PIN_4, OUTPUT);
    digitalWrite(OUTPUT_PIN_4, LOW);
    while (1)
    {
        espnow_packet packet;
        xQueueReceive(control_ouput_state_queue, &packet, portMAX_DELAY);
        deserializeJson(doc, packet.data);
        String output = String((const char *)doc["output"]);
        String state = String((const char*)doc["state"]);
        uint8_t pin;
        if(output == String("output 1")) 
        {
            pin = OUTPUT_PIN_1;
        } else if(output == String("output 2")) 
        {
            pin = OUTPUT_PIN_2;
        } else if(output == String("output 3")) 
        {
            pin = OUTPUT_PIN_3;            
        } else if(output == String("output 4")) 
        {
            pin = OUTPUT_PIN_4;           
        }
        if(state == String("HIGH")) 
        {
            digitalWrite(pin, HIGH);
        } else if(state == String("LOW")) 
        {
            digitalWrite(pin, LOW);
        } else if(state == String("CHANGE")) 
        {
            digitalWrite(pin, digitalRead(pin)?LOW:HIGH);
        }
        xQueueSend(output_state_queue, &output, portMAX_DELAY);
    }
}

void get_output_state_task(void *pvParameters) 
{
    get_output_state_queue = xQueueCreate(5, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    while (1)
    {
        espnow_packet packet;
        xQueueReceive(get_output_state_queue, &packet, portMAX_DELAY);
        deserializeJson(doc, packet.data);
        String output = String((const char *)doc["output"]);
        xQueueSend(output_state_queue, &output, portMAX_DELAY);
    }
}

void output_state_task(void *pvParameters)
{
    output_state_queue = xQueueCreate(10, sizeof(String));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    doc["object"] = String("output state");
    doc["node"] = node;
    while(1) 
    {
        String output;
        xQueueReceive(output_state_queue, &output, portMAX_DELAY);
        uint8_t pin;
        doc["output"] = output;
        if(output == String("output 1")) {
            pin = OUTPUT_PIN_1;
        } else if(output == String("output 2")) {
            pin = OUTPUT_PIN_2;
        } else if(output == String("output 3")) {
            pin = OUTPUT_PIN_3;            
        } else if(output == String("output 4")) {
            pin = OUTPUT_PIN_4;           
        }
        String buffer;
        if(digitalRead(pin)) {
            buffer = String("HIGH");
        } else {
            buffer = String("LOW");
        }
        doc["state"] = buffer;
        espnow_packet packet;
        memcpy(packet.mac, hub_mac, ESP_NOW_ETH_ALEN);
        serializeJson(doc, packet.data);
        xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
    }
}

void trigger_buzzer_task(void *pvParameters) 
{
    trigger_buzzer_queue = xQueueCreate(3, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH);
    while (1)
    {
        espnow_packet packet;
        xQueueReceive(trigger_buzzer_queue, &packet, portMAX_DELAY);
        deserializeJson(doc, packet.data);
        String s_interval = String((const char*)doc["interval"]);
        int interval = atoi(s_interval.c_str());
        digitalWrite(BUZZER_PIN, LOW);
        vTaskDelay(interval*1000 / portTICK_PERIOD_MS);
        digitalWrite(BUZZER_PIN, HIGH);
    }
}

void restart_task(void *pvParameters) 
{
    restart_queue = xQueueCreate(1, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    while (1)
    {
        espnow_packet packet;
        xQueueReceive(restart_queue, &packet, portMAX_DELAY);
        ESP.restart();
    }
}

void clear_eeprom_task(void *pvParameters) 
{
    pinMode(EEPROM_PIN, INPUT_PULLUP);
    attachInterrupt(EEPROM_PIN, eeprom_pin_isr, FALLING);
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        pref.begin("infor", false);
        pref.putString("hub_mac", "");
        pref.putString("node", "");
        pref.end();
        blink_led(5, 200);
        ESP.restart();
    }
}
void touch_1_task(void *pvParameters) 
{
    pinMode(TOUCH_PIN_1, INPUT_PULLDOWN);
    attachInterrupt(TOUCH_PIN_1, touch_1_isr, RISING);
    touch_1_smp = xSemaphoreCreateBinary();
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    doc["object"] = String("touch button");
    doc["node"] = node;
    doc["touch"] = "touch 1";
    while (1)
    {
        uint8_t time = 0;
        if( xSemaphoreTake( touch_1_smp , portMAX_DELAY) == pdTRUE ) {
            time++;
            while ( xSemaphoreTake ( touch_1_smp , 500 / portTICK_PERIOD_MS) == pdTRUE) {
                time++;
                if(time == 3) break;
            }
            if( (time == 1) && digitalRead(TOUCH_PIN_1) ) {
                doc["detect"] = String("long");
            } else {
                switch (time)
                {
                case 1:
                    doc["detect"] = String("single");
                    break;
                case 2:
                    doc["detect"] = String("double");
                    break;
                case 3:
                    doc["detect"] = String("triple");   
                    break;
                }
            }
            espnow_packet packet;
            memcpy(packet.mac, hub_mac, ESP_NOW_ETH_ALEN);
            serializeJson(doc, packet.data);
            xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
        }
    }
}
void touch_2_task(void *pvParameters) 
{
    pinMode(TOUCH_PIN_2, INPUT_PULLDOWN);
    attachInterrupt(TOUCH_PIN_2, touch_2_isr, RISING);
    touch_2_smp = xSemaphoreCreateBinary();
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    doc["object"] = String("touch button");
    doc["node"] = node;
    doc["touch"] = "touch 2";
    while (1)
    {
        uint8_t time = 0;
        if( xSemaphoreTake( touch_2_smp , portMAX_DELAY) == pdTRUE ) {
            time++;
            while ( xSemaphoreTake ( touch_2_smp , 500 / portTICK_PERIOD_MS) == pdTRUE) {
                time++;
                if(time == 3) break;
            }
            if( (time == 1) && digitalRead(TOUCH_PIN_2) ) {
                doc["detect"] = String("long");
            } else {
                switch (time)
                {
                case 1:
                    doc["detect"] = String("single");
                    break;
                case 2:
                    doc["detect"] = String("double");
                    break;
                case 3:
                    doc["detect"] = String("triple");   
                    break;
                }
            }
            espnow_packet packet;
            memcpy(packet.mac, hub_mac, ESP_NOW_ETH_ALEN);
            serializeJson(doc, packet.data);
            xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
        }
    }
}
void touch_3_task(void *pvParameters) 
{
    pinMode(TOUCH_PIN_3, INPUT_PULLDOWN);
    attachInterrupt(TOUCH_PIN_3, touch_3_isr, RISING);
    touch_3_smp = xSemaphoreCreateBinary();
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    doc["object"] = String("touch button");
    doc["node"] = node;
    doc["touch"] = "touch 3";
    while (1)
    {
        uint8_t time = 0;
        if( xSemaphoreTake( touch_3_smp , portMAX_DELAY) == pdTRUE ) {
            time++;
            while ( xSemaphoreTake ( touch_3_smp , 500 / portTICK_PERIOD_MS) == pdTRUE) {
                time++;
                if(time == 3) break;
            }
            if( (time == 1) && digitalRead(TOUCH_PIN_3) ) {
                doc["detect"] = String("long");
            } else {
                switch (time)
                {
                case 1:
                    doc["detect"] = String("single");
                    break;
                case 2:
                    doc["detect"] = String("double");
                    break;
                case 3:
                    doc["detect"] = String("triple");   
                    break;
                }
            }
            espnow_packet packet;
            memcpy(packet.mac, hub_mac, ESP_NOW_ETH_ALEN);
            serializeJson(doc, packet.data);
            xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
        }
    }
}
void touch_4_task(void *pvParameters) 
{
    pinMode(TOUCH_PIN_4, INPUT_PULLDOWN);
    attachInterrupt(TOUCH_PIN_4, touch_4_isr, RISING);
    touch_4_smp = xSemaphoreCreateBinary();
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    doc["object"] = String("touch button");
    doc["node"] = node;
    doc["touch"] = "touch 4";
    while (1)
    {
        uint8_t time = 0;
        if( xSemaphoreTake( touch_4_smp , portMAX_DELAY) == pdTRUE ) {
            time++;
            while ( xSemaphoreTake ( touch_4_smp , 500 / portTICK_PERIOD_MS) == pdTRUE) {
                time++;
                if(time == 3) break;
            }
            if( (time == 1) && digitalRead(TOUCH_PIN_4) ) {
                doc["detect"] = String("long");
            } else {
                switch (time)
                {
                case 1:
                    doc["detect"] = String("single");
                    break;
                case 2:
                    doc["detect"] = String("double");
                    break;
                case 3:
                    doc["detect"] = String("triple");   
                    break;
                }
            }
            espnow_packet packet;
            memcpy(packet.mac, hub_mac, ESP_NOW_ETH_ALEN);
            serializeJson(doc, packet.data);
            xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
        }
    }
}
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
    if(status == ESP_NOW_SEND_SUCCESS) {
        espnow_send_success = true;
    }
}
void espnow_send_task(void *pvParameter) 
{
    espnow_send_queue = xQueueCreate(10, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    peer.channel = 1;
    peer.encrypt = false;
    while(1) {
        espnow_packet packet;
        xQueueReceive(espnow_send_queue, &packet, portMAX_DELAY);
        memcpy(peer.peer_addr, packet.mac, ESP_NOW_ETH_ALEN);
        esp_now_add_peer(&peer);
        espnow_send_success = false;
        esp_now_send(packet.mac, packet.data, strlen((char *)packet.data));
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if(espnow_send_success != true) {
            doc["object"] = String("send error");
            doc["node"] = node;
            doc["payload"] = String("send fail");
            espnow_packet temp_packet;
            memcpy(temp_packet.mac, hub_mac, ESP_NOW_ETH_ALEN);
            serializeJson(doc, temp_packet.data);
            xQueueSend(espnow_send_queue, &temp_packet, portMAX_DELAY);
        }
        esp_now_del_peer(packet.mac);
    }
}
void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len) 
{
    espnow_packet packet;
    memcpy(packet.mac, mac_addr, ESP_NOW_ETH_ALEN);
    memcpy(packet.data, data, data_len);
    xQueueSendFromISR(espnow_recv_queue, &packet, NULL);
}
void process_espnow_packet_task(void *pvParameters) 
{
    espnow_recv_queue = xQueueCreate(10, sizeof(espnow_packet));
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    while (1)
    {
        espnow_packet packet;
        bool check_except = true;
        xQueueReceive(espnow_recv_queue, &packet, portMAX_DELAY);
        espnow_packet buffer = packet;
        check_except &= (deserializeJson(doc, buffer.data) == DeserializationError::Ok);
        check_except &= (String((const char *)doc["node"]) == node);
        check_except &= (String((const char *)doc["object"]) != String("add node"));
        if(check_except) {
            String object = String((const char *)doc["object"]);
            if(object == String("delete node")) 
            {
                xQueueSend(delete_node_queue, &packet, portMAX_DELAY);
            } else if(object == String("add subnode")) 
            {
                xQueueSend(add_subnode_queue, &packet, portMAX_DELAY);
            } else if(object == String("set input config")) 
            {
                xQueueSend(set_input_config_queue, &packet, portMAX_DELAY);
            } else if(object == String("get input config")) 
            {
                xQueueSend(get_input_config_queue, &packet, portMAX_DELAY);
            } else if(object == String("get input state")) 
            {
                xQueueSend(get_input_state_queue, &packet, portMAX_DELAY);
            } else if(object == String("control output")) 
            {
                xQueueSend(control_ouput_state_queue, &packet, portMAX_DELAY);
            } else if(object == String("get output state")) 
            {
                xQueueSend(get_output_state_queue, &packet, portMAX_DELAY);
            } else if(object == String("trigger buzzer")) 
            {
                xQueueSend(trigger_buzzer_queue, &packet, portMAX_DELAY);
            } else if(object == String("restart")) 
            {
                xQueueSend(restart_queue, &packet, portMAX_DELAY);
            } else {
                memcpy(packet.mac, hub_mac, ESP_NOW_ETH_ALEN);
                xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
            }
        }
    }
}
void process_espnow_packet_task_in_config(void *pvParameters) 
{
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    while(1) {
        espnow_packet packet;
        xQueueReceive(espnow_recv_queue, &packet, portMAX_DELAY);
        if(!deserializeJson(doc, packet.data)) {
            String object = String((const char *)doc["object"]);
            if(object == "add node") {
                node = String((const char*)doc["node"]);
                memcpy(hub_mac, packet.mac, ESP_NOW_ETH_ALEN);
                xTaskNotifyGive(send_confirm_task_handle);
            }
        }
    }
}
void send_espnow_confirm_packet_task(void *pvParameters) 
{
    DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
    doc["object"] = "confirm add node";
    doc["mac"] = WiFi.macAddress();
    uint8_t buffer[ESP_NOW_MAX_DATA_LEN];
    peer.channel = 1;
    peer.encrypt = false;
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        doc["node"] = node;
        memcpy(peer.peer_addr, hub_mac, ESP_NOW_ETH_ALEN);
        esp_now_add_peer(&peer);
        serializeJson(doc, buffer);
        espnow_send_success = false;
        esp_now_send(hub_mac, buffer, strlen((char *)buffer));
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if(espnow_send_success == true) {
            pref.begin("infor", false);
            pref.putString("node", node);
            pref.putString("hub_mac", String((const char*)hub_mac));
            pref.end();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ESP.restart();
        }
    }
}
void get_infor() 
{
    pref.begin("infor", true);
    String pv_mac = pref.getString("hub_mac", "");
    memcpy(hub_mac, pv_mac.c_str(), ESP_NOW_ETH_ALEN);
    node = pref.getString("node", "");
    pref.end();
    if( (String((char*)hub_mac) != "") && (node != "") ) {
        char macStr[18] = { 0 };
        sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", hub_mac[0], hub_mac[1], hub_mac[2], hub_mac[3], hub_mac[4], hub_mac[5]);
        s_hub_mac = String(macStr);
        return;
    } else {
        espnow_recv_queue = xQueueCreate(10, sizeof(espnow_packet));
        setup_espnow(espnow_send_cb, espnow_recv_cb);
        xTaskCreate(process_espnow_packet_task_in_config, "infor recv", 2048, NULL, 10, NULL);
        xTaskCreate(send_espnow_confirm_packet_task, "confirm send", 2048, NULL, 10, &send_confirm_task_handle);
    }
    while(1) {
        blink_led(1, 200);
    }
}

void setup() 
{
    Serial.begin(115200);
    xTaskCreate(clear_eeprom_task, "clear eeprom", 2048, NULL, 10, &clear_eeprom_task_handle);
    get_infor();
    xTaskCreate(delete_node_task, "delete node", 4096, NULL, 10, NULL);
    xTaskCreate(add_subnode_task, "add subnode", 4096, NULL, 10, NULL);
    xTaskCreate(set_input_config_task, "set config", 4096, NULL, 10, NULL);
    xTaskCreate(get_input_config_task, "get config", 4096, NULL, 10, NULL);
    xTaskCreate(input_config_task, "input config", 4096, NULL, 10, NULL);
    xTaskCreate(get_input_state_task, "get input state", 4096, NULL, 10, NULL);
    xTaskCreate(input_state_task, "input state", 4096, NULL, 10, NULL);
    xTaskCreate(set_analog_config_task, "set a config", 4096, NULL, 10, NULL);
    xTaskCreate(get_analog_config_task, "get a config", 4096, NULL, 10, NULL);
    xTaskCreate(analog_config_task, "analog config", 4096, NULL, 10, NULL);
    xTaskCreate(get_analog_value_task, "get analog val", 4096, NULL, 10, NULL);
    xTaskCreate(analog_value_task, "analog value", 4096, NULL, 10, NULL);
    xTaskCreate(control_output_state_task, "control output", 4096, NULL, 10, NULL);
    xTaskCreate(get_output_state_task, "get output", 4096, NULL, 10, NULL);
    xTaskCreate(output_state_task, "output state", 4096, NULL, 10, NULL);
    xTaskCreate(trigger_buzzer_task, "trigger buzzer", 2048, NULL, 10, NULL);
    xTaskCreate(restart_task, "restart task", 2048, NULL, 10, NULL);
    xTaskCreate(touch_1_task, "touch 1 task", 4096, NULL, 10, NULL);
    xTaskCreate(touch_2_task, "touch 2 task", 4096, NULL, 10, NULL);
    xTaskCreate(touch_3_task, "touch 3 task", 4096, NULL, 10, NULL);
    xTaskCreate(touch_4_task, "touch 4 task", 4096, NULL, 10, NULL);
    xTaskCreate(espnow_send_task, "send espnow", 4096, NULL, 10, NULL);
    xTaskCreate(process_espnow_packet_task, "process espnow", 4096, NULL, 10, NULL);
    setup_espnow(espnow_send_cb, espnow_recv_cb);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);
}

void loop()
{

}