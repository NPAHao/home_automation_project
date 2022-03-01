//Include essential libraries
#include "main.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>


//Define macro
#define DHT11_PIN           19
#define D_INPUT_PIN_1       21
#define D_INPUT_PIN_2       22
#define D_INPUT_PIN_3       23
#define D_INPUT_PIN_4       25
#define D_OUTPUT_PIN_1      26
#define D_OUTPUT_PIN_2      27
#define D_OUTPUT_PIN_3      32
#define D_OUTPUT_PIN_4      33
#define TOUCH_PIN_1         34
#define TOUCH_PIN_2         35
#define TOUCH_PIN_3         36
#define TOUCH_PIN_4         39



//Define typedef


//Declare global varialble
struct wifi_infor {
    const char *ssid;
    const char *password;
} wifi;

struct mqtt_infor {
    const char *server;
    const char *user_name;
    const char *password;
    const char *device_name;
} mqtt;


//Declare task handle variable
TaskHandle_t gpio_task_handle;
TaskHandle_t blink_led_handle;
TaskHandle_t dht11_task_handle;
TaskHandle_t wifi_task_handle;


//Declare Object
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;


                                //Callback Function
/**
 * @brief MQTT callback 
 * 
 * @param topic Topic that has new value
 * @param message Message is sent from that topic
 * @param length Length of message, in byte
 */
void mqtt_callback(char* topic, byte* message, unsigned int length){
    //save data into queue
}


void IRAM_ATTR touch_pin_1_isr() {
    //do something
}
void IRAM_ATTR touch_pin_2_isr() {
    //do something
}
void IRAM_ATTR touch_pin_3_isr() {
    //do something
}
void IRAM_ATTR touch_pin_4_isr() {
    //do something
}
void IRAM_ATTR d_input_pin_1_isr() {
    //do something
}
void IRAM_ATTR d_input_pin_2_isr() {
    //do something
}
void IRAM_ATTR d_input_pin_3_isr() {
    //do something
}
void IRAM_ATTR d_input_pin_4_isr() {
    //do something
}


                                //Task Define
/**
 * @brief GPIO setup and control task
 * 
 * @param pvPara 
 */
void gpio_task(void *pvPara) {
    pinMode(D_OUTPUT_PIN_1, OUTPUT);
    pinMode(D_OUTPUT_PIN_2, OUTPUT);
    pinMode(D_OUTPUT_PIN_3, OUTPUT);
    pinMode(D_OUTPUT_PIN_4, OUTPUT);
    pinMode(TOUCH_PIN_1, INPUT_PULLDOWN);
    pinMode(TOUCH_PIN_2, INPUT_PULLDOWN);
    pinMode(TOUCH_PIN_3, INPUT_PULLDOWN);
    pinMode(TOUCH_PIN_4, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN_1), touch_pin_1_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN_2), touch_pin_2_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN_3), touch_pin_3_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN_4), touch_pin_4_isr, RISING);
}

/**
 * @brief This task will setup onboard LED and blink it until the essential setup is complete
 * 
 * @param pvPara 
 */
void blink_led_task(void *pvPara) {
    pinMode(LED_BUILTIN, OUTPUT);
    while (1)
    {
        digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN)?0:1);
        vTaskDelay( 75 / portTICK_PERIOD_MS);
    }
}


void dht11_task(void *pvPara) {
    
}


/**
 * @brief WiFi setup task
 * 
 * @param pvPara 
 */
void wifi_task(void *pvPara){
    wifi_infor sudo_wifi = *(wifi_infor*)pvPara;
    Serial.print("Connect to : ");
    Serial.println(sudo_wifi.ssid);
    WiFi.begin(sudo_wifi.ssid, sudo_wifi.password);
    while (WiFi.status() != WL_CONNECTED)
    {
        vTaskDelay( 500 / portTICK_PERIOD_MS );
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("WiFi channel: ");
    Serial.println(WiFi.channel());
    vTaskDelay( 3000 / portTICK_PERIOD_MS);
    vTaskSuspend(blink_led_handle);
    digitalWrite(LED_BUILTIN, LOW);
    while (1)
    {
    }      
}


/**
 * @brief When incoming MQTT msg is complete save into queue, this task will be get change to run
 * 
 * @param pvPara 
 */
void process_mqtt_msg(void *pvPara){
    while (1)
    {
        /* code */
    }
}


/**
 * @brief MQTT task for setup and reconnect
 * 
 * @param pvPara 
 */
void mqtt_task(void *pvPara) {
    mqtt_infor sudo_mqtt = *(mqtt_infor*)pvPara;
    client.setServer(sudo_mqtt.server, 1883);
    client.setCallback(mqtt_callback);
    while(1) {
        while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("WiFiClient")) {
        Serial.println("connected");
        // Subscribe
        client.subscribe( strcat( (char*)sudo_mqtt.user_name, "/output1stt" ) );
        client.subscribe( strcat( (char*)sudo_mqtt.user_name, "/output2stt" ) );
        client.subscribe( strcat( (char*)sudo_mqtt.user_name, "/output3stt" ) );
        client.subscribe( strcat( (char*)sudo_mqtt.user_name, "/output4stt" ) );
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            }
        }
    }
}


/**
 * @brief Save credentials information pernamently task
 * 
 * @param pvPara 
 */
void save_credentials(void *pvPara) {
    
}


/**
 * @brief setup task
 * 
 */
void setup() {
    Serial.begin(115200);
    
    wifi.ssid = "NPAH";
    wifi.password = "hao12345";

    mqtt.server = "192.168.1.21";
    mqtt.user_name = "user";
    mqtt.password = "password";
    mqtt.device_name = "esp32";

    xTaskCreate(blink_led_task, "Blink LED Task", 1024, NULL, 10, &blink_led_handle);

    xTaskCreate(wifi_task, "WiFi_task", 3072, &wifi, 10, &wifi_task_handle);
    // vTaskDelete(NULL);
}


/**
 * @brief This task will be delete, PC never reachs here
 * 
 */
void loop(){}
