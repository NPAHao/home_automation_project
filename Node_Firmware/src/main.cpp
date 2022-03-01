//Include essential libraries
#include "main.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <DHT.h>
#include <string.h>


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


//Declare semaphore
SemaphoreHandle_t   touch_1_smp = NULL;
SemaphoreHandle_t   touch_2_smp = NULL;
SemaphoreHandle_t   touch_3_smp = NULL;
SemaphoreHandle_t   touch_4_smp = NULL;


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
    xSemaphoreGiveFromISR(touch_1_smp , NULL);
}
void IRAM_ATTR touch_pin_2_isr() {
    xSemaphoreGiveFromISR(touch_2_smp , NULL);
}
void IRAM_ATTR touch_pin_3_isr() {
    xSemaphoreGiveFromISR(touch_3_smp , NULL);
}
void IRAM_ATTR touch_pin_4_isr() {
    xSemaphoreGiveFromISR(touch_4_smp , NULL);
}
void IRAM_ATTR d_input_pin_1_isr() {
    client.publish( strcat( (char*)mqtt.user_name, "d_input1"), "strigg");
}
void IRAM_ATTR d_input_pin_2_isr() {
    client.publish( strcat( (char*)mqtt.user_name, "d_input2"), "strigg");
}
void IRAM_ATTR d_input_pin_3_isr() {
    client.publish( strcat( (char*)mqtt.user_name, "d_input3"), "strigg");
}
void IRAM_ATTR d_input_pin_4_isr() {
    client.publish( strcat( (char*)mqtt.user_name, "d_input4"), "strigg");
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
 * @brief This task will be used for indicating the times the button was touched
 * 
 * @param pvPara 
 */
void touch_pin_1_task(void *pvPara) {
    uint8_t time;
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
                client.publish( strcat( (char*)mqtt.user_name, "/touch1"), "longtouch");
            } else {
                switch (time)
                {
                case 1:
                    client.publish( strcat( (char*)mqtt.user_name, "/touch1"), "singletouch");
                    break;
                case 2:
                    client.publish( strcat( (char*)mqtt.user_name, "/touch1"), "doubletouch");
                    break;
                case 3:
                    client.publish( strcat( (char*)mqtt.user_name, "/touch1"), "trippletouch");
                    break;               
                }
            }
        }
    }
}


/**
 * @brief This task will be used for indicating the times the button was touched
 * 
 * @param pvPara 
 */
void touch_pin_2_task(void *pvPara) {
    uint8_t time;
    while (1)
    {
        time = 0;
        if( xSemaphoreTake( touch_2_smp , portMAX_DELAY) == pdTRUE ) {
            time++;
            while ( xSemaphoreTake ( touch_2_smp , 500 / portTICK_PERIOD_MS) == pdTRUE) {
                time++;
                if(time == 3) break;
            }
            if( (time == 1) && digitalRead(TOUCH_PIN_2) ) {
                client.publish( strcat( (char*)mqtt.device_name, "/touch2"), "longtouch");
            } else {
                switch (time)
                {
                case 1:
                    client.publish( strcat( (char*)mqtt.device_name, "/touch2"), "singletouch");
                    break;
                case 2:
                    client.publish( strcat( (char*)mqtt.device_name, "/touch2"), "doubletouch");
                    break;
                case 3:
                    client.publish( strcat( (char*)mqtt.device_name, "/touch2"), "trippletouch");
                    break;               
                }
            }
        }
    }
}

/**
 * @brief This task will be used for indicating the times the button was touched
 * 
 * @param pvPara 
 */
void touch_pin_3_task(void *pvPara) {
    uint8_t time;
    while (1)
    {
        time = 0;
        if( xSemaphoreTake( touch_3_smp , portMAX_DELAY) == pdTRUE ) {
            time++;
            while ( xSemaphoreTake ( touch_3_smp , 500 / portTICK_PERIOD_MS) == pdTRUE) {
                time++;
                if(time == 3) break;
            }
            if( (time == 1) && digitalRead(TOUCH_PIN_3) ) {
                client.publish( strcat( (char*)mqtt.device_name, "/touch3"), "longtouch");
            } else {
                switch (time)
                {
                case 1:
                    client.publish( strcat( (char*)mqtt.device_name, "/touch3"), "singletouch");
                    break;
                case 2:
                    client.publish( strcat( (char*)mqtt.device_name, "/touch3"), "doubletouch");
                    break;
                case 3:
                    client.publish( strcat( (char*)mqtt.device_name, "/touch3"), "trippletouch");
                    break;               
                }
            }
        }
    }
}


/**
 * @brief This task will be used for indicating the times the button was touched
 * 
 * @param pvPara 
 */
void touch_pin_4_task(void *pvPara) {
    uint8_t time;
    while (1)
    {
        time = 0;
        if( xSemaphoreTake( touch_4_smp , portMAX_DELAY) == pdTRUE ) {
            time++;
            while ( xSemaphoreTake ( touch_4_smp , 500 / portTICK_PERIOD_MS) == pdTRUE) {
                time++;
                if(time == 3) break;
            }
            if( (time == 1) && digitalRead(TOUCH_PIN_4) ) {
                client.publish( strcat( (char*)mqtt.device_name, "/touch4"), "longtouch");
            } else {
                switch (time)
                {
                case 1:
                    client.publish( strcat( (char*)mqtt.device_name, "/touch4"), "singletouch");
                    break;
                case 2:
                    client.publish( strcat( (char*)mqtt.device_name, "/touch4"), "doubletouch");
                    break;
                case 3:
                    client.publish( strcat( (char*)mqtt.device_name, "/touch4"), "trippletouch");
                    break;               
                }
            }
        }
    }
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
    DHT dht(DHT11_PIN, DHT11);
    dht.begin();
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount ();
    while (1)
    {   
        vTaskDelayUntil( &xLastWakeTime, 10000 / portTICK_PERIOD_MS);
        // Reading temperature or humidity takes about 250 milliseconds!
        float h = dht.readHumidity();
        // Read temperature as Celsius (the default)
        float t = dht.readTemperature();
        if (isnan(h) || isnan(t)) {
            Serial.println(F("Failed to read from DHT sensor!"));
        }
        client.publish( strcat( (char*)mqtt.device_name, "dht11_humi"), String(h).c_str());
        client.publish( strcat( (char*)mqtt.device_name, "dht11_temp"), String(t).c_str());
    }
}


/**
 * @brief WiFi setup task
 * 
 * @param pvPara 
 */
void wifi_task(void *pvPara){
    Serial.print("Connect to : ");
    Serial.println(wifi.ssid);
    WiFi.begin(wifi.ssid, wifi.password);
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
    vTaskDelete(wifi_task_handle);    
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
    client.setServer(mqtt.server, 1883);
    client.setCallback(mqtt_callback);
    while(1) {
        while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("WiFiClient")) {
        Serial.println("connected");
        // Subscribe
        client.subscribe( strcat( (char*)mqtt.device_name, "/output1" ) );
        client.subscribe( strcat( (char*)mqtt.device_name, "/output2" ) );
        client.subscribe( strcat( (char*)mqtt.device_name, "/output3" ) );
        client.subscribe( strcat( (char*)mqtt.device_name, "/output4" ) );
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

    touch_1_smp = xSemaphoreCreateBinary();
    touch_2_smp = xSemaphoreCreateBinary();
    touch_3_smp = xSemaphoreCreateBinary();
    touch_4_smp = xSemaphoreCreateBinary();

    xTaskCreate(blink_led_task, "Blink LED Task", 1024, NULL, 10, &blink_led_handle);

    xTaskCreate(wifi_task, "WiFi_task", 3072, NULL, 10, &wifi_task_handle);

    vTaskDelete(NULL);
}


/**
 * @brief This task will be delete, PC never reachs here
 * 
 */
void loop(){}
