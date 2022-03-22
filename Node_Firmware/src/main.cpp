//Include essential libraries
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <DHT.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "pv_func.h"


//Declare global varialble
wifi_infor wifi;

mqtt_infor mqtt;

String *topic_list = nullptr;

//Declare task handle variable
TaskHandle_t gpio_task_handle;
TaskHandle_t dht11_task_handle;
TaskHandle_t wifi_task_handle;


//Declare Object
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;


//Declare semaphore
SemaphoreHandle_t   touch_1_smp = NULL;


                                //Callback Function
/**
 * @brief MQTT callback 
 * 
 * @param topic Topic that has new value
 * @param message Message is sent from that topic
 * @param length Length of message, in byte
 */
void mqtt_callback(char* topic, byte* message, unsigned int length){
    String topicTemp = String(topic);
    String msgTemp;
    for(int i = 0; i < length; i++) {
        msgTemp += (char)message[i];
    }

    if( topicTemp == (mqtt.device_name + "/output1") ) {
        if( msgTemp == "strigg") {
            digitalWrite(D_OUTPUT_PIN_1, digitalRead(D_OUTPUT_PIN_1)?0:1);
            client.publish( (mqtt.device_name + "/output1stt").c_str(), digitalRead(D_OUTPUT_PIN_1)?"ON":"OFF");
        }
    } else if( topicTemp == (mqtt.device_name + "/peerlistsend") ) {
        uart_msg msg;
        msg.code = ADD_PEER;
        msg.now_msg.len = 0;
        for(int i = 0; i < length; i+= 6) {
            for(int j =0; j < 6; j++) {
                msg.now_msg.mac_addr += msgTemp[i+j];
            }
            Serial2.write((uint8_t *)&msg, 9);
            msg.now_msg.mac_addr.clear();
        }
    } else if( topicTemp == (mqtt.device_name + "/alertconfirm") ) {
        //send confirm uart msg
    }
}


void IRAM_ATTR touch_pin_1_isr() {
    xSemaphoreGiveFromISR(touch_1_smp , NULL);
}

void IRAM_ATTR d_input_pin_1_isr() {
    client.publish( (mqtt.user_name+ "d_input1").c_str(), "strigg");
}


                                //Task Define
/**
 * @brief GPIO setup and control task
 * 
 * @param pvPara 
 */
void gpio_task(void *pvPara) {
    pinMode(D_OUTPUT_PIN_1, OUTPUT);

    pinMode(TOUCH_PIN_1, INPUT);

    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN_1), touch_pin_1_isr, RISING);

    attachInterrupt(digitalPinToInterrupt(D_INPUT_PIN_1), d_input_pin_1_isr, RISING);  
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
                client.publish( (mqtt.user_name + "/touch1").c_str(), "longtouch");
            } else {
                switch (time)
                {
                case 1:
                    client.publish( (mqtt.user_name + "/touch1").c_str(), "singletouch");
                    break;
                case 2:
                    client.publish( (mqtt.user_name + "/touch1").c_str(), "doubletouch");
                    break;
                case 3:
                    client.publish( (mqtt.user_name + "/touch1").c_str(), "trippletouch");
                    break;               
                }
            }
        }
    }
}


/**
 * @brief This task will setup DHT11 and send data periodly 10s
 * 
 * @param pvPara 
 */
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
        // check
        if (isnan(h) || isnan(t)) {
            Serial.println(F("Failed to read from DHT sensor!"));
        }
        //publish
        client.publish( (mqtt.device_name + "/dht11_humi").c_str(), String(h).c_str());
        client.publish( (mqtt.device_name + "/dht11_temp").c_str(), String(t).c_str());
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
    WiFi.begin(wifi.ssid.c_str(), wifi.password.c_str());
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
    while (1)
    {
        if(WiFi.status() != WL_CONNECTED) {
            vTaskDelay( 500 / portTICK_PERIOD_MS );
            Serial.print(".");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief 
 * 
 * @param pvPara 
 */
void handle_rx_msg_from_uart(void *pvPara) {
    while(1) {
        if(Serial2.available() > 0) {
            //UART msg format :   [payload_len][code][6 x MAC][data.....][data_len]
            uint8_t payload_len = Serial2.read();
            //UART msg now in buffer :         [code][6 x MAC][data.....][data_len]
            String msg;
            uart_msg uart_msg;
            Serial2.readBytes(&msg[0], payload_len);
            parse_uart_msg(&uart_msg, msg);
            switch (uart_msg.code)
            {
            case GET_PEER:
                client.publish( (mqtt.device_name + "/peerlist").c_str(), "GET" );
                break;
            case FW_MSG:
                
                break;
            }
        }
    }
}


/**
 * @brief MQTT task for setup and reconnect
 * 
 * @param pvPara 
 */
void mqtt_task(void *pvPara) {
    client.setServer(mqtt.server.c_str(), 1883);
    client.setCallback(mqtt_callback);
    while(1) {
        if(!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("WiFiClient")) {
        Serial.println("connected");
        client.publish( (mqtt.device_name + "/devicestate").c_str() , "ON");
        client.subscribe( (mqtt.device_name + "/output1").c_str() );

        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            }
        }
        client.loop();
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
    Serial2.begin(115200);
    pinMode(D_OUTPUT_PIN_1, OUTPUT);
    wifi.ssid = "Phi Hung";
    wifi.password = "26022000";

    mqtt.server = "192.168.1.10";

    mqtt.device_name = "esp32";

    touch_1_smp = xSemaphoreCreateBinary();

    xTaskCreate(wifi_task, "WiFi_task", 3072, NULL, 10, &wifi_task_handle);
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    xTaskCreate(mqtt_task, "mqtt task", 4096, NULL, 10, NULL);

    vTaskDelete(NULL);
}


/**
 * @brief This task will be delete, PC never reachs here
 * 
 */
void loop(){}
