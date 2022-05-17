//Include necessary libraries
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <MFRC522.h>
#include <SPI.h>

#define SS_PIN    21
#define RST_PIN   22
#define SIZE_BUFFER     18
#define MAX_SIZE_BLOCK  16
#define greenPin     12
#define redPin       32

MFRC522::MIFARE_Key key;

MFRC522::StatusCode status;

MFRC522 mfrc522(SS_PIN, RST_PIN); 

void readingData() {
  mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid)); 
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  byte buffer[SIZE_BUFFER] = {0};
  byte block = 2;
  byte size = SIZE_BUFFER;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Authentication failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    digitalWrite(redPin, HIGH);
    delay(1000);
    digitalWrite(redPin, LOW);
    return;
  }
  status = mfrc522.MIFARE_Read(block, buffer, &size);
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Reading failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    digitalWrite(redPin, HIGH);
    delay(1000);
    digitalWrite(redPin, LOW);
    return;
  } else {
      digitalWrite(greenPin, HIGH);
      delay(1000);
      digitalWrite(greenPin, LOW);
  }
  Serial.print(F("\nData from block ["));
  Serial.print(block);
  Serial.print(F("]: "));
  for(uint8_t i = 0; i < MAX_SIZE_BLOCK; i++) {
    Serial.write(buffer[i]);
  }
  Serial.println(" ");
}

void writingData() {
  mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid)); 
  Serial.setTimeout(30000L) ;     
  Serial.println(F("Enter the data to be written with the '#' character at the end \n[maximum of 16 characters]:"));
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  byte buffer[MAX_SIZE_BLOCK] = "";
  byte block;
  byte dataSize;
  dataSize = Serial.readBytesUntil('#', (char*)buffer, MAX_SIZE_BLOCK);
  for(byte i=dataSize; i < MAX_SIZE_BLOCK; i++) {
    buffer[i] = ' ';
  }
  block = 1;
  String str = (char*)buffer;
  Serial.println(str);
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("PCD_Authenticate() failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    digitalWrite(redPin, HIGH);
    delay(1000);
    digitalWrite(redPin, LOW);
    return;
  }
  status = mfrc522.MIFARE_Write(block, buffer, MAX_SIZE_BLOCK);
  if(status != MFRC522::STATUS_OK) {
    Serial.print(F("MIFARE_Write() failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    digitalWrite(redPin, HIGH);
    delay(1000);
    digitalWrite(redPin, LOW);
    return;
  }
  else{
    Serial.println(F("MIFARE_Write() success "));
    digitalWrite(greenPin, HIGH);
    delay(1000);
    digitalWrite(greenPin, LOW);
  }
}

int menu() {
  Serial.println(F("\nChoose an option:"));
  Serial.println(F("0 - Reading data"));
  Serial.println(F("1 - Writing data\n"));

  while(!Serial.available());

  int op = (int)Serial.read();

  while(Serial.available()) {
    if(Serial.read() == '\n') break; 
    Serial.read();
  }
  return (op - '0');
}

void setup() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  mfrc522.PCD_Init(); 
  Serial.println("Approach your reader card...");
  Serial.println();
}

void loop() {
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
    return;
  }
  if ( ! mfrc522.PICC_ReadCardSerial()) {
    return;
  }
  int op = menu();
  if(op == 0) {
    readingData();
  } else if(op == 1) {
    writingData();
  } else {
    Serial.println(F("Incorrect Option!"));
    return;
  }
  mfrc522.PICC_HaltA(); 
  mfrc522.PCD_StopCrypto1();  
}