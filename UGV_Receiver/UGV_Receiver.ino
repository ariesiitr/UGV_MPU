#include <SPI.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

uint8_t pin1 = 5;
uint8_t pin2 = 4;
uint8_t pin3 = 0;
uint8_t pin4 = 2;
uint8_t pin5 = 12;
uint8_t pin6 = 13;

typedef struct  test_struct {
  float r;
  float p; 
  float y;
} test_struct;

test_struct Angles;

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&Angles, incomingData, sizeof(Angles));
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
  pinMode(pin5, OUTPUT);
  pinMode(pin6, OUTPUT);

  digitalWrite(pin1, LOW);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, LOW);

}

void right() {
 digitalWrite(pin1, HIGH);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, LOW);
  delay(500);
}

void backward() {
  digitalWrite(pin2, HIGH);
	digitalWrite(pin1, LOW);
	digitalWrite(pin4, HIGH);
	digitalWrite(pin3, LOW);
  for (int i = 0; i < 256; i++) {
    analogWrite(pin5, i);
    analogWrite(pin6, i);
    delay(5);
  }
  delay(200);
}


void forward() {
  digitalWrite(pin1, HIGH);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, HIGH);
	digitalWrite(pin4, LOW);
  for (int i = 0; i < 256; i++) {
    analogWrite(pin5, i);
    analogWrite(pin6, i);
    delay(5);
  }
  delay(200);
}

void left() {
 digitalWrite(pin1, LOW);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, HIGH);
	digitalWrite(pin4, LOW);
  delay(500);
}


void stop() {
  digitalWrite(pin1, LOW);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, LOW);
  delay(1000);
}

void loop() {

 if (Angles.p < -17) {
   forward();
 } 
 if (Angles.p > 20) {
   backward();
 }
 if (Angles.r > 30) {
   left();
 } 
 if (Angles.r < -30) {
   right();
 } 
 if(Angles.p >= -17 && Angles.p <= 20 && Angles.r <= 30 && Angles.r >= -30 )
 {
   stop();
 }
}