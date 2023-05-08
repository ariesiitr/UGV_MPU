#include <SPI.h>
#include <ESP8266WiFi.h>
#include <espnow.h>


// Motor A connections
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

// Create a struct_message called myData
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

  // Turn off motors - Initial state
 digitalWrite(pin1, LOW);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, LOW);}
  

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info



void right() {
 digitalWrite(pin1, HIGH);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, LOW);
}

void backward() {
  digitalWrite(pin2, HIGH);
	digitalWrite(pin1, LOW);
	digitalWrite(pin4, HIGH);
	digitalWrite(pin3, LOW);
}


void forward() {
  digitalWrite(pin1, HIGH);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, HIGH);
	digitalWrite(pin4, LOW);
}

void left() {
 digitalWrite(pin1, LOW);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, HIGH);
	digitalWrite(pin4, LOW);
}


void stop() {
 digitalWrite(pin1, LOW);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, LOW);
}

void loop() {

  Serial.print("HEllo");
  analogWrite(pin5, 255);
	analogWrite(pin6, 255);

	// forward();
  // delay(1000);
  // backward();
  // delay(1000);
  // right();
  // delay(1000);
  // left();
  // delay(1000);
  // stop();
  // delay(1000);

	
 if (Angles.p < -17) {
   forward();
 } else if (Angles.p > 20) {
   backward();
 } else if (Angles.r > 30) {
   left();
 } else if (Angles.r < -30) {
   right();
 } else {
   stop();
 }
Serial.println(Angles.r);
Serial.println(Angles.p);

delay(2000);
}
