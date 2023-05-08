#include <SPI.h>
#include<Wire.h>

#include <ESP8266WiFi.h>
#include <espnow.h>

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress1[] = {0xDC, 0x4F, 0x22, 0x76, 0x0D, 0xBC};



const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

// RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";
typedef struct test_struct {
    float r;
float p; 
float y;
} test_struct;
test_struct Angles;

// struct Angles {

// float r;
// float p; 
// float y;

// };
unsigned long lastTime = 0;  
unsigned long timerDelay = 2000;  // send readings timer

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  char macStr[18];
  Serial.print("Packet to:");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

   WiFi.mode(WIFI_STA);
  WiFi.disconnect();
   // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress1, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);



  // radio.begin();
  // radio.openWritingPipe(address);
  // radio.setPALevel(RF24_PA_MIN);
  // radio.stopListening();
  //calculate_IMU_error();
}

void loop() {
   // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  // roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  // pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  roll=AccX;
  pitch=AccY;

  //Angles data;
  Angles.r = roll;
  Angles.p = pitch;
  Angles.y = yaw;
  esp_now_send(0, (uint8_t *) &Angles, sizeof(Angles));
  // radio.write(&data, sizeof(data));
  Serial.println("Hello");
  Serial.println(roll);
  Serial.println(pitch);
  Serial.println(yaw);

  
  delay(1000);
}

// void calculate_IMU_error() {
//   // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
//   // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
//   // Read accelerometer values 200 times
//   while (c < 200) {
//     Wire.beginTransmission(MPU);
//     Wire.write(0x3B);
//     Wire.endTransmission(false);
//     Wire.requestFrom(MPU, 6, true);
//     AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//     AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//     AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//     // Sum all readings
//     AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
//     AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
//     c++;
//   }
//   //Divide the sum by 200 to get the error value
//   AccErrorX = AccErrorX / 200;
//   AccErrorY = AccErrorY / 200;
//   c = 0;
//   // Read gyro values 200 times
//   while (c < 200) {
//     Wire.beginTransmission(MPU);
//     Wire.write(0x43);
//     Wire.endTransmission(false);
//     Wire.requestFrom(MPU, 6, true);
//     GyroX = Wire.read() << 8 | Wire.read();
//     GyroY = Wire.read() << 8 | Wire.read();
//     GyroZ = Wire.read() << 8 | Wire.read();
//     // Sum all readings
//     GyroErrorX = GyroErrorX + (GyroX / 131.0);
//     GyroErrorY = GyroErrorY + (GyroY / 131.0);
//     GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
//     c++;
//   }
//   //Divide the sum by 200 to get the error value
//   GyroErrorX = GyroErrorX / 200;
//   GyroErrorY = GyroErrorY / 200;
//   GyroErrorZ = GyroErrorZ / 200;
//   // Print the error values on the Serial Monitor
//   Serial.print("AccErrorX: ");
//   Serial.println(AccErrorX);
//   Serial.print("AccErrorY: ");
//   Serial.println(AccErrorY);
//   Serial.print("GyroErrorX: ");
//   Serial.println(GyroErrorX);
//   Serial.print("GyroErrorY: ");
//   Serial.println(GyroErrorY);
//   Serial.print("GyroErrorZ: ");
//   Serial.println(GyroErrorZ);
// }
