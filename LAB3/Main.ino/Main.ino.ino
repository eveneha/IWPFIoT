#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEDevice.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

Adafruit_MPU6050 mpu;

double gyroX, gyroY, gyroZ, accX, accY, accZ, roll, pitch, yaw = 0.0;
double rollDeg, pitchDeg, yawDeg = 0.0; 
const double alpha = 0.98; // Filter coefficient  
unsigned long init_time = 0;
float dt = 0.0;  
BLECharacteristic *pCharacteristic;

double normalizeAngle(double angle) {
  angle  = fmod(angle, 2 * M_PI);
  if (angle <0) {
    angle += 2* M_PI; 
  }
  return angle* 180 / M_PI; 

}

void updateOrientation(){
  
  roll += gyroX * dt; 
  pitch += gyroY * dt; 
  yaw += gyroZ * dt; 

  roll = normalizeAngle(roll);
  pitch = normalizeAngle(pitch);
  yaw = normalizeAngle(yaw);

  double accAngleX = atan2(accY, accZ); 
  double accAngleY = atan2(-accX, sqrt(accY * accY + accZ * accZ)); 

  roll = alpha * (roll + gyroX *dt) + (1 - alpha) * accAngleX;
  pitch = alpha * (roll + gyroY *dt) + (1 - alpha) * accAngleY;
}



void setup() {
  // Sensor setup -----------------------------------------------------------------------
  Serial.begin(115200);
  while (!Serial)
    delay(10); 

  Serial.println("Adafruit MPU6050 test!");

  Wire.begin(18,19);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);




  // BLE setup --------------------------------------------------------------------------
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEDevice::init("Long name works now");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_BROADCAST
                                       );

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);

  
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long real_time = millis();
  dt = (real_time - init_time) / 1000.0; 

  // Calcular roll, pitch y yaw a partir de los ángulos de orientación
  accX = a.acceleration.x; 
  accY = a.acceleration.y; 
  accZ = a.acceleration.z; 
  gyroX = g.gyro.x; 
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
  updateOrientation(); 
  init_time = real_time; 

  // seting up data for broadcasting
  String data = "Roll: " + String(roll) + ", Pitch: " + String(pitch) + ", Yaw: " + String(yaw);
  pCharacteristic->setValue(data.c_str());

  // printing in serial monitor
  Serial.println("");
  Serial.print("Angle X: ");
  Serial.print(roll);
  Serial.print(" deg, Angle Y: ");
  Serial.print(pitch);
  Serial.print(" deg, Angle Z: ");
  Serial.print(yaw);
  Serial.println(" deg");

  delay(2000);
}