#include <NimBLEDevice.h>
#include <Arduino_GFX_Library.h>
#include "ESP32_ISR_Servo.h"
#include "SCServo.h"

TaskHandle_t ReadBat = NULL;
TaskHandle_t WriteServo = NULL;

SemaphoreHandle_t Semaphore = NULL;


static NimBLEServer* pServer;
static NimBLECharacteristic* CharacteristicPOS;
static NimBLECharacteristic* CharacteristicFIR;
volatile static NimBLECharacteristic* CharacteristicINF;
static NimBLECharacteristic* CharacteristicINI;
static NimBLECharacteristic* CharacteristicBAT;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID                "DEAD"
#define CHARACTERISTIC_POS_UUID     "DCA1"
#define CHARACTERISTIC_FIR_UUID     "DCA2"
#define CHARACTERISTIC_INF_UUID     "DCA3"
#define CHARACTERISTIC_INI_UUID     "DCA4"
#define CHARACTERISTIC_BAT_UUID     "DCA5" //Level

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 34
#define S_TXD 33

// Select different ESP32 timer number (0-3) to avoid conflict
#define USE_ESP32_TIMER_NO          0

#define MIN_MICROS      500  //544
#define MAX_MICROS      1000
#define MIN_MICROS_S    500  //544
#define MAX_MICROS_S    1000
#define PIN_INT         48 // was 16
#define PIN_BAT         16 // ADC for BAT
#define PIN_ESC1        17
#define PIN_ESC2        18
#define PIN_SERVO1      39
#define TFT_BL          10

bool deviceConnected = false;
bool oldDeviceConnected = false;

int escIndex1   = -1;
int escIndex2   = -1;
int servoIndex1 = -1;

int pos = 0;

int speed_in = 0;
int speed_out = 0;
bool speed_first = true;
bool busy = false;

int pos_x = 0;
int pos_y = 0;
int speed_x = 0;
int speed_y = 0;
int separator_1, separator_2, separator_3, separator_4, separator_5;

volatile int fire = 0;
int fire_mode = 0;
int fire_speed = 0;
int fire_burst = 0;
int fire_auto_speed = 0;

volatile int servo_back = 125;
volatile int servo_front = 10;

// Battery BLE
uint8_t bat_level = 57;

// Motor Speed
volatile unsigned int motor_speed = 0;

/* create a hardware fire timer */
hw_timer_t * timer = NULL;

Arduino_DataBus *bus = new Arduino_ESP32SPI(6 /* DC */, 5 /* CS */, 3 /* SCK */, 2 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_GC9107(bus, 1 /* RST */, 1 /* rotation */, true /* IPS */);

SCSCL sc;

class ServerCallbacks: public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) {
      Serial.println("Client connected");
      gfx->println("Client connected");
      Serial.println("Multi-connect support: start advertising");
      NimBLEDevice::startAdvertising();
  };
  void onDisconnect(NimBLEServer* pServer) {
      Serial.println("Client disconnected - start advertising");
      gfx->println("Client disconnected");
      NimBLEDevice::startAdvertising();
  };
  void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
      Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
  };
};

// Fire Servo Interrupt
void IRAM_ATTR onTimer(){
  gfx->print("Servo End");
  Serial.println("End FIRE Timer");
  timerAlarmDisable(timer);		// stop alarm
  timerDetachInterrupt(timer);	// detach interrupt
  timerEnd(timer);			// end timer
  ESP32_ISR_Servos.setPosition(servoIndex1, servo_back);
  busy = false;
  CharacteristicINF->setValue(std::string("Ready"));
  //xSemaphoreTake(Semaphore, portMAX_DELAY);
  motor_speed = 0;
  //xSemaphoreGive(Semaphore);
  ESP32_ISR_Servos.setPosition(escIndex1, motor_speed);
  ESP32_ISR_Servos.setPosition(escIndex2, motor_speed);
}

/** Handler class for characteristic actions */
class CharacteristicCallbacks: public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* pCharacteristic){
      Serial.print(pCharacteristic->getUUID().toString().c_str());
      Serial.print(": onRead(), value: ");
      Serial.println(pCharacteristic->getValue().c_str());
  };

  void onWrite(NimBLECharacteristic* pCharacteristic) {
      Serial.print(pCharacteristic->getUUID().toString().c_str());
      Serial.print(": onWrite(), value: ");
      Serial.println(pCharacteristic->getValue().c_str());
      if (pCharacteristic->getUUID() == CharacteristicPOS->getUUID() ){
        String value = pCharacteristic->getValue().c_str();
        if (value.length() > 0) {
          pos_x = value.toInt();
          separator_1 = value.indexOf(";");
          pos_y = value.substring(separator_1+1).toInt();
          Serial.println(pos_x);
          Serial.println(pos_y);
          sc.RegWritePos(0, pos_x, 0, speed_x);
          sc.RegWritePos(1, pos_y, 0, speed_y);
          sc.RegWriteAction();
          pCharacteristic->setValue(std::string("Ok")); // Return status
        }
      }
      if (pCharacteristic->getUUID() == CharacteristicFIR->getUUID() ){
        String value = pCharacteristic->getValue().c_str();
        if (value.length() > 0) {
          if (!busy) {
            Serial.println(fire);
            xSemaphoreTake(Semaphore, portMAX_DELAY);
            fire = value.toInt();
            motor_speed = fire_speed;
            CharacteristicINF->setValue(std::string("Busy"));
            xSemaphoreGive(Semaphore);
          }
        }
      }
      if (pCharacteristic->getUUID() == CharacteristicINI->getUUID() ){
        String value = pCharacteristic->getValue().c_str();
        if (value.length() > 0) {
          if ((value.charAt(0)) == 'P') {
            speed_x = value.substring(1).toInt();
            separator_1 = value.indexOf(";");
            speed_y = value.substring(separator_1+1).toInt();
            Serial.println("Setup Position values");
            Serial.println(speed_x);
            Serial.println(speed_y);
            pCharacteristic->setValue(std::string("Ok")); // Return status
          }
          if ((value.charAt(0)) == 'F') {
            fire_mode = value.substring(1).toInt();
            separator_1 = value.indexOf(";");
            fire_speed = value.substring(separator_1+1).toInt();
            separator_2 = value.indexOf(";", separator_1+1);
            fire_burst = value.substring(separator_2+1).toInt();
            separator_3 = value.indexOf(";", separator_2+1);
            fire_auto_speed = value.substring(separator_3+1).toInt();
            Serial.println("Setup Fire values");
            Serial.println(fire_mode);
            Serial.println(fire_speed);
            Serial.println(fire_burst);
            Serial.println(fire_auto_speed);
            pCharacteristic->setValue(std::string("Ok")); // Return status
          }
          if ((value.charAt(0)) == 'S') {
            servo_back = value.substring(1).toInt();
            separator_1 = value.indexOf(";");
            servo_front = value.substring(separator_1+1).toInt();
            Serial.println("Setup Position values");
            Serial.println(servo_back);
            Serial.println(servo_front);
            pCharacteristic->setValue(std::string("Ok")); // Return status
            ESP32_ISR_Servos.setPosition(servoIndex1, servo_back);
          }
        }
      }
  };
} ;

static CharacteristicCallbacks chrCallbacks;

// Measure dart speed
// 70mm dart
void measure_speed()
{
  if(speed_first)
  {
    speed_in = millis();
    speed_first = false;
  }else{
    speed_out = millis();
    speed_first = true;
    gfx->println(speed_out-speed_in);
  }
}

void read_bat_loop(void * parameter)
{
  for (;;) {
    if (deviceConnected) {
      bat_level = random(0,100);
      //CharacteristicBAT->setValue(&bat_level, 1);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void write_servo_loop(void * parameter)
{
  int current_value = 0;
  int new_value = 0;
  int test_value = 0;
  int f = 0;
  int s_b = 0;
  int s_f = 0;
  for (;;) {
    xSemaphoreTake(Semaphore, portMAX_DELAY);
    current_value = motor_speed;
    f = fire;
    s_b = servo_back;
    s_f = servo_front;
    xSemaphoreGive(Semaphore);
    if(f){
      Serial.println("Start motors");
      ESP32_ISR_Servos.setPosition(escIndex1, current_value);
      ESP32_ISR_Servos.setPosition(escIndex2, current_value);
      vTaskDelay(300 / portTICK_PERIOD_MS);
      Serial.println("Push dart");
      ESP32_ISR_Servos.setPosition(servoIndex1, s_f);
      vTaskDelay(300 / portTICK_PERIOD_MS);
      ESP32_ISR_Servos.setPosition(servoIndex1, s_b);
      Serial.println("Stop motors");
      while (current_value > 0){
        current_value = current_value - 5;
        ESP32_ISR_Servos.setPosition(escIndex1, current_value);
        ESP32_ISR_Servos.setPosition(escIndex2, current_value);
        Serial.println(current_value);
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
      xSemaphoreTake(Semaphore, portMAX_DELAY);
      fire = 0;
      motor_speed = 0;
      CharacteristicINF->setValue(std::string("Ready"));
      xSemaphoreGive(Semaphore);
    }
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

void setup()
{
  Serial.begin(115200);

  Semaphore = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(read_bat_loop, "ReadBat", 1000, NULL, 0, &ReadBat, 1); 
  xTaskCreatePinnedToCore(write_servo_loop, "WriteServo", 1000, NULL, 0, &WriteServo, 1);

  // Setup LCD Screen
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  gfx->begin(80000000);
  gfx->fillScreen(BLACK);
  digitalWrite(TFT_BL, LOW);
  //Serial.begin(115200);

  // BlueTooth Server
  gfx->setCursor(0, 0);
  gfx->println(xPortGetCoreID());
  gfx->println("Starting BLE work!");

  /** sets device name */
  NimBLEDevice::init("Dart-Turret");

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Set Servive
  NimBLEService* pDeadService = pServer->createService(SERVICE_UUID);

  // Set Characteristic POS
  CharacteristicPOS = pDeadService->createCharacteristic(
                                              CHARACTERISTIC_POS_UUID,
                                              NIMBLE_PROPERTY::READ |
                                              NIMBLE_PROPERTY::WRITE
                                            );
  CharacteristicPOS->setValue("");
  CharacteristicPOS->setCallbacks(&chrCallbacks);

  // Set Characteristic FIR
  CharacteristicFIR = pDeadService->createCharacteristic(
                                              CHARACTERISTIC_FIR_UUID,
                                              NIMBLE_PROPERTY::READ |
                                              NIMBLE_PROPERTY::WRITE
                                            );
  CharacteristicFIR->setValue("");
  CharacteristicFIR->setCallbacks(&chrCallbacks);

  // Set Characteristic INF
  CharacteristicINF = pDeadService->createCharacteristic(
                                              CHARACTERISTIC_INF_UUID,
                                              NIMBLE_PROPERTY::READ |
                                              NIMBLE_PROPERTY::WRITE
                                            );
  CharacteristicINF->setValue("Ready");
  CharacteristicINF->setCallbacks(&chrCallbacks);

  // Set Characteristic INI
  CharacteristicINI = pDeadService->createCharacteristic(
                                              CHARACTERISTIC_INI_UUID,
                                              NIMBLE_PROPERTY::READ |
                                              NIMBLE_PROPERTY::WRITE
                                            );
  CharacteristicINI->setValue("");
  CharacteristicINI->setCallbacks(&chrCallbacks);

  // Set Characteristic BAT
  CharacteristicBAT = pDeadService->createCharacteristic(
                                              CHARACTERISTIC_BAT_UUID,
                                              NIMBLE_PROPERTY::READ |
                                              NIMBLE_PROPERTY::WRITE
                                            );
  CharacteristicBAT->setValue("");
  CharacteristicBAT->setCallbacks(&chrCallbacks);

  /** Start the services when finished creating all Characteristics and Descriptors */
  pDeadService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  /** Add the services to the advertisment data **/
  pAdvertising->addServiceUUID(pDeadService->getUUID());
  /** If your device is battery powered you may consider setting scan response
   *  to false as it will extend battery life at the expense of less data sent.
   */
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("Advertising Started");
  gfx->println("BLE Started");


  /* Setup : Loading servo, ESC motor 1 & ESC motor 2
     Loading servo
      - low position : 125
      - high position : 10
  */
  escIndex1 = ESP32_ISR_Servos.setupServo(PIN_ESC1, MIN_MICROS, MAX_MICROS);
  escIndex2 = ESP32_ISR_Servos.setupServo(PIN_ESC2, MIN_MICROS, MAX_MICROS);
  servoIndex1 = ESP32_ISR_Servos.setupServo(PIN_SERVO1, MIN_MICROS_S, MAX_MICROS_S);
  delay(500);
  ESP32_ISR_Servos.setPosition(escIndex1, 0);
  ESP32_ISR_Servos.setPosition(escIndex2, 0);
  ESP32_ISR_Servos.setPosition(servoIndex1, 125);

  // Setup : Speed module
  pinMode(PIN_INT,  INPUT_PULLUP);
  attachInterrupt(PIN_INT, measure_speed, CHANGE);

  // Setup : Serial servos
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sc.pSerial = &Serial1;
  //---- ESC
  //Select ESP32 timer USE_ESP32_TIMER_NO
  ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);
}

void loop()
{
  delay(1000);
}
