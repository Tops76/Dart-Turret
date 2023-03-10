#include <Arduino_GFX_Library.h>
#include "ESP32_ISR_Servo.h"
#include "SCServo.h"

#include <NimBLEDevice.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
BLECharacteristic * pTxPosCharacteristic;
volatile bool deviceConnected = false;
volatile bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID              "DEAD" // UART service UUID
#define CHARACTERISTIC_UUID_RX    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TXPOS "6E400005-B5A3-F393-E0A9-E50E24DCCA9E"

TaskHandle_t read_bat = NULL;
TaskHandle_t pos_loop = NULL;

SemaphoreHandle_t Semaphore = NULL;

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
#define PIN_INT         16 // was 48
#define PIN_BAT         16 // ADC for BAT
#define PIN_ESC1        17
#define PIN_ESC2        18
#define PIN_SERVO1      39
#define TFT_BL          10


int escIndex1   = -1;
int escIndex2   = -1;
int servoIndex1 = -1;

volatile unsigned long dart_speed = 100;
volatile unsigned long speed_in = 0;
volatile unsigned long speed_out = 0;
volatile bool speed_first = true;
volatile int shout = 0;

volatile int pos_x = 0;
volatile int pos_y = 0;
volatile int speed_x = 0;
volatile int speed_y = 0;
volatile int separator_1, separator_2, separator_3, separator_4, separator_5;

volatile int fire = 0;
volatile int fire_mode = 0;
volatile int fire_speed = 0;
volatile int fire_burst = 0;
volatile int fire_auto_speed = 0;

volatile int servo_back = 120;
volatile int servo_front = 5;
String rxString;
String txString;
String rxPosString;
String txPosString;

// Motor Speed
volatile unsigned int motor_speed = 0;

/* create a hardware fire timer */
hw_timer_t * timer = NULL;

Arduino_DataBus *bus = new Arduino_ESP32SPI(6 /* DC */, 5 /* CS */, 3 /* SCK */, 2 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_GC9107(bus, 1 /* RST */, 1 /* rotation */, true /* IPS */);

SCSCL sc;

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */  
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      rxString = pCharacteristic->getValue();
      if (rxString.length() > 0) {
        switch (rxString[0]) {
          case 'P':
            txPosString = "P;Busy";
            break;
          case 'F':
            txString = "F;Busy";
            break;
          case 'B':
            txString = "B;Busy";
            break;
          default:
            txString = "X;Error";
            break;
        }
      }
    }
};

// Reset dart speed measure
// Fire Interrupt
void IRAM_ATTR onTimer(){
  Serial.println("End Timer");
  timerAlarmDisable(timer);		// stop alarm
  timerDetachInterrupt(timer);	// detach interrupt
  timerEnd(timer);			// end timer
  speed_first = true;
  shout = -1;
}

// Measure dart speed
// 70mm dart
void measure_speed()
{
  unsigned long speed;

  if(speed_first)
  {
    speed_in = micros();
    speed_first = false;
  }else{
    speed_out = micros();
    timerAlarmDisable(timer);		// stop alarm
    timerDetachInterrupt(timer);	// detach interrupt
    timerEnd(timer);			// end timer
    speed_first = true;
    dart_speed = int(70.0 / (speed_out - speed_in) * 3600.0);
    shout = 1;
  }

}


void readBatLoop(void * parameter)
{
  for (;;) {
    if (deviceConnected) {
      txString = "B;50";
      //bat_level = random(0,100);
      //CharacteristicBAT->setValue(&bat_level, 1);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void moveTurret(const String& dataString)
{
  Serial.println(dataString);
  pos_x = dataString.toInt();
  separator_1 = dataString.indexOf(";");
  pos_y = dataString.substring(separator_1+1).toInt();
  separator_2 = dataString.indexOf(";", separator_1+1);
  speed_x = dataString.substring(separator_2+1).toInt();
  separator_3 = dataString.indexOf(";", separator_2+1);
  speed_y = dataString.substring(separator_3+1).toInt();
  Serial.println("Position values");
  Serial.println(pos_x);
  Serial.println(pos_y);
  Serial.println(speed_x);
  Serial.println(speed_y);
  sc.RegWritePos(0, pos_x, 0, speed_x);
  sc.RegWritePos(1, pos_y, 0, speed_y);
  sc.RegWriteAction();
  txPosString = "P;Ready";
}

void fireDart(const String& dataString)
{
  fire = dataString.toInt();
  separator_1 = dataString.indexOf(";");
  fire_mode = dataString.substring(separator_1+1).toInt();
  separator_2 = dataString.indexOf(";", separator_1+1);
  fire_speed = dataString.substring(separator_2+1).toInt();
  separator_3 = dataString.indexOf(";", separator_2+1);
  fire_burst = dataString.substring(separator_3+1).toInt();
  separator_4 = dataString.indexOf(";", separator_3+1);
  fire_auto_speed = dataString.substring(separator_4+1).toInt();
  Serial.println("Fire values :");
  Serial.println(fire);
  Serial.println(fire_mode);
  Serial.println(fire_speed);
  Serial.println(fire_burst);
  Serial.println(fire_auto_speed);

  motor_speed = fire_speed;

  ESP32_ISR_Servos.setPosition(escIndex1, motor_speed);
  ESP32_ISR_Servos.setPosition(escIndex2, motor_speed);
  vTaskDelay(300 / portTICK_PERIOD_MS);
  ESP32_ISR_Servos.enable(servoIndex1);
  Serial.println(servo_front);
  ESP32_ISR_Servos.setPosition(servoIndex1, servo_front);

  // Set timer fot shouting error
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 500000, true);
  timerAlarmEnable(timer);

  while (shout == 0){
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  Serial.println(motor_speed);
  vTaskDelay(300 / portTICK_PERIOD_MS);
  Serial.println(servo_back);
  ESP32_ISR_Servos.setPosition(servoIndex1, servo_back);

  // Slow down the motors
  while (motor_speed > 0){
    motor_speed = motor_speed - 5;
    ESP32_ISR_Servos.setPosition(escIndex1, motor_speed);
    ESP32_ISR_Servos.setPosition(escIndex2, motor_speed);
    Serial.println(motor_speed);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  gfx->setCursor(0, 0);
  gfx->fillScreen(BLACK);
  gfx->println(speed_in);
  gfx->println(speed_out);
  gfx->println(dart_speed);
  if (shout == -1){
    txString = "F;Ready;ERR";
  }else{
    txString = "F;Ready;";
    txString += dart_speed;
  }

  motor_speed = 0;
  ESP32_ISR_Servos.disable(servoIndex1);
}

void setup()
{
  txString.reserve(64);
  rxString.reserve(64);
  txPosString.reserve(64);
  rxPosString.reserve(64);

  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  Semaphore = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(readBatLoop, "ReadBat", 1000, NULL, 0, &read_bat, 1); 
  //xTaskCreatePinnedToCore(posLoop, "PosLoop", 1000, NULL, 0, &pos_loop, 1);

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
  gfx->println("Starting BlueTooth work!");
  // Create the BLE Device
  BLEDevice::init("DART Turret");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic( CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY);
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, NIMBLE_PROPERTY::WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pTxPosCharacteristic = pService->createCharacteristic( CHARACTERISTIC_UUID_TXPOS, NIMBLE_PROPERTY::NOTIFY);

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  gfx->println("BLE Started");

  /* Setup : Loading servo, ESC motor 1 & ESC motor 2
     Loading servo
      - low position : 125
      - high position : 10
  */

  escIndex1 = ESP32_ISR_Servos.setupServo(PIN_ESC1, MIN_MICROS, MAX_MICROS);
  escIndex2 = ESP32_ISR_Servos.setupServo(PIN_ESC2, MIN_MICROS, MAX_MICROS);
  servoIndex1 = ESP32_ISR_Servos.setupServo(PIN_SERVO1, MIN_MICROS_S, MAX_MICROS_S);
  ESP32_ISR_Servos.setPosition(escIndex1, 0);
  ESP32_ISR_Servos.setPosition(escIndex2, 0);
  ESP32_ISR_Servos.setPosition(servoIndex1, 125);
  delay(500);
  ESP32_ISR_Servos.disable(servoIndex1);
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
    if (deviceConnected && txString.length() != 0) {
      pTxCharacteristic->setValue(String(txString.c_str()));
      pTxCharacteristic->notify();
      txString = "";
      delay(10); // bluetooth stack will go into congestion, if too many packets are sent
    }
    if (deviceConnected && txPosString.length() != 0) {
      pTxPosCharacteristic->setValue(String(txPosString.c_str()));
      pTxPosCharacteristic->notify();
      txPosString = "";
      delay(10); // bluetooth stack will go into congestion, if too many packets are sent
    }
    if (rxString.length() != 0){
      switch (rxString[0]) {
        case 'P':
          moveTurret(rxString.substring(2));
          break;
        case 'F':
          gfx->println("Fire Started");
          fireDart(rxString.substring(2));
          gfx->println("Fire Ended");
          break;
        case 'B':
          Serial.println("Batery\n");
          break;
        case 'S':
          Serial.println("Setup\n");
          break;
        default:
          Serial.println("Error\n");
          break;
      }
      rxString = "";
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
    }
    delay(10);
}
