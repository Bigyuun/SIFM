
/******************************************************************************************
 * Project : gripper_ws :: main
 *
 * @file main.ino
 * @author Dae-Yun Jang (bigyun9375@gmail.com)
 * @git  https://github.com/Bigyuun/SIFM
 * @version 0.0
 * @date 2024-04-09
 * @copyright Copyright (c) 2024
 * @brief
 * Read Loadcell(HX711) and Force-torque Sensor(CAN comm)
 * Write Serial print
 * @note
 * MCU          : Arduin DUE
 ******************************************************************************************/

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include <SPI.h>
#include <mcp2515.h> // mcp2515 라이브러리를 설치해야 합니다. (https://github.com/autowp/arduino-mcp2515/)
#include <HX711.h>

#define FORCE_CAN_ID 0x1A
#define TORQUE_CAN_ID 0x1B
// #define LOADCELL_1_/ID 0x07F

// Load Cell pin
#define NUMBER_OF_LOADCELL_MODULE        2

#define HX711_DOUT_1                     4  // mcu > HX711 no 1 dout pin
#define HX711_SCK_1                      5  // mcu > HX711 no 1 sck pin
#define CALIBRATION_VALUE_1              2000

#define HX711_DOUT_2                     6  // mcu > HX711 no 2 dout pin
#define HX711_SCK_2                      7  // mcu > HX711 no 2 sck pin
#define CALIBRATION_VALUE_2              3000

// CAN comm pin
#define MCP2515_SPI_CS                   10

// Serial communication Parameters
#define BAUDRATE                         115200
#define BANDWIDTH                        10

// ROTS Threads Frequency Setting
#define RTOS_FREQUENCY                   100   // Hz (Default)
#define RTOS_FREQUENCY_LOADCELLUPDATE    100   // Hz (Default)
#define RTOS_FREQUENCY_FORCETORQUESENSOR 200   // Hz (Default)
#define RTOS_FREQUENCY_EEPROM_SAVE       250   // H-z (Default)
#define RTOS_FREQUENCY_MONITORING        25    // Hz (Default)
#define RTOS_FREQUENCY_SERIALREADING     500   // Hz (Default)
#define RTOS_FREQUENCY_SERIALWRITING     200   // Hz (Default)
#define TOTAL_SYSTEM_DELAY               1     // ms

#define PIN_BOARD_LED                    LED_BUILTIN    // pin 13 for LED in board


// MCP2515 mcp2515(MCP2515_SPI_CS);                 // SPI CS Pin 10
// SemaphoreHandle_t mutex;
MCP2515 mcp2515(MCP2515_SPI_CS);  // SPI CS Pin 10
struct can_frame kcan_read_msg_;
struct can_frame kcan_send_msg_;
QueueHandle_t integerQueue;

/******************************************************************************************
 * Class & Enum & Structure
 ******************************************************************************************/
//===========================================================
// timer structure
//===========================================================
struct LoopTimeChecker
{
  float loop_time_checker_LoadCellUpdate = -1;
  float loop_time_checker_ForceTorqueUpdate = -1;
  float loop_time_checker_MotorOperation = -1;
  float loop_time_checker_CANReead       = -1;
  float loop_time_checker_EEPROMUpdate   = -1;
  float loop_time_checker_SerialWriting  = -1;
  float loop_time_checker_SerialReading  = -1;
};


//===========================================================
// Force torque sensor
//===========================================================
// class ForceTorqueSensor
// {
//   public:
//     MCP2515 mcp2515(10);  // SPI CS Pin 10
//     struct can_frame kcan_read_msg_;
//     struct can_frame kcan_send_msg_;
//     LoopTimeChecker TimeChecker;

//   public:
//     float fx_ = 0, fy_ = 0, fz_ = 0; 
//     float tx_ = 0, ty_ = 0, tz_ = 0;
//     long can_id_force_ = 0x00;
//     long can_id_torque_ = 0x00;

//     ForceTorqueSensor();
//     ForceTorqueSensor(long force_can_id, long torque_can_id);
//     ~ForceTorqueSensor();
//     uint8_t initialize();
//     uint8_t read();
//     uint8_t parsing(can_frame can_msg);
//     uint8_t update(float fx, float fy, float fz, float tx, float ty, float tz);
//     uint8_t read_data(float * data);
//     uint8_t spin(void *pvParameters);
// };
// ForceTorqueSensor::ForceTorqueSensor()
// {
//   this->can_id_force_ = FORCE_CAN_ID;
//   this->can_id_torque_ = TORQUE_CAN_ID;
//   initialize();
// }
// ForceTorqueSensor::ForceTorqueSensor(long force_can_id, long torque_can_id)
// {
//   this->can_id_force_ = force_can_id;
//   this->can_id_torque_ = torque_can_id;
//   initialize();
// }
// ForceTorqueSensor::~ForceTorqueSensor()
// {

// }
// uint8_t ForceTorqueSensor::initialize()
// { /*MCP2515 초기화*/
//   // SPI.begin();
//   mcp2515.reset();                              // MCP2515 비트레이트 설정 (여기서는 1Mbps로 설정)
//   mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);   // 정상 모드로 설정
//   mcp2515.setNormalMode();
//   // Serial.println("------- CAN Read ----------");
//   // Serial.println("ID  DLC   DATA");
//   return 1;
// }
// uint8_t ForceTorqueSensor::parsing(can_frame can_msg){
//   if (can_msg.can_id == this->can_id_force_){  // 26
//     // mN 으로 force 계산
//     this->fx_ = (can_msg.data[0]*256 + can_msg.data[1])-30000;
//     this->fy_ = (can_msg.data[2]*256 + can_msg.data[3])-30000;
//     this->fz_ = (can_msg.data[4]*256 + can_msg.data[5])-30000;
//     // Serial.println("FORCE");
//   }
//   else if (can_msg.can_id == this->can_id_torque_){ //27
//     // mNm 으로 torque 계산
//     this->tx_ = (can_msg.data[0]*256 + can_msg.data[1])-30000;
//     this->ty_ = (can_msg.data[2]*256 + can_msg.data[3])-30000;
//     this->tz_ = (can_msg.data[4]*256 + can_msg.data[5])-30000;
//     // Serial.println("TORQUE");
//   }

//   return 1;
// }
// uint8_t ForceTorqueSensor::update(float fx, float fy, float fz, float tx, float ty, float tz){
//   this->fx_ = fx;
//   this->fy_ = fy;
//   this->fz_ = fz;
//   this->tx_ = tx;
//   this->ty_ = ty;
//   this->tz_ = tz;
//   return 1;
// }
// uint8_t ForceTorqueSensor::read_data(float * data){
//   int data_arr_length = sizeof(data)/sizeof(data[0]);
//   if (data_arr_length > 6) {
//     return -1;
//   }
//   data[0] = this->fx_;
//   data[1] = this->fy_;
//   data[2] = this->fz_;
//   data[3] = this->tx_;
//   data[4] = this->ty_;
//   data[5] = this->tz_;
//   return 1;
// }
// uint8_t ForceTorqueSensor::spin(void *pvParameters)
// {
//   static unsigned long curt_time = millis();
//   static unsigned long prev_time = millis();
//   static unsigned long temp_time = 0;

//   TickType_t delayTime = *((TickType_t*)pvParameters); // Use task parameters to define delay
//   while(true)
//   {
//     curt_time = millis();
//     temp_time = curt_time - prev_time;
//     prev_time = curt_time;

//     if (mcp2515.readMessage(&this->kcan_read_msg_) == MCP2515::ERROR_OK) {
//       // CAN 메시지 출력
//       Serial.print(kcan_read_msg_.can_id, HEX); // ID 출력
//       Serial.print(" ");
//       Serial.print(kcan_read_msg_.can_dlc, HEX); // DLC 출력
//       Serial.print(" ");
//       for (int i = 0; i < kcan_read_msg_.can_dlc; i++) {
//         Serial.print(kcan_read_msg_.data[i], HEX);
//       }
//       Serial.println("");

//       parsing(this->kcan_read_msg_);
//     }

//     // TimeChecker.loop_time_checker_ForceTorqueUpdate = temp_time;
//     // vTaskDelay(((1000 / RTOS_FREQUENCY_FORCETORQUESENSOR) - TOTAL_SYSTEM_DELAY) / portTICK_PERIOD_MS);
//     vTaskDelay(delayTime / portTICK_PERIOD_MS);
//   }
//   return 1;
// }

// //===========================================================
// // Loadcell
// //===========================================================
// class Loadcell : public HX711
// {
//   public:
//     HX711 lc1, lc2;
    
//     float data_[NUMBER_OF_LOADCELL_MODULE];
//     LoopTimeChecker TimeChecker;

//   public:
//     Loadcell();
//     ~Loadcell();
//     uint8_t initialize();
//     uint8_t update();
//     void getdata();
//     uint8_t spin(void *pvParameters);
// };
// Loadcell::Loadcell()
// {
//   this->initialize();
// }
// Loadcell::~Loadcell()
// {

// }
// uint8_t Loadcell::initialize(){

//   this->lc1.begin(DOUT1, SCK1);
//   this->lc1.set_scale(CALIBRATION_VALUE_1);
//   this->lc1.tare();
//   this->lc2.begin(DOUT2, SCK2);
//   this->lc2.set_scale(CALIBRATION_VALUE_2);
//   this->lc2.tare();

//   return 1;
// }
// void Loadcell::getdata(){
//   this->data_[0] = lc1.read();
//   this->data_[1] = lc2.read();
// }
// uint8_t Loadcell::spin(void *pvParameters)
// {
//   TickType_t delayTime = *((TickType_t*)pvParameters); // Use task parameters to define delay
//   static unsigned long curt_time = millis();
//   static unsigned long prev_time = millis();
//   static unsigned long temp_time = 0;

//   while (true)
//   {
//     curt_time = millis();
//     temp_time = curt_time - prev_time;
//     prev_time = curt_time;

//     getdata();
//     // this->loadcell_1.update();
//     // this->loadcell_2.update();

//     // TimeChecker.loop_time_checker_LoadCellUpdate = temp_time;
//     // vTaskDelay(((1000 / RTOS_FREQUENCY_LOADCELLUPDATE) - TOTAL_SYSTEM_DELAY) / portTICK_PERIOD_MS);
//     vTaskDelay(delayTime / portTICK_PERIOD_MS);
//   }
//   return 1;
// }


/******************************************************************************************
 * Global variables
 ******************************************************************************************/
LoopTimeChecker TimeChecker;
static volatile long fx_ = 0, fy_ = 0, fz_ = 0; 
static volatile long tx_ = 0, ty_ = 0, tz_ = 0;
static volatile long lc_data_[NUMBER_OF_LOADCELL_MODULE] = {0};

// ForceTorqueSensor FTS;
// Loadcell LC;

/******************************************************************************************
 * Functions
 ******************************************************************************************/
uint8_t init_objects();
void PrintOnMutex(String msg);

uint8_t RTOSInit();
void ForceTorqueNode();
void LoadCellNode();
void SerialWriteNode();
void SerialReadNode();

//===========================================================
// Node Manager (RTOS)
//===========================================================

uint8_t RTOSInit(){
  Serial.println("RTOS initializing ...");
  xTaskCreate(LoadCellNode,
              "LoadCellUpdate",
              256,
              NULL,
              1,
              NULL);
  xTaskCreate(ForceTorqueNode,
              "MonitorAllParameters",
              256,
              NULL,
              2,
              NULL);
  xTaskCreate(SerialWriteNode,
              "SerialWriteNode",
              256,
              NULL,
              3,
              NULL);
  // xTaskCreate(SerialReadNode,
  //             "SerialReadingNode",
  //             256,
  //             NULL,
  //             tskIDLE_PRIORITY + 1,
  //             NULL);

  // Serial.println("Done");
  // if vTaskStartScheduler doen't work, under line will be operated.
  // vTaskStartScheduler();
  // Serial.println("vTaskStartScheduler() failed!!!");

  return 0;
}



void SerialWriteNode(void *pvParameters)
{
  static unsigned long curt_time = millis();
  static unsigned long prev_time = millis();
  static unsigned long temp_time = 0;
  static unsigned long count_ = 0;

  const char C_STX = '/';
  const char C_ETX = ';';
  String str_send_buffer = "";
  static long count = 0;

  while(true)
  {
    curt_time = millis();
    temp_time = curt_time - prev_time;
    prev_time = curt_time;
    str_send_buffer = "";
    str_send_buffer += String(C_STX);
    str_send_buffer += String(fx_) + ",";
    str_send_buffer += String(fy_) + ",";
    str_send_buffer += String(fz_) + ",";
    str_send_buffer += String(tx_) + ",";
    str_send_buffer += String(ty_) + ",";
    str_send_buffer += String(tz_) + ",";
    str_send_buffer += String(lc_data_[0]) + ",";
    str_send_buffer += String(lc_data_[1]);
    str_send_buffer += String(C_ETX);
    str_send_buffer += String(count++) + "/";
    str_send_buffer += String(TimeChecker.loop_time_checker_SerialWriting);
    Serial.println(str_send_buffer);
    TimeChecker.loop_time_checker_SerialWriting = temp_time;
    // vTaskDelay(((1000 / RTOS_FREQUENCY_SERIALWRITING) - TOTAL_SYSTEM_DELAY) / portTICK_PERIOD_MS);
  }
}

void SensorsNode(void *pvParameters){

  HX711 lc1, lc2;
  lc1.begin(HX711_DOUT_1, HX711_SCK_1);
  lc1.set_scale(CALIBRATION_VALUE_1);
  lc1.tare();
  lc2.begin(HX711_DOUT_2, HX711_SCK_2);
  lc2.set_scale(CALIBRATION_VALUE_2);
  lc2.tare();
  // float fx_ = 0, fy_ = 0, fz_ = 0; 
  // float tx_ = 0, ty_ = 0, tz_ = 0;
  canid_t can_id_force_ = FORCE_CAN_ID;
  canid_t can_id_torque_ = TORQUE_CAN_ID;

  while(true)
  {
    if (mcp2515.readMessage(&kcan_read_msg_) == MCP2515::ERROR_OK) {
      // CAN 메시지 출력
      Serial.print(kcan_read_msg_.can_id, HEX); // ID 출력
      Serial.print(" ");
      Serial.print(kcan_read_msg_.can_dlc, HEX); // DLC 출력
      Serial.print(" ");
      for (int i = 0; i < kcan_read_msg_.can_dlc; i++) {
        Serial.print(kcan_read_msg_.data[i], HEX);
      }
      Serial.println("");

      if (kcan_read_msg_.can_id == can_id_force_){  // 26
        // mN 으로 force 계산
        fx_ = (kcan_read_msg_.data[0]*256 + kcan_read_msg_.data[1])-30000;
        fy_ = (kcan_read_msg_.data[2]*256 + kcan_read_msg_.data[3])-30000;
        fz_ = (kcan_read_msg_.data[4]*256 + kcan_read_msg_.data[5])-30000;
        // Serial.println("FORCE");
      }
      else if (kcan_read_msg_.can_id == can_id_torque_){ //27
        // mNm 으로 torque 계산
        tx_ = (kcan_read_msg_.data[0]*256 + kcan_read_msg_.data[1])-30000;
        ty_ = (kcan_read_msg_.data[2]*256 + kcan_read_msg_.data[3])-30000;
        tz_ = (kcan_read_msg_.data[4]*256 + kcan_read_msg_.data[5])-30000;
        // Serial.println("TORQUE");
      }
    }

    lc_data_[0] = lc1.read();
    lc_data_[1] = lc2.read();

  }
  // FTS.spin(pvParameters);
}


// void SerialWriteNode(void *pvParameters)
// {
//   static unsigned long curt_time = millis();
//   static unsigned long prev_time = millis();
//   static unsigned long temp_time = 0;
//   static unsigned long count_ = 0;
//   const char C_STX = '/';
//   const char C_ETX = ';';


//   TickType_t delayTime = *((TickType_t*)pvParameters); // Use task parameters to define delay
//   static long count = 0;
//   while (true)
//   {
//     curt_time = millis();
//     temp_time = curt_time - prev_time;
//     prev_time = curt_time;

//     str_send_buffer = "";
//     str_send_buffer += String(C_STX);
//     str_send_buffer += String(fx_) + ",";
//     str_send_buffer += String(fy_) + ",";
//     str_send_buffer += String(fz_) + ",";
//     str_send_buffer += String(tx_) + ",";
//     str_send_buffer += String(ty_) + ",";
//     str_send_buffer += String(tz_) + ",";
//     str_send_buffer += String(lc_data_[0]) + ",";
//     str_send_buffer += String(lc_data_[1]);
//     str_send_buffer += String(C_ETX);
//     str_send_buffer += String(count++);

//     Serial.println(str_send_buffer);

    // if (xSemaphoreTake(xMutex, (portTickType)10) == true)
    // {
    //   Serial.println(str_send_buffer);
    //   xSemaphoreGive(xMutex);
    // }
    // TimeChecker.loop_time_checker_SerialWr/iting = temp_time;
    // vTaskDelay(((1000 / RTOS_FREQUENCY_SERIALWRITING) - TOTAL_SYSTEM_DELAY) / portTICK_PERIOD_MS);
    // vTaskDelay(delayTime / portTICK_PERIOD_MS);
//   }
// }


// void PrintOnMutex(String msg)
// {
//   if(xSemaphoreTake( mutex, ( TickType_t ) 5 ) == pdTRUE )
//   {
//     Serial.println(msg);
//     xSemaphoreGive(mutex);
//   }
// }





void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SPI.begin();

  integerQueue = xQueueCreate(10, // Queue length
                              sizeof(int) // Queue item size
                              );

  if (integerQueue != NULL) {
    xTaskCreate(SerialWriteNode,
                "SerialWriteNode",
                256,
                NULL,
                2,
                NULL);
    xTaskCreate(SensorsNode,
                "SensorsNode",
                256,
                NULL,
                2,
                NULL);
  }
}

void loop() {

}





























