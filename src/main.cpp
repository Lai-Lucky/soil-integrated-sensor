// 攀登：土壤传感器 - OneNet 物联网接入
// 供电：12V
// 通信：RS485-TTL


#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h> // 添加 FreeRTOS 头文件
#include <freertos/task.h>     // 添加 FreeRTOS 任务管理头文件
#include <freertos/semphr.h>   // 添加 FreeRTOS 信号量头文件


/************** 函数声明 ***************/
uint16_t CRC16(const uint8_t *data, uint16_t length);
bool checkCRC(const uint8_t *data, uint16_t len);
void parseModbusData(const uint8_t *data, uint16_t len);
void sendSensorData(double data) ;
void ZigBeec_controller(int switchs);


void soilsensor_task_vtask(void *pv); //土壤传感器任务

TaskHandle_t soilsensor_task_handle;  //土壤传感器任务句柄


/************* 询问命令 *************/
const byte send_byte[6][8] = {
  {0x01,0x03,0x00,0x06,0x00,0x01,0x64,0x0B}, // PH
  {0x01,0x03,0x00,0x13,0x00,0x01,0x75,0xCF}, // 温度
  {0x01,0x03,0x00,0x12,0x00,0x01,0x24,0x0F}, // 湿度
  {0x01,0x03,0x00,0x1E,0x00,0x01,0xE4,0x0C}, // 氮
  {0x01,0x03,0x00,0x1F,0x00,0x01,0xB5,0xCC}, // 磷
  {0x01,0x03,0x00,0x20,0x00,0x01,0x85,0xC0}  // 钾
};


/************ 串口屏属性标识符 ************/
const char* lcd_names[] = {"x0"/*酸碱度*/, 
                           "x1"/*温度*/, 
                           "x2"/*湿度*/, 
                           "x3"/*氮*/, 
                           "x4"/*磷*/, 
                           "x5"/*钾*/};


/************ OneNet平台的属性标识符 ************/
const char* sensor_names[] = {"soil-PHM"/*酸碱度*/, 
                              "soil-TM"/*温度*/, 
                              "soil-HM"/*湿度*/, 
                              "soil-NM"/*氮*/, 
                              "soil-PM"/*磷*/, 
                              "soil-KM"/*钾*/};


/******** 变量 ********/
byte temp[7]; // 传感器返回数据
int asr = 0;  // 传感器轮询索引


/************* 程序初始化 *************/
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);

  xTaskCreate(soilsensor_task_vtask,"soilsensor_task_vtask",4096,NULL,1,&soilsensor_task_handle);//创建土壤传感器任务
  
}



/************* 主循环 *************/
void loop() {
}



/************* CRC 计算 *************/
uint16_t CRC16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) 
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) 
    {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}



/************* CRC 校验 *************/
bool checkCRC(const uint8_t *data, uint16_t len) {
  if (len < 3) return false;
  uint16_t computedCRC = CRC16(data, len - 2);
  uint16_t receivedCRC = data[len - 2] | (data[len - 1] << 8);
  return computedCRC == receivedCRC;
}



/************* 解析数据并上传 *************/
void parseModbusData(const uint8_t *data, uint16_t len) {

  if (data[1] == 0x03) 
  {
    uint16_t dataLength = data[2]; // 数据字节数

    if (dataLength == 2) 
    {  // 7字节
      uint16_t regValue = (data[3] << 8) | data[4];
      double value = regValue / 10.0;
     
      sendSensorData(value);
    } 
    else if (dataLength == 4) 
    { // 9字节
      uint16_t regValue_H = (data[3] << 8) | data[4];
      uint16_t regValue_L = (data[5] << 8) | data[6];
      uint32_t regValue = (regValue_H << 16) | regValue_L;
     
      sendSensorData((int32_t)regValue);
    } 
    
  }
}



/************JSON数据构建************/
void sendSensorData(double data) 
{
  ZigBeec_controller(1);
  vTaskDelay(pdMS_TO_TICKS(100));
  JsonDocument doc;
  doc["id"] = String(millis());  // 使用时间戳作为唯一ID
  doc["version"] = "1.0";
  doc["params"][sensor_names[asr]]["value"]= data;

  String payload;
  serializeJson(doc, payload);

  Serial.println(payload.c_str());

  ZigBeec_controller(0);
  vTaskDelay(pdMS_TO_TICKS(100));

  Serial.printf("\xff\xff\xff");
  data*=10.0;
  Serial.printf("%s.val=%d\xff\xff\xff",lcd_names[asr],(int)data);
  vTaskDelay(pdMS_TO_TICKS(100));
}



/************ZigBee模块控制************/
void ZigBeec_controller(int switchs){
  if(switchs==1)
  {
    digitalWrite(34,HIGH);//M1
    digitalWrite(35,HIGH);//M0
  }
  else
  {
    digitalWrite(34,LOW);
    digitalWrite(35,HIGH);
  }

}



/********************* 土壤传感器任务 *********************/
void soilsensor_task_vtask(void *pv){

  while(1)
  {
    // 发送请求
    Serial2.write(send_byte[asr], 8);

    vTaskDelay(pdMS_TO_TICKS(200));

    // 读取响应数据
    if (Serial2.available()) 
    {
      int len = Serial2.available();
      Serial2.readBytes(temp, len);
      parseModbusData(temp, len);

      if (checkCRC(temp, len)) 
      {
        asr = (asr + 1) % 6; // 轮询下一个传感器
      } 
      
    vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}

