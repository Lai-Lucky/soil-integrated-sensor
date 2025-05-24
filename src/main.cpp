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
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void sendSensorData(double data) ;

void wifi_vtask(void *pv);            //wifi连接任务
void mqtt_vtask(void *pv);            //mqtt连接任务
void soilsensor_task_vtask(void *pv); //土壤传感器任务
void lcdwifi_vtask(void *pv);         //接收屏幕wifi更改任务

TaskHandle_t wifi_task_handle;        //WiFi任务句柄
TaskHandle_t mqtt_task_handle;        //MQTT任务句柄
TaskHandle_t soilsensor_task_handle;  //土壤传感器任务句柄
TaskHandle_t lcdwifi_task_handle;     //接收屏幕wifi更改任务句柄

SemaphoreHandle_t mqtt_mutex;         // MQTT 互斥锁


WiFiClient espClient;
PubSubClient client(espClient);

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
const char* sensor_names[] = {"soil-PH"/*酸碱度*/, 
                              "soil-T"/*温度*/, 
                              "soil-H"/*湿度*/, 
                              "soil-N"/*氮*/, 
                              "soil-P"/*磷*/, 
                              "soil-K"/*钾*/};

/******** 变量 ********/
byte temp[7]; // 传感器返回数据
int asr = 0;  // 传感器轮询索引

/************* 程序初始化 *************/
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  mqtt_mutex = xSemaphoreCreateMutex(); //创建互斥锁

  xTaskCreate(wifi_vtask,"wifi_vtask",4096,NULL,1,&wifi_task_handle);//创建wifi连接任务
  xTaskCreate(mqtt_vtask,"mqtt_vtask",4096,NULL,1,&mqtt_task_handle);//创建mqtt连接任务
  xTaskCreate(soilsensor_task_vtask,"soilsensor_task_vtask",4096,NULL,1,&soilsensor_task_handle);//创建土壤传感器任务
  xTaskCreate(lcdwifi_vtask,"lcdwifi_vtask",4096,NULL,1,&lcdwifi_task_handle);//创建接收屏幕wifi更改任务
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
      Serial.printf("analyse data: %f\n", value);//Serial.printf("解析数据: %f\n", value);
      sendSensorData(value);
    } 
    else if (dataLength == 4) 
    { // 9字节
      uint16_t regValue_H = (data[3] << 8) | data[4];
      uint16_t regValue_L = (data[5] << 8) | data[6];
      uint32_t regValue = (regValue_H << 16) | regValue_L;
      Serial.printf("analyse data: %d\n", regValue);//Serial.printf("解析数据: %d\n", regValue);
      sendSensorData((int32_t)regValue);
    } 
    else {
      Serial.println("Abnormal data length");//Serial.println("数据长度异常");
    }
  }
}


/************* WiFi 连接 *************/
void setup_wifi() {
  Serial.println("Connect to WiFi...");//Serial.println("连接 WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.print(".");
  }

  Serial.println("\nThe WiFi connection was successful !");//Serial.println("\nWiFi 连接成功!");
  Serial.print("IP : ");
  Serial.println(WiFi.localIP());

  if(WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi 连接成功!");
    Serial.print("IP 地址: ");
    Serial.println(WiFi.localIP());
  
    Serial.printf("\xff\xff\xff");
    Serial.printf("b1.bco=GREEN\xff\xff\xff");
    Serial.printf("b1.bco2=GREEN\xff\xff\xff");
  }
  else
  {
    Serial.printf("\xff\xff\xff");
    Serial.printf("b1.bco=64528\xff\xff\xff");
    Serial.printf("b1.bco2=64528\xff\xff\xff");
  }
}

/************* MQTT 订阅回调函数 *************/
void callback(char* topic, byte* payload, unsigned int length) {
   Serial.print("收到 MQTT 消息，主题: \n");
  Serial.println(topic);
  Serial.print("内容: ");
  for (int i = 0; i < length; i++) 
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.println();
}

/************* 连接 MQTT 服务器 *************/
void reconnect() {
  while (!client.connected()) 
  {
   Serial.printf("\xff\xff\xff");
    Serial.printf("b2.bco=64528\xff\xff\xff");
    Serial.printf("b2.bco2=64528\xff\xff\xff");

  
    if (client.connect(device_id, product_id, api_key)) 
    {

      Serial.printf("\xff\xff\xff");
      Serial.printf("b2.bco=GREEN\xff\xff\xff");
      Serial.printf("b2.bco2=GREEN\xff\xff\xff");
      Serial.printf("errormag.txt=\" \"\xff\xff\xff");
      Serial.printf("errornum.txt=\" \"\xff\xff\xff");

      client.subscribe(replyTopic); // 订阅属性下发
    } 
    else 
    {
      Serial.printf("\xff\xff\xff");
      Serial.printf("b2.bco=64528\xff\xff\xff");
      Serial.printf("b2.bco2=64528\xff\xff\xff");

      // Serial.printf("连接失败, 状态码=%d, 重试...\n", client.state());

      Serial.printf("\xff\xff\xff");
      Serial.printf("errornum.txt=\"E%s\"\xff\xff\xff",(String)client.state());

      Serial.printf("\xff\xff\xff");
      switch (client.state()) 
      {
        case -4: Serial.printf("errormag.txt=\"连接超时\"\xff\xff\xff"); break;
        case -3: Serial.printf("errormag.txt=\"连接丢失\"\xff\xff\xff"); break;
        case -2: Serial.printf("errormag.txt=\"连接失败\"\xff\xff\xff"); break;
        case -1: Serial.printf("errormag.txt=\"断开连接\"\xff\xff\xff"); break;
        case 1: Serial.printf("errormag.txt=\"协议错误\"\xff\xff\xff"); break;
        case 2: Serial.printf("errormag.txt=\"客户端标识无效\"\xff\xff\xff"); break;
        case 3: Serial.printf("errormag.txt=\"服务器不可用\"\xff\xff\xff"); break;
        case 4: Serial.printf("errormag.txt=\"用户名或密码错误\"\xff\xff\xff"); break;
        case 5: Serial.printf("errormag.txt=\"未授权\"\xff\xff\xff"); break;
        default: Serial.printf("errormag.txt=\"未知错误\"\xff\xff\xff");
      }
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

/************JSON数据构建************/
void sendSensorData(double data) 
{
 JsonDocument doc;
  doc["id"] = String(millis());  // 使用时间戳作为唯一ID
  doc["version"] = "1.0";
  doc["params"][sensor_names[asr]]["value"]= data;

  String payload;
  serializeJson(doc, payload);
  if (client.publish(pubTopic, payload.c_str())) 
  {
    Serial.println("数据已发送: " + payload);
    Serial.println();
  } 
  else 
  {
    Serial.println("发送失败");
  }

  Serial.printf("\xff\xff\xff");
  data*=10.0;
  Serial.printf("%s.val=%d\xff\xff\xff",lcd_names[asr],(int)data);


  vTaskDelay(pdMS_TO_TICKS(1000));
}

/********************* wifi连接任务 *********************/
void wifi_vtask(void *pv){

  setup_wifi();//WiFi连接
  while(1)
  {
    if(WiFi.status() != WL_CONNECTED)//检查WiFi连接状态
    { 
      Serial.printf("\xff\xff\xff");
      Serial.printf("b1.bco=64528\xff\xff\xff");
      Serial.printf("b1.bco2=64528\xff\xff\xff");
      Serial.println("The Wi-Fi connection has been lost !");
      WiFi.begin();//重连

      int timeout = 0;
      while(WiFi.status() != WL_CONNECTED && timeout < 20)// 等待重新连接，最多等20秒
      {
        vTaskDelay(pdMS_TO_TICKS(1000));  //延时1秒 pdMS_TO_TICKS将1000毫秒转换成对应的时钟节拍
        Serial.print(".");
        timeout++;
      }

      if(WiFi.status() == WL_CONNECTED)// 检查是否重新连接成功
      {
        Serial.printf("\xff\xff\xff");
        Serial.printf("b1.bco=GREEN\xff\xff\xff");
        Serial.printf("b1.bco2=GREEN\xff\xff\xff");
        Serial.println("WIFI reconnection successful !");
        Serial.print("IP : ");
        Serial.println(WiFi.localIP());
      } 
      else 
      {
        Serial.printf("\xff\xff\xff");
        Serial.printf("b1.bco=64528\xff\xff\xff");
        Serial.printf("b1.bco2=64528\xff\xff\xff");
        Serial.println("WIFI reconnection failed !");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); //每个循环有延时最优，5秒检查一次
  }
}

/********************* mqtt连接任务 *********************/
void mqtt_vtask(void *pv){

  client.setServer(mqtt_server, mqtt_port);
  client.connect(device_id, product_id, api_key);
  client.setCallback(callback);
  while(1)
  { 
    reconnect();//断网则重连

   if(xSemaphoreTake(mqtt_mutex,pdMS_TO_TICKS(100)) == pdTRUE)//检查是否上锁,防止多个任务同时操作client
   {
    client.loop();
    xSemaphoreGive(mqtt_mutex);  // 释放互斥锁
   }

   vTaskDelay(pdMS_TO_TICKS(1000)); 
  }
}

/********************* 土壤传感器任务 *********************/
void soilsensor_task_vtask(void *pv){

  while(1)
  {
    // 发送请求
    Serial2.write(send_byte[asr], 8);
    Serial.printf("Send the query: %s\n", sensor_names[asr]);//Serial.printf("发送查询: %s\n", sensor_names[asr]);
    Serial.println();

    vTaskDelay(pdMS_TO_TICKS(200));

    // 读取响应数据
    if (Serial2.available()) 
    {
      int len = Serial2.available();
      Serial2.readBytes(temp, len);
      Serial.println("Receive sensor data");//Serial.println("接收传感器数据");
      
      if(xSemaphoreTake(mqtt_mutex,pdMS_TO_TICKS(100)) == pdTRUE)//检查是否上锁
      {
        parseModbusData(temp, len);
        xSemaphoreGive(mqtt_mutex);  // 释放互斥锁
      }

      if (checkCRC(temp, len)) 
      {
        Serial.println("CRC verification was successful.\n");//Serial.println("CRC 校验成功\n");
        asr = (asr + 1) % 6; // 轮询下一个传感器
      } 
      else 
      {
        Serial.println("CRC verification was failed \n");//Serial.println("CRC 校验失败\n");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/********************* WIFI更改任务 *********************/
void lcdwifi_vtask(void *pv){

  byte lcd_data[128];
  int wifi_changed_flage=0;
  while(1)
  {
    if (Serial.available()) 
    {
      int len = Serial.readBytes(lcd_data, 128);

      for (int i = 0; i < len ; i++) 
      {
        if (lcd_data[i] == 0x55 && (lcd_data[i + 1] == 0x01 || lcd_data[i + 1] == 0x02)) 
        {
          uint8_t type = lcd_data[i + 1];
          int data_start = i + 2;

          // 查找包尾 0x0D 0x0A
          int data_end = -1;
          for (int j = data_start; j < len - 1; j++) 
          {
            if (lcd_data[j] == 0x0D && lcd_data[j + 1] == 0x0A) 
            {
              data_end = j;
              break;
            }
          }

          if (data_end != -1) {
            char temp[128] = {0};
            int k = 0;
            for (int j = data_start; j < data_end && k < 127; j++) 
            {
              temp[k++] = (char)lcd_data[j];
            }
            temp[k] = '\0'; 

            if (type == 0x01) 
            {
              ssid = String(temp);
            } 
            else if (type == 0x02) 
            {
              password = String(temp);
            }
            i = data_end + 1; 
            wifi_changed_flage=1;
          }
        }
      }

      if (wifi_changed_flage)
      {
        Serial.println("SSID: " + ssid);
        Serial.println("Password: " + password);
        setup_wifi(); // 自定义函数连接 WiFi
        wifi_changed_flage=0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  
}