// 攀登：土壤传感器 - OneNet 物联网接入
// 供电：12V
// 通信：RS485-TTL


#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/************** 函数声明 ***************/
uint16_t CRC16(const uint8_t *data, uint16_t length);
bool checkCRC(const uint8_t *data, uint16_t len);
void parseModbusData(const uint8_t *data, uint16_t len);
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void sendSensorData(double data) ;

/************** WiFi 配置 **************/
const char* ssid = "abc";         // WiFi SSID
const char* password = "12345678"; // WiFi 密码

/************ OneNet MQTT 配置 ************/
const char* mqtt_server = "mqtts.heclouds.com";  
const int mqtt_port = 1883; 
const char* device_id = "test-v1";    
const char* product_id = "ix3yxLe12r"; 
const char* api_key = "version=2018-10-31&res=products%2Fix3yxLe12r%2Fdevices%2Ftest-v1&et=999986799814791288&method=md5&sign=aLfwfxqst6gFtQuC3WhnLA%3D%3D";

// **MQTT 主题**
const char* pubTopic = "$sys/ix3yxLe12r/test-v1/thing/property/post";
const char* replyTopic="$sys/ix3yxLe12r/test-v1/thing/property/post/reply";

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

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

/************* 主循环 *************/
void loop() {
  if (!client.connected()) 
  {
    reconnect();
  }
  client.loop();

  // 发送请求
  Serial2.write(send_byte[asr], 8);
  Serial.printf("发送查询: %s\n", sensor_names[asr]);
  Serial.println();

  delay(200);

  // 读取响应数据
  if (Serial2.available()) 
  {
    int len = Serial2.available();
    Serial2.readBytes(temp, len);
    Serial.println("接收传感器数据");
    parseModbusData(temp, len);

    if (checkCRC(temp, len)) 
    {
      Serial.println("CRC 校验成功\n");
      asr = (asr + 1) % 6; // 轮询下一个传感器
    } 
    else 
    {
      Serial.println("CRC 校验失败\n");
    }
  }

  delay(1000);
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
      Serial.printf("解析数据: %f\n", value);
      sendSensorData(value);
    } 
    else if (dataLength == 4) 
    { // 9字节
      uint16_t regValue_H = (data[3] << 8) | data[4];
      uint16_t regValue_L = (data[5] << 8) | data[6];
      uint32_t regValue = (regValue_H << 16) | regValue_L;
      Serial.printf("解析数据: %d\n", regValue);
      sendSensorData((int32_t)regValue);
    } 
    else {
      Serial.println("数据长度异常");
    }
  }
}


/************* WiFi 连接 *************/
void setup_wifi() {
  Serial.println("连接 WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nWiFi 连接成功!");
  Serial.print("IP 地址: ");
  Serial.println(WiFi.localIP());
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
    Serial.print("连接 OneNet MQTT...");

    if (client.connect(device_id, product_id, api_key)) 
    {
      Serial.println("连接成功!");
      client.subscribe(replyTopic); // 订阅属性下发
    } 
    else 
    {
      Serial.printf("连接失败, 状态码=%d, 5秒后重试...\n", client.state());
      switch (client.state()) 
      {
        case -4: Serial.println("连接超时"); break;
        case -3: Serial.println("连接丢失"); break;
        case -2: Serial.println("连接失败"); break;
        case -1: Serial.println("断开连接"); break;
        case 1: Serial.println("协议错误"); break;
        case 2: Serial.println("客户端标识无效"); break;
        case 3: Serial.println("服务器不可用"); break;
        case 4: Serial.println("用户名或密码错误"); break;
        case 5: Serial.println("未授权"); break;
        default: Serial.println("未知错误");
      }
      delay(5000);
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
  } 
  else 
  {
    Serial.println("发送失败");
  }

  delay(200);
}