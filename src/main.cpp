// 攀登：土壤传感器 - OneNet 物联网接入
// 供电：12V
// 通信：RS485-TTL
// 版本：2.0（新增 MQTT 数据上传）

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

/**************函数声明***************/
uint16_t CRC16(const uint8_t *data, uint16_t length);
bool checkCRC(const uint8_t *data, uint16_t len);
void parseModbusData(const uint8_t *data, uint16_t len);
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

/**************WiFi 配置**************/
const char* ssid = "abc";         // WiFi SSID
const char* password = "12345678"; // WiFi 密码

/************OneNet MQTT 配置************/
const char* mqtt_server = "mqtts.heclouds.com";  
const int mqtt_port = 1883; 
const char* device_id = "test-v1";    
const char* product_id = "ix3yxLe12r"; 
const char* api_key = "version=2018-10-31&res=products%2Fix3yxLe12r%2Fdevices%2Ftest-v1&et=999986799814791288&method=md5&sign=aLfwfxqst6gFtQuC3WhnLA%3D%3D";

// MQTT 主题
const char* pubTopic = "$sys/ix3yxLe12r/test-v1/thing/property/post";

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

const char* sensor_names[] = {"soil-PH", "soil-T", "soil-H", "soil-N", "soil-P", "soil-K"};

/********变量********/
byte temp[7]; // 传感器返回数据
int asr = 0;  // 传感器轮询索引

/*************程序初始化*************/
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

/*************主循环*************/
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // 发送请求
  Serial2.write(send_byte[asr], 8);
  Serial.printf("发送查询: %s\n", sensor_names[asr]);

  delay(200);

  // 读取响应数据
  if (Serial2.available()) {
    Serial2.readBytes(temp, sizeof(temp));
    Serial.println("接收传感器数据:");
    parseModbusData(temp, sizeof(temp));

    if (checkCRC(temp, sizeof(temp))) {
      Serial.println("CRC 校验成功\n");
      asr = (asr + 1) % 6; // 轮询下一个传感器
    } else {
      Serial.println("CRC 校验失败\n");
    }
  }

  delay(2000);
}

/************* CRC 计算 *************/
uint16_t CRC16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
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

/************* 解析 Modbus 数据并上传 MQTT *************/
void parseModbusData(const uint8_t *data, uint16_t len) {
  Serial.print("数据 (HEX): ");
  for (uint16_t i = 0; i < len; i++) {
    Serial.printf("%02X ", data[i]);
  }
  Serial.println();

  if (data[1] == 0x03) {  // 确保是读取寄存器
    double value = ((data[3] << 8) | data[4]) / 10.0;
    Serial.printf("%s: %.2f\n", sensor_names[asr], value);

    // 生成 JSON 数据
    String jsonPayload = "{\"id\":\"soil-v1\",\"params\":{\"";
    jsonPayload += sensor_names[asr];
    jsonPayload += "\":{\"value\":";
    jsonPayload += String(value, 6);
    jsonPayload += "}}}";

    // 发送 MQTT 数据
    if (client.connected()) {
      client.publish(pubTopic, jsonPayload.c_str());
      Serial.println("已发送数据: " + jsonPayload);
    } else {
      Serial.println("MQTT 连接失败，数据未发送");
    }
  }
}

/************* WiFi 连接 *************/
void setup_wifi() {
  Serial.println("连接 WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi 连接成功!");
  Serial.print("IP 地址: ");
  Serial.println(WiFi.localIP());
}

/************* MQTT 订阅回调函数 *************/
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("收到 MQTT 消息，主题: ");
  Serial.println(topic);
  Serial.print("内容: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/************* 连接 MQTT 服务器 *************/
void reconnect() {
  while (!client.connected()) {
    Serial.print("连接 OneNet MQTT...");

    if (client.connect(device_id, product_id, api_key)) {
      Serial.println("连接成功!");
      client.subscribe(pubTopic);  // 订阅数据推送主题
    } else {
      Serial.printf("连接失败, 状态码=%d,5秒后重试...\n", client.state());
      delay(5000);
    }
  }
}
