#ifndef _WIFI_CONTROL_H_
#define _WIFI_CONTROL_H_

#include <WiFi.h>
#include <M5AtomS3.h>
#include <IcsHardSerialClass.h>

// #define WIFI_AP 0
#define WIFI_STA 1
// #define WIFI_DUAL 2

#define ACK_OP 0xFE
#define SEARCH_ID_OP 0x20
#define READ_SERVO_OP 0x21
#define M_S_CV_OP 0x10

#define BUFF_SIZE 128

class WiFiControl
{
public:
  WiFiControl();
  ~WiFiControl();
  void setWiFiConnection(String ssid, String pass, uint8_t mode);
  void setServerConnection(IPAddress server_ip, uint16_t port, IPAddress subnet=IPAddress(255, 255, 255, 0));
  void begin();
  void connect();
  bool isConnected();
  void setPrintFunc(void (*printFunc)(String));
  void rcb4Task(void *args) ;
  void startICS(HardwareSerial* serial, byte en_pin=6,
                byte tx_pin=38, byte rx_pin=5,
                long baudrate=1250000, int timeout=20);

private:
  String ssid_, pass_;
  IPAddress serverIP_, subnet_;
  uint16_t port_;
  uint8_t wifiMode_;
  WiFiServer *server_;
  WiFiClient client_;
  IcsHardSerialClass krs_;

  void (*printFunc_)(String);
  TaskHandle_t thp_[1];
  uint8_t rx_buff[BUFF_SIZE];
  uint8_t tx_buff[BUFF_SIZE];
  uint8_t tx_size = 0;
  uint8_t ids[18];
  uint8_t id_count = 0;

  void print(String message);
  void write(uint8_t *data, size_t length);
  int read();
  int read(uint8_t* buffer, size_t length);
  uint8_t rcb4_checksum(uint8_t* byte_list, size_t len);
  void executeCmd(uint8_t* rx_buffer, size_t length);
  void searchServoId();
  void servoCmd(uint8_t command, uint8_t* servo_info);
};

#endif
