#ifndef _WIFI_CONTROL_H_
#define _WIFI_CONTROL_H_

#include <WiFi.h>

// #define WIFI_AP 0
#define WIFI_STA 1
// #define WIFI_DUAL 2

class WiFiControl
{
public:
  WiFiControl();
  ~WiFiControl();
  void setWiFiConnection(String ssid, String pass, uint8_t mode);
  void setServerConnection(IPAddress server_ip, uint16_t port, IPAddress subnet=IPAddress(255, 255, 255, 0));
  void begin();
  bool isConnected();
  void setPrintFunc(void (*printFunc)(String));

private:
  String ssid_, pass_;
  IPAddress serverIP_, subnet_;
  uint16_t port_;
  uint8_t wifiMode_;
  WiFiServer *server_;
  WiFiClient client_;
  void (*printFunc_)(String);

  int read();
  void write(uint8_t *data, int length);
  void print(String message);
};

#endif
