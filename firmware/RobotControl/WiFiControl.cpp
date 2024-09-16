#include "WiFiControl.h"

WiFiControl::WiFiControl() : server_(nullptr), printFunc_(nullptr) {}

WiFiControl::~WiFiControl() {
  if (server_ != nullptr) {
    server_->stop();
    delete server_;
  }
}

void WiFiControl::setWiFiConnection(String ssid, String pass, uint8_t mode=WIFI_STA) {
  ssid_ = ssid;
  pass_ = pass;
  wifiMode_ = mode;
}

void WiFiControl::setServerConnection(IPAddress server_ip, uint16_t port=11411, IPAddress subnet) {
  serverIP_ = server_ip;
  port_ = port;
  subnet_ = subnet;
}

void WiFiControl::begin() {
  if(wifiMode_ == WIFI_AP) {
    WiFi.softAP(ssid_, pass_);
    delay(100);
    WiFi.softAPConfig(serverIP_, serverIP_, subnet_);
    IPAddress myIP = WiFi.softAPIP();
    server_ = new WiFiServer(80);
    server_->begin();
    client_ = server_->available();
    print("Waiting for client");
    print("IP: " + myIP.toString());
    print("port: 80");
  }else if(wifiMode_ == WIFI_STA){
    WiFi.begin(ssid_, pass_);
    while( WiFi.status() != WL_CONNECTED) {
      delay(10);  
    }
    print("Connecting to server");
    print("IP: " + serverIP_.toString());
    print("port: " + String(port_));
    client_.connect(serverIP_, port_);
    if (isConnected()) {
      print("Succeed");
    } else {
      print("Failed to connect to server");
    }
  }
}

bool WiFiControl::isConnected() {
  return client_.connected();
}

void WiFiControl::setPrintFunc(void (*printFunc)(String)) {
  printFunc_ = printFunc;
}

int WiFiControl::read() {
  return client_.read();
}

void WiFiControl::write(uint8_t *data, int length) {
    for (int i = 0; i < length; i++) client_.write(data[i]);
}

void WiFiControl::print(String message) {
  if (printFunc_) {
    printFunc_(message);
  }
}
