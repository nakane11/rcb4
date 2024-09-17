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
  xTaskCreatePinnedToCore([](void *args) {
                            static_cast<WiFiControl*>(args)->execute(args);
                          }, "Core0a", 4096, this, 3, &thp_[0], 0);
}

void WiFiControl::connect() {
  if (!isConnected()) {
    client_.connect(serverIP_, port_);
  }
}

bool WiFiControl::isConnected() {
  return client_.connected();
}

void WiFiControl::setPrintFunc(void (*printFunc)(String)) {
  printFunc_ = printFunc;
}

void WiFiControl::execute(void *args) {
  while (true) {
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 0);
    uint8_t rx_size = read();
    if(rx_size>0){
      rx_buff[0] = rx_size;
      size_t bytesRead = read(&rx_buff[1], rx_size-1);
      if(rx_buff[bytesRead] == rcb4_checksum(rx_buff, bytesRead)) {
          for(int i=1;i<bytesRead;i++){
            uint8_t data = rx_buff[i];
            print(String(data));
          }
      }
    }
    uint8_t len = 2;
    uint8_t tx_size = len + 2;
    tx_buff[0] = tx_size;
    tx_buff[tx_size-1] = rcb4_checksum(tx_buff, tx_size-1);
    tx_buff[1] = 1;
    tx_buff[2] = 2;
    write(tx_buff,tx_size);
    delay(1000);
  }
}


int WiFiControl::read() {
  connect();
  return client_.read();
}

int WiFiControl::read(uint8_t* buffer, uint8_t length) {
  connect();
  return client_.read(buffer, length);
}

void WiFiControl::write(uint8_t *data, int length) {
  for (int i = 0; i < length; i++) {
      connect();
      client_.write(data[i]);
  }
}

void WiFiControl::print(String message) {
  if (printFunc_) {
    printFunc_(message);
  }
}

 uint8_t WiFiControl::rcb4_checksum(uint8_t* byte_list, size_t len) {
    uint32_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
        checksum += (byte_list[i] & 0xFF);  // Mask each byte with 0xFF
    }
    return checksum & 0xFF;  // Mask the final result with 0xFF
}
