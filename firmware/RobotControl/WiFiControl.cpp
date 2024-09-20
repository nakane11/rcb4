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
    M5.Lcd.fillScreen(GREEN);
    delay(1000);

    while (1) {
      M5.Lcd.setCursor(0, 0);
      print("Connecting to server");
      print("IP: " + serverIP_.toString());
      print("port: " + String(port_));
      client_.connect(serverIP_, port_);
      if (isConnected()) {
        M5.Lcd.fillScreen(BLUE);
        print("Succeed");
        delay(500);
        break;
      } else {
        print("Failed to connect to server");
        M5.Lcd.fillScreen(RED);
        delay(500);
      }
    }
  }
  M5.Lcd.clear();
  M5.Lcd.setCursor(0, 0);
  xTaskCreatePinnedToCore([](void *args) {
                            static_cast<WiFiControl*>(args)->rcb4Task(args);
                          }, "rcb4Task", 4096, this, 1, &thp_[0], 0);
}

void WiFiControl::connect() {
  if (!isConnected()) {
    client_.stop();
    M5.Lcd.println("try to connect");
    client_.connect(serverIP_, port_);
    delay(100);
  }
}

bool WiFiControl::isConnected() {
  return client_.connected();
}

void WiFiControl::setPrintFunc(void (*printFunc)(String)) {
  printFunc_ = printFunc;
}

void WiFiControl::rcb4Task(void *args) {
  while (true) {
    uint8_t rx_size = read();
    if(rx_size>0){
      rx_buff[0] = rx_size;
      size_t bytesRead = read(&rx_buff[1], rx_size-1);
      if(rx_buff[bytesRead] != rcb4_checksum(rx_buff, bytesRead)) {
        continue;
      }
      executeCmd(rx_buff, bytesRead);
      write(tx_buff,tx_size);
    }
    delay(200);
  }
}

void WiFiControl::startICS(HardwareSerial* serial, byte en_pin, byte tx_pin,
                           byte rx_pin, long baudrate, int timeout) {
  krs_ = IcsHardSerialClass(serial, en_pin, baudrate, timeout);
  serial->begin(baudrate, SERIAL_8E1, rx_pin, tx_pin, false, timeout);
  krs_.begin();
}

int WiFiControl::read() {
  connect();
  return client_.read();
}

int WiFiControl::read(uint8_t* buffer, size_t length) {
  connect();
  return client_.read(buffer, length);
}

void WiFiControl::write(uint8_t *data, size_t length) {
  connect();
  M5.Lcd.println("writing ");
  for (int i = 0; i < length; i++) {
    M5.Lcd.print(data[i]);
    M5.Lcd.print(' ');
    client_.write(data[i]);
  }
  M5.Lcd.println("writing end");
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

void WiFiControl::executeCmd(uint8_t* rx_buffer, size_t length) {
  uint8_t command = rx_buffer[1];
  tx_size = 4;
  tx_buff[2] = 0x06;

  switch (command) {
  case ACK_OP: {
    break;
  }
  case SEARCH_ID_OP: {
    searchServoId();
    tx_size = 3 + id_count;
    for(int i=0;i<id_count;i++) {
      tx_buff[i+2] = ids[i];
    }
    break;
  }
  case M_S_CV_OP: {
    servoCmd(command, &rx_buffer[2]);
    break;
  }
  case READ_SERVO_OP: {
    tx_size = 3 + id_count*2;
    for(int i=0;i<id_count;i++) {
      uint16_t position = krs_.getPos(ids[i]);
      tx_buff[2 + i * 2] = position & 0xFF;          // Low byte
      tx_buff[2 + i * 2 + 1] = (position >> 8) & 0xFF;  // High byte
    }
    break;
  }
  case SERVO_FREE_OP: {
    setFree(command, &rx_buffer[2]);
    break;
  }
  default: break;
  }
  tx_buff[0] = tx_size;
  tx_buff[1] = command;
  tx_buff[tx_size-1] = rcb4_checksum(tx_buff, tx_size-1);

}

void WiFiControl::searchServoId() {
  id_count = 0;
  for (int i = 0; i < 18; ++i) {
    int pos = krs_.getPos(i);
    if (pos != -1) {
      ids[id_count] = i;
      id_count++;
    }
  }
}

void WiFiControl::servoCmd(uint8_t command, uint8_t* servo_info) {
  uint16_t position;
  int j = 0;
  switch (command) {
  case M_S_CV_OP: {
    // M5.Lcd.clear();
    // M5.Lcd.setCursor(0, 0);
    for (int i=0; i<18; i++) {
      uint8_t idx = i*2;
      if ((servo_info[idx>>3] >> (idx%8)) & 0x1) {
        position = *(uint16_t *)&servo_info[6+2*j];
        krs_.setPos(i, position);
        j++;
      }
    }
  }
  default: break;
  }
}

void WiFiControl::setFree(uint8_t command, uint8_t* servo_ids) {
    M5.Lcd.setCursor(0, 0);
    for (int i=0; i<18; i++) {
      uint8_t idx = i*2;
      if ((servo_ids[idx>>3] >> (idx%8)) & 0x1) {
        krs_.setFree(i);
      }
    }
}

