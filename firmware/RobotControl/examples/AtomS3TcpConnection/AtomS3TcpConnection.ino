#include <M5AtomS3.h>
#include <WiFiControl.h>

WiFiControl wifiControl;

  IPAddress serverIP(192, 168, 97, 190); // サーバーのIPアドレス
  uint16_t port = 11411; // ポート番号
void setup(){
  M5.begin();
  M5.Lcd.setRotation(3);  // 画面向き設定（USB位置基準 0：下/ 1：右/ 2：上/ 3：左）
  M5.Lcd.setTextSize(2);  // 文字サイズ（整数倍率）
  
  String ssid = "xxxxxxxx";
  String pass = "xxxxxxxx";
  wifiControl.setWiFiConnection(ssid, pass, WIFI_STA);

  wifiControl.setServerConnection(serverIP, port);
  wifiControl.setPrintFunc([](String message) { M5.Lcd.println(message);});
  wifiControl.begin();
  M5.Lcd.clear();
  M5.Lcd.setCursor(0, 0);
  delay(1000);
}

void loop(){
  delay(1000);
}
