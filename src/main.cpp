#include <WiFi.h>
#include <esp_wifi.h>
#include "types.h"
#include "web_interface.h"
#include "deauth.h"
#include "definitions.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

#define BUTTON_LIST 2    // D2
#define BUTTON_DEAUTH 7  // D7
#define TRIMPOT A0       // A0 for selecting networks

#define LCD_ADDR 0x3F
#define LCD_COLS 16
#define LCD_ROWS 2

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

int curr_channel = 1;

int selectedNetwork = 0;  // Tracks currently selected network
bool attackInProgress = false;

void setup() {
  Serial.begin(115200);
    Serial.println("Starting...");
    pinMode(BUTTON_LIST, INPUT_PULLUP);
    pinMode(BUTTON_DEAUTH, INPUT_PULLUP);
    pinMode(TRIMPOT, INPUT);

    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Ready...");
    Serial.println("LCD initialized");
     
    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect();
    Serial.println("WiFi disconnected...");
    delay(100);
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
#endif
#ifdef LED
  pinMode(LED, OUTPUT);
#endif

  WiFi.mode(WIFI_MODE_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  start_web_interface();
}

void loop() {
  static int lastTrimpotValue = 0;
    int trimpotValue = analogRead(TRIMPOT);
    int numNetworks = WiFi.scanNetworks();
    
    if (numNetworks > 0) {
        selectedNetwork = map(trimpotValue, 0, 4095, 0, numNetworks - 1);
        if (abs(trimpotValue - lastTrimpotValue) > 50) { // Reduce LCD flicker
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Net: ");
            lcd.print(WiFi.SSID(selectedNetwork));
            
            lcd.setCursor(0, 1);
            lcd.print("RSSI: ");
            lcd.print(WiFi.RSSI(selectedNetwork));
            lastTrimpotValue = trimpotValue;
        }
    }

    if (digitalRead(BUTTON_LIST) == LOW) {
        lcd.clear();
        lcd.print("Scanning...");
        numNetworks = WiFi.scanNetworks();
    }

    if (digitalRead(BUTTON_DEAUTH) == LOW) {
        lcd.clear();
        lcd.print("Deauthing...");
        start_deauth(selectedNetwork, DEAUTH_TYPE_SINGLE, 1);
    }

  delay(100);
  if (deauth_type == DEAUTH_TYPE_ALL) {
    if (curr_channel > CHANNEL_MAX) curr_channel = 1;
    esp_wifi_set_channel(curr_channel, WIFI_SECOND_CHAN_NONE);
    curr_channel++;
    delay(10);
  } else {
    web_interface_handle_client();
  }
}