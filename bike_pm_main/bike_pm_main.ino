/************************************************************************************************************************
Masterarbeit von Sarah Wauschkuhn,
Hochschule für Technik und Wirtschaft Berlin , 
2025

Hinweis: für das Projekt ist es im Vorfeld nötig einen Account bei TTN und Open Sensor Map anzulegen
TTN --> https://www.thethingsnetwork.org/docs/devices/registration/
Open Sensor Map --> https://opensensemap.org/register
Bitte prüfe deine LoRaWAN-Netzabdeckung für TTN -->  https://ttnmapper.org/heatmap/
Bitte prüfe bei Anpassung des Sendeintervalls, ob Fair Use Policy eingehalten wurde --> https://www.thethingsnetwork.org/airtime-calculator

Näheres hierzu ist in der dazugehörigen Masterthesis "Entwicklung eines Feinstaubmessgeräts für Radfahrende" zu finden.

Achtung ggf. C:\Users\name\Documents\Arduino\libraries\Heltec_ESP32_Dev-Boards\src\driver\sx1276.c in --> sx1276.c_backup umwandeln, macht sonst Probleme
************************************************************************************************************************/

//ESP32ChipID=9CD9A67A9834
// Basis-Bibliotheken für Arduino und Heltec
#include "Arduino.h"
#include <heltec.h>            // LoRa + OLED + ESP32-Funktionen (Heltec)

//für Heltec und Oled Display
#include "HT_SSD1306Wire.h"

//für HM3301 
#include <Tomoto_HM330X.h>
#include <Wire.h>

//für DHT11
#include <DHT.h>

//GPS
#include "HT_TinyGPS++.h"

//Lora
// LoRaWAN LMIC-Bibliotheken
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


//GPS
TinyGPSPlus gps;
//int year, month, day, hour, minute, second; //Zeit
double lat = 0.0, lon = 0.0; //Koordinaten
// GPS Pins definieren
#define GPS_RX 17  // Empfängt Daten vom GPS (GPS TX)
#define GPS_TX 12  // Sendet Daten an GPS (GPS RX)
HardwareSerial gpsSerial(2); // serial2 wird genutzt

// LoRa-Parameter (Europa)
// AppEUI, DevEUI und AppKey als Hex-Byte-Array (LSB-First)
static const u1_t PROGMEM APPEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; /* Füge eigene App EUI ein */ 
static const u1_t PROGMEM DEVEUI[8]  = { -, -, -, -, -, -, -, - }; /* Füge eigene Dev EUI ein, ist individuell und wird beim erstellen des Devices nach APPEUI generiert */ 
static const u1_t PROGMEM APPKEY[16] = { -, -, -, -, -, -, -, -,
                                         -, -, -, -, -, -, -, - }; /* Füge eigenen App Key ein */ 

const lmic_pinmap lmic_pins = {
  .nss = 18,      // LoRa_CS
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,      // LoRa_RST
  .dio = {26, 35, 34}  // DIO0, DIO1, DIO2
};

static osjob_t sendjob;
//const unsigned TX_INTERVAL = 30; // ****Datenübertragung alle 30 Sekunden**** --> gut zum testen, doch kann das Netz schnell überlasten
//const unsigned TX_INTERVAL = 60; //****** Datenübertragung alle 60 Sekunden**** ---> Mittelweg, aber noch besser mehr Zeit geben
//const unsigned TX_INTERVAL = 120; // ****Datenübertragung alle 120 Sekunden****

unsigned long lastSensorRead = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastSendTime = 0;

//Zeitmanagement, wichtig, da LoRa-Datenübertragung sehr getacktet ist. Die Prozesse des Devices müssen daher getaktet werden
const unsigned long sensorInterval = 5000;      // alle 5 Sekunden Sensoren auslesen
const unsigned long displayInterval = 2000;     // alle 2 Sekunden Display aktualisieren
const unsigned long sendInterval = 120000;      // alle 120 Sekunden senden

String loraStatusText = "";                // Text wie „Verbunden“, „Sende…“ etc.
unsigned long loraStatusTime = 0;          // Zeitpunkt der letzten Statusänderung
const unsigned long statusDisplayDuration = 4000;  // wie lange Status angezeigt wird (ms)

//HM330x - Feinstaub
//int pm1 = 0; //hierfür gibt es keine WHO Grenzwerte >> Falls doch gewünscht Programmversion mit pm1 wählen 
int pm25 = 0;
int pm10 = 0;
Tomoto_HM330X sensor;
//Achtung Standart für I2C kommunikation für heltec wifi lora v2! 
#define SDA 4
#define SCL 15

//DHT22
//#define DHT_PIN 13  > nicht verwenden Boot strap pin
#define DHT_PIN 25
#define DHT_TYPE DHT22       // DHT Typ, für DHT11 > #define DHT_TYPE DHT11   
DHT dht(DHT_PIN, DHT_TYPE);    // Initialisiere den DHT11 Sensor
float hum, temp;

//Warn-LED (RGB LED, Widerstände 120 OHM vor jedem PIN)
const int ledRotPin = 22;
const int ledGruenPin = 23;
const int ledBlauPin =13; 

//rotate only for GEOMETRY_128_64
#define SDA_OLED 4
#define SCL_OLED 15
#define RST_OLED 16
static SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

//BitmapBild für Fahrrad erstellt mit GIMP
#define im_width 125
#define im_height 64
static unsigned char im_bits[] = {
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff,
   0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xc3, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe1, 0xff, 0xff, 0xff, 0xff, 0x1f,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xf0, 0xff,
   0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x00, 0xfe, 0xff,
   0xff, 0x3f, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff,
   0x1f, 0x00, 0xfe, 0xff, 0xff, 0x1f, 0xfe, 0xff, 0xff, 0xff, 0xff, 0x1f,
   0xff, 0xff, 0xff, 0xff, 0x1f, 0x00, 0xfe, 0xff, 0xff, 0x1f, 0xff, 0xff,
   0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc7, 0xff, 0xff,
   0xff, 0x3f, 0xfe, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xc7, 0xff, 0xff, 0xff, 0x3f, 0xfe, 0xff, 0xff, 0xff, 0xff, 0x1f,
   0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0xff, 0xff, 0xff, 0x7f, 0xfe, 0xff,
   0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0xff, 0xff,
   0xff, 0x7f, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff,
   0xff, 0x1f, 0xff, 0xff, 0xff, 0x7f, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x1f,
   0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0x7f, 0xf8, 0xff,
   0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xfe, 0xff,
   0xff, 0x3f, 0xf8, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff,
   0xff, 0x0f, 0xfe, 0xff, 0xff, 0x3f, 0xf8, 0xff, 0xff, 0xff, 0xff, 0x1f,
   0xff, 0xff, 0xff, 0xff, 0xff, 0x47, 0xfc, 0xff, 0xff, 0x1f, 0xf1, 0xff,
   0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc7, 0xf8, 0xff,
   0xff, 0x8f, 0xf3, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xe3, 0xf8, 0xff, 0xff, 0x87, 0xe3, 0xff, 0xff, 0xff, 0xff, 0x1f,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xf3, 0xf1, 0xff, 0xff, 0xc7, 0xe3, 0xff,
   0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0x1f, 0xf8, 0xf1, 0xf3, 0xff,
   0xff, 0xe3, 0xc7, 0x81, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0x01,
   0x80, 0xf8, 0xe3, 0xff, 0xff, 0xf1, 0x07, 0x00, 0xf8, 0xff, 0xff, 0x1f,
   0xff, 0xff, 0x7f, 0x00, 0x00, 0xf8, 0xc7, 0xff, 0xff, 0xf1, 0x07, 0x00,
   0xe0, 0xff, 0xff, 0x1f, 0xff, 0xff, 0x1f, 0xf8, 0x1f, 0xf8, 0xc7, 0xff,
   0xff, 0xf8, 0x01, 0xff, 0x81, 0xff, 0xff, 0x1f, 0xff, 0xff, 0x0f, 0xff,
   0x7f, 0xf0, 0x8f, 0xff, 0x7f, 0xfc, 0x80, 0xff, 0x0f, 0xff, 0xff, 0x1f,
   0xff, 0xff, 0x87, 0xff, 0x3f, 0xe0, 0x8f, 0xff, 0x3f, 0x7e, 0x18, 0xff,
   0x1f, 0xfe, 0xff, 0x1f, 0xff, 0xff, 0xe3, 0xff, 0x3f, 0xc2, 0x1f, 0xff,
   0x3f, 0x3e, 0x1c, 0xff, 0x7f, 0xfc, 0xff, 0x1f, 0xff, 0xff, 0xf1, 0xff,
   0x1f, 0x8f, 0x1f, 0xff, 0x1f, 0x1f, 0x3f, 0xff, 0xff, 0xf8, 0xff, 0x1f,
   0xff, 0xff, 0xf8, 0xff, 0x9f, 0x1f, 0x3f, 0xfe, 0x8f, 0x8f, 0x3f, 0xfe,
   0xff, 0xf1, 0xff, 0x1f, 0xff, 0x7f, 0xfc, 0xff, 0x8f, 0x1f, 0x7f, 0xfc,
   0x87, 0x8f, 0x7f, 0xfe, 0xff, 0xe3, 0xff, 0x1f, 0xff, 0x7f, 0xfc, 0xff,
   0xcf, 0x3f, 0x7e, 0xfc, 0xc7, 0xc7, 0x7f, 0xfc, 0xff, 0xe3, 0xff, 0x1f,
   0xff, 0x3f, 0xfe, 0xff, 0xc7, 0x7f, 0xfc, 0xf8, 0xe3, 0xe3, 0x7f, 0xfc,
   0xff, 0xc7, 0xff, 0x1f, 0xff, 0x3f, 0xff, 0xff, 0xe3, 0x7f, 0xfc, 0xf8,
   0xf1, 0xe3, 0xff, 0xfc, 0xff, 0xcf, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0xff,
   0xe3, 0xff, 0xfc, 0xf1, 0xf0, 0xf3, 0xff, 0xf8, 0xff, 0x8f, 0xff, 0x1f,
   0xff, 0x1f, 0xff, 0xff, 0xf1, 0xff, 0xf8, 0x01, 0xf8, 0xf1, 0xff, 0xf9,
   0xff, 0x8f, 0xff, 0x1f, 0xff, 0x9f, 0xff, 0xff, 0xf1, 0xff, 0xf8, 0x03,
   0xfc, 0xf1, 0xff, 0xf1, 0xff, 0x9f, 0xff, 0x1f, 0xff, 0x9f, 0xff, 0xff,
   0xf8, 0xff, 0xf9, 0x03, 0xfc, 0xf9, 0xff, 0xf1, 0xff, 0x9f, 0xff, 0x1f,
   0xff, 0x8f, 0xff, 0xff, 0xf8, 0xff, 0xf9, 0xf1, 0xf8, 0xf9, 0xff, 0xf3,
   0xff, 0x1f, 0xff, 0x1f, 0xff, 0x8f, 0xff, 0x7f, 0xf8, 0xff, 0xf1, 0xf8,
   0xf9, 0xf9, 0xff, 0xe3, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x8f, 0xff, 0x7f,
   0x00, 0x00, 0x00, 0xf8, 0xf9, 0xf9, 0xff, 0xe7, 0xff, 0x1f, 0xff, 0x1f,
   0xff, 0x8f, 0xff, 0x7f, 0x00, 0x00, 0x00, 0xf8, 0xf9, 0xf9, 0xff, 0xe7,
   0xff, 0x1f, 0xff, 0x1f, 0xff, 0x8f, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xf9,
   0xf9, 0xf9, 0xff, 0xff, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x9f, 0xff, 0xff,
   0xff, 0xff, 0xf9, 0x61, 0xf8, 0xf9, 0xff, 0xff, 0xff, 0x9f, 0xff, 0x1f,
   0xff, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xf9, 0x03, 0xfc, 0xf9, 0xff, 0xff,
   0xff, 0x9f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x07,
   0xfe, 0xf1, 0xff, 0xff, 0xff, 0x8f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0xff,
   0xff, 0xff, 0xf8, 0xff, 0xff, 0xf1, 0xff, 0xff, 0xff, 0x8f, 0xff, 0x1f,
   0xff, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xf3, 0xff, 0xff,
   0xff, 0xcf, 0xff, 0x1f, 0xff, 0x3f, 0xfe, 0xff, 0xff, 0x7f, 0xfc, 0xff,
   0xff, 0xe3, 0xff, 0xff, 0xff, 0xc7, 0xff, 0x1f, 0xff, 0x3f, 0xfe, 0xff,
   0xff, 0x7f, 0xfe, 0xff, 0xff, 0xe7, 0xff, 0xff, 0xff, 0xc7, 0xff, 0x1f,
   0xff, 0x7f, 0xfc, 0xff, 0xff, 0x3f, 0xfe, 0xff, 0xff, 0xc7, 0xff, 0xff,
   0xff, 0xe3, 0xff, 0x1f, 0xff, 0xff, 0xf8, 0xff, 0xff, 0x1f, 0xff, 0xff,
   0xff, 0x8f, 0xff, 0xff, 0xff, 0xf1, 0xff, 0x1f, 0xff, 0xff, 0xf0, 0xff,
   0xff, 0x0f, 0xff, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xf0, 0xff, 0x1f,
   0xff, 0xff, 0xe1, 0xff, 0xff, 0x87, 0xff, 0xff, 0xff, 0x1f, 0xfe, 0xff,
   0x7f, 0xf8, 0xff, 0x1f, 0xff, 0xff, 0xc3, 0xff, 0xff, 0xc3, 0xff, 0xff,
   0xff, 0x3f, 0xfc, 0xff, 0x3f, 0xfc, 0xff, 0x1f, 0xff, 0xff, 0x87, 0xff,
   0xff, 0xe1, 0xff, 0xff, 0xff, 0x7f, 0xf8, 0xff, 0x1f, 0xfe, 0xff, 0x1f,
   0xff, 0xff, 0x0f, 0xfe, 0x7f, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xff,
   0x07, 0xff, 0xff, 0x1f, 0xff, 0xff, 0x3f, 0xf0, 0x0f, 0xfc, 0xff, 0xff,
   0xff, 0xff, 0x03, 0xff, 0xc0, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0x00,
   0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0xf0, 0xff, 0xff, 0x1f,
   0xff, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00,
   0xfc, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0x1f 
  };


///////////////////////////////////////////////////////////////////////////////////////////////////////
//Startbildschirmprogramm1, Quelle: DrawingDemo der Heltec ESP32 Dev Boards Beispiele//////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
void drawLines() {
  for (int16_t i=0; i<display.getWidth(); i+=4) {
    display.drawLine(0, 0, i, display.getHeight()-1);
    display.display();
    delay(10);
  }
  for (int16_t i=0; i<display.getHeight(); i+=4) {
    display.drawLine(0, 0, display.getWidth()-1, i);
    display.display();
    delay(10);
  }
  delay(250);

  display.clear();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//Programmbildschirm - Anzeigesteuerung für OLED, ohne Lora-Status/////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
/*void showOLED(int pm25,int pm10, int hum, int temp){
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);

    //Feinstaubwerte festlegen
    display.drawString(0   , 0  ,   "PM");
    display.drawString(45  , 0  ,   "[ug/m^3]");
    display.drawString(0   , 16 ,   "PM2.5");
    display.drawString(56  , 16 ,   (String)pm25);
    display.drawString(0   , 32 ,   "PM10");
    display.drawString(56  , 32 ,   (String)pm10);

    //Risiko anzeigen
    display.drawString(94 , 0  ,  "RISIKO" );
    String pm25Risk = risikoPM25(pm25);
    display.drawString(94 , 16 ,    pm25Risk);
    String pm10Risk = risikoPM10(pm10);
    display.drawString(94 , 32 ,   pm10Risk);

    // Temperatur und Luftfeuchtigkeit anzeigen
    display.drawString(0   , 48 , "Temp: " + String((int)temp) + " °C");
    display.drawString(65   , 48 , "Hum: " + String((int)hum) + " %");

    //GPS ok, Lora ok, PM Failed ok
    //oder andere straße vorschlagen 
}*/

///////////////////////////////////////////////////////////////////////////////////////////////////////
//Programmbildschirm - Anzeigesteuerung für OLED, inklusive Lora-Status////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
void showOLED(int pm25, int pm10, int hum, int temp) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT); //Aussichtung 
  display.setFont(ArialMT_Plain_10); //Schriftart

  // Feinstaubwerte festlegen
  display.drawString(0   , 0  , "PM");
  display.drawString(45  , 0  , "[ug/m^3]");
  display.drawString(0   , 16 , "PM2.5");
  display.drawString(56  , 16 , (String)pm25);
  display.drawString(0   , 32 , "PM10");
  display.drawString(56  , 32 , (String)pm10);

  // Risiko anzeigen
  display.drawString(94 , 0  , "RISIKO");
  String pm25Risk = risikoPM25(pm25);
  display.drawString(94 , 16 , pm25Risk);
  String pm10Risk = risikoPM10(pm10);
  display.drawString(94 , 32 , pm10Risk);

  // Temperatur und Luftfeuchtigkeit anzeigen (leicht zusammengeschoben)
  //display.drawString(0   , 48 , "Temp: " + String((int)temp) + "°C");
  //display.drawString(60  , 48 , "Hum: " + String((int)hum) + "%");

  // -------- Temp, Hum 2. Version
  display.drawString(0   , 48 , "T: " + String((int)temp) + "°C");
  display.drawString(50  , 48 , "H: " + String((int)hum) + "%");

  /*//Variante 2 --- vorbereitet für Weiterentwicklung des Prototyps, verbesserte Fehlerbehandlung 
  // Feinstaubwerte festlegen
  display.drawString(0   , 0  , "PM");
  display.drawString(45  , 0  , "[ug/m^3]");

  // PM2.5
  display.drawString(0   , 16 , "PM2.5");
  display.drawString(56  , 16 , pm25 >= 0 ? String(pm25) : "---");

  // PM10
  display.drawString(0   , 32 , "PM10");
  display.drawString(56  , 32 , pm10 >= 0 ? String(pm10) : "---");

  // Risiko anzeigen
  display.drawString(94 , 0  , "RISIKO");
  display.drawString(94 , 16 , pm25 >= 0 ? risikoPM25(pm25) : "-");
  display.drawString(94 , 32 , pm10 >= 0 ? risikoPM10(pm10) : "-");

  // Temperatur und Luftfeuchtigkeit anzeigen
  display.drawString(0   , 48 , (temp >= -40 && temp <= 85) ? ("T: " + String((int)temp) + "°C") : "T: ---");
  display.drawString(50  , 48 , (hum >= 0 && hum <= 100) ? ("H: " + String((int)hum) + "%") : "H: ---");
  */


  // LoRa Status-Icon anzeigen, falls aktiv (rechts unten)
  if (millis() - loraStatusTime < statusDisplayDuration && loraStatusText != "") {
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(124, 48, getLoRaSymbol());  // Symbol z. B."..." oder "ok"
  }

  display.display();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
//Lorastatus für Programmbildschirm///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
String getLoRaSymbol() {
  if (loraStatusText.indexOf("Verbinde") >= 0) return "...";
  if (loraStatusText.indexOf("verbunden") >= 0) return "ok";
  if (loraStatusText.indexOf("fehlgeschl") >= 0) return "er";
  if (loraStatusText.indexOf("Sende") >= 0) return ">>";
  if (loraStatusText.indexOf("Upload") >= 0) return "up";
  return "no";
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Funktion zur Bewertung des PM2.5-Risikos nach WHO-Richtlinien (2021)////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
// Tagesmittelwerte:
// - ≤ 15 µg/m³ → gesundheitlich unbedenklich (grün)
// - 16–25 µg/m³ → leicht erhöhtes Risiko (gelb)
// - > 25 µg/m³ → gesundheitlich bedenklich (rot)
String risikoPM25(int pm25) {
  if (pm25 <= 15) {
    return "Niedrig";  // WHO-Grenzwert eingehalten
  } 
  else if (pm25 <= 25) {
    return "Mittel";   // Grenzwert leicht überschritten
  } 
  else {
    return "Hoch";     // Deutlich über WHO-Grenzwert
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Funktion zur Bewertung des PM10-Risikos nach WHO-Richtlinien (2021)/////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
// Tagesmittelwerte:
// - ≤ 45 µg/m³ → gesundheitlich unbedenklich (grün)
// - 46–60 µg/m³ → leicht erhöhtes Risiko (gelb)
// - > 60 µg/m³ → gesundheitlich bedenklich (rot)
String risikoPM10(int pm10) {
  if (pm10 <= 45) {
    return "Niedrig";  // WHO-Grenzwert eingehalten
  } 
  else if (pm10 <= 60) {
    return "Mittel";   // Grenzwert leicht überschritten
  } 
  else {
    return "Hoch";     // Deutlich über WHO-Grenzwert
  }
}

//Hier einkommentieren, falls EU-Grenzwerte erwünscht sind -- Hinweis: WHO besitzt Stand 2025 strengere Richtlinien als EU
// Reformen sehen schrittweise Anpassungen an WHO-Richtwerte bis 2030 vor.
/*///////////////////////////////////////////////////////////////////////////////////////////////////////
// Funktion zur Bewertung des PM2.5-Risikos nach EU-Grenzwerten (Tagesmittelwert, Stand 2008) /////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
// Tagesmittelwerte (empfohlen):
// - ≤ 25 µg/m³ → gesundheitlich unbedenklich (grün)
// - 26–50 µg/m³ → leicht erhöhtes Risiko (gelb)
// - > 50 µg/m³ → gesundheitlich bedenklich (rot)
String risikoPM25_EU(int pm25) {
  if (pm25 <= 25) {
    return "Niedrig";  // EU-Grenzwert eingehalten
  } 
  else if (pm25 <= 50) {
    return "Mittel";   // Grenzwert leicht überschritten
  } 
  else {
    return "Hoch";     // Deutlich über EU-Grenzwert
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Funktion zur Bewertung des PM10-Risikos nach EU-Grenzwerten (Tagesmittelwert, Stand 2008) /////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
// Tagesmittelwerte (empfohlen):
// - ≤ 50 µg/m³ → gesundheitlich unbedenklich (grün)
// - 51–75 µg/m³ → leicht erhöhtes Risiko (gelb)
// - > 75 µg/m³ → gesundheitlich bedenklich (rot)
String risikoPM10_EU(int pm10) {
  if (pm10 <= 50) {
    return "Niedrig";  // EU-Grenzwert eingehalten
  } 
  else if (pm10 <= 75) {
    return "Mittel";   // Grenzwert leicht überschritten
  } 
  else {
    return "Hoch";     // Deutlich über EU-Grenzwert
  }
}*/

///////////////////////////////////////////////////////////////////////////////////////////////////////
//Energiemanagement////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
void VextON(void){
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// ****Helferfunktionen für LoRaWAN****///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
/*Erklärung für Funktionen:
Die LMIC-Bibliothek ruft diese Funktionen auf, um an die notwendigen Zugangsdaten zu kommen. 
Statt die Daten direkt im Code anzugeben, nutzt LMIC diese Helferfunktionen.

1.Funktion:
> APPEUI (Application EUI, 8 Byte) wird in den Puffer(buf) kopiert.
> APPEUI ist eine eindeutige ID für deine TTN-Anwendung- wird vom LMIC-Stack aufgerufen, 
  wenn er diese ID benötigt.
2.Funktion: 
> DEVEUI (Device EUI, 8 Byte) wird in Puffer(buf) kopiert.
> Jedes LoRaWAN-Gerät hat eine eindeutige Device EUI- wird beim TTN-Beitritt (Join Request) benötigt.
3.Funktion: 
> APPKEY (Application Key, 16 Byte) wird in Puffer (buf) kopiert.
> !!! Wichtig: AppKey muss geheim bleiben!!! Er sichert die Kommunikation.
> TTN nutzt den AppKey, um das Gerät zu authentifizieren.
*/
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16);}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// setop()/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  //RGB LED
  pinMode(ledRotPin, OUTPUT);  
  digitalWrite(ledRotPin, LOW); //R
  pinMode(ledGruenPin, OUTPUT);  
  digitalWrite(ledGruenPin, LOW); //G 
  pinMode(ledBlauPin, OUTPUT);  
  digitalWrite(ledBlauPin, LOW);  //B

  VextON();
  delay(100);

  //************************Starte Startbildschirm************************************************
  display.init();
  display.clear();
  display.display();
  
  display.setContrast(255);
  display.screenRotate(ANGLE_180_DEGREE); //Ausrichtung Knopf Links vom Display, Garmin unter Display
  //display.screenRotate(ANGLE_0_DEGREE); //Ausrichtung Knopf Rechts vom Display, Garmin über Display

  drawLines(); //Funktion Startbildschirm 1
  delay(1000);
  display.clear();

  display.clear();  // Bildschirm löschen
  display.drawXbm(0, 0, im_width, im_height, im_bits);  // Das Bild anzeigen
  display.display();   // Anzeige aktualisieren
  delay(5000);          // 1 Sekunde warten

  
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.clear();
  display.display();
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 32-16/2, "Hello BIKE_PM");
  display.display();
  delay(2000);
  display.clear();
  //*********************Ende Startbildschirm*********************************************************

  //HM330X starten
  if (!sensor.begin(0x40)) Serial.println("HM3301 Fehler!"); //Quelle für i2c: Sensor
  Serial.println();

  //DHT11 starten
  dht.begin();

  // LMIC initialisieren
  os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Disable link-check mode and ADR, because ADR tends to complicate testing.
    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF9, 14);
    LMIC_startJoining();

  delay(200);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////
//loop/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  unsigned long now = millis();

  // GPS verarbeiten
  while (gpsSerial.available()) { //ist GPS da?
    gps.encode(gpsSerial.read());  // GPS-Daten kontinuierlich parsen (NMEA-Zeichenkette dekodieren)
  }
  if (gps.location.isValid()) { //sind die Daten gueltig?
    lat = gps.location.lat(); //Latitude extrahieren
    lon = gps.location.lng(); //Longitude extrahieren
  }

  // Sensoren auslesen
  if (now - lastSensorRead > sensorInterval) {
    lastSensorRead = now;

    // HM3301 auslesen
    if (!sensor.readSensor()) {
      Serial.println("HM3301 Fehler");
      pm25 = -1; //absichtlicher Fehler, da nicht möglich
      pm10 = -1; 
    } else {
      pm25 = sensor.std.getPM2_5(); //gibt int aus
      pm10 = sensor.std.getPM10(); //gibt int aus 
    }

    // DHT22 auslesen
    hum = dht.readHumidity();
    temp = dht.readTemperature();
    if (isnan(hum) || isnan(temp)) {
      hum = -1; //absichtlicher Fehler, unwahrscheinlich, dass alle Sensoren -1 ausgeben 
      temp = -1;
    }

    // RGB-LED-Farblogik zur visuellen Darstellung der Luftqualitätsstufen gemäß PM2.5- und PM10-Grenzwerten (WHO)
    if (pm25 <= 15 && pm10 <= 45) { //gruen
      digitalWrite(ledGruenPin, HIGH);
      digitalWrite(ledRotPin, LOW);
      digitalWrite(ledBlauPin, LOW);
    } else if (pm25 <= 25 && pm10 <= 75) {//gelb
      digitalWrite(ledGruenPin, HIGH);
      digitalWrite(ledRotPin, HIGH);
      digitalWrite(ledBlauPin, LOW);
    } else { //rot
      digitalWrite(ledGruenPin, LOW);
      digitalWrite(ledRotPin, HIGH);
      digitalWrite(ledBlauPin, LOW);
    }

    // Debug-Ausgabe
    /*Serial.print("GPS OK? "); Serial.println(gps.location.isValid() ? "JA" : "NEIN");
    Serial.print("PM2.5: "); Serial.println(pm25);
    Serial.print("PM10: "); Serial.println(pm10);
    Serial.print("Temp: "); Serial.println(temp);
    Serial.print("Hum: "); Serial.println(hum);
    Serial.print("Lat: "); Serial.println(lat, 6);
    Serial.print("Lon: "); Serial.println(lon, 6);
    Serial.print("Satelliten: "); Serial.println(gps.satellites.value());
    Serial.print("HDOP: "); Serial.println(gps.hdop.hdop());*/
  }

  // OLED-Anzeige aktualisieren, wenn das Intervall überschritten ist
  /*now -               Zeit die seit Start von millis() vergangen ist
   lastDisplayUpdate -  Der Zeitpunkt (in ms), an dem das Display zuletzt aktualisiert wurde
   displayIntervall -   Zeitpunkt wann Display das letzte mal aktualisiert wurde*/
  if (now - lastDisplayUpdate > displayInterval) {
    lastDisplayUpdate = now; 
    showOLED(pm25, pm10, hum, temp); // Sensorwerte auf dem Display anzeigen
    //display.display();
  }

  // LoRaWAN Daten senden
  if (now - lastSendTime > sendInterval) { // Prueft, ob seit dem letzten Sendezeitpunkt genug Zeit vergangen ist (Intervallsteuerung).
    lastSendTime = now; // Zeitstempel aktualisieren, um Intervallsteuerung zu ermoeglichen; verhindert mehrfaches Senden im selben Intervall
    do_send(&sendjob); // Sendevorgang starten: Payload wird vorbereitet und ueber LoRaWAN uebertragen; uebergibt einen Auftrag an LMIC, um Sensordaten als Payload ueber LoRaWAN zu uebertragen.
  }

  /*
  // Weiterentwicklung um Fehlerhafte Weiterleitung an OSEM auszuschließen
    // Prüfung auf die -1 bei ALLEN Sensoren vorliegt
    bool all_invalid = 
      (pm25 == -1) &&
      (pm10 == -1) &&
      (temp == -1) &&
      (hum == -1);

  if (now - lastSendTime > sendInterval) {
    lastSendTime = now;
    if (!all_invalid) {
        do_send(&sendjob);
    } else {
        Serial.println("Alle Sensorwerte ungültig, keine LoRa-Übertragung."); //Sendevorgang verhindern, falls ungültige Werte --> da Kontaktfehler; noch am Starten etc. 
    }
  }*/

  // LMIC weiterlaufen lassen
  os_runloop_once(); 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// LMIC Sende-Funktion/////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) { //Abfrage ob der LoRaWAN-Stack aktuell mit Senden oder Empfangen beschaeftigt ist (OP_TXRXPEND ist ein Bit-Flag)
    Serial.println("LoRa TX läuft noch...");
  } else {
    // Payload vorbereiten
    byte payload[14];  // 2x PM, 1x Temp, 1x RH, 2x GPS à 4 Bytes = 14

    // Feinstaub (PM2.5 & PM10)
    payload[0] = pm25 >> 8; //oberes Byte von pm25; ... >> N: Rechtsverschiebung – holt das Byte an Position N
    payload[1] = pm25 & 0xFF; //unteres Byte von pm25; ... & 0xFF: Maskiert alle Bits außer den unteren 8 (ein Byte)
    payload[2] = pm10 >> 8; //oberes Byte von pm10
    payload[3] = pm10 & 0xFF; //unteres Byte von pm10

    // Temperatur & Luftfeuchtigkeit (nur ganzzahlig, je 1 Byte)
    payload[4] = (int)temp;
    payload[5] = (int)hum;

    // GPS-Koordinaten in Mikrograd umwandeln
    int32_t lat_bin = lat * 1e6;  // z.B. 52.520008 → 52520008
    int32_t lon_bin = lon * 1e6;

    // Latitude (Big Endian)
    payload[6]  = (lat_bin >> 24) & 0xFF;// Byte 1 (höchstes)
    payload[7]  = (lat_bin >> 16) & 0xFF;
    payload[8]  = (lat_bin >> 8)  & 0xFF;
    payload[9]  =  lat_bin        & 0xFF;// Byte 4 (niedrigstes)

    // Longitude (Big Endian)
    payload[10] = (lon_bin >> 24) & 0xFF;// Byte 1 (höchstes)
    payload[11] = (lon_bin >> 16) & 0xFF;
    payload[12] = (lon_bin >> 8)  & 0xFF;
    payload[13] =  lon_bin        & 0xFF;// Byte 4 (niedrigstes)

    // Senden
    /*  1: LoRaWAN-Portnummer (in TTN entsprechend zugeordnet, z.B. für einen Payload-Decoder).
        payload: Pointer auf das zu sendende Byte-Array.
        sizeof(payload): Länge des Payloads in Bytes (hier 14).
        0: Unbestätigte Übertragung (kein ACK erforderlich) >> siehe TTN Fair Policy, sonst weniger Airtime am Tag moeglich*/
    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    Serial.println("LoRaWAN Daten gesendet");
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//Lora Event Handler///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
/*Event-Handler für die LoRaWAN-Bibliothek LMIC. Sie wird automatisch aufgerufen, wenn das LoRa-Modem 
ein bestimmtes Ereignis (event) meldet –z. B. Verbindungsaufbau, Datenübertragung, Fehlermeldung. 
Hier wird auf die verschiedenen LMIC-Ereignisse reagiert und entsprechende Meldungen auf dem seriellen
Monitor ausgegeben.*/
void onEvent(ev_t ev) {
  Serial.print("LMIC Event: ");
  Serial.println(ev); //gibt numerischen Wert des Events zur Diagnose aus

  switch (ev) {
    case EV_JOINING:
      Serial.println("Beitritt zu TTN läuft...");
      loraStatusText = "Verbinde mit TTN...";
      loraStatusTime = millis();
      break;

    case EV_JOINED:
      Serial.println("Erfolgreich mit TTN verbunden!");
      loraStatusText = "TTN verbunden";
      loraStatusTime = millis();
      // Optionale Link Check Abschaltung:
      LMIC_setLinkCheckMode(0);
      break;

    case EV_JOIN_FAILED:
      Serial.println("Beitritt zu TTN fehlgeschlagen!");
      loraStatusText = "TTN Join fehlgeschl";
      loraStatusTime = millis();
      break;

    case EV_TXSTART:
      Serial.println("Sende Daten...");
      loraStatusText = "Sende";
      loraStatusTime = millis();
      break;

    case EV_TXCOMPLETE:
      Serial.println("Datenübertragung abgeschlossen.");
      loraStatusText = "Upload";
      loraStatusTime = millis();
      break;

    case EV_TXCANCELED:
      Serial.println("Senden abgebrochen!");
      loraStatusText = "Senden abgebrochen";
      loraStatusTime = millis();
      break;

    case EV_REJOIN_FAILED:
      Serial.println("Rejoin fehlgeschlagen!");
      loraStatusText = "Rejoin fehlgeschl";
      loraStatusTime = millis();
      break;

    default:
      Serial.println("Anderes LoRaWAN Ereignis");
      loraStatusText = "LoRa Event: " + String(ev);
      loraStatusTime = millis();
      break;
  }
}
