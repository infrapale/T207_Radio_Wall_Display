/***************************************************
  This is our GFX example for the Adafruit ILI9341 TFT FeatherWing
  ----> http://www.adafruit.com/products/3315

  Check out the links above for our tutorials and wiring diagrams

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_STMPE610.h>
#include <Fonts/FreeMonoBoldOblique12pt7b.h>
#include <Fonts/FreeSerif18pt7b.h>
#include <Fonts/FreeSerif24pt7b.h>
#include <Fonts/FreeSerif9pt7b.h>
#include "Secret.h"
#include "sens_db.h"

#ifdef ESP8266
   #define STMPE_CS 16
   #define TFT_CS   0
   #define TFT_DC   15
   #define SD_CS    2
#endif
#ifdef ESP32
   #define STMPE_CS 32
   #define TFT_CS   15
   #define TFT_DC   33
   #define SD_CS    14
#endif
#ifdef TEENSYDUINO
   #define TFT_DC   10
   #define TFT_CS   4
   #define STMPE_CS 3
   #define SD_CS    8
#endif
#ifdef ARDUINO_STM32_FEATHER
   #define TFT_DC   PB4
   #define TFT_CS   PA15
   #define STMPE_CS PC7
   #define SD_CS    PC5
#endif
#ifdef ARDUINO_NRF52_FEATHER /* BSP 0.6.5 and higher! */
   #define TFT_DC   11
   #define TFT_CS   31
   #define STMPE_CS 30
   #define SD_CS    27
#endif
#if defined(ARDUINO_MAX32620FTHR) || defined(ARDUINO_MAX32630FTHR)
   #define TFT_DC   P5_4
   #define TFT_CS   P5_3
   #define STMPE_CS P3_3
   #define SD_CS    P3_2
#endif


// Anything else!
#if defined (__AVR_ATmega32U4__) || defined(ARDUINO_SAMD_FEATHER_M0) || defined (__AVR_ATmega328P__) || defined(ARDUINO_SAMD_ZERO) || defined(__SAMD51__) || defined(__SAM3X8E__)
   #define STMPE_CS 6
   #define TFT_CS   9
   #define TFT_DC   10
   #define SD_CS    5
#endif

#include <RH_RF69.h>
//#define RFM_FEATHER_M0_M4
#define ARDUINO_SAMD_FEATHER_M0
//#define Astrid_ATmega328P
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.0   //915.0
#if defined(RFM_FEATHER_M0_M4)  //feateher wing without CPU for M0/M4
  #define RFM69_CS      10   // "B"
  #define RFM69_RST     11   // "A"
  #define RFM69_INT     6    // "D"
  #define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
  #define LED           13
#endif
#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif


#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13
#endif

#if defined (Astrid_ATmega328P)  // made  by Tom Höglund
  #define RFM69_INT     2  // 
  #define RFM69_CS      10  //
  #define RFM69_RST     9  // 
  #define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
#endif


#define TEXT_SIZE 1
#define TEXT_VISIBLE_ROWS 25
#define TEXT_VISIBLE_CHAR 44

#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000

char text_buffer[TEXT_VISIBLE_ROWS][TEXT_VISIBLE_CHAR];
uint8_t show_from=0;
uint8_t insert_at=0;
unsigned long show_menu_millis;
boolean menu_is_active;
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);
int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() {
  delay(4000);

  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  Serial.println("T207 Radio Wall Display");

  tft.begin();
  if (!ts.begin()) { 
    Serial.println("Unable to start touchscreen.");
  } 
  else { 
    Serial.println("Touchscreen started."); 
  }
  show_menu_millis = millis();
  menu_is_active = false;
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  
  //tft.setTextSize(2);
  tft.setFont(&FreeSerif24pt7b);
  //tft.setFont(&FreeMonoBoldOblique12pt7b);
  tft.println("T207 Radio Display 2019");
  tft.print("RST pin = "); tft.println(RFM69_RST,DEC);
  tft.print("CS pin = ");tft.println(RFM69_CS,DEC);
  tft.print("INT pin = ");tft.println(RFM69_INT,DEC);
  
  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    tft.println("RFM69 radio init failed");
    while (1);
  }
  tft.println("RFM69 radio init OK!");
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    tft.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  //              1234567890123456
  //uint8_t key[] ="Xyzabde123456789"; //exactly the same 16 characters/bytes on all nodes!

  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  tft.print("RFM69 radio @");  tft.print((int)RF69_FREQ,DEC);  tft.println(" MHz");

  // Test code
  test_sens_db();

  parse_msg("{\"Z\":\"Dock\",\"S\":\"Temp\",\"V\":997.00,\"R\":\"\"}");
  if( show_menu_millis > millis()){
    show_menu();
  }
  else {
    update_display();
  }
}


void loop(void) {
  
   // char buf [TEXT_VISIBLE_CHAR];
   // Serial.print(".");
   if (rf69.available()) {
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;
      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf69.lastRssi(), DEC);
      AddRow((char*)buf);
      parse_msg((char*)buf);
      //printMsgLog();
      if( show_menu_millis > millis()){
        //show_menu();
      }
      else {
        update_display();
      }

      if (strstr((char *)buf, "Hello World")) {
        // Send a reply!
        uint8_t data[] = "And hello back to you";
        rf69.send(data, sizeof(data));
        rf69.waitPacketSent();
        Serial.println("Sent a reply");
        Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks
      }
    } else {
      Serial.println("Receive failed");
    }
  }
  // The touchscreen part is still not utilized
  if (! ts.bufferEmpty()){
     TS_Point p = ts.getPoint();
     p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
     p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
     //if (ts.touched())
     Serial.print(p.x); Serial.print(" - ");Serial.println(p.y);
     show_menu_millis = millis() + 5000;
     menu_is_active = true;
     show_menu();
  }

  if( show_menu_millis > millis()){
        //show_menu();
  }
  else {
    if(menu_is_active)
      update_display();
      menu_is_active = false;
  }

   
  /*  if(millis()-print_all_millis > 2000){
      Serial.println("Now printing");
      printText();
      print_all_millis = millis();
    }
  */  
}
void show_menu(){
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ILI9341_RED); 
  tft.setFont(&FreeSerif24pt7b); 
  tft.setTextColor(ILI9341_LIGHTGREY);
  tft.println("Menu is under");
  tft.println("construction");
  
}

void AddRow( char *txt){
   uint8_t i;
   if(++insert_at > TEXT_VISIBLE_ROWS-1){
      insert_at = 0;      
   } 
   if (insert_at < TEXT_VISIBLE_ROWS){
      for (i=0;txt[i]!=0 && i< TEXT_VISIBLE_CHAR-1;i++){
         text_buffer[insert_at][i]=txt[i];
      }   
      text_buffer[insert_at][++i] = 0;
   }   
   
 
   Serial.println(txt);
}

void update_display(void){
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ILI9341_RED); 
  tft.setFont(&FreeSerif24pt7b); 
  tft.setTextColor(ILI9341_LIGHTGREY);
  tft.println("Villa Astrid");
  //setTextColor(uint16_t c, uint16_t bg)
  for (int i = 0; i < NBR_COLLECTED_SENSORS; i++){
    switch(i){
      case 0: tft.setTextColor(ILI9341_RED);
        break;
      case 1: tft.setTextColor(ILI9341_YELLOW);
        break;
      case 2: tft.setTextColor(ILI9341_CYAN);
        break;
        
    }
    tft.print(collect_sens[i].name);
    tft.print(" ");
    tft.println(collect_sens[i].value);  
  }
}

void printMsgLog(void){
  uint8_t i;
  uint8_t row;
  row = insert_at;
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);

  for(i=0;i< TEXT_VISIBLE_ROWS;i++){
    tft.println(text_buffer[row]);
    //Serial.println(text_buffer[row]);
    if(row == 0 ) {
      row = TEXT_VISIBLE_ROWS-1;
    }  
    else {
       row--;
    }   
  }
}

unsigned long testText() {
  tft.fillScreen(ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2);
  for (uint8_t row = 1;row < 100; row++){
     tft.print("Hello World! @ row ");
     tft.println(row);
  }
  
  return micros() - start;
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
