/*
Simple Deep Sleep with Timer Wake Up
=====================================
ESP32 offers a deep sleep mode for effective power
saving as power is an important factor for IoT
applications. In this mode CPUs, most of the RAM,
and all the digital peripherals which are clocked
from APB_CLK are powered off. The only parts of
the chip which can still be powered on are:
RTC controller, RTC peripherals ,and RTC memories

This code displays the most basic deep sleep with
a timer to wake it up and how to store data in
RTC memory to use it over reboots

This code is under Public Domain License.

Author:
Pranav Cherukupalli <cherukupallip@gmail.com>
*/
#include <Wire.h>
#include "Ambient.h"
#include "driver/adc.h" 
#include "esp_adc_cal.h"

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) */

//Measured data
float temperature=0;
float humidity=0;
float pressure=0;
float voltage=0;


//Deepsleep
RTC_DATA_ATTR int bootCount = 0;

//---WIFI and Ambient
WiFiClient client;
Ambient ambient;
const char* ssid = "YOUR SSID";
const char* password = "YOUR PASSWORD";
unsigned int channelId = 00000; // Ambient channel ID <= Put your channel ID
const char* writeKey = "YOUR WRITEKEY"; // WriteKey


//---LCD
unsigned char LCD_AQM0802_ADDR = 0x3E;

//---SHT31
unsigned char SHT31_ADDR = 0x45;
unsigned char SOFT_RESET_MSB = 0x30;
unsigned char SOFT_RESET_LSB = 0xA2;
unsigned char CLEAR_STATUS_REGISTER_MSB = 0x30;
unsigned char CLEAR_STATUS_REGISTER_LSB = 0x41;
unsigned char SINGLE_SHOT_HIGH_MSB = 0x24;
unsigned char SINGLE_SHOT_HIGH_LSB = 0x00;

//---BM1383
unsigned char BM1383_ADDR = 0x5D;

//---To check power voltage
//const int adcPin = 33; //IO33 pin13 ADC1_5
const int adcPin = 36; //IO36 SENSOR=VP ADC1_CH0
const int swPin = 26;  //IO26 pin15 GPIO26
const int R1 = 10000;
const int R2 = 10000;

//---Button
const int button1Pin = 27; //IO27
const int button2Pin = 32; //IO32
volatile unsigned long time_prev1 = 0, time_now1;
volatile unsigned long time_prev2 = 0, time_now2;
unsigned long time_chat = 20;
bool button1wasPressed = false;
bool button2wasPressed = false;


//WIFI related functions---------
int init_wifi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
    }
    Serial.print("WiFi connected\r\nIP address: ");
    Serial.println(WiFi.localIP());
    return 0;
}

int init_ambient(){
    ambient.begin(channelId, writeKey, &client); 
    return 0;
}

int sendto_ambient(){
    ambient.set(1, String(temperature).c_str());
    ambient.set(2, String(humidity).c_str());
    ambient.set(3, String(pressure).c_str());
    ambient.set(4, String(voltage).c_str());
    if((button1wasPressed==true) && (button2wasPressed==false))ambient.set(5, "1");
    else ambient.set(5, "0");
    if((button1wasPressed==true) && (button2wasPressed==true))ambient.set(6, "1");
    else ambient.set(6, "0");
    ambient.send();
    return 0;
}
//WIFI related functions---------

//Button interrupt related functions---------
int init_ButtonInput() {
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  attachInterrupt(button1Pin, Button1Input, FALLING);
  attachInterrupt(button2Pin, Button2Input, FALLING);
  return 0;
}

void Button1Input() {   //Consider about chattering.
  time_now1 = millis(); 
  if( time_now1-time_prev1 > time_chat){
      Serial.println("Button1");
      button1wasPressed=true;
  }
  time_prev1 = time_now1; 
}

void Button2Input() {   //Consider about chattering.
  time_now2 = millis(); 
  if( time_now2-time_prev2 > time_chat){
      Serial.println("Button2");
      button2wasPressed=true;
  }
  time_prev2 = time_now2; 
}
//Button interrupt related functions---------

//Voltage check related functions---------
esp_adc_cal_characteristics_t adcChar;
#define ADC_VREF        1100            // ADC calibration data

int init_VoltageCheck() {
  pinMode(swPin, OUTPUT);
  digitalWrite(swPin, HIGH);

  adc_power_on();
  // IO33=ADC1_CH5
  adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_0);
  // Set ADC1 12bit(0~4095)
  adc1_config_width(ADC_WIDTH_BIT_12);
    // Set ADC1 11dB
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, ADC_VREF, &adcChar);
  return 0;
}

int VoltageCheck() {
  uint32_t voltage_tmp=0;

  digitalWrite(swPin, LOW);
  delay(500);

    esp_adc_cal_get_voltage(ADC_CHANNEL_0, &adcChar, &voltage_tmp);
    voltage=voltage_tmp/500; //voltage_tmp/1000 * 2
    Serial.println("Voltage: " + String(voltage) + "[V]");
   digitalWrite(swPin, HIGH);

  return 0;
}

int VoltageCheck_analogread() {
  float tmp_value[5];
  float value=0;
  int i;

  digitalWrite(swPin, LOW);
  delay(500);
  
  for(i=0;i<5;i++) {
    tmp_value[i]= analogRead(adcPin);
    value+=tmp_value[i];
    delay(10);
  }
  value=value/5;
  
  Serial.print("Voltage ");
  digitalWrite(swPin, HIGH);

  voltage = value * (R1+R2) / R2 *(3.3/4095) + 0.06; //0.06:correction around 0v

  Serial.print("ADC:");
  Serial.print(voltage);
  Serial.print(" Value:");
  Serial.println(value);

  return 0;
}
//Voltage check related functions---------

//BM1383 related functions---------
int init_bm1383() {
  int value;
  
  Wire.beginTransmission(BM1383_ADDR);  //Power ON
  Wire.write(0x12);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(BM1383_ADDR); // Reset Release
  Wire.write(0x13);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(10);
  
  Wire.beginTransmission(BM1383_ADDR); // average 16,continue
  Wire.write(0x14);
  Wire.write(0x8a);
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(BM1383_ADDR);
  Wire.write(0x12);
  Wire.endTransmission();
  Wire.requestFrom(BM1383_ADDR, 1);
  value=Wire.read();
  Wire.endTransmission(); 
//  Serial.print("BM1383 0x12=1:OK --- ? ");
//  Serial.println(value);
  if(value!=1) return -1;
  
  Wire.beginTransmission(BM1383_ADDR);
  Wire.write(0x13);
  Wire.endTransmission();
  Wire.requestFrom(BM1383_ADDR, 1);
  value=Wire.read();
  Wire.endTransmission(); 
//  Serial.print("BM1383 0x13=1:OK --- ? ");
//  Serial.println(value);
  if(value!=1) return -1;
  else return 0;
}

int BM1383Check() {
  unsigned int tmp_value[3];

  Wire.beginTransmission(BM1383_ADDR);
  Wire.write(0x1a);
  Wire.endTransmission();
  Wire.requestFrom(BM1383_ADDR, 3);
  tmp_value[0]=Wire.read();
  tmp_value[1]=Wire.read();
  tmp_value[2]=Wire.read();
  Wire.endTransmission();
  pressure = (tmp_value[0]*16384 + tmp_value[1]*64 + (tmp_value[2]>>2)) /2048.0;
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println("hPa");

  return 0;
}
//BM1383 related functions---------


//SHT31 related functions----------
int init_sht31() {
  Wire.beginTransmission(SHT31_ADDR);
  Wire.write(SOFT_RESET_MSB);
  Wire.write(SOFT_RESET_LSB);
  Wire.endTransmission();
  delay(500);

  Wire.beginTransmission(SHT31_ADDR);
  Wire.write(CLEAR_STATUS_REGISTER_MSB);
  Wire.write(CLEAR_STATUS_REGISTER_LSB);
  Wire.endTransmission();
  delay(500);
}

int SHT31Check(){
  unsigned int tmp_value[6];
  unsigned int i, t ,h;

  Wire.beginTransmission(SHT31_ADDR);
  Wire.write(SINGLE_SHOT_HIGH_MSB);
  Wire.write(SINGLE_SHOT_HIGH_LSB);
  Wire.endTransmission();
  delay(300);

  Wire.requestFrom(SHT31_ADDR, 6);
  for (i=0; i<6; i++){
      tmp_value[i] = Wire.read();
  }
  Wire.endTransmission();

  t = (tmp_value[0] << 8) | tmp_value[1];
  temperature = (float)(t) * 175 / 65535.0 - 45.0;
  h = (tmp_value[3] << 8) | tmp_value[4];
  humidity = (float)(h) / 65535.0 * 100.0;
  
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("C");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  return 0;
}
//SHT31 related functions----------

//I2C LCD related functions--------
int i2cwritecmd_lcd(byte cmd) {
    Wire.beginTransmission(LCD_AQM0802_ADDR);
    Wire.write(0x00); Wire.write(cmd);
    return Wire.endTransmission();
}
 
int i2cwritedata_lcd(byte data) {
    Wire.beginTransmission(LCD_AQM0802_ADDR);
    Wire.write(0x40);
    Wire.write(data);
    return Wire.endTransmission();
}
 
void lcdcu_set(int x, int y) {
    byte ca = (x + y * 0x40) | (0x80); i2cwritecmd_lcd(ca);
}
 
void lcdclear() {
    i2cwritecmd_lcd(0x01); delay(1);
}
 
void lcdhome() {
    i2cwritecmd_lcd(0x02); delay(1);
}
 
void init_lcd() {
    delay(145);
    i2cwritecmd_lcd(0x38); delay(1);
    i2cwritecmd_lcd(0x39); delay(1);
    i2cwritecmd_lcd(0x14); delay(1);
    i2cwritecmd_lcd(0x70); delay(1);
    i2cwritecmd_lcd(0x56); delay(2);
    i2cwritecmd_lcd(0x6C); delay(300);
    i2cwritecmd_lcd(0x38); delay(1);
    i2cwritecmd_lcd(0x0C); delay(2);
    i2cwritecmd_lcd(0x01); delay(2);
    delay(500);
}
 
void i2cprint_lcd( String pdata) {
    int n = pdata.length();
    for (int i = 0; i < n; i = i + 1) {
        i2cwritedata_lcd(pdata.charAt(i));
    delay(1);
    }
}
//I2C LCD related functions--------

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : 
          Serial.println("Wakeup caused by external signal using RTC_IO"); 
          button1wasPressed=true;
          break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void Wait_button2(){
  int i=0;
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : 
      if(button2wasPressed==false){
        Serial.println("Waiting for button2 for 10 sec."); 
        for(i=0;i<100;i++){
          if(button2wasPressed==true) i=100;
          delay(100);
        }
        Serial.println("Waiting was finished."); 
      }
      break;
    default : break;
  }
}

void setup(){  
  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_27,0); //1 = High, 0 = Low
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");
  
  init_ButtonInput();
  
  init_wifi();
  init_ambient();
 
  init_VoltageCheck();
  
  Wire.begin();
  init_lcd();
  init_sht31();
  init_bm1383();

//--Voltage check
//  VoltageCheck();
  VoltageCheck_analogread();

//--SHT31---
  SHT31Check();

//--BM1383---
  BM1383Check();
  
//--LCD---
  lcdclear();
  lcdhome();
  i2cprint_lcd(String(voltage));
  i2cprint_lcd("vcc ");
  delay(1000);

  lcdclear();
  lcdhome();
  i2cprint_lcd(String(temperature));
  i2cprint_lcd("C ");
  lcdcu_set(0,1);
  i2cprint_lcd(String(humidity));
  i2cprint_lcd("%");
  delay(1000);
    
  lcdclear();
  lcdhome();
  i2cprint_lcd(String(pressure));
  lcdcu_set(0,1);
  i2cprint_lcd("hPa ");
  if(button1wasPressed==true) i2cprint_lcd("b1");

  Wait_button2();
  if(button2wasPressed==true) i2cprint_lcd("b2");

  sendto_ambient();

  /*
  Next we decide what all peripherals to shut down/keep on
  By default, ESP32 will automatically power down the peripherals
  not needed by the wakeup source, but if you want to be a poweruser
  this is for you. Read in detail at the API docs
  http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
  Left the line commented as an example of how to configure peripherals.
  The line below turns off all RTC peripherals in deep sleep.
  */
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  /*
  Now that we have setup a wake cause and if needed setup the
  peripherals state in deep sleep, we can now start going to
  deep sleep.
  In the case that no wake up sources were provided but deep
  sleep was started, it will sleep forever unless hardware
  reset occurs.
  */
  button1wasPressed=false;
  button2wasPressed=false;
  Serial.println("Going to sleep now");
  Serial.flush(); 
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void loop(){
  //This is not going to be called
}
