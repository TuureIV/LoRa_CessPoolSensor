#include <Arduino.h>
#include "heltec.h"
#include <esp_adc_cal.h>
#include <driver/adc.h>
#include "heltec.h"

#define BAND    868E6  //you can set band here directly,e.g. 868E6,915E6
#define Fbattery    3700 
#define echoPin GPIO_NUM_32 // to pin Echo of HC-SR04 //vihreä
#define trigPin GPIO_NUM_33 // to pin Trig of HC-SR04 //punainen

#define MAXBATT                 4200    // The default Lipo is 4200mv when the battery is fully charged.
#define LIGHT_SLEEP_VOLTAGE     3750    // Point where start light sleep
#define MINBATT                 3300    // The default Lipo is 3200mv when the battery is empty...this WILL be low on the 3.3v rail specs!!!

#define VOLTAGE_DIVIDER         3.20    // Lora has 220k/100k voltage divider so need to reverse that reduction via (220k+100k)/100k on vbat GPIO37 or ADC1_1 (early revs were GPIO13 or ADC2_4 but do NOT use with WiFi.begin())
#define DEFAULT_VREF            1100    // Default VREF use if no e-fuse calibration
#define VBATT_SAMPLE            500     // Battery sample rate in ms
#define VBATT_SMOOTH            50      // Number of averages in sample
#define ADC_READ_STABILIZE      5       // in ms (delay from GPIO control and ADC connections times)
#define LO_BATT_SLEEP_TIME      10*60*1000*1000     // How long when low batt to stay in sleep (us)
#define VBATT_GPIO              21      // Heltec GPIO to toggle VBatt read connection ... WARNING!!! This also connects VEXT to VCC=3.3v so be careful what is on header.  Also, take care NOT to have ADC read connection in OPEN DRAIN when GPIO goes HIGH

uint16_t Sample();
void DrawBattery(uint16_t, bool = false); 
long duration; // variable for the duration of sound wave travel
uint16_t voltage;
uint16_t v;
double pct = 100;


/*MESSAGE STUFF*/
String outgoing;              // outgoing message
byte localAddress = 0x01;     // address of this device
byte destination = 0x00;      // destination to send to
byte msgCount = 0;            // count of outgoing messages
long lastSendTime = 0;        // last send time
int interval = 5000;          // interval between sends
esp_adc_cal_characteristics_t *adc_chars;

/*Depth stuff*/
int depth = 0;
int sampleAmount = 30;

int GetDepth();
void SendMessage(String outgoing);
uint16_t ReadVBatt();
uint16_t Sample();
uint8_t GetPrecentage();
void drawBattery(uint16_t voltage, bool sleep);
void PrintToOled(String ms1,String ms4, String ms2, String ms3);

void setup()
{
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.LoRa Disable*/, true /*Serial Enable*/, false /*PABOOST Enable*/, BAND /*long BAND*/);

  /*Battery measuring setup*/
  adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1,ADC_ATTEN_DB_6);

  pinMode(VBATT_GPIO,OUTPUT);
  digitalWrite(VBATT_GPIO, LOW);              // ESP32 Lora v2.1 reads on GPIO37 when GPIO21 is low
  delay(ADC_READ_STABILIZE);     

  for (uint8_t i = 0;i < VBATT_SMOOTH;i++) {
    Sample();
  }

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
}

void loop()
{ 
  voltage = Sample();
  
  if (millis() - lastSendTime > interval){
  
    //depth = GetDepth();
    depth = random(200);
    Heltec.display->clear();
    PrintToOled("DEPTH: ",  String(depth) + "cm", "V: " + String(round(voltage/10.0)/100.0) +"v", "BAT: "+ String((int)round(pct))+"%");
    String message = ":"+ String(depth) + ":"+ String((round(voltage/10.0)/100.0)) + ":" +String((int)round(pct));   // send a message
    
    Serial.println("Sending->" + message);
    SendMessage(message);
    lastSendTime = millis();            // timestamp the message

    if (v < MINBATT) {                  // Low Voltage cut off shut down to protect battery as long as possible
      Heltec.display->setColor(WHITE);
      Heltec.display->setFont(ArialMT_Plain_10);
      Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
      Heltec.display->drawString(64,24,"Shutdown!!");
      Heltec.display->display();
      delay(2000);     
    }
  }
}

void PrintToOled(String ms1,String ms4, String ms2, String ms3) { // PrintToOled("","","","","");
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_24);
  Heltec.display->drawString(0,12, ms1);
  Heltec.display->drawString(0,35, ms4);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0,0, ms2);
  Heltec.display->drawString(42,0, ms3);
  drawBattery(voltage, voltage < LIGHT_SLEEP_VOLTAGE);
  Heltec.display->display();
}

void drawBattery(uint16_t voltage, bool sleep) {
  Heltec.display->setColor(BLACK);
  Heltec.display->fillRect(99,0,29,24);

  Heltec.display->setColor(WHITE);
  Heltec.display->drawRect(100,4,18,8);
  Heltec.display->fillRect(118,6,2,4);

  v = voltage;
  if (v < MINBATT) {v = MINBATT;}
  if (v > MAXBATT) {v = MAXBATT;}
  pct = GetPrecentage();
  uint8_t bars = round(pct / 5.5555);
  Heltec.display->fillRect(100,4,bars,8);
}

uint8_t GetPrecentage(){
  return 2808.3808 * pow(round(voltage/10.0)/100.0, 4) - 43560.9157 * pow(round(voltage/10.0)/100.0, 3)
         + 252848.5888 * pow(round(voltage/10.0)/100.0, 2) - 650767.4615 * round(voltage/10.0)/100.0 + 626532.5703;
}

uint16_t Sample() {
  static uint8_t i = 0;
  static uint16_t samp[VBATT_SMOOTH];
  static int32_t t = 0;
  static bool f = true;
  if(f){ for(uint8_t c=0;c<VBATT_SMOOTH;c++){ samp[c]=0; } f=false; }   // Initialize the sample array first time
  t -= samp[i];   // doing a rolling recording, so remove the old rolled around value out of total and get ready to put new one in.
  if (t<0) {t = 0;}
  // ADC read
  uint16_t voltage = ReadVBatt();
  samp[i]=voltage;
  t += samp[i];
  if(++i >= VBATT_SMOOTH) {i=0;}
  uint16_t s = round(((float)t / (float)VBATT_SMOOTH));

  return s;
}

uint16_t ReadVBatt() {
  uint16_t reading = 666;

  digitalWrite(VBATT_GPIO, LOW);              // ESP32 Lora v2.1 reads on GPIO37 when GPIO21 is low
  delay(ADC_READ_STABILIZE);                  // let GPIO stabilize
  pinMode(ADC1_CHANNEL_1, OPEN_DRAIN);        // ADC GPIO37
  reading = adc1_get_raw(ADC1_CHANNEL_1);
  pinMode(ADC1_CHANNEL_1, INPUT);             // Disconnect ADC before GPIO goes back high so we protect ADC from direct connect to VBATT (i.e. no divider)
  uint16_t voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);  
  voltage*=VOLTAGE_DIVIDER;

  return voltage;
}

int GetDepth() {
  int measured_distance = 0;
  int samples = 0;
  for (size_t i = 0; i < sampleAmount; i++)
  {
     // Clears the trigPin condition
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    measured_distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    samples += measured_distance;
  }
  return measured_distance ; //= samples/30
}

void SendMessage(String outgoing){
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}