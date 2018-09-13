// This #include statement was automatically added by the Particle IDE.
#include <I2Cdev.h>

// This #include statement was automatically added by the Particle IDE.
#include <neopixel.h>

#include <Artnet.h>

#include <I2Cdev.h>
#include <MPU6050.h>

#if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE) && !defined (PARTICLE)
    #include "Wire.h"
#endif

// Neopixel settings
const int numLeds = 5; // change for your setup
const int numberOfChannels = numLeds * 3; // Total number of channels you want to receive (1 led = 3 channels)
const byte dataPin = 5;
Adafruit_NeoPixel leds = Adafruit_NeoPixel(numLeds, dataPin, WS2812);

void rainbow(uint8_t wait);
uint32_t Wheel(byte WheelPos);

// Artnet settings
Artnet artnet;
const int startUniverse = 0; // CHANGE FOR YOUR SETUP most software this is 1, some software send out artnet first universe as 0.

// Check if we got all universes
const int maxUniverses = numberOfChannels / 512 + ((numberOfChannels % 512) ? 1 : 0);
bool universesReceived[maxUniverses];
bool sendFrame = 1;
int previousDataLength = 0;

MPU6050 accelgyro1(0x68);
//MPU6050 accelgyro2(0x69); // <-- use for AD0 high //or for second MPU

int16_t ax1, ay1, az1;
int16_t gx1, gy1, gz1;

int16_t ax1_prev, ay1_prev, az1_prev;
int16_t gx1_prev, gy1_prev, gz1_prev;

#define OUTPUT_READABLE_ACCELGYRO

#if defined (PARTICLE)
#define LED_PIN D7 // (Particle is D7)
#else
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#endif

bool blinkState = false;

const int gyroSetting1 = 2;   //use hand shake also about less than 1k
const int accelSetting1 = 0;

const float gravity = 9.80665;
//1 g = 9.80665 m/s2

unsigned long sampleTime = 0;
unsigned long prevTime = 0;
float dt;

float gyroAngX1 = 0;
float gyroAngY1 = 0;
float gyroAngZ1 = 0;

//------For Gyro speed modes----------------------
unsigned long startTime = 0;
unsigned int mode = 0;

unsigned int lowbrightness = 30;
unsigned int highbrightness = 255;

int speedThresh = 150;

unsigned long seqDuration = 5000;   //how long the brightness will stay increased.
//------------------------------------------------

//-------For Artnet detection---------------------
unsigned long startTimerNoArt = 0;
unsigned int statusArtNet = 0;
unsigned long noArtDuration = 5000; //if no Artnet detected beyond this duration, the sequence will be triggered
//------------------------------------------------

String myIDStr = String(Spark.deviceID());

void updateGyroRead()
{
    accelgyro1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);

    dt = 1/1000.0;  //divided by 1000 to convert to seconds from millisecond. We already know dt will be 1 millisecond since we are using timer.
    
    #ifdef OUTPUT_READABLE_ACCELGYRO
    /*
        Serial.print("gyro #1 in deg:  \t");
        Serial.print(gyroAngX1+=convert2deg(gx1,1)); Serial.print(" deg"); Serial.print("\t");
        Serial.print(gyroAngY1+=convert2deg(gy1,1)); Serial.print(" deg"); Serial.print("\t");
        Serial.print(gyroAngZ1+=convert2deg(gz1,1)); Serial.println(" deg");
    */    
        Serial.print("gyro #1 in deg/s:  \t");
        //Serial.print(gyroAngX1=convert2degS(gx1,1)); Serial.print(" deg/s"); Serial.print("\t");
        //Serial.print(gyroAngY1=convert2degS(gy1,1)); Serial.print(" deg/s"); Serial.print("\t");
        Serial.print(gyroAngZ1=convert2degS(gz1,1)); Serial.println(" deg/s");
    #endif
    
    switch (mode){
        case 0:
            if (gyroAngZ1>speedThresh||gyroAngZ1<(-1*speedThresh)){
                leds.setBrightness(highbrightness);
                leds.show();
                startTime = millis();
                mode = 1;
            }
            break;
        case 1:
            if ((millis()-startTime)>seqDuration){
                leds.setBrightness(lowbrightness);
                leds.show();
                startTime = 0;
                mode = 0;
            }
            break;
        default:
            break;
    }
    Serial.println("Mode: " + mode);
    
    ax1_prev, ay1_prev, az1_prev = ax1, ay1, az1;
    gx1_prev, gy1_prev, gz1_prev = gx1, gy1, gz1;
}

Timer gyroTimer(1, updateGyroRead);     //dt is now always 1 millisecond


void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif    
    
    Serial.begin(38400);
    
    Serial.println("Initializing I2C devices...");
    accelgyro1.initialize();
    
    accelgyro1.setFullScaleGyroRange(gyroSetting1); //0 = +/- 250 degrees/sec | 1 = +/- 500 degrees/sec | 2 = +/- 1000 degrees/sec | 3 =  +/- 2000 degrees/sec
                                                    //0 = 131 LSB/deg/sec | 1 = 65.5 LSB/deg/sec | 2 = 32.8 LSB/deg/sec | 3 = 16.4 LSB/deg/sec
    accelgyro1.setFullScaleAccelRange(accelSetting1);  //0 = +/- 2g | 1 = +/- 4g | 2 = +/- 8g | 3 =  +/- 16g
                                          //0 = 16,384 LSB/g | 1 = 8,192 LSB/g | 2 = 4,096 LSB/g | 3 = 2,048 LSB/g 
    
    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro1.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    accelgyro1.setXGyroOffset(40);
    accelgyro1.setYGyroOffset(-30);
    accelgyro1.setZGyroOffset(70);

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    
    artnet.begin();//mac, ip);
    leds.begin();
    initTest();
    
    // this will be called for each packet received
    artnet.setArtDmxCallback(onDmxFrame);
    Serial.println(WiFi.localIP());
    
    leds.setBrightness(lowbrightness);
    leds.show();
    
    gyroTimer.start();
}

void loop()
{   
    //Serial.println(Spark.deviceID());
    Serial.println(WiFi.localIP());
    
    if (WiFi.ready()){
        Serial.println("Wifi Connected");
        
        switch(statusArtNet){
            case 0:   //initial state
                if (artnet.read() == ART_DMX){
                    // print out our data
                    Serial.println("Artnet Detected");
                }
                else {
                    Serial.println("Artnet not available. Timer Start.");
                    statusArtNet = 1;
                    startTimerNoArt = millis();
                }
                break;
                
            case 1:   //Artnet not detected
                if (artnet.read() == ART_DMX){
                    // print out our data
                    Serial.println("Artnet Detected. Reset timer");
                    //Since Artnet was detected, we will reset the timer and resume normal operation
                    statusArtNet = 0;
                    startTimerNoArt = 0;
                }
                else {
                    //Art Net Still Not available. Check timer.
                    if ((millis()-startTimerNoArt)>noArtDuration){
                        Serial.println("Timer Triggered.");
                        //Sequence here.
                        Serial.println("Seq Start!");
                        //rainbow(30);  //Sequence here.
                        colorWipe(leds.Color(255, 0, 0), 50); // Red
                        colorWipe(leds.Color(0, 255, 0), 50); // Green
                        colorWipe(leds.Color(0, 0, 255), 50); // Blue
                        //theaterChaseRainbow(50);
                        //delay(2000);
                        Serial.println("Seq End");
                        delay(500);         //SHort delay only for visibility on Serial Monitor
                        if (artnet.read() == ART_DMX){
                            Serial.println("ArtNet Detected. Resuming normal ArtNet read.");
                            startTimerNoArt = 0;
                            statusArtNet = 0;
                        }
                    }
                    else{
                        Serial.println("Artnet still not available");
                    }
                }
                break;
            default:
                break;
        }
        
        artnet.read();

    }
    else{
        Serial.println("Wifi Disconnected");
        rainbowNoWifi(30);  //Sequence here.
    }
    
}

void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data)
{
  sendFrame = 1;
  // set brightness of the whole strip 
  if (universe == 15)
  {
    //leds.setBrightness(data[0]);
    //leds.show();
  }

  // Store which universe has got in
  if ((universe - startUniverse) < maxUniverses)
    universesReceived[universe - startUniverse] = 1;

  for (int i = 0 ; i < maxUniverses ; i++)
  {
    if (universesReceived[i] == 0)
    {
      //Serial.println("Broke");
      sendFrame = 0;
      break;
    }
  }

  // read universe and put into the right part of the display buffer
  for (int i = 0; i < length / 3; i++)
  {
    int led = i + (universe - startUniverse) * (previousDataLength / 3);
    if (led < numLeds)
      leds.setPixelColor(led, data[i * 3], data[i * 3 + 1], data[i * 3 + 2]);
  }
  previousDataLength = length;     
  
  if (sendFrame)
  {
    leds.show();
    // Reset universeReceived to 0
    memset(universesReceived, 0, maxUniverses);
  }
}

void initTest()
{
  for (int i = 0 ; i < numLeds ; i++)
    leds.setPixelColor(i, 127, 0, 0);
  leds.show();
  delay(500);
  for (int i = 0 ; i < numLeds ; i++)
    leds.setPixelColor(i, 0, 127, 0);
  leds.show();
  delay(500);
  for (int i = 0 ; i < numLeds ; i++)
    leds.setPixelColor(i, 0, 0, 127);
  leds.show();
  delay(500);
  for (int i = 0 ; i < numLeds ; i++)
    leds.setPixelColor(i, 0, 0, 0);
  leds.show();
}

float convert2g(float accel,int sensNum){
   //0 = 16,384 LSB/g | 1 = 8,192 LSB/g | 2 = 4,096 LSB/g | 3 = 2,048 LSB/g 
  int settingAccel;
  if (sensNum==1){
    settingAccel = accelSetting1;
  }
  //else {
    //settingAccel = accelSetting2;
  //}
  
  switch (settingAccel){
    case 0: 
      accel = (float) accel;
      accel = accel/16384;
      return accel;
      break;

    case 1:
      accel = (float) accel;
      accel = accel/8192;
      return accel;
      break;

    case 2:
      accel = (float) accel;
      accel = accel/4096;
      return accel;
      break;

    case 3:
      accel = (float) accel;
      accel = accel/2048;
      return accel;
      break;

    default:
      accel = (float) accel;
      accel = accel/16384;
      return accel;
      break;
  }

}

float convert2ms2(float accel,int sensNum){
   //0 = 16,384 LSB/g | 1 = 8,192 LSB/g | 2 = 4,096 LSB/g | 3 = 2,048 LSB/g
  int settingAccel;
   
  if (sensNum==1){
    settingAccel = accelSetting1;
  }
  //else {
    //settingAccel = accelSetting2;
  //}
  accel = (float) accel;
  switch (settingAccel){
    case 0: 
      
      accel = accel/16384;
      return accel*gravity;
      break;

    case 1:
      
      accel = accel/8192;
      return accel;
      break;

    case 2:
      
      accel = accel/4096;
      return accel*gravity;
      break;

    case 3:
      
      accel = accel/2048;
     return accel*gravity;
      break;

    default:
      Serial.println("Accel defaulted");
      accel = accel/16384;
      return accel*gravity;
      break;
  }
}

float convert2degS(float gyro,int sensNum){
  //0 = +/- 250 degrees/sec | 1 = +/- 500 degrees/sec | 2 = +/- 1000 degrees/sec | 3 =  +/- 2000 degrees/sec
  //0 = 131 LSB/deg/sec | 1 = 65.5 LSB/deg/sec | 2 = 32.8 LSB/deg/sec | 3 = 16.4 LSB/deg/sec
  int settingGyro;
   
  if (sensNum==1){
    settingGyro = gyroSetting1;
  }
  //else {
    //settingGyro = gyroSetting2;
  //}

  gyro = (float) gyro;
  
  switch (settingGyro){
    case 0: 
      gyro = gyro/131;
      return gyro;
      break;

    case 1:
      gyro = gyro/65.5;
      return gyro;
      break;

    case 2:
      gyro = gyro/32.8;
      return gyro;
      break;

    case 3:
      gyro = gyro/16.4;
      return gyro;
      break;

    default:
      Serial.println("Gyro defaulted");
      gyro = gyro/131;
      return gyro;
      break;
  }
}
float convert2deg(float gyro,int sensNum){
  //0 = +/- 250 degrees/sec | 1 = +/- 500 degrees/sec | 2 = +/- 1000 degrees/sec | 3 =  +/- 2000 degrees/sec
  //0 = 131 LSB/deg/sec | 1 = 65.5 LSB/deg/sec | 2 = 32.8 LSB/deg/sec | 3 = 16.4 LSB/deg/sec
  int settingGyro;
   
  if (sensNum==1){
    settingGyro = gyroSetting1;
  }
  //else {
    //settingGyro = gyroSetting2;
  //}

  gyro = (float) gyro;
  float Theta;
  
  switch (settingGyro){
    case 0: 
      Theta = (gyro*dt)/131.0;
      return Theta;
      break;

    case 1:
      Theta = (gyro*dt)/65.5;
      return Theta;
      break;

    case 2:
      Theta = (gyro*dt)/32.8;
      return Theta;
      break;

    case 3:
      Theta = (gyro*dt)/16.4;
      return Theta;
      break;

    default:
      Serial.println("Gyro defaulted");
      Theta = (gyro*dt)/131.0;
      return Theta;
      break;
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<leds.numPixels(); i++) {
      leds.setPixelColor(i, Wheel((i+j) & 255));
    }
    leds.show();
    delay(wait);
  }
}
void rainbowNoWifi(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<leds.numPixels(); i++) {
        if (WiFi.ready()){
            return;
        }
      leds.setPixelColor(i, Wheel((i+j) & 255));
    }
    leds.show();
    delay(wait);
  }
}

void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < leds.numPixels(); i=i+3) {
        leds.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      leds.show();

      delay(wait);

      for (uint16_t i=0; i < leds.numPixels(); i=i+3) {
        leds.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

void theaterChaseRainbowNoWifi(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < leds.numPixels(); i=i+3) {
        if (WiFi.ready()){
            return;
        }
        leds.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      leds.show();

      delay(wait);

      for (uint16_t i=0; i < leds.numPixels(); i=i+3) {
        if (WiFi.ready()){
            return;
        }
        leds.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<leds.numPixels(); i++) {
    leds.setPixelColor(i, c);
    leds.show();
    delay(wait);
  }
}

void colorWipeNoWifi(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<leds.numPixels(); i++) {
    if (WiFi.ready()){
        return;
    }
    leds.setPixelColor(i, c);
    leds.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return leds.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return leds.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return leds.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}