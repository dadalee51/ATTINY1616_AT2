#include <Arduino.h>
/*** AT2 - master 0x17
 AT2 of DryBot v2.3a, may 10 2024.
 */
#include <Wire.h>
#define TF_OFF
#define ACC_OFF
#define CLR_OFF

#ifdef ACC_ON
  #include "SparkFun_LIS2DH12.h"
  SPARKFUN_LIS2DH12 accel;
#endif
#ifdef TF_ON
  #include <VL53L0X.h>
  VL53L0X sensor; //0x29
#endif
#ifdef CLR_ON
  #include "veml6040.h"
  VEML6040 RGBWSensor;
#endif

#define FOR(I,N) for(int I=0;I<N;I++)
#define PA0 17 //UPDI
#define PA1 14 //MOSI
#define PA2 15 //MISO
#define PA3 16 //CLK
#define PA4 0  //SS
#define PA5 1  
#define PA6 2  //DAC
#define PA7 3

#define PB0 9 //SCL
#define PB1 8 //SDA
#define PB2 7 //TXD
#define PB3 6 //RXD
#define PB4 5 
#define PB5 4 

#define PC0 10
#define PC1 11
#define PC2 12
#define PC3 13

//int SEN13 = PC0; //LDR
int SEN13 = PC0;
int SEN14 = PC1;
int SEN1  = PC2; //IR sensor
int SEN2  = PC3;
int SEN3  = PB2;
int MB1 = PB3;
int MB2 = PA3;
int MC1 = PA4;
int MC2 = PA5;
int MD1 = PB4;
int MD2 = PB5;
int ENC_MD_A = PA6;
int ENC_MD_B = PA7;

//pwm pins: pb3,4,5, pa3,4,5, pc0/1 : please choose correct settings.

int WLED2 = PA2; //dryBot LEDW 0 = off
int RLED2 = PA1;

#ifdef TF_ON
  int head=0;
#endif
#ifdef ACC_ON
  float z_acc=0.0;
#endif
#ifdef CLR_ON
  int red=0; int mred=0;
  int blue=0; int mblue=0; 
  int green=0; int mgreen=0;
#endif

void TCA9548A(uint8_t bus);
void drive_motor(int,int,int,int);
void signalling(int);
void to_RGB(long color);
void to_MotorA(int dir, int speed);
void debugData(long val);
/*** Wire interface **********************************************/
#define MASTER_ADDRESS 0x17 
#define SLAVE_ADDRESS 0x12 
#define BUFFER_SIZE 20 
char receivedData[BUFFER_SIZE]; 
int dataLength = 0; 
//master send
void receiveData(int numBytes) {
  dataLength = numBytes;
  Wire.readBytes(receivedData, numBytes);
}
//master read
void sendData() {
  Wire.write(receivedData, dataLength);
}


void setup() {

  pinMode(RLED2,OUTPUT);
  pinMode(WLED2,OUTPUT);
  pinMode(ENC_MD_A,INPUT);
  pinMode(ENC_MD_B,INPUT);
  pinMode(MB1, OUTPUT); 
  pinMode(MB2, OUTPUT);
  pinMode(MC1, OUTPUT); 
  pinMode(MC2, OUTPUT);
  pinMode(MD1, OUTPUT); 
  pinMode(MD2, OUTPUT);
  pinMode(SEN13, INPUT);
  pinMode(SEN14, INPUT);
  pinMode(SEN1, INPUT);
  pinMode(SEN2, INPUT);
  pinMode(SEN3, INPUT);
  
  // ADC1.CTRLA = ADC_ENABLE_bm;
  // ADC1.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV256_gc;
  // ADC1.MUXPOS = ADC_MUXPOS_AIN0_gc;

// #ifdef ADC1
// FOR(i,10){
//   digitalWrite(WLED2,1);
//   delay(30);
//   digitalWrite(WLED2,0);
//   delay(30);
// }
// #endif

  //Wire.begin(SLAVE_ADDRESS); // join i2c bus as slave
  Wire.begin(); // join i2c bus as master
  //Switch colour sensor
  //TCA9548A(1);
  //Wire.onReceive(receiveData); // callback for receiving data
  //Wire.onRequest(sendData); // callback for sending data

  digitalWrite(RLED2, 1); // 0 on, 1 off
  digitalWrite(WLED2, 0); // 1 on, 0 off
  //digitalWrite(MD1, 0); // 0 on, 1 off
  //digitalWrite(MD2, 0); // 0 on, 1 off
  #ifdef TF_ON
  sensor.setTimeout(500);
  if (!sensor.init()) {
    FOR(k,3){
    signalling(30);
    delay(1000);
    }
  }
  sensor.startContinuous();
  #endif
  
  #ifdef ACC_ON
  if (!accel.begin()) {
    signalling(30);
    delay(100);
  }
  #endif

  #ifdef CLR_ON
    if(!RGBWSensor.begin()) {
      FOR(i,5){
        signalling(30);
        delay(200);
      }
    }
  #endif

  #ifdef CLR_ON
  //switch to first clr sensr
  //TCA9548A(0);
  RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  delay(500);
  FOR(i,5){
    if(!RGBWSensor.begin()) {
      signalling(30);
      delay(1000);
    }
  }
  //switch to first clr sensr
  // TCA9548A(1);
  // //RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  // delay(500);
  // FOR(i,5){
  //   if(!RGBWSensor.begin()) {
  //     signalling(30);
  //     delay(1000);
  //   }
  // }
  #endif
  randomSeed(1450);
}
// arduino long type has 4 bytes, 0xFFFFFFFF, signed. ranged -2,147,483,648 to 2,147483,647
void loop() {  
  delay(100);
  to_RGB( random(0xAAAAAA)); //RGB proof i2c works
  delay(100);
  #ifdef TF_ON
  head=sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) FOR(k,3)signalling(50);
  if(head > 300){
    digitalWrite(WLED2,1); //turn on
    to_WLED1(1);
  }else{
    digitalWrite(WLED2,0);
    to_WLED1(0);
  }
  #endif

  #ifdef ACC_ON
  z_acc = accel.getZ();
  if (z_acc < 0){
    digitalWrite(WLED1, 1); //on
  }else{
    digitalWrite(WLED1, 0);//Wled off
  }
  #endif
  #ifdef CLR_ON
    delay(40);
    TCA9548A(0);
    delay(40);
    //RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_TRIG_ENABLE + VEML6040_AF_FORCE + VEML6040_SD_ENABLE);
    red = RGBWSensor.getRed();
    green = RGBWSensor.getGreen();
    blue = RGBWSensor.getBlue();
    mred = map(red, 0,30000,254,0); //red is very sensitive, 200 to 900, was already a high amount, how to lower?
    mgreen = map(green, 0,1000,254,0);
    mblue = map(blue, 0,700,254,0); //the lower the range, more amplify
    analogWrite(RGB_R, mred);
    analogWrite(RGB_B, mblue);
    analogWrite(RGB_G, mgreen);
    delay(40);
    // delay(40);
    // TCA9548A(1);
    // delay(40);
    // RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_TRIG_ENABLE + VEML6040_AF_FORCE + VEML6040_SD_ENABLE);
    // red = RGBWSensor.getRed();
    // green = RGBWSensor.getGreen();
    // blue = RGBWSensor.getBlue();
    // mred = map(red, 0,5000,254,0); //red is very sensitive, 200 to 900, was already a high amount, how to lower?
    // mgreen = map(green, 0,800,254,0);
    // mblue = map(blue, 0,700,254,0); //the lower the range, more amplify
    // analogWrite(RGB_R, mred);
    // analogWrite(RGB_B, mblue);
    // analogWrite(RGB_G, mgreen);
  #endif 
}

/*
 * power = mimic lego's design: +/-255
 * example drive_motor(MA, 255)
 * drive_motor(MB, -50)
 */
void drive_motor(int p1, int p2, int dir, int speed){
  if(dir==1){
    digitalWrite(p2,0);
    analogWrite(p1,speed);
  }else if (dir==-1){
    digitalWrite(p1,0);
    analogWrite(p2,speed);    
  }else if(dir==0) {
    digitalWrite(p1, 0);
    digitalWrite(p2, 0);
  }else if(dir==10){
    digitalWrite(p1, 1);
    digitalWrite(p2, 0);
  }else if(dir==11){
    digitalWrite(p1, 0);
    digitalWrite(p2, 1);
  }
}

void signalling(int delaytime) {
  // Blink the LED as a signal
  for (int i = 0; i < 3; i++) {
    digitalWrite(RLED2, HIGH);
    delay(delaytime);
    digitalWrite(RLED2, LOW);
    delay(delaytime);
  }
}

//This is the Multiplexor control
void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

// SENDING to COLOR RGB
void to_RGB(long color){
  // if(color>>16 == 1)digitalWrite(WLED2,1);
  // else digitalWrite(WLED2,0);
  Wire.beginTransmission(0x12); 
  Wire.write(0x01);
  Wire.write(0x00);
  Wire.write(0x01);//padding byte
  Wire.write(color>>16); //R
  Wire.write(color>>8); //G
  Wire.write(color); //B
  Wire.write(0x00); 
  Wire.endTransmission(); 
}

void to_MotorA(int dir, int speed){
  //control motor
  Wire.beginTransmission(0x12); 
  Wire.write(0x23);//0
  Wire.write(0x00);//1
  Wire.write(0x00); //2
  Wire.write((char)dir); //3 --> 1 or -1 to drive
  Wire.write((char)speed); //4 1 to 127
  Wire.write(0x00); //this was required!!
  Wire.endTransmission(); 
}

void to_WLED1(int val){
  Wire.beginTransmission(0x12); 
  Wire.write(0x0F);
  Wire.write(0x33);
  Wire.write(0x01);//padding byte
  Wire.write((char)val);
  Wire.write(0x01);//padding byte was required!!
  Wire.endTransmission(); 
}

void debugData(long val){
  int rled_flip=0;
    FOR(i,16){
      digitalWrite(WLED2,(val>>i)&1);
      digitalWrite(RLED2,rled_flip);
      rled_flip = !rled_flip;
      delay(30);
      digitalWrite(WLED2,0);
      digitalWrite(RLED2,rled_flip);
      rled_flip = !rled_flip;
      delay(30);
    }
}
