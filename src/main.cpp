#include <Arduino.h>
/*** AT2
 AT2 of DryBot v2.3a, may 10 2024.
 */
#include <Wire.h>
#define TF_OFF
#define ACC_OFF
#define CLR_ON

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

#define FOR(I,N) for(int I=0;i<N;I++)
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

int SEN13 = PC0;
int SEN14 = PC1;
int SEN1  = PC2;
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
//#define MY_ADDRESS 0x17 // address for this slave device
//#define SLAVE_ADDRESS 0x12

#ifdef TF_ON
  int head=0;
#endif
#ifdef ACC_ON
  float z_acc=0.0;
#endif

void show_RGB(long);
void drive_motor(int,int,int,int);
void signalling(int);

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
  


  //Wire.begin(MY_ADDRESS); // join i2c bus as slave
  Wire.begin(); // join i2c bus as master
  //Switch colour sensor
  TCA9548A(1);
  //Wire.onReceive(receiveData); // callback for receiving data
  //Wire.onRequest(sendData); // callback for sending data
  digitalWrite(RLED2, 0); //off
  digitalWrite(WLED2, 1); // 1 on for light
  
  #ifdef TF_ON
  sensor.setTimeout(500);
  if (!sensor.init()) {
    FOR(3){
    signalling(30);
    delay(100);
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
}

// arduino long type has 4 bytes, 0xFFFFFFFF, signed. ranged -2,147,483,648 to 2,147483,647
void loop() {  

  #ifdef TF_ON
  head=sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) FOR(3)signalling(50);
  if(head > 100){
    digitalWrite(WLED1,1); //turn on
  }else{
    digitalWrite(WLED1,0);
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

    RGBWSensor.getRed();
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
