#include <Arduino.h>
#include <stdio.h>
/*** AT2 - slave 0x17
 AT2 of DryBot v2.3a, may 10 2024.
 Drybot v2.5b may19th 2024. update pin positions.
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
#define SEN13 PIN_PC0 //LDR
#define SEN14 PIN_PC1 //LDR
#define SEN1  PIN_PC2 //IR Sensors left
#define SEN2 PIN_PC3  
#define SEN3 PIN_PB2  //IR Sensor right -unable to use due to wrong pin assignment.
#define ENC_MC_A PIN_PB3  //update 2.5b
#define ENC_MC_B PIN_PA3
#define MC1 PIN_PA4
#define MC2 PIN_PA5
#define MD1 PIN_PB4
#define MD2 PIN_PB5
#define ENC_MD_A PIN_PA6
#define ENC_MD_B PIN_PA7
//PWM pins - see build flags.
#define WLED2 PIN_PA2 //dryBot LEDW 0 = off
#define RLED2 PIN_PA1 //dryBot LEDR 1 = off
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
//header for AT2
void TCA9548A(uint8_t bus);
void drive_motor(int,int,int,int);
void signalling(int);
void to_RGB(long color);
void to_MotorA(int dir, int speed);
void debugData(long val);
void to_Char(char* val, int lngth);
void to_Long(long val);
void to_WLED1(char val);
void to_Int(int val);
void receiveData(int numBytes);
void sendData();
/*** Wire interface **********************************************/
#define MY_ADDRESS 0x14
#define AT1_SLAVE 0x12
#define BUFFER_SIZE 20 
char receivedData[BUFFER_SIZE]; 
char repositoryData[BUFFER_SIZE]; //4:color, 2:LDR, 2:MCenc, 2:MDenc
int dataLength = 0; 
int postflag = 0;
int outLength = 8;
int sendflag = 0;
//master send
void receiveData(int numBytes) {
  //postflag = 0;
  dataLength = numBytes;
  Wire.readBytes(receivedData, numBytes);
  postflag = 1;//mark data ready
}
//master read
void sendData() {
  Wire.write(repositoryData, outLength);
}
/****Setup function ================================================*/
void setup() {
  //delay(2000);
  pinMode(RLED2,OUTPUT);
  pinMode(WLED2,OUTPUT);
  pinMode(ENC_MD_A,INPUT);
  pinMode(ENC_MD_B,INPUT);
  pinMode(ENC_MC_A, OUTPUT);  //udpate 2.5b
  pinMode(ENC_MC_B, OUTPUT);  //update 2.5b
  pinMode(MC1, OUTPUT); 
  pinMode(MC2, OUTPUT);
  pinMode(MD1, OUTPUT); 
  pinMode(MD2, OUTPUT);
  pinMode(SEN13, INPUT);
  pinMode(SEN14, INPUT);
  pinMode(SEN1, INPUT);
  pinMode(SEN2, INPUT);
  pinMode(SEN3, INPUT);
  init_ADC1(); //required by author
  Wire.setClock(400000);
  analogReference1(INTERNAL2V5); // set reference to the desired voltage, and set that as the ADC reference.
  analogReference1(VDD); // Set the ADC reference to VDD. Voltage selected previously is still the selected, just not set as the ADC reference.  
  //analogReference1(INTERNAL0V55);//dont do this!
  Wire.onReceive(receiveData); // callback for receiving data
  Wire.onRequest(sendData); // callback for sending data
  Wire.begin(MY_ADDRESS); // join i2c bus as slave
  //Wire.begin();
  //Switch colour sensor
  //TCA9548A(1);
  digitalWrite(RLED2, 0); // 0 on, 1 off
  digitalWrite(WLED2, 0); // 1 on, 0 off
  //digitalWrite(MD1, 0); // 0 on, 1 off
  //digitalWrite(MD2, 0); // 0 on, 1 off
  #ifdef TF_ON
  
  if (!sensor.init()) {
    FOR(k,3){
    signalling(30);
    delay(1000);
    }
  }
  sensor.setTimeout(500);
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

// drive_motor(MD1,MD2,-1,120);
// delay(300);
// drive_motor(MD1,MD2,0,0);
}
// arduino long type has 4 bytes, 0xFFFFFFFF, signed. ranged -2,147,483,648 to 2,147483,647
long anval =0;
long data=0xFFFFFF;
long curLDRTime = millis();
long curIRTime = millis();
void loop() {
  // delay(1);
  delayMicroseconds(500);

  int rled_flip=0;  
  if(postflag == 1){
    if(receivedData[0]=='M' && receivedData[1]=='C'){
      //drive motor C
      drive_motor(MC1, MC2, (char)receivedData[2], (char)receivedData[3]); //only works when bytes.
    }else if(receivedData[0]=='M' && receivedData[1]=='D'){
      //drive motor D
      drive_motor(MD1, MD2, (char)receivedData[2], (char)receivedData[3]); //only works when bytes.
    }else if(receivedData[0]=='W' && receivedData[1]=='L'){
      //drive WLED2
      digitalWrite(WLED2, receivedData[3]=='A'?1:0);
    }else if(receivedData[0]=='C' && receivedData[1]=='h'){
      //received Char array
      int lngth = (int)receivedData[3];
      // FOR(i,lngth){
      //   Serial.print((char)receivedData[4+i]);
      // }
    }else if(receivedData[0]=='L' && receivedData[1]=='o'){
      //received long value
      long rec = *(long*)(&receivedData[3]);
     }else if(receivedData[0]=='I' && receivedData[1]=='n'){
      //received int value
      int rec = *(int*)(&receivedData[3]);
    }else{
      //not in spec.
      digitalWrite(RLED2,rled_flip);
      rled_flip = !rled_flip;
    }
    postflag = 0;
  }else{
    FOR(i,dataLength) receivedData[i]=0;
  }

  //read LDR every second
  if (millis() - curLDRTime > 1000){
    anval = analogRead1(SEN13); //LDR right
    repositoryData[0] = (anval & 0xFF )<< 8;
    repositoryData[1] = (anval & 0xFF );
    anval = analogRead1(SEN14); //LDR left
    repositoryData[2] = (anval & 0xFF )<< 8;
    repositoryData[3] = (anval & 0xFF );
    curLDRTime = millis();
  }
  
  if (millis() - curIRTime > 500){
    anval = analogRead1(SEN1); //IR right
    repositoryData[4] = (anval & 0xFF )<< 8;
    repositoryData[5] = (anval & 0xFF );
    anval = analogRead(SEN3); //IR left
    repositoryData[6] = (anval & 0xFF )<< 8;
    repositoryData[7] = (anval & 0xFF );
    curIRTime = millis();
  }

  //to_RGB( 0xFFFFFF - random((long)0xA1A1A1)); //RGB proof i2c works
  //to_RGB(0xAAFFAA);
  //anval = analogRead1(SEN13); 
  //anval = analogRead1(SEN14); 

  //read from color sensor

  #ifdef TF_ON
  head=sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) FOR(k,3)signalling(50);
  if(head > 300){
    digitalWrite(WLED2,1); //turn on
    to_WLED1('A');
  }else{
    digitalWrite(WLED2,0);
    to_WLED1('B');
  }
  //to_Int(head);
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

  // }
  
}

/*
 * dir 1/ 0 / -1 / 10 / 11 / 2
 * power = mimic lego's design: 0~127
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
  }else if(dir==2) {
    digitalWrite(p1, 1);
    digitalWrite(p2, 1);
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
  Wire.beginTransmission(AT1_SLAVE); 
  Wire.write('R');
  Wire.write('G');
  Wire.write('B');//padding byte
  Wire.write(color>>16 & 0xFF); //R
  Wire.write(color>>8 & 0xFF); //G
  Wire.write(color & 0xFF); //B
  Wire.write('E'); 
  Wire.endTransmission(); 
  delay(1);
}

void to_MotorA(int dir, int speed){
  //control motor
  Wire.beginTransmission(AT1_SLAVE); 
  Wire.write('M');//0
  Wire.write('A');//1
  Wire.write('1'); //2
  Wire.write((char)dir); //3 --> 1 or -1 to drive
  Wire.write((char)speed); //4 1 to 127
  Wire.write('E'); //this was required!!
  Wire.endTransmission(); 
}

//sending 0 as string ?
void to_WLED1(char val){
  Wire.beginTransmission(AT1_SLAVE); 
  Wire.write('W');
  Wire.write('L');
  Wire.write('1');//padding byte
  Wire.write((char)val);
  Wire.write('E');//padding byte was required!!
  Wire.endTransmission(); 
}

void to_Char(char* val, int lngth){
  Wire.beginTransmission(AT1_SLAVE); 
  Wire.write('C');
  Wire.write('h');
  Wire.write('r');
  Wire.write((char)lngth);
  FOR(c,lngth){
    Wire.write(*val);
    val++;
  }
  Wire.write('E');//padding byte was required!!
  Wire.endTransmission(); 
}

void to_Long(long val){
  Wire.beginTransmission(AT1_SLAVE); 
  Wire.write('L');
  Wire.write('o');
  Wire.write('n');
  FOR(i,4)Wire.write(val>>(i*8) & 0xFF);
  Wire.write('E');//padding byte was required!!
  Wire.endTransmission(); 
}

void to_Int(int val){
  Wire.beginTransmission(AT1_SLAVE); 
  Wire.write('I');
  Wire.write('n');
  Wire.write('t');
  FOR(i,2)Wire.write(val>>(i*8) & 0xFF);
  Wire.write('E');//padding byte was required!!
  Wire.endTransmission(); 
}
/**
 * Display any byte, ex, 0b00001111, numdig=8
 * wled blink first 4 times.
 * rled blink all 8 times.
*/

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
