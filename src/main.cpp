#include <Arduino.h>
#include <stdio.h>
/*** AT2 - master 0x17
 AT2 of DryBot v2.3a, may 10 2024.
 */
#include <Wire.h>
#define TF_ON
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
#define SEN13 PIN_PC0
#define SEN14 PIN_PC1
#define SEN1  PIN_PC2 //IR sensor
#define SEN2 PIN_PC3
#define SEN3 PIN_PB2
#define MB1 PIN_PB3
#define MB2 PIN_PA3
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
/*** Wire interface **********************************************/
#define SLAVE_ADDRESS 0x12 
#define BUFFER_SIZE 20 
/****Setup function ================================================*/
void setup() {
  delay(2000);
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
  init_ADC1(); //required by author

  analogReference1(INTERNAL2V5); // set reference to the desired voltage, and set that as the ADC reference.
  analogReference1(VDD); // Set the ADC reference to VDD. Voltage selected previously is still the selected, just not set as the ADC reference.  
  //analogReference1(INTERNAL0V55);

  Wire.begin(); // join i2c bus as master
  //Switch colour sensor
  //TCA9548A(1);
  digitalWrite(RLED2, 1); // 0 on, 1 off
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
}
// arduino long type has 4 bytes, 0xFFFFFFFF, signed. ranged -2,147,483,648 to 2,147483,647
long anval =0;
int counter = 0;
void loop() {  
  counter++;
  // to_RGB( random(0xAAAAAA)); //RGB proof i2c works
  if (counter % 4 == 0)  to_RGB( random(0xAAAAAA)); //RGB proof i2c works
  if (counter > 100) counter=0;
  anval = analogRead1(SEN13); 
  to_Long(anval);
  anval = analogRead1(SEN14); 
  to_Long(anval);

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
  Wire.write('R');
  Wire.write('G');
  Wire.write('B');//padding byte
  Wire.write((char)color>>16 & 0xFF); //R
  Wire.write(color>>8 & 0xFF); //G
  Wire.write(color & 0xFF); //B
  Wire.write('E'); 
  Wire.endTransmission(); 
  delay(1);
}

void to_MotorA(int dir, int speed){
  //control motor
  Wire.beginTransmission(0x12); 
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
  Wire.beginTransmission(0x12); 
  Wire.write('W');
  Wire.write('L');
  Wire.write('1');//padding byte
  Wire.write((char)val);
  Wire.write('E');//padding byte was required!!
  Wire.endTransmission(); 
}

void to_Char(char* val, int lngth){
  Wire.beginTransmission(0x12); 
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
  Wire.beginTransmission(0x12); 
  Wire.write('L');
  Wire.write('o');
  Wire.write('n');
  FOR(i,4)Wire.write(val>>(i*8) & 0xFF);
  Wire.write('E');//padding byte was required!!
  Wire.endTransmission(); 
}

void to_Int(int val){
  Wire.beginTransmission(0x12); 
  Wire.write('I');
  Wire.write('n');
  Wire.write('t');
  FOR(i,2)Wire.write(val>>(i*8) & 0xFF);
  Wire.write('E');//padding byte was required!!
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
