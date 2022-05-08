// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//MPU6050 Global Variable
MPU6050 accelgyro;

//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax1, ay1, az1;
int16_t gx1, gy1, gz1;
float ax, ay, az;
float gx, gy, gz; 

//Global Declaration for Low Pass Filter
float pres_ax=0.0;
float pres_ay=0.0;
float pres_az=0.0;

//Global Declaration for High Pass Filter
float pres_gx=0; 
float pres_gy=0;
float pres_gz=0;
float prev_x_gx=0;
float prev_x_gy=0;
float prev_x_gz=0;



// Global Declaration for Complementary Filter Roll
float PrevGyroAngleY=0;
float roll=0;
float AccAngleY=0;
float GyroAngleY=0;



//#define OUTPUT_READABLE_ACCELGYRO

#define LED_PIN 13
bool blinkState = false;



//Motor Driver Pin configuration
#define enA 9
#define in1 4
#define in2 5
#define enB 10
#define in3 6
#define in4 7



//Pin Configuration for operating Electromagnet 
#define mag 51
//Motor Speed
int motorSpeedA = 0;
int motorSpeedB = 0;



//Declaring Global Varibales for Communication
int anlogMSB = 0;
int anlogLSB = 0;
int anlogMSB2 = 0;
int anlogLSB2 = 0;
int num1 = 0;
int num2 = 0;
int num3 = 0; 



void communicate()
{

  //Serial.println("I am Communicate");
  
 

}



void Read_Acc_Gyro()
{
  Serial.println("in gyro");
  accelgyro.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  //For Accelereometer
  ax=ax1;
  ay=ay1;
  az=az1;
  gx=gx1;
  gy=gy1;
  gz=gz1;
  
  //Serial.print(ax); Serial.println("\t");
  float AccSens = 16384.00;
  
  if (ax > 32767)
  {
    ax = ax - 65536;
  }
  if (ay > 32767)
  {
    ay = ay - 65536;
  }
  if (az > 32767)
  {
    az = az - 65536;
  }
  
  ax = ax/AccSens;
  ay = ay/AccSens;
  az = az/AccSens;

  //For Gyroscope 
  float GyroSens = 131.00;
  
  if (gx > 32767){
    gx = gx - 65536;
  }
  if (gy > 32767){
    gy = gy - 65536;
  }
  if (gz > 32767){
    gz = gz - 65536;
  }
  
  gx = gx/GyroSens;
  gy = gy/GyroSens;
  gz = gz/GyroSens;
  

  //Function Calling
  Low_Pass_Filter(ax, ay, az);
  High_Pass_Filter(gx, gy, gz);
  Complementary_Filter_Roll(ax, ay, az, gx, gy, gz);
  
  

  //Printing the Values to Serial Port
/*  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz); */
  
 
}


 void Low_Pass_Filter(float pres_ax, float pres_ay, float pres_az)
 {
  float pi=3.1415;
  float f_cut = 5;
  float dT = 0.01;  //time in seconds
  float Tau= 1/(2*pi*f_cut);
  float alpha = Tau/(Tau+dT);

  
  ax = (1-alpha)*pres_ax + alpha*ax;
  
  ay = (1-alpha)*pres_ay + alpha*ay;
  
  az = (1-alpha)*pres_az + alpha*az;

  //Serial.println(typeof(ax));
  
  }



void High_Pass_Filter(float pres_gx, float pres_gy, float pres_gz)  
{
  float pi=3.1415;
  float f_cut = 5;
  float dT = 0.01;  //time in seconds
  float Tau= 1/(2*pi*f_cut);
  float alpha = Tau/(Tau+dT);


  gx = (1-alpha)*gx + (1-alpha)*(pres_gx - prev_x_gx);
  prev_x_gx = pres_gx;
  
  gy = (1-alpha)*gy + (1-alpha)*(pres_gy - prev_x_gy);
  prev_x_gy = pres_gy;
  
  gz = (1-alpha)*gz + (1-alpha)*(pres_gz - prev_x_gz);
  prev_x_gz = pres_gz;
}



void Complementary_Filter_Roll(float ax, float ay, float az, float gx, float gy, float gz)
{
  float alpha = 0.03;
  float elapsedTime = 0.01;

  AccAngleY = atan2(ax,az)*57.2957;  
  if (AccAngleY > 90){
     AccAngleY = -(AccAngleY - 180);
}
  if (AccAngleY < -90){
     AccAngleY = -(AccAngleY + 180);
  }
  
  GyroAngleY = PrevGyroAngleY + (-1)*gy*elapsedTime;
  
  if (GyroAngleY > 90){
     GyroAngleY = -(GyroAngleY - 180);
  }
  if (GyroAngleY < -90){
     GyroAngleY = -(GyroAngleY + 180);
  }
  
  roll = (1 - alpha)*GyroAngleY + alpha*AccAngleY;
  PrevGyroAngleY = roll;
  Serial.print(roll); Serial.println("\t");
  }

void setup() {
  
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(mag,OUTPUT);
  Serial1.begin(9600);
 // Serial.begin(9600);
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);

    // initialize device
    // Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}

void loop(){
  // put your main code here, to run repeatedly:

  
//Read_Acc_Gyro();
 int anlogMSB = 0;
int anlogLSB = 0;
int anlogMSB2 = 0;
int anlogLSB2 = 0;
int num1 = 0;
int num2 = 0;
int num3 = 0;

      if(Serial1.available() >=20)
{
  if(Serial1.read()==0x7E)
  {
      //communicate();
       for(int i =1; i<13; i++)
    {
      byte db=Serial1.read();
    } 

    
      anlogMSB=Serial1.read();
      anlogLSB=Serial1.read();
      anlogMSB2=Serial1.read();
      anlogLSB2=Serial1.read();
      num1 = (anlogMSB<<8|anlogLSB);
      num2 = (anlogMSB2<<8|anlogLSB2);
      num3 = Serial1.read();
      Serial.print(num1);
      Serial.print("...in func..");
      Serial.print(num2);
      Serial.println(".....");
      //Serial.print("5");
  
    
      
      //Serial.println(num1);
      if(num3==3)
      { 
        digitalWrite(mag,HIGH);
      }
      else
      {
        digitalWrite(mag,LOW);
      }
       


      if ((num1 > 940 && num1 < 1000) && num2 == 1023) {
    
     // Set Motor A backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    
    motorSpeedA = 250;
    motorSpeedB = 250;
    //Serial.print("Backward");
    //Serial.print(".....");
    //Serial.print(motorSpeedA);
    //Serial.print(".....");
    //Serial.println(motorSpeedB);
  }
  else if ((num1 >= 0 && num1 < 10) && num2 > 944) {

    // Set Motor A forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    
    
    motorSpeedA = 250;
    motorSpeedB = 250;
    //Serial.print("Forward");
    //Serial.print(".....");
    //Serial.print(motorSpeedA);
    //Serial.print(".....");
    //Serial.println(motorSpeedB);

  }
  // If joystick stays in middle the motors are not moving
  else {
    motorSpeedA = 0;
    motorSpeedB = 0;
  }


  if (num1 == 1023 && (num2 >= 0 && num2 <10 )) {
            
    motorSpeedA = 250 ;
    motorSpeedB = 250 ;

     // Set Motor A backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
   
    
    //Serial.print("Left");
    //Serial.print(".....");
    //Serial.print(motorSpeedA);
    //Serial.print(".....");
    //Serial.println(motorSpeedB);

  }
  if (num1 == 1023 && (num2 < 1023 && num2 >= 940)) {
    
    motorSpeedA = 250 ;
    motorSpeedB = 250 ;

    
    // Set Motor A forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
   
    
    //Serial.print("Right");
    //Serial.print(".....");
    //Serial.print(motorSpeedA);
    //Serial.print(".....");
    //Serial.println(motorSpeedB);

  }

  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);
      
      for(int i =1; i<=3; i++)
    {
      byte dw=Serial1.read();
      }
      //delay(1000);
  }}}
  
