/*
▪ * Team Id: #1122
▪ * Author List: Nishan Poojary, Prathamesh Pokhare, Pratik Poojary, Yagnesh Oza
▪ * Filename: Main_Control.ino
▪ * Theme: BIPED PATROL
▪ * Functions: communicate, 
 READ_ACCEL_GYRO,  LOW_AND_HIGH_PASS_FILTER, COMPLIMENTARY_FILTER, Motor1, Motor2, ai0, ai1, ai2, ai3
▪ * Global Variables: Global Declaration Below
▪ */

//Declaring variables Volatile so that arduino does'nt optimise them as their value changes every moment.
// Including the installed libraries I2Cdev and MPU6050
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
volatile int tempISR=0;
// Arduino Pins Declarations for Motor Driver Circuit 

#define enA 9
#define in1 4
#define in2 5
#define enB 10
#define in3 6
#define in4 7

//LED, Electromagnet and Buzzer Pin Declarations

#define mag 51
#define buz 44
#define GreenLED 45   //For Red LED
#define RedLED 46   //For Green LED
#define Common_pin 47 //Common Pin for LEDS


//MPU6050 Global Variable
MPU6050 accelgyro;

//Global Declaration for Accelerometer and Gyroscope Values in X, Y, Z Direction Respectively

volatile int16_t ax1, ay1, az1;
volatile int16_t gx1, gy1, gz1;
volatile float ax, ay, az;
volatile float gx, gy, gz; 

//Global Declaration for Low Pass Filter
volatile float pres_ax=0.0;
volatile float pres_ay=0.0;
volatile float pres_az=0.0;

//Global Declaration for High Pass Filter
volatile float pres_gx=0; 
volatile float pres_gy=0;
volatile float pres_gz=0;
volatile float prev_x_gx=0;
volatile float prev_x_gy=0;
volatile float prev_x_gz=0;
volatile float angvel = 0.0;
volatile float angle = 0.0;
volatile float u = 0.0;
volatile float uv = 0.0;
volatile float absuv = 0.0;
volatile float rev = 0.0; 
volatile float vel = 0.0;
float alphac = 0.24;
float elapsedTime = 0.01;
float AccSens = 16384.00;//Accelerometer Sensitivity Value 
float GyroSens = 131.00;//Gyroscope Sensitivity Value

//Global Declaration for Complementary Filter Roll
volatile float PrevGyroAngleY;
volatile float roll;
volatile float AccAngleY;
volatile float GyroAngleY;

//Global Declaration of the Encoder Variables
volatile signed int temp, counter, counter1 = 0; //This variable will increase or decrease depending on the rotation of encoder
volatile int fdist=0;    
unsigned long time;

volatile float d = 0.068;  
volatile float cir = 0.031415*d;
volatile float ang = 0.0;
volatile float ang1 = 0.0;
volatile float ang2 = 0.0;

volatile float rev1 = 0.0;
volatile float vel1 = 0.0;
volatile float rev2 = 0.0;
volatile float vel2 = 0.0;

volatile float prevxdist = 0.0;
volatile float prevtime=0;
volatile float prevroll = 0.0;


//Global Declaration for variables used Communication Function
volatile int anlogMSB;
volatile int anlogLSB;
volatile int anlogMSB2;
volatile int anlogLSB2;
volatile int num1;
volatile int num2;
volatile int num3;
volatile int num4;

int motorSpeedA = 0;
int motorSpeedB = 0;


void LED_init(){
    pinMode(GreenLED, OUTPUT);
    pinMode(Common_pin, OUTPUT);
    pinMode(RedLED, OUTPUT);
   
    digitalWrite(GreenLED, HIGH);
    digitalWrite(Common_pin, HIGH);
    digitalWrite(RedLED, HIGH);
  }



//For LED in MPU6050
#define LED_PIN 13
bool blinkState = false;



void setup() {
  
// join I2C bus
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

// initialize serial communication
    accelgyro.initialize();

//Pin Declaration for Arduino as Output and Input     
    pinMode(47,OUTPUT);
    pinMode(44,OUTPUT);
    pinMode(45,OUTPUT);
    pinMode(46,OUTPUT);
    pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
    pinMode(3, INPUT_PULLUP); 
    pinMode(14, INPUT_PULLUP); // internal pullup input pin 2 
    pinMode(15, INPUT_PULLUP); 
   
  
//Setting up interrupt
    //A rising pulse from encodenren activated ai0(). 
    attachInterrupt(0, ai0, RISING);
   
    //B rising pulse from encodenren activated ai1(). 
    attachInterrupt(1, ai1, RISING);

    //A rising pulse from encodenren activated ai2().
    attachInterrupt(digitalPinToInterrupt(18), ai2, RISING);

    //A rising pulse from encodenren activated ai3().
    attachInterrupt(digitalPinToInterrupt(19), ai3, RISING);


//Timer Interrupt used for Obtaining Data from MPU6050 at 5ms   
    cli();//Disabling all Interrupts
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    // set compare match register for 1hz increments
    // turn on Normal mode
    TCCR1B =0x02;
    // enable timer compare interrupt
    TIMSK1 =0x01;
    TCNT1=0xD8EF;

//Timer Interrupt used for Obtaining Data from Encoder Motors and Writing PWM values to motor at 5ms
    TCCR3A = 0;// set entire TCCR1A register to 0
    TCCR3B = 0;// same for TCCR1B
    // set compare match register for 1hz increments
    // turn on Normal mode
    TCCR3B =0x02;
    // enable timer compare interrupt
    TIMSK3 =0x01;
    TCNT3=0xD8EF;

//Begining Serial Communication for Serial 1 and 2 at Baud Rate 9600    
    Serial.begin(9600);
    Serial2.begin(9600);
    sei();//Enabling all Interrupts
}


ISR(TIMER1_OVF_vect)        // interrupt service routine for Reading Data from MPU6050
{
  //tempISR++;
  //Serial.println(tempISR);
    sei();
    TCNT1=0x4E20;
 
    accelgyro.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    
    //For Accelereometer
    ax=ax1;
    az=az1;

    //For Gyroscope
    gy=gy1;

//Calling Functions 
    READ_ACCEL_GYRO();
    //LOW_AND_HIGH_PASS_Filter();
    COMPLIMENTARY_Filter();
    
  
}


ISR(TIMER3_OVF_vect)        // interrupt service routine for Obtaining Data from Encoder Motors and Writing PWM values to motor
{
  //tempISR++;
 // Serial.println(tempISR);
    TCNT3=0x4E20;            // preload timer
    
    angle = roll/57.2957;

    time = millis();//Time to calculate velocity and angular velocity

//Calling Functions 
    Motor1();
    Motor2();
  
   // Serial.println(ang1);
   // Serial.println(ang2)    
  
    ang=(ang1+ang2)/2;//Average Angular  Displacement
    rev=(0.034*ang);//Calculating Distance
    vel = ((rev - prevxdist)/(time - prevtime)*1000);//Calculating Velocity

    //float angvel = 0.0; 
    angvel = ((angle - prevroll)/(time - prevtime)*1000);//Calculating Angular Velocity
    prevroll = angle;//Previous Angle
    prevtime = time;//Previous Time
    prevxdist = rev;//Previous Distance
  
    
    /*
    Serial.print(GyroAngleY);
    Serial.print(", ");*/
    //Serial.print(gy);
    Serial.println(angle);
    Serial.print(" ");
    Serial.println(angvel);
//Tested LQR Values
    u = (-90*rev - 111*vel + 356*angle + 56*angvel);//u obtained is Voltage
     // u = (0*rev + 0*vel +2612.4*angle + 1231*angvel);
    //uv = abs(141.17*u + 2.4*(rev/0.034));//Expression to convert Input Torque into Voltage

//Voltage to PWM Conversion
    uv = constrain(u,0,12);//Constraining Voltage obtained between 0 to 12
    absuv = map(u, 0,12, 0, 255);//Mapping Voltage into PWM values 
    
    if(u<0)
    {
      //this will set motor in BACKWARD direction
   
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enA,absuv);//Writing Correction PWM values to Motor1
      analogWrite(enB,absuv);//Writing Correction PWM values to Motor2
    }
     
    else if(u>=0)
    {
      //this will set motor in FORWARD direction
    
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enA,absuv);//Writing Correction PWM values to Motor1
      analogWrite(enB,absuv);//Writing Correction PWM values to Motor2
    }
}

/*
 * Function Name:- LOW_AND_HIGH_PASS_Filter
 * Input:- None
 * Output:- None
 * Logic:- Passes the raw data through low pass and high pass filter.
 * Example:- LOW_AND_HIGH_PASS_Filter()
 */

void LOW_AND_HIGH_PASS_Filter()
{

//Variables used for Low and High Pass Filter   
    float pi=3.1415;
    float f_cut = 20;
    float dT = 0.005;  //time in seconds
    float Tau= 1/(2*pi*f_cut);
    float alpha = Tau/(Tau+dT);

//Passing Accelerometer values ax and az from Low Pass Filter   
    ax = (1-alpha)*pres_ax + alpha*ax;
    
    az = (1-alpha)*pres_az + alpha*az;

//Passing Gyroscope values gy from High Pass Filter        
    gy = (1-alpha)*gy + (1-alpha)*(pres_gy - prev_x_gy);
    prev_x_gy = pres_gy;
    
  }

/*
 * Function Name:- READ_ACCEL_GYRO
 * Input:- None
 * Output:- None
 * Logic:- Reads Raw data from MPU6050.
 * Example:- READ_ACCEL_GYRO()  
 */

void READ_ACCEL_GYRO()
{
   
   

//Getting 2's Complement of ax, az respectively   
    if (ax > 32767)
    {
      ax = ax - 65536;
    }
    if (az > 32767)
    {
      az = az - 65536;
    }

//Getting Values of ax, az within the range     
    ax = ax/AccSens;
    az = az/AccSens;

    //For Gyroscope 
    

//Getting 2's Complement of gy  
    if (gy > 32767){
     gy = gy - 65536;
    }

//Getting Values of gy within the range  
    gy = gy/GyroSens;
   //gSerial.println(gy);
  
  }


/*
 * Function Name:- COMPLIMENTARY_Filter
 * Input:- None
 * Output:- None
 * Logic:- Passes the raw data through compimentary filter.
 * Example:- COMPLIMENTARY_Filter();  
 */

void COMPLIMENTARY_Filter()
{

//Variables used for Complementary Filter   
   

    AccAngleY = atan2(ax,az)*57.2957;//Getting angle in Y Direction from Accelerometer readings ax,az in degrees  

//Limiting Accelerometer values within range -90 to 90 degrees    
    if (AccAngleY >= 90){
       AccAngleY = -(AccAngleY - 180);
    }
    if (AccAngleY <= -90){
       AccAngleY = -(AccAngleY + 180);
    }
      //Serial.print(AccAngleY);
     // Serial.print(" ");
    GyroAngleY = PrevGyroAngleY + (-1)*gy*elapsedTime;

//Limiting Gyroscope values within range -90 to 90 degrees   
    if (GyroAngleY >= 90){
       GyroAngleY = -(GyroAngleY - 180);
    }
    if (GyroAngleY <= -90){
       GyroAngleY = -(GyroAngleY + 180);
       
    }
    //Serial.println(GyroAngleY);
  
    roll = (1 - alphac)*GyroAngleY + alphac*AccAngleY;//Final Filter Values for angle 
    //PrevGyroAngleY = roll;//Previous Angle
   //gSerial.println(roll);
   PrevGyroAngleY = roll;
  }

 
/*
 * Function Name:- communicate
 * Input:- None
 * Output:- None
 * Logic:- It takes the data from the received frames and then manipulates them acoording to the user. 
 * Example:- communicate();
 */

void communicate()
{
  
    //Retrieving data from xbee
   
    if(Serial2.available() >=20)
    {
      if(Serial2.read()==0x7E)
      {
        
        for(int i =1; i<13; i++)
          {
            
            // Discard bytes
            byte db=Serial2.read();
            
          }
          
     anlogMSB=Serial2.read(); //  First number's MSB value at position 13 
     anlogLSB=Serial2.read(); //  First number's LSB value at position 14
     anlogMSB2=Serial2.read(); //  Second number's MSB value at position 15
     anlogLSB2=Serial2.read(); //  Second number's LSB value at position 16
      
     num1 = (anlogMSB<<8|anlogLSB); // Combining MSB and LSB to get First Number
     num2 = (anlogMSB2<<8|anlogLSB2); // Combining MSB and LSB to get Second Number
     num3 = Serial2.read(); //  Third number value at position 17
     byte db=Serial2.read();
     num4 = Serial2.read();  //Fourth number value at position 19
               
     // Condition to set Electromagnet ON or OFF
       
     if(num3==3)
     { 
       digitalWrite(mag,HIGH);// Electromagnet ON  
     }
     else
     {
       digitalWrite(mag,LOW); // Electromagnet OFF
     }

     if(num4==3)
     {   
        digitalWrite(RedLED,HIGH);
        if (num3==3)
        {
          digitalWrite(RedLED,LOW);
          digitalWrite(GreenLED,HIGH);
        }
        digitalWrite(buz,LOW);        
      }
      else
      {
        digitalWrite(buz,HIGH);
        digitalWrite(GreenLED,LOW);
        digitalWrite(RedLED,LOW);
      }

    
// Motion in Backward Direction
      
    if ((num1 > 940 && num1 < 1000) && num2 == 1023) {
    
      // Set Motor A backward
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    
      // Set Motor B backward
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);

      // Setting Motor Speed
      motorSpeedA = 250;
      motorSpeedB = 250;
  }


// Motion in Forward Direction
  
    else if ((num1 >= 0 && num1 < 10) && num2 > 944) {

      // Set Motor A forward
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
  
      // Set Motor B forward
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    
      // Setting Motor Speed
      motorSpeedA = 250;
      motorSpeedB = 250;
  }

  
// If joystick stays in middle the motors are not moving
  
    else {
      motorSpeedA = 0;
     motorSpeedB = 0;
    }

 
// Motion in Left Direction 
  
    if (num1 == 1023 && (num2 >= 0 && num2 <10 )) {
      // Setting Motor Speed        
      motorSpeedA = 250 ;
      motorSpeedB = 250 ;

      // Set Motor A backward
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
   
      // Set Motor B forward
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
  }



// Motion in Right Direction
  
    if (num1 == 1023 && (num2 < 1023 && num2 >= 920)) {
    
      // Setting Motor Speed
      motorSpeedA = 250 ;
      motorSpeedB = 250 ;

      // Set Motor A forward
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
   
      // Set Motor B backward
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);  
  }
  

// Passing Motor Speed values to enA and enB pins

    analogWrite(enA, motorSpeedA);
    analogWrite(enB, motorSpeedB);
  
    
//Discard the remaining values of the received frame

      for(int i =1; i<=3; i++)
     {
        // Discard bytes
        byte dw=Serial2.read();
     }
   }
  }
}


/*
 * Function Name:- Motor1
 * Input:- None 
 * Output:- None
 * Logic:- It computes the wheel rotation and calculates first wheel angle.
 * Example:- communicate();
 */

  
void Motor1()
{
  
    ang1 = 0.0;
// Send the value of counter
  
    if( counter != temp ){
      ang1=(counter*3.1415)/270;
    }
  }

/*
 * Function Name:- Motor2
 * Input:- None
 * Output:- None
 * Logic:- It computes the wheel rotation and calculates second wheel angle.
 * Example:- communicate();
 */

 
void Motor2()
{
    ang2 = 0.0;
// Send the value of counter
  
    if( counter1 != temp ){
      ang2=(counter*3.1415)/270;
    }
}


  
void loop() {

//Calling Communivcation Function for Communication Between Xbee and arduino   
  //communicate();

}


/*
 * Function Name:- ai0
 * Input:- None
 * Output:- None
 * Logic:- Used for counting the first wheel rotation for forward direction.
 * Example:-  ai0();
 */


void ai0() {
// ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
// Check pin 3 to determine the direction
    if(digitalRead(3)==LOW) {
      counter++;
    }else{
      counter--;
    }
  }

/*
 * Function Name:- ai1
 * Input:- None
 * Output:- None
 * Logic:- Used for counting the second wheel rotation for forward direction.
 * Example:-  ai1();
 */
   
  void ai1() {
// ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
// Check with pin 2 to determine the direction
    if(digitalRead(2)==LOW) {
      counter--;
    }else{
      counter++;
    }
  }

/*
 * Function Name:- ai2
 * Input:- None
 * Output:- None
 * Logic:- Used for counting the first wheel rotation for backward direction.
 * Example:-  ai2();
 */

  void ai2() {
// ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
// Check pin 3 to determine the direction
    if(digitalRead(19)==LOW) {
      counter1--;
    }else{
      counter1++;
    }
  }

/*
 * Function Name:- ai3
 * Input:- None
 * Output:- None
 * Logic:- Used for counting the second wheel rotation for backward direction.
 * Example:-  ai3();
 */
 
  void ai3() {
// ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
// Check pin 3 to determine the direction
    if(digitalRead(18)==LOW) {
      counter1++;
    }else{
      counter1--;
    }
  }
  
