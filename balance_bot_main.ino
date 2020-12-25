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

void setup() {
 
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(mag,OUTPUT);
  Serial1.begin(9600);
}

void loop() {
  
if(Serial1.available() >=20)
{
  if(Serial1.read()==0x7E)
  {
    for(int i =1; i<13; i++)
    {
      // Discard bytes
      byte db=Serial1.read();
      }
      int anlogMSB=Serial1.read(); //  First number's MSB value at position 13 
      int anlogLSB=Serial1.read(); //  First number's LSB value at position 14
      int anlogMSB2=Serial1.read(); //  Second number's MSB value at position 15
      int anlogLSB2=Serial1.read(); //  Second number's LSB value at position 16
      
      int num1 = (anlogMSB<<8|anlogLSB); // Combining MSB and LSB to get First Number
      int num2 = (anlogMSB2<<8|anlogLSB2); // Combining MSB and LSB to get Second Number
      int num3 = Serial1.read(); //  Third number value at position 17

      // Testing on Serial Monitor
      //Serial.print(num1);
      //Serial.print(".....");
      //Serial.print(num2);
      //Serial.print(".....");

      // Condition to set Electromagnet ON or OFF
      if(num3==3)
      { 
        digitalWrite(mag,HIGH);// Electromagnet ON
      }
      else
      {
        digitalWrite(mag,LOW); // Electromagnet OFF
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

    // Testing on Serial Monitor
    //Serial.print("Backward");
    //Serial.print(".....");
    //Serial.print(motorSpeedA);
    //Serial.print(".....");
    //Serial.println(motorSpeedB);
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

    // Testing on Serial Monitor
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
   
    // Testing on Serial Monitor
    //Serial.print("Left");
    //Serial.print(".....");
    //Serial.print(motorSpeedA);
    //Serial.print(".....");
    //Serial.println(motorSpeedB);

  }
  // Motion in Right Direction
  if (num1 == 1023 && (num2 < 1023 && num2 >= 940)) {
    // Setting Motor Speed
    motorSpeedA = 250 ;
    motorSpeedB = 250 ;

    
    // Set Motor A forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
   
    // Testing on Serial Monitor
    //Serial.print("Right");
    //Serial.print(".....");
    //Serial.print(motorSpeedA);
    //Serial.print(".....");
    //Serial.println(motorSpeedB);

  }

  // Passing Motor Speed values to enA and enB pins
  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);
      
      for(int i =1; i<=3; i++)
    {
      // Discard bytes
      byte dw=Serial1.read();
      }
      //delay(1000);
      }
  }
}
