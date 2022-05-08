//#include <TimerOne.h>

/*d=6.8cm*/
volatile signed int temp, counter, counter1 = 0; //This variable will increase or decrease depending on the rotation of encoder
int fdist=0;    
unsigned long time;

float d = 6.8;  
float cir = 3.1415*d;
float rev1 = 0.0;
float vel1 = 0.0;
float rev2 = 0.0;
float vel2 = 0.0;

float prevxdist1 = 0.0;
float prevtime1 = 0.0;
float prevxdist2 = 0.0;
float prevtime2 = 0.0;

void setup() {
  Serial.begin (9600);

  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  
  pinMode(3, INPUT_PULLUP); // 

pinMode(14, INPUT_PULLUP); // internal pullup input pin 2 
  
  pinMode(15, INPUT_PULLUP); 
  
//Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
   
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);

   attachInterrupt(digitalPinToInterrupt(18), ai2, RISING);


    attachInterrupt(digitalPinToInterrupt(19), ai3, RISING);
  
  }

void Motor1()
{

  rev1 = 0.0;
  vel1;
  // Send the value of counter
  if( counter != temp ){
    //int Fcount = counter/2;
    time = millis();

    rev1 = ((counter/2)*1.0)/270*cir;

    //vel1 = abs(((rev1 - prevxdist1)/(time - prevtime1))*1000);
    
    //prevxdist1 = rev1;
    prevtime1 = time;
    /*if(Fcount==270)
    {
      fdist=fdist+d;
      Serial.println (counter/2);
      }*/
     //fdist=(Fcount/270)*cir; 
    //Serial.print(fdist);
    //Serial.println("cm");
  /*Serial.print (rev);
  Serial.print ("..........");
  Serial.println (vel);
  temp = counter;*/
  }
  }
void Motor2()
{
  float d = 6.8;  
  float cir = 3.1415*d;
  rev2 = 0.0;
  vel2;
  // Send the value of counter
  if( counter1 != temp ){
    //int Fcount = counter/2;
    time = millis();

    rev2 = ((counter1/2)*1.0)/270*cir;

    //vel2 = abs(((rev2 - prevxdist2)/(time - prevtime2))*1000);
    
    //prevxdist2 = rev2;
    prevtime2 = time;
    /*if(Fcount==270)
    {
      fdist=fdist+d;
      Serial.println (counter/2);
      }*/
     //fdist=(Fcount/270)*cir; 
    //Serial.print(fdist);
    //Serial.println("cm");
  /*Serial.print (rev);
  Serial.print ("..........");
  Serial.print (vel);
  Serial.println("..........");
  temp = counter1;*/
  }
}


     
  void loop() {
  int rev = 0;  
  Motor1();
  Motor2();
  rev = (rev1 + rev2)/2;
  Serial.println(rev);


  
  }



   
  void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
  counter++;
  }else{
  counter--;
  }
  }
   
  void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
  counter--;
  }else{
  counter++;
  }
  }

  void ai2() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(19)==LOW) {
  counter1--;
  }else{
  counter1++;
  }
  }

  void ai3() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(18)==LOW) {
  counter1++;
  }else{
  counter1--;
  }
  }


  int PID(float pitch) {  


    float targetAngle = 0.0;

    // Calculate time since last time PID was called (~10ms)
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
    unsigned long thisTime = millis();
    float timeChange = float(thisTime - lastTime);

    // Calculate Error
    float error = targetAngle - pitch;


    // Calculate our PID terms
    // PID values are multiplied/divided by 10 in order to allow the
    // constants to be numbers between 0-10.
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
    float pTerm = Kp * error * 10;
    iTerm += Ki * error * timeChange / 10;  
    float dTerm = Kd * (pitch - lastpitch) / timeChange * 100; 
  
  if (Ki == 0) iTerm = 0;
    lastpitch = pitch;
    lastTime = thisTime;


    // Obtain PID output value
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
    float PIDValue = pTerm + iTerm - dTerm;

    // Set a minimum speed (motors will not move below this - can help to reduce latency)
    //if(PIDValue > 0) PIDValue = PIDValue + 10;
    //if(PIDValue < 0) PIDValue = PIDValue - 10;

  // Limit PID value to maximum PWM values
    /*if (PIDValue > 127) PIDValue = 127;
    else if (PIDValue < -128) PIDValue = -128; */

    return int(PIDValue);

}
