#include <SoftwareSerial.h>
#include <PID_v1.h>//PID library

SoftwareSerial DebugSerial(2, 3);

#define BLYNK_PRINT DebugSerial
#include <BlynkSimpleStream.h>

char auth[] = "f6b6efbcbf024e559c2c3c08fd3eb207";
WidgetLCD lcd(V1);//LED display widget of Bylnk Portt V1

#define PIN6 6//pin6 for getting LOW time of 555 timer output
#define PIN9 9//pin9 outputs PWM for heating up 
#define PIN10 10//pin10 outputs PWM for cooling down

//Initialization 
float Tset;//the Temperature we set
float Tnow = 0;//current Temperature
float middle = 0; //variable for calculation
unsigned long duration = 0;//low time of the output signal of 555 timer
float D;//duration / 10000.0

//Parameters of PID
float Kp = 3.0;
float Ki = 0.002;
float Kd = 0;
double Input, Output, Setpoint;

PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd, DIRECT);//myPID function to control Temperature


//Main Function
void setup()
{
  DebugSerial.begin(9600);//debug consol of Blynk
  Serial.begin(9600);
  
  //set pin mode
  pinMode(PIN6, INPUT);//Pin6 as the input of 555 timer
  pinMode(PIN9, OUTPUT);//Pin9 as output of Hbridge
  pinMode(PIN10, OUTPUT);//Pin10 as output of Hbridge
  
  Blynk.begin(auth, Serial);
  while (Blynk.connect() == false) {
    // Wait until Arduino connected
  }
  
  
  //display some texts on LCD 
  lcd.clear(); //Use it to clear the LCD Widget
  lcd.print(0, 0, "Tset    Tnow");//display text
  

  myPID.SetMode(AUTOMATIC);//turn PID loop
}

  
//Get the set temperature from the slider of the Blynk app Port V0
BLYNK_WRITE(V0){
    Tset = 5 + param.asInt()*45/255.0; //set temperature from 5 degree to 50 degree
    lcd.print(0,1,Tset);//display the temperature that we set
    }
    

void loop()
{
  Blynk.run();//run Blynk
 
  float D = 0;
  duration = pulseIn(PIN6, LOW);//get the low time of the output of 555 timer
  D = duration/10000.0;
  Setpoint = Tset;// Setpoint is the Temperaure that we et
  Tnow = 
  -0.029*D*D*D + 1.024*D*D - 14.43*D + 86.11;//The current temperature is a function of duration

  //PID
  if(Tnow < Tset){
    Input = Tnow;
    myPID.Compute();//run the PID loop
    analogWrite(PIN9,Output + 150);//Heating up
    analogWrite(PIN10, 0);//shut
  }   else if (Tnow > Tset){
            middle = Tset + Tset - Tnow;
            Input = middle;
            myPID.Compute();//run the PID loop
            analogWrite(PIN9,0);//shut
            analogWrite(PIN10, Output + 150);//Cooling down
    }    else{
               analogWrite(PIN9,0);
               analogWrite(PIN10,0);
      }
 //Serial.println(duration);
 lcd.print(8,1,Tnow);//display the current temperature on app
}

