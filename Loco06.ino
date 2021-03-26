
/*
F1 EF 88 00 Go Forward
F2 E0 58 00  Go Back

*/

#define NEXP 50.


#define PROP   0.001  // PID Proportional coeff. multyplyer
#define INTEGR 0.00000001   // PID Integral coeff. multyplyer 

int Speed,Turn=88,PWMRight,PWMLeft,OldSpeed;
byte ModeLight,j13=15,j13C=100,LightBitz[8],ModeCourse,OldModeCourse,OldTurn;
float Aavr;
volatile int AngleCourse;
volatile long TimeAngle;
int CourseDiff, Course;
float Ap,Ai,I,Rotate;
// void AngleM (void);




// End of Ours Variables
byte commands[4] = {0x00,0x00,0x00,0x00};
//Variables will be used to determine the frequency at which the sensor readings are sent 
//to the phone, and when the last command was received.  
unsigned long timer0 = 2000;  //Stores the time (in millis since execution started) 
unsigned long timer1 = 0;  //Stores the time when the last sensor reading was sent to the phone
//14 byte payload that stores the sensor readings
byte sensors[14] = {0xee,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xcc};
//Constant used to caculate the 9V battery voltage (9.04 mV per step)
float stepSize = 9.04;
//The union allows you to declare a customized data type, in this case it can be either 
//a float or a byte array of size 4. What we need is to store a float which is 4
//bytes long and retrieve each of the 4 bytes separately.
union u_sensor0{
  byte a[4];  
  float b;
}
sensor0;
union u_sensor1{
  byte c[4];  
  float d;
}
sensor1;
int i = 0;

void setup()
{       

 // PIN Mode definitions
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
    pinMode(13, OUTPUT);

attachInterrupt(0,AngleM, CHANGE);  // definition of external interrupt function


    
// Timer 2 configuration. We use timer2 interruption for soft UART transmitter on 9600 bod
 
ASSR=0x00;
TCCR2A=0x02;
TCCR2B=0x02;
TCNT2=0x00;
OCR2A=0xCD;
OCR2B=0x00;
TIMSK2=0x02;



   Serial.begin(9600);  
   Serial.print ("Blut Tooth controlled Loco06  9600 ");
}

void loop()
{

  
  if(Serial.available()==4){   // Here we read 4 bytes of Blue Tooth control
    commands[0] = Serial.read();  //Direction
    commands[1] = Serial.read();  //Speed
    commands[2] = Serial.read();  //Angle
    commands[3] = Serial.read();  //Lights and buttons states
    /*
     Since the last byte yields the servo's angle (between 0-180), it can never be 255. At times, the two
     previous commands pick up incorrect values for the speed and angle. Meaning that they get the direction 
     correct 100% of the time but sometimes get 255 for the speed and 255 for the angle.
     */
    if((commands[2]<=0xb4)&&((commands[0]<=0xf5)&&(commands[0]>=0xf1))){
      //Make sure that the command received involves controlling the car's motors (0xf1,0xf2,0xf3)
      if(commands[0] <= 0xf3){
        if(commands[0] == 0xf1){  //Check if the move forward command was received
          ; //  redCar.forward_1W(commands[1]);
        
          Speed= int (1.068*float(commands[1]));  // Scale the speed absolut value in Go Forward case
        
        }
        else if(commands[0] == 0xf2){  //Check if the move back command was received     
          ; // heretest ; // heretest ; // heretest ; // heretest ; // heretest redCar.back_1W(commands[1]);
        
        Speed= - int (1.068*float(commands[1]));  // Scale the speed absolut value in Go Back case
          
        }
        else{  //Check if the stop command was received    
          ; // heretest ; // heretest ; // heretest ; // heretest ; // heretest redCar.stopped_1W();    
          Speed = 0;
        }       
        //Steer front wheels only if the new angle is not equal to the previous angle
     Turn= commands[2]; //   leftRight.write(commands[2]);          
      }
      else if(commands[0] == 0xf5){
        //Stop everything
        ; // heretest ; // heretest ; // heretest ; // heretest ; // heretest redCar.stopped_1W(); 
        Speed =0;
        //digitalWrite(pinFrontLights,LOW);
        //digitalWrite(pinBackLights,LOW);      
      }
      else{
        //Here you put the code that will control the tilt pan (commands[0] == 0xf4)   
        
      } 
      //Check the front/back lights and other toggles   
      //               _______________________________________________
      //command[3] =  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |  binary
      //              |_____|_____|_____|_____|_____|_____|_____|_____|
      //Buttons ---->  Front  Back  Horn   A     B     C     D     E   
      //Front lights
      if(bitRead(commands[3],7)){
        //digitalWrite(pinFrontLights,HIGH);
          
    ModeLight |= 0x04;
    LightBitz[2]=1;
      }
      else{
          
    ModeLight &= ~0x04;
    LightBitz[2]=0;
        // digitalWrite(pinFrontLights,LOW);
      }        
      
         

    }
    else{
      //Resetting the Serial port (clearing the buffer) in case the bytes are not being read in correct order.
      Serial.end();
      Serial.begin(9600);
    }
  }
      // If no Serial Data Set is not received jet
  else{
    timer0 = millis();  //Get the current time (millis since execution started)
    if((timer0 - timer1)>=477)
    {
      if(1)
      {
      //Check if it has been 477ms since sensor reading were sent
      //Calculate the 9V's voltage by multiplying the step size by the step number (analogRead(0)) 
      //This value will be in mV, which is why it's multiplied by 0.001 to convert into Volts.
      sensor0.b = Ap;  
      //Break the sensor0 float into four bytes for transmission
      sensors[1] = sensor0.a[0];
      sensors[2] = sensor0.a[1];
      sensors[3] = sensor0.a[2];
      sensors[4] = sensor0.a[3];
      //Get sensor 2's reading
      sensor1.d = Ai*100000.;  
      //Break the sensor1 float into four bytes for transmission
      sensors[5] = sensor1.c[0];
      sensors[6] = sensor1.c[1];
      sensors[7] = sensor1.c[2];
      sensors[8] = sensor1.c[3];
      //Get the remaining reading from the analog inputs
      sensors[9] = 0; //map(analogRead(2),0,1023,0,255);
      sensors[10] =byte(128+PWMRight/2); //map(analogRead(3),0,1023,0,255);
      sensors[11] = byte(128+PWMLeft/2);; //map(analogRead(4),0,1023,0,255);
      sensors[12] =   Turn; // ModeLight ; // map(analogRead(5),0,1023,0,255); 
      //Send the six sensor readings
      Serial.write(sensors,14);
      //Store the time when the sensor readings were sent
      }
      else
      {

Serial.print("Ap=");
Serial.print(Ap,4);
 Serial.print(" Ai=");
 Serial.print(Ai,6);

Serial.print(" Trn= ");
Serial.print(Turn);
Serial.print(" MdC= ");
Serial.print(ModeCourse);

Serial.print(" Diff= ");
Serial.print(CourseDiff);

Serial.print(" Rtt= ");
Serial.print(Rotate);

Serial.print(" Cmnd[2]=");
Serial.print(commands[2]);

Serial.print(" I=");
Serial.print(I);


Serial.println("");


      }
       Ap=PROP*analogRead(A6);
       Ai=INTEGR*analogRead(A7);
      timer1 = millis();
    }
  }

// Here Auto Course Stabilisation
// it works only if No Turn selected and the A button is ON

 
 


// Here Motor Control  
// motor control for crane
 PWMRight=  Speed;
PWMLeft= int( (Turn-88)*255/33      );

// END motor control for crane
if(Turn>88)  // if turn left
{
 
PWMLeft=  int (  (  105.-Turn)*Speed  /17.        ) ;  // Scale left motor
if(PWMLeft>255)
PWMLeft=255;

if(PWMLeft<-255)
PWMLeft=-255;
PWMRight=Speed;

}
else if(Turn<88) // if turn rigth
{
//tTurn=  int(255.*float((Time1-NRMANGLE))/(NRMANGLE-LOWANGLE));// Scale rigth motor


PWMRight= int (  (  Turn - 75.)*Speed  /17.        ) ;
//Serial.print(PWMRight);
if(PWMRight<-255)
PWMRight=-255;

if(PWMRight>255)
PWMRight=255;

PWMLeft=Speed;

//Serial.print("  ");
//Serial.println(PWMRight);
}
else 
{
PWMRight=  Speed;
PWMLeft=Speed;

}





if(PWMLeft>255)
PWMLeft=255;

if(PWMLeft<-255)
PWMLeft=-255;
//PWMRight=Speed;
 
if(PWMRight<-255)
PWMRight=-255;

if(PWMRight>255)
PWMRight=255;

 


 

// Control the motors using internal arduino IDE analogWrite () functions 
if(PWMRight>=0)
{
analogWrite(6,PWMRight);
PORTD&= ~0x80;
  
}
else
{
analogWrite(6,255+PWMRight);
PORTD|= 0x80;
  
}

if(PWMLeft>=0)
{
analogWrite(5,PWMLeft);
PORTD&= ~0x10;
  
}
else
{
analogWrite(5,255+PWMLeft);
PORTD|= 0x10;
  
}


  // Here Light Control - we cunstruct the Light Mode to transmit to Light Controller

 if(Turn<88) // if turn left set bit 0x08 to HIGH
    {
    ModeLight|=0x08;
    ModeLight&= ~0x10;
    LightBitz[3]=1;
    LightBitz[4]=0;
    }
    else if (Turn>88)// if turn right set bit 0x08 to HIGH
{
    ModeLight&=  ~0x08;
    ModeLight |= 0x10;

LightBitz[3]=0;
    LightBitz[4]=1;
    
    }
else if ( ( commands[3]&0x40)==0)  
{
ModeLight&=  ~0x08;
  ModeLight&= ~0x10;

LightBitz[3]=0;
    LightBitz[4]=0;
  
}

if(  ( commands[3]&0x40)>0)  
{

  ModeLight|=0x08;
    ModeLight |= 0x10;
    LightBitz[3]=1;
    LightBitz[4]=1;
  
}


if(  ( commands[3]&0x10)  ==0x10 )  // Switch auto course 
 

 ModeCourse=1;
 else
 ModeCourse=0;
  
 if( ModeCourse ==1  && commands[2]==88    && (OldModeCourse ==0  || OldTurn !=88))  // Fix the course angle if button "auto course" become HIGH and no Turn
 {

// Serial.println("here timemmmmm");
  Course=AngleCourse;
 }

OldTurn=commands[2];
OldModeCourse=ModeCourse;


 if(Speed>60)
 {
      ModeLight |= 0x40;
LightBitz[6]=1;
     
 }
      else
      {
      ModeLight &=~0x40;
      LightBitz[6]=0;
}

      if(Speed< -60)
      {
      ModeLight |= 0x20;
        LightBitz[5]=1;
      }
      else
      {
      ModeLight &=~0x20;

          LightBitz[5]=0;
      }

      if(Speed>3)
      {
      ModeLight |=0x01;

          LightBitz[0]=1;
      }
      else
      {
      ModeLight &= ~0x01;
         LightBitz[0]=0;

      }

 if(Speed < -3)
 {
      ModeLight |=0x02;

 LightBitz[1]=1;
 }
      else
      {
      ModeLight &= ~0x02;
         LightBitz[1]=0;

      }
if(j13==100)
{
 
Aavr= 2.*Speed/(NEXP+1)+ (1.- 2.0/(NEXP+1))*Aavr;

//if(  int(Aavr)<Speed)
//Aavr=float(Speed);

if (Speed >= 0   && ( int(Aavr)>Speed             ))
{
ModeLight |= 0x80;
 LightBitz[7]=1;
}
else  
{
ModeLight &= ~0x80;
LightBitz[7]=0;
}

OldSpeed=Speed;
}

}







// UART Emulation

ISR(TIMER2_COMPA_vect)   // Subroutine soft UART but  transmitter only - send byte ModeLight bit by bit
{
//  ModeLight=4;
j13++;

if(j13>9)
;
else if (j13==0)
{
PORTB &= ~0x20;

}

else if (j13==9)
{
PORTB |= 0x20;
}
 else 
{
if((ModeLight & (1<<(j13-1))   )>0)
PORTB |= 0x20;
else
PORTB &= ~0x20;
 
// PORTB^=0x20;
}


}



void AngleM(void) // Subroutine external interruption for Angle pulse measurement

{
if( (PIND&0x04 )  == 0x04)
TimeAngle=micros();
else{
int tAngle = int(micros()-TimeAngle);
if(tAngle>=200  && tAngle<=3800)
AngleCourse=tAngle-200;

}
}
