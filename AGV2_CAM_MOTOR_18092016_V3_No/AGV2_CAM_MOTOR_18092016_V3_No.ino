const int NUMBER_OF_ELEMENTS = 6;
const char Message0000 [] PROGMEM = "**************  Verze programu  ****************"; 
const char Message0001 [] PROGMEM = "30.3.2016"; 
const char Message0002 [] PROGMEM = " - slowing down while detecting risk of losing line for camera + option of deactivating by tag";
const char Message0003 [] PROGMEM = " - choosing between analog optical sensor, magnetic and camera";
const char Message0004 [] PROGMEM = " - deleting oscilation while slowing down or detecting an obstacle";
const char Message0005 [] PROGMEM = "************************************************";
const char * const SoftwareVersion[NUMBER_OF_ELEMENTS] PROGMEM = { Message0000, Message0001, Message0002, Message0003, Message0004, Message0005};

#include "ModbusBLVmotor.h"
ModbusBLVmotor md;            //assigning the motor control into variable md

const byte SensorType = 3;            //1 - infrared sensors, 2 - magnet, 3 - camera

#include <digitalWriteFast.h>


//x.. define TAGCODE as byte int instead of String to avoid unidentify values
#define ERRO 0
#define WALK 1
#define STOP 2
#define LEFT 3
#define RIGT 4
#define PASS 5


/*
Converts value of type long on 4 values of byte and back
*/
#define makeBLong(B4, B3, B2, B1)  (((long) B4) << 24 | ((long) B3) << 16 | ((long) B2) << 8 | (B1))
#define loB4(w) ((w) >> 24)
#define loB3(w) ((w) >> 16 & 0xff)
#define loB2(w) ((w) >> 8 & 0xff)
#define loB1(w) ((w) & 0xff)

// Definition of accuracy
const unsigned char PS_16 = (1 << ADPS2);                                      //the worst accuracy and also the greatest speed
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

//Motor definition
unsigned long MotorPrevMillis = 0;      //time of the last motor speed check in milisec 
long MotorInterval = 10;                //period of motor speed regulation in ms - just for start and braking, turning is dealt with in Line

//Camera Interval
int NormalLineInterval = 10;

typedef struct
{
  int SpeedSet;         //Pre-set speed
  int Speed;            //Current speed
  int Step;             //Step value while speeding up or braking
} 
Motor_type;
Motor_type Motor[3]=    //For 2 motors (zero index is not used)
{
  {0, 0, 1},
  {0, 0, 1},
  {0, 0, 1}
};

//Definition of control
typedef struct
{
  int SpeedSet;         //Pre-set speed
  int SpeedR;           //Regulated speed - gives the top speed with respect for line loss, stop command etc...
  int SpeedCam;         //Speed calculated using the information from camera
  boolean CrashSensor;           //Flag for something in front of impact sensors
  boolean CameraSlowDown;        //Flag for slowing down if camera detects too fast enterance into the crossroad
  boolean RucniOvladani;         //If True - turns off line following and turns on manual override
  boolean Stop;                  //If True - The AGV stops and doesnt move until False
  boolean Otocka;                //If True - I set the top speed as MaxSOtocka and search for the end of line. Once I find it, I turn by RightOtocka until I find the line again
  boolean RightOtocka;           //The direction of turning while turning perpendicularly
  int MaxSOtocka;                //Wheel top speed while turning
  boolean Otacim;                //Flag carrying message if I began turning
} 
Rizeni_type;
Rizeni_type Rizeni = {350, 350, 350, false ,false, false, true, false, false, 120, false};


/*
-100, 1500, 45, 4000


*/
//Definice cary

typedef struct
{
  long Koef;            //koeficient of calculated PID - smaller value = not that big oscillation but doesnt follow line that good 
  long kp;              //koeficient of current position - smaller value = smaller oscillation depending on current position  
  long ki;              //koeficient of summed error - smallest value, in calculation we will divide by 100        
  long kd;              //koeficient of direction change - smaller value - more disturbed on straight line      
  long Zmena;           //value of the last  change in turning
  long ZmenaN;          //summed value of changes in turning
  long PID;             //Result of regulation calculation
  unsigned long ZtrataCary;      //count of how many times in a row I havent seen the line
  unsigned long PrevMillis;      //time of the last motor speed check in milisec
  unsigned long Interval;        //time period of motor speed check in milisec
  unsigned long MamCaru;         //count of how many times in a row I have seen the line
  unsigned long RegulacePrevMillis;  //time of previous regulation depending on PID
  int RegulaceInterval;              //Period of time in which I am regulating
  int RegulaceKrok;                  //step of change
  int RegulaceHodnota;               //the value of regulation
} 
Line_type;
Line_type Line = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0};

boolean Odboc = false;           //Flag containing information about turning or going straight
boolean OdbocLevo = false;       //Flag containing information about which direction is the turning happening

int Dioda = 13;

boolean bPLCcommOK = false;

byte TAGcode = 0; 
byte oldTAGcode = 0; //x.. for backup the previous valid tagcode
String sTAGcode = "xxxx";

void setup()
{
  if ((SensorType == 1) || (SensorType == 2)) {
    NormalLineInterval = 10;
    Line.Koef = 100;
    Line.kp = 1500;
    Line.ki = 4;
    Line.kd = 4000;
  }
  if (SensorType == 3) {
    NormalLineInterval = 1000/24;                        //For camera with FPS 24 1000/24
    Line.Koef = -10000;    //-10000
    Line.kp = 40000;       //40000
    Line.ki = 3;           //3
    Line.kd = 20000;      //200000  //x..
  }
  Line.Interval = NormalLineInterval;      //x make it 100ms (digital)
  Line.RegulaceInterval = Line.Interval / 1;  //x.. try to 1 = singgle step  5

  Serial1.begin(115200);                 //Communication with arduino mega    
  Serial.begin(115200);                  //Communication with PC
  Serial2.begin(115200,SERIAL_8E1);               //Communication witch motor
  Serial3.begin(115200);                 //Communication with Raspberry Pi
  
  for (int i = 0; i < NUMBER_OF_ELEMENTS; i++) {
    char * ptr = (char *) pgm_read_word (&SoftwareVersion [i]);
    char buffer [150]; 
    strcpy_P (buffer, ptr);
    Serial.println (buffer);
    delay(100);                          //Pause to secure that the serial port to RPi doesnt overload
  }
  Serial.println ();
  
  md.init();                             //Initialization of motor control
  pinModeFast(Dioda, OUTPUT);
  digitalWriteFast(Dioda, LOW);
  pinModeFast(3, INPUT);
}

void loop() {
  
  //delay(500);
  //Common part for all sensors
  unsigned long currentMillis = millis();                 //saving current time in ms into currentMillis variable
  
  if(Serial1.available()) {
    SendRecieveToMotor(0, 0, 0, true);
  }
  
   /*if (md.getM1Fault()) {
      md.setSpeeds(0, 0);
      SendRecieveToMotor(21, 1, 1, false);
      delay(2000);
    }
    if (md.getM1Fault()) {
      md.setSpeeds(0, 0);
      SendRecieveToMotor(21, 2, 1, false);
      delay(2000);
    }*/
    
   switch (TAGcode)
   {
     case 0: sTAGcode = "ERRO"; break;
     case 1: sTAGcode = "WALK"; break;
     case 2: sTAGcode = "STOP"; break;
     case 3: sTAGcode = "LEFT"; break;
     case 4: sTAGcode = "RIGT"; break;
     case 5: sTAGcode = "PASS"; break;
     default: sTAGcode = "----"; break;
   }

   Serial.print(">>TAGCODE = "); Serial.println(sTAGcode);
   
   //x... Check and recover TAGcode to walk straight
   if (TAGcode != ERRO && TAGcode != WALK && TAGcode != LEFT && TAGcode != RIGT && TAGcode != PASS)
    TAGcode == oldTAGcode; 
   else
    oldTAGcode = TAGcode;
    
    if (TAGcode == PASS)
    {
      Motor[1].Step = 4;
      Motor[2].Step = 4;
      Rizeni.CrashSensor = true;
    }
   
   if (TAGcode == STOP) //|| !TAGcode.equals("WALK")|| !TAGcode.equals("LEFT")|| !TAGcode.equals("RIGT")) //found RFID then check the TAG
   {
      //md.setBrakes(000,000);
      md.setM1Speed(0);
      md.setM2Speed(0);
      md.setSpeeds(0,0); //stop
      Motor[1].Speed = 0;
      Motor[2].Speed = 0;
      Line.RegulaceHodnota = 0;
      Line.RegulaceKrok = 0;
      Rizeni.Stop = true;
      //Line.RegulaceHodnota = 0;
      //Line.RegulaceKrok = 0;
   }
   
   if (TAGcode == WALK ||TAGcode == LEFT || TAGcode == RIGT || TAGcode == PASS ) //line tracking as normal
   {    
    //md.setSpeeds(100, 100);
    if((currentMillis - Line.RegulacePrevMillis) > 100) //Line.RegulaceInterval) //x..
    { //In pre-set it commences regulation with the calculated PID
      Line.RegulacePrevMillis = currentMillis;   
          
      if (Line.ZtrataCary == 0) {
        Line.RegulaceHodnota += Line.RegulaceKrok;                          //With every iteration it changes the regulation by one step
        md.setSpeeds(Motor[1].Speed - Line.RegulaceHodnota, Motor[2].Speed + Line.RegulaceHodnota);  //Sending this newly calculated value to motors
        
        Serial.print("RegulaceHodnota = "); Serial.println(Line.RegulaceHodnota,DEC);
        Serial.print("M1, M2 Real speed = ");
        Serial.print(Motor[1].Speed - Line.RegulaceHodnota,DEC); Serial.print(",");    
        Serial.println(Motor[2].Speed + Line.RegulaceHodnota,DEC);   
     
      }
    }
    
    if((currentMillis - MotorPrevMillis) > MotorInterval) { //In pre-set it commences regulation of speed
      MotorPrevMillis = currentMillis;                      
      SetSpeed();
      /*
      Serial.print("M1, M2 Base speed = ");
      Serial.print(Motor[1].Speed,DEC); Serial.print(",");    
      Serial.println(Motor[2].Speed,DEC);
      */     
    }
   
   
  
    //Control using camera  //x... should try to move outside the main if condition
    if (SensorType == 3) {
      if(Serial3.available()) {
        Set_CameraPrevMillis(currentMillis);
        LineMCamera();
      }
      if((currentMillis - Get_CameraPrevMillis()) > ((NormalLineInterval * 15) / 10)) {   //Alerting line error if the interval for data reception is exceeded
        Set_CameraPrevMillis(currentMillis);
        LineError();
        Serial.println("Camera Communication ERROR!");
      }
      if(Serial.available()) {
        LineMCameraTest();
      }
    }  
    
  }
   
  //Control using analog optical sensors or magnetic tape
  if ((SensorType == 1) || (SensorType == 2)) {
    if((currentMillis - Line.PrevMillis) > Line.Interval) { //In pre-set interval commences line detection and its evaluation
      Line.PrevMillis = currentMillis;                      
      if (SensorType == 1) LineMAnalog();                   //New analog sensors
      if (SensorType == 2) LineMagnetSerial();              //Magnetic line communicating using serial port
    } 
    if(Serial3.available()) {
      DecodeMagnet();
    }
  }
}
