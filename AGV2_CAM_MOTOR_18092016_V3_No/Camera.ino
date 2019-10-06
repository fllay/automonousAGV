const byte NumOfByte = 10;
char chReadData[NumOfByte];
unsigned long CameraPrevMillis = 0;
int test;
unsigned long Get_CameraPrevMillis() {
  return CameraPrevMillis;
}

unsigned long Set_CameraPrevMillis(unsigned long Val) {
  CameraPrevMillis = Val;
}

void LineError() {
  Line.ZtrataCary ++;                                     //Counting the number of consecutive error readings
  Line.RegulaceHodnota = 0;
  Line.RegulaceKrok = 0;
  Line.MamCaru = 0;
  Line.PID = 0;
  if ((Line.ZtrataCary % 5) == 0) {                       //If the residue after integer division is 5 (with every fifth iteration of this function)
    MotorInterval = 10;
    Motor[1].Step = 2;
    Motor[2].Step = 2;
    md.setSpeeds(Motor[1].Speed, Motor[2].Speed);         //Setting smae speed for both motors (continuing going straight without regulation) - the calculation of speed itself undergoes in function Stop in main procedure
  }
}

//regulatory deviation is calculated from the position of guiding line
void CountPID(int8_t Val) {
  long PID = 0;
  int pKoef;
  Line.MamCaru ++; //count no of seeing line
  if ((Line.ZtrataCary >= 5) && (Line.MamCaru < 5)) goto Skyp; 
  if (Line.ZtrataCary >= 5) {                             //If there was a line loss and now it is seen again under one of middle sensors (for mid sensors I must state number 8, for all sensors 15)
    Line.ZmenaN = 0;                                      //Deleting cumulated value of line position (preventing AGV from having too big PID from standing over outer line)
    Rizeni.SpeedR = Rizeni.SpeedCam;                      
  } 
  Line.ZtrataCary = 0;                                    //reseting error count
  /*
  Calculation of deflection koeficient from measured value.
  (Line.kp * HodnotaCary) - is regulatory deviation given by current line position - middle effect on the outcome (moderate koeficient 0.2)
  (Line.ki * Line.ZmenaN) - is regulatory deviation given by cumulated error - greatest effect on the outcome (smallest koeficient 0.07)
  (Line.kd * (HodnotaCary - Line.Zmena)) - is regulatory deviation given by line shifting compared to previous position - greatest effect on outcome (greatest koeficient 5.0)
  (Koef) - all the koeficients are smaller than 1, so we multiply it by 100 and compare the result
  */
  if (SensorType == 3) pKoef = 8000 - ((400 - Motor[1].Speed) * 10);
  if ((SensorType == 2) || (SensorType == 1)) pKoef = Line.Koef;
  PID = ((Line.kp * Val) + (Line.ki * Line.ZmenaN) + (Line.kd * (Val - Line.Zmena))) / (pKoef);
  Line.Zmena = Val;                                       //Saving current value as previous
  Line.ZmenaN += Val;                                     //Cumulated error
  if (SensorType == 1) PID = PID / -1;                         //Original calculation for analog optical sensors
  if (SensorType == 2) PID = PID / -5;                    //Magnetic sensors have greater resolution, so we have to decrease PID
  if (SensorType == 3) PID = PID / -4;                    //Camera has greater resolution, so we have to decrease PID
  if (!Rizeni.RucniOvladani) {                            //If manual control is deactivated
    long MaxTurn = PID;                                   //Assigning calculated PID into value of max turn
    int SpeedAkt = Motor[1].Speed;                        //x.. Cap the cal PID value to the current speed of Motor
    if (abs(MaxTurn) > abs(SpeedAkt)) {                   //If the calculated deviation is greater than current speed
      if (MaxTurn >= 0) {                                 //If PID is possitive
        MaxTurn = abs(SpeedAkt);                         
      } else {                                            //PID negative
        MaxTurn = abs(SpeedAkt) * (-1);                   
      }
    }
    Line.PID = MaxTurn;
    int PomProm = Line.Interval / Line.RegulaceInterval;  //Calculation to determine number of regulations between readings
    int PomProm2 = MaxTurn - Line.RegulaceHodnota;        //Calculating the difference between current and desired regulation
    Line.RegulaceKrok = (PomProm2 / PomProm);               //Calculating regulatory step
    //Serial.println(String(Val) + "\t" + String(PID) + "\t" + String(md.getM1CurrentMilliamps()) + "\t" + String(md.getM2CurrentMilliamps())); 
  }
  Skyp:;
}

byte CountBigDiff = 0;
byte MinSpeedCam = 80;        //80

boolean CameraSlowDownEnable = true;     //Flag deciding if slowing ased on camera information is allowed
void Set_CameraSlowDownEnable(boolean Val) {
  CameraSlowDownEnable = Val;
}

void CamSlowDown1(byte LineVal) {
  if (CameraSlowDownEnable) {
    CountBigDiff++;
    if (CountBigDiff == 1) {
      Rizeni.CameraSlowDown = true;
      CountBigDiff = 0;
      byte bPom = 10+((abs(LineVal)/3)-2);
      Rizeni.SpeedR = (Rizeni.SpeedR/bPom)*10;
      if (Rizeni.SpeedR < MinSpeedCam) {
        if (Rizeni.SpeedCam > MinSpeedCam) Rizeni.SpeedR = MinSpeedCam; else Rizeni.SpeedR = Rizeni.SpeedCam;
      }
    }
  }
}

void CamSlowDown(byte LineVal) {
  if (CameraSlowDownEnable) {
    CountBigDiff++;
    if (CountBigDiff == 1) {
      Rizeni.CameraSlowDown = true;
      CountBigDiff = 0;
      byte bPom = 10+((LineVal/2)-2);
      if (LineVal > 8) bPom = 2*(10+((LineVal))); //x...if (LineVal > 5) bPom = 6*(10+((LineVal)));
      //CameraBrakingStep = 1200;
      Rizeni.SpeedR = (Rizeni.SpeedCam/bPom)*8;
      if (Rizeni.SpeedR < MinSpeedCam) {
        if (Rizeni.SpeedCam > MinSpeedCam) Rizeni.SpeedR = MinSpeedCam; else Rizeni.SpeedR = Rizeni.SpeedCam;
      }
    }
  }
}

void CamSpeedUp() {
  CountBigDiff = 0;
  Rizeni.CameraSlowDown = false;
  if (Rizeni.SpeedR < Rizeni.SpeedCam) Rizeni.SpeedR = (Rizeni.SpeedR*11)/10;
  if (Rizeni.SpeedR > Rizeni.SpeedCam) Rizeni.SpeedR = Rizeni.SpeedCam;
}
      
char PrevLineValNow = 0;                 //Value of previously seen line. At start it is equal to 0
char PrevLineValNext = 0;                //Value of previously seen line. At start it is equal to 0
        
void GoFront(byte Next, byte End) {
  char Diff = 127;
  char LineValNow = 0;
  char LineValNext = 0;
  byte LineCountNow = Next - 1;
  byte LineCountNext = End - Next - 1;
  boolean LineErrorNow = false;
  boolean LineErrorNext = false;
  
  if (LineCountNow > 1) {                //If it sees more than one line and it has to go straight (concurrence of two lines)
    for (int i=1; i<Next; i++) {         //Checking through all seen lines and searching for the one closest to original position
      char tDiff = abs(PrevLineValNow - chReadData[i]);    //Calculating distance from line that was seen in previous reading
      if (tDiff < Diff) {                //If that distance is smaller than the smallest found
        Diff = tDiff;                    //Saving it as the smallest
        LineValNow = chReadData[i];      //Saving this position as the right one for control
      }
    }
  } else {
    if (LineCountNow < 1) {
      LineErrorNow = true;
      LineValNow = 0;
    } else {
      LineValNow = chReadData[1];          //If there is seen only 1 line
    }
  }
  PrevLineValNow = LineValNow;
  
  Diff = 127;
  if (LineCountNext > 1) {               //If it sees more than one line and it has to go straight (concurrence of two lines)
    for (int i=Next+1; i<End; i++) {     //Checking through all seen lines and searching for the one closest to original position
      char tDiff = abs(PrevLineValNext - chReadData[i]);    //Calculating distance from line that was seen in previous reading
      if (tDiff < Diff) {                //If that distance is smaller than the smallest found
        Diff = tDiff;                    //Saving it as the smallest
        LineValNext = chReadData[i];     //Saving this position as the right one for control
      }
    }
  } else {
    if (LineCountNext < 1) {
      LineErrorNext = true;
      LineValNext = 0;
    } else {
      LineValNext = chReadData[Next+1];          //If there is seen only 1 line
    }
  }
  PrevLineValNext = LineValNext;  
  if ((!LineErrorNext) && (!LineErrorNow)) {
    //If the future line is closer to center, then it regulates using the future line
    if ((abs(LineValNext) < abs(LineValNow)) && (abs(LineValNow) < 10)) //x..10
      CountPID(LineValNext); 
    else 
      CountPID(LineValNow);  
    //Serial.println("Aktualni i vzdalena OK");
    if (abs(LineValNext) > 40) {  //x...
      CamSlowDown(abs(LineValNext));
    } else if (abs(abs(LineValNow)-abs(LineValNext)) > 20) {  
      CamSlowDown((abs(abs(LineValNow)-abs(LineValNext)))*4);
    } else {
      CamSpeedUp();
    }
  } else if (!LineErrorNow) {
    CountPID(LineValNow);
    if (abs(LineValNow) > 6) {
      CamSlowDown(abs(LineValNow));
    } else {
      CamSpeedUp();
    }
    //Serial.println("Pouze Aktualni");
  } else if (!LineErrorNext) {
    CountPID(LineValNext);
    if (abs(LineValNext) > 6) {
      CamSlowDown(abs(LineValNext));
    } else {
      CamSpeedUp();
    }
    //Serial.println("Pouze vzdalena");
  } else {
    LineError();
  }
}

void Turn(byte Next, byte End) {
  char Max;
  char LineValNow = 0;
  char LineValNext = 0;
  byte LineCountNow = Next - 1;
  byte LineCountNext = End - Next - 1;
  
  if (LineCountNow > 1) {                //If it sees more than one line and it has to go straight (concurrence of two lines)
    if (OdbocLevo) Max = -128; else Max = 127; 
    for (int i=1; i<Next; i++) {         //Checking through all seen lines and searching for the most left or right
      if (OdbocLevo) {
        if (Max < chReadData[i]) {       //Searching for greatest positive value (positive means right side of sensors) 
          Max = chReadData[i];
        }
      } else {
        if (Max > chReadData[i]) {       //Searching for greatest negative value (negative means left side of sensors) 
          Max = chReadData[i];
        }  
      }
    }
    LineValNow = Max;
    
  } else {
    if (LineCountNow < 1) {
      LineError();
      LineValNow = 0;
    } else {
      LineValNow = chReadData[1];         //If there is seen only 1 line
      
    }
  }
  
  if (LineCountNext > 1) {               //If it sees more than one line and it has to go straight (concurrence of two lines)
    if (OdbocLevo) Max = -128; else Max = 127; 
    for (int i=Next+1; i<End; i++) {     //Checking through all seen lines and searching for the one closest to original position
      if (OdbocLevo) {
        if (Max < chReadData[i]) {       //Searching for greatest positive value (positive means right side of sensors) 
          Max = chReadData[i];
        }
      } else {
        if (Max > chReadData[i]) {      //Searching for greatest negative value (negative means left side of sensors) 
          Max = chReadData[i];
        }  
      }
    }
    LineValNext = Max;
    //Serial.print(F("Dve cary a pozice spravne cary budoucnost: ")); Serial.print(LineValNext, DEC); Serial.println(" *");
  } else {
    if (LineCountNext < 1) {
      LineError();
      LineValNext = 0;
    } else {
      LineValNext = chReadData[Next+1];          //If there is seen only 1 line
      //Odboc = false;                             //x.. if already turn and see only one line then Go straight
      //Serial.print(F("Jedna cara a pozice spravne cary budoucnost: ")); Serial.print(LineValNext, DEC); Serial.println(" *");
    }
  }
  char LineValAkt;
  if (LineCountNext > 1) {
    if (LineCountNow <= 1) {
      LineValAkt = LineValNext;
      //Serial.println(F("More than one line in future and one or less in present"));
      Serial.println(F("F>1 and P=<1"));
    } else {
      //Serial.println(F("More than one line in future and 2 lines in present"));
      Serial.println(F("F>1 and P=2"));
      LineValAkt = LineValNext;    //x..try next
    }
  } else {
    if (LineCountNext = 1) {
      if (LineCountNow < 1) {
        //Serial.println(F("One line in future and no line in present"));
        Serial.println(F("F=1 and P=0"));
        LineValAkt = LineValNext;
      } else {
        //Serial.println(F("One line in future and one line in present"));
        Serial.println(F("F=1 and P=1"));
        LineValAkt = LineValNow;  
        //Odboc = false; //x...
        //TAGcode = WALK;
      }
    } else {
      //Serial.println(F("No line in future"));
      Serial.println(F("F=0"));
      LineValAkt = LineValNow;
    }
  }
  
  if (abs(LineValAkt) > 30) //x.. limit to +/-25
  { Odboc = false; 
      if(LineValAkt>0)
        LineValAkt=30;
      else
        LineValAkt=-30;
   }
  //x else
   
  CountPID(LineValAkt);
  
  if (OdbocLevo) Serial.print("LEFT "); else Serial.print("RIGHT ");
  Serial.print("Turning... LineValAkt= "); Serial.println(LineValAkt, DEC);
  
}

void LineMCameraTest() {
  boolean ReadOK = false;
  unsigned long Start = millis();
  byte poz = 0;
  byte pozNext = 0;
  while (((millis() - Start) < 20) && (!ReadOK)) {
    if (Serial.available()) {
      char chRead = Serial.read();
      chReadData[poz] = chRead;
      if (chRead == 59) pozNext = poz;
      if (chRead == 64) ReadOK = true; else poz++;
    }
  }
  /*
  for(int i=0; i<poz; i++) {
    Serial.print(String((byte)chReadData[i]) + "; ");
  }
  Serial.println();
  */
  //Position "poz" gives complete length of recieved string and position "pozNext" gives the position of separatorof current and future line - number of characters between 0 and "pozNext" gives the number of lines in presence and between "pozNext" and "poz" it gives the number of lines in future
  if ((chReadData[0] == 59) && (chReadData[pozNext] == 59) && (chReadData[poz] == 64)) {           //Data have been correctly read
    for(int i=1; i<poz; i++) {
      chReadData[i] = chReadData[i] - 48;
    }
    ReadOK = true;
    if (!Odboc) {
      GoFront(pozNext, poz);
    } else {
      Turn(pozNext, poz);
    }
  } else {
    ReadOK = false;
    Serial.println("Line data error");
    LineError();
  }
}


void LineMCamera() {
  boolean ReadOK = false;
  unsigned long Start = millis();
  byte poz = 0;
  byte pozNext = 0;
  while (((millis() - Start) < 10) && (!ReadOK)) {
    if (Serial3.available()) {
      char chRead = Serial3.read();
      chReadData[poz] = chRead;
      if (chRead == -128) pozNext = poz;
      if (chRead == 127) ReadOK = true; else poz++;
    }
  }
  
  /*
  for(int i=0; i<=poz; i++) {
    Serial.print(chReadData[i], DEC);
    Serial.print("; ");
  }
  Serial.println();
  */
  
  //Position "poz" gives complete length of recieved string and position "pozNext" gives the position of separatorof current and future line 
  //- number of characters between 0 and "pozNext" gives the number of lines in presence and between "pozNext" and "poz" it gives the number of lines in future
  if ((chReadData[0] == -128) && (chReadData[pozNext] == -128) && (chReadData[poz] == 127)) {           //Data have been correctly read
    if (poz>2) {
      for (int i=1; i<pozNext; i++) {
        Serial.print(chReadData[i], DEC);
        Serial.print(", ");
      }
      Serial.print(" | ");
      for (int i=pozNext+1; i<poz; i++) {
        Serial.print(chReadData[i], DEC);
        Serial.print(", ");
      }
      Serial.println();
      
      
    }
    ReadOK = true;
    if (!Odboc) {
      GoFront(pozNext, poz);
      //Serial.println("Go straight..");
    } else {
      Turn(pozNext, poz);
      //Serial.println("Turning..");
    }
  } else {
    ReadOK = false;
    Serial.println("Line data error");
    LineError();
  }
}
 

