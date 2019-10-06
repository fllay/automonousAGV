byte ReadBData[8];              //Fill it with incoming Byte string {127, x, y, z1, z2, z3, z4, 64} -  64 is the end x = which part I change, y = which parameter, z1-z4 = set value 
byte WriteBData[8];             //Fill it with outcoming Byte string {127, x, y, z1, z2, z3, z4, 64} -  64 is the end x = which part I change, y = which parameter, z1-z4 = set value
long ReadLData;                 //Convert z1-z4 to a value

byte SpeedRatio = 10;           //Cislo, kterym snizuji nastavenou rychlost. Pro stejnou rychlost 10, pri 20% zpomaleni 12 - pouze pro kameru

void SendRecieveToMotor(byte fn1, byte fn2, long fn3, boolean Recieve)
{
  RepeatAll:                                          //"goto" in case sending fails
  if (!Recieve)                                       //If there is no flag for data recieve
  {
    WriteBData[0] = 127;                              //on first position I put 127 - setting it is outcoming string
    WriteBData[1] = fn1;                              //value given to this function
    WriteBData[2] = fn2;                              //value given to this function
    WriteBData[3] = loB1(fn3);                        //Value fn3 of type long I convert on 4 values of type byte
    WriteBData[4] = loB2(fn3);                        //Value fn3 of type long I convert on 4 values of type byte
    WriteBData[5] = loB3(fn3);                        //Value fn3 of type long I convert on 4 values of type byte
    WriteBData[6] = loB4(fn3);                        //Value fn3 of type long I convert on 4 values of type byte
    WriteBData[7] = 64;                               //the last position in string is a control value 64
    Serial1.write(WriteBData, sizeof(WriteBData));    
  }
  unsigned long currentMillis = millis();             //current number of miliseconds from the start 
  RepeatRecieve:                                      //"goto" in case sending fails
  char c = 0;                                         //Temporary variable in which the incoming communication is saved
  byte Pocet = 0;                                     //Temporary variable in which I count incoming characters
  while ((c != 64) || (Pocet < 7))                    //Repeat until the 64 character is recieved or it exceeds the length
  {
    if(Serial1.available())                           //If there are any data on serial port 1
    {
      c = Serial1.read();                             //Saving first incoming character
      if (Pocet > 0)                                  //If I already saved the opening character
      {
        ReadBData[Pocet] = c;                         //Saving recieved character on its position
        Pocet ++;                                     //Increasing position (number of characters) by 1
      }
      if (((c == 127) || (c == 126) || (c == 125)) && (Pocet == 0))     //When I get the beginning character and it also is the first one, then I save it and 7 more are coming
      {
        ReadBData[0] = c;
        Pocet ++;
      }
    }
    if((millis() - currentMillis) > 10)       //If I dont get 8 characters in 10ms -> sending report about damaged string
      {
        ReadBData[0] = 125;                   //first array position gives the state of communication. 125 - the incoming communication was not correct, repeat
        ReadBData[7] = 64;                    //ending character - evaluation whether the string ok or not is based on first and last character
        Serial1.write(ReadBData, sizeof(ReadBData));    //I send back the recieved data, while changing only the first character
        //Serial.println("Prekrocena doba cteni z komunikacniho arduina");
        goto BezDat;                          //I quit waiting for data so it is possible for the program to continue
      }
    } 
    if (ReadBData[7] != 64)                   //If the last character is not 64, I return error message
    {
      ReadBData[0] = 125;                    //125 - communication is not OK, repeat
      ReadBData[7] = 64;                     
      Serial1.write(ReadBData, sizeof(ReadBData));    //I send back the recieved data, while changing only the first character
     
      goto BezDat;                          //I quit waiting for data so it is possible for the program to continue
    }
    else                                    //If the last character in array is identical with ending character, then I send back message that data are OK
    {
      ReadLData = makeBLong(ReadBData[6], ReadBData[5], ReadBData[4], ReadBData[3]);      //z1-z4 converting to type long
      if (ReadBData[0] = 127)               //New data incoming
      {
        ReadBData[0] = 126;                   
        Update(ReadBData[1], ReadBData[2], ReadLData);    //processing string
        Serial1.write(ReadBData, sizeof(ReadBData));      //I send back the recieved data, while changing only the first character
        //Serial.println("Data prectena v poradku");
      }
      if (ReadBData[0] = 126)               //Incoming communication confirmation
      {
         
      }
      if (ReadBData[0] = 125)               //Incoming signalisation of damaged communication
      {
        
      } 
    }
    BezDat:;
}

//processing and evaluation of incoming string
void Update(byte fn1, byte fn2, long fn3)               //Je ocekavan retezec ve formatu x/y/z/@
{
  /*
  String text = ("Retezec> ");
  text += fn1;
  text += (",");
  text += fn2;
  text += (",");
  text += fn3;
  Serial.println(text);
  */
  switch (fn1)      // x says what is the concern, y specifies element and the value is what we set the element on
  {
  /*case 1:           // 1 Main speed, 2 Regulated speed, 3 manual override or autopilot, 4 regulatory step, 5 STOP, 6 TURN
    switch (fn2)
    {
    case 1:
      if ((SensorType == 1) || (SensorType == 2)) Rizeni.SpeedSet = (fn3 / 10) * 10;        //Pro srovnani rychlostni s mensimi koly
      //For camera AGV we have slow down by 20%
      if (SensorType == 3) Rizeni.SpeedSet = (fn3 / SpeedRatio) * 10;        //for comparison with smaller wheels
      Line.PID = 0;
      Line.RegulaceHodnota = 0;
      Line.RegulaceKrok = 0;
      
      //Line.ZtrataCary = 0;
      break;
    case 2:
      if ((SensorType == 1) || (SensorType == 2)) Rizeni.SpeedR = (fn3 / 10) * 10;        //for comparison with smaller wheels
      //Pro kamerove AGV musim snizit rychlost o 20%
      if (SensorType == 3) Rizeni.SpeedR = (fn3 / SpeedRatio) * 10;        //for comparison with smaller wheels
      Rizeni.SpeedCam = Rizeni.SpeedR;
      Line.PID = 0;
      Line.RegulaceHodnota = 0;
      Line.RegulaceKrok = 0;
      //Line.ZtrataCary = 0;
      break;
    case 3:
      if (fn3 == 1) 
      {
        Rizeni.RucniOvladani = true;
        md.setM1Speed(0);
        md.setM2Speed(0);
        //Rizeni.Stop = true;
      }
      else 
      {
        Rizeni.RucniOvladani = false;
        md.setM1Speed(0);
        md.setM2Speed(0);
        Line.ZtrataCary = 0;
        //Rizeni.Stop = false;
      }
      break;
    case 4:
      //Motor[1].Step = fn3;
      //Motor[2].Step = fn3;
      if (fn3 == 1) {
        Rizeni.CrashSensor = false;
      }
      if (fn3 == 4) {
        Rizeni.CrashSensor = true;
      }
      break;
    /*case 5:
      if (fn3 == 1){
        Rizeni.Stop = true;
      }
      else {
        Rizeni.Stop = false;
        Line.ZmenaN = 0;
        Line.ZtrataCary = 0;
      }
      break;  */
   /* case 6:            //Command for turning or riding straight forward
      switch (fn3)
      {
        case 1:        //Straight forward   
          Odboc = false;  
          break;
        case 2:        //Turn left   
          Odboc = true;
          OdbocLevo = true;
          break;
        case 3:        //Turn right    
          Odboc = true;
          OdbocLevo = false;
      }   
      break;
    case 7:            //Command for the direction prependicular to the previous direction
      switch (fn3)
      {
        case 1:        //stop turning   
          Rizeni.Otocka = false;
          Rizeni.Otacim = false;
          break;
        case 2:        //Go left     
          Rizeni.Otocka = true;
          Rizeni.RightOtocka = false;
          Rizeni.Otacim = false;
          break;
        case 3:        //Go right
          Rizeni.Otocka = true;
          Rizeni.RightOtocka = true;
          Rizeni.Otacim = false;
          break;
      }
      break;
    case 8:            //Command for allowing and prohibiting slowing based on camera
      switch (fn3) {
        case 1:        //Turn off slowing down by camera 
          Set_CameraSlowDownEnable(false);
          break;
        case 2:        //Turn on slowing down by camera 
          Set_CameraSlowDownEnable(true);
          break;
      }   
      break;
    }
    break;
  case 2:         // Incoming data concern the sensors - 1 Koef, 2 kp, 3 ki, 4 kd, 5 frequency of update in ms
    switch (fn2)
    {
    case 1:
      //Line.Koef = (-1)*fn3;
      break;
    case 2:
      //Line.kp = fn3;
      break;
    case 3:
      //Line.ki = fn3;
      break;
    case 4:
      //Line.kd = fn3;
      break;
    case 5:
      //Line.Interval = fn3;
      break;
    }
    break;
  case 3:        // Incoming data control motors directly
    switch (fn2)
    {
    case 1:
      md.setM1Speed(fn3);
      break;
    case 2:
      md.setM2Speed(fn3);
      break;
    }
    break;
  case 4:        // not used
    break;
    */
 case 5:        // not used
    bPLCcommOK = true;
    switch (fn2)
    {
    case 1: //"ERRO"       
      TAGcode = ERRO;  
      break;
    case 2: //"WALK"
      //md.setSpeed(100,100); //run  
      TAGcode = WALK; 
      //Serial.println ("WALK");
      Odboc = false;  
      Rizeni.Stop = false;  
      Rizeni.CrashSensor = false;
      Rizeni.Otocka = false;
          Rizeni.Otacim = false;
      //Rizeni.RucniOvladani = false;
      break;
    case 3: //"STOP"
      //md.setSpeed(0,0); //stop 
      TAGcode = STOP;  
      //Serial.println ("STOP");
      //Rizeni.RucniOvladani = true;
        md.setM1Speed(0);
        md.setM2Speed(0);
      //md.setSpeeds(0,0);
      Rizeni.Stop = true;
    //Line.RegulaceHodnota = 0;
      //Line.RegulaceKrok = 0;
      break;
    case 4: //"PASS" 
      TAGcode = PASS; 
      //md.setSpeeds(200,200); 
      break;
    case 5: //"----"
      switch (fn3)
      {
       case 1:        //Straight forward   
          Odboc = false;
          OdbocLevo = false;
          Rizeni.CrashSensor = false;
          Rizeni.Otocka = false;
          Rizeni.Otacim = false;
          TAGcode = WALK;  
          break;
        case 2:        //Turn left   
          Odboc = true;
          OdbocLevo = true;
          Rizeni.Otocka = true;
          Rizeni.RightOtocka = false;
          //Rizeni.Otacim = false;
          TAGcode = LEFT;
          break;
        case 3:        //Turn right    
          Odboc = true;
          OdbocLevo = false;
          Rizeni.Otocka = true;
          Rizeni.RightOtocka = true;
          //Rizeni.Otacim = false;
          TAGcode = RIGT;
          //md.setSpeeds(0,0); delay(200);
          //Rizeni.SpeedR = MinSpeedCam; //x.. slowdown
          break;
        default:
          Odboc = true;
          OdbocLevo = false;
          TAGcode = ERRO;
          break;
      }
    }   
    
  default:        // recieved unknown string - returning error message
    //Serial.println("*0/0/0/@");
    break;
  }
}
