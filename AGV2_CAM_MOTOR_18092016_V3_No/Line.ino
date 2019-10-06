boolean White = true;                                       //Flag telling if I am on white line - false means im on black
typedef struct
{
  int Pin;              //Pin to which sensor is connected
  int PinM;             //Pin to which is magnetic sensor connected
  int ValLeft;          //Line value for specific sensor from left
  int ValRight;         //Line value for specific sensor from right
  int Val;              //Line value for optical sensors
  int AnalogVal;        //Measured value from optical sensors
} 
lSenzorA_type;
lSenzorA_type lSenzorA[9] =         //For 8 analog sensors (0 index isnt used) - Filling with current setting depending on request for driving direction
{
  {99, 0, 0, 0, 0, 0},
  {A8, 99, 0, 0, -7, 0},            //At creform AGV the outer sensor doesnt work, need to ban it
  {A9, A12, 1, -25, -5, 0},
  {A10, A11, 3, -17, -3, 0},
  {A11, A10, 7, -11, -1, 0},
  {A12, A9, 11, -7, 1, 0},
  {A13, A8, 17, -3, 3, 0},
  {A14, A14, 25, -1, 5, 0},
  {A15, 99, 0, 0, 7, 0}             //At creform AGV the outer sensor doesnt work, need to ban it
};

boolean bLineError;

void LineMagnetSerial() {
  Serial3.write("?D\r");
  if (Odboc) {
    if (OdbocLevo) {
      Serial3.write("?TS 1\r");
    } else {
      Serial3.write("?TS 2\r");
    }
  } else {
    Serial3.write("?T\r");
  }
}

void DecodeMagnet() {
  char c;
  String Reply = "";
  boolean bReply = true;
  String sType = "";
  int iValue = 0;
  boolean bRov = false;
  boolean bMinus = false;
  while (c != 13) {
    if(Serial3.available()) {                         //If there are data available in port 3 (other than USB)
      c = Serial3.read();                             //Loading incoming character in variable "c"
      if (c=='?') bReply = false;
      if (bReply) {
        if (c=='=') bRov = true;
        if (c=='-') bMinus = true;
        if (!bRov) sType = sType + c;
        if ((bRov) && (c!='=')&& (c>=48) && (c<=57)) iValue = (iValue * 10) + (c-48);
      }
    }
  }
  if (bMinus) iValue = iValue * (-1);
  //if (bReply) Serial.println(sType + "\t" + iValue);
  if (sType == "D") {
    if (iValue == 1) bLineError = false; else bLineError = true;
  }
  if ((!LineError) && ((sType == "T") || (sType == "TS"))) {
    CountPID(iValue);
  } else {
    LineError();
  }
}

//Detecting guiding line and evaluating it
void LineMAnalog() { 
  //long Zacatek = micros();
  int ByteVal = 0;                                          //Byte value of seen sensors
  byte PocCar = 0;                                          //Gives the number of sensors that see the line
  byte PozCary = 0;                                         //Gives the position of sensor that has seen the line as last
  int HodnotaCary = 0;                                      //Summed up valuse from all sensors tha see the line
  int MaxValAnalog = 0;                                     //Minimal measured analog value... decision between black and white is based on this value
  int MinValAnalog = 1025;                                  //Maximal measured analog value... decision between black and white is based on this value
  String Retezec1 = "";                                     //Data string to be sent to PC
  int MaxLineVal = 15;
  for (int i = 8; i >= 1; i--) {                            //Reading sensors
    if (lSenzorA[i].Pin != 99) {                            //If the sensor pin is set to a valid value
      lSenzorA[i].AnalogVal = analogRead(lSenzorA[i].Pin);                                //Reading analog value from sensor pin
      
      if ((lSenzorA[i].AnalogVal >= MaxValAnalog) && (lSenzorA[i].AnalogVal > 100)) MaxValAnalog = lSenzorA[i].AnalogVal;    //If the measured value is greater than the current maximum, save it as maximum
      if ((lSenzorA[i].AnalogVal <= MinValAnalog) && (lSenzorA[i].AnalogVal > 10)) MinValAnalog = lSenzorA[i].AnalogVal;     //If the measured value is greater than the current minimum, save it as minimum
      Retezec1 = Retezec1 + lSenzorA[i].AnalogVal + "\t";                                 //Saving data from sensors into a string
      
      if ((lSenzorA[i].AnalogVal < 150) && (Rizeni.Otocka)) lSenzorA[i].AnalogVal = 0;    //Attempt to set aside random line vision while rotating
      if ((lSenzorA[i].AnalogVal < 150) && (Odboc)) lSenzorA[i].AnalogVal = 0;            //Attempt to set aside random line vision while turning
    }
  }
  Retezec1 += String(md.getM1CurrentMilliamps()) + "\t" + String(md.getM2CurrentMilliamps()) + "\t";      //Saving electrical current of motor 1 and 2 in the string
  boolean MalyRozdil;
  int MinimalniRozdil = 100;
  if ((MaxValAnalog-MinValAnalog) < MinimalniRozdil) MalyRozdil = true; else MalyRozdil = false;
  MinValAnalog = (MinValAnalog * 17) / 10;                      //Saving 170% of original value into minimal analog value to decide whether the difference between maximum and minimum is sufficient
  if (Rizeni.Otocka) MinValAnalog = (MinValAnalog * 18) / 10;   //Saving 180% of original value into minimal analog value to decide whether the difference between maximum and minimum is sufficient - I need to be sure I see line while rotating
  if ((MinValAnalog <= MaxValAnalog) &&(!MalyRozdil)) {         //If maximal value is greater than minimal value and the difference is sufficient
    if (Odboc) {                                   //If I should turn
      MaxValAnalog = (MaxValAnalog * 5) / 10;      //If Im turning, I consider as white everything greater than 70% of max value
      int prvniSenzor;
      int posledniSenzor;
      int krokSenzor;
      if (OdbocLevo) {
        //Line evaluation while turning left
        prvniSenzor = 8;
        posledniSenzor = 1;
        krokSenzor = -1;
      } else {
        //Line evaluation while turning right
        prvniSenzor = 1;
        posledniSenzor = 8;
        krokSenzor = 1;
      }
      for (int i = prvniSenzor; i != posledniSenzor + krokSenzor; i = i + krokSenzor) { //Going through measured values and evaluating what is the white line and what isnt
        if (lSenzorA[i].AnalogVal >= MaxValAnalog) {          //If sensor value is greater than requested value -> sensor is seeing line
          ByteVal = (ByteVal * 2) + 1;                        //Variable for data transfer to COM arduino
          if (((PozCary == 0) || (PozCary == (i-krokSenzor))) && (PocCar < 2)) {        //If this is the first line I see or the previous neighboured this one, everything is alright
            HodnotaCary += lSenzorA[i].Val;                   //Adding current sensor value into this variable. Value gives the distance form center and sign gives direction
            PocCar ++;                                        //Increasing number of seen lines
            PozCary = i;                                      //Saving current sensor index into line position
          }
          //Retezec1 = Retezec1 + "1\t";                      //Saving values from all sensors and TAB sign in the string
        } else {                                              //If sensor value is lower than requested value - sensor is seeing black
          ByteVal = (ByteVal * 2) + 0;                        //Variable to be sent to COM arduino
        }
      }
      //Retezec1 = Retezec1 + "\n";  
    } else {
      //Nova metoda - hleda nejstrmejsi zmenu nahoru a dolu
      int maxUp = 0;
      int maxDown = 0;
      byte sensorUp, sensorDown;
      for (int i=1; i<=7; i++) {
        if ((lSenzorA[i].AnalogVal - lSenzorA[i+1].AnalogVal) > maxDown) {
          maxDown = lSenzorA[i].AnalogVal - lSenzorA[i+1].AnalogVal;
          sensorDown = i;
        }
        if ((lSenzorA[i].AnalogVal - lSenzorA[i+1].AnalogVal) < maxUp) {
          maxUp = lSenzorA[i].AnalogVal - lSenzorA[i+1].AnalogVal;
          sensorUp = i;
        }
      }
      PozCary =1;
      if (White) { 
        if (abs(maxUp) > abs(maxDown)) {                     //If the difference UP is greater than difference DOWN, then I probably see edge of the line
          //Maximal width of line is 4 sensors.. If the edge UP is on 4th sensor and higher a simultaneously edge DOWN is on same or lower sensor, then Im on the rim of line a have to set artificial edge DOWN
          if ((sensorUp >= 4) && (sensorDown <= sensorUp)) sensorDown = 8;
        } else {                                             //If the difference DOWN is greater than difference UP, then I probably see edge of the line
          //Maximal width of line is 4 sensors.. If the edge DOWN is on 4th sensor and higher a simultaneously edge UP is on same or lower sensor, then Im on the rim of line a have to set artificial edge UP
          if ((sensorDown <= 4) && (sensorUp >= sensorDown)) sensorUp = 0;
        }
        for (int i=sensorUp+1; i<=sensorDown; i++) {
          HodnotaCary += lSenzorA[i].Val;                     //Adding the current sensor value in this variable. It gives the distance of line from center and sign gives direction
          PocCar ++;                                          //Increasing the number of seen lines
        }
      } else {
        if (abs(maxUp) < abs(maxDown)) {                     //If the difference UP is smaller than difference DOWN, then I probably see edge of the line
          //Maximal width of line is 4 sensors.. If the edge UP is on 4th sensor and higher a simultaneously edge DOWN is on same or lower sensor, then Im on the rim of line a have to set artificial edge DOWN
          if ((sensorDown >= 4) && (sensorUp <= sensorDown)) sensorUp = 8;
        } else {                                             //If the difference DOWN is greater than difference UP, then I probably see edge of the line
          //Maximal width of line is 4 sensors.. If the edge DOWN is on 4th sensor and higher a simultaneously edge UP is on same or lower sensor, then Im on the rim of line a have to set artificial edge UP
          if ((sensorUp <= 4) && (sensorDown >= sensorUp)) sensorDown = 0;
        }
        for (int i=sensorDown+1; i<=sensorUp; i++) {
          HodnotaCary += lSenzorA[i].Val;                     //Adding the current sensor value in this variable. It gives the distance of line from center and sign gives direction
          PocCar ++;                                          //Increasing the number of seen lines
        }
      }
      //Serial.print(F("Senzor Up: ")); Serial.println(sensorUp);
      //Serial.print(F("Senzor Down: ")); Serial.println(sensorDown);
      //Serial.print(F("Pocet car: ")); Serial.println(PocCar);
      //Serial.print(F("Hodnota cary: ")); Serial.println(HodnotaCary);
    }
  } else {                                                    //If the maximal value is smaller than minimal enlarged, then the difference is too small and I send line error message
    Retezec1 += "No line!";                                //Adding info in the string
    PozCary = -2;                                             //Saving error value in line position
  }
  if (((PocCar > 4) && (!Odboc)) || (PozCary < 1) || (PocCar < 1) || (abs(HodnotaCary) > MaxLineVal)) {       //If I see more than 4 lines and Im not in crossroad, or I dont see any, or I have error line position
    LineError();
  } else {                                                  //If the measurement went OK, then I evaluate the data
    CountPID(HodnotaCary / PocCar);
  }
  Vynechej:;
  //Printing the string with info about the line - really slows down the program -> normally banned
  Serial.println(Retezec1);
}
