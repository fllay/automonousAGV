const byte NormalBrakingStep = 5;            //Speed change step upon classic slowing down, stopping on tag
const byte EmergencyBrakingStep = 800;         //Speed change step upon seeng obstacle in sensors
const byte CameraBrakingStep = 400;          //Speed change step while going too fast in a turn (using camera)
const byte HeatStep = 1;                     //Speed change step upon pulling away

void SetSpeed() {
  if (!Rizeni.RucniOvladani) {
    if ((Rizeni.Stop) || (Line.ZtrataCary >= 5)) {       //If I recieved command to STOP or havent seen line 5 times in a row
      int SpeedMot1 = 0;                                 //Setting zero speed for left motor
      int SpeedMot2 = 0;                                 //Setting zero speed for right motor
      Motor[2].Step = Motor[1].Step = NormalBrakingStep; //Setting speed change step to classic stopping
      //test = 223;
      if (((Rizeni.Otocka == true) && (abs(Motor[1].Speed < abs(Motor[1].Step)))) || (Rizeni.Otacim == true)) {   //If I have command to turn and motor already stopped or turnin already commenced
        if (Rizeni.RightOtocka == true) {                //If it should turn right
          SpeedMot1 = Rizeni.MaxSOtocka;                 //Setting left motor to max speed
          SpeedMot2 -= Rizeni.MaxSOtocka;                //Setting right motor to negative max speed
          Rizeni.Otacim = true;
        } else {
          SpeedMot1 -= Rizeni.MaxSOtocka;               //Setting left motor to negative max speed
          SpeedMot2 = Rizeni.MaxSOtocka;                //Setting right motor to max speed
          Rizeni.Otacim = true;
        }
      }
      if (abs(Motor[1].Speed - SpeedMot1) >= Motor[1].Step) {                                       //If current speed of motor 1 is different from 0
        if (Motor[1].Speed < SpeedMot1) Motor[1].Speed += Motor[1].Step;                            //If current speed of motor 1 is smaller than 0, add 1 step
        if (Motor[1].Speed > SpeedMot1) Motor[1].Speed -= Motor[1].Step;                            //If current speed of motor 1 is greater than 0, remove 1 step
      }  
      if  (abs(Motor[2].Speed - SpeedMot2) >= Motor[2].Step) {                                      //If current speed of motor 2 is different from 0
        if (Motor[2].Speed < SpeedMot2) Motor[2].Speed += Motor[2].Step;                            //If current speed of motor 2 is smaller than 0, add 1 step
        if (Motor[2].Speed > SpeedMot2) Motor[2].Speed -= Motor[2].Step;                            //If current speed of motor 2 is greater than 0, remove 1 step
      }
    } else {                                                                                        //If there is no command to STOP
      if (Rizeni.Otacim == true) { Motor[1].Speed = 0; Motor[2].Speed = 0; Rizeni.Otacim = false; } 
      if (Motor[1].SpeedSet != Rizeni.SpeedR) Motor[1].SpeedSet = Rizeni.SpeedR;            //If motor 1 speed is different from regulated sped -> change it to regulated
      if (Motor[2].SpeedSet != Rizeni.SpeedR) Motor[2].SpeedSet = Rizeni.SpeedR;            //If motor 2 speed is different from regulated sped -> change it to regulated
      if ((Rizeni.Otocka == true) && (Motor[1].SpeedSet > Rizeni.MaxSOtocka)) Motor[1].SpeedSet = Rizeni.MaxSOtocka;  //Setting motor 1 speed for perpendicular turning
      if ((Rizeni.Otocka == true) && (Motor[2].SpeedSet > Rizeni.MaxSOtocka)) Motor[2].SpeedSet = Rizeni.MaxSOtocka;  //Setting motor 2 speed for perpendicular turning
      if (abs(Motor[1].Speed - abs(Motor[1].SpeedSet)) >= Motor[1].Step) {                  //If the difference between set and current speed is greater than speed change step for motor 1
        if (Motor[1].Speed < Motor[1].SpeedSet) {                                           //If current speed of motor 1 is smaller than set speed for motor 1
          MotorInterval = 20;                                                              //Interval of changing speed is set to 30ms
          Motor[1].Step = HeatStep;                                                         //Set speed change step to pulling away value
          Motor[1].Speed += Motor[1].Step;                                                  //Increase current speed of motor 1 by set step
          Motor[2].Speed += Motor[1].Step;                                                  //Increase current speed of motor 2 by set step
        }
        if (Motor[1].Speed > Motor[1].SpeedSet) {                                           //If current speed of motor 1 is greater than set speed for motor 1
          MotorInterval = 20;                                                               //Interval of changing speed is set to 10ms
          if (Rizeni.CrashSensor) Motor[1].Step = EmergencyBrakingStep; else Motor[1].Step = NormalBrakingStep;       //Set the speed change step to normal braking of emergency braking depending on the reason, why AGV is slowing down
          if (Rizeni.CameraSlowDown) Motor[1].Step = CameraBrakingStep;
          Motor[1].Speed -= Motor[1].Step;                                                  //Decrease current speed of motor 1 by set step
          Motor[2].Speed -= Motor[1].Step;           //Decrease current speed of motor 1 by set step
          Motor[1].Speed = abs(Motor[1].Speed);
          Motor[2].Speed = abs(Motor[2].Speed);
          
          
        }
      }
    }
  }
}  
