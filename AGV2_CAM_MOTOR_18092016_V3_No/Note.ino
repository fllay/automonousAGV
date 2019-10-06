/*
  Connected PINs:
  Digital 9, 10 - PWM motor control output
  Digital 2, 4, 6, 7, 8, 12 - setting of motor direction a error reading
  Analog A0, A1 - motor current measuring
  Useable functions for motor control:
  md.setM1Speed(x);  - sets motor 1 speed to x (x in range -400 to +400)
  md.setM2Speed(x);  - sets motor 2 speed to x (x in range -400 to +400)
  md.setSpeeds( x, y); - sets motor 1 speed to x and motor 2 to y
  md.setM1Brake(x); - if x = 1 motor stops, if x = 0 brakes release
  md.setM2Brake(x); - if x = 1 motor stops, if x = 0 brakes release
  md.setBrakes( x, x); - if x = 1 motor stops, if x = 0 releases brakes for both motors at the same time
  md.getM1urrentMilliamps()  - returns current value of current consumption of motor 1 in mA
  md.getM2urrentMilliamps()  - returns current value of current consumption of motor 2 in mA
  md.getM1Fault() - returns motor 1 error message
 */
 
 /*
 Fast communication with PINs - using instead of digitalRead, digitalWrite and PinMode - Port Manipulation
 DDRx - Setting Arduino PIN as input or output > DDRF = DDRF | B00110011 - by Leonardo I set A2 A3 A4 A5 to output - symbol | is OR and it is there so I wont affect others
                                                       DDRF = DDRF & B11001100 - by Leonardo I set A2 A3 A4 A5 to input - symbol & is AND and it is there so I wont affect others
 PORTx - If the PIN is set as output, then it sets PINs to HIGH or LOW > PORTF = PORTF & B11001100 - By Leonardo I set A2 A3 A4 A5 to HIGH - symbol & is AND and it is there so I wont affect others
 PINx - If the PIN is set as input, I can read it whole at once> int Nacteno = PINF - I can use the mask again, If I am interested in only one Byte If ((PINF & B00000001) != 0) - if it is on A5 HIGH
 Used in this program for Leonardo> A2=PF5, A3=PF4, 11=PB7, A4=PF1, A5=PF0
 */
 
/*
  digitalWriteFast(Pin, Value);
  pinModeFast(Pin, OUTPUT/INPUT);
  digitalReadFast(Pin);
*/



