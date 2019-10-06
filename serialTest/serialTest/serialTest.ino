



int led_pin = 13;


void setup() {
  // put your setup code here, to run once:
  
   
  Serial2.begin(9600,SERIAL_8E1); 
  Serial4.begin(9600,SERIAL_8E1);
  pinMode(led_pin, OUTPUT);



}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(led_pin, HIGH);  // set to as HIGH LED is turn-off
  delay(200);                   // Wait for 0.1 second
  digitalWrite(led_pin, LOW);   // set to as LOW LED is turn-on
  delay(200);         // Wait for 0.1 second

  
  Serial2.write("2");
  delay(200);  
  Serial4.write("4");
  delay(200);  

}
