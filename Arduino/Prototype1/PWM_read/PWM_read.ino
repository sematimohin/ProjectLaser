#define pulsePin 8
#define PWMpin1 5
#define PWMpin2 9

int ontime,offtime;
float freq,period;
   
void setup() {
  Serial.begin(9600);
  
  pinMode(pulsePin,INPUT);
  pinMode(PWMpin1, OUTPUT);
  pinMode(PWMpin2, OUTPUT);
  
  analogWrite(PWMpin1, 64);//0-255
  analogWrite(PWMpin2, 64);//0-255
}

void loop() {
   ontime = pulseIn(pulsePin,HIGH);
   offtime = pulseIn(pulsePin,LOW);
   period = ontime+offtime;
   freq = 1000000.0/period;
      
   Serial.print("freq=");
   Serial.println(freq);
   delay (1000);
}
