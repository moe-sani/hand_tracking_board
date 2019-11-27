int val=0;
#define pin_S0 8
#define pin_S1 5
#define pin_S2 9
#define pin_S3 6
#define pin_S4 10
#define pin_S5 7
#define pin_S6 2

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pin_S0, OUTPUT);
  pinMode(pin_S1, OUTPUT);
  pinMode(pin_S2, OUTPUT);
  pinMode(pin_S3, OUTPUT);
  pinMode(pin_S4, OUTPUT);
  pinMode(pin_S5, OUTPUT);
  pinMode(pin_S6, OUTPUT);

  digitalWrite(LED_BUILTIN,0);
  digitalWrite(pin_S0,0);
  digitalWrite(pin_S1,0);
  digitalWrite(pin_S2,0);
  digitalWrite(pin_S3,0);
  digitalWrite(pin_S4,0);
  digitalWrite(pin_S5,0);
  digitalWrite(pin_S6,0);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(pin_S0,0);
  digitalWrite(pin_S1,0);
  digitalWrite(pin_S2,0);
  digitalWrite(pin_S3,0);
  digitalWrite(pin_S4,0);
  digitalWrite(pin_S5,0);
  digitalWrite(pin_S6,0);

  digitalWrite(pin_S0,1);
  delay(1);
  digitalWrite(pin_S0,0);
  delay(1);
  digitalWrite(pin_S1,1);
  delay(1);
  digitalWrite(pin_S1,0);
  delay(1);
  digitalWrite(pin_S2,1);
  delay(1);
  digitalWrite(pin_S2,0);
  delay(1);
  digitalWrite(pin_S3,1);
  delay(1);
  digitalWrite(pin_S3,0);
  delay(1);
  digitalWrite(pin_S4,1);
  delay(1);
  digitalWrite(pin_S4,0);
  delay(1);
  digitalWrite(pin_S5,1);
  delay(1);
  digitalWrite(pin_S5,0);
  delay(1);
  digitalWrite(pin_S6,1);
  delay(1);
  digitalWrite(pin_S6,0);
  delay(1);
}
