int val=0;
void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, INPUT);
  digitalWrite(LED_BUILTIN,0);
}

void loop() {
  // put your main code here, to run repeatedly:
val=digitalRead(2);
digitalWrite(LED_BUILTIN,val);
delay(10);
}
