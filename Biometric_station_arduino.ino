void setup() {
  Serial.begin(115200);
  Serial.println("Hello World!");
  
 
  
 

}

void loop() {
  // put your main code here, to run repeatedly:
 double level = 12.33;
  int pubg = 1234;
  
  Serial.print("[");
  Serial.print(level);
  Serial.print("|");
  Serial.print(pubg);
  Serial.println("]");
  delay(200);
}
