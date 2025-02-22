const int DIR = 12;
const int STEP = 14;
const int DIR1 = 35;
const int STEP1 = 34;
const int  steps_per_rev = 200;

void setup()
{
  Serial.begin(115200);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(DIR1, OUTPUT);
}
void loop()
{
  digitalWrite(DIR, HIGH);
  digitalWrite(DIR1, HIGH);
  Serial.println("Spinning Clockwise...");
  
  for(int i = 0; i<steps_per_rev; i++)
  {
    digitalWrite(STEP, HIGH);
    digitalWrite(STEP1, HIGH);
    delayMicroseconds(2000);
    digitalWrite(STEP, LOW);
    digitalWrite(STEP1, LOW);
    delayMicroseconds(2000);
  }
  delay(1000); 
  
  digitalWrite(DIR, LOW);
  digitalWrite(DIR1, LOW);
  Serial.println("Spinning Anti-Clockwise...");

  for(int i = 0; i<steps_per_rev; i++)
  {
    digitalWrite(STEP, HIGH);
    digitalWrite(STEP1, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP, LOW);
    digitalWrite(STEP1, LOW);
    delayMicroseconds(1000);
  }
  delay(1000);
}