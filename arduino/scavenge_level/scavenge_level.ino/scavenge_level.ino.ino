// Fred Nicolls, March 2018

const int RELAYPIN=7;
const int TLEVPIN=3;  // trigger
const int FLEVPIN=2;  // full overflow
const int PMPERIOD=10000;  // minimum pump on period

unsigned long lasthightime = 0;

void setup() 
{
  pinMode(RELAYPIN, OUTPUT);  
  digitalWrite(RELAYPIN, LOW);  // off
  pinMode(TLEVPIN, INPUT_PULLUP);  
  pinMode(FLEVPIN, INPUT_PULLUP);  
}

void loop() 
{
  // If full then pump always off
  if( digitalRead(FLEVPIN)==LOW ) {
    digitalWrite(RELAYPIN, LOW);  // off
    delay(200);  return;
  }

  // Pump on if trigger level high
  if( digitalRead(TLEVPIN)==LOW ) {
    digitalWrite(RELAYPIN, HIGH);  // on
    lasthightime = millis();
  }

  // Pump off
  if( millis()-lasthightime>PMPERIOD ) {
    digitalWrite(RELAYPIN, LOW);  // off
  }

  delay(100);
}
