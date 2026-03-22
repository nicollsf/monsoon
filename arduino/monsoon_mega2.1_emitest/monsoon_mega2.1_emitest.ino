//String message; //string that stores the incoming message

//const int RPINS0[8] = {23, 25, 27, 29, 31, 33, 35, 37};  // power relay board
const int RPINS0[8] = {31, 33, 35, 37, 31, 33, 35, 37};  // power relay board
const int RPINS1[8] = {39, 41, 43, 45, 47, 49, 51, 53};  // valve relay board
//const int RPINS1[8] = {39, 39, 39, 39, 39, 39, 39, 39};  // valve relay board

void setup()
{
  Serial.begin(9600); //set baud rate
  Serial3.begin(9600); //set baud rate

  Serial.println("");  Serial.println("Entered setup()");

  // Evidently bootloader might reset this register?
  Serial.print("MCUSR: ");
  Serial.println(MCUSR);
  if ( MCUSR & _BV(EXTRF) ) {
    // Reset button or otherwise some software reset
    Serial.println("Reset button was pressed.");
  }
  if ( MCUSR & (_BV(BORF) | _BV(PORF)) ) {
    // Brownout or Power On
    Serial.println("Power loss occured!");
  }
  if ( MCUSR & _BV(WDRF) ) {
    //Watchdog Reset
    Serial.println("Watchdog Reset");
  }
  // Clear all MCUSR registers immediately for 'next use'
  MCUSR = 0;

  for ( int i = 0; i < 8; i++ ) pinMode(RPINS0[i], OUTPUT);
  for ( int i = 0; i < 8; i++ ) pinMode(RPINS1[i], OUTPUT);
  for ( int i = 0; i < 8; i++ ) digitalWrite(RPINS0[i], HIGH);
  for ( int i = 0; i < 8; i++ ) digitalWrite(RPINS1[i], HIGH);
}


unsigned long lastchtime = 0;
void loop()
{
  char inchar;
  char mess1[32], endchar1 = 'K';
  char mess2[32], endchar2 = 10;
  while ( Serial3.available() > 0 ) {
    inchar = Serial3.read();
    if ( inchar == -1 ) {
      Serial.print("Serial3.read returned zero bytes");
      break;
    }

    if ( inchar == '+') {
      Serial.println("Chomping AT message: +");
      int bytesread = Serial3.readBytesUntil(endchar1, mess1, 30);
      mess1[bytesread] = endchar1;  mess1[bytesread + 1] = '\0';
      Serial.print(mess1);
      bytesread = Serial3.readBytesUntil(endchar2, mess2, 30);
      Serial.print(mess2);
      mess2[bytesread] = endchar2;  mess2[bytesread + 1] = '\0';
      Serial.println("[Chomped]");
    } else {
      Serial.print(inchar);  Serial.print("[");  Serial.print((int)inchar);
      Serial.println("]");  Serial.flush();

      Serial3.print("*L");  Serial3.print(inchar);  Serial3.print("[");
      Serial3.print((int)inchar);
      Serial3.println("]*");  Serial3.flush();
    }
  }

  if ( millis() - lastchtime > 2000 ) {
    int currstate = digitalRead(RPINS1[0]);
    for ( int i = 0; i < 8; i++ ) digitalWrite(RPINS0[i], !currstate);
    for ( int i = 0; i < 8; i++ ) digitalWrite(RPINS1[i], !currstate);
    lastchtime = millis();
    Serial.print(".");
  }

  delay(100); //delay
  //Serial.print(".");
  //Serial3.print("tic");
}

