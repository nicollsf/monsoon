const int RSTSENS = 6;

const int nvals = 100;
unsigned long mstimes[nvals];
int vals[nvals];
int nextind = -1;

void setup()
{
//  Serial.begin(9600); //set baud rate
//  Serial.println("");  Serial.println("Entered setup()");
  Serial3.begin(9600); //set baud rate

  pinMode(RSTSENS, INPUT);
  nextind = 0;

  pinMode(LED_BUILTIN, OUTPUT);
//  digitalWrite(LED_BUILTIN, LOW); 
}


unsigned long mlastsent = 0;
void sendhist(void) 
{
  for( int i=0; i<nvals; i++ ) Serial3.print(vals[i]);
  Serial3.println("");
  mlastsent = millis();
}



void loop()
{
  int currstate = digitalRead(RSTSENS);
  digitalWrite(LED_BUILTIN, currstate); 
  if( nextind<nvals ) vals[nextind++] = currstate;
  if( nextind==nvals ) {
    if( mlastsent==0 ) sendhist();
    if( millis()-mlastsent>5000 ) sendhist();
  }

  delay(75);
}

