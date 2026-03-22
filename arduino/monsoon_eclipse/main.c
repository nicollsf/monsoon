//#define F_CPU 8000000UL
//#include <avr/io.h>
//#include <util/delay.h>
//
//int main(void) {
//	//Serial.begin(9600);
//    DDRB |= 1<<PB0;
//    while(1) {
//    		PORTB &= -(1<<PB0);
//        _delay_ms(100);
//        PORTB |= 1<<PB0;
//        _delay_ms(900);
//        //Serial.println("Fred");
//    }
//    return(0);
//}

/*
 * main.cpp
 *
 *  Created on: 17 Oct 2010
 *      Author: Andy
 */

#include <arduino.h>
#include <wiring.h>

extern "C" void __cxa_pure_virtual() {
  for(;;);
}

int main(void) {
  init();
  setup();

  for(;;)
    loop();

  return 0; // not reached
}

void setup() {
// setup pins

  pinMode(13,OUTPUT);
}

void loop() {
  digitalWrite(13,HIGH);
  delay(1000);
  digitalWrite(13,LOW);
  delay(1000);
}

