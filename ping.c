/**
 * This sketch reads a PING ultrasonic rangefinder and returns the distance
 *  to the closest object in range. To do this, it sends a pulse to the sensor to
 *  initiate a reading, then listens for a pulse to return. The length of the
 *  returning pulse is proportional to the distance of the object from the sensor.
 * 
 *  The circuit:
 *  - +V connection of the PING attached to +5V
 *  - GND connection of the PING attached to ground
 *  - SIG connection of the PING attached to digital pin 4
 *  - ECHO connection of the PING attached to digital pin 5
 *  - BUZZER sensor attached to digital pin 2
 */

#include <avr/io.h>
#include <util/delay.h>

#define TERM_IO_POLLUTE_NAMESPACE_WITH_DEBUGGING_GOOP
#include "term_io.h"
#include "util.h"
#include "avr_util.h"

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
#define microsecondsToClockCycles(a) ( ((a) * (F_CPU / 1000L)) / 1000L )
#define microsecondsToCentimeters(a) ( (((a) / 29) / 2))

#define CLEAN_DELAY_MS 2
#define PING_DELAY_MS 5
#define BUZZER_DELAY_MS 100

#define ALERT_DISTANCE_CM 5
#define MAX_PULSES 100

#define PING_DD DDD4
#define PING_PORT PD4

#define ECHO_DD DDD5
#define ECHO_PORT PD5
#define ECHO_PIN PIND5


#define BUZZER_DD DDB5
#define BUZZER_PORT PB5



static void set_pins_initialize()
{

  // All Pins output
  
  // set pin 4 (Ping) for output
  DDRD |= (_BV (PING_DD));/* leave all the other bits alone, just set bit 4 to 1*/
  PORTD &= ~(_BV(PING_PORT)); /* leave all the other bits alone, just set bit 4 to 0*/

  // set pin 5 (Echo) for INPUT (pull up) 
  DDRD |= (_BV (ECHO_DD)); /* leave all the other bits alone, just set bit 5 to 0*/
  PORTD &= ~_BV(ECHO_PORT); /* leave all the other bits alone, just set bit 5 to 0 (tristate)*/


  // Buzzer
  // set pin 6 (Buzzer) for output
  DDRB |= _BV (BUZZER_DD);/* leave all the other bits alone, just set bit 5 to 1*/
  PORTB &= ~(_BV(BUZZER_PORT)); /* leave all the other bits alone, just set bit 5 to 0*/


  PFP("DDRD value:" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(DDRD));
  PFP("PORTD value:" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(PORTD));
  PFP("ECHO_PIN value:" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(_BV(ECHO_PIN)));

  PFP("DDRB value:" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(DDRB));
  PFP("PORTB value:" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(PORTB));


}

static void set_pin_ping (uint8_t value)
{

 
  if ( value == HIGH ) {

    PORTD |= _BV(PING_PORT);

  }else {

    PORTD &= ~(_BV (PING_PORT));
  }
  
  PFP("Set Pin value.PORTD value:" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(PORTD));
  
}

static void set_pin_buzzer (uint8_t value)
{
  
  if ( value == HIGH ) {

    PORTB |= _BV (BUZZER_PORT);
  
  }else {

    PORTB &= ~(_BV (BUZZER_PORT));
  }
  
  PFP("Set pin buzzer.PORTB value:" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(PORTB));

}


static long pulse_pin_echo(int timeout_pulses, uint8_t state_mask)
{

 
  unsigned long pulses = 0; // Time in cycles

  // convert the timeout from microseconds to a number of times through 
   // the initial loop; it takes 16 clock cycles per iteration. 
   unsigned long numloops = 0; 
   unsigned long maxloops = microsecondsToClockCycles(timeout_pulses) / 16; 
 
   PFP("Max Loops:%d \n", maxloops);
   PFP("PIND value:" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(PIND));

   
   // wait for any previous pulse to end 
   while ((PIND && _BV(ECHO_PIN)) != state_mask) {
      PFP ("Wait pulses to start: %d \n", numloops);    
      PFP("PIND value:" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(PIND));
   
      if (numloops++ == maxloops) 
         return 0; 
      pulses++;  
   }


   numloops=0;
   // wait for any previous pulse to end 
   while ((PIND && _BV(ECHO_PIN)) == state_mask) {
      PFP ("Wait pulses: %d \n", numloops);    
      PFP("PIND value:" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(PIND));
   
      if (numloops++ == maxloops) 
         return 0; 
      pulses++;
        
   }

  PFP ("pulses: %d \n", pulses);    
      
  return clockCyclesToMicroseconds(pulses);
}

int main (void)
{

	  // This isn't what we're testing exactly, but we need to know if its
    // working or not to interpret other results.
  term_io_init ();
  PFP ("\n");
  PFP ("\n");
  PFP ("term_io_init() worked.\n");
  PFP ("\n");

  set_pins_initialize();

  PFP ("pings initialized.\n");
  PFP ("\n");
 

  while(1) {


  
    // The PING is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    // Clear ping value
    PFP ("set ping off to clear value.\n");
    PFP ("\n");

    set_pin_ping(LOW);
    _delay_ms(CLEAN_DELAY_MS);

    PFP ("set pin value high to ping signal on.\n");
    PFP ("\n");

    // Set High value 
    set_pin_ping(HIGH);
    _delay_ms(PING_DELAY_MS);

    // Clear ping value
    PFP ("set ping off to clear value.\n");
    PFP ("\n");

    // Turn off ping signal
    set_pin_ping(LOW);

    PFP ("Read Echo signal. pin 5 value.\n");
    PFP ("\n");

    // Read ms waiting to receive echo signal
    long ms = pulse_pin_echo(MAX_PULSES, HIGH) + PING_DELAY_MS/100;
    PFP ("Ping time microseconds: %i ms\n", ms);
    
    int cm = (int) microsecondsToCentimeters(ms);
    PFP ("Centimeters: %i cm\n", cm);
    
    if ((cm>0) && (cm <= ALERT_DISTANCE_CM)) {
     PFP ("Alert. Object is so close run buzzer\n");

     set_pin_buzzer(HIGH);
     _delay_ms(BUZZER_DELAY_MS);
    
     PFP ("Turn off buzzer\n");
     set_pin_buzzer(LOW);

   }  

 } // end while

}



