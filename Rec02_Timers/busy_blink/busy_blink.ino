void basic_delay(uint16_t ms);

void setup() {
  // put your setup code here, to run once:
  // Set PB5 (Pin 5 on PORTB aka Arduino Pin 13) as OUTPUT:
  DDRB |= _BV(5);
}

void loop() {
  // put your main code here, to run repeatedly:
//  PORTB |= _BV(5);  
//  basic_delay(1);
//  PORTB &= ~_BV(5);
//  basic_delay(1);

  // Alternatively...

  PORTB ^= _BV(5); // :) :)
  basic_delay(1); // Verify this on a scope. I did :)
}

// An experimentally-verified delay routine using simple busy-wait for loop:
void basic_delay(uint16_t ms)
{
  volatile uint16_t i = 0U;
  volatile uint16_t j = 0U;
  for(i=0; i<ms; ++i) // Repeat for number of requested millis delay
  {
    // I am too lazy to actually justify why 1095 is the chosen value...
    for(j=0; j<1095; ++j) // This is a 1 ms delay. Tuned experimentally.
    {
      ++j;
    }
  }
}

