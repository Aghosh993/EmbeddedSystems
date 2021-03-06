// This sketch utilizes the Compare Channel A interrupt to 
// blink an LED at 2 kHz (i.e. with a net waveform period of 1 ms)

// Function to initialize PORTB Pin 5 (i.e. Arduino Digital pin 13)
// as a Digital Output:

void gpio_init(void)
{
  DDRB |= _BV(5);
}
void setup() {
  // Disable all interrupts while we perform configuration:
  cli();
  
  gpio_init(); // Initialize I/O line(s)

  // Initialize timer 1 control registers to known state
  // AKA pretend Atmel's datasheet is lying to us...
  
  TCCR1A = 0;
  TCCR1B = 0;

  // CS11 and CS10 Set to have Timer Clock = SYSCLK/64
  // SYSCLK = 16 MHz for the Arduino Uno
  // WGM12 sets output compare for channel A (i.e. OCR1A is the TOP)
  // We set OCR1A to 124 as 0.5*SYSCLK/(64*(124+1)) Yields a 1000 Hz period between rising edges of Pin 13 being toggled

  TCCR1B = 1<<CS11 | 1<<CS10 | 1<<WGM12;
  OCR1A = 124;

  // Enable the compare match on channel A interrupt for Timer 1
  TIMSK1 |= _BV(OCIE1A);

  // Enable all interrupts and move on to loop:
  sei();
}

void loop() {
  // Do absolutely nothing here... :)
}

ISR(TIMER1_COMPA_vect)
{
  // Toggle PORTB Pin 5 i.e. Digital Pin 13 here:
  PORTB ^= _BV(5);
  // Do other important stuff.. steer the missile or something:
}

