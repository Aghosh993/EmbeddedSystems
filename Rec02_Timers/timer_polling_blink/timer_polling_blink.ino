// This sketch utilizes the Compare Channel A in polling mode to 
// blink an LED at 2 kHz (i.e. with a net waveform period of 1 ms)

// Function to initialize PORTB Pin 5 (i.e. Arduino Digital pin 13)
// as a Digital Output:

void gpio_init(void)
{
  DDRB |= _BV(5); // Set PORTB Pin 5 (i.e. Digital pin 13) to OUTPUT
}
void setup() {
  cli();
  // put your setup code here, to run once:
  gpio_init(); // Initialize I/O line(s)
  
  TCCR1A = 0;
  TCCR1B = 0;

  // CS11 and CS10 Set to have Timer Clock = SYSCLK/64
  // SYSCLK = 16 MHz for the Arduino Uno
  // WGM12 sets output compare for channel A (i.e. OCR1A is the TOP)
  // We set OCR1A to 124 as 0.5*SYSCLK/(64*(124+1)) Yields a 1000 Hz period between rising edges of Pin 13 being toggled

  TCCR1B = 1<<CS11 | 1<<CS10 | 1<<WGM12;
  OCR1A = 124;
}

void loop() {
  // put your main code here, to run repeatedly:
  // If we have a compare match on A (OCR1A), toggle Pin 13:
  if(TIFR1 & _BV(OCF1A))
  {
    TIFR1 |= _BV(OCF1A);
    PORTB ^= _BV(5);
  }
}
