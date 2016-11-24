/*
 * @Brief:
 * Quiz 2 possible solution.
 * This application sets PB5 and PD2 as OUTPUTS, and implements a RECEIVE interrupt on the
 * UART of an Atmega328p MCU
 * When a byte 0x31 (ASCII '1') is received on the UART, PB5 and PD2 are turned ON
 * When a byte 0x30 (ASCII '0') is received on the UART, PB5 and PD2 are turned OFF
 * When the ADC value exceeds 204 (i.e. approx. 1 V with respect to an assumed 5V AVcc),
 * the outputs PB5 and PD2 are also turned off, and a byte 0x45 is transmitted over
 * the UART. This is an ASCII 'E'
 * 
 * (c) Abhimanyu Ghosh
 * TA, EL4144 (Intro to Embedded Systems)
 * Fall 2016
 * NYU Tandon School of Engineering
 * 
 * License: GPL v3.0
 */
 
void setup() {

  // Disable/mask all interrupt sources while we're in setup state so we don't
  // end up in an ISR we didn't write a handler for yet...
  cli();
  
  // UART setup, 9600 8n1 with RX ISR enabled
  // 103 UBRR for U2X bit=0, yields 0.2% aka Good Enough (tm)
  // Could also use U2X=1 in which case pick 207 for UBRR
  UBRR0 = 103;
  
  // UCSR0A is primarily a status register, no bits applicable here so either write a 0 or
  // don't bother writing anything...
  UCSR0A = 0;
  
  // Enable Receive Complete interrupt, and UART receiver+transmitter:
  UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);
  
  // Set 8-bit data length, no parity + 1 stop bit aka 8n1 data format:
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
  
  // Set PB5 and PD2 as outputs, per requirements given in the quiz for this application:
  DDRB |= _BV(PB5);
  DDRD |= _BV(PD2);

  // Digital IO setup, PB5, PD2 as OUTPUTS
  PORTB &= ~_BV(PB5);
  PORTD &= ~_BV(PD2);

  // Analog in on ADC3 (PC3) setup, continuous conversion:
  // Use internal analog reference, and set MUX bits per the datasheet, to select ADC3 channel:
  ADMUX = _BV(REFS0) | _BV(MUX1) | _BV(MUX0);

  // Enable the ADC, enable Auto Trigger and set prescaler so the ADC clock is somewhere
  // in the range of 50 kHz-100 kHz (this is a requirement per the datasheet):
  ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

  // No bits applicable to be written here. This 0-write is optional...
  ADCSRB = 0;

  // Disable Digital IO buffer on the ADC3 pin to avoid HW conflict over the pin that
  // we're now using for ADC purposes:
  DIDR0 |= _BV(ADC3D);

  // Set Pin PC3 (i.e. the GPIO pin that connects to ADC3 channel) as an INPUT
  // if it isn't already:
  DDRD &= ~_BV(PC3);

  // Set the ADC to start conversions, now that we've finished all setup of the peripheral:
  ADCSRA |= _BV(ADSC);

  // Enable all interrupts, now that we're done with setup, and jump to user loop():
  sei();
}

void loop() {
  /*
   * If the ADC value exceeds 204, the voltage at the ADC exceeds:
   * 204 * (5 Volts) / 1023 = 1 V (approx.)
   * This condition triggers the turning OFF of PB5 and PD2, and
   * the transmission of a byte 0x45 over the UART (i.e. ASCII 'E')
   */
  if(ADC >= 204)
  {
    // Switch off the IO lines:
    PORTD &= ~_BV(PD2);
    PORTB &= ~_BV(PB5);
    
    // Transmit byte 0x45:
    UDR0 = 0x45;

    // Wait for the transmit to finish:
    while(!(UCSR0A & _BV(TXC0)));

    // Clear the transmit complete flag by writing a 1 to it
    // (Per the datasheet recommendation)
    UCSR0A |= _BV(TXC0);
  }
}

/*
 * Interrupt service routine to process UART receive events:
 */
ISR(USART_RX_vect)
{
  // Store received byte into a temporary location to clear receive flag in hardware:
  uint8_t recv_byte = UDR0;

  // If the byte is 0x31, turn ON PB5 and PD2:
  if(recv_byte == 0x31)
  {
    PORTB |= _BV(PB5);
    PORTD |= _BV(PD2);
  }

  // If the byte is 0x30, turn OFF PB5 and PD2:
  if(recv_byte == 0x30)
  {
    PORTB &= ~_BV(PB5);
    PORTD &= ~_BV(PD2);
  }
}

