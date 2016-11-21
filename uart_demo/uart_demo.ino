/*
 * Sends a string of length len. The string shall be a user-supplied
 * pointer to an array of uint8_t.
 */
void send_String(uint8_t *str, uint8_t len)
{
  uint8_t i = 0U;
  for(i=0U; i<len; ++i) // Iterate over data buffer
  {
    UDR0 = str[i];
    while(!(UCSR0A & _BV(TXC0)));
    UCSR0A |= _BV(TXC0);
  }
}

void setup() {
  cli();  // Globally disable all interrupts while performing setup
  
  // UBRR can be 8 for U2X = 0, but the baud error is too high... :/
  // Look at the datasheet and check out the table of UBRR values vs 
  // actual baud rates and % error!! Error > 3.5% is bad and leads to
  // garbage data transfer!!
  
  UBRR0L = 16; 
  UBRR0H = 0;
  
  UCSR0A = _BV(U2X0); // Want the 2X baud multiplier to have acceptable
                      // error in the baud rate.
  UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0); // Enable receive complete interrupt
                                                  // Enable receiver
                                                  // Enable transmitter
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // Set 8-bit data, no parity, 1 stop i.e. 8n1

  DDRB |= _BV(5); // Set PB5 (i.e. Arduino board Pin 13) high to use the LED as an
                  // output
  sei(); // Globally enable all interrupts
}

// Do a whole lot of nothing here, since everything's event-driven and
// handled by interrupts:
void loop() {
  // put your main code here, to run repeatedly:
}

// UART Receive interrupt service handler:
ISR(USART_RX_vect)
{
  uint8_t received_byte = UDR0;

  switch(received_byte)
  {
    case 'h':
      send_String((uint8_t *)"On\r\n", 4);
      PORTB |= _BV(5);
      break;
    case 'l':
      send_String((uint8_t *)"Off\r\n", 5);    
      PORTB &= ~_BV(5);
      break;
    default:
      send_String((uint8_t *)"got char\r\n", 10);
      break;
  }
}

