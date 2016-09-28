void gpio_init(void)
{
  DDRB |= _BV(5);
}
void setup() {
  // put your setup code here, to run once:
  gpio_init();
}

void loop() {
  // put your main code here, to run repeatedly:

}
