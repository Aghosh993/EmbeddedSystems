#include <stdio.h>
#include <stdint.h>

int main(int argc, char** argv)
{
	// A simple example of using a Union in C to convert from a set of 8-bit unsigned integers to a signed 16-bit integer.
	union {
		uint8_t input[2];
		int16_t output;
	} u8_to_int16;

	u8_to_int16.input[0] = 10;
	u8_to_int16.input[1] = 255;

	printf("%d\n", u8_to_int16.output);
}