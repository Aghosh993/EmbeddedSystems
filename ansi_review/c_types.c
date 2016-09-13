#include <stdio.h>

int main(int argc, char** argv)
{
	int a = 1;
	float b = 2.5f;
	char c = 'a';
	double pi_value = 3.14159265359;

	int pi_int = (int)pi_value;
	int magic = (int)c;

	printf("a: %d b: %f c: %c pi_value: %0.11lf\n", a, b, c, pi_value);
	// printf("%d\n", magic);
	// printf("%d\n", c);
}