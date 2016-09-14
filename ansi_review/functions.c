#include <stdio.h>
#include <math.h>

// It's good practice to declare function prototypes as below. These normally go in a seperate header file:

void foo(void);
void bar(int i);
float pythag(float a, float b);
void sub1(float* inputs, int len);
void print_arr(float a[], int len);
int fib(int n);
int increment(void);

int main(int argc, char** argv)
{
	foo();
	bar(9001);
	printf("Hypotenuse of a 3, 4-sided right triangle is %f\n", pythag(3.0f, 4.0f));

	float a[3] = {1.0f, 2.5f, 3.0f};

	printf("Original array:\n");
	print_arr(a, 3);
	printf("Modified array:\n");
	sub1(a, 3);
	print_arr(a, 3);

	printf("Fib of 10 is %d\n", fib(10));

	int i = 0;
	for(i = 0; i < 10; ++i)
	{
		printf("Iterator is now at %d\n", increment());
	}
}

void foo(void)
{
	printf("Hello, from foo-land!!\n");
}

void bar(int i)
{
	printf("I hail from bar! I was passed an input of %d...\n", i);
}

// Pass by value, when we do not intend to modify the arguments

float pythag(float a, float b)
{
	return (float)sqrt(a*a + b*b);
}

/*
	Subtracts 1.0f from each element in argument "inputs". "len" must be the actual length of "inputs", or segfault will result!!
	Note that "inputs" is being passed by REFERENCE, as we want to modify the argument!!
 */

void sub1(float* inputs, int len)
{
	int i = 0;
	for(i = 0; i < len; ++i)
	{
		inputs[i] -= 1;
	}
}

void print_arr(float a[], int len)
{
	int i = 0;
	for(i = 0; i < len; ++i)
	{
		// Print comma-seperated list of elements:
		printf("%f, ", a[i]);
	}
	// Print a trailing newline to be nice...
	printf("\n");
}

/*
	An example of a recursive function implementation of Fibonacci sequence:
 */
int fib(int n)
{
	switch(n)
	{
		case 0:
			return 0;
			// break; // We don't need this here since it's preceded by a return statement...
		case 1:
			return 1;
		default:
			return fib(n-1) + fib(n-2);
	}
}

/*
	Example of static variable:
 */
int increment(void)
{
	static int i = 0;
	return i++; // Change this to ++i and see what happens to the output when you run it repeatedly :) :)
}