#include <stdio.h>

int main(int argc, char** argv)
{
	int a = 10;
	int b = 2;

	printf("%d\n", a/b);

	printf("%d\n", b/a);

	printf("%f\n", (float)b/(float)a);
}