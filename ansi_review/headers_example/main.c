#include "foo.h"

#include <stdio.h>

#include "foo.h"

int main(int argc, char** argv)
{
	foo();

	int i = incr(0);

	printf("0+1=%d\n", i);
}