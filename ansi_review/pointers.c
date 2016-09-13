#include <stdio.h>

int main(int argc, char** argv)
{
	int arr[10];

	int i = 0;

	for(i = 0; i < 10; ++i)
	{
		arr[i] = i;
	}

	// Standard way of printing an array out:

	// for(i = 0; i < 10; ++i)
	// {
	// 	printf("%d\n", arr[i]);
	// }

	int *p = &arr[0];

	i = 0;
	// Something's wrong for some versions of the code below.. can you tell? :)
	
	while(i<10)
	{
		// One way to do things:

		// printf("%d\n", *p);
		// i++;
		// p++;
		
		// Another way:

		printf("%d\n", *(arr+2*i));
		++i;
	}
}