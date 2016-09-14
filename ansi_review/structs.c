#include <stdio.h>
#include <math.h>
#include <stdlib.h> // For malloc, free

typedef struct {
	float x;
	float y;
	float z;
} point;

point *getNewPoint(float x_0, float y_0, float z_0)
{
	point *p = (point *)malloc(sizeof(point));
	p->x = x_0;
	p->y = y_0;
	p->z = z_0;
	return p;
}

float distance(point a, point b)
{
	float dx = b.x - a.x;
	float dy = b.y - a.y;
	float dz = b.z - a.z;

	return sqrt(dx*dx + dy*dy + dz*dz);
}

float movePoint(point *p, float dx, float dy, float dz)
{
	p->x += dx;
	p->y += dy;
	p->z += dz;
}

int main(int argc, char** argv)
{
	point *a = getNewPoint(0.0, 0.0, 0.0);
	point *b = getNewPoint(0.0, 3.0, 4.0);
	if(!a || !b) // If null pointer exception, abort...
	{
		fprintf(stderr, "ERROR, NULL PTR EXCEPTION!!\n");
		return -1;
	}

	printf("Distance from point a to point b is %f\n", distance(*a, *b));

	movePoint(a, 1, 2, 3);
	printf("Point a is now located at x: %f y: %f z: %f\n", a->x, a->y, a->z);

	free(a);
	free(b);
}