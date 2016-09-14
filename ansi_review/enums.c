#include <stdio.h>

typedef enum {
	STATE_INIT,
	STATE_STEP1,
	STATE_STEP2,
	STATE_TERMINATED
} statevar;

void kick_state_machine(statevar *s, int *sel)
{
	switch(*s)
	{
		case STATE_INIT:
			if(*sel <= 0)
			{
				*s = STATE_STEP1;
			}
			else
			{
				*sel -= 1;
			}
			break;
		case STATE_STEP1:
			*sel += 1;
			if(*sel >= 10)
			{
				*s = STATE_STEP2;
			}
			break;
		case STATE_STEP2:
			*sel *= 2;
			if(*sel >= 512)
			{
				*s = STATE_TERMINATED;
			}
			break;
		case STATE_TERMINATED:
			*sel /= 10;
			break;
	}
}

int main(int argc, char** argv)
{
	statevar s = STATE_INIT;
	int a = 100;

	while(s != STATE_TERMINATED)
	{
		kick_state_machine(&s, &a);
		printf("%d\n", a);
	}
}