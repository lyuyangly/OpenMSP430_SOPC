#include "omsp.h"


// Four-Digit, Seven-Segment LED Display driver
#define DIGIT0        (*(volatile unsigned char *)  0x0090)
#define DIGIT1        (*(volatile unsigned char *)  0x0091)
#define DIGIT2        (*(volatile unsigned char *)  0x0092)
#define DIGIT3        (*(volatile unsigned char *)  0x0093)

void delay(volatile int t)
{
	volatile int i, j;
	for(j = 0; j < t; j++)
		for(i = 0; i< 1024;i++);

}

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;
    P2DIR  = 0x00;
    P3DIR  = 0xff;
	DIGIT0 = 0x99;
	DIGIT1 = 0x92;
	DIGIT2 = 0x82;
	DIGIT3 = 0xf8;

	unsigned int p = 0;

	while (1)
	{
		p++;
		delay(1000);
		P3OUT = ~p;
		delay(1000);
	}
	return 0;
}

