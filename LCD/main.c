#include "stm32f10x.h"
#include "delay.h"
#include "lcd16x2.h"

int main(void)
{
	// Initialize delay functions
	DelayInit();
	lcd16x2_init(LCD16X2_DISPLAY_ON_CURSOR_OFF_BLINK_OFF);
	while (1)
	{
		lcd16x2_gotoxy(0,0);
		lcd16x2_puts("CAO VAN HUNG");
		lcd16x2_gotoxy(0,1);
		lcd16x2_puts("CAO VAN TUNG");
		DelayMs(500);
		
		lcd16x2_clrscr();
		DelayMs(500);
	}
}

