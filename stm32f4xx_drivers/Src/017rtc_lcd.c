#include <stdio.h>

///*
#include "ds1307.h"
#include "lcd.h"

#define SYSTICK_TIM_CLK	16000000UL

char *days[] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

char* get_day_of_week(uint8_t day_code);
void number_to_string(uint8_t num, char *buf);
char* time_to_string(RTC_time_t *rtc_time);
char *date_to_string(RTC_date_t *rtc_date);
void init_systick_timer(uint32_t tick_hz);


static void mdelay(uint32_t milis)
{
	for (int i = 0; i < (milis * 1000); ++i) {
		// Nothing, just increment var
	}
}


int main017rtc_lcd(void)
{
	RTC_time_t current_time;
	RTC_date_t current_date;

	printf("RTC test\n");

	lcd_init();

	lcd_print_string("RTC Test...");

	mdelay(2000);
	lcd_display_clear();
	lcd_display_return_home();

	if (ds1307_init()) {
		printf("RTC init has failed\n");
		while (1);
	}

	init_systick_timer(1);  // Every 1 second

	current_date.day = FRIDAY;
	current_date.date = 15;
	current_date.month = 1;
	current_date.year = 21;

	current_time.hours = 4;
	current_time.minutes = 25;
	current_time.seconds = 41;
	current_time.time_format = TIME_FORMAT_12HRS_PM;

	ds1307_set_current_date(&current_date);
	ds1307_set_current_time(&current_time);

	while (1);

	return 0;
}


char* get_day_of_week(uint8_t day_code)
{
	return days[day_code-1];
}

void number_to_string(uint8_t num, char *buf)
{
	if (num < 10) {
		buf[0] = '0';
		buf[1] = num + 48;
	}
}

// hh:mm:ss
char* time_to_string(RTC_time_t *rtc_time)
{
	char buf[9];
	buf[2] = ':';
	buf[5] = ':';

	number_to_string(rtc_time->hours, buf);
	number_to_string(rtc_time->hours, &buf[3]);
	number_to_string(rtc_time->hours, &buf[6]);

	buf[8] = '\n';

	return buf;
}

// dd/mm/yy
char *date_to_string(RTC_date_t *rtc_date)
{
	char buf[9];
	buf[2] = ':';
	buf[5] = ':';

	number_to_string(rtc_date->date, buf);
	number_to_string(rtc_date->month, &buf[3]);
	number_to_string(rtc_date->year, &buf[6]);

	buf[8] = '\n';

	return buf;
}

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

	// Calculation of reload value
	uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

	// Clear the value of SVR
	*pSRVR &= ~(0x00FFFFFF);

	// Load the value into SVR
	*pSRVR |= count_value;

	// Do some settings
	*pSCSR |= (1 << 1);  // Enables SysTick exception request
	*pSCSR |= (1 << 2);  // Indicates the clock source, processor clock source

	// Enable the systick
	*pSCSR |= (1 << 0);  // Enables the counter
}

void SysTick_Handler(void)
{
	RTC_time_t current_time;
	RTC_date_t current_date;


	ds1307_get_current_time(&current_time);

	lcd_set_cursor(1, 1);

	char *am_pm;
	if (current_time.time_format != TIME_FORMAT_24HRS) {
		am_pm = (current_time.time_format) ? "PM" :"AM";
//		printf("Current time = %s %s\n", time_to_string(&current_time), am_pm);
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
	} else {
//		printf("Current time = %s\n", time_to_string(&current_time));
		lcd_print_string(time_to_string(&current_time));
	}

	ds1307_get_current_date(&current_date);
//	printf("Current date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&current_date));
}
//*/
