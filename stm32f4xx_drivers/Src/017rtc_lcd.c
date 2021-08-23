#include <stdio.h>
//#include "ds1307.h"


char *days[] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


int main017rtc_lcd(void)
{
	RTC_time_t current_time;
	RTC_date_t current_date;

	printf("RTC test\n");

	if (ds1307_init()) {
		printf("RTC init has failed\n");
		while (1);
	}

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

	ds1307_get_current_date(&current_date);
	ds1307_get_current_time(&current_time);

	char *am_pm;
	if (current_time.time_format != TIME_FORMAT_24HRS) {
		am_pm = (current_time.time_format) ? "PM" :"AM";
		printf("Current time = %s %s\n", time_to_string(&current_time), am_pm);
	} else {
		printf("Current time = %s\n", time_to_string(&current_time));
	}

	printf("Current date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));


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

