#ifndef __PRINT_UTILS__
#define __PRINT_UTILS__

#include "stdint.h"
#include "stddef.h"

typedef struct {
	uint8_t h;
	uint8_t m;
	uint8_t s;
} time_hms_t;

size_t print_time(char* buf, time_hms_t* time);
size_t print_fixed_decimal(char *buf, int num, int dot_shift);


#endif
