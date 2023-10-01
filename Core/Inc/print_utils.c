#include "print_utils.h"

unsigned int ten_pow(unsigned int exp){
	static const int pows[5] = {1, 10, 100, 1000, 10000};
	return pows[exp];
}

size_t print_integer(char *buf, unsigned int num){
	if (num==0) {
		buf[0] = '0';
		buf[1] = 0;
		return 1;
	}
	if (num<0){
		buf[0] = '-';
		return 1+print_integer(buf+1, -num);
	}
	size_t num_len = 0;
	while (num>0){
		char digit = num%10 + '0';
		for (int i=num_len-1; i>=0; i--) buf[i+1] = buf[i];
		buf[0] = digit;
		num_len++;
		num/=10;
	}
	buf[num_len] = 0;
	return num_len;
}

size_t print_zeropad_integer(char *buf, int num, size_t digits){
	size_t num_len = print_integer(buf, num);
	if (num_len < digits){
		int diff = digits - num_len;
		int i;
		for (i=num_len-1; i>=0; i--){
			buf[i + diff] = buf[i];
		}
		for (i=diff-1; i>=0; i--){
			buf[i] = '0';
		}
		num_len+=diff;
		buf[num_len] = 0;
	}
	return num_len;
}


size_t print_fixed_decimal(char *buf, int num, int dot_shift){
	unsigned int factor = ten_pow(dot_shift);
	int integer_part = num / factor;
	int decimal_part = num % factor;
	size_t str_buf_idx = print_integer(buf, integer_part);
	buf[str_buf_idx++] = '.';
	str_buf_idx += print_integer(buf + str_buf_idx, decimal_part);
	return str_buf_idx;
}

size_t print_time(char * buf, time_hms_t* time){
	size_t str_buf_idx = print_zeropad_integer(buf, time->h, 2);
	buf[str_buf_idx++] = ':';
	str_buf_idx += print_zeropad_integer(buf + str_buf_idx, time->m, 2);
	buf[str_buf_idx++] = ':';
	str_buf_idx += print_zeropad_integer(buf + str_buf_idx, time->s, 2);
	return str_buf_idx;
}
