
#ifndef oled_h
#define oled_h

#include <stdint.h>

int initialize_oled(void);
int write_text_to_oled(char* l1, char* l2);
void oled_set_power(uint8_t oled_state);
void close_oled(void);
void invert_oled_color(void);











#endif
