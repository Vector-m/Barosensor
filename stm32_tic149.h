#ifndef __stm32_tic149_H_
#define __stm32_tic149_H_


#include <stm32f10x_i2c.h>
 

#define tic149_ADDR           (0x78)
#define ClockSpeed             400000


// const  char font[][6];

void i2c_start(void);
void i2c_stop(void);
void i2c_tx_adr (unsigned char sl_adr);
void i2c_tx (unsigned char data_tr);
void PAUSE_MS(unsigned long ms);


void LCD_init(void);
void LCD_clear(void);
void tic149_set_addr(unsigned char y, unsigned char x);
void tic149_put_char(unsigned char c);
void tic149_put_str_op(char *str);
void tic149_put_str(unsigned char y, unsigned char x, char *str);
unsigned char bit_reverse(unsigned char b);

#endif // __stm32_tic149_H_
