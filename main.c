#include <stm32f10x.h>
#include <stm32f10x_conf.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_i2c.h>
#include <stm32_tic149.h>
#include <stdio.h>
#include  <stdint.h>
#include  <bcd.h>

GPIO_InitTypeDef       GPIO_InitStructure;
I2C_InitTypeDef         I2C_InitStructure;

char str[100];
int a = 0;   // int - 32bit DWORD
long b = 2;			  // long - 32bit DWORD



uint16_t E = 3;

unsigned char STATUS;			//0x00

unsigned char OUT_P_MSB;		//0x01
unsigned char OUT_P_CSB;
unsigned char OUT_P_LSB;

uint32_t Padc;

unsigned char OUT_T_MSB;		//0x04
unsigned char OUT_T_LSB;

unsigned char DR_STATUS;		//0x06

unsigned char OUT_P_DELTA_MSB;		//0x07
unsigned char OUT_P_DELTA_CSB;
unsigned char OUT_P_DELTA_LSB;


unsigned char OUT_T_DELTA_MSB;		//0x0A
unsigned char OUT_T_DELTA_LSB;

int i;
uint8_t* pBuf;  //указатель на буфер вывода   bcd.h

float p_print = 0;


int UNION_ (char Mb,char Lb)
{
	union ls {unsigned char CH[4];int U;} u;
u.CH[1] = Mb;
u.CH[0] = Lb;
u.CH[2] = 0;
u.CH[3] = 0;


return u.U;
}




void SCI_s8dec_Out (MSB,LSB)
		{
	char a, b, c;
	char r;
/*
** Determine sign and output
*/
if (MSB > 0x7F)
			{r= ('-');
			MSB = ~MSB + 1;}
						else{
							r= ('+');}
/*
** Calculate
*/
a = MSB / 100; //Shift the data over since it is MSB only
r = MSB % 100;

//b = (byte)(r / 10);
//c = (byte)(r % 10);
/*
** Format
*/
if (a == 0)
{
a = 0xF0;
if (b == 0)
{
b = 0xF0;
}
}
/*
** Output result
*/


if (r) { sprintf (str,"t -0d%1d%1DeC",a,b,c);}
else   { sprintf (str,"t +0d%1d%1DeC",a,b,c);};
	     tic149_put_str (4,1,str);
}


int main(void)
{

pBuf = BCD_GetPointerBuf(); //инициируем переменную

E=2;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	     /* I2C1 Periph clock enable */
	     RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	   /* Configure I2C1 pins: SCL and SDA ----------------------------------------*/
	     GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	     GPIO_Init(GPIOB, &GPIO_InitStructure);

	   /* I2C1 configuration ------------------------------------------------------*/
	     I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	     I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
	     I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	     I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	     I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	     I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
	     I2C_Init(I2C1, &I2C_InitStructure);
	     I2C_AcknowledgeConfig(I2C1, ENABLE);






	   /* Enable I2C1 */
	     I2C_Cmd(I2C1, ENABLE);

	     //RED LED PORT INIT
	      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

	      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	      GPIO_Init( GPIOB , &GPIO_InitStructure);


	     LCD_init();
	     LCD_clear();


	     /*********************************************************\
	     * IIC Write Register
	     \*********************************************************/
	     void IIC_RegWrite(unsigned char address, unsigned char reg,unsigned char val)
	     {
	    	GPIOB->ODR |= GPIO_ODR_ODR2; 								//RED LED ON
	    	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));				/* Циклическая проверка занятости шины */	 	 //IICC_TX = 1; // Transmit Mode
		    I2C_GenerateSTART(I2C1, ENABLE);							/* Send START condition */
		    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); /* Test on EV5 and clear it */		//IIC_Start(); // Send Start
		    I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);/* Send MPL115A2 address for write */	//IIC_CycleWrite(address); // Send IIC "Write" Address
		    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));/* Test on EV6 and clear it */
		    I2C_SendData(I2C1,reg);	 /* Send the byte to be written */  //IIC_CycleWrite(reg); // Send Register
		    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ));/* Test on EV8 and clear it */
		    I2C_SendData(I2C1,val);	 /* Send the byte to be written */ //IIC_CycleWrite(val); // Send Value
		    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ));/* Test on EV8 and clear it */
		    I2C_GenerateSTOP(I2C1, ENABLE);									//	IIC_Stop(); // Send Stop
		    GPIOB->ODR &= ~GPIO_ODR_ODR2;//RED LED OFF
	     }

	     /*********************************************************\
	     * IIC Read Register
	     \*********************************************************/
	     unsigned char IIC_RegRead(unsigned char address, unsigned char reg)
	     {
	     unsigned char b;

	     GPIOB->ODR |= GPIO_ODR_ODR2; 								//RED LED ON
	     while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));				/* Циклическая проверка занятости шины */	 	 //IICC_TX = 1; // Transmit Mode
	     I2C_GenerateSTART(I2C1, ENABLE);							/* Send START condition */
	     while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); /* Test on EV5 and clear it */

	     I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);/* Send MPL115A2 address for write */	//IIC_CycleWrite(address); // Send IIC "Write" Address
	     while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));/* Test on EV6 and clear it */
	     I2C_SendData(I2C1,reg);	 /* Send the byte to be written */  //IIC_CycleWrite(reg); // Send Register
	     while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ));/* Test on EV8 and clear it */


	     I2C_GenerateSTART(I2C1, ENABLE);							/* Send START condition */
	     while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); /* Test on EV5 and clear it *///Send Repeat Start


	     I2C_Send7bitAddress(I2C1, address, I2C_Direction_Receiver);
	     while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));/* Test on EV6 and clear it *///IIC_CycleWrite(address+1); // Send IIC "Read" Address

	     while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));    /* Test on EV7 and clear it */


	     b = I2C_ReceiveData(I2C1);// b = IIC_CycleRead(1); // *** Dummy read: reads

	     I2C_AcknowledgeConfig(I2C1, DISABLE);
	       PAUSE_MS(10);
	     I2C_GenerateSTOP(I2C1, ENABLE);									//	IIC_Stop(); // Send Stop

         I2C_AcknowledgeConfig(I2C1, ENABLE);
         GPIOB->ODR &= ~GPIO_ODR_ODR2;//RED LED OFF
	     return b;
	     }

	     IIC_RegWrite(0xC0, 0x26,0x38);
	     IIC_RegWrite(0xC0, 0x13,0x07);
	     IIC_RegWrite(0xC0, 0x26,0x39);
	     while (!(IIC_RegRead(0xC0, 0x00)& 0x08));

	     /* Выводит двоичное представление одного байта */





	     while(1)
    {



	    	 OUT_P_MSB = IIC_RegRead(0xC0, 0x01);
	    	 OUT_P_CSB = IIC_RegRead(0xC0, 0x02);
	    	 OUT_P_LSB = IIC_RegRead(0xC0, 0x03);
	    	 OUT_T_MSB = IIC_RegRead(0xC0, 0x04);
	    	 OUT_T_LSB = IIC_RegRead(0xC0, 0x05);
	    	 STATUS = IIC_RegRead(0xC0, 0x00);


	    	 Padc = OUT_P_MSB;
	    	 Padc = (Padc << 8);
	    	 Padc = Padc+OUT_P_CSB;
	    	Padc = (Padc << 2);
	    	Padc = Padc+(OUT_P_LSB>>6);

	     tic149_put_str (0,7,"MPL3115A2");



	     sprintf (str,"P=%01d Pa",Padc);
	        tic149_put_str (2,1,str);

	     sprintf (str,"P=%01d,%01d mm.Hg",((int)(Padc*75)/10)/1000,((int)(Padc*75)/10)%1000);
	        	        tic149_put_str (3,1,str);




	     //	  for(i = 7; i >= 0; i--) { sprintf(str,"%d", (STATUS & 128)==128); STATUS = STATUS << 1;tic149_put_str (6,9-i,str); };


	     	 // for(i = 7; i >= 0; i--) { sprintf(str,"%d", (OUT_T_MSB & 128)==128); OUT_T_MSB = OUT_T_MSB << 1;tic149_put_str (6,18-i,str); };

	     	 sprintf (str,"T %01d,%04d Degrees C",OUT_T_MSB,(int)((OUT_T_LSB>>4)*625));
	     	 	     	     tic149_put_str (5,1,str);

	    	 //  p_print = (mpl115a1_CalculatePressure(PComp));
	    	 //    p_print =  p_print/16*10;

	    	//     sprintf (str, "P=%d,%d kPa (%d mmHg)",(int)p_print/10,(int)p_print%10,(int)(p_print*7.5)/10);
	    // 	 BCD_Uchar(OUT_T_MSB);
	    // 	 tic149_put_str (7,0,pBuf);
	    // 	 BCD_4Int((int)((OUT_T_LSB>>4)*625));
	    // 	 tic149_put_str (7,3,pBuf);
PAUSE_MS(200);



    }
}
