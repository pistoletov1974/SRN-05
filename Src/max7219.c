#include "max7219.h"
#include "spi.h"


typedef enum {
	REG_NO_OP 			= 0x00 << 8,
	REG_DIGIT_0 		= 0x01 << 8,
	REG_DIGIT_1 		= 0x02 << 8,
	REG_DIGIT_2 		= 0x03 << 8,
	REG_DIGIT_3 		= 0x04 << 8,
	REG_DIGIT_4 		= 0x05 << 8,
	REG_DIGIT_5 		= 0x06 << 8,
	REG_DIGIT_6 		= 0x07 << 8,
	REG_DIGIT_7 		= 0x08 << 8,
	REG_DECODE_MODE 	= 0x09 << 8,
	REG_INTENSITY 		= 0x0A << 8,
	REG_SCAN_LIMIT 		= 0x0B << 8,
	REG_SHUTDOWN 		= 0x0C << 8,
	REG_DISPLAY_TEST 	= 0x0F << 8,
} MAX7219_REGISTERS;

typedef enum {
	DIGIT_1 = 1, DIGIT_2 = 2, DIGIT_3 = 2, DIGIT_4 = 2
} MAX7219_Digits;

void max7219_clean(void);
void sendData(uint16_t data);

static uint16_t SYMBOLS[] = { 0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70,
		0x7F, 0x7B, 0x00 };

void init_max7219(uint8_t intensivity) {

	__HAL_SPI_ENABLE(&hspi1);  //_HAL_SPI_ENABLE(&hspi1);	  //SPI_Cmd(SPI1, ENABLE);



	max7219_setIntensivity(intensivity);
	max7219_clean();
}

void max7219_setIntensivity(uint8_t intensivity) {
	if (intensivity > 0x0F)
		return;
	sendData(REG_SHUTDOWN | 0x01);
	sendData(REG_DECODE_MODE | 0x00);
	sendData(REG_SCAN_LIMIT | 0x07);
	sendData(REG_INTENSITY | intensivity);
}

void max7219_clean() {
	sendData(REG_DIGIT_0 | 0x00);
	sendData(REG_DIGIT_1 | 0x00);
	sendData(REG_DIGIT_2 | 0x00);
	sendData(REG_DIGIT_3 | 0x00);
	sendData(REG_DIGIT_4 | 0x00);
	sendData(REG_DIGIT_5 | 0x00);
	sendData(REG_DIGIT_6 | 0x00);
	sendData(REG_DIGIT_7 | 0x00);
}

void sendData(uint16_t data) {
	volatile uint8_t  data_send[2];
	data_send[0]=data;
	data_send[1] = (data >>8);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *) data_send ,1, 10); //SPI_I2S_SendData(SPI1, data);
//	while ();
	 //  GPIO_ResetBits(GPIOA, GPIO_Pin_6);
	// asm("nop");

	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6, GPIO_PIN_SET);   //GPIO_SetBits(GPIOA, GPIO_Pin_6);
}

uint16_t getSymbol(uint8_t number) {
	return SYMBOLS[number];
}




void displayNumberLow (uint16_t number) {
	uint8_t dig1,dig2,dig3,dig4;
	// значения разрядов входного числа  dig1...dig4
	number = number % 10000;
	dig1 = number /1000;
	dig2 = number /100 %10;
	dig3 = number/10%10;
	dig4 = number%10;
	if (dig1==0) 
	{dig1=10;
		if (dig2==0) {
		  dig2=10;
        if (dig3==0) {
				dig3=10;}		  
		}
	}
	sendData(REG_SHUTDOWN | 0x01);
	sendData(REG_DECODE_MODE | 0x00);
	sendData(REG_SCAN_LIMIT | 0x07);
	sendData(REG_DIGIT_0 | getSymbol(dig4));
	sendData(REG_DIGIT_1 | getSymbol(dig3));
	sendData(REG_DIGIT_2 | getSymbol(dig2));
	sendData(REG_DIGIT_3 | getSymbol(dig1));
	

}



void displayNumberHigh (uint16_t number) {
	uint8_t dig1,dig2,dig3,dig4;
	// значения разрядов входного числа  dig1...dig4
	
	dig1 = number /1000;
	dig2 = number /100 %10;
	dig3 = number/10%10;
	dig4 = number%10;
	if (dig1==0) 
	{dig1=10;
		if (dig2==0) {
		  dig2=10;
        if (dig3==0) {
				dig3=10;}		  
		}
	}
	sendData(REG_SHUTDOWN | 0x01);
	sendData(REG_DECODE_MODE | 0x00);
	sendData(REG_SCAN_LIMIT | 0x07);
	sendData(REG_DIGIT_4 | getSymbol(dig4));
	sendData(REG_DIGIT_5 | getSymbol(dig3));
	sendData(REG_DIGIT_6 | getSymbol(dig2));
	sendData(REG_DIGIT_7 | getSymbol(dig1));
	

}


