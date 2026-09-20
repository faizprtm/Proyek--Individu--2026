


#include "main.h"
#include "TM1637.h"


#define CLK_HIGH() HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET)
#define CLK_LOW() HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
#define DATA_HIGH() HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_SET)
#define DATA_LOW() HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_RESET);

void Delay_us (int time)
{
	for (int i=0; i<time; i++)
	{
		for (int j=0; j<8; j++)
		{
			__asm__("nop");
		}
	}
}
void start (void)
{
	CLK_HIGH();
	DATA_HIGH();
	Delay_us (2);
	DATA_LOW();
}

void stop (void)
{
	CLK_LOW();
	Delay_us (2);
	DATA_LOW();
	Delay_us (2);
	CLK_HIGH();
	Delay_us (2);
	DATA_HIGH();
}

void waitforAck (void)
{
	CLK_LOW();
	Delay_us (5); // After the falling efge of the eight clock delay for 5us, ACK signals the beginning of the judgement
	CLK_HIGH();
	Delay_us (2);
	CLK_LOW();
}

void writeByte (uint8_t byte)
{
    int i;
    for (i = 0; i<8; i++)
    {
        CLK_LOW();
        if (byte & 0x01) // low front
        {
            DATA_HIGH();
        }
        else
        {
            DATA_LOW();
        }
        Delay_us (3);
        byte = byte >> 1;
        CLK_HIGH();
        Delay_us (3);
    }
}

void TM1637_WriteData (uint8_t Addr, uint8_t *data, int size)
{
    start();
    writeByte(0x40);
    waitforAck();
    stop();

    start();
    writeByte(Addr);
    waitforAck();
    for (int i=0; i<size; i++)
    {
        writeByte(data[i]);
        waitforAck();
    }
    stop();

    start();
    writeByte(0x8A);
    waitforAck();
    stop();
}
