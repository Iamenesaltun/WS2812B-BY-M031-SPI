
#include <stdio.h>
#include "NuMicro.h"

#define SPI_CLK_FREQ    2500000
#define LED_NUMBER 10

void SYS_Init(void);
void SPI_Init(void);
unsigned int data_bit8_to_bit24(uint8_t data);
void send_led_commands(uint8_t Led_number, uint8_t G,uint8_t R,uint8_t B);


void send_array_clear(void){
uint8_t led_counter=0;

for(led_counter=0;led_counter<=LED_NUMBER;led_counter++){
SPI_WRITE_TX(SPI0,0x924924);
SPI_WRITE_TX(SPI0, 0x924924);
SPI_WRITE_TX(SPI0, 0x924924);
while(SPI_IS_BUSY(SPI0));	
}
CLK_SysTickDelay(200);	
}


void send_led_commands(uint8_t Led_number, uint8_t G,uint8_t R,uint8_t B){
uint8_t led_counter=0;
uint32_t dataG,dataR,dataB=0;

dataG=data_bit8_to_bit24(G);  //Transform 8bit green data to 24bit data
dataR=data_bit8_to_bit24(R);  //Transform 8bit red data to 24bit data
dataB=data_bit8_to_bit24(B);  //Transform 8bit blue data to 24bit data

for(led_counter=0;led_counter<Led_number;led_counter++){
SPI_WRITE_TX(SPI0,0x924924);   //Send 0 bit turn off green color
SPI_WRITE_TX(SPI0, 0x924924);  //Send 0 bit turn off red color 
SPI_WRITE_TX(SPI0, 0x924924);  //Send 0 bit turn off blue color
while(SPI_IS_BUSY(SPI0));	     // wait for communication
}

SPI_WRITE_TX(SPI0, dataG); // Send green color code
SPI_WRITE_TX(SPI0, dataR); // Send red color code
SPI_WRITE_TX(SPI0, dataB); // Send blue color code
while(SPI_IS_BUSY(SPI0));	 // wait for communication

CLK_SysTickDelay(200);  //wait for 200us for reset code
}

unsigned int data_bit8_to_bit24(uint8_t data){
uint32_t conv_data=0;
int8_t x=0;
uint32_t bit0=4;  //100 
uint32_t bit1=6;  //110

for(x=7;x>=0;x--){
if (data&(1<<x))
conv_data=conv_data|(bit1<<(3*x));
else
conv_data=conv_data|(bit0<<(3*x));
}
return conv_data;
}

int8_t counter=0;
int main(void)
{
/* Unlock protected registers */
SYS_UnlockReg();
/* Init System, IP clock and multi-function I/O. */
SYS_Init();
/* Lock protected registers */
SYS_LockReg();
/* Init SPI */
SPI_Init();
send_array_clear();

while(1){

for(counter=LED_NUMBER-1;counter>=0;counter--){
send_led_commands(counter,255-counter*28,counter*28,0);
CLK_SysTickDelay(100000); //wait 100 miliseconds
}

CLK_SysTickDelay(1000000); //wait 3 seconds
CLK_SysTickDelay(1000000);
CLK_SysTickDelay(1000000);

send_array_clear(); //Turn off all LEDs on strip

CLK_SysTickDelay(1000000); //wait 1 second

}
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
		CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HIRC_DIV2);
    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select HIRC as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select PCLK1 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Setup SPI0 multi-function pins */
    /* PA.3 is SPI0_SS,   PA.2 is SPI0_CLK,
       PA.1 is SPI0_MISO, PA.0 is SPI0_MOSI*/
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA3MFP_Msk |
                                       SYS_GPA_MFPL_PA2MFP_Msk |
                                       SYS_GPA_MFPL_PA1MFP_Msk |
                                       SYS_GPA_MFPL_PA0MFP_Msk)) |
                    (SYS_GPA_MFPL_PA3MFP_SPI0_SS |
                     SYS_GPA_MFPL_PA2MFP_SPI0_CLK |
                     SYS_GPA_MFPL_PA1MFP_SPI0_MISO |
                     SYS_GPA_MFPL_PA0MFP_SPI0_MOSI);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 24-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2.5MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0,24, SPI_CLK_FREQ);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);
}


