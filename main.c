/**
 * @file main.c
 * @author Zaryab Shahzaib (zshazai@andrew.cmu.edu)
 * @author Ahmed Saqib (msaqib@andrew.cmu.edu)
 * @author Umaymah Imran (uimran@andrew.cmu.edu)
 * @brief 
 * @version 0.1
 * @date 2020-02-28
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <stdint.h>
#include <stdbool.h>
#include <tm4c123gh6pm.h>
#include <sysctl.h>
#include "timer.h"
#include <math.h>
#include <serial.h>

void PLLInit()
{
    SYSCTL_RCC2_R |= 0x80000000;
    SYSCTL_RCC2_R |= 0x00000800;
    SYSCTL_RCC_R = (SYSCTL_RCC_R & ~0x000007C0) + 0x00000540;
    SYSCTL_RCC2_R &= ~0x00000070;
    SYSCTL_RCC2_R &= ~0x00002000;
    SYSCTL_RCC2_R |= 0x40000000;
    SYSCTL_RCC2_R = (SYSCTL_RCC2_R & ~0x1FC00000) + (4 << 22);
    while ((SYSCTL_RIS_R &0x00000040)==0){};
    SYSCTL_RCC2_R &= ~0x00000800;
}


void PWM_Init(uint16_t periodA, uint16_t periodB, uint16_t highA, uint16_t highB)
{
    SYSCTL_RCGCTIMER_R |= 0x02;
    SYSCTL_RCGCGPIO_R |= 0x02;
    while((SYSCTL_PRGPIO_R & 0X02) == 0) {};
    GPIO_PORTB_AFSEL_R |= 0X30;
    GPIO_PORTB_DEN_R |= 0X30;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0XFF00FFFF) + 0X00770000;
    TIMER1_CTL_R &= ~0X00000101;
    TIMER1_CFG_R = 0X00000004;
    TIMER1_TAMR_R = 0X0000000A;
    TIMER1_TBMR_R = 0X0000000A;
    TIMER1_TAILR_R = periodA - 1;
    TIMER1_TBILR_R = periodB - 1;
    TIMER1_TAMATCHR_R = periodA - highA - 1;
    TIMER1_TBMATCHR_R = periodB - highB - 1;
    TIMER1_CTL_R |= 0X00000101;
}

void pwm_dutyA(uint16_t high)
{
    TIMER1_TAMATCHR_R = TIMER1_TAILR_R - high;
}

void pwm_dutyB(uint16_t high)
{
    TIMER1_TBMATCHR_R = TIMER1_TBILR_R - high;
}

void ADC0_InitTimer0A(uint32_t period){
  volatile uint32_t delay;
  SYSCTL_RCGCADC_R |= 0x01;     // 1) activate ADC0 
  SYSCTL_RCGCGPIO_R |= 0x08;    // Port D clock
  delay = SYSCTL_RCGCGPIO_R;    // allow time for clock to stabilize
  GPIO_PORTD_DIR_R &= ~0x08;    // make PD3 input
  GPIO_PORTD_AFSEL_R |= 0x08;   // enable alternate function on PD3
  GPIO_PORTD_DEN_R &= ~0x08;    // disable digital I/O on PD3
  GPIO_PORTD_AMSEL_R |= 0x08;   // enable analog functionality on PD3
  ADC0_PC_R = 0x07;             // 2) configure for 125K samples/sec
  ADC0_SSPRI_R = 0x3210;        // 3) sequencer 0 is highest, sequencer 3 is lowest
  SYSCTL_RCGCTIMER_R |= 0x01;   // 4) activate timer0
  delay = SYSCTL_RCGCGPIO_R;
  TIMER0_CTL_R = 0x00000000;    // disable timer0A during setup
  TIMER0_CTL_R |= 0x00000020;   // enable timer0A trigger to ADC
  TIMER0_CFG_R = 0;             // configure for 32-bit timer mode
  TIMER0_TAMR_R = 0x00000002;   // configure for periodic mode, default down-count settings
  TIMER0_TAPR_R = 0;            // prescale value for trigger
  TIMER0_TAILR_R = period-1;    // start value for trigger
  TIMER0_IMR_R = 0x00000000;    // disable all interrupts
  TIMER0_CTL_R |= 0x00000001;   // enable timer0A 32-b, periodic, no interrupts

  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF) | 0x40000000;
  NVIC_EN0_R = 1<<19;
  ADC0_ACTSS_R &= ~0x08;        // 5) disable sample sequencer 3
  ADC0_EMUX_R = (ADC0_EMUX_R&0xFFFF0FFF)+0x5000; // 6) timer trigger event
  ADC0_SSMUX3_R = 4;            // 7) PD4 is channel 4
  ADC0_SSCTL3_R = 0x06;         // 8) set flag and end                       
  ADC0_IM_R |= 0x08;            // 9) enable SS3 interrupts
  ADC0_ACTSS_R |= 0x08;         // 10) enable sample sequencer 3
  NVIC_PRI4_R = (NVIC_PRI4_R&0xFFFF00FF)|0x00004000; // 11)priority 2
  NVIC_EN0_R = 1<<17;           // 12) enable interrupt 17 in NVIC

}
int ADCvalue;
int createdSamp = 0;
int distance;

void ADC0Seq3_Handler(void){
  double voltage;

  ADC0_ISC_R = 0x08;          // acknowledge ADC sequence 3 completion
  ADCvalue = ADC0_SSFIFO3_R;  // 12-bit result

  voltage = ADCvalue * (3.3/4096.0);
  distance = 0.3811102 + (149.5288 - 0.3811102)/(1 + pow((voltage/0.1272262),1.248105));

  createdSamp = 1;
}

int Done = 0;
int Period = 0;
int First = 0;
int complete = 0;
int countFreq = 0;
double sum = 0;
double red = 0;
double green = 0;
double blue = 0;

void Timer2A_Handler(void)
{
// calculate the period
Period = (First - TIMER2_TAR_R) & 0x00FFFFFF;
    // remember the timer for next time
First = TIMER2_TAR_R;
// set flag to say that period has a new value
Done = 1;
countFreq++;
TIMER2_ICR_R = 0x00000004; // acknowledge the interrupt

}
void Timer0A_Handler(void){}

int main(void)
{
    PLLInit();
    SystickInit();
    SYSCTL_RCGCTIMER_R |= 0X04;     // Activate Timer 0
    SYSCTL_RCGCGPIO_R |= 0x02;      // Activate Port B
    GPIO_PORTB_DIR_R &= ~0x01;      // Make PB6 an input
    GPIO_PORTB_AFSEL_R  |= 0x01;    // Alternate Functionality Select for PB6, PB4
    GPIO_PORTB_DEN_R |= 0x01;       // Enable Digital Functionality

    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFFFFF0) + 0x00000007;  // Enable clock functionality for PB6, PB4

    TIMER2_CTL_R &= ~0x00000001;    // Disable timer 0 for configuration
    TIMER2_CFG_R = 0x00000004;      // Configure for 16-bit capture mode
    TIMER2_TAMR_R = 0x00000007;     // Enable capture mode + Edge time mode

    TIMER2_CTL_R |= 0x00000004;     // configure for rising edge and falling edge
    TIMER2_TAPR_R = 0xFF;           // Activate pre-scale
    TIMER2_TAILR_R = 0x0000FFFF;    // Start value for count down
    TIMER2_IMR_R |= 0x00000004;     // Enable Input capture interrupts

    TIMER2_ICR_R = 0X00000004;      // Clear Timer0A capture match flag

    TIMER2_CTL_R |= 0x00000001;     // Timer 0A 24-bit activation
    NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF) | 0x80000000;
    NVIC_EN0_R = 1<<23;

    SYSCTL_RCGCGPIO_R |= 0x10;
    GPIO_PORTE_LOCK_R = 0x4C4F434B;
    GPIO_PORTE_CR_R = 0xFF;
    GPIO_PORTE_AMSEL_R = 0x00;
    GPIO_PORTE_PCTL_R = 0x00000000;
    GPIO_PORTE_DIR_R = 0x33;
    GPIO_PORTE_AFSEL_R = 0x00;
    GPIO_PORTE_DEN_R = 0xFF;
    GPIO_PORTE_DATA_R = 0x0;

    SYSCTL_RCGCGPIO_R |= 0x01;
    GPIO_PORTA_LOCK_R = 0x4C4F434B;
    GPIO_PORTA_CR_R = 0xFF;
    GPIO_PORTA_AMSEL_R = 0x00;
    GPIO_PORTA_PCTL_R = 0x00000000;
    GPIO_PORTA_DIR_R = 0xFC; // PA2-3 connected to left MD, PA4-5 right MD
    GPIO_PORTA_AFSEL_R = 0x00;
    GPIO_PORTA_DEN_R = 0xFF;
    GPIO_PORTA_DATA_R = 0x54;
    GPIO_PORTE_DATA_R |= 0x10;


    int accept = 0;
    int P;
    int R;
    char *color = "continue";

    int sum = 0;
    int numSamp = 5;
    int foundSamp = 0;
    int readColor = 0;
    int readDist = 1;
    int readRemain = 0;

    ADC0_InitTimer0A(16000000);
    PWM_Init(800000, 800000, 200000, 200000);
    SetupSerial();

    while(true){
        if (createdSamp == 1 && readDist == 1)
        {
            if (distance >= 17)
            {
                foundSamp++;
                if (foundSamp == numSamp)
                {
                    readColor = 1;
                    readDist = 0;
                    foundSamp = 0;
                }
            }
            else
            {
                foundSamp = 0;
            }
            createdSamp = 0;
        }

        if (Done == 1 && readColor == 1)
        {
            if ((GPIO_PORTE_DATA_R & 0x3) == 0x0)
            {
                if (countFreq >= 2)
                {
                    countFreq = 0;
                    red = Period / 800.0;
                    GPIO_PORTE_DATA_R &= ~0x3;
                    GPIO_PORTE_DATA_R |= 0x1;

                }
            }
            if ((GPIO_PORTE_DATA_R & 0x3) == 0x1)
            {
                if (countFreq >= 2)
                {
                    countFreq = 0;
                    blue = Period / 800.0;
                    GPIO_PORTE_DATA_R &= ~0x3;
                    GPIO_PORTE_DATA_R |= 0x03;

                }
            }
            if ((GPIO_PORTE_DATA_R & 0x3) == 0x3)
            {
                if (countFreq >= 2)
                {
                    countFreq = 0;
                    green = Period / 800.0;
                    GPIO_PORTE_DATA_R &= ~0x3;
                    GPIO_PORTE_DATA_R |= 0x0;
                    accept=1;

                }
            }
            if (accept == 1)
            {
                if ((red >= 15 && red <= 45) && (blue >= 70 && blue <= 110) && (green >= 85 && green <= 125))
                {
                    color = "red";
                    GPIO_PORTA_DATA_R = 0xA8;
                    GPIO_PORTE_DATA_R &= ~0x30;
                    GPIO_PORTE_DATA_R |= 0x20;
                    readRemain = 1;
                    pwm_dutyA(200000);
                    pwm_dutyB(200000);

                }
                else if (((red >= 55 && red <= 85) && (blue >= 25 && blue <= 50) && (green >= 70 && green <= 115)) && (readRemain == 1))
                {
                    color = "purple";
                    GPIO_PORTA_DATA_R = 0xA4;
                    GPIO_PORTE_DATA_R &= ~0x30;
                    GPIO_PORTE_DATA_R |= 0x10;
                    pwm_dutyA(200000);
                    pwm_dutyB(425000);
                }
                else if (((red >= 135 && red <= 170) && (blue >= 15 && blue <= 45) && (green >= 40 && green <= 80)) && (readRemain == 1))
                {
                    color = "blue";
                    GPIO_PORTA_DATA_R = 0xA8;
                    GPIO_PORTE_DATA_R &= ~0x30;
                    GPIO_PORTE_DATA_R |= 0x20;
                    pwm_dutyA(200000);
                    pwm_dutyB(200000);
                }
                else if (((red >= 70 && red <= 120) && (blue >= 60 && blue <= 90) && (green >= 35 && green <= 70)) && (readRemain == 1))
                {
                    color = "green";
                    GPIO_PORTA_DATA_R = 0x0;
                    GPIO_PORTE_DATA_R &= ~0x30;

                }
                else if (((red >= 15 && red <= 25) && (blue >= 35 && blue <= 45) && (green >= 13 && green <= 26)) && (readRemain == 1))
                {
                    color = "yellow";
                }
                else
                {
                    color = "continue";
                }

                SerialWriteInt(red);
                SerialWriteInt(blue);
                SerialWriteInt(green);
                accept = 0;
            }
        }
    }
}






