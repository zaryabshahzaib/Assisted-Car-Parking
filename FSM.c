/**
 * @file FSM.c
 * @author Zaryab Shahzaib (zshahzai@qatar.cmu.edu)
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
#include "serial.h"


/**
 * @brief Initialize Phase-Locked Loop
 * 
 */
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


struct State 
{
    int transition[17];
    char output[17]; // 1 ms units
    char output2[17]; // list of next states
};


const int arr[]= {0x0E,0x0D,0X0B,0X07};

const char total[4][4] = {
   {'1', '2', '3', 'A'},
   {'4', '5', '6', 'B'},
   {'7', '8', '9', 'C'},
   {'*', '0', '#', 'D'}
};

const int index[4][4] = {
   {1, 2, 3, 4},
   {5, 6, 7, 8},
   {9, 10, 11, 12},
   {13, 14, 15, 16}
};

/** Define Constants **/
#define s0 0  //base
#define s1 1 //button 2
#define s2 2 //button 3
#define s3 3 //button 4
#define s4 4 //button 5
#define s5 5 //button 6
#define s6 6 //button 7
#define s7 7 //button 8
#define s8 8 //button 9
#define s11 9 //2 pressed twice
#define s22 10 //button 3
#define s33 11 //button 4
#define s44 12 //button 5
#define s55 13 //button 6
#define s66 14 //button 7
#define s77 15 //button 8
#define s88 16 //button 9
#define s111 17 //2 pressed twice
#define s222 18 //button 3
#define s333 19 //button 4
#define s444 20 //button 5
#define s555 21 //button 6
#define s666 22 //button 7
#define s777 23 //button 8
#define s888 24 //button 9
#define s1111 25 //2 pressed twice
#define s2222 26 //button 3
#define s3333 27 //button 4
#define s4444 28 //button 5
#define s5555 29 //button 6
#define s6666 30 //button 7
#define s7777 31 //button 8
#define s8888 32 //button 9


typedef const struct State State_t;
State_t FSM[33] = {
     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'!','1','!','!','A','!','!','!','B','!','!','!', 'C', '*', '0', '#', 'D'},{'?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?'}},

     {{s0,s0,s11,s2,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'A','A','!','A','A','A','A','A','A','A','A','A','A','A','A','A','A'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s22,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'D','D','D','!','D','D','D','D','D','D','D','D', 'D', 'D', 'D', 'D', 'D'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s33,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'G','G','G','G','G','!','G','G','G','G','G','G', 'G', 'G', 'G', 'G', 'G'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s44,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'J','J','J','J','J','J','!','J','J','J','J','J', 'J', 'J', 'J', 'J', 'J'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s55,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'M','M','M','M','M','M','M','!','M','M','M','M', 'M', 'M', 'M', 'M', 'M'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s66,s7,s8,s0,s0,s0,s0,s0},{'P','P','P','P','P','P','P','P','P','!','P','P', 'P', 'P', 'P', 'P', 'P'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s77,s8,s0,s0,s0,s0,s0},{'T','T','T','T','T','T','T','T','T','T','!','T', 'T', 'T', 'T', 'T', 'T'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s7,s88,s0,s0,s0,s0,s0},{'W','W','W','W','W','W','W','W','W','W','W','!', 'W', 'W', 'W', 'W', 'W'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s111,s2,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'B','B','!','B','B','B','B','B','B','B','B','B', 'B', 'B', 'B', 'B', 'B'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s222,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'E','E','E','!','E','E','E','E','E','E','E','E', 'E', 'E', 'E', 'E', 'E'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s333,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'H','H','H','H','H','!','H','H','H','H','H','H', 'H', 'H', 'H', 'H', 'H'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s444 ,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'K','K','K','K','K','K','!','K','K','K','K','K', 'K', 'K', 'K', 'K', 'K'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s555,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'N','N','N','N','N','N','N','!','N','N','N','N', 'N', 'N', 'N', 'N', 'N'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s666,s7,s8,s0,s0,s0,s0,s0},{'R','R','R','R','R','R','R','R','R','!','R','R', 'R', 'R', 'R', 'R', 'R'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s777,s8,s0,s0,s0,s0,s0},{'U','U','U','U','U','U','U','U','U','U','!','U', 'U', 'U', 'U', 'U', 'U'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s7,s888,s0,s0,s0,s0,s0},{'X','X','X','X','X','X','X','X','X','X','X','!', 'X', 'X', 'X', 'X', 'X'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1111,s2,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'C','C','!','C','C','C','C','C','C','C','C','C', 'C', 'C', 'C', 'C', 'C'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2222,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'F','F','F','!','F','F','F','F','F','F','F','F', 'F', 'F', 'F', 'F', 'F'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3333,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'I','I','I','I','I','!','I','I','I','I','I','I', 'I', 'I', 'I', 'I', 'I'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4444 ,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'L','L','L','L','L','L','!','L','L','L','L','L', 'L', 'L', 'L', 'L', 'L'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5555,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'O','O','O','O','O','O','O','!','O','O','O','O', 'O', 'O', 'O', 'O', 'O'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6666,s7,s8,s0,s0,s0,s0,s0},{'S','S','S','S','S','S','S','S','S','!','S','S', 'S', 'S', 'S', 'S', 'S'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s7777,s8,s0,s0,s0,s0,s0},{'V','V','V','V','V','V','V','V','V','V','!','V', 'V', 'V', 'V', 'V', 'V'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s7,s8888,s0,s0,s0,s0,s0},{'Y','Y','Y','Y','Y','Y','Y','Y','Y','Y','Y','!', 'Y', 'Y', 'Y', 'Y', 'Y'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'2','2','!','2','2','2','2','2','2','2','2','2', '2', '2', '2', '2', '2'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'3','3','3','!','3','3','3','3','3','3','3','3', '3', '3', '3', '3', '3'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'4','4','4','4','4','!','4','4','4','4','4','4', '4', '4', '4', '4', '4'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4 ,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'5','5','5','5','5','5','!','5','5','5','5','5', '5', '5', '5', '5', '5'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'6','6','6','6','6','6','6','!','6','6','6','6', '6', '6', '6', '6', '6'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'7','7','7','7','7','7','7','7','7','!','7','7', '7', '7', '7', '7', '7'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'8','8','8','8','8','8','8','8','8','8','!','8', '8', '8', '8', '8', '8'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},

     {{s0,s0,s1,s2,s0,s3,s4,s5,s0,s6,s7,s8,s0,s0,s0,s0,s0},{'9','9','9','9','9','9','9','9','9','9','9','!', '9', '9', '9', '9', '9'},{'?','1','?','?','A','?','?','?','B','?','?','?','C','*','0','#','D'}},
 };


/**
 * @brief Main function implements required funtionality
 * 
 * @return int 
 */
int main(void)
{
    char val;
    char pressedbutton;
    char final[2];
    
    bool debouncephase = false;
    bool buttonpressed = false;
    bool starttimer = true;

    int i;
    int j;
    int debounceCounter = 0;    
    int ival;
    int jval;
    int timeCounter = 0;
    int currentstate = 0;
    int cs = 0;
    int bindex;

    bool state[4][4] = {
       {false, false, false, false},
       {false, false, false, false},
       {false, false, false, false},
       {false, false, false, false}
    };

    PLLInit();
    SetupSerial();
    SystickInit();
    
    SYSCTL_RCGCGPIO_R |= 0x0A; //Enable clock for PORT F and B
    GPIO_PORTB_LOCK_R = 0x4C4F434B; // this value unlocks the GPIOCR register.
    GPIO_PORTB_CR_R = 0xFF;
    GPIO_PORTB_AMSEL_R = 0x00; // disable analog functionality
    GPIO_PORTB_PCTL_R = 0x00000000; // Select GPIO mode in PCTL
    GPIO_PORTB_DIR_R = 0x0F;//Configure pins 0-3 to be output port B
    GPIO_PORTB_AFSEL_R = 0x00; // Disable alternate functionality
    GPIO_PORTB_DEN_R = 0xFF; //Enable digital ports

    GPIO_PORTD_LOCK_R = 0x4C4F434B; // this value unlocks the GPIOCR register.
    GPIO_PORTD_CR_R = 0xFF;
    GPIO_PORTD_AMSEL_R = 0x00; // disable analog functionality
    GPIO_PORTD_PCTL_R = 0x00000000; // Select GPIO mode in PCTL
    GPIO_PORTD_DIR_R = 0x00; //Configure pins 0-3 to be input D
    GPIO_PORTD_AFSEL_R = 0x00; // Disable alternate functionality
    GPIO_PORTD_DEN_R = 0xFF; //Enable digital ports


    
    while(1){

        if (starttimer){
            timeCounter++;
        }

        if(debouncephase){
            debounceCounter++;
        }

        for (i=0; i<4; i++){
            GPIO_PORTB_DATA_R = arr[i];
            for (j=0; j<4; j++){
                 if (GPIO_PORTD_DATA_R == arr[j]){
                       state[i][j] = true;
                       debouncephase = true;
                 }
                 if(GPIO_PORTD_DATA_R != arr[j] && state[i][j] && debounceCounter >=100){
                     debouncephase = false;
                     debounceCounter = 0;
                     state[i][j] = false;
                     val = total[i][j];
                     ival= i;
                     jval = j;
                     buttonpressed = true;
                     pressedbutton = val;
                 }

            }
        }

        if (timeCounter >= 10000){
            if(FSM[cs].output[0] != '!'){
                final[0] = FSM[cs].output[0];
                final[1] = 0;
                SerialWrite(final);
            }
            if (FSM[cs].output2[0] != '?'){
                final[0] = FSM[cs].output2[0];
                final[1] = 0;
                SerialWrite(final);
            }
            cs = FSM[cs].transition[0];
            timeCounter = 0;
            buttonpressed = false;
        }

        if(buttonpressed){
            buttonpressed = false;
            timeCounter = 0;
            bindex = index[ival][jval];
            if(FSM[cs].output[bindex] != '!'){
                final[0] = FSM[cs].output[bindex];
                final[1] = 0;
                SerialWrite(final);
            }
            if (FSM[cs].output2[bindex] != '?'){
                final[0] = FSM[cs].output2[bindex];
                final[1] = 0;
                SerialWrite(final);
            }
            cs = FSM[cs].transition[bindex];
        }
        SysTick_Wait100microsec(1);
    }
}