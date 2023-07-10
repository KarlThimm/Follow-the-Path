/*===================================CPEG222====================================
 * Program:      Project 3 template
 * Authors:     Karl Thimm
 * Date:        10/22/2021
 * This is a template that you can use to write your project 3 code, for mid-stage and final demo.
==============================================================================*/
/*-------------- Board system settings. PLEASE DO NOT MODIFY THIS PART ----------*/
#ifndef _SUPPRESS_PLIB_WARNING          //suppress the plib warning during compiling
#define _SUPPRESS_PLIB_WARNING
#endif
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
/*----------------------------------------------------------------------------*/

#include <xc.h>   //Microchip XC processor header which links to the PIC32MX370512L header
#include <stdio.h>  // need this for sprintf
#include <time.h>   // need this to randomize the seed of random numbers
#include <sys/attribs.h>
#include "proj4config.h" // Basys MX3 configuration header
#include "proj4led.h"    // Digilent Library for using the on-board LEDs
#include "proj4ssd.h"    // Digilent Library for using the on-board SSDs
#include "proj4lcd.h"    // Digilent Library for using the on-board LCD
#include "proj4acl.h"    // Digilent Library for using the on-board accelerometer
#include "proj4srv.h"



// below are keypad row and column definitions based on the assumption that JB will be used and columns are CN pins
// If you want to use JA or use rows as CN pins, modify this part
#define sw0 PORTFbits.RF3 //Sets up switch 0
#define sw1 PORTFbits.RF5 //Sets up switch 1
#define sw6 PORTBbits.RB10 //Sets up switch 6
#define sw7 PORTBbits.RB9 //Sets up switch 7
#define IR4 PORTDbits.RD9 //Pmod sensor 4
#define IR3 PORTDbits.RD11 //Pmod sensor 3
#define IR2 PORTDbits.RD10 //Pmod sensor 2
#define IR1 PORTDbits.RD8 //Pmod sensor 1
#define SSD_EMPTY_DIGIT 18
#define SYS_FREQ (80000000L)
#define _80Mhz_ (80000000L)
#define LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz 1426
#define LOOPS_NEEDED_TO_DELAY_ONE_MS (LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz * (SYS_FREQ / _80Mhz_))
#define TRUE 1
#define FALSE 0
#define BUTTON_DEBOUNCE_DELAY_MS 20
#define BtnC_RAW PORTFbits.RF0
#define BtnU_RAW PORTBbits.RB1
#define BtnD_RAW PORTAbits.RA15
#define BtnR_RAW PORTBbits.RB8
#define BtnL_RAW PORTBbits.RB0
char ssdIsOn = 0;
unsigned char d4 = SSD_EMPTY_DIGIT; //variable for SSD
unsigned char d3 = SSD_EMPTY_DIGIT; //variable for SSD
unsigned char d2 = SSD_EMPTY_DIGIT; //variable for SSD
unsigned char d1 = SSD_EMPTY_DIGIT; //variable for SSD
int counter = 0;
int seconds = 0; //Timer variable
int a = 0;
int b = 0;
//int line = 0;
char buttonsCLocked = FALSE; //Variable for button C when locked
char buttonsULocked = FALSE; //Variable for button U when locked
char buttonsLLocked = FALSE; //Variable for button L when locked
char buttonsRLocked = FALSE; //Variable for button R when locked
char buttonsDLocked = FALSE; //Variable for button D when locked
char pressedUnlockedBtnC = FALSE; //Variable for button D when unlocked
char pressedUnlockedBtnU = FALSE; //Variable for button U when unlocked
char pressedUnlockedBtnL = FALSE; //Variable for button L when unlocked
char pressedUnlockedBtnR = FALSE; //Variable for button R when unlocked
char pressedUnlockedBtnD = FALSE; //Variable for button D when unlocked
int buttonsLocked = FALSE;
int clap = 0; //Variable for claps
//int count = 0;


// subrountines
void CNConfig();
//void Game(eKey key) ;
void updateLCD();
void toggle_SSD();
void SSD_Timer3();
void handle_raw_button_presses();
void delay_ms();
void clap_detect();
void initialize_ports();
//Timer3ISR(void);

int main(void) {

    /* Initialization of LED, LCD, SSD, etc */
    DDPCONbits.JTAGEN = 0; // Required to use Pin RA0 (connected to LED 0) as IO
    TRISFbits.TRISF3 = 1; //Ports for buttons
    TRISFbits.TRISF5 = 1;
    TRISBbits.TRISB10 = 1;
    ANSELBbits.ANSB10 = 0;
    TRISBbits.TRISB9 = 1;
    ANSELBbits.ANSB9 = 0;
    TRISBbits.TRISB8 = 0;
    TRISAbits.TRISA15 = 0;
    ANSELBbits.ANSB8 = 0;
    TRISBbits.TRISB4 = 1; //For Microphone
    ANSELBbits.ANSB4 = 1; //Ansel For microphone
    LCD_Init();
    ACL_Init();
    LCD_Init();
    SRV_Init();
    SSD_Init();
    toggle_SSD();
    SSD_Timer3();
    LED_Init();
    handle_raw_button_presses(); //Button debouncing
    delay_ms();
    PMODS_InitPin();
    ADC_Init(); //For reading ADC
    clap_detect(); //Detect claps
    initialize_ports();

    while (1) {
        LCD_WriteStringAtPos("Team:BoatMobile", 0, 0); //Prints team on LCD
        LCD_WriteStringAtPos("Team ID:26", 1, 0); //Prints team ID on LCD
        handle_raw_button_presses(); //Calls button debouncing
        clap_detect(); //calls clap detection
        //delay_ms();
        //int count = 0;
        //count++;
        if (pressedUnlockedBtnC || clap >= 2) { //Starts program if claps = 2 or bnt C is pressed
            T3CONbits.ON = 1;
            /*if(sw0 == 0 && sw1 == 0 && sw6 == 0 && sw7 == 0)
            {
                T3CONbits.ON = 0;
                seconds=0;
                d4=17;
                d3=17;
                d2=17;
                d1=17;        
                toggle_SSD();
            }
            else if(sw0 == 1 && sw1 == 1 && sw6 == 1 && sw7 == 1)
            {
                T3CONbits.ON = 0;
                seconds=0;
                d4=17;
                d3=17;
                d2=17;
                d1=17;        
                toggle_SSD();
            }
            else if(sw0 == 0 && sw1 == 0 && sw6 == 1 && sw7 == 1)
            {
                T3CONbits.ON = 0;
                seconds=0;
                d4=17;
                d3=17;
                d2=17;
                d1=17;        
                toggle_SSD();
            }
            else if(sw0 == 1 && sw1 == 1 && sw6 == 0 && sw7 == 0)
            {
                T3CONbits.ON = 0;
                seconds=0;
                d4=17;
                d3=17;
                d2=17;
                d1=17;        
                toggle_SSD();
            }
            else
            {
                T3CONbits.ON = 1;
            }
        
            if (sw0 == 0 && sw1 == 0) {
                SRV_SetPulseMicroseconds0(1500);
                LATA &= 0xFF00;
                LCD_WriteStringAtPos("STP",1,13);
            
            }
            else if (sw0 == 1 && sw1 == 0) {
                SRV_SetPulseMicroseconds0(1000);
                LCD_WriteStringAtPos("FWD",1,13);
                LATA |= 0xFFC0;
            
            }
            else if (sw0 == 0 && sw1 == 1) {
                SRV_SetPulseMicroseconds0(2000);
                LCD_WriteStringAtPos("REV",1,13);
                LATA |= 0xFF30;
            }
            else if (sw0 = 1 && sw1 == 1) {
                SRV_SetPulseMicroseconds0(1500);
                LATA &= 0xFF00;
                LCD_WriteStringAtPos("STP",1,13);
            }
            if (sw6 == 0 && sw7 == 0) {
                SRV_SetPulseMicroseconds1(1500);
                LCD_WriteStringAtPos("STP",1,0);
                LATA &= 0xFF00;
            }
            else if (sw6 == 1 && sw7 == 0) {
                SRV_SetPulseMicroseconds1(2000);
                LCD_WriteStringAtPos("FWD",1,0);
                LATA |= 0xFF03;
            }
            else if (sw6 == 0 && sw7 == 1) {
                SRV_SetPulseMicroseconds1(1000);
                LCD_WriteStringAtPos("REV",1,0);
                LATA |= 0xFF0C;
            }
            else if (sw6 == 1 && sw7 == 1) {
                SRV_SetPulseMicroseconds1(1500);
                LCD_WriteStringAtPos("STP",1,0);
                LATA &= 0xFF00;
            }*/
            if (IR4 == 0 && IR3 == 0 && IR2 == 0 && IR1 == 0) //0 Start/Stop, need code so it stops on second line not first
            {
                //LCD_WriteStringAtPos("0000", 1, 0);
                //T3CONbits.ON = 1;
                if (seconds < 380) {
                    SRV_SetPulseMicroseconds0(1000);
                    SRV_SetPulseMicroseconds1(2000);
                } else if (seconds >= 380) {
                    SRV_SetPulseMicroseconds0(1500);
                    SRV_SetPulseMicroseconds1(1500);
                    T3CONbits.ON = 0;
                }
            } else if (IR4 == 1 && IR3 == 0 && IR2 == 0 && IR1 == 0) //1 Move left, left side not over track
            {
                SRV_SetPulseMicroseconds0(1500);
                SRV_SetPulseMicroseconds1(2000);
            } else if (IR4 == 0 && IR3 == 1 && IR2 == 0 && IR1 == 0) //2 Unlikely sensor reading
            {
                SRV_SetPulseMicroseconds0(1000);
                SRV_SetPulseMicroseconds1(2000);
            } else if (IR4 == 1 && IR3 == 1 && IR2 == 0 && IR1 == 0) //3 Move left, left side not over track
            {
                SRV_SetPulseMicroseconds0(1500);
                SRV_SetPulseMicroseconds1(2000);
            } else if (IR4 == 0 && IR3 == 0 && IR2 == 1 && IR1 == 0) //4 Unlikely sensor reading
            {
                SRV_SetPulseMicroseconds0(1000);
                SRV_SetPulseMicroseconds1(2000);
            } else if (IR4 == 1 && IR3 == 0 && IR2 == 1 && IR1 == 0) //5 Move left, left side not over track
            {
                SRV_SetPulseMicroseconds0(1500);
                SRV_SetPulseMicroseconds1(2000);
            } else if (IR4 == 0 && IR3 == 1 && IR2 == 1 && IR1 == 0) //6 Unlikely sensor reading 
            {
                SRV_SetPulseMicroseconds0(1000);
                SRV_SetPulseMicroseconds1(2000);
            } else if (IR4 == 1 && IR3 == 1 && IR2 == 1 && IR1 == 0) //7 Move left, left side not over track
            {
                SRV_SetPulseMicroseconds0(1500);
                SRV_SetPulseMicroseconds1(2000);
            } else if (IR4 == 0 && IR3 == 0 && IR2 == 0 && IR1 == 1) //8 Move right, right side not over track
            {
                SRV_SetPulseMicroseconds0(1000);
                SRV_SetPulseMicroseconds1(1500);
            } else if (IR4 == 1 && IR3 == 0 && IR2 == 0 && IR1 == 1) //9 Move forward, centered on track
            {
                SRV_SetPulseMicroseconds0(1000);
                SRV_SetPulseMicroseconds1(2000);
            } else if (IR4 == 0 && IR3 == 1 && IR2 == 0 && IR1 == 1) //10 Unlikely sensor reading
            {
                SRV_SetPulseMicroseconds0(1000);
                SRV_SetPulseMicroseconds1(2000);
            } else if (IR4 == 1 && IR3 == 1 && IR2 == 0 && IR1 == 1) //11 Unlikely sensor reading 
            {
                SRV_SetPulseMicroseconds0(1000);
                SRV_SetPulseMicroseconds1(2000);
            } else if (IR4 == 0 && IR3 == 0 && IR2 == 1 && IR1 == 1) //12 move right, left side not on track 
            {
                SRV_SetPulseMicroseconds0(1000);
                SRV_SetPulseMicroseconds1(1500);
            } else if (IR4 == 1 && IR3 == 0 && IR2 == 1 && IR1 == 1) //13 Unlikely sensor reading
            {
                SRV_SetPulseMicroseconds0(1000);
                SRV_SetPulseMicroseconds1(2000);
            } else if (IR4 == 0 && IR3 == 1 && IR2 == 1 && IR1 == 1) //14 move right, left side not over track
            {
                SRV_SetPulseMicroseconds0(1000);
                SRV_SetPulseMicroseconds1(1500);
            } /*else if (IR4 == 1 && IR3 == 1 && IR2 == 1 && IR1 == 1) //15 Completely off track
            {
                SRV_SetPulseMicroseconds0(2000);
                SRV_SetPulseMicroseconds1(1000);
            }*/


            //int Test = ADC_AnalogRead(4); //For reading claps  
        }
    }
}

void __ISR(_TIMER_3_VECTOR, ipl7) Timer3ISR(void) { //Sets up timer ISR
    //lat_SSD_AN1 = lat_SSD_AN1 = lat_SSD_AN1 = lat_SSD_AN1 = 1;
    IEC0bits.T3IE = 0;
    counter++;
    if (counter == 1) {
        seconds++;
        counter = 0;
    }
    d4 = 17;
    d3 = seconds / 100; //Counts up from 0
    d2 = (seconds % 100) / 10;
    d1 = (seconds % 100) % 10;
    toggle_SSD();
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
}

void SSD_Timer3() { //SSD timer setup
    PR3 = 3787;
    T3CONbits.TCKPS = 7; //1:264
    T3CONbits.TGATE = 0;
    T3CONbits.TCS = 0;
    T3CONbits.ON = 0; //timer off
    IPC3bits.T3IP = 7; //priority
    IPC3bits.T3IS = 2; //sub priority
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    macro_enable_interrupts();
}

/*void __ISR(_TIMER_4_VECTOR, ipl7) Timer4ISR(void) //ISR For claps to start robot(might not need)
{
 
}*/

void toggle_SSD() { //Toggles SSD for timer
    //int number;
    if (ssdIsOn) {
        // clear the SSD
        SSD_WriteDigits(SSD_EMPTY_DIGIT, SSD_EMPTY_DIGIT, SSD_EMPTY_DIGIT, SSD_EMPTY_DIGIT, 0, 0, 0, 0);
    } else {
        // show decimal number with leading zeroes
        /*number = rand() % 256;
        unsigned char d4 = (number / 1000) % 10;
        unsigned char d3 = (number / 100) % 10;
        unsigned char d2 = (number / 10) % 10;
        unsigned char d1 = number % 10; */
        // logic to remove leading zeroes could go here
        SSD_WriteDigits(d1, d2, d3, d4, 0, 0, 0, 0);

    }
}

/*void initADC() //Initialize ADC (Manual)
{
    AD1CON1 = 0; // manual conversion sequence control
    AD1CHS = 0x00040000; // Connect RB4/AN4 as CH0 input ..
    AD1CSSL = 0; // no scanning required
    AD1CON2 = 0; // use MUXA, AVss/AVdd used as Vref+/-
    AD1CON3 = 0x0002; // Tad= 6 x Tpb
    AD1CON1bits.ON = 1;   // turn on the ADC
}

short readADC() //Basic conversion routine (manual)
{
    AD1CON1bits.SAMP = 1; // 1. start sampling
    for (TMR2=0; TMR2<100; TMR2++); //2. wait for sampling time
    AD1CON1bits.SAMP = 0; // 3. start the conversion
    while (!AD1CON1bits.DONE); // 4. wait conversion complete
    return ADC1BUF0; // 5. read result
}*/

/*void initADC() //Initialize ADC (Automatic)
{
    AD1CON1 = 0x00E0; // automatic conversion after sampling
    AD1CHS = 0x00040000; // Connect RB4/AN4 as CH0 input
    AD1CSSL = 0; // no scanning required
    AD1CON2 = 0; // use MUXA, AVss/AVdd used as Vref+/-
    AD1CON3 = 0x1F3F; // Tad = 128 x Tpb, Sample time = 31 Tad
    AD1CON1bits.ON = 1;   // turn on the ADC
}

short readADC() //Basic conversion routine (Automatic)
{
    AD1CON1bits.SAMP = 1; // 1. start sampling
    while(AD1CON1bits.SAMP);  // 2. wait until acquisition is done
    while (!AD1CON1bits.DONE); // 3. wait conversion complete
    return ADC1BUF0; // 4. read conversion result
}*/

void delay_ms(int ms) { //Delay function
    int i;
    for (i = 0; i < ms * LOOPS_NEEDED_TO_DELAY_ONE_MS; i++) {
    }
}

void handle_raw_button_presses() { //Button debouncing function
    pressedUnlockedBtnC = FALSE;
    pressedUnlockedBtnU = FALSE;
    pressedUnlockedBtnD = FALSE;
    pressedUnlockedBtnL = FALSE;
    pressedUnlockedBtnR = FALSE;
    if ((BtnC_RAW || BtnU_RAW || BtnD_RAW || BtnR_RAW || BtnL_RAW) && !buttonsLocked) {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLocked = TRUE;
        pressedUnlockedBtnC = BtnC_RAW;
        pressedUnlockedBtnU = BtnU_RAW;
        pressedUnlockedBtnD = BtnD_RAW;
        pressedUnlockedBtnL = BtnL_RAW;
        pressedUnlockedBtnR = BtnR_RAW;
    } else if (!(BtnC_RAW || BtnU_RAW || BtnD_RAW || BtnR_RAW || BtnL_RAW) && buttonsLocked) {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLocked = FALSE;
    }
}

void clap_detect() { //Function for detecting claps
    if (ADC_AnalogRead(4) > 610) {
        clap++;
    }
}

void initialize_ports() { //Ports for buttons
    DDPCONbits.JTAGEN = 0; // Statement is required to use Pin RA0 as IO
    TRISFbits.TRISF0 = 1;
    TRISBbits.TRISB1 = 1;
    ANSELBbits.ANSB1 = 0;
    TRISBbits.TRISB0 = 1;
    ANSELBbits.ANSB0 = 0;
    TRISFbits.TRISF4 = 1;
    TRISBbits.TRISB8 = 1;
    ANSELBbits.ANSB8 = 0;
    TRISAbits.TRISA15 = 1;
    TRISFbits.TRISF3 = 1;
}