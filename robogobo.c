// Robot PWM Controller code

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <wait.h>
#include "clock.h"
#include "uart0.h"
#include "uart1.h"
#include "tm4c123gh6pm.h"
#include "parseFields.h"

#define MAX_CHARS 80
#define MAX_FIELDS 10

// pin assignments
#define MOTOR_0_A   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4))) //pb4
#define MOTOR_0_B   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4))) //pb5
#define MOTOR_1_A   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) //pe4
#define MOTOR_1_B   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4))) //pe5

#define MOTOR_2_A   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4))) //pb6
#define MOTOR_2_B   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 7*4))) //pb7
#define MOTOR_3_A   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4))) //pc4
#define MOTOR_3_B   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4))) //pc5

//motor masks
#define MOTOR_0_A_MASK 16
#define MOTOR_0_B_MASK 32
#define MOTOR_1_A_MASK 16
#define MOTOR_1_B_MASK 32

#define MOTOR_2_A_MASK 64
#define MOTOR_2_B_MASK 128
#define MOTOR_3_A_MASK 16
#define MOTOR_3_B_MASK 32

//globals
volatile int16_t forward, turn;



// finite state machine for UART read
typedef enum{
    WAIT_HEADER,
    WAIT_CMD,
    WAIT_HI,
    WAIT_LO
}readState;

//global variable for state machine
volatile readState state = WAIT_HEADER;
volatile char cmd;
volatile uint8_t hi, lo;


// Initialize Hardware
void initHw()
{
    // Initialize system clock to 80 MHz
    initSystemClockTo80Mhz();
    _delay_cycles(6);

    //module clocks
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R4;
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;

    //GPIO enable
    GPIO_PORTB_DIR_R |= MOTOR_0_A_MASK | MOTOR_0_B_MASK | MOTOR_2_A_MASK | MOTOR_2_B_MASK;
    GPIO_PORTB_DEN_R |= MOTOR_0_A_MASK | MOTOR_0_B_MASK | MOTOR_2_A_MASK | MOTOR_2_B_MASK;
    GPIO_PORTC_DIR_R |= MOTOR_3_A_MASK | MOTOR_3_B_MASK;
    GPIO_PORTC_DEN_R |= MOTOR_3_A_MASK | MOTOR_3_B_MASK;
    GPIO_PORTE_DIR_R |= MOTOR_1_A_MASK | MOTOR_1_B_MASK;
    GPIO_PORTE_DEN_R |= MOTOR_1_A_MASK | MOTOR_1_B_MASK;

    //PWM Enable
    PWM0_CTL_R = 0;
    GPIO_PORTB_AFSEL_R |= MOTOR_0_A_MASK | MOTOR_0_B_MASK | MOTOR_2_A_MASK | MOTOR_2_B_MASK;
    GPIO_PORTC_AFSEL_R |= MOTOR_3_A_MASK | MOTOR_3_B_MASK;
    GPIO_PORTE_AFSEL_R |= MOTOR_1_A_MASK | MOTOR_1_B_MASK;

    //assign gpio pins to pwm outputs
    GPIO_PORTB_PCTL_R  |= GPIO_PCTL_PB6_M0PWM0 | GPIO_PCTL_PB7_M0PWM1 | GPIO_PCTL_PB4_M0PWM2 | GPIO_PCTL_PB5_M0PWM3;
    GPIO_PORTC_PCTL_R  |= GPIO_PCTL_PC4_M0PWM6 | GPIO_PCTL_PC5_M0PWM7;
    GPIO_PORTE_PCTL_R  |= GPIO_PCTL_PE4_M0PWM4 | GPIO_PCTL_PE5_M0PWM5;
    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;
    SYSCTL_RCC_R &= ~(0x000E0000); // div by 2

    //init pwm generators
    PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO;
    PWM0_0_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO;
    PWM0_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
    PWM0_1_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
    PWM0_2_GENA_R = PWM_2_GENA_ACTCMPAD_ONE | PWM_2_GENA_ACTLOAD_ZERO;
    PWM0_2_GENB_R = PWM_2_GENB_ACTCMPBD_ONE | PWM_2_GENB_ACTLOAD_ZERO;
    PWM0_3_GENA_R = PWM_3_GENA_ACTCMPAD_ONE | PWM_3_GENA_ACTLOAD_ZERO;
    PWM0_3_GENB_R = PWM_3_GENB_ACTCMPBD_ONE | PWM_3_GENB_ACTLOAD_ZERO;

    //set pwm load and compare values
    PWM0_0_CMPA_R = 0;      //0: blr
    PWM0_0_CMPB_R = 0;      //1: blf
    PWM0_0_LOAD_R = 1024;
    PWM0_1_CMPA_R = 0;      //2: frf
    PWM0_1_CMPB_R = 0;      //3: brr
    PWM0_1_LOAD_R = 1024;
    PWM0_2_CMPA_R = 0;      //4: brf
    PWM0_2_CMPB_R = 0;      //5: frr
    PWM0_2_LOAD_R = 1024;
    PWM0_3_CMPA_R = 0;      //6: flf
    PWM0_3_CMPB_R = 0;      //7: flr
    PWM0_3_LOAD_R = 1024;

    //enable pwm blocks
    PWM0_0_CTL_R |= PWM_0_CTL_ENABLE;
    PWM0_1_CTL_R |= PWM_0_CTL_ENABLE;
    PWM0_2_CTL_R |= PWM_0_CTL_ENABLE;
    PWM0_3_CTL_R |= PWM_0_CTL_ENABLE;

    //enable pwm outputs
    PWM0_CTL_R |= PWM_CTL_GLOBALSYNC0 | PWM_CTL_GLOBALSYNC1 | PWM_CTL_GLOBALSYNC2 | PWM_CTL_GLOBALSYNC3;
    PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN | PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN;
    PWM0_ENABLE_R |= PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;

    // Configure Timer 0
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER0_TAILR_R = 8e4;                           // set load value to 8e4 for 100 Hz interrupt rate
    TIMER0_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    TIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER0A-16);              // turn-on interrupt 35 (TIMER0A)

    _delay_cycles(6);

    initUart0();
    initUart1();

    //configure Uart0 RX interrupt
    UART0_IM_R |= UART_IM_RXIM | UART_IM_RTIM;
    UART0_IFLS_R = UART_IFLS_RX1_8;
    NVIC_EN0_R |= 1 << (INT_UART0-16);
}

void motorIsr(void)
{

    int16_t m1, m2;
    m1 = abs(forward - turn);  // right side motors
    m2 = abs(forward + turn);  // left side motors

    // if m1 or m2 go outside the maximum
    // PWM_CMP value of 1023, then they are both scaled down
    // otherwise, the PWM module will fault.
    int16_t maxVal = (m1 > m2) ? m1 : m2;
    if (maxVal > 1023) {
        m1 = (m1 * 1023) / maxVal;
        m2 = (m2 * 1023) / maxVal;
    }

    if (forward - turn >= 0)
    {
        PWM0_1_CMPB_R = 0;   // back right reverse
        PWM0_2_CMPB_R = 0;   // front right reverse
        PWM0_2_CMPA_R = m1;   // back right forward
        PWM0_1_CMPA_R = m1;   // front right forward
    }
    else
    {
        PWM0_2_CMPA_R = 0;   // back right forward
        PWM0_1_CMPA_R = 0;   // front right forward
        PWM0_1_CMPB_R = m1;  // back right reverse
        PWM0_2_CMPB_R = m1;  // front right reverse
    }

    if (forward + turn >= 0)
    {
        PWM0_0_CMPA_R = 0;   // back left reverse
        PWM0_3_CMPB_R = 0;   // front left reverse
        PWM0_0_CMPB_R = m2;  // back left forward
        PWM0_3_CMPA_R = m2;  // front left forward
    }
    else
    {
        PWM0_0_CMPB_R = 0;   // back left forward
        PWM0_3_CMPA_R = 0;   // front left forward
        PWM0_0_CMPA_R = m2;  // back left reverse
        PWM0_3_CMPB_R = m2;  // front left reverse
    }
    TIMER0_ICR_R = TIMER_ICR_TATOCINT;              // clear interrupt flag
}

void Uart0Isr(void)
{
    // while rx buffer is not empty
    while ((UART0_FR_R & UART_FR_RXFE) == 0)
    {
        //read byte
        char c = getcUart0();
        
        //enter data parsing state machine
        //data is received as either MD:0000
        //                      or   MT:0000
        // Header:    'M'
        // Command:   'D' or 'T'
        // hi byte:   0-255
        // lo byte:   0-255

        switch (state)
        {
            case WAIT_HEADER:
                if (c == 'M')
                    state = WAIT_CMD;
                break;

            case WAIT_CMD:
                if (c == 'D' || c == 'T')
                {
                    cmd = c;
                    state = WAIT_HI;
                }
                else
                {
                    state = WAIT_HEADER;
                }
                break;

            case WAIT_HI:
                hi = c;
                state = WAIT_LO;
                break;

            case WAIT_LO:
            {
                lo = c;
                int16_t val = ((hi << 8) | lo);

                if (cmd == 'D')
                    forward = -val;
                else if (cmd == 'T')
                    turn = val;

                state = WAIT_HEADER;
                break;
            }
        }
    }

    UART0_ICR_R = UART_ICR_RXIC | UART_ICR_RTIC; //clear int flag
}


int main(void)
{
    //start of program, initialize PWM to 0
    initHw();
    forward = 0;
    turn = 0;
    while (true)
    {}
}
