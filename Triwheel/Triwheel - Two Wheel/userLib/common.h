#ifndef common
#define common

#include <stdint.h>
#include <stdbool.h>
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/inc/hw_memmap.h"
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/inc/hw_types.h"
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/inc/hw_gpio.h"
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/inc/hw_ints.h"
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/inc/hw_uart.h"
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/inc/tm4c123gh6pm.h"
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/driverlib/sysctl.h"
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/driverlib/gpio.h"
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/driverlib/interrupt.h"
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/driverlib/timer.h"
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/driverlib/uart.h"
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/driverlib/pin_map.h"
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/driverlib/pwm.h"
#include "C:/Users/hp/Desktop/TivaWare_C_Series-2.1.2.111/driverlib/qei.h"

enum {A, B};
extern int32_t maxPWM, minPWM;

#define PIDfrequency 200
#define PWMfrequency 2000
#define QEIfrequency 40
#define motorDirectionRegister GPIO_PORTF_BASE
#define A1 GPIO_PIN_0
#define A2 GPIO_PIN_1
#define B1 GPIO_PIN_2
#define B2 GPIO_PIN_3

void setPWM(int pwm,int i);
int calculateRPM(int i);
unsigned absolute(int x);
void UART_TransmitString(char* string,int i);
void UART_OutDec(int x, int i);
void GraphPlot0(int data1, int data2, int data3, int data4);
void GraphPlot1(int data1, int data2, int data3, int data4);

#endif
