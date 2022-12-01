#include "mbed.h"
#include <MKL25Z4.h>
#include <string>
#include <iostream>

// main() runs in its own thread in the OS
#define RS 0x04 /* PTA2 Pin */
#define RW 0x10 /* PTA4 Pin */
#define EN 0x20 /* PTA5 Pin */

AnalogIn pot(PTB1);   /* Potentiometer middle pin connected to P0_11, other two ends connected to GND and 3.3V */
AnalogIn TempSensor(PTB0);
AnalogIn Fz(PTB2);
AnalogIn swch(PTB3);
DigitalOut led(LED1);  /* LED blinks with a delay based on the analog input read */

DigitalOut IN1(PTE31); //Pines del puente H
DigitalOut IN2(PTA17);

/*Functions*/
void delayMs(int n);

void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void); 

void LCD_set(float n);
void LCD_displayWish(float n, int ms);
void LCD_displayTemp(float n, int ms);

void giroStop();
void giroIzq();
void giroDer();

void holdOn();

void vRead();

int sensorValue;
float value; //Variable para FZ0340
int over = 0;

/*Main function*/
int main(void){
    LCD_init();
    int chck = 0;
    float ain, temporal, wished_temp;    // Variables to store the analog input
    float f;                // Variables for temperature
    ain = pot.read(); /* Read analog value (output will be any value between 0 and 1 */
    while (1) {
        vRead();
        if(over == 1){
            while(over){
                LCD_command(1); /* clear display */
                LCD_command(0x80); /* set cursor at first line */
                delayMs(500);
                giroStop();
                LCD_data('O');
                LCD_data('v');
                LCD_data('e');

                LCD_data('r');
                LCD_data('l');
                LCD_data('O');
                LCD_data('a');
                LCD_data('d');
                delayMs(1000);
                vRead();

            }
        }
        float statusON = swch.read();
        if(statusON < 0.5){
            holdOn();
        }
        else{
            f = (5.0 * (TempSensor.read() * 690) * 100.0)/1024.0; //Read Temperature
            if (chck == 0){
                LCD_displayTemp(f,2500);
                chck++;
            }
            temporal = ain*100; //Variable temporal para la lectura de la temperatura deseada
            ain = pot.read(); /* Read analog value (output will be any value between 0 and 1 */
            wished_temp = (ain*100);
            if((int)temporal != (int)(ain*100)){
                LCD_displayWish(wished_temp, 1000);
            }
            if(f < wished_temp - 2 && over == 0){
                vRead();
                if (over == 1) {

                    break;
                }
                giroIzq();
                while(f < wished_temp - 2 && over == 0){
                    vRead();
                    if (over == 1) {
                        break;
                    }
                    while(swch.read()<0.5){
                        holdOn();
                    }
                    LCD_command(1); /* clear display */
                    LCD_command(0x80); /* set cursor at first line */
                    delayMs(500);
                    LCD_data(' ');
                    LCD_data(' ');
                    LCD_data(' ');
                    LCD_data(' ');
                    LCD_data('H');
                    LCD_data('e');
                    LCD_data('a');
                    LCD_data('t');
                    LCD_data('i');
                    LCD_data('n');
                    LCD_data('g');
                    delayMs(1000);
                    vRead();
                    if (over == 1) {
                        break;
                    }
                    while(swch.read()<0.5){
                        holdOn();
                    }

                    f = (5.0 * (TempSensor.read() * 690) * 100.0)/1024.0; //Read Temperature
                    LCD_displayTemp(f, 1000);
                    vRead();
                    if (over == 1) {
                        break;
                    }
                    while(swch.read()<0.5){
                        holdOn();
                    }

                    temporal = ain*100; //Variable temporal para la lectura de la temperatura deseada
                    ain = pot.read(); /* Read analog value (output will be any value between 0 and 1 */
                    wished_temp = (ain*100);
                    LCD_displayWish(wished_temp, 1000);
                    vRead();
                    if (over == 1) {
                        break;
                    }
                    while(swch.read()<0.5){
                        holdOn();
                    }
                    
                }
                giroStop();
                LCD_command(1); /* clear display */
                LCD_command(0x80); /* set cursor at first line */
                delayMs(500);
                LCD_data(' ');
                LCD_data(' ');
                LCD_data(' ');
                LCD_data(' ');
                LCD_data(' ');
                LCD_data(' ');
                LCD_data('D');
                LCD_data('o');
                LCD_data('n');
                LCD_data('e');
                delayMs(1000);
            }
            else if(f > wished_temp + 2 && over == 0){
                vRead();
                if (over == 1) {
                    break;
                }
                while(swch.read()<0.5){
                        holdOn();
                    }
                giroDer();
                while(f > wished_temp + 2 && over == 0){
                    vRead();
                    if (over == 1) {
                        break;
                    }
                    while(swch.read()<0.5){
                        holdOn();
                    }
                    LCD_command(1); /* clear display */
                    LCD_command(0x80); /* set cursor at first line */
                    delayMs(500);
                    LCD_data(' ');
                    LCD_data(' ');
                    LCD_data(' ');
                    LCD_data(' ');
                    LCD_data('C');
                    LCD_data('o');
                    LCD_data('o');
                    LCD_data('l');
                    LCD_data('i');
                    LCD_data('n');
                    LCD_data('g');
                    delayMs(1000);
                    vRead();
                    if (over == 1) {
                        break;
                    }
                    while(swch.read()<0.5){
                        holdOn();
                    }
                    f = (5.0 * (TempSensor.read() * 690) * 100.0)/1024.0; //Read Temperature
                    LCD_displayTemp(f, 1000);
                    vRead();
                    if (over == 1) {
                        break;
                    }
                    while(swch.read()<0.5){
                        holdOn();
                    }
                    temporal = ain*100; //Variable temporal para la lectura de la temperatura deseada
                    ain = pot.read(); /* Read analog value (output will be any value between 0 and 1 */
                    wished_temp = (ain*100);
                    LCD_displayWish(wished_temp, 1000);
                    vRead();
                    if (over == 1) {
                        break;
                    }
                    while(swch.read()<0.5){
                        holdOn();
                    }
                }
                giroStop();
                LCD_command(1); /* clear display */
                LCD_command(0x80); /* set cursor at first line */
                delayMs(500);
                LCD_data(' ');
                LCD_data(' ');
                LCD_data(' ');
                LCD_data(' ');
                LCD_data(' ');
                LCD_data(' ');
                LCD_data('D');
                LCD_data('o');
                LCD_data('n');
                LCD_data('e');
                delayMs(1000);
            }
        }

    }
}

void holdOn(){
    giroStop();
    LCD_command(1); /* clear display */
    LCD_command(0x80); /* set cursor at first line */
    delayMs(500);
    LCD_data('O');
    LCD_data('n');
    LCD_data(' ');
    LCD_data('h');
    LCD_data('o');
    LCD_data('l');
    LCD_data('d');
    wait_us(500000);
}


/* initialize and configurates the LCD on the FRDM board */
void LCD_init(void){
    SIM->SCGC5 |= 0x1000; /* enable clock to Port D */
        
    PORTD->PCR[0] = 0x100; /* make PTD0 pin as GPIO */
    PORTD->PCR[1] = 0x100; /* make PTD1 pin as GPIO */
    PORTD->PCR[2] = 0x100; /* make PTD2 pin as GPIO */
    PORTD->PCR[3] = 0x100; /* make PTD3 pin as GPIO */
    PORTD->PCR[4] = 0x100; /* make PTD4 pin as GPIO */
    PORTD->PCR[5] = 0x100; /* make PTD5 pin as GPIO */
    PORTD->PCR[6] = 0x100; /* make PTD6 pin as GPIO */
    PORTD->PCR[7] = 0x100; /* make PTD7 pin as GPIO */
        
    PTD->PDDR = 0xFF; /* make PTD7-0 as output pins */
        
    SIM->SCGC5 |= 0x0200; /* enable clock to Port A */
    PORTA->PCR[2] = 0x100; /* make PTA2 pin as GPIO */
    PORTA->PCR[4] = 0x100; /* make PTA4 pin as GPIO */
    PORTA->PCR[5] = 0x100; /* make PTA5 pin as GPIO */
    PTA->PDDR |= 0x34; /* make PTA5, 4, 2 as out pins*/
    delayMs(30); 

    /* initialization sequence */
    LCD_command(0x38);
    delayMs(1);
    LCD_command(0x01);/* Clear screen */ 

    LCD_command(0x38);/* Set 8-bit data, 2-line, 5x7 font */
    LCD_command(0x06);/* Move cursor right */
    LCD_command(0x01);/* Clear screen, move cursor to home */
    LCD_command(0x0F);/* Turn on display, cursor blinking */
}

/* Sends command to the LCD*/
void LCD_command(unsigned char command){
    PTA->PCOR = RS | RW; /* RS = 0, R/W = 0 */
    PTD->PDOR = command;
    PTA->PSOR = EN; /* pulse E */

    delayMs(0);
    PTA->PCOR = EN; 

    if (command < 4)
    delayMs(4); /* command 1 and 2 needs up to 1.64ms */
    else
    delayMs(1); /* all others 40 us */
}

/* Sends data to the LCD*/
void LCD_data(unsigned char data){

    PTA->PSOR = RS; /* RS = 1, R/W = 0 */

    PTA->PCOR = RW;

    PTD->PDOR = data;

    PTA->PSOR = EN; /* pulse E */

    delayMs(0);
    PTA->PCOR = EN;
    delayMs(1);
}

/* Delay n milliseconds */
/* The CPU core clock is set to MCGFLLCLK at */ /*41.94 MHz in SystemInit(). */
void delayMs(int n) {
    int i;
    SysTick->LOAD = 41940 - 1;
    SysTick->CTRL = 0x5; /* Enable the timer and choose sysclk as the clock source */

    for(i = 0; i < n; i++) {
        while((SysTick->CTRL & 0x10000) == 0)
        /* wait until the COUTN flag is set */
        { }
    }
    SysTick->CTRL = 0; 
    /* Stop the timer (Enable = 0) */
}

void LCD_set(float n){
    string floatString = to_string(n);
    char temp[floatString.length() + 1]; 
	strcpy(temp, floatString.c_str()); 
    for (int i = 0; i<floatString.length(); i++){
        LCD_data((unsigned char)temp[i]);
    }
    delayMs(1000);
}

void LCD_displayTemp(float n, int ms){
    LCD_command(1); /* clear display */
    LCD_command(0x80); /* set cursor at first line */
    delayMs(500);
    LCD_data('C');
    LCD_data('u');
    LCD_data('r');
    LCD_data('r');
    LCD_data('e');
    LCD_data('n');
    LCD_data('t');
    LCD_data(' ');
    LCD_data('T');
    LCD_data('e');
    LCD_data('m');
    LCD_data('p');
    LCD_data('.');
    LCD_data(' ');
    LCD_command(0xC0);
    LCD_data(' ');
    LCD_data(' ');
    LCD_set(n);
    delayMs(ms);
}

void LCD_displayWish(float n, int ms){
    LCD_command(1); /* clear display */
    LCD_command(0x80); /* set cursor at first line */
    delayMs(500);
    LCD_data('W');
    LCD_data('i');
    LCD_data('s');
    LCD_data('h');
    LCD_data('e');
    LCD_data('d');
    LCD_data(' ');
    LCD_data('T');
    LCD_data('e');
    LCD_data('m');
    LCD_data('p');
    LCD_data('.');
    LCD_data(' ');
    LCD_command(0xC0);
    LCD_data(' ');
    LCD_data(' ');
    LCD_set(n);
    delayMs(ms);
}

void giroIzq(){ //Calienta Adentro de la planta
    IN1 = 0;
    IN2 = 1;
    return;
}

void giroDer(){ //Enfria Adentro de la planta
    IN1 = 1;
    IN2 = 0;
    return;
}

void giroStop(){
    IN1 = 0;
    IN2 = 0;
    return;
}

void vRead(){
    over = 0;
    if((int)(Fz.read()*100) == 67){
        value = 11;
    }
    else{
        value = (11*Fz.read())/0.677;
    }//(int)(Fz.read()*100);
    if(value>12.50){
        giroStop();
        over = 1;
    }
    return;
}
