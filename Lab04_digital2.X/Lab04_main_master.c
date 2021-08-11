/*
 * Archivo:   Lab03_main_master.c
 * Dispositivo: PIC16F887
 * Autor: Margareth Vela 
 * 
 * Programa: I2C
 * Hardware: 
 * 
 * Creado: Agosto 09, 2021
 * Última modificación: Agosto 11, 2021
 */

//------------------------------------------------------------------------------
//                          Importación de librerías
//------------------------------------------------------------------------------
#include <xc.h>
#include <stdint.h>
#include "I2C.h"

//------------------------------------------------------------------------------
//                          Directivas del compilador
//------------------------------------------------------------------------------
#define _XTAL_FREQ 8000000 //Para delay

//------------------------------------------------------------------------------
//                          Palabras de configuración
//------------------------------------------------------------------------------
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT//Oscillator Selection bits(INTOSCIO 
                              //oscillator: I/O function on RA6/OSC2/CLKOUT pin, 
                              //I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT disabled and 
                          //can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR  
                                //pin function is digital input, MCLR internally 
                                //tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
                                //protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
                                //protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit 
                                //Internal/External Switchover mode is disabled
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit 
                                //(Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF         //Low Voltage Programming Enable bit(RB3/PGM pin 
                                //has PGM function, low voltage programming 
                                //enabled)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out 
                                //Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
                                //(Write protection off)

//------------------------------------------------------------------------------
//                          Variables
//------------------------------------------------------------------------------
uint8_t VALOR_ADC = 0;
uint8_t z;

//------------------------------------------------------------------------------
//                          Prototipos
//------------------------------------------------------------------------------
void setup(void);  //Configuración

//------------------------------------------------------------------------------
//                          Código Principal
//------------------------------------------------------------------------------
void main(void) {
    setup();   
    while(1){
        
        I2C_Master_Start();         //INICIALIZAMOS LA COMUNICACION
        I2C_Master_Write(0x71);     //ESCRIBIMOS A LA DIRECCION PARA LEER
        PORTD = I2C_Master_Read(0); //AGREGAMOS EL VALOR AL PORTD
        I2C_Master_Stop();          //DETENEMOS LA COMUNICACION
        __delay_ms(200);
    }
    return;
}

//------------------------------------------------------------------------------
//                          Interrupciones
//------------------------------------------------------------------------------
void __interrupt()isr(void){
    di();
    
    ei();                           //POP
}

//------------------------------------------------------------------------------
//                          Configuración
//------------------------------------------------------------------------------
void setup(void){
    //Configuracion reloj
    OSCCONbits.IRCF2 = 1; //Frecuencia a 8MHZ
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1;
    
    //Configurar entradas y salidas
    ANSELH = 0x00;//Pines digitales
    ANSEL = 0x00; //Pines digitales
    
    TRISA = 0x00; //Salidas
    TRISB = 0x00;
    TRISC = 0x00; 
    TRISD = 0x00; 
    TRISE = 0x00; 
    
    PORTA = 0x00; //Se limpian los puertos
    PORTB = 0x00;
    PORTC = 0x00;    
    PORTD = 0x00;
    PORTE = 0x00;
    
    //Configurar la interrupcion
    INTCONbits.GIE = 0;  //Enable interrupciones globales
    INTCONbits.PEIE = 0;           
    I2C_Master_Init(100000);        // INICIALIZAR MASTER A FRECUENCIA DE 100kHz
}