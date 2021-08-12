/*
 * Archivo:   Lab04_main_slave.c
 * Dispositivo: PIC16F887
 * Autor: Margareth Vela 
 * 
 * Programa: I2C
 * Hardware: Potenciómetro en PORTA
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
uint8_t POT; //Para ADC
uint8_t z; //Para I2C

//------------------------------------------------------------------------------
//                          Prototipos
//------------------------------------------------------------------------------
void setup(void);  //Configuración

//------------------------------------------------------------------------------
//                          Código Principal
//------------------------------------------------------------------------------
void main(void) {
    setup(); 
    ADCON0bits.GO = 1; //Inicia conversion ADC
    while(1){

    }
    return;
}

//------------------------------------------------------------------------------
//                          Interrupciones
//------------------------------------------------------------------------------
void __interrupt()isr(void){
    di();                   //PUSH
     if (ADIF == 1){                            
        POT = ADRESH;
        ADIF = 0; //Limpiar la bandera de ADC
        __delay_us(60);
        ADCON0bits.GO = 1; //Inicia la conversión de ADC
    }
       if(PIR1bits.SSPIF == 1){ 

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
            z = SSPBUF;    //Lee el valor del buffer y lo agrega a la variable
            SSPCONbits.SSPOV = 0;       //Se limpia la bandera de overflow
            SSPCONbits.WCOL = 0;        //Se limpia el bit de colision
            SSPCONbits.CKP = 1;         //Se habilita SCL
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) {
            z = SSPBUF;     //Lee el valor del buffer y lo agrega a la variable
            PIR1bits.SSPIF = 0;         //Limpia la bandera de SSP
            SSPCONbits.CKP = 1;         //Habilita los pulsos del reloj SCL
            while(!SSPSTATbits.BF);     //Hasta que la recepcion se realice
            __delay_us(250);
            
        }else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            z = SSPBUF; //Lee el valor del buffer y lo agrega a la variabl
            BF = 0;
            SSPBUF = POT;//Escribe el valor de la variable al buffer
            SSPCONbits.CKP = 1;//Habilita los pulsos del reloj SCL
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
       
        PIR1bits.SSPIF = 0;    
    }
    
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
    ANSEL = 0x01; //Primer pin analogico
    
    TRISA = 0x01; //Para POT
    TRISB = 0x00;
    TRISC = 0x08; 
    TRISD = 0x00; 
    TRISE = 0x00; 
    
    PORTA = 0x00; //Se limpian los puertos
    PORTB = 0x00;
    PORTC = 0x00;    
    PORTD = 0x00;
    PORTE = 0x00;
    
    //Configurar ADC
    ADCON1bits.ADFM = 0; //Justificar a la izquierda
    ADCON1bits.VCFG0 = 0; //Vss
    ADCON1bits.VCFG1 = 0; //VDD

    ADCON0bits.ADCS = 0b10; //ADC oscilador -> Fosc/32
    ADCON0bits.CHS = 0;     //Comenzar en canal 0       
    ADCON0bits.ADON = 1;    //Habilitar la conversión ADC
    __delay_us(50); 
    ADCON0bits.GO = 1;
    
    //Configurar la interrupcion
    INTCONbits.GIE = 1;  //Enable interrupciones globales
    INTCONbits.PEIE = 1;           
    I2C_Slave_Init(0x70); //Direccion del esclavo
}