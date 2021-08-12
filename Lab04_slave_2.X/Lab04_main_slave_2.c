/*
 * Archivo:   Lab04_main_slave_2.c
 * Dispositivo: PIC16F887
 * Autor: Margareth Vela 
 * 
 * Programa: I2C
 * Hardware: Push buttons en PORTB
 * 
 * Creado: Agosto 09, 2021
 * Última modificación: Agosto 12, 2021
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
        __delay_ms(10);
    }
    return;
}

//------------------------------------------------------------------------------
//                          Interrupciones
//------------------------------------------------------------------------------
void __interrupt()isr(void){
    di();                   //PUSH
     if(INTCONbits.RBIF){
        if (PORTBbits.RB0 == 0){ //Si el botón de incremento está presionado,
            PORTA++; //se incrementa PORTA
        }
        if(PORTBbits.RB1 == 0) {//Si el botón de decremento está presionado,
            PORTA--; //se decrementa PORTA
        }

        INTCONbits.RBIF = 0; //Se limpia la bandera
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
            SSPBUF = PORTA;//Escribe el valor de la variable al buffer
            SSPCONbits.CKP = 1;//Habilita los pulsos del reloj SCL
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
       
        PIR1bits.SSPIF = 0;    
    }
    
    ei();                                           //POP
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
    
    TRISA = 0xF0;               //Para contador de 4 bits
    TRISB = 0X03;               //Para push buttons
    
    PORTA = 0x00; //Se limpian los puertos
    PORTB = 0x03;
    
    //Habilitar pullups
    OPTION_REGbits.nRBPU = 0;
    WPUB = 0x03;
    
    //Interrupcion PORTB
    INTCONbits.RBIE = 1; //Enable Interrupt on change
    IOCB = 0x03;
    INTCONbits.RBIF = 0; //Se limpia la bandera de Interrupt on change	
    
    //Configurar interrupciones
    INTCONbits.GIE = 1;  //Enable interrupciones globales
    INTCONbits.PEIE = 1; //Enable interrupciones perifericas
    INTCONbits.RBIE = 1; //Enable interrupt on change
    INTCONbits.RBIF = 0; //Se limpia la bandera IOC
    I2C_Slave_Init(0x80); //Direccion del esclavo
}