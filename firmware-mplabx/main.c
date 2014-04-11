#include <xc.h>
#include <stdio.h>

#define _XTAL_FREQ 32000000     // Velocidade do oscilador (interno ou externo)

#pragma config FOSC = INTOSC    // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is digital input, MCLR internally tied to VDD)
#pragma config BOREN = OFF      // Brown-out Detect Enable bit (BOD disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

////////////////////////////////////////////////variaveis globais///////////////////////////////////////////////////////////
char caracter;
bit char_received = 0;
///////////////////////////////////////////////////interrupção//////////////////////////////////////////////////////////////

/* Trata as interrupções do PIC */
void interrupt RS232(void)
{
    /* Lê caracter da entrada serial */
    if (RCIF) {
        caracter = RCREG;
        char_received = 1;
        RCIF = 0; //  limpa flag de interrupção de recepção
    }
}
///////////////////////////////// Funçoes usadas pela uart //////////////////////////////////////////////////////

/* Envia um caracter via porta serial. */
void putch(char data) {
    while(!TXIF) // Aguarda liberar o buffer
        continue;
    TXREG = data;
}

/* Configura o clock do processador. */
void configure_Oscillator() {
    /* Ajusta o clock para 32MHz. */
    OSCCONbits.IRCF = 0b00001110; // Define o oscilador interno em 8MHz.
    OSCCONbits.SPLLEN = 0x01; // Liga o PPL 4x.
    OSCCONbits.SCS = 0x00; // Seleciona a fonte como o oscilador interno.
}

/* Configura as portas de entrada e saída. */
void configure_Ports() {
    ANSELB = 0; // I/O digital em todas as portas B.
    TRISB = 0b00100000; // Configura RB5(12) (pino - RX) como entrada, o restante como saída. Nota, o pino RB7(10) é o TX.

    OPTION_REGbits.nWPUEN = 1; // Weak pullups habilitados individualmente.
}

/*
 * Configura a porta serial.
 *
 * baud_rate: Baude rate desejado.
 * mode: (0) - Modo de baixa velocidade; (1) Modo de alta velocidade.
 *
 * Por padrão é usado o modo 8 bits e sem paridade, mas se necessário ajuste aqui a configuração desejada.
 * Verifique o datasheet para ver a porcentagem de erro e se a velocidade é possivel para o cristal utilizado.
 */
void configure_SerialPort(long baud_rate, int mode) {
    /* Desabilita as portas I2C e SPI */
    SSP1CON1bits.SSPEN = 0;
    SSP2CON1bits.SSPEN = 0;
    
    RCSTA = 0b10010000; // Habilita a porta serial, recepção de 8 bits em modo contínuo, assíncrono.
    int valor;          // Valor da configuração para o gerador de baud rate.
    
    if (mode == 1) {
        /* modo = 1, modo de alta velocidade. */
        TXSTA = 0b00100100;                                   // Modo assíncrono, transmissão de 8 bits.
        valor = (int) (((_XTAL_FREQ / baud_rate) - 16) / 16); // Cálculo do valor do gerador de baud rate.
    } else {
        /* modo = 0, modo de baixa velocidade. */
        TXSTA = 0b00100000;                                   // Modo assíncrono, transmissão de 8 bits.
        valor = (int) (((_XTAL_FREQ / baud_rate) - 64) / 64); // Cálculo do valor do gerador de baud rate.
    }
    SPBRG = valor;
    RCIE = 1; // Habilita interrupção de recepção.
    TXIE = 0; // Deixa interrupção de transmissão desligada, pois corre se o risco de ter uma interrupção escrita e leitura ao mesmo tempo.
}

/* Configura as interrupções. */
void configure_Interrupts() {
    PEIE = 1; // Habilita a interrupção de periféricos do PIC
    GIE = 1;  // Global Interrupt Enable bit
}
//////////////////////////////////////////////////////Rotina principal///////////////////////////////////////////////////////////////

void main(void) {

    /* Configurações iniciais. */
    configure_Oscillator();
    configure_Ports();
    configure_SerialPort(19200, 1); // Modo de alta velocidade
    configure_Interrupts();

    printf("Usando a serial MPlab XC8 \n\r");
    printf("Digite algo \n\r");
    for (;;) {
        if (char_received) { // Possui dado a ser lido
            printf(" \n\r caracter digitado :");
            putch(caracter);
            char_received = 0;
        }
    }
}