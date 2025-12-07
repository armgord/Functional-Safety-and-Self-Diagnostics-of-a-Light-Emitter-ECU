// Reto ITESM OPMobility TC2009B AD25
/*Diseño usando microcontroladores y arquitectura computacional (Gpo 601)
 TC2009B.601
*/
/*Moises Arturo Flores Cayon
  Armando Gordillo Ramirez
  Juan Pablo Bastian Gutierrez*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <xc.h>
#include <stdint.h>
#include <pic18f45k50.h>
#include <math.h>

// =================== CONFIGURACION EEPROM ===================
#define ADDR_ERR_CORTOS     0x00
#define ADDR_ERR_HIGH_V     0x01
#define ADDR_ERR_LOW_V      0x02
#define ADDR_ERR_TEMP       0x03

// =================== CALIBRACION ===================
#define VDD_VOLTAGE         5000.0  
#define R_TOP_K             9.9     
#define R_BOTTOM_K          6.86    
#define V_CORRECTION        0.9  // Para ajustar discrepancias

// =================== LIMITES ===================
#define HISTERESIS          150     
#define TEMP_REDUCIR_PWM    30.0    
#define TEMP_APAGADO        35.0    

#define V_SHUTDOWN_HIGH     8000    // > 8.0V
#define V_WARN_HIGH_1       7000    
#define V_WARN_HIGH_2       6000    
#define V_SHUTDOWN_LOW      4000    // < 4.0V

#define PWM_MAX             1023    
#define PWM_DUTY_TEMP       600     
#define PWM_DUTY_7V         600     
#define PWM_DUTY_6V         800     

#define NTC_BETA            3950.0
#define NTC_R25             10900.0
#define NTC_T25             298.15
#define R_FIXED             10000.0
#define _XTAL_FREQ          16000000UL

// Mapping
#define Row1                LATAbits.LATA0
#define Row2                LATAbits.LATA1
#define Row3                LATAbits.LATA2
#define Row4                LATAbits.LATA3
#define TEMP_CHANNEL        5
#define VOLT_CHANNEL        6

const uint8_t NOT_KEY = 0xFF;

// =================== GLOBALES ===================
uint8_t estado_previo_cortos = 0xFF;
uint8_t nivel_voltaje_actual = 0; 

// =================== FUNCIONES ===================
void CLK_Initialize(void);
void UART_Initialize(void);
void putch(char data);
void inicializarpuertos(void);
void PWM_Init(void);
void PWM_SetDuty(uint16_t duty);
uint16_t ADC_Read(uint8_t channel);
float leer_temperatura_ntc(void);
uint16_t leer_voltaje_mv(void);

// Funciones EEPROM
void EEPROM_Write(uint8_t address, uint8_t data);
uint8_t EEPROM_Read(uint8_t address);
void registrar_error_eeprom(uint8_t direccion);
void menu_ver_eeprom(void);

// Funciones Cortos
uint8_t Ncortos(void);
void apagar_todo_seguridad(void);
void checar_cortos_rapido(void);    
void logica_proteccion_completa(void); 

// Funciones Monitoreo y Estado
void modo_monitoreo(void);
void reportar_estado_cortos(uint8_t estado_actual);
void reportar_shutdown_generico(const char* causa);

// Funciones Keypad y Menu
uint8_t getKey(void);
uint8_t esperar_tecla(void);
void keypad_init(void);
void menu_principal(void);
void menu_mostrar(void);
void menu_encender(void);
void menu_apagar(void);

// =================== MAIN ===================
int main(void) {
    CLK_Initialize();
    __delay_ms(50);
    UART_Initialize();
    inicializarpuertos();
    PWM_Init();
    
    printf("\r\n========================================\r\n");
    printf("   RETO TC2009B ITESM OPMobility \r\n");
    printf("========================================\r\n");
    
    // Estado inicial: ON
    Row1 = 1; Row2 = 1; Row3 = 1; Row4 = 1;
    PWM_SetDuty(PWM_MAX);
    
    menu_principal();
    return 0;
}

// =================== FUNCIONES EEPROM ===================
void EEPROM_Write(uint8_t address, uint8_t data) {
    EECON1bits.EEPGD = 0; EECON1bits.CFGS = 0; EEADR = address; EEDATA = data; EECON1bits.WREN = 1;
    INTCONbits.GIE = 0; EECON2 = 0x55; EECON2 = 0xAA; EECON1bits.WR = 1;
    while(EECON1bits.WR);
    INTCONbits.GIE = 1; EECON1bits.WREN = 0;
}

uint8_t EEPROM_Read(uint8_t address) {
    EECON1bits.EEPGD = 0; EECON1bits.CFGS = 0; EEADR = address; EECON1bits.RD = 1;
    return EEDATA;
}

void registrar_error_eeprom(uint8_t direccion) {
    uint8_t actual = EEPROM_Read(direccion);
    if (actual == 0xFF) actual = 0;
    EEPROM_Write(direccion, actual + 1);
}

void menu_ver_eeprom(void) {
    printf("\r\n=== HISTORIAL EEPROM ===\r\n");
    
    uint8_t err_cortos = EEPROM_Read(ADDR_ERR_CORTOS);
    uint8_t err_highV  = EEPROM_Read(ADDR_ERR_HIGH_V);
    uint8_t err_lowV   = EEPROM_Read(ADDR_ERR_LOW_V);
    uint8_t err_temp   = EEPROM_Read(ADDR_ERR_TEMP);
    
    if(err_cortos == 0xFF) err_cortos = 0;
    if(err_highV == 0xFF)  err_highV = 0;
    if(err_lowV == 0xFF)   err_lowV = 0;
    if(err_temp == 0xFF)   err_temp = 0;

    printf("1. Cortos acumulados:      %u\r\n", err_cortos);
    printf("2. Fallas Sobre-Voltaje:   %u\r\n", err_highV);
    printf("3. Fallas Bajo-Voltaje:    %u\r\n", err_lowV);
    printf("4. Fallas Temperatura:     %u\r\n", err_temp);
    
    printf("\r\n[*] Volver  |  [C] Borrar Historial\r\n");
    
    while(1) {
        // Mantener la seguridad activa en submenu
        uint8_t k = esperar_tecla(); 
        if (k == '*') break;
        if (k == 'C') {
            printf("\r\nBorrando memoria...\r\n");
            EEPROM_Write(ADDR_ERR_CORTOS, 0); EEPROM_Write(ADDR_ERR_HIGH_V, 0);
            EEPROM_Write(ADDR_ERR_LOW_V, 0);  EEPROM_Write(ADDR_ERR_TEMP, 0);
            printf("Memoria limpia.\r\n");
            break;
        }
    }
}

// =================== ADC ===================
uint16_t ADC_Read(uint8_t channel) {
    ADCON0 &= 0b10000011; ADCON0 |= (channel << 2); ADCON0bits.ADON = 1;
    __delay_us(50); ADCON0bits.GO = 1; while (ADCON0bits.GO);
    return ((ADRESH << 8) | ADRESL);
}

uint16_t leer_voltaje_mv(void) {
    uint32_t acumulado = 0;
    for(int i=0; i<8; i++) { acumulado += ADC_Read(VOLT_CHANNEL); __delay_us(10); }
    uint16_t adc = acumulado / 8;
    float Vpin_mv = ((float)adc * VDD_VOLTAGE) / 1023.0;
    float divisor_factor = (R_TOP_K + R_BOTTOM_K) / R_BOTTOM_K;
    return (uint16_t)(Vpin_mv * divisor_factor * V_CORRECTION);
}

float leer_temperatura_ntc(void) {
    uint16_t adc = ADC_Read(TEMP_CHANNEL);
    float Vadc = ((float)adc * (VDD_VOLTAGE/1000.0)) / 1023.0;
    if (Vadc <= 0.1) return -40.0; if (Vadc >= 4.9) return 150.0;
    float Rntc = (Vadc * R_FIXED) / ((VDD_VOLTAGE/1000.0) - Vadc);
    float tempK = 1.0 / ((1.0 / NTC_T25) + (1.0 / NTC_BETA) * log(Rntc / NTC_R25));
    return tempK - 273.15;
}

// =================== PROTECCION ===================

void checar_cortos_rapido(void) {
    if (Ncortos() >= 2) {
        apagar_todo_seguridad();
        printf("Falla detectada. Escribiendo EEPROM...\r\n");
        registrar_error_eeprom(ADDR_ERR_CORTOS); // GUARDA
        reportar_shutdown_generico("2+ CORTOCIRCUITOS");
        while(1) { LATAbits.LATA4 = !LATAbits.LATA4; __delay_ms(200); } 
    }
}

void logica_proteccion_completa(void) {
    
    float tempC = leer_temperatura_ntc();
    uint16_t volt_mV = leer_voltaje_mv();
    
    // 1. SHUTDOWNS (Guardan en EEPROM y matan el sistema)
    if (volt_mV >= V_SHUTDOWN_HIGH) {
        apagar_todo_seguridad();
        registrar_error_eeprom(ADDR_ERR_HIGH_V); // GUARDA
        reportar_shutdown_generico("SOBRE-VOLTAJE (>8V)");
        printf("Lectura final: %u mV\r\n", volt_mV);
        while(1) __delay_ms(500); 
    }
    
    if (volt_mV <= V_SHUTDOWN_LOW) {
        apagar_todo_seguridad();
        registrar_error_eeprom(ADDR_ERR_LOW_V); // GUARDA
        reportar_shutdown_generico("BAJO-VOLTAJE (<4V)");
        printf("Lectura final: %u mV\r\n", volt_mV);
        while(1) __delay_ms(500); 
    }

    if (tempC >= TEMP_APAGADO) {
        apagar_todo_seguridad();
        registrar_error_eeprom(ADDR_ERR_TEMP); // GUARDA
        reportar_shutdown_generico("SOBRE-TEMPERATURA (>35C)");
        while(1) __delay_ms(500); 
    }

    // 2. CONTROL PWM
    uint16_t pwm_objetivo = PWM_MAX;
    if (tempC >= TEMP_REDUCIR_PWM) {
        pwm_objetivo = PWM_DUTY_TEMP; 
    } else {
        if (volt_mV >= V_WARN_HIGH_1) {
            pwm_objetivo = PWM_DUTY_7V;
            if(nivel_voltaje_actual != 2) nivel_voltaje_actual = 2;
        } else if (volt_mV >= V_WARN_HIGH_2) {
            if (nivel_voltaje_actual == 2 && volt_mV > (V_WARN_HIGH_1 - HISTERESIS)) pwm_objetivo = PWM_DUTY_7V;
            else { pwm_objetivo = PWM_DUTY_6V; if(nivel_voltaje_actual != 1) nivel_voltaje_actual = 1; }
        } else {
            if (nivel_voltaje_actual == 1 && volt_mV > (V_WARN_HIGH_2 - HISTERESIS)) pwm_objetivo = PWM_DUTY_6V;
            else { pwm_objetivo = PWM_MAX; if(nivel_voltaje_actual != 0) nivel_voltaje_actual = 0; }
        }
    }
    PWM_SetDuty(pwm_objetivo);
}

// =================== MONITOREO ===================
void modo_monitoreo(void) {
    printf("\r\n[MODO MONITOREO ACTIVO]\r\n");
    printf("Presione '*' para salir.\r\n");
    estado_previo_cortos = 0xFF; 
    uint8_t loop_visual = 0;

    while (1) {
        checar_cortos_rapido();
        logica_proteccion_completa();

        // Salida
        if (getKey() == '*') { while(getKey() == '*'); return; }

        loop_visual++;
        if (loop_visual >= 5) {
            loop_visual = 0;
            float t = leer_temperatura_ntc();
            uint16_t v = leer_voltaje_mv();
            uint16_t duty_actual = (CCPR2L << 2) | CCP2CONbits.DC2B;
            printf("T:%.1f V:%.2f PWM:%u   \r", t, v/1000.0, duty_actual);
        }
        
        uint8_t estado_cortos = 0;
        if (!PORTAbits.RA4) estado_cortos |= 0x01;
        if (!PORTAbits.RA5) estado_cortos |= 0x02;
        if (!PORTAbits.RA6) estado_cortos |= 0x04;
        if (!PORTAbits.RA7) estado_cortos |= 0x08;
        
        if (estado_cortos != estado_previo_cortos) {
            printf("\r\n");
            reportar_estado_cortos(estado_cortos);
            estado_previo_cortos = estado_cortos;
        }
        
        if (!PORTAbits.RA4) Row1 = 0;
        if (!PORTAbits.RA5) Row2 = 0;
        if (!PORTAbits.RA6) Row3 = 0;
        if (!PORTAbits.RA7) Row4 = 0;

        __delay_ms(100); 
    }
}

// =================== MENU PRINCIPAL ===================

void menu_principal(void) {
    keypad_init();
    while(1) {
        menu_mostrar();
        uint8_t opcion = esperar_tecla(); // 
        
        switch(opcion) {
            case '1': modo_monitoreo(); break;
            case '2': menu_encender(); break;
            case '3': menu_apagar(); break;
            case '4': menu_ver_eeprom(); break;
            default: printf("Opcion invalida\r\n"); break;
        }
    }
}

void menu_mostrar(void) {
    printf("\r\n========== MENU ==========\r\n");
    printf("1) MODO MONITOREO\r\n");
    printf("2) Encender Lineas\r\n");
    printf("3) Apagar Lineas\r\n");
    printf("4) HISTORIAL FALLAS\r\n");
    printf("===============================\r\n");
    printf("Elija opcion: ");
}

uint8_t esperar_tecla(void) {
    uint8_t tecla = NOT_KEY;
    
    while(1) {
        //REVISION DE SEGURIDAD CONTINUA (No importa tiempo para presionar tecla, sistema sigue revisando cada ciclo)
        checar_cortos_rapido();
        logica_proteccion_completa(); 
        
        tecla = getKey();
        if (tecla != NOT_KEY) {
            __delay_ms(50); // Debounce
            if (getKey() == tecla) {
                while (getKey() != NOT_KEY); // Esperar soltar
                return tecla;
            }
        }
        __delay_ms(10); 
    }
}

// Keypad Menu & Drivers
void menu_encender(void) {
    printf("\r\nEncender (1-4): ");
    uint8_t k = esperar_tecla();
    if(k=='1') Row1=1; else if(k=='2') Row2=1; else if(k=='3') Row3=1; else if(k=='4') Row4=1;
}
void menu_apagar(void) {
    printf("\r\nApagar (1-4): ");
    uint8_t k = esperar_tecla();
    if(k=='1') Row1=0; else if(k=='2') Row2=0; else if(k=='3') Row3=0; else if(k=='4') Row4=0;
}

void apagar_todo_seguridad(void) { Row1=0; Row2=0; Row3=0; Row4=0; PWM_SetDuty(0); }
uint8_t Ncortos(void) {
    uint8_t c=0;
    if(!PORTAbits.RA4) c++; if(!PORTAbits.RA5) c++;
    if(!PORTAbits.RA6) c++; if(!PORTAbits.RA7) c++;
    return c;
}
void reportar_shutdown_generico(const char* causa) { printf("\r\n!!! SHUTDOWN: %s !!!\r\n", causa); }
void reportar_estado_cortos(uint8_t estado_actual) {
    if (estado_actual & 0x01) printf("[!] CORTO L1 ");
    if (estado_actual & 0x02) printf("[!] CORTO L2 ");
    if (estado_actual & 0x04) printf("[!] CORTO L3 ");
    if (estado_actual & 0x08) printf("[!] CORTO L4 ");
    printf("\r\n");
}

void CLK_Initialize(void) { OSCCONbits.IRCF = 0b111; OSCCONbits.SCS = 0b10; while(!OSCCONbits.HFIOFS); }
void inicializarpuertos(void) {
    ANSELA=0; TRISAbits.TRISA4=1; TRISAbits.TRISA5=1; TRISAbits.TRISA6=1; TRISAbits.TRISA7=1;
    TRISA &= 0xF0; 
    ANSELEbits.ANSE0=1; TRISEbits.TRISE0=1; 
    ANSELEbits.ANSE1=1; TRISEbits.TRISE1=1; 
    TRISCbits.TRISC6=0; TRISCbits.TRISC7=1; TRISCbits.TRISC1=0; 
    ADCON0=0; ADCON1=0; ADCON2=0b10101010; LATA &= 0xF0;
}
void PWM_Init(void) { T2CON=0; T2CONbits.T2CKPS=0b01; PR2=124; CCPR2L=0; CCP2CONbits.DC2B=0; CCP2CONbits.CCP2M=0b1100; TMR2=0; T2CONbits.TMR2ON=1; }
void PWM_SetDuty(uint16_t duty) { if(duty>1023) duty=1023; CCPR2L=(duty>>2); CCP2CONbits.DC2B=(duty&0x03); }
void UART_Initialize(void) { TXSTA1bits.TXEN=0; RCSTA1bits.SPEN=0; TXSTA1bits.BRGH=1; BAUDCON1bits.BRG16=1; SPBRGH1=0x01; SPBRG1=0xA0; TXSTA1bits.SYNC=0; RCSTA1bits.SPEN=1; TXSTA1bits.TXEN=1; RCSTA1bits.CREN=1; }
void putch(char data) { while (!TXSTA1bits.TRMT); TXREG1 = data; }

void keypad_init(void) { ANSELB=0; TRISB=0x0F; WPUB=0x0F; INTCON2bits.RBPU=0; }
uint8_t getKey(void) {
    LATB = 0b11100000; __delay_ms(1); if(!PORTBbits.RB0) return '1'; if(!PORTBbits.RB1) return '4'; if(!PORTBbits.RB2) return '7'; if(!PORTBbits.RB3) return '*';
    LATB = 0b11010000; __delay_ms(1); if(!PORTBbits.RB0) return '2'; if(!PORTBbits.RB1) return '5'; if(!PORTBbits.RB2) return '8'; if(!PORTBbits.RB3) return '0';
    LATB = 0b10110000; __delay_ms(1); if(!PORTBbits.RB0) return '3'; if(!PORTBbits.RB1) return '6'; if(!PORTBbits.RB2) return '9'; if(!PORTBbits.RB3) return '#';
    LATB = 0b01110000; __delay_ms(1); if(!PORTBbits.RB0) return 'A'; if(!PORTBbits.RB1) return 'B'; if(!PORTBbits.RB2) return 'C'; if(!PORTBbits.RB3) return 'D';
    return NOT_KEY;
}