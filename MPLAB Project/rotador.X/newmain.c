// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define _XTAL_FREQ 10000000 // Frequência de clock 10 MHz
#define SLAVE_ADDRESS_LCD 0x4E //Endereço do drive do LCD

void I2C_init (uint32_t clock)
{
    SSPADD = (_XTAL_FREQ/(4*clock))-1;  // here clock is the BR/clock speed
    SSPCON = 0b00101000;     //first 4 bits I2c master mode , 6th bit is SSPEN enables the scl and sda line
    SSPSTAT = 0;
    TRISC3 = 1;
    TRISC4 = 1;
}

void I2C_wait ()
{
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));    // wait for start bit to clear in SSPSTAT and bits 0 to 4 in SSPCON2
}

void I2C_start (void)
{
    I2C_wait ();
    SSPCON2 |= 0x01; // SEN=1 -> initiate the START condition on SDA and SCL pins
}


/*void I2C_repeated_start (void)
{
    I2C_wait();
    SSPCON2 |= 0x02; // RSEN=1  -> initiate REPEATED START condition on SDA and SCL pins
}

*/
void I2C_stop (void)
{
    I2C_wait ();
    SSPCON2 |= 0x04; // PEN=1 -> initiate the STOP condition on SDA and SCL pins
}


void I2C_write (uint8_t data)
{
    I2C_wait ();
    SSPBUF = data;  // load data into SSPBUF register
}


void lcd_send_data (unsigned char data)
{
	unsigned char data_l, data_u;
	data_l = (data<<4)&0xf0;  //select lower nibble by moving it to the upper nibble position
	data_u = data&0xf0;  //select upper nibble

	I2C_start();
	I2C_write (SLAVE_ADDRESS_LCD);
	I2C_write (data_u|0x0D);  //enable=1 and rs =1
	I2C_write (data_u|0x09);  //enable=0 and rs =1

	I2C_write (data_l|0x0D);  //enable =1 and rs =1
	I2C_write (data_l|0x09);  //enable=0 and rs =1

	I2C_stop();
}

// send command to LCD

void lcd_send_cmd (unsigned char data)
{
	unsigned char data_l, data_u;
	data_l = (data<<4)&0xf0;  //select lower nibble by moving it to the upper nibble position
	data_u = data&0xf0;  //select upper nibble

	I2C_start();
	I2C_write (SLAVE_ADDRESS_LCD);
	I2C_write (data_u|0x0C);  //enable=1 and rs =0
	I2C_write (data_u|0x08);  //enable=0 and rs =0

	I2C_write (data_l|0x0C);  //enable =1 and rs =0
	I2C_write (data_l|0x08);  //enable=0 and rs =0

	I2C_stop();
}

void lcd_clear ()
{
    lcd_send_cmd (0x80); // Primeira linha
    for (int i=0;i<60;i++) lcd_send_data (' ');

    lcd_send_cmd (0xc0); // Segunda linha
    for (int i=0;i<60;i++) lcd_send_data (' ');

}

// initialise LCD

void lcd_init (void)
{
	lcd_send_cmd (0x02);
	lcd_send_cmd (0x28);
	lcd_send_cmd (0x0c);
	lcd_send_cmd (0x80);
}

// send string to LCD

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void adc_init(){
    ADCON1 = 0x80; //0b10000000 //Justificado à direita, Vref+ e Vref- = Vdd e Vss
    PORTA = 0x00;
    TRISA0 = 1;
    ANSELbits.ANS0 = 1; // ANSEL 0x01 Entrada analogica em port A0/AN0
    ADCON0 = 0xc1; //Frc clock, Canal An0 e Modulo ADC ligado
}

unsigned char adc_do(){
    unsigned char w;
    __delay_ms(1);
    //ADCON0 = ADCON0 | 0x00000010 ; // Operador Or seta quando quero e mantem o valor se for igual a 0
    //Operador AND (&) Zera um bit se coloco 0 e mantem o bit se coloco 1
    // Para limpar um bit específico ou seja, colocar nível lógico 0, utiliza-se o operador & (AND).
    ADCON0bits.GO_nDONE = 1;
    while (ADCON0bits.GO_nDONE);
    w = ADRESL >> 2;
    w = (((ADRESH & 0b00000011) << 6 ) + w  ) ; // Operador or (|) seta quando quero e mantem o valor se for igual a 0)
    return w;
}


typedef struct {
    unsigned bPDT:1 ; //bit Pulso detectado
    unsigned bLWR:1 ; //bit Low Rotation detectado
    unsigned bHRD:1 ;// bit High Rotation Detected
    unsigned char bTAP; // bit Tempo Atual de (duração de) Pulso

    char value[3] ;
} RPMCON_REG ;

RPMCON_REG RPMCON;

void __interrupt() my_isr(void){
    RPMCON.bPDT = 1 ;

    INTCONbits.INTF = 0; // sai da interrupção
}

void medeRPM(void);

void main(void) {

    strcpy(RPMCON.value, "xxx");

    //RPMCON.bPDT = 0;
    //RPMCON.bAUX = 0;
    TRISC = 0x00;
    TRISB = 0xFF;
    ANSEL = 0x01;
    ANSELH = 0x00;
    PORTCbits.RC2 = 0;
    PORTCbits.RC5 = 0;
    OPTION_REGbits.T0CS = 0; //Habilita o modo Timer do tmr0
    OPTION_REGbits.PSA = 0 ; //Deixa o prescale pro timer e nao pro watchdog
    OPTION_REG = OPTION_REG | 0x07 ; // Configura prescaler do timer em 1:256
    INTCONbits.GIE = 1; // Habilita interrupções globais
    INTCONbits.INTE = 1; // Habilita interrupção externa
    OPTION_REGbits.INTEDG = 1; //Configura a detecção de borda pra detecção de subida

 // PWM initialise
    PR2 = 0xFF; //Periodo que reseta o timer sempre que os dois são iguais
    T2CON = 0x06; //Liga o timer e prescale em 1:16
    CCPR1L = 0x00; //Registrador responsavel pelo duty cicly do PWM, 8MSB
    CCP1CON = 0x0C; // Registrador que configura o PWM;

    // initialise I2C at 100 KHz
    I2C_init(100000);

    lcd_init ();
    adc_init();
    lcd_clear();
    lcd_send_cmd (0x80);
    lcd_send_string("Iniciando...");
    __delay_ms(500);

    while (1) /// MAIN LOOP do Código ================================================================================
    {


        CCPR1L = adc_do();
        medeRPM();

        if (RPMCON.bLWR){
        lcd_send_cmd (0x80);
        lcd_send_string("RPM ABAIXO de xx");

        }
        else {
            if ((RPMCON.bHRD)){
                lcd_send_cmd (0x80);
                lcd_send_string("High Rotation D.");

            }
            else {
                lcd_send_cmd (0x80);
                lcd_send_string("RPM: ");
                lcd_send_string(RPMCON.value);
                lcd_send_string("        ");
            }
        }


    }


    return;
}
void medeRPM(void){
    unsigned int aux = 0, bHiTAP = 0;
    unsigned short int unidade, dezena, centena;
    float tpr = 0.0;
    RPMCON.bLWR = 0;
    RPMCON.bPDT = 0;
    RPMCON.bHRD = 0;
    TMR0 = 0x00;
    INTCONbits.T0IF = 0;
    while(RPMCON.bPDT == 0){ // Espero o primeiro pulso para começar a medir ou indentificar RPM mt baixo;

        if (INTCONbits.T0IF == 1){
            TMR0 = 0x00;
            INTCONbits.T0IF = 0;
            bHiTAP += 1;
        }
        if (bHiTAP >= 37 ){ // 37 vezes o timer 0 estourou
            RPMCON.bLWR = 1;
            return;
        }

    }
    // Identificou o primeiro pulso
    RPMCON.bPDT = 0;
    TMR0 = 0x00;
    INTCONbits.T0IF = 0;
    bHiTAP = 0;
    while(RPMCON.bPDT == 0){ // Espero o segundo pulso para medir

        if (INTCONbits.T0IF == 1){

            TMR0 = 0x00;
            INTCONbits.T0IF = 0;
            bHiTAP += 1;
        }
        if (bHiTAP >= 37 ){ // 37 vezes o timer 0 estourou
            RPMCON.bLWR = 1;
            return;
        }
    }
    RPMCON.bTAP = TMR0;
    tpr = ((float)RPMCON.bTAP * (106.0 / 1000000.0)); // 106us é a unidade mínima medida pelo timer
    tpr += ((float)bHiTAP * (27136.0 / 1000000.0) );
    tpr = (24.0*tpr)  ;  // 24 é o numero de pulsos em uma rotação
    aux = 60.0 / tpr ; //calcula o tempo que levará para dar 60 rotações baseado no tempo levado pra dar 1 pulso;
    aux = aux*1.03;
    /*
    if ((aux - 999) > 1){
        RPMCON.bHRD = 1;
        return;
    }
    else {
    */
        centena = ( aux / 100 );
        dezena = (aux/10) - (centena*10);
        unidade = aux - (centena*100) - (dezena*10);

        RPMCON.value[0] = 0x30 + centena ;
        RPMCON.value[1] = 0x30 + dezena ;
        RPMCON.value[2] = 0x30 + unidade;



    //}

    return;
}
