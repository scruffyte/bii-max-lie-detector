/** LIE DETECTOR FINAL RELEASE CANDIDATE *****************************

BY MAX JACOBY & BII TARTAGLIA, 2022

WHEN USED WITH THE CORRESPONDING CIRCUIT, THIS CODE TAKES MEASUREMENTS OF 
HEAR RATE AND SKIN RESISTANCE, AND GIVES AN ESTIMATION OF WHETHER THE USER
IS LYING BASED ON THE RESULTS.

** C O N F I G U R A T I O N   B I T S ******************************/

#pragma config FOSC = INTIO67, FCMEN = OFF, IESO = OFF                      // CONFIG1H
#pragma config PWRT = OFF, BOREN = OFF, BORV = 30                           // CONFIG2L
#pragma config WDTEN = OFF, WDTPS = 32768                                   // CONFIG2H
#pragma config MCLRE = ON, LPT1OSC = OFF, PBADEN = ON, CCP2MX = PORTC       // CONFIG3H
#pragma config STVREN = ON, LVP = OFF, XINST = OFF                          // CONFIG4L
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF                   // CONFIG5L
#pragma config CPB = OFF, CPD = OFF                                         // CONFIG5H
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF               // CONFIG6L
#pragma config WRTB = OFF, WRTC = OFF, WRTD = OFF                           // CONFIG6H
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF           // CONFIG7L
#pragma config EBTRB = OFF                                                  // CONFIG7H

/** I N C L U D E S **************************************************/

#include "p18f45k20.h"
#include "ADC.h"
#include <delays.h>

/** D E F I N E S ****************************************************/

#define CLEAR_SCREEN  	0b00000001
#define FOUR_BIT  		0b00101100
#define LINES_5X7  		0b00111000
#define CURSOR_BLINK  	0b00001111
#define CURSOR_RIGHT  	0b00000110

#define DATA_PORT  LATD
#define RS_PIN     PORTDbits.RD6
#define E_PIN      PORTDbits.RD7

/** V A R I A B L E S ***********************************************/

#pragma udata                       // declare statically allocated uinitialized variables

//--- for displaying 3 digit int on the LCD ---//
char dispUnits;                     // 4th digit only 
char dispTenths;                    // 3rd digit only
char dispHundredths;                // 2nd digit only
char dispThousandths;               // 1st digit only
char dispTHT;                       // 1st, 2nd & 3rd digits
char dispTH;                        // 1st and 2nd digits

int pulseCount = 0000;              // int increaced by highISR when a pulse is detected on RB0
float baselineTimerElapsed;         // time the baseline button was pushed for, set by timerSeconds
int baselinePulseTotal;             // count of pulses while the baseline button was pushed
float baselineBPM = 0000;           // user BPM calculated from baselinePulseTotal/baselineTimerElapsed
float readingTimerElapsed;          // time the reading button was pushed for, set by timerSeconds
int readingPulseTotal;              // count of pulses while the reading button was pushed
float readingBPM;                   // user BPM calculated from readingPulseTotal/readingTimerElapsed
int timerCount = 0000;              // int increaded by highISR when timer rolls over
float lieScore = 0000;              // GSR+BPM for the reading
float lieBaseline = 0000;           // GSR+BPM for the baseline
int resultLoop;                     // counter for result animations 
float baselineGSR = 0;              // ADC value for baseline GSR
float readingGSR = 0;               // ADC value for reading GSR
int ADCvalue = 0000;                // 10 bit ADC value from adc_convert
int flashLoop;                      // counter for flashing RGB LEDs on result
unsigned int GSRavrSum = 0;         // sum of all ADC conversions 
int while1toggle = 0;               // toggled by low ISRs to break while(1) pause and start over
int ADC_count = 0;

/** I N T E R R U P T S *********************************************/

//--- high priority interrupt vector ---//

#pragma code InterruptVectorHigh = 0x08
void InterruptVectorHigh (void)
{
  _asm
    goto InterruptServiceHigh               //jump to interrupt routine
  _endasm
}

//--- Low priority interrupt vector ---//

#pragma code InterruptVectorLow = 0x18
void InterruptVectorLow (void)
{
  _asm
    goto InterruptServiceLow                //jump to interrupt routine
  _endasm
}

/** F U N C T I O N S ***********************************************/

//--- LCD ---//

void Delay5milli(void)					    //Suitable delay for LCD
{
    Delay1KTCYx(2);           						
}

void SetAddr(unsigned char DDaddr)
{
    DATA_PORT &= 0xf0;                      // Write upper nibble
    DATA_PORT |= (((DDaddr | 0b10000000)>>4) & 0x0f);

    RS_PIN = 0;                             // Set control bit
    Delay5milli();
    E_PIN = 1;                              // Clock the cmd and address in
    Delay5milli();
    E_PIN = 0;

    DATA_PORT &= 0xf0;                      // Write lower nibble
    DATA_PORT |= (DDaddr&0x0f);

    Delay5milli();
    E_PIN = 1;                              // Clock the cmd and address in
    Delay5milli();
    E_PIN = 0;
}

void WriteCmd(unsigned char cmd)
{
    DATA_PORT &= 0xf0;
    DATA_PORT |= (cmd>>4)&0x0f;           
    RS_PIN = 0;                     		// Set control signals for command
    Delay5milli();
    E_PIN = 1;                      		// Clock command in
    Delay5milli();
    E_PIN = 0;

    DATA_PORT &= 0xf0;              		// Lower nibble interface
    DATA_PORT |= cmd&0x0f;
    Delay5milli();
    E_PIN = 1;                      		// Clock command in
    Delay5milli();
    E_PIN = 0;
}

void WriteChar(char data)
{
    DATA_PORT &= 0xf0;
    DATA_PORT |= ((data>>4)&0x0f);

    RS_PIN = 1;                     		// Set control bits
    Delay5milli();
    E_PIN = 1;                      		// Clock nibble into LCD
    Delay5milli();
    E_PIN = 0;

    DATA_PORT &= 0xf0;              		// Lower nibble interface
    DATA_PORT |= (data&0x0f);

    Delay5milli();
    E_PIN = 1;                      		// Clock nibble into LCD
    Delay5milli();
    E_PIN = 0;
}

void WriteString(const rom char *buffer)    
{		 
    while(*buffer)                  		// Write data to LCD up to null
    {
        Delay5milli();
        WriteChar( *buffer);          		// Write character to LCD
        buffer++;                     		// Increment buffer
    }
    return;
}     

//--- ADC ---//

void ADC_Init(void)
{ 	
    ANSELbits.ANS0 = 1;	    // set RA0 to analog
    ADCON1 = 0;             // Sets bits VCFG1 and VCFG0 in ADCON1 so the ADC voltage reference is VSS to VDD
    ADCON2 = 0b10111000;    // right justify and set aquisition time
    ADCON0 = 0b00000001;    // Select channel 0 (AN0) to read the AN0 voltage and turn on ADC
}

void ADC_Convert(void)
{
    ADCON0bits.GO_DONE = 1;             // start conversion
    while (ADCON0bits.GO_DONE == 1);    // wait for it to complete
    ADCvalue = ADRESL + (ADRESH * 256); // store 10bit result
}

//--- TIMER ---//

void Timer1_Init(void)
{
    // Init Timer
    TMR1H = 00001011;              // preset timer count to 3035 (at 4:1 prescale = 0.5s rollover)
    TMR1L = 11011011;              // ^^
    INTCONbits.PEIE = 1;           // peripheral interrupt enable
    PIR1bits.TMR1IF = 0;           // clear interrupt flag
    PIE1bits.TMR1IE = 1;           // enable overflow interrupt
    T1CONbits.T1CKPS = 0b00000001; // 1:4 prescaler
    T1CONbits.TMR1ON = 1;          // start timer
}

void programFlame(void)
{
    WriteCmd  ( 0b00001011);       // display off
    SetAddr	 ( 0x00 );
    WriteChar ( 0x00 );
    WriteChar ( 0x01 );
    WriteChar ( 0x02 );
    WriteChar ( 0x03 );
    WriteChar ( 0x04 );
    WriteChar ( 0x05 );
    WriteChar ( 0x06 );
    WriteChar ( 0x07 );
    WriteCmd  ( 0x40 );            // writing to display
    // first character
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00000001 );
    WriteChar ( 0b00000001 );
    WriteChar ( 0b00000010 );
    // second character
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00001000 );
    WriteChar ( 0b00011000 );
    WriteChar ( 0b00001000 );
    WriteChar ( 0b00001000 );
    WriteChar ( 0b00001000 );
    WriteChar ( 0b00000100 );
    // third character
    WriteChar ( 0b00000010 );
    WriteChar ( 0b00000100 );
    WriteChar ( 0b00000100 );
    WriteChar ( 0b00010100 );
    WriteChar ( 0b00001001 );
    WriteChar ( 0b00001001 );
    WriteChar ( 0b00000101 );
    WriteChar ( 0b00000011 );
    // fourth character
    WriteChar ( 0b00000100 );
    WriteChar ( 0b00000010 );
    WriteChar ( 0b00001001 );
    WriteChar ( 0b00011010 );
    WriteChar ( 0b00001010 );
    WriteChar ( 0b00001010 );
    WriteChar ( 0b00001100 );
    WriteChar ( 0b00010000 );
    // fifth character
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00000010 );
    WriteChar ( 0b00000011 );
    WriteChar ( 0b00000010 );
    WriteChar ( 0b00000010 );
    WriteChar ( 0b00000110 );
    WriteChar ( 0b00000100 );
    // sixth character
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00000000 );
    WriteChar ( 0b00010000 );
    WriteChar ( 0b00010000 );
    WriteChar ( 0b00001000 );
    // seventh character
    WriteChar ( 0b00000100 );
    WriteChar ( 0b00001000 );
    WriteChar ( 0b00010010 );
    WriteChar ( 0b00001011 );
    WriteChar ( 0b00001010 );
    WriteChar ( 0b00001010 );
    WriteChar ( 0b00000110 );
    WriteChar ( 0b00000001 );
    // eigth character
    WriteChar ( 0b00001000 );
    WriteChar ( 0b00000100 );
    WriteChar ( 0b00000100 );
    WriteChar ( 0b00000101 );
    WriteChar ( 0b00010010 );
    WriteChar ( 0b00010010 );
    WriteChar ( 0b00010100 );
    WriteChar ( 0b00011000 );
    WriteCmd  ( 0b00001100);                    //display back on
}

void lyingMessage(void)
{
    SetAddr (0x80); 							// flame animation first frame
    WriteChar ( 0x00 );
    WriteChar ( 0x01 );
    WriteString("LIAR! LIAR!");

    SetAddr (0xC0);                             // moves character to begining of second line
    WriteChar ( 0x02 );
    WriteChar ( 0x03 );
    WriteString("Pants on fire");

    Delay10KTCYx(10); 							// keeps message on screen 

    SetAddr (0x80); 							// flame animation second frame
    WriteChar ( 0x04 );
    WriteChar ( 0x05 );

    SetAddr (0xC0);                             // moves character to begining of second line
    WriteChar ( 0x06 );
    WriteChar ( 0x07 );

    Delay10KTCYx(10); 							// keeps message on screen 
}

void truthMessage(void)
{
    SetAddr (0x80); 				            //truth animation first frame
    WriteChar ( 0x2A );
    WriteChar ( 0xEB );
    WriteChar ( 0x2A );
    WriteString(" it's the ");
    WriteChar ( 0x2A );
    WriteChar ( 0xEB );
    WriteChar ( 0x2A );
    SetAddr (0xC0);                             // moves character to begining of second line
    WriteString("     TRUTH!     ");
    Delay10KTCYx(10); 							// keeps message on screen 
    WriteCmd  ( CLEAR_SCREEN); 					// clears screen
}

void init_io(void)
{
    ANSEL  = 0;	                        	    // set i/o to digital
    ANSELH = 0;                                 // set rest of i/o to digital
    TRISD  = 0;                 		        // sets port D to output
    LATD   = 0b00000000;                       	// turns off port D outputs so display is blank
    TRISA = 0; 					            	//sets PORTA as all outputs for RGB LEDs
    LATA = 0b11111111; 			        		//turns off PORTA outputs 
    TRISAbits.TRISA0 = 1;		                // RA0 = input for pot ADC
    TRISBbits.TRISB0 = 1;		                // RB0 = input for pulse sensor
    TRISBbits.TRISB1 = 1;                       // RB1 = input for baseline button
    TRISBbits.TRISB2 = 1;                       // RB2 = input for reading button
    TRISCbits.TRISC3 = 0;                       // RC3 clock pulse verification
    TRISCbits.TRISC2 = 0;                       // RC2 pulse detection verification
    LATCbits.LATC3 = 0;                         // turn off RC3 initially
    LATCbits.LATC2 = 0;                         // initial set to off
}

void init_interrupts(void)
{
    RCONbits.IPEN = 1;                          // Enable priority levels on interrupts
    INTCONbits.GIEH = 1;                        // high priority interrupt enable
    INTCONbits.GIEL = 1;                        // Low priority interrupt enable
    INTCON2bits.INTEDG0 = 0;                    // interrupt on falling edge of INT0 (pulled high) (pulse sensor)
    INTCON2bits.INTEDG1 = 0;                    // interrupt on rising edge of INT1 (pulled low) (baseline button)
    INTCON2bits.INTEDG2 = 0;                    // interrupt on rising edge of INT2 (pulled low) (reading button)
    INTCONbits.INT0IF = 0;                      // clears INT0 flag so it doesn't trigger immediately
    INTCON3bits.INT1IF = 0;                     // clears INT1 flag
    INTCON3bits.INT2IF = 0;                     // clears INT2 flag
    INTCON3bits.INT1IP = 0;                     // INT1 low prioroty
    INTCON3bits.INT2IP = 0;                     // INT2 low prioroty
                                                // NOTE: INT0 is ALWAYS a high priority interrupt so no need to set
    INTCONbits.INT0IE = 1;                      // enable INT0 external interrupt
    INTCON3bits.INT1IE = 1;                     // enable INT1 external interrupt
    INTCON3bits.INT2IE = 1;                     // enable INT2 external interrupt
                                                // NOTE2: additional interrupts are initilized in timer function
}

void init_lcd(void)
{
    WriteCmd ( 0x02 );							// sets 4bit operation
    WriteCmd ( CLEAR_SCREEN );                  // clears the screen
    WriteCmd ( FOUR_BIT & LINES_5X7 );			// sets 5x7 and multiline operation.
    programFlame();                             // write the custom flame characters to the LCD
    WriteCmd ( CURSOR_BLINK );					// blinks cursor
    WriteCmd ( CURSOR_RIGHT  );					// moves cursor right	
    WriteString("Hello!");                      // test the lcd and show the program is running
    Delay1KTCYx(200);                           // wait a second
    WriteCmd ( CLEAR_SCREEN );                  // clears the screen
}

void reset_button_interrupts(void)
{
    INTCON3bits.INT1IF = 0;                     // clears INT1 flag
    INTCON3bits.INT2IF = 0;                     // clears INT2 flag
    INTCON3bits.INT1IE = 1;                     // enable INT1 external interrupt
    INTCON3bits.INT2IE = 1;                     // enable INT2 external interrupt
}

void extract_4digits(int number)
{
                                                // extract 4 digits from int input for LCD (e.g. 1234)
    dispUnits = number%10;                      // extract units (xxx4)
    dispTHT = number/10;                        // extract THT (123x)
    dispTenths = dispTHT%10;                    // extract tenths (xx3x)
    dispTH = dispTHT/10;                        // extract TH (12xx)
    dispHundredths = dispTH%10;                 // extract hundredths (x2xx)
    dispThousandths = dispTH/10;                // extract thousandths (1xxx)
}

void write_4digits(void)
{
    WriteChar(0x30 + dispThousandths);          // write digits from extract_4digits
    WriteChar(0x30 + dispHundredths);
    WriteChar(0x30 + dispTenths);
    WriteChar(0x30 + dispUnits);
}
/** MAIN ********************************************************/

#pragma code    // declare executable instructions

void main(void)
{
    init_io();                                  // init io 
    init_lcd();                                 // init lcd
    Timer1_Init();                              // Run timer init function and starts the timer
    ADC_Init();                                 // init ADC
    start:
    init_interrupts();                          // init interrupts 
    WriteCmd( CLEAR_SCREEN );
    //--- check if a baseline has been taken and promp the user accordingly ---//
    if(baselineBPM == 0)
    {
        WriteString("take a baseline");
    }
    else
    {
        WriteString("ready to read");
    }
    while(while1toggle == 0);                    // while1toggle is toggled at the end the ISRs to break here
    while1toggle = 0;                            // reset toggle after ISRs
    goto start;
} // main

/** INTERRUPT SERVICE ROUTINES **************************************/

#pragma interrupt InterruptServiceHigh           // "interrupt" pragma also for high priority
void InterruptServiceHigh(void)
{
    // Check for INT0 interrupt
    if (INTCONbits.INT0IF)
    {
        INTCONbits.INT0IF = 0;                   // clear (reset) flag
        pulseCount++;                            // increment pulse count by 1
        LATAbits.LATA3 = ~LATAbits.LATA3;        // flip blue RGB for pulse detector 
    }

    // check for TM1IF interrupt
    if  (PIR1bits.TMR1IF)
    {
        PIR1bits.TMR1IF = 0;                     // clear (reset) flag
        timerCount++;                            // increments timer count by 1
        TMR1H = 00001011;                        // set timer to rollover every 0.5s for better resolution
        TMR1L = 11011011;                        // ^^
        T1CONbits.T1CKPS = 0b00000001;           // set 1:4 prescale because I read that it resets on write to TMR1L/H
    }
}

void InterruptServiceLow(void)
{
    if(INTCON3bits.INT1IF)
    {
        WriteCmd( CLEAR_SCREEN );
        WriteString("baseline...");
        pulseCount = 0000;                                                       // clear pulse counter
        timerCount = 0000;                                                       // clear timer
        while(1)                                                                 // wait until baseline button is released
        {
            if(timerCount % 4 == 0)                                              // if timer is a multiple of 4 (every 2 seconds)
            {
                ADC_Convert();                                                   // take an ADC reading
                GSRavrSum = (GSRavrSum + ADCvalue);                              // add that reading to a total so an average can be taken
                ADC_count++;
            }
            if(PORTBbits.RB1 == 1)                                               // if baseline button is relased calculate all the things
            {
                baselineTimerElapsed = (timerCount/2);                           // store the timer value as seconds elapsed
                baselinePulseTotal = pulseCount;                                 // store the pulse count 
                baselineGSR = (GSRavrSum/ADC_count);                             // calculates average GSR
                GSRavrSum = 0;
                ADC_count = 0;
                baselineBPM = ((baselinePulseTotal / baselineTimerElapsed)*60);  // find a BPM value using that
                lieBaseline = (baselineBPM + baselineGSR);                       // find combo value from GSR and BPM
                WriteCmd( CLEAR_SCREEN );
                WriteString("BPM=");
                extract_4digits(baselineBPM);
                write_4digits(); 
                SetAddr (0xC0);                                                  // go to second line
                WriteString("GSR=");
                extract_4digits(ADCvalue);
                write_4digits(); 
                Delay1KTCYx(1000);
                while1toggle = 1;                                                // to break wait in main
                reset_button_interrupts();
                return;                                                          // return to the start
            }
        }       
    }
    if(INTCON3bits.INT2IF)                                                       // if reading button is pushed
    {
        WriteCmd( CLEAR_SCREEN );
        WriteString("reading...");
        pulseCount = 0000;                                                       // clear pulse counter
        timerCount = 0000;                                                       // clear timer
        while(1)                                                                 // wait until reading button is released
        {
            if(timerCount % 4 == 0)                                              // if timer is a multiple of 4 (every 2 seconds)
            {
                ADC_Convert();                                                   // take an ADC reading
                GSRavrSum = (GSRavrSum + ADCvalue);                              // add that reading to a total so an average can be taken
            }
            if(PORTBbits.RB2 == 1)                                               // if reading button is relased calculate all the things
            {
                readingTimerElapsed = (timerCount/2);                            // store the timer value as seconds elapsed
                readingPulseTotal = pulseCount;                                  // store the pulse count 
                readingGSR = (GSRavrSum/(timerCount/2));                         // calculates average GSR
                GSRavrSum = 0;                                                   // clear sum value for next measurement
                readingBPM = ((readingPulseTotal / readingTimerElapsed)*60);     // find a BPM value using that
                lieScore = (readingBPM + readingGSR);                            // find the overall lying value
                WriteCmd( CLEAR_SCREEN );
                WriteString("BPM=");
                extract_4digits(readingBPM);
                write_4digits(); 
                SetAddr (0xC0);                                                  // go to second line
                WriteString("GSR=");
                extract_4digits(ADCvalue);
                write_4digits();
                Delay1KTCYx(1000);
                if( lieScore > lieBaseline )
                {
                    for(resultLoop=0; resultLoop <= 3; resultLoop++)             // loop the flame animation 5 times
                    {
                        for(flashLoop=0; flashLoop <= 4; flashLoop++)
                        {
                            LATAbits.LATA1 = 0;
                            Delay1KTCYx(10);
                            LATAbits.LATA1 = 1;
                            Delay1KTCYx(10);
                        }
                        lyingMessage();                                              // call the flame animation
                    }
                    while1toggle = 1;                                                // to break wait in 
                    reset_button_interrupts();
                    return;                                                          // return to the start
                }
                if( lieScore <= lieBaseline )
                {
                    for(resultLoop=0; resultLoop <= 2; resultLoop++)
                    {
                        for(flashLoop=0; flashLoop <= 4; flashLoop++)
                        {
                            LATAbits.LATA2 = 0;
                            Delay1KTCYx(10);
                            LATAbits.LATA2 = 1;
                            Delay1KTCYx(10);
                        }
                        truthMessage();
                    }
                    while1toggle = 1;                                                // to break wait in main
                    reset_button_interrupts();
                    return;                                                          // return to the start
                }
            }
        } 
    }
}
