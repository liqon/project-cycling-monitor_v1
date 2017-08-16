/*

    project-cycling-monitor_v1
    Copyright (C) 2017  liqon

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/atomic.h>

/*  DEFINITIONS  */
#define BYTE unsigned char
#define PWM_SOURCE      10
#define SENSOR_READ     8
#define TIMER0_PINA     6
#define DISP_DATAPIN    11
#define DISP_CLK        12
#define DISP_DIG3       3
#define DISP_DIG4       4

//Buffers
static uint8_t MOV_circ_buffer1[8];
static uint8_t MOV_circ_buffer1_i;
static uint8_t DD_buffer1;
static uint8_t EDGE_buffer1[3];

//Flags
static BYTE Flags = 0;
#define T0_F   0 //Timer0 flag
#define SC_F   1 //Seconds counter
//Counters
static uint16_t SecondsTicks; //FULL = 40 000 = 5s
static uint8_t wheelRevs;
//Statistics
static uint8_t max_speed;
static uint8_t avg_speed;
static uint16_t time1;
static uint16_t ODO;
static uint16_t dist1;
static uint32_t dist_speed_ms; //aboves derived from this
//Display
static uint8_t digit1;
static uint8_t digit10;
//Settings
uint16_t perimeter_mm = 2045;

ISR(TIMER0_COMPA_vect)
{
    Flags |= _BV(T0_F);
}
void setupTimer0()
{
    //N = 4, max count = 125
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        PRR &= ~_BV(PRTIM0); //Disable Power Reduction for TC0
        TCCR0A = 0;          // clear control register A
        TCCR0B = 0;          // clear control register B
        OCR0A = 124;                       //f_OC0A = F_CLK / (2 * 8 * (1 + 124)); F_CLK = 16MEG; F_0C0A ~= 8 kHz
        TCCR0A = _BV(WGM01) | _BV(COM0A0); //CTC, OCR0A
        TCCR0B = _BV(CS00);                //8 prescaler
        TIFR0 &= ~(_BV(OCF0B) | _BV(TOV0)); 
        TIFR0 |= _BV(OCF0A); 
        TIMSK0 &= ~(_BV(OCIE0B) | _BV(TOIE0));
        TIMSK0 |= _BV(OCIE0A);
    }
}

void setPwmDuty(BYTE duty_select)
{
    switch (duty_select)
    {
    case 0:
        analogWrite(PWM_SOURCE, 13);
        break;
    case 1:
        analogWrite(PWM_SOURCE, 127);
        break;
    case 2:
        analogWrite(PWM_SOURCE, 205);
        break;
    default:
        analogWrite(PWM_SOURCE, 13);
        break;
    }
}

void TimeInterrupt1() //~8kHz
{
    //Get new sample
    int acq_sample = DigitalRead(SENSOR_READ);
    uint8_t new_sample = 0;
    int8_t y_dd = 0;
    if (acq_sample == HIGH)
        new_sample = 1;
    if (acq_sample == LOW)
        new_sample = 0;
    //Digital Filtering
    MOV_FIR1(new_sample, new_sample);
    //Digital Differentiator
    Differentiator1_Ord1(new_sample, y_dd); 
    DigitalDiode(y_dd, new_sample); //allows signal higher >1
    //Rising Edge Detection
    bool edge1 = false;
    RisingEdgeDetect(new_sample, 0, edge1);
    if (edge1 == true)
        wheelRevs++;
    //Seconds Counters
    SecondsTicks++;
    if (SecondsTicks == 39900)//40000) = 5s
    {
        Flags |= _BV(SC_F);
    }
}
void InitBuffers(uint8_t init_value)
{
    int k = 0;
    for (; k < 8; k++)
    {
        MOV_circ_buffer1[k] = init_value;
    }
    MOV_circ_buffer1_i = 0;
    for (k = 0; k < 8; k++)
    {
        MOV_circ_buffer1[k] = init_value;
    }
}
static inline void MOV_FIR1(uint8_t new_sample, uint8_t *ret)
{
    //y[n] = sum ( x[n] + ... x[n-7] )
    //log1 = 1
    //log0 = 0
    int k = 0, l;
    uint8_t res = 0;
#define N 8
    //ones(1,8)/8
    MOV_circ_buffer1[MOV_circ_buffer1_i] = new_sample;
    MOV_circ_buffer1_i = (MOV_circ_buffer1_i + 1) % 8;
    l = MOV_circ_buffer1_i;
    for (k = 0; k < N; k++)
    {
        res += MOV_circ_buffer1[l];
        l = (MOV_circ_buffer1_i + 1);
    }
    *ret = res;
#undef N
}
static inline void Differentiator1_Ord1(uint8_t new_sample, int8_t *ret) //Digital Differentiator 1st ord. Gain -2
{
    //y[n] = - 2 * (x[n] - x[n-1])
    *ret = DD_buffer1 - new_sample;
    ret =<< 1;
    DD_buffer1 = new_sample;
}
static inline void DigitalDiode(int8_t input, uint8_t *ret)
{
    *ret = (input > 1) ? ((uint8_t)input : 0);
}
static inline void RisingEdgeDetect(uint8_t input, int link_num, bool *ret)
{
    /*
    0 - detector to counter
    1 - second etalon to counter reset
    2 - CLK to second etalon
    */
    if (EDGE_buffer1[link_num] == FALSE && input == TRUE)
        *ret = true;
    else *ret = false;
}
void DistoSpeed_ms(uint32_t rps_5s)
{
    //Update every 5s
    //Perimeter in mm
    //Return s0,w32,f1 in mm*s^1
    dist_speed_ms = ((uint32_t)perimeter_mm * rps_5s) << 1;
}
#ifdef SPEED_MS
uint8_t ActualSpeed_ms()
{
    //Return s0,w8,f1
    uint8_t ret = 0;
    uint16_t m_res = dist_speed_ms >> 5; //Divide by 5000=32*156.25~=32*156=2^5 * 156
    ret = m_res / 0x9c;// /156; opt. ret |= 0; adc rz,0
    return ret;
}
#endif

#define SPEED_KMH
#ifdef SPEED_KMH
uint8_t ActualSpeed_kmh()
{
    uint8_t ret = 0, digits = 0;
    uint16_t m_res = dist_speed_ms >> 5; //Divide by 1388,8=32*156.25~=32*156=2^5 * 156
    ret = m_res / 0x2b;// /156; opt. ret |= 0; adc rz,0
    digits = ret >> 1;
    digit10 = ConvertToDigit(digits / 10);
    digit1 = ConvertToDigit(kmh % 10);
    return ret;
}
#endif
void ConvertToDigit(uint8_t input, bool dp = false, uint8_t *output)
{   
    //HIGH = LED off
    //LOW = LED on
    switch(input)
    {                           //ABCDEFG.dp
        case 0: *output = 0x3;  //00000011    
            break;
        case 1: *output = 0x9f; //10011111   
            break;
        case 2: *output = 0x25; //00100101
            break;
        case 3: *output = 0x0d; //00001101   
            break;
        case 4: *output = 0x99; //10011001    
            break;
        case 5: *output = 0x49; //01001001    
            break;
        case 6: *output = 0x41; //01000001 
            break;
        case 7: *output = 0x1f; //00011111  
            break;
        case 8: *output = 0x1;  //00000001    
            break;
        case 9: *output = 0x9; //00001001    
            break;
        default: *output = 0xff;
    }
    if (dp == true)
        *output &= 0xfe;
}
static inline void UpdateDisplay()
{
    //dig 4 - dec. units
    //dig 3 - decades

    //dig 4
    digitalWrite(DISP_DIG4, HIGH);
    digitalWrite(DISP_DIG3, LOW);
    shiftOut(DISP_DATAPIN, DISP_CLK, LSBFIRST, digit1);
    digitalWrite(DISP_DIG4, LOW);
    digitalWrite(DISP_DIG3, HIGH);
    shiftOut(DISP_DATAPIN, DISP_CLK, LSBFIRST, digit10);
    //delay(10);
    
}
static inline void TurnOffDisp()
{
    digitalWrite(DISP_DIG4, LOW);
    digitalWrite(DISP_DIG3, LOW);
}
static inline void Distance1_m()
{
    uint32_t m_res = dist_speed_ms >> 2;
    dist1 += m_res / 0xfa;
}

void setup()
{
    // put your setup code here, to run once:
    pinMode(PWM_SOURCE, OUTPUT);
    pinMode(SENSOR_READ, INPUT);
    pinMode(DISP_CLK, OUTPUT);
    pinMode(DISP_DATAPIN, OUTPUT);
    pinMode(DISP_DIG3, OUTPUT);
    pinMode(DISP_DIG4, OUTPUT);
    InitBuffers(0);
    setPwmDuty(0);
    setupTimer0();
    //Init' static variables
    SecondsTicks = 0; //FULL = 40 000 = 5s
    wheelRevs = 0;
//Statistics
    max_speed = 0;
    avg_speed = 0;
    time1 = 0;
    ODO = 0;
    dist1 = 0;
    dist_speed_ms = 0; //aboves derived from this
//Display
    digit1 = 0;
    digit10 = 0;
}
void loop()
{
    //Don't use delay()
    UpdateDisplay();
    if ((Flags & _BV(T0_F)) == TRUE)
    {
        TimeInterrupt1();
        //unset timer interrupt flag
        Flags &= ~_BV(T0_F);
    }
    TurnOffDisp();
    if ((Flag & _BV(SC_F)) == TRUE)
    {
        DistoSpeed_ms(wheelRevs);
        ActualSpeed_km();
        Distance1_m();
        time1 += 5;
        Flags &= ~_BV(SC_F);
    }
}
