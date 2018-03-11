/******************************
  Soldering-Iron Controller V1.0

  Firmware used to control a soldering iron.
  Designed to be used with Arduino Lilypad, Atmega328P 

  Created: 2018-02-18
  Author: Håkan Gerner
*******************************/

#include <U8g2lib.h>
#include <U8x8lib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <EEPROM.h>


/*----------CONFIG----------*/
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, 10, 9, 8);

#define VCC 5                         //VCC voltage
#define R1_RESISTANCE 100000          
#define R2_RESISTANCE 270

/*--------------------------*/

#define STOP_TIMER1 TCCR1B &= ~(1 << CS11)      // Prescaler 1/8
#define START_TIMER1 TCCR1B |= (1 << CS11)

#define START_TIMER2 TCCR2B |= _BV(CS22)
#define STOP_TIMER2 TCCR2B &= ~_BV(CS22)      // Prescaler 1/8

/*--------GPIO PORTS--------*/
#define GPIO_ZEROCROSSING PD2   //D2
#define GPIO_HEAT         PD3   //D3
#define GPIO_STAND        PC3   //
/*--------------------------*/

/*-------ADC CHANNELS-------*/
#define ADC_CHANNELS_NUM 3
volatile uint16_t rawIronADC;          //ADC0
volatile uint16_t rawPotADC;           //ADC1
volatile uint16_t rawAmbientADC;       //ADC2

volatile uint16_t *ADC_CHANNELS[] = {&rawIronADC, &rawPotADC, &rawAmbientADC};
/*--------------------------*/

/*-------IDLE CONTROL-------*/
bool ironIsIdle = false;
bool ironInStand = false;
uint16_t ironIdleTimeoutInSec = 300;
uint16_t ironIdleTempInC = 180;
/*--------------------------*/

/*-------TEMP----------*/
uint16_t latestIronTemp = 0;
int16_t zeroCrossing = 0;
double Kp = 80.0;
char KpCompensation = 0;
bool tempIsStable = false;


float runningAverage(float M) {
  const uint8_t LENGTH = 10;
  static int values[LENGTH];
  static uint8_t index = 0, count = 0;
  static float sum = 0;

  sum -= values[index];
  values[index] = M;
  sum += values[index];
  index++;
  index = index % LENGTH;
  
  if (count < LENGTH) 
    count++;
  
  return sum / count;
}

float median(float M) {
    const uint8_t LENGTH = 5;
    static uint8_t index = 0;
    static int values[LENGTH];
    
    values[index] = M;
    index++;
    index = index % LENGTH;
    
    float temp;
    int i, j;
    // the following two loops sort the array x in ascending order
    for(i = 0; i < LENGTH - 1; i++) {
        for(j = i + 1; j < LENGTH; j++) {
            if(values[j] < values[i]) {
                // swap elements
                temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }

    if(LENGTH%2==0) {
        // if there is an even number of elements, return mean of the two elements in the middle
        return((values[LENGTH/2] + values[LENGTH/2 - 1]) / 2.0);
    } else {
        // else return the element in the middle
        return values[LENGTH/2];
    }
}

uint16_t getAmbientTempToC()
{
  uint16_t tempC = (1023 - rawAmbientADC) * 0.143 - 49;
  return tempC;
}

uint16_t getIronTempToC()
{
  int tempC = round((median(((rawIronADC * (VCC / 1023.0)) / (1 + R1_RESISTANCE / R2_RESISTANCE)) * 37800 + getAmbientTempToC())));
  return tempC;
}

uint16_t getTargetTempToC(uint16_t maxTemp = 400)
{
 uint16_t tempC = 0;
 const char tempStep = 5;
 
 if (ironIsIdle)
    tempC = ironIdleTempInC;
  else
    tempC =  map(rawPotADC, 0, 1023, 0, floor(maxTemp / tempStep)) * tempStep;

  return tempC;
}

void updateScreen(char* desc, int16_t temp)
{
  char buf[10]; 
  snprintf(buf, 10, "%d%c%c", temp, '°', 'C');
  
  
  u8g2.clearBuffer();                         // Clear the internal memory
  u8g2.enableUTF8Print();
  u8g2.setFont(u8g2_font_mercutio_sc_nbp_tf); // Choose a suitable font
  
  u8g2.drawStr((128 - u8g2.getStrWidth(desc)) / 2, 10, desc);


  
  u8g2.setFont(u8g2_font_logisoso38_tf);     // Choose a suitable font
  u8g2.drawStr((128 - u8g2.getStrWidth(buf))/2, 60, buf);
  
  u8g2.sendBuffer();                          // Transfer internal memory to the display
}

/*void updateScreenIs(int16_t tipTemp, int16_t targetTemp)
{
  char buf[2][5];
  snprintf (buf[0], 5, "%d", tipTemp);
  snprintf (buf[1], 5, "%d", targetTemp);
  
  u8g2.clearBuffer();                         // Clear the internal memory
  u8g2.setFont(u8g2_font_mercutio_sc_nbp_tf); // Choose a suitable font
  
  //u8g2.drawStr((128 - u8g2.getStrWidth("Iron temp"))/2, 10, "Iron temp");           // Write something to the internal memory
  u8g2.drawStr(128 - u8g2.getStrWidth("Set"), 10, "Set");
  u8g2.drawStr(0, 10, "Iron temp");

  
  u8g2.setFont(u8g2_font_logisoso28_tr   );     // Choose a suitable font
  u8g2.drawStr(0, 60, buf[0]);
  u8g2.drawStr(128 - u8g2.getStrWidth(buf[1]), 60, buf[1]);
  
  u8g2.sendBuffer();                          // Transfer internal memory to the display
}
*/

void timer1_Start(){
  //want to start/ restart the timer here
  TCNT1 = 0;                // Clear the timer
  TCCR1B |= _BV(CS11) | _BV(CS10);    // prescaler=64 16 MHz clock
  TCCR1B |= (1 << WGM12);
}

void timer2_Start(){
  //want to start/ restart the timer here
  TCNT2 = 0;                // Clear the timer
  TCCR2B |= _BV(CS22);    // prescaler=64 16 MHz clock
  TCCR2A |= (1 << WGM21);
}

void TIMER1_Init()
{ 
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1);
  TCCR1B |= (1 << WGM12);
  OCR1A = 15350;              // COMPA at 20/16ths microseconds
  TCNT1 = 0;                // Clear the timer
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt channel A:
}

void TIMER2_Init()
{ 
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= (1 << COM2A1);
  TCCR2A |= (1 << WGM21);
  OCR2A = 150;               // 0.4ms (halv of 0.8ms zero-crossing time)
  TCNT2 = 0;                // Clear the timer
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt channel A:  
}

void ADC_Init(void)
{
                                             //Prescaler at 128 so we have an 125Khz clock source
  ADCSRA |= ((0 << ADPS2) | (0 << ADPS1) | (1 << ADPS0)); 
  ADMUX  |= (1 << REFS0);
  ADMUX  |= (0 << REFS1);
  ADCSRA |= (1 << ADIE);
  ADCSRA |= (1 << ADATE);                    //Signal source, in this case is the free-running
  ADCSRA |= (1 << ADEN);                     //Power up the ADC
  ADCSRA |= (1 << ADSC);                     //Start converting
}

/* Zero-crossing interrupt pin init */
void INT_Init(void)
{
  EICRA |= (0 << ISC01) | (1 << ISC00);
  EIMSK |= (1 << INT0);
  EIFR  |= (1 << INTF0);

  PCICR |= (1 << PCIE1);
  PCIFR |= (1 << PCIF1);
  PCMSK1 |= (1 << PCINT11);
}



ISR(ADC_vect) // ADC Interrupt enable
{
  static uint8_t isrDelay = 0;
  static uint8_t ADC_channel = 0;
  
  if (isrDelay < 1)                         //Interrupt delay to let ADC stabilize
  {
    ++isrDelay;
  }
  else
  {
    *ADC_CHANNELS[ADC_channel] = ADCW;
    if (++ADC_channel >= ADC_CHANNELS_NUM)
      ADC_channel = 0;

    ADMUX = (ADMUX &  0xF0) | ADC_channel;  //Select ADC Channel
    ADCSRA |= (1 << ADSC);                  // Start A2D Conversions
    isrDelay = 0;
  }
}




/* Zero crossing interrupt */
ISR(INT0_vect) //Zero crossing interrupt, On change
{
  //Serial.println(latestIronTemp);
  if (PIND & (1 << GPIO_ZEROCROSSING))
  {
    if (zeroCrossing <= 0) {
      PORTD &= ~(1 << GPIO_HEAT);
      START_TIMER1;
    }
    else {
      if (latestIronTemp < getTargetTempToC())
      {
        PORTD |= (1 << GPIO_HEAT);
        //START_TIMER2;   
      } 
    }
    zeroCrossing--;
  }
}


/* Zero crossing timer */
ISR(TIMER2_COMPA_vect)
{
  PORTD |= (1 << GPIO_HEAT);
  STOP_TIMER2;
}






/* Temp-measurement timer */
ISR(TIMER1_COMPA_vect)
{
  latestIronTemp = getIronTempToC();
  double nextMeasurement = ((double(getTargetTempToC() + KpCompensation) - double(latestIronTemp))/(double(getTargetTempToC()) + double(latestIronTemp))) * Kp;
  zeroCrossing = floor(nextMeasurement);

  STOP_TIMER1;
}


/* Iron stand change interrupt */
ISR(PCINT1_vect) //Iron stand sensor interrupt, On change
{
  if (PINC & (0 << GPIO_STAND))
  {
    ironInStand = false;
  }
  else
  {
    ironInStand = true;
  }
}



  /*IDLE TIMER*/
  /*
  static uint16_t idleCount = 0;
  if (!ironIsIdle && ironInStand)
  {
    idleCount++;
  
    if (idleCount >= ironIdleTimeoutInSec)
    {
      ironIsIdle = true;
      idleCount = 0;
    }
  }
*/

void setup() {
  cli();
  ADC_Init();
  INT_Init();

  TIMER1_Init();
  TIMER2_Init();
  
  DDRD |= (1 << GPIO_HEAT);
  
  Serial.begin(115200);
  u8g2.begin();

  for (int i = 0; i < 5; i++)
    getIronTempToC();
  
  sei();

}



void loop() {
  static unsigned long screenSetCooldown = 0;
  static uint16_t lastTipTemp = 0, lastTargetTemp = 0;
  uint16_t tipTemp = latestIronTemp, targetTemp = getTargetTempToC();
  static bool updateSet, lastUpdateSet = false;

  if (targetTemp != lastTargetTemp)
  {
    screenSetCooldown = millis();
  }
  
  if (millis() - screenSetCooldown >= 1000) {
    updateSet = false;
  }
  else {
    updateSet = true;
  }
  
  (abs(tipTemp - targetTemp) < 5) ? tempIsStable = true : tempIsStable = false;
  
  tipTemp = ((tipTemp+5)/5) * 5;
  
  if (abs(tipTemp - lastTipTemp) > 0 || abs(targetTemp - lastTargetTemp > 0) || updateSet != lastUpdateSet)
  {
    noInterrupts();
    if (updateSet)
    {
      updateScreen("Set", targetTemp);
      lastUpdateSet = updateSet;
    }
    else
    {
      updateScreen("Iron Temp", tipTemp);
    }
    
    lastTipTemp = tipTemp;
    lastTargetTemp = targetTemp;
    interrupts();
  }
}
