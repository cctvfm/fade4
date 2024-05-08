#include <FlashAsEEPROM.h>
#include <FlashStorage.h>
#include <MIDI.h>
#include <ResponsiveAnalogRead.h>
#include <MIDIUSB.h>


#define MAXPOTVALUE 1015  //Change this to a lower value on V1 boards with certain power supplies (<5V)
//for example:
//#define MAXPOTVALUE 975

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();

MIDI_CREATE_DEFAULT_INSTANCE();

#define POT4 A0 //pcb pots are backwards on V1.0
#define POT3 A1
#define POT2 A2
#define POT1 A3

#define DIGIT1 11 //PA16  TCC0/WO[6] F
#define DIGIT2 10 //PA18  TCC0/WO[2] F
#define DIGIT3 13 //PA17  TCC0/WO[7] F
#define DIGIT4 12 //PA19  TCC0/WO[3] F

#define SW2 MOSI
#define SW1 SCK

#define SEG_A 2
#define SEG_B 3
#define SEG_C 4
#define SEG_D 5
#define SEG_E 6
#define SEG_F 7
#define SEG_G 8
#define SEG_P 9

#define NUMBER 0
#define LETTER 1
#define RAW 2
#define HEX 16

unsigned char segments[]={SEG_A,SEG_B,SEG_C,SEG_D,SEG_E,SEG_F,SEG_G,SEG_P};
unsigned char digits[]={DIGIT1, DIGIT2, DIGIT3, DIGIT4};

unsigned char LEDchars[4];
unsigned int LEDbrightness = 60;
unsigned int currentbrightness = 60;

unsigned long int lastLEDUpdate = 0;

#define SMOOTHNESS 0.004

ResponsiveAnalogRead analogPOTS[4] = {
    ResponsiveAnalogRead(POT1, true, SMOOTHNESS),
    ResponsiveAnalogRead(POT2, true, SMOOTHNESS),
    ResponsiveAnalogRead(POT3, true, SMOOTHNESS),
    ResponsiveAnalogRead(POT4, true, SMOOTHNESS)
};

FlashStorage(flash_valid, bool);
FlashStorage(saved_brightness,unsigned char);

FlashStorage(saved_message1,unsigned char);
FlashStorage(saved_message2,unsigned char);
FlashStorage(saved_message3,unsigned char);
FlashStorage(saved_message4,unsigned char);

FlashStorage(saved_channel1,unsigned char);
FlashStorage(saved_channel2,unsigned char);
FlashStorage(saved_channel3,unsigned char);
FlashStorage(saved_channel4,unsigned char);


unsigned int potvalues[4];
unsigned char ccvalues[4]={0,0,0,0};

bool saveDataFlag = 0;
unsigned long int saveDataTime = 0;

void setup() {
  // put your setup code here, to run once:



  pinMode(POT1, INPUT);
  pinMode(POT2, INPUT);
  pinMode(POT3, INPUT);
  pinMode(POT4, INPUT);

  randomSeed(analogRead(POT1)); //used for boot animation
  
  pinMode(DIGIT1, OUTPUT);
  pinMode(DIGIT2, INPUT);
  pinMode(DIGIT3, INPUT);
  pinMode(DIGIT4, INPUT);

  setupTCC0();



  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SEG_A, OUTPUT);
  pinMode(SEG_B, OUTPUT);
  pinMode(SEG_C, OUTPUT);
  pinMode(SEG_D, OUTPUT);
  pinMode(SEG_E, OUTPUT);
  pinMode(SEG_F, OUTPUT);
  pinMode(SEG_G, OUTPUT);
  pinMode(SEG_P, OUTPUT);



 
  analogPOTS[0].enableEdgeSnap();
  analogPOTS[1].enableEdgeSnap();
  analogPOTS[2].enableEdgeSnap();
  analogPOTS[3].enableEdgeSnap();
  

  
  //Serial.begin(9600);
  startTimer(400);  //start interrupt for LED characters
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOn();
  welcome();

  loadData();
}

#define FALSE 0
#define TRUE 1
unsigned char channels[4]= {1,1,1,1};
unsigned char messages[4] = {1,11,71,74};
unsigned char options[4] = {60,1,1,1};

#define NORMAL 0
#define SET_MESSAGE 1
#define SET_CHANNEL 2
#define SET_OPTIONS 3
bool fadeout = 0;
unsigned char state = 0;
void loop() {
  //THE MAIN LOOP HERE

  static unsigned long fadeouttime = 0;
  static unsigned long lastupdate = 0;


  if((millis()-lastupdate) > 20)
  {
    update_all();
    lastupdate = millis();
  }

  MIDI.read();

  if(fadeout == 1)  //if we're in fadeout mode (==1)
  {
    if(millis()-fadeouttime > 10)//and 10 ms has elapsed since the last time we did this
    {
      fadeouttime = millis(); //catch the new time of the fade loop
      
      if(currentbrightness == 0)  //if we're at 0
      {
        fadeout = 2;
      }
      else
        reducebrightness(); //reduce the brightness by x
    }
  }




  if((millis()-lastLEDUpdate) > 3000 && (fadeout == 0))
  {
    //if the LEDs have been on for more than 4 seconds, clear them
    if(state == NORMAL) // if we're in normal operation
    { 
      //clearLED();
      fadeout=1;
      fadeouttime=millis();
      currentbrightness = LEDbrightness;
    }

    else
    {
        //keep the displays on if we're in Channel/Message/Options mode

    }
  }
  
  if(saveDataFlag == 1)
  {
    if(millis()-saveDataTime > 10000)
    {
        //only write to eeprom every 10 seconds and only if something has changed
        saveData();
        saveDataFlag = 0;
    }
  }
  //delay(20);
    
}

void update_all (void)
{

  #define BUTTONHOLDTIME 1000
  #define DEBOUNCETIME 10

  unsigned char temp;
  unsigned char x;
  int y;
  static unsigned char update[4] = {FALSE,FALSE,FALSE,FALSE};
  static unsigned int buttonpressedtime[3];
  static bool buttonpressed[3];
  if((digitalRead(SW1) == 0) && (digitalRead(SW2)==1))  //Sw1 is low but sw2 is not
  {
      
      if(buttonpressed[0]==1 && state != SET_MESSAGE) //it was previously pressed
      {
          if((millis()-buttonpressedtime[0]) > BUTTONHOLDTIME)  //have we waited long enough?
          {
            //yes, set the state
            state = SET_MESSAGE;
            clearLED();  //we're in the set message mode
            LEDchars[0]=charactertoLED('E',LETTER,0);
            LEDchars[1]=charactertoLED('I',LETTER,0);
            LEDchars[2]=charactertoLED('D',LETTER,0);
            LEDchars[3]=charactertoLED('T',LETTER,1);
            delay(100);

            
          }

      }
      else if(buttonpressed[0] == 0)  //first time pressed
      {
        buttonpressedtime[0]= millis(); //save the time it was pressed
        buttonpressed[0]=1;
      }
  }

  else if ((digitalRead(SW1) == 1) && (digitalRead(SW2)==0))  //button two pressed
  {
      if(buttonpressed[1]==1 && state != SET_CHANNEL) //it was previously pressed
      {
          if((millis()-buttonpressedtime[1]) > BUTTONHOLDTIME)  //have we waited long enough?
          {
            //yes, set the state
            state = SET_CHANNEL;
            clearLED();  //we're in the set message mode
            LEDchars[0]=charactertoLED('C',LETTER,0);
            LEDchars[1]=charactertoLED('A',LETTER,0);
            LEDchars[2]=charactertoLED('H',LETTER,0);
            LEDchars[3]=charactertoLED('N',LETTER,1);
            delay(100);

            
          }

      }
      else if(buttonpressed[1] == 0)  //first time pressed
      {
        buttonpressedtime[1]= millis(); //save the time it was pressed
        buttonpressed[1]=1;
      }
  }
  else if ((digitalRead(SW1) == 0) && (digitalRead(SW2)==0))  //BOTH BUTTONS PRESSED
  {
    if(buttonpressed[2]==1 && state != SET_OPTIONS) //it was previously pressed
      {
          if((millis()-buttonpressedtime[2]) > BUTTONHOLDTIME)  //have we waited long enough? Double time for options menu
          {
            //yes, set the state
            state = SET_OPTIONS;
            clearLED();  //we're in the set message mode
            LEDchars[0]=charactertoLED('O',LETTER,0);
            LEDchars[1]=charactertoLED('T',LETTER,0);
            LEDchars[2]=charactertoLED('P',LETTER,0);
            LEDchars[3]=charactertoLED('N',LETTER,1);
            delay(200);

            
          }

      }
      else if(buttonpressed[2] == 0)  //first time pressed
      {
        buttonpressedtime[2]= millis(); //save the time it was pressed
        buttonpressed[2]=1;
      }
  }

  else if((digitalRead(SW1)==1) && (digitalRead(SW2)==1))  //neither button pressed
  {
    if(buttonpressed[0] == 1) //button was previously pressed, but now it isn't
    {
      if((millis() - buttonpressedtime[0]) > DEBOUNCETIME)
      {
        //only clear the flag if we're out of the debounce window
        buttonpressed[0]=0;
        if(state ==SET_MESSAGE)
        {
          state=NORMAL;
          clearLED(); 
          delay(100);
          //Save Settings here!!!  <------------------------
        }
      }
    }
    if(buttonpressed[1] == 1) //button was previously pressed, but now it isn't
    {
      if((millis() - buttonpressedtime[1]) > DEBOUNCETIME)
      {
        //only clear the flag if we're out of the debounce window
        buttonpressed[1]=0;
        if(state == SET_CHANNEL)
        {
          state=NORMAL;
          clearLED(); 
          delay(100);
          //Save Settings here!!!  <------------------------
        }
      }
    }
    if(buttonpressed[2] == 1) //button was previously pressed, but now it isn't
    {
      if((millis() - buttonpressedtime[2]) > DEBOUNCETIME)
      {
        //only clear the flag if we're out of the debounce window
        buttonpressed[2]=0;
        if(state == SET_OPTIONS)
        {
          state=NORMAL;
          clearLED(); 
          delay(100);
          //Save Settings here!!!  <------------------------
        }
      }
    }
  }

  //THE POT READING PART
  for (x=0;x<4;x++)
  {
    analogPOTS[x].update();
    if(analogPOTS[x].hasChanged()) {
      potvalues[x]=analogPOTS[x].getValue();
      if(state==NORMAL)
        update[x]=TRUE;

      if(state == SET_MESSAGE)
      {
        messages[x]= map(potvalues[x],0,MAXPOTVALUE,0,129);
        if(messages[x]>129)
          messages[x]=129;
        //we're in the SET_MESSAGE MODE, so movement of the fader sets a new message
        //display the current value on the LEDs
        if(messages[x]<=127)  //standard CC
        {
          LEDchars[0]=charactertoLED(x+1,NUMBER,0);
          LEDchars[1]=charactertoLED((messages[x]%100)/10,NUMBER,0);
          LEDchars[2]=charactertoLED(messages[x]/100,NUMBER,0);
          LEDchars[3]=charactertoLED(messages[x]%10, NUMBER,1);
        }
        else if(messages[x]==128) //Pitch Bend
        {
          
          LEDchars[0]=charactertoLED(x+1,NUMBER,0);
          LEDchars[1]=charactertoLED('P',LETTER,0);
          LEDchars[2]=charactertoLED(0,RAW,0);
          LEDchars[3]=charactertoLED('B', LETTER,1);
        }
        else if(messages[x]==129) //Program Change
        {
          
          LEDchars[0]=charactertoLED(x+1,NUMBER,0);
          LEDchars[1]=charactertoLED('P',LETTER,0);
          LEDchars[2]=charactertoLED(0,RAW,0);
          LEDchars[3]=charactertoLED('C', LETTER,1);
        }
        saveDataFlag = 1; //mark the flag so we save the data
      }
      
      else if(state == SET_CHANNEL)
      {
        channels[x]= map(potvalues[x],0,MAXPOTVALUE,0,15);
        if(channels[x]>15)
          channels[x] = 15;
        //we're in the SET_CHANNEL MODE, so movement of the fader sets a new message
        //display the current value on the LEDs

        LEDchars[0]=charactertoLED(x+1,NUMBER,0);
        LEDchars[1]=charactertoLED(((channels[x]+1)%100)/10,NUMBER,0);
        LEDchars[2]=charactertoLED(0,RAW,0);
        LEDchars[3]=charactertoLED((channels[x]+1)%10, NUMBER,0);
        saveDataFlag = 1;

      }

      else if(state == SET_OPTIONS)
      {
        if(x==0)
        {
          
          options[x]= map(potvalues[x],0,MAXPOTVALUE,10,99);
          if(options[x]>99)
            options[x] = 99;
          LEDchars[0]=charactertoLED('B',LETTER,0);
          LEDchars[1]=charactertoLED((options[x]%100)/10,NUMBER,0);
          LEDchars[2]=charactertoLED(0,RAW,0);
          LEDchars[3]=charactertoLED(options[x]%10, NUMBER,0);
        }
        updateOptions();
        saveDataFlag = 1;
      }
      
    }
  }

  for (x=0;x<4;x++)
  {
    if(update[x]==TRUE)
    {
      update[x]=FALSE;
      
      sendMIDI(potvalues[x],channels[x],messages[x],x);
    }
  }
}
bool firstbootread[4]={1,1,1,1};
unsigned int lastdata[4]={0,0,0,0};
void sendMIDI (unsigned int data, unsigned char channel, unsigned char message, unsigned char lane)
{
  unsigned char temp;
  if((message >=0) && (message<=127)) //standard midi CC
  {
    //data = constrain(data,25,1010);
    //temp=map(data,25,1010,0,127);
    //temp = constrain(data,0,MAXPOTVALUE);
    temp = map(data,0,MAXPOTVALUE, 0, 127);
    if(temp>127)
      temp = 127;
    if(firstbootread[lane])
    {
      lastdata[lane]=temp;
      firstbootread[lane]=0;
    }
    if(temp!=lastdata[lane])
    {
      //create midi message here
      /*
      Serial.print("lane: ");
      Serial.print(lane);
      Serial.print(" channel: ");
      Serial.print(channel);
      Serial.print(" CC: ");
      Serial.print(message);
      Serial.print(" data: ");
      Serial.print(temp);
      Serial.print(" raw: ");
      Serial.println(data);
      Serial.println("");
      */
      lastdata[lane]=temp;
      LEDchars[0]=charactertoLED('C',LETTER,0);
      LEDchars[1]=charactertoLED((temp%100)/10,NUMBER,0);
      LEDchars[2]=charactertoLED(temp/100,NUMBER,0);
      LEDchars[3]=charactertoLED(temp%10, NUMBER,0);

      

        MIDI.sendControlChange(message, temp, channel+1);
        midiEventPacket_t event = {0x0B, (uint8_t)(0xB0 | channel), message, temp};
        MidiUSB.sendMIDI(event);
        MidiUSB.flush();

    }    
    else
    {
      //Serial.print(".");  //for debugging only
      //Serial.println(lastdata[lane]);
    }

  }
  else if(message == 128) //Pitch Bend
  { //wide range, 14 bit?
    int pbval;
    pbval = data<<4;

    if(firstbootread[lane])
    {
      lastdata[lane]=pbval;
      firstbootread[lane]=0;
    }
    if(pbval!=lastdata[lane])
    {
      lastdata[lane]=pbval;
      midiEventPacket_t event = {0x0E, (uint8_t)(0xE0 | channel), (uint8_t)(data>>8), (uint8_t)data};
      pbval = pbval - 8192;
  
      
      MIDI.sendPitchBend(pbval, channel+1);
      
      MidiUSB.sendMIDI(event);
      MidiUSB.flush();
  
      temp = data&0xFF;
      
      
      LEDchars[0]=charactertoLED('B',LETTER,0);
      LEDchars[1]=charactertoLED((data%256)/16,HEX,0);
      LEDchars[2]=charactertoLED(data/256,HEX,0);
      LEDchars[3]=charactertoLED(data%16, HEX,0);
    }
  }
  
  else if(message == 129) //Program Change
  {
    temp = map(data,0,MAXPOTVALUE, 0, 127);
    if(temp>127)
      temp = 127;


    if(firstbootread[lane])
    {
      lastdata[lane]=temp;
      firstbootread[lane]=0;
    }
    if(temp!=lastdata[lane])
    {
      lastdata[lane]=temp;
      MIDI.sendProgramChange(temp,channel+1);
      midiEventPacket_t event = {0x0C, (uint8_t)(0xC0 | channel), (uint8_t)(temp)};
      MidiUSB.sendMIDI(event);
      MidiUSB.flush();
  
       
  
      LEDchars[0]=charactertoLED('P',LETTER,0);
      LEDchars[1]=charactertoLED((temp%100)/10,NUMBER,0);
      LEDchars[2]=charactertoLED(temp/100,NUMBER,0);
      LEDchars[3]=charactertoLED(temp%10, NUMBER,0);
    }
  }
  
  else
    return;
    //invalid message type


}
unsigned char lastoptions[4];
void updateOptions (void)
{
  if(options[0] != lastoptions[0])
  { //update brightness
    LEDbrightness = options[0];
    REG_TCC0_CCB0 = LEDbrightness;
    lastoptions[0]=options[0];
  }

  
}
void clearLED (void)
{
  LEDchars[0] = 0;
  LEDchars[1] = 0;
  LEDchars[2] = 0;
  LEDchars[3] = 0;
}
void printLED (char digit)
{
  char x;
 
  
  PORT->Group[g_APinDescription[DIGIT1].ulPort].PINCFG[g_APinDescription[DIGIT1].ulPin].bit.PMUXEN = 0;
  PORT->Group[g_APinDescription[DIGIT2].ulPort].PINCFG[g_APinDescription[DIGIT2].ulPin].bit.PMUXEN = 0;
  PORT->Group[g_APinDescription[DIGIT3].ulPort].PINCFG[g_APinDescription[DIGIT3].ulPin].bit.PMUXEN = 0;
  PORT->Group[g_APinDescription[DIGIT4].ulPort].PINCFG[g_APinDescription[DIGIT4].ulPin].bit.PMUXEN = 0;
  
  for(x=0;x<8;x++)
  {
    if(((0x01 << x) & (LEDchars[digit])))
      digitalWrite(segments[x],HIGH);
    else
      digitalWrite(segments[x],LOW);
  }
  
  PORT->Group[g_APinDescription[digits[digit]].ulPort].PINCFG[g_APinDescription[digits[digit]].ulPin].bit.PMUXEN = 1;

  

}



unsigned char charactertoLED(unsigned char data, unsigned char type, bool decimal)
{
  char numbers[] = {
        B00111111,  //  0
        B00000110,  //  1
        B01011011,  //  2
        B01001111,  //  3
        B01100110,  //  4
        B01101101,  //  5
        B01111101,  //  6
        B00000111,  //  7
        B01111111,  //  8
        B01101111,  //  9        
    };
    char letters[] = {
        B01110111,  //  A
        B01111100,  //  B
        B01011000,  //  C
        B01011110,  //  D
        B01111001,  //  E
        B01110001,  //  F
        B00111101,  //  G
        B01110100,  //  H
        B00110000,  //  I
        B00011110,  //  J
        B00000000,  //  NO K
        B00111000,  //  L      
        B00000000,  // NO M
        B01010100,  // N
        B00111111,  // O
        B01110011,  // P  
        B01100111,  // Q  
        B01010000,  // r
        B01101101,  // S
        B01111000,  // t
        B00011100,  // u
        B00000000,  // no V
        B00000000,  // no W
        B00000000,  // no X
        B01101110,  // y
        B00000000   // no z        
    };


  lastLEDUpdate = millis();
  restorebrightness();
  switch (type){

    case NUMBER:
      if((data>=0) && (data<= 9))
      {
        if(decimal ==0)
          return numbers[data];
        else
          return (numbers[data]|B10000000);
      }
      break;
    case LETTER:
      if(data>='A' && (data<='Z'))
        return letters[(data-'A')];
      break;
    case RAW:
      return data;
      break;
    case HEX:
    {
      if((data>=0) && (data <=15))  //range 0-15 for a hex digit
      {
        if(data<10)
        {
          if(decimal == 0)
            return numbers[data];
          else
            return numbers[data]|B10000000;
        }
        else
        {
          if(decimal == 0)
            return letters[data-10];
          else
            return letters[data-10]|B10000000;
        }
      }
    }
  }

  
  return 0;
}

void welcome (void)
{
  unsigned char temp = 0;
  unsigned char x,y;
  unsigned int temp2;
  for (x=0;x<20;x++)
  {
    
    temp = 1<< random(7);
    LEDchars[0] = temp;
    temp = 1<<random(7);
    LEDchars[1] = temp;
    temp = 1<<random(7);
    LEDchars[2] = temp;
    temp = 1<<random(7);
    LEDchars[3] = temp;
    for(y=0;y<60;y++)
    {
      analogPOTS[0].update();
      temp2=analogPOTS[0].getValue();
      analogPOTS[1].update(); //update all faders so we don't jump at the start
      temp2=analogPOTS[1].getValue();
      analogPOTS[2].update();
      temp2=analogPOTS[2].getValue();
      analogPOTS[3].update();
      temp2=analogPOTS[3].getValue(); 
      delay(1);   
    }
  }

  clearLED();

  //Uncomment this section to test LED brightnesses
  //Used for matching LED display brightness (some are sometimes brighter than others?)
  /*
  LEDchars[0]  = charactertoLED(8,NUMBER,1);
  LEDchars[1]  = charactertoLED(8,NUMBER,1);
  LEDchars[2]  = charactertoLED(8,NUMBER,1);
  LEDchars[3]  = charactertoLED(8,NUMBER,1);
  delay(10);
  while(1);
  */
  //------------------------------------------------
}

// TIMER INTERRUPT STUFF HERE

unsigned long int lasttc3micros;
void TC3_Handler() {
  static char counter = 0;
  //TcCount16* TC = (TcCount16*) TC3;
  
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC3->COUNT16.INTFLAG.bit.MC0 == 1) {
    TC3->COUNT16.INTFLAG.bit.MC0 = 1;
    
      
    printLED(counter);
    counter++;
    if(counter>3)
    counter = 0;
  }

}

////////////////////////

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  //Serial.println(TC->COUNT.reg);
  //Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void setupTCC0(void) // used to set up fast PWM on pins 1,9,2,3
{
 
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(2) |          //// Divide the 48MHz clock source by divisor N=1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            //// Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          //// Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  //enable our 4 pins to be PWM outputs
  PORT->Group[g_APinDescription[DIGIT1].ulPort].PINCFG[g_APinDescription[DIGIT1].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[DIGIT2].ulPort].PINCFG[g_APinDescription[DIGIT2].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[DIGIT3].ulPort].PINCFG[g_APinDescription[DIGIT3].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[DIGIT4].ulPort].PINCFG[g_APinDescription[DIGIT4].ulPin].bit.PMUXEN = 1;
  
  //assign the 4 outputs to the PWM registers on PMUX
  PORT->Group[g_APinDescription[DIGIT1].ulPort].PMUX[g_APinDescription[DIGIT1].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;  // digit1 is on PA16 = even     
  PORT->Group[g_APinDescription[DIGIT2].ulPort].PMUX[g_APinDescription[DIGIT2].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F; // 2 is on PA18 = even 
  PORT->Group[g_APinDescription[DIGIT3].ulPort].PMUX[g_APinDescription[DIGIT3].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F;  // 3 is on PA17 = odd
  PORT->Group[g_APinDescription[DIGIT4].ulPort].PMUX[g_APinDescription[DIGIT4].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F; // 4 is on PA19 = odd

  //set drive str for all 4

  PORT->Group[g_APinDescription[DIGIT1].ulPort].PINCFG[g_APinDescription[DIGIT1].ulPin].bit.DRVSTR = 1;
  PORT->Group[g_APinDescription[DIGIT2].ulPort].PINCFG[g_APinDescription[DIGIT2].ulPin].bit.DRVSTR = 1;
  PORT->Group[g_APinDescription[DIGIT3].ulPort].PINCFG[g_APinDescription[DIGIT3].ulPin].bit.DRVSTR = 1;
  PORT->Group[g_APinDescription[DIGIT4].ulPort].PINCFG[g_APinDescription[DIGIT4].ulPin].bit.DRVSTR = 1;
  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK4 //0 only works for interrup?
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_TCC0_WEXCTRL |= TCC_WEXCTRL_OTMX(0x2);
  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |           // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTH |
                    TCC_WAVE_WAVEGEN_NFRQ;     // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  
  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: Freq = 48Mhz/(2*N*PER)
  REG_TCC0_PER = 99;                           // Set the FreqTcc and period of the PWM on TCC1
  while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization
 
  //REG_TCC0_CC1 = 50;                             // TCC1 CC1 - on D3  50% pin 9
  //while (TCC0->SYNCBUSY.bit.CC1);                   // Wait for synchronization
  REG_TCC0_CCB0 = LEDbrightness;                             // TCC1 CC0 - on D11 50% pin 1
  while (TCC0->SYNCBUSY.bit.CC0);                   // Wait for synchronization
  //  REG_TCC0_CC2 = 50;                             // TCC1 CC1 - on D3  50% pin 2
  //while (TCC0->SYNCBUSY.bit.CC2);                   // Wait for synchronization
  //REG_TCC0_CC3 = 50;                             // TCC1 CC0 - on D11 50% pin 3
  //while (TCC0->SYNCBUSY.bit.CC3);                   // Wait for synchronization
 
  // Divide the GCLOCK signal by 1 giving  in this case 48MHz (20.83ns) TCC1 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV2 |    // Divide GCLK4 by 1 ****************************************************************************
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
  /*
  TCC0->INTENSET.reg = 0;
  TCC0->INTENSET.bit.CNT = 1;  //*****************************************************
  TCC0->INTENSET.bit.MC0 = 0;

  NVIC_EnableIRQ(TCC0_IRQn);
  */
  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE ;
  
}

void loadData (void)
{
  //check if this is the first boot:

  if(flash_valid.read())
  {
    //this is not our first boot, load defaults.
    LEDbrightness = saved_brightness.read();

    messages[0]=saved_message1.read();
    messages[1]=saved_message2.read();
    messages[2]=saved_message3.read();
    messages[3]=saved_message4.read();

    channels[0]=saved_channel1.read();
    channels[1]=saved_channel2.read();
    channels[2]=saved_channel3.read();
    channels[3]=saved_channel4.read();

  }

  else  //valid == 0
  {
    //this is our first boot. Save all the defaults into "eeprom"
    saveData();

    //Write a 1 so this check passes next time
    flash_valid.write(1);
  }

 

}

void saveData (void)  //check if the data is the same already so we don't write unnecessarily
{
  //options
  if(saved_brightness.read() != LEDbrightness)
    saved_brightness.write(LEDbrightness);

  //messages
  if(saved_message1.read() != messages[0])
    saved_message1.write(messages[0]);

  if(saved_message2.read() != messages[1])
    saved_message2.write(messages[1]);

  if(saved_message3.read() != messages[2])
    saved_message3.write(messages[2]);

  if(saved_message4.read() != messages[3])
    saved_message4.write(messages[3]);  

  //channels
  if(saved_channel1.read() != channels[0]) 
    saved_channel1.write(channels[0]);

  if(saved_channel2.read() != channels[1]) 
    saved_channel2.write(channels[1]);  

  if(saved_channel3.read() != channels[2]) 
    saved_channel3.write(channels[2]);

  if(saved_channel4.read() != channels[3]) 
    saved_channel4.write(channels[3]);    

  saveDataTime = millis();
}

//these two functions used for fading in and out
void reducebrightness (void)
{
  currentbrightness = currentbrightness - 1;
  REG_TCC0_CCB0 = currentbrightness;
}

void restorebrightness (void)
{ 
  if(currentbrightness != LEDbrightness)
  {
    REG_TCC0_CCB0 = LEDbrightness;
    currentbrightness = LEDbrightness;
  }
  fadeout = 0;
}
