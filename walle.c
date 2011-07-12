#include<avr/io.h>
#include<stdlib.h>
#include <compat/deprecated.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include<avr/interrupt.h>

unsigned char min[7],max[7],threshold[7],sensorbyte=0,ps=0,u=0,flag=1,flag1=1,flag2=1,flag3=1;
long int mpos=0,control=0,i,node=0,a,b,pa,pb,c,d,pc,pd,stage=0,temp=0,countl=0,countr=0,count=0,m,n,r=1,s=1,t=1;

void leftforward()
{
 sbi(PORTC,0);
 cbi(PORTC,1);
 cbi(PORTC,2);
 cbi(PORTC,3);

}
void rightforward()
{
 cbi(PORTC,0);
 cbi(PORTC,1);
 sbi(PORTC,2);
 cbi(PORTC,3);
 
}

void botforward()
{
 sbi(PORTC,0);
 cbi(PORTC,1);
 sbi(PORTC,2);
 cbi(PORTC,3);
}

void botstop()
{
 cbi(PORTC,0);
 cbi(PORTC,1);
 cbi(PORTC,2);
 cbi(PORTC,3);
}

void port_init(void)
{
 PORTA = 0xFF;
 DDRA  = 0x00;
 PORTB = 0xFF;  
 DDRB  = 0x00;
 PORTC = 0x00; //m103 output only
 DDRC  = 0xFF;
 PORTD = 0xFF;
 DDRD  = 0x00;
}

//TIMER1 initialize - prescale:1
// WGM: 10) PWM phz correct, TOP= ICRn
// desired value: 4KHz
// actual value:  4.000KHz (0.0%)
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xF8; //setup
 TCNT1L = 0x30;
 OCR1AH = 0x07;
 OCR1AL = 0xD0;
 OCR1BH = 0x07;
 OCR1BL = 0xD0;
 ICR1H  = 0x07;
 ICR1L  = 0xD0;
 TCCR1A = 0xA2;
 TCCR1B = 0x11; //start Timer
}

void adc_init(void)
{
 ADCSRA=0X00;
 ADMUX=0X60;//0x40 for 10 bits
 ADCSRA=0X87;
 ACSR=0X80;
}

unsigned char adc_start(unsigned char channel)
{
 unsigned char i;
 
     ADCH=0x00;

	 i=channel&0x07;
	 ADMUX=i|0x60;                //i|0x40 for 10 bits
	 ADCSRA|=1<<ADSC;
	   
		 while(ADCSRA & (1<<ADSC));       // wait for conv. to complete
		    unsigned char temp=ADCH;      //unsigned int temp=ADC;   for 10 bits
   
 return temp;
}


void delay(int x)
{
 unsigned char i,j;
 for(i=0;i<x;i++)
  for(j=0;j<4;j++)
   _delay_ms(250);
}

void init_encoders(void) 			// TO  initialise initial position of encoders
{									// DO NOT MESS. TESTED WORKING.

if(bit_is_set(PINB,2))
  pa=1;
 else
  pa=0;
 if(bit_is_set(PINB,3))
  pb=1;
 else 
  pb=0; 
  
  if(bit_is_set(PINB,0))
  pc=1;
 else
  pc=0;
 if(bit_is_set(PINB,1))
  pd=1;
 else 
  pd=0;
  
} 
  
  
  
void encoder(void)
{										// ENCODER POLLING
if(bit_is_set(PINB,2))
  a=1;
 else
  a=0;
 if(bit_is_set(PINB,3))
  b=1;
 else 
  b=0; 

if( pa != a || pb != b  )
{


if(pa==0 && pb==0)
{
 if(a==0 && b==1)
{
countl++;
}

else if(a==1 && b==0)
{
countl++;
}

}

else if(pa==0 && pb==1)
{ 
if(a==1 && b==1)
{
countl++;
}
else if(a==0 && b==0)
{
countl++;
}

}

else if(pa==1 && pb==1)
{ if(a==1 && b==0)
{
countl++;
}
else if(a==0 && b==1)
{
countl++;
}

}

else if(pa==1 && pb==0)
{ 
if(a==0 && b==0)
{
countl++;
}
else if(a==1 && b==1)
{
countl++;
}

}

pa=a;
pb=b;

}


if(bit_is_set(PINB,0))
  c=1;
 else
  c=0;
 if(bit_is_set(PINB,1))
  d=1;
 else 
  d=0; 

if( pc != c || pd != d  )
{


if(pc==0 && pd==0)
{
 if(c==0 && d==1)
{
countr++;
}

else if(c==1 && d==0)
{
countr++;
}

}

else if(pc==0 && pd==1)
{ 
if(c==1 && d==1)
{
countr++;
}
else if(c==0 && d==0)
{
countr++;
}

}

else if(pc==1 && pd==1)
{ if(c==1 && d==0)
{
countr++;
}
else if(c==0 && d==1)
{
countr++;
}

}

else if(pc==1 && pd==0)
{ 
if(c==0 && d==0)
{
countr++;
}
else if(c==1 && d==1)
{
countr++;
}

}

pd=d;
pc=c;

}


}


void checksensors(void)
{
sensorbyte=0;

unsigned char i,temp[8];

	 for(i=0;i<8;i++)
	 {
	 
	  temp[i]=adc_start(i);
	  if(temp[i]<threshold[i])
	  sensorbyte|=(1<<i);
	 
	 }
	 
}
 
 
 void calibrateblack(void)
{
	unsigned char j,i,temp[8];

	for(j=0;j<8;j++) 
	 {
		  max[j]=adc_start(j);
		  
		  for(i=0;i<10;i++)
		 {
			  temp[j]=adc_start(j);
			  
			  if(temp[j]<max[j])
			  {
			  max[j]=temp[j];
			  }
			  
		 }
	 
	}

}

void calibratewhite(void)
{

	unsigned char j,i,temp[8];
	 
	 for(j=0;j<8;j++) 
	 {
		  min[j]=adc_start(j);
		  
		  for(i=0;i<10;i++)
		 {
			  temp[j]=adc_start(j);
			  
			 if(temp[j]<min[j])
			  {
			  min[j]=temp[j];
			  }
		  
		 }
	 
	}

}


void setthreshold(void)
{

	unsigned char i,eeprom_addr=0x0000;;
	char diff;
	
	 for(i=0;i<8;i++)
	 {
	 
		 diff=max[i]-min[i];
		 threshold[i]=min[i]+(diff>>1);
		  
	 }
	 for(i=0;i<8;i++)
 {
  eeprom_write_byte(eeprom_addr,threshold[i]);
  eeprom_addr++;
  }
}

void retrievethreshold(void)		// Get thresholds from eeprom
{
unsigned int i,eeprom_addr=0x0000;
 for(i=0;i<8;i++)
 {
 threshold[i]=eeprom_read_byte(eeprom_addr);
 eeprom_addr++;

 }
 
}
void flick (void)
{
unsigned int i=0;

	for(i=0;i<5;i++)
	{
		PORTC=0xff;
		_delay_ms(100);
		PORTC=0x00;
		_delay_ms(100);
	}

}


void line_track(void)
{

checksensors();

switch(sensorbyte)
{
case 0b00000100:botforward();
				 break;
case 0b00010000:leftforward(); 
                  break;
case 0b00011000:leftforward();
                  break;
case 0b00011100:leftforward();
                  break;
case 0b00000001:rightforward(); 
                  break;
case 0b00000011:rightforward(); 
                  break;
case 0b00001000:leftforward(); 
                  break;
case 0b00000010:rightforward(); 
                  break;
case 0b00000111:rightforward(); 
                  break;
case 0b00000110:botforward(); 
                  break;
case 0b00001100:botforward(); 
                  break;
case 0b00000000:botstop(); 
                  break;
case 0b00011111:botstop(); 
                  break;
				  	  
default :botforward();		break;
}
}


 
void init_devices(void)
{
 timer1_init();
 port_init();
 adc_init();
}

void main(void)
{
init_devices();
retrievethreshold();
init_encoders();
while(1)
{
/*leftforward();
rightforward();
}}*/

        if(bit_is_clear(PIND,0))              //pressing s1 blink1portpin will execute
		{
		calibratewhite();
		PORTC=0XFF;
		setthreshold();
		
		flick();
		}
		
		if(bit_is_clear(PIND,1))              //pressing s2 pattern1 will execute
		{
		calibrateblack();
		
		setthreshold();
		
		flick();
		}
		
		if(bit_is_clear(PIND,2))              //pressing s2 pattern1 will execute
		{
		while(1)
		{
		line_track();
		}}
		}}