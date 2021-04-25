#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> 
#include <avr/wdt.h>

#define BAUD 9600
#define BUFF_LEN 255
#define BAUD_PRESCALE (((F_CPU / (BAUD * 16UL))) - 1)

uint8_t ledPin = 0;
uint16_t timeCount = 0;
void initIntimer(){
  TCCR0B = 0x05;    // clock frequency / 1024
  OCR0B = 0x00;     // Output compare
  TCNT0 = 0;        // Start to counter 0 to zero
  TIMSK0 = 0x01;    // Enable overflow interrupt  
}

void uart_start(void) {
  UCSR0B |= (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);    //transmit side of hardware
  UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);  //receive side of hardware
  UBRR0L = BAUD_PRESCALE;                   //set the baud to 9600, have to split it into the two registers
  UBRR0H = (BAUD_PRESCALE >> 8);            //high end of baud register
}
void uart_start1(void) {
  UCSR1B |= (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);    //transmit side of hardware
  UCSR1C |= (1 << UCSZ10) | (1 << UCSZ11);  //receive side of hardware
  UBRR1L = BAUD_PRESCALE;                   //set the baud to 9600, have to split it into the two registers
  UBRR1H = (BAUD_PRESCALE >> 8);            //high end of baud register
}
void uart_start3(void) {
  UCSR3B |= (1 << RXCIE3) | (1 << RXEN3) | (1 << TXEN3);    //transmit side of hardware
  UCSR3C |= (1 << UCSZ30) | (1 << UCSZ31);  //receive side of hardware
  UBRR3L = BAUD_PRESCALE;                   //set the baud to 9600, have to split it into the two registers
  UBRR3H = (BAUD_PRESCALE >> 8);            //high end of baud register
}

void uart_sendint(uint8_t data) {
    while ((UCSR0A & (1 << UDRE0)) == 0); //make sure the data register is cleared
    UDR0 = data;                          //send the data
}
void uart_sendstr(char *data) {
    while (*data) {
        while ((UCSR0A & (1 << UDRE0)) == 0); //make sure the data register is cleared
        UDR0 = *data;                         //goes through and splits the string into individual bits, sends them
        data += 1;                            //go to new bit in string
    }
}

ISR(USART1_RX_vect) {
      char snum[5];
      itoa(UDR1, snum, 10);
      uart_sendstr(snum);
      uart_sendstr("\r\n");
}

ISR(USART3_RX_vect) {
      char snum[5];
      itoa(UDR3, snum, 10);
      uart_sendstr(snum);
      uart_sendstr("\r\n");
}

char      input_buffer[BUFF_LEN];
uint16_t  read_spot;

ISR(USART0_RX_vect) {                         //sets up the interrupt to recieve any data coming in
    //uart_sendint(UDR0);
    input_buffer[read_spot] = UDR0;
    read_spot++;                              //and "exports" if you will the data to a variable outside of the register
    if(read_spot > BUFF_LEN-1) read_spot = 0;
}

ISR(TIMER0_OVF_vect){
  timeCount++;
  if(timeCount == 61) // Timer overflown for the 61th time
  {
    PORTB ^= ledPin;  // Toggle the LED
    timeCount = 0;    // Reset overflow counter
  }
}

ISR(PCINT0_vect){
  uart_sendstr("hallo\r\n");
  _delay_ms(1000);
}



void pwm0A(uint8_t value) {
  TCCR0A |= (1<<COM0A1);
  OCR0A = value;
}

void pwm0B(uint8_t value) {
  TCCR0A |= (1<<COM0B1);
  OCR0B = value;
}

void pwm1A(uint16_t value) {
  TCCR1A |= (1<<COM1A1);
  OCR1A = value;
}

void pwm1B(uint16_t value) {
  TCCR1A |= (1<<COM1B1);
  OCR1B = value;
}

void pwm2A(uint8_t value) {
  TCCR2A |= (1<<COM2A1);
  OCR2A = value;
}

void pwm2B(uint8_t value) {
  TCCR2A |= (1<<COM2B1);
  OCR2B = value;
}


struct data {
  uint8_t data1;
  uint16_t data2;
  };

union buff {
  uint8_t load[8];
  data dataLcd;
};
buff buffLcd;


int main(void)
{
  cli();
  _delay_ms(1000);
  DDRB |= 0xff;
  DDRB |= (0<<DDB0);
  PORTB|= (1<<PORTB0);
  //DDRB |= _BV(PB7); // Set PB7 as output, ignore the rest
  //ledPin = _BV(PB7);
  
  uart_start();
  uart_start1();
  uart_start3();
  initIntimer();

  TCCR0B |= (1<<CS00) | (1<<CS01);
  TCCR0A |= (1<<WGM00); //fast pwm with top as 0xFF
  DDRD |= (1<<5);
  DDRD |= (1<<6);       //set the OCR0 pins as outputs

//  TCCR1B |= (1<<CS11) | (1<<CS10);  //set timer1 clock prescaler to 64
//  TCCR1A |= (1<<WGM10)| (1<<WGM12); //fast pwm (8bit) with top as 0x03FF
//  DDRB |= (1<<1);
//  DDRB |= (1<<2);                   //set the OCR1 pins as outputs

//  TCCR2B |= (1<<CS22);//set timer2 clock prescaler to 64
//  TCCR2A |= (1<<WGM20);//fast pwm with top as 0xFF
//  DDRD |= (1<<3);
//  DDRB |= (1<<3); //set the OCR2 pins as outputs

  PCICR  |= (1<<PCIE0);
  PCMSK0 |= (1<<PCINT0);

  
  sei();
  pwm0A(100);
  while(1)
  {
    wdt_enable(WDTO_1S);
    if(read_spot>0){
//      memcpy(buffLcd.load,input_buffer, 7);
//      char snum[5];
//      itoa(buffLcd.dataLcd.data2, snum, 10);
//      uart_sendstr(snum);
//      uart_sendstr("\r\n");
      uart_sendstr(input_buffer);
      memset(input_buffer, 0, sizeof(input_buffer));
      read_spot = 0;
    }
    
    //PORTB |= 0xff;
    //_delay_ms(1000);
    //PORTB &= ~0xff;
    _delay_ms(500);
  }
  return 0;
}
