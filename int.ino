#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> 

#define BAUD 9600
#define BUFF_LEN 255
#define BAUD_PRESCALE (((F_CPU / (BAUD * 16UL))) - 1)

void uart_start(void) {
  UCSR0B |= (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);    //transmit side of hardware
  UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);  //receive side of hardware
  UBRR0L = BAUD_PRESCALE;                   //set the baud to 9600, have to split it into the two registers
  UBRR0H = (BAUD_PRESCALE >> 8);            //high end of baud register
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

void uart_start3(void) {
  UCSR3B |= (1 << RXCIE3) | (1 << RXEN3) | (1 << TXEN3);    //transmit side of hardware
  UCSR3C |= (1 << UCSZ30) | (1 << UCSZ31);  //receive side of hardware
  UBRR3L = BAUD_PRESCALE;                   //set the baud to 9600, have to split it into the two registers
  UBRR3H = (BAUD_PRESCALE >> 8);            //high end of baud register
}
void uart_start1(void) {
  UCSR1B |= (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);    //transmit side of hardware
  UCSR1C |= (1 << UCSZ10) | (1 << UCSZ11);  //receive side of hardware
  UBRR1L = BAUD_PRESCALE;                   //set the baud to 9600, have to split it into the two registers
  UBRR1H = (BAUD_PRESCALE >> 8);            //high end of baud register
}


char      input_buffer[BUFF_LEN];
uint16_t  read_spot;

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

ISR(USART0_RX_vect) {                         //sets up the interrupt to recieve any data coming in
    //uart_sendint(UDR0);
    input_buffer[read_spot] = UDR0;
    read_spot++;                              //and "exports" if you will the data to a variable outside of the register
    if(read_spot > BUFF_LEN-1) read_spot = 0;
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
  uart_start();
  uart_start1();
  uart_start3();
  
  sei();
  while(1)
  {

    if(read_spot>0){
      memcpy(buffLcd.load,input_buffer, 7);
      char snum[5];
      itoa(buffLcd.dataLcd.data2, snum, 10);
      uart_sendstr(snum);
      uart_sendstr("\r\n");
      //uart_sendstr(input_buffer);
      memset(input_buffer, 0, sizeof(input_buffer));
      read_spot = 0;
    }
    
    PORTB |= 0xff;
    _delay_ms(1000);
    PORTB &= ~0xff;
    _delay_ms(1000);
  }
  return 0;
}


