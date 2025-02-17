#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

//ROM COMMANDS
#define READ_ROM          0x33
#define MATCH_ROM         0x55
#define SKIP_ROM          0xCC
#define ALARM_SEARCH      0xEC
#define SEARCH_ROM        0xF0

//FUNCTION COMMANDS
#define CONVERT           0x44
#define W_SCRATCHPAD      0x4E
#define R_SCRATCHPAD      0xBE
#define COPY_SCRATCHPAD   0x48
#define RECALL            0xB8

//Configuration
#define TH                0x18            //25C
#define TL                0x14            //20C
#define CONF              0b01111111      //rozdzielczość 12-bitowa
#define DEVICES           2               //ilość podpiętych urządzeń na magistrali One-Wire

//........................................................ DS18B20 ................................................................
void reset()
{
  //Master jako transmiter
  DDRD |= (1<<PD2);
  PORTD &= ~(1<<PD2);
  _delay_us(480);

  //Zwolnienie magistrali 
  DDRD &= ~(1<<PD2);
  _delay_us(60);
}

void presence()
{
  //Czas na odpowiedz czujnika
  _delay_us(60);

  //Sprawdzenie czy czujnik odpowiedział (PD2 jest w stanie niskim) 
  if(!(PIND & (1<<PD2)))
  {
    _delay_us(240); 
  }
}

void writeByte(uint8_t byte_)
{
  for(int i=0; i<8; i++)
  {
    writeBit((byte_>>i) & 0b00000001);
  }
}

void writeBit(uint8_t bit_)
{
  //Master jako transmiter
  DDRD |= (1<<PD2);

  //Magistrala w dół
  PORTD &= ~(1<<PD2);
  
  if(bit_== 1)
  {
    _delay_us(2);

    //Zwolnienie magistrali
    DDRD &= ~(1<<PD2);

    _delay_us(60);
  }
  else
  {
    _delay_us(60);
    
    //Zwolnienie magistrali
    DDRD &= ~(1<<PD2);
    
    _delay_us(2);
  } 
}

uint8_t readBit()
{
  uint8_t bit_;
  
  DDRD |= (1<<PD2);
  PORTD &= ~(1<<PD2);
  _delay_us(2);
    
  //Zwolnienie magistrali
  DDRD &= ~(1<<PD2);
  _delay_us(15);
    
  if(PIND & (1<<PD2))
  {
    bit_ = 1;
  }
  else
  {
    bit_ = 0;
  }
  
  _delay_us(45);

  return bit_;
}

uint8_t readByte()
{
  uint8_t byte_ = 0b00000000;
  uint8_t bit_;
  
  for(int i=0; i<8; i++)
  {
    bit_ = readBit();

    if(bit_ == 1)
    {
      byte_ = byte_ | (1<<i);
    }

     _delay_us(45);
  }
  
  return byte_;
}

void wScratchpad()
{
  writeByte(TH);
  writeByte(TL);
  writeByte(CONF);
}

void rScratchpad()
{
  uint8_t ROWS[9];

  for(int i=0; i<9; i++)
  {
    ROWS[i] = readByte();
  }

  uint8_t crc = countCRC(ROWS);

  if(crc != ROWS[8])
  {
    //Błędny crc, odczyt powtórzony
    DDRB |= (1<<PB5);
    PORTB |= (1<<PB5);  
    
    writeByte(R_SCRATCHPAD);
    rScratchpad();
  }
  else
  {
    return;
  }
}

uint8_t countCRC(uint8_t *rows)
{
  uint8_t crc = 0b00000000;
  
  for(int i=0; i<8; i++)
  {
    crc ^= rows[i];

    for(int j=0; j<8; j++)
    {
      if((crc&0b00000001)==1)
      {
        crc = (crc >> 1) ^ 0x8C;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

void cScratchpad()
{
  DDRD |= (1<<PD2);
  PORTD |= (1<<PD2);
  _delay_us(10);

  DDRD &= ~(1<<PD2);
  _delay_ms(10);
}

uint8_t** searchRom() {
  
  //alokacja pamięci na wskaźniki do wierszy
  uint8_t** devices = malloc(DEVICES * sizeof(uint8_t*));
    
  //alokacja pamięci na każdy wiersz (kolumny)
  for (int i = 0; i < DEVICES; i++) {
    devices[i] = malloc(8 * sizeof(uint8_t));
  }
    
  //zerowanie tablicy
  for (int i = 0; i<DEVICES; i++){
    for(int j =0; j<8; j++){
      devices[i][j] = 0;
    }
  }
   
  uint8_t bit1 = 0;
  uint8_t bit0 = 0;
  uint8_t search_direction = 0;
  uint8_t last_conflict = 0; 
  uint8_t device_found = 0; 

  do {
    reset();
    presence();
    writeByte(SEARCH_ROM);

    for (int i = 0; i < 64; i++) {  
      bit0 = readBit();  
      bit1 = readBit();  

      if (bit0 == 0 && bit1 == 1) {
        //wszystkie urządzenia wysyłają 0
        search_direction = 0;  
      } 
      else if (bit0 == 1 && bit1 == 0) {
        //wszystkie urządzenia wysyłają 1
        search_direction = 1;  
      } 
      else if (bit0 == 0 && bit1 == 0) {
        //konflikt
        if (i == last_conflict) {
          search_direction = 1;  
        } 
        else if (i > last_conflict) {
          search_direction = 0;  
          last_conflict = i;  
        } 
        else {
          search_direction = 0;  
        }
      } 
      
      else {
        //brak urządzeń
        last_conflict = 0;
        break;
      }

      //zapisanie kierunku
      writeBit(search_direction);

      //zapisanie bitu do adresu ROM
      if (search_direction == 1) {
        devices[device_found][i / 8] = (1<<(i%8)) | devices[device_found][i / 8];  
      }
    }
    
    device_found++;  

  } while (last_conflict != 0 && device_found < DEVICES); 

  return devices;  
}

void showRom(uint8_t** ROM_address)
{
  char string[50];
  
   for (int i = 0; i < DEVICES; i++) {
    for(int j=0; j<8; j++){
      sprintf(string, "%02X", ROM_address[i][j]);
      USART_String(string);
      USART_String(" ");
    }
    USART_String("             ");
  }
  USART_String("\n");
}

void freeMemory(uint8_t** ROM_address)
{
   //zwolnienie pamięci
  for (int i = 0; i < DEVICES; i++) {
    free(ROM_address[i]);
  }
  free(ROM_address);
}

float readTemperature(uint8_t** ROM_address, uint8_t number)
{
    reset();
    presence();

    writeByte(MATCH_ROM);
    for(int i=0; i<8; i++){
      writeByte(ROM_address[number][i]);
    }
    writeByte(CONVERT);
    // Czekaj dopóki konwersja trwa
    while(!(PIND & (1<<PD2))) {}

    reset();
    presence();
    writeByte(MATCH_ROM);
    for(int i=0; i<8; i++){
      writeByte(ROM_address[number][i]);
    }
    writeByte(R_SCRATCHPAD);

    uint8_t ROWS[4];
    for(int i = 0; i <4; i++)
    {
       ROWS[i] = readByte();
    }

    // Odczyt temperatury
    uint16_t read_temp = (ROWS[1] << 8) | ROWS[0]; 
    float temp = read_temp / 16.0; 

    uint8_t th = TH; //ROWS[2];
    uint8_t tl = TL; //ROWS[3];
    
    checkAlarm(th,tl,temp);

    return temp; 
}

void checkAlarm(uint8_t th, uint8_t tl, float temp)
{
  //reset();
  //presence();
  //writeByte(ALARM_SEARCH);

  //_delay_us(60);

  //if(!(PIND & (1<<PD2)))
  
  if(temp>=th)
  {
    DDRB |= (1<<PB5);
    PORTB |= (1<<PB5);
    USART_String("ALARM -> ");
  }
  else
  {
    PORTB &= ~(1<<PB5);
    USART_String("NO ALARM -> ");
  }
}

void DS18B20_init(uint8_t** ROM_address, uint8_t number)
{
  
  for(int j=0; j<DEVICES; j++){
    reset();
    presence();
    
    writeByte(MATCH_ROM);
    
    for(int i=0; i<8; i++){
      writeByte(ROM_address[j][i]);
    }
    writeByte(W_SCRATCHPAD);
    wScratchpad();

    writeByte(COPY_SCRATCHPAD);
    cScratchpad();
  }
  
  reset();
  presence();
  
  writeByte(MATCH_ROM);
  for(int i=0; i<8; i++){
    writeByte(ROM_address[number][i]);
  }
  writeByte(CONVERT);
  while(!(PIND & (1<<PD2))) {}

  reset();
  presence();
  
  writeByte(MATCH_ROM);
  for(int i=0; i<8; i++){
    writeByte(ROM_address[number][i]);
  }

  writeByte(R_SCRATCHPAD);
  rScratchpad();
 
}

//......................................................... USART .................................................................
void USART_init()
{
  UBRR0H = (unsigned char)(103>>8);
  UBRR0L = (unsigned char)103;
    
  //Adres we/wy rejestrów danych nadawczych USART i rejestry odbierania danych USART
  //UDR0

  //By bufor transmisji mógłbyć zapisany
  UCSR0A |= (1<<UDRE0);

  //Włączenie odbiornika
  UCSR0B |= (1<<RXEN0);

  //Włączenie nadajnika
  UCSR0B |= (1<<TXEN0);

  //Liczba bitów danych w ramce
  UCSR0C |= (1<<UCSZ00);
  UCSR0C |= (1<<UCSZ01);

  //Bit stopu
  UCSR0C &= ~(1 << USBS0);  
}

void USART_Transmit( unsigned char data )
 {
 /* Wait for empty transmit buffer */
 while ( !( UCSR0A & (1<<UDRE0)) )
 ;
 /* Put data into buffer, sends the data */
 UDR0 = data;
 }

unsigned char USART_Receive()
 {
 /* Wait for data to be received */
 while ( !(UCSR0A & (1<<RXC0)) )
 ;
 /* Get and return received data from buffer */
 return UDR0;
 }

void USART_String(const char *array)
{
  int i=0;
  while(array[i]!='\0')
  {
    USART_Transmit(array[i]);
    i++;
  }
}



//.......................................................... MAIN .................................................................
int main()
{
  //Czujnik na PD2 (wejście)
  DDRD &= ~(1<<PD2);
  PORTD |= (1<<PD2);

  DS18B20_init(searchRom(), 0);
  USART_init();
  
  char str_temp1[20];
  char str_temp2[20];
  float temp =0;
  uint8_t** ROM_address = searchRom();
  showRom(ROM_address);

 
  while(1)
  {
    for(int i=0; i<DEVICES; i++)
    {
      temp = readTemperature(ROM_address,i);
      dtostrf(temp, 7, 4, str_temp1);
      sprintf(str_temp2, "temperatura: %sC", str_temp1);
      USART_String(str_temp2);
      USART_String("    ");
    }
    USART_String("\n");
    
    _delay_ms(1000);
  }
  
  freeMemory(ROM_address);
  return 0;
}
