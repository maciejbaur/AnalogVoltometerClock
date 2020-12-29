#define NO_PORTD_PINCHANGES    // to indicate that port b will not be used for pin change interrupts
#define NO_PORTC_PINCHANGES    // to indicate that port c will not be used for pin change interrupts
#include <PinChangeInt.h>      // biblioteka przerwań pin change
#include <Wire.h>
#include <DS1307RTC.h>         // biblioteki do obsługi zegara RTC
#include <Time.h>              // biblioteki do obsługi zegara RTC
#include <EEPROM.h>            // Needed to access the eeprom read write functions
#include <LiquidCrystal_I2C.h> // biblioteka I2C dla LCD
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Ustawienie adresu ukladu na 0x27

const short int PHOTORES_PIN=A0;     //pin, pod który podłączony jest fotorezystor
const short int LIGHT_PIN=5;         //pin, pod który podłączona jest baza tranzystora NPN
const short int HOUR_PIN=3;          //pin, pod który podłączony jest woltomierz pokazujący godzinę
const short int MIN_PIN=6;           //pin, pod który podłączony jest woltomierz pokazujący minuty
const short int BUT1_PIN=13;         //przycisk +1H
const short int BUT2_PIN=12;         //przycisk -1H
const short int BUT3_PIN=11;         //przycisk +1M
const short int BUT4_PIN=10;         //przycisk -1M
const short int BUT5_PIN=9;          //przycisk włączenia trybu zmiany godziny
short int DAYLIGHT_VALUE;            //do jakiego odczytu traktujemy jako "dzień" - czyli nie zapalamy diod
short int NIGHTLIGHT_VALUE;          //jaką wartość minimalną osiąga odczyt - wartość w całkowicie ciemnym pomieszczeniu
short int NIGHTSTART;
short int NIGHTSTOP;
bool change_time=false;              //flaga trybu zmiany czasu
bool updatetoRTC=false;              //flaga przesłania nowej godziny do zegara
bool getfromRTC=false;               //flaga odczytania godziny z zegara do zmiennej
short int h_temp=0;                  //tymczasowa godzina - do trybu zmiany czasu
short int m_temp=0;                  //tymczasowa minuta - do trybu zmiany czasu
bool sendtime=false;
tmElements_t prev_tm;

//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF); 
  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);
  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

void wait(int miliseconds)
{//funkcja do debounce, nie korzystająca z przerwań
  for(int i=0;i<miliseconds;i++)
  {
    delayMicroseconds(1000);
  }
}

void enabletimechange()
{//obsługa przycisku 5
  wait(50);//debounce
  if(change_time)
  {//jeżeli jesteśmy w trybie zmiany godziny
    change_time=false;  //wyłącz tryb zmiany godziny
    digitalWrite(LIGHT_PIN,HIGH);
    updatetoRTC=true;   //ustaw flagę sygnalizującą wgranie nowej godziny do układu
  }
  else
  {//jeżeli nie jesteśmy w trybie zmiany godziny
    change_time=true;  //włącz tryb zmiany godziny
    getfromRTC=true;   //ustaw flagę sygnalizującą konieczność odczytania godziny z układu do zmiennej lokalnej
  }  
}

void plushour()
{//obsługa przycisku 1
  wait(50);//debounce 
  if(change_time)
  {//jesteśmy w trybie zmiany czasu
    h_temp++;    //zmień godzinę
    if(h_temp>23)
    {//zapewnia zapętlenie, 22<->23<->0<->1
      h_temp=0;  
    }
  }
}

void minushour()
{//obsługa przycisku 2
  wait(50);//debounce 
  if(change_time)
  {//jesteśmy w trybie zmiany czasu
    h_temp--;    //zmień godzinę
    if(h_temp<0)
    {//zapewnia zapętlenie, 22<->23<->0<->1
      h_temp=23;  
    }
  }
}

void plusminute()
{//obsługa przycisku 3
  wait(50);//debounce 
  if(change_time)
  {//jesteśmy w trybie zmiany czasu
    m_temp++;    //zmień godzinę
    if(m_temp>59)
    {//zapewnia zapętlenie, 58<->59<->0<->1
      m_temp=0;  
    }
  }
}

void minusminute()
{//obsługa przycisku 4
  wait(50);//debounce 
  if(change_time)
  {//jesteśmy w trybie zmiany czasu
    m_temp--;    //zmień godzinę
    if(m_temp<0)
    {//zapewnia zapętlenie, 58<->59<->0<->1
      m_temp=59;  
    }
  }
}

int SerialRead4Int() 
{
  char temp;
  short int val;
  while(Serial.available()==0)
  {}
  temp=Serial.read();
  if((temp>='0')&&(temp<='9'))
  {
    val=temp-'0';
    val=val*10;
  }
  else
  {
    return -1;  
  }
  while(Serial.available()==0)
  {}
  temp=Serial.read();
  if((temp>='0')&&(temp<='9'))
  {
    val+=(temp-'0');
    val=val*10;
  }
  else
  {
    return -1;  
  }
  while(Serial.available()==0)
  {}
  temp=Serial.read();
  if((temp>='0')&&(temp<='9'))
  {
    val+=(temp-'0');
    val=val*10;
  }
  else
  {
    return -1;  
  }
  while(Serial.available()==0)
  {}
  temp=Serial.read();
  if((temp>='0')&&(temp<='9'))
  {
    val+=(temp-'0');
  }
  else
  {
    return -1;  
  }
  return val;
}

tmElements_t SerialReadTime() 
{
  int temp,temp2;
  tmElements_t out;
  while(Serial.available()==0)
  {}
  temp=Serial.read();
  if((temp>='0')&&(temp<='9'))
  {
    temp2=(temp-'0')*10;
    temp=0;  
  }
  while(Serial.available()==0)
  {}
  temp=Serial.read();
  if((temp>='0')&&(temp<='9'))
  {
    temp2+=temp-'0';
    temp=0;  
  }
  out.Hour=temp2;
  while(Serial.available()==0)
  {}
  temp=Serial.read();
  if((temp>='0')&&(temp<='9'))
  {
    temp2=(temp-'0')*10;
    temp=0;  
  }
  while(Serial.available()==0)
  {}
  temp=Serial.read();
  if((temp>='0')&&(temp<='9'))
  {
    temp2+=temp-'0';
    temp=0;  
  }
  out.Minute=temp2;
  while(Serial.available()==0)
  {}
  temp=Serial.read();
  if((temp>='0')&&(temp<='9'))
  {
    temp2=(temp-'0')*10;
    temp=0;  
  }
  while(Serial.available()==0)
  {}
  temp=Serial.read();
  if((temp>='0')&&(temp<='9'))
  {
    temp2+=temp-'0';
    temp=0;  
  }
  out.Second=temp2;
  return out;
}

void setup()
{//deklaracje pinów, podpięcia przerwań pod piny przycisków
  DAYLIGHT_VALUE=EEPROMReadInt(0);
  if((DAYLIGHT_VALUE==65535)||(DAYLIGHT_VALUE==-1))
  {
    DAYLIGHT_VALUE=850; 
    EEPROMWriteInt(0, DAYLIGHT_VALUE);
  }
  NIGHTLIGHT_VALUE=EEPROMReadInt(2);
  if((NIGHTLIGHT_VALUE==65535)||(NIGHTLIGHT_VALUE==-1))
  {
    NIGHTLIGHT_VALUE=600;
    EEPROMWriteInt(2, NIGHTLIGHT_VALUE);    
  }
  NIGHTSTART=EEPROMReadInt(4);
  if((NIGHTSTART==65535)||(NIGHTSTART==-1))
  {
    NIGHTSTART=2200;
    EEPROMWriteInt(4, NIGHTSTART);
  }
  NIGHTSTOP=EEPROMReadInt(6);
  if((NIGHTSTOP==65535)||(NIGHTSTOP==-1))
  {
    NIGHTSTOP=700;
    EEPROMWriteInt(6, NIGHTSTOP);    
  }
  pinMode(PHOTORES_PIN,INPUT);
  pinMode(LIGHT_PIN,OUTPUT);
  pinMode(HOUR_PIN,OUTPUT);
  pinMode(MIN_PIN,OUTPUT);
  pinMode(BUT1_PIN,INPUT_PULLUP);
  PCintPort::attachInterrupt(BUT1_PIN, &plushour, FALLING);
  pinMode(BUT2_PIN,INPUT_PULLUP);
  PCintPort::attachInterrupt(BUT2_PIN, &minushour, FALLING);
  pinMode(BUT3_PIN,INPUT_PULLUP);
  PCintPort::attachInterrupt(BUT3_PIN, &plusminute, FALLING);
  pinMode(BUT4_PIN,INPUT_PULLUP);
  PCintPort::attachInterrupt(BUT4_PIN, &minusminute, FALLING);
  pinMode(BUT5_PIN,INPUT_PULLUP);
  PCintPort::attachInterrupt(BUT5_PIN, &enabletimechange, FALLING);
  digitalWrite(LIGHT_PIN,HIGH);
  Serial.begin(9600);

  lcd.begin(16,2);       // Inicjalizacja LCD 2x16 
  lcd.backlight();       // załączenie podwietlenia 
  lcd.setCursor(0,0);    // Ustawienie kursora w pozycji 0,0 (pierwszy wiersz, pierwsza kolumna)
  lcd.print("Inicjalizacja...");
  lcd.setCursor(0,1);    // Ustawienie kursora w pozycji 0,0 (drugi wiersz, pierwsza kolumna)
  lcd.print("www.baur.pl");
  delay(2000);
  lcd.noBacklight();     // wyłączenie podwietlenia 
  lcd.clear();
}

void loop()
{
  if(sendtime)
  {
    Serial.println(4);
    tmElements_t tm;
    if (RTC.read(tm))
    {
      Serial.println(tm.Hour);
      Serial.println(tm.Minute);   
      Serial.println(tm.Second);
    }
    else
    {
      Serial.println(-1);
      Serial.println(-1);
      Serial.println(-1);
    }
    delay(100);  
  }
  if(getfromRTC)
  {//jeżeli należy pobrać godzinę z układu do zmiennej lokalnej
    getfromRTC=false;  //reset flagi
    tmElements_t tm;
    RTC.read(tm);      //pobranie godziny
    h_temp=tm.Hour;
    m_temp=tm.Minute;  //podstawienie jej do zmiennych lokalnych
  }
  if(updatetoRTC)
  {//jeżeli należy wysłać nową godzinę do układu
    updatetoRTC=false;  //reset flagi
    tmElements_t tm;
    tm.Hour=h_temp;
    tm.Minute=m_temp;   //przygotowanie danych do wysłania
    RTC.write(tm);      //wysłanie danych do układu
    lcd.clear();        //wyczyszczenie wyświetlacza LCD
  }
  if(change_time)
  {//jesteśmy w trybie zmiany czasu
    digitalWrite(LIGHT_PIN,HIGH); //zapal diodę - sygnalizacja trybu zmiany godziny
    show(h_temp,m_temp); // wyświetl tymczasową godzinę
    lcd.backlight();     // załączenie podwietlenia 
    lcd.setCursor(0,1);  // Ustawienie kursora w pozycji 0,0 (pierwszy wiersz, pierwsza kolumna)
    print2digits(h_temp);// Wyświenie nastawianej godziny
    lcd.write(':');
    print2digits(m_temp);// Wyświetlenie nastawianej minuty
    lcd.write(" SET CLOCK");
  }
  else
  {//jeżeli jesteśmy w trybie wyświetlania czasu
    if(Serial.available()>0)
    {
      int recived=Serial.read();
      switch(recived)
      {
      case '1':
        {
          Serial.println(1);
          Serial.println(DAYLIGHT_VALUE);  
          Serial.println(NIGHTLIGHT_VALUE);
          Serial.println(NIGHTSTART);
          Serial.println(NIGHTSTOP);
        }
        break;
      case '2':
        {
          Serial.println(2);
          Serial.println(analogRead(PHOTORES_PIN));
        }
        break;
      case '3':
        {
          while(Serial.available()==0)
          {}
          DAYLIGHT_VALUE=SerialRead4Int();
          if(DAYLIGHT_VALUE!=EEPROMReadInt(0))
          {
            EEPROMWriteInt(0, DAYLIGHT_VALUE);  
          }
          NIGHTLIGHT_VALUE=SerialRead4Int();
          if(NIGHTLIGHT_VALUE!=EEPROMReadInt(2))
          {
            EEPROMWriteInt(2, NIGHTLIGHT_VALUE);  
          }
          NIGHTSTART=SerialRead4Int();
          if(NIGHTSTART!=EEPROMReadInt(4))
          {
            EEPROMWriteInt(4, NIGHTSTART);  
          }
          NIGHTSTOP=SerialRead4Int();          
          if(NIGHTSTOP!=EEPROMReadInt(6))
          {
            EEPROMWriteInt(6, NIGHTSTOP);  
          }
        }
        break;
      case '4':
        {
          sendtime=!sendtime;
        }
        break;
      case '5':
        {
          while(Serial.available()==0)
          {}
          tmElements_t tm=SerialReadTime();
          RTC.write(tm);
        }
        break;
      } 
    }
    update();            //wyświetl aktualną godzinę
    setlight();          //dostosuj podświetlenie
  }
}

void update(void)
{//funkcja pobiera godzinę z RTC i przekazuje ją do funkcji show do wyświetlenia
  tmElements_t tm;
  if (RTC.read(tm))
  {
    if(tm.Minute!=prev_tm.Minute)
    {
      show(tm.Hour,tm.Minute); 
      prev_tm=tm;
    }
  }
  //lcd.noBacklight();   // wyłączenie podwietlenia
  setLCDlight();         // załącz podświetlenie LCD zgodnie z harmonogramem godzinowym
  lcd.setCursor(4,0);    // Ustawienie kursora w pozycji 0,0 (pierwszy wiersz, pierwsza kolumna)
  print2digits(tm.Hour);
  lcd.write(':');
  print2digits(tm.Minute);
  lcd.write(':');
  print2digits(tm.Second);
}

void show(int h,int m)
{//funkcja wyświetlająca godzinę
  h=h%12;
  h=h*10;
  h=h+(map(m,0,60,0,10));
  h-=5;
  if(h<0)
  {
    h+=5;  
    h=abs(h);
    h+=115;
  }
  analogWrite(HOUR_PIN,map(h,0,120,0,255));
  analogWrite(MIN_PIN,map(m,0,60,0,255));
}

void setLCDlight(void)
{//funkcja ustalająca podświetlenie LCD
  tmElements_t tm;
  lcd.noBacklight();
  if (RTC.read(tm))
  {
    int akt=((tm.Hour)*100+tm.Minute);
    if((akt<NIGHTSTART)&&(akt>NIGHTSTOP))
    {
      lcd.backlight();  
    }
  }
}
  
void setlight(void)
{//funkcja ustalająca podświetlenie zegarów
  tmElements_t tm;
  bool podswietl=false;
  if (RTC.read(tm))
  {
    int akt=((tm.Hour)*100+tm.Minute);
    if((akt<NIGHTSTART)&&(akt>NIGHTSTOP))
    {
      podswietl=true;  
    }
  }
  int light=analogRead(PHOTORES_PIN);
  if((light<DAYLIGHT_VALUE)&&(podswietl))
  {//jeżeli odczytana wartość jest poniżej progu zapalającego diody - wylicz wartość PWM i podaj ją na diody
    if(light<=NIGHTLIGHT_VALUE)
    {
      digitalWrite(LIGHT_PIN,HIGH);
    }
    else
    {
      analogWrite(LIGHT_PIN,map(light,NIGHTLIGHT_VALUE,DAYLIGHT_VALUE,0,220));
    }
  }
  else
  {//jeżeli wartość jest powyżej progu - wyłącz diody
    digitalWrite(LIGHT_PIN,LOW);  
  }
}

// Zamiana wartości na liczbę dwucyfrową
void print2digits(int number) {
  if (number >= 0 && number < 10) {
    lcd.write('0');
  }
  lcd.print(number);
}
