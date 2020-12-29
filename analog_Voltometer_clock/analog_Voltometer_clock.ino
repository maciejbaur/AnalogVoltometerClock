// to indicate that port b will not be used for pin change interrupts
#define NO_PORTB_PINCHANGES    
// to indicate that port c will not be used for pin change interrupts
#define NO_PORTC_PINCHANGES    

//biblioteka przerwań pin change
#include <PinChangeInt.h>      

//biblioteki do obsługi zegara RTC
#include <Wire.h>
#include <DS1307RTC.h>
#include <Time.h>              

//biblioteki wyświetlacza LCD
#include <LiquidCrystal_I2C.h> // biblioteka I2C dla LCD
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Ustawienie adresu ukladu na 0x27

const int PHOTORES_PIN=A0;     //pin, pod który podłączony jest fotorezystor
const int LIGHT_PIN=5;         //pin, pod który podłączona jest baza tranzystora PNP
const int HOUR_PIN=3;          //pin, pod który podłączony jest woltomierz pokazujący godzinę
const int MIN_PIN=6;           //pin, pod który podłączony jest woltomierz pokazujący minuty
const int BUT1_PIN=0;          //przycisk +1H
const int BUT2_PIN=1;          //przycisk -1H
const int BUT3_PIN=2;          //przycisk +1M
const int BUT4_PIN=4;          //przycisk -1M
const int BUT5_PIN=7;          //przycisk włączenia trybu zmiany godziny
const int DAYLIGHT_VALUE=600;  //do jakiego odczytu traktujemy jako "dzień" - czyli nie zapalamy diod
const int NIGHTLIGHT_VALUE=850;//jaką wartość minimalną osiąga odczyt - wartość w całkowicie ciemnym pomieszczeniu
bool change_time=false;        //flaga trybu zmiany czasu
bool updatetoRTC=false;        //flaga przesłania nowej godziny do zegara
bool getfromRTC=false;         //flaga odczytania godziny z zegara do zmiennej
int h_temp=0;                  //tymczasowa godzina - do trybu zmiany czasu
int m_temp=0;                  //tymczasowa minuta - do trybu zmiany czasu

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

void setup()
{//deklaracje pinów, podpięcia przerwań pod piny przycisków
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
    show(h_temp,m_temp); //wyświetl tymczasową godzinę

    lcd.backlight();    // załączenie podwietlenia 
    lcd.setCursor(0,1); // Ustawienie kursora w pozycji 0,0 (pierwszy wiersz, pierwsza kolumna)
    print2digits(h_temp);
    lcd.write(':');
    print2digits(m_temp);
    lcd.write(" SET CLOCK");
  }
  else
  {//jeżeli jesteśmy w trybie wyświetlania czasu
    update();            //wyświetl aktualną godzinę
    setlight();          //dostosuj podświetlenie
    int count=1;
    for(int i=0;i<count;i++)
    {//odczekaj 15 sekund, ale przerwij czekanie jeżeli w trakcie tych 15 sekund układ wejdzie w tryb zmiany godziny
      if(change_time)
      {
        break;  
      }
      else
      {
        delay(1000);  
      }
    }
  }    
}

void update(void)
{//funkcja pobiera godzinę z RTC i przekazuje ją do funkcji show do wyświetlenia
  tmElements_t tm;
  if (RTC.read(tm))
  {
    show(tm.Hour,tm.Minute);

    lcd.noBacklight();     // wyłączenie podwietlenia 
    lcd.setCursor(0,0); // Ustawienie kursora w pozycji 0,0 (pierwszy wiersz, pierwsza kolumna)
    print2digits(tm.Hour);
    lcd.write(':');
    print2digits(tm.Minute);
    lcd.write(':');
    print2digits(tm.Second);

    lcd.setCursor(0,1); //Ustawienie kursora w pozycji 0,1 (drugi wiersz, pierwsza kolumna)
    lcd.write("www.baur.pl");
  }
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

void setlight(void)
{//funkcja ustalająca podświetlenie zegarów
  int light=analogRead(PHOTORES_PIN);
  if(light<DAYLIGHT_VALUE)
  {//jeżeli odczytana wartość jest poniżej progu zapalającego diody - wylicz wartość PWM i podaj ją na diody
    analogWrite(LIGHT_PIN,map(light,NIGHTLIGHT_VALUE,DAYLIGHT_VALUE,0,255));
  }
  else
  {//jeżeli wartość jest powyżej progu - wyłącz diody
    digitalWrite(LIGHT_PIN,LOW);  
  }
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    lcd.write('0');
  }
  lcd.print(number);
}
