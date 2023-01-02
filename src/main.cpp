#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <BME280I2C.h>
#include <string>
#include <stdio.h>
#include <cmath>
#include <iostream>



// ZMIENNE

// Kontroler PID
class PID
{
public:
  // Kp -  proportional gain
  // Ki -  Integral gain
  // Kd -  derivative gain
  // dt -  loop interval time
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable    
  PID(  double dt, 
        double max, double min, double Kp, double Kd, double Ki);
  ~PID();
  double calculate(double setpoint, double pv);
  void reset();
  double get_Kp() { return _Kp;  }
  double get_Kd() { return _Kd; }
  double get_Ki() { return _Ki; }
  void wydruki_ON() { _drukuj = true; }
  void wydruki_OFF() { _drukuj = false; }
protected:
  double _dt;
  double _max_val;
  double _min_val;
  double _Kp;
  double _Kd;
  double _Ki;
  double _pre_error;
  double _integral;
  // double _lastT;
  bool _drukuj;
};



/**
 * Implementation
 */
PID::PID(
   double dt, 
   double max, double min, double Kp, double Kd, double Ki) :
   _dt(dt),
  _max_val(max),
  _min_val(min),
  _Kp(Kp),
  _Kd(Kd),
  _Ki(Ki),
  _pre_error(0),
  _integral(0)
  //_lastT(0)
{
  wydruki_ON();
}

void PID::reset()
{
   _pre_error = 0;
   _integral = 0;   
}

double PID::calculate(double setpoint, double pv)
{
  // double t = millis();
  double dt = _dt / 1000; 

  // _lastT = t;
  // Calculate error
  double error = setpoint - pv;

  // Proportional term
  double Pout = _Kp * error;

  // Integral term
  _integral += error * dt;
  double Iout = _Ki * _integral;

  // Derivative term
  double derivative = (error - _pre_error) / dt;
  double Dout = _Kd * derivative;

  // Calculate total output
  double output = Pout + Iout + Dout;  
  double output_org = output;

  // Restrict to max/min
  if (output > _max_val)
    output = _max_val;
  else if (output < _min_val)
    output = _min_val;

  // Save error to previous error
  _pre_error = error;

  if ( _drukuj)
  {
    Serial.printf("calc: Kp=%.4f, Kd=%.4f, Ki=%.4f, err=%.2f => %.2f (%.2f)\n", _Kp, _Kd, _Ki, error, output, output_org);
  }
  return output;
}

PID::~PID()
{
}

// Kontroler PID z fuzzy logic gain scheduling
class FuzzyPID : public PID
{
  public:
    FuzzyPID(double dt, double max, double min, double Kp, double Kd, double Ki);
    double calculate(double setpoint, double pv);
    void gain_scheduling(double error, double d_error );
    
  private:
    // funkcje przynaleznosci i dla error i delta-error:
    enum E_Member {
      E_Negative_Big,
      E_Negative_Medium,
      E_Negative_Small,
      E_Zero,
      E_Positive_Small,
      E_Positive_Medium,
      E_Positive_Big      
    };
     const char *Nazwa_E_Member(E_Member e) {
      const char *n = "Nieznana";
      switch (e)
      {
        case E_Negative_Big:
        n = "E_Negative_Big";
        break;
        case E_Negative_Medium:
        n = "E_Negative_Medium";
        break;
        case E_Negative_Small:
        n = "E_Negative_Small";
        break;
        case E_Zero:
        n = "E_Zero";
        break;  
        case E_Positive_Small:
        n = "E_Positive_Small";
        break;
        case E_Positive_Medium:
        n = "E_Positive_Medium";
        break;
        case E_Positive_Big:
        n = "E_Positive_Big";
        break;
      }
      return n;
    };

    // funkcje przynaleznosci i dla K
    enum K_Member {
      K_Small,
      K_Big
    };
    const char* Nazwa_K_Member(K_Member k)
    {
      const char *n = "Nieznanan";
      switch (k) 
      {
        case K_Big:
        n = "Big";
        break;
        case K_Small:
        n = "Small";
        break;
      }
      return n;
    }
    // funkcje przynaleznosci dla alfa, gdzie Ki = sqrt(Kp) / alpha * Kd
    enum A_Member {
      A_Small = 2,
      A_Medium_Small = 3,
      A_Medium = 4,
      A_Big = 5
    };
    static K_Member Kp_Rules[E_Positive_Big + 1][E_Positive_Big + 1];    
    static K_Member Kd_Rules[E_Positive_Big + 1][E_Positive_Big + 1];
    static A_Member Alpha_Rules[E_Positive_Big + 1][E_Positive_Big + 1];

    double _max_error; // maksymalny error, do normalizacji wartosci err do uzycia w regulach
    double _max_d_error; // maksymalny przrost bledu
    //bool  _pierwsze_calc;

    double _Kp_max;
    double _Kp_min;
    double _Kd_max;
    double _Kd_min;

    bool _pierwsze_wywolanie;
    
    E_Member fuzify_err(double e);
    double defuzzify_K( K_Member m );
    double defuzzify_Alpha(A_Member a);
   
};


FuzzyPID::K_Member FuzzyPID:: Kp_Rules[E_Positive_Big + 1][E_Positive_Big + 1] = {
   //          E_NB,     E_NM,    E_NS,    E_Zero, E_PS,    E_PM,    E_PB       
   /* NB */  { K_Big,   K_Big,   K_Big,   K_Big,  K_Big,   K_Big,   K_Big   },
   /* NM */  { K_Small, K_Big,   K_Big,   K_Big,  K_Big,   K_Big,   K_Small },
   /* NS */  { K_Small, K_Small, K_Big,   K_Big,  K_Big,   K_Small, K_Small },
   /* Z */   { K_Small, K_Small, K_Small, K_Big,  K_Small, K_Small, K_Small },
   /* PS */  { K_Small, K_Small, K_Big,   K_Big,  K_Big,   K_Small, K_Small },
   /* PM */  { K_Small, K_Big,   K_Big,   K_Big,  K_Big,   K_Big,   K_Small },
   /* PB */  { K_Big,   K_Big,   K_Big,   K_Big,  K_Big,   K_Big,   K_Big   }
};


FuzzyPID::K_Member FuzzyPID::Kd_Rules[E_Positive_Big + 1][E_Positive_Big + 1] = {
      { K_Big,   K_Big,   K_Big,   K_Big,  K_Big,   K_Big,   K_Big   },
      { K_Small, K_Big,   K_Big,   K_Big,  K_Big,   K_Big,   K_Small },
      { K_Small, K_Small, K_Big,   K_Big,  K_Big,   K_Small, K_Small },
      { K_Small, K_Small, K_Small, K_Big,  K_Small, K_Small, K_Small },
      { K_Small, K_Small, K_Big,   K_Big,  K_Big,   K_Small, K_Small },
      { K_Small, K_Big,   K_Big,   K_Big,  K_Big,   K_Big,   K_Small },
      { K_Big,   K_Big,   K_Big,   K_Big,  K_Big,   K_Big,   K_Big   }
};

FuzzyPID::A_Member FuzzyPID::Alpha_Rules[E_Positive_Big + 1][E_Positive_Big + 1] = {
  { A_Small,        A_Small,        A_Small,        A_Small,        A_Small,        A_Small,        A_Small        },
  { A_Medium_Small, A_Medium_Small, A_Small,        A_Small,        A_Small,        A_Medium_Small, A_Medium_Small },
  { A_Medium,       A_Medium_Small, A_Medium_Small, A_Small,        A_Medium_Small, A_Medium_Small, A_Medium       },
  { A_Big,          A_Medium,       A_Medium_Small, A_Medium_Small, A_Medium_Small, A_Medium,       A_Big          },
  { A_Medium,       A_Medium_Small, A_Medium_Small, A_Small,        A_Medium_Small, A_Medium_Small, A_Medium       },
  { A_Medium_Small, A_Medium_Small, A_Small,        A_Small,        A_Small,        A_Medium_Small, A_Medium_Small },
  { A_Small,        A_Small,        A_Small,        A_Small,        A_Small,        A_Small,        A_Small        }
};


FuzzyPID::FuzzyPID(double dt, double max, double min, double Kp, double Kd, double Ki) :
  PID( dt, max, min, Kp, Kd, Ki )
{
  // Wzmocnienie przy ktorym kontroler P utrzymuje oscylacje
  //  Z artykulu:
  //double Ku = 2.4495;
  //_Kp_max = Ku * 0.6;
  //_Kp_min = Ku * 0.32;
  //_Kd_max = Ku * 0.15;
  //_Kd_min = Ku * 0.08;

  // Z doswiadczenia ze zwyklym PID:
  // Kp = 2,4495,                 
  // Kd = 20.0471,                 
  // Ki = 0.0599
  _Kp_max = 4.7;             
  _Kp_min = 1.0;             
  _Kd_max = 0.00; //40.0;            
  _Kd_min = 0.00;  //0.001;        
  
  _max_d_error = 0;
  _max_error = 0;  
  _pierwsze_wywolanie = true;

  // wydruki z klasy bazowej wylaczamy:
  wydruki_OFF();
}

double FuzzyPID::calculate(double setpoint, double pv)
{
  double error = setpoint - pv;
  double d_error = error - _pre_error;
  double output = PID::calculate(setpoint, pv);
    
  
  // pierwsze wywolanie
  if ( _max_error == 0 )
  {
    _max_error = abs(error);
    // Poczatkowo bardzo maly przyrost bledu: ustali sie przy nastepnych odczytach
    _max_d_error = 0.000001;
  }
  else {
    // Serial.printf("MAX_ERROR: %f. MAX_DERROR: %f\n", _max_error, _max_d_error);            
        
    // wolamy gain_scheduling jezeli juz ustalony err i d_err    
    if ( abs(error) > abs(_max_error) ) {
         _max_error = abs(error);
    }
    if ( abs(d_error) > abs(_max_d_error))
    {
      _max_d_error = abs(d_error);
    }    
     gain_scheduling( error, d_error); // pre_error liczony przez calcuate
  }

  return output;
}


void FuzzyPID::gain_scheduling(double error, double d_error)
{
  // normalizujemy blad i pochodna, zeby byly w przedziale <-1,1>
  double e  = error / _max_error;
  double de = d_error / _max_d_error;
  E_Member m_e = fuzify_err(e);
  E_Member m_de = fuzify_err(de);
  
  K_Member m_Kp = Kp_Rules[m_e][m_de];
  K_Member m_Kd = Kd_Rules[m_e][m_de];
  A_Member m_Alpha = Alpha_Rules[m_e][m_de];  

   double prev_Kp = _Kp;
   double prev_Kd = _Kd;
   double prev_Ki = _Ki;

  _Kp = (_Kp_max - _Kp_min) * defuzzify_K(m_Kp) + _Kp_min;
  _Kd = (_Kd_max - _Kd_min) * defuzzify_K(m_Kd) + _Kd_min;
  _Ki = sqrt(_Kp) / (defuzzify_Alpha(m_Alpha) * _Kd);

  if ( abs(_Kp - prev_Kp) > 0.0001 ||
       abs(_Kd - prev_Kd) > 0.0001 ||
       abs(_Kd - prev_Kd) > 0.0001 )
  {
    Serial.printf("err: %.4f, %s, d_err: %.4f, %s => m_Kp: %s, m_Kd: %s\n", e, Nazwa_E_Member(m_e), de, Nazwa_E_Member(m_de), Nazwa_K_Member(m_Kp), Nazwa_K_Member(m_Kd));
    Serial.printf("----- ZMIANA: Kp: %.4f -> %.4f, Kd: %.4f -> %.4f, Ki: %.4f -> %.4f\n", prev_Kp, _Kp, prev_Kd, _Kd, prev_Ki, _Ki);
  }
}

FuzzyPID::E_Member FuzzyPID::fuzify_err(double e)
{
  E_Member m;
  double krok = 2.0 / 7.0;
  if ( e >= -1.0 && e <= (-1.0 + krok) )
  {
    m = E_Negative_Big;
  }
  else if ( e > -1.0 + krok && e <= (-1.0 + (2.0 * krok)) )
  {
    m = E_Negative_Medium;
  }
  else if ( e > -1.0 + (2.0*krok) && e <= (-1.0 + (3.0 * krok)) )
  {
    m = E_Negative_Small;
  }
  else if ( e > -1.0 + (3.0*krok) && e <= (-1.0 + (4.0 * krok)) )
  {
    m = E_Zero;
  }
  else if ( e > -1.0 + (4.0*krok) && e <= (-1.0 + (5.0 * krok)) )
  {
    m = E_Positive_Small;
  }
  else if ( e > -1.0 + (5.0*krok) && e <= (-1.0 + (6.0 * krok)) )
  { 
    m = E_Positive_Medium;
  }
  else if ( e > -1.0 + (6.0*krok) && e <= (-1.0 + (7.0 * krok)) )
  {
    m = E_Positive_Big;
  }
  return m;
}

double FuzzyPID::defuzzify_K( K_Member m )
{
  double k = 0;
  if (m == K_Big )
  {
    k = 0.45;
  }
  else // small
  {
    k = 0.05;
  }
  return k;
}

double FuzzyPID::defuzzify_Alpha( A_Member m )
{
  // ??
  return (double) m;
}




//sterowanie
double temp_zadana = 24.0;
int tryb = 0;
int duty_heatbed = 0;
int duty_peltier = 0;
const int freq = 1000; //czestotliwosc
const int kanal1 = 0; //kanaly (mozna dac pare pinow w jeden kanal i beda mialy identyczny sygnal)
const int kanal2 = 1;
const int resolution = 9;   
//rozdzielczosc 8bit -> 0 do 255

// regulator pid
// 
const double min_punkt_grzania = 200.0;
const double max_punkt_grzania = 450.0;
const double min_punkt_chlodzenia = 0.0;

// const double pid_range = 255;
// Okres probkowania regulatora [ms]
const double T = 1000;
static PID pid (T, 
                max_punkt_grzania - min_punkt_grzania,  // max
                0,  // -(max_punkt_grzania - min_punkt_grzania),   // min
                2.4495,       //1.4289,            // Kp
                20.0471,      //0.00,              // Kd
                0.0599);      //0.0032);           // Ki

static FuzzyPID fpid(T,
                max_punkt_grzania - min_punkt_grzania,  // max
                0,  // -(max_punkt_grzania - min_punkt_grzania),   // min
                1.4289,    //2.4495,                 // Kp
                0.00,      //20.0471,                 // Kd
                0.0032);   //0.0599);                // Ki


//sciezki do plikow
const char *ssidPath = "/ssid.txt";
const char *passPath = "/pass.txt";
const char *ipPath = "/ip.txt";
const char *gatewayPath = "/gateway.txt";
const char *indexPath = "/index.html";
const char *stylePath = "/style.css";
const char *wifimanagerPath = "/wifimanager.html";
const char *MQTTPath = "/mqttip.txt";
std::vector<const char*> filePaths = {ssidPath,passPath,ipPath,gatewayPath,indexPath,stylePath,wifimanagerPath,MQTTPath};

// WiFi
bool connected_once = false;

String ssid;
String pass;
String ip;
String gateway;

IPAddress localIP;
// IPAddress localIP(192, 168, 1, 200);
IPAddress localGateway;
// IPAddress localGateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

//TODO przerobic to na funkcje
unsigned long currentMillis;
unsigned long previousMillis = 0;
unsigned long currentMillis2;
unsigned long previousMillis2 = 0;
const long wifi_timeoutMillis = 10000;
unsigned long previousRegMillis = 0;

AsyncWebServer server(80);
//MQTT
WiFiClient wifi_client;
PubSubClient client(wifi_client);

String mqtt_server;

//czujnik
bool status_bme280;
BME280I2C czujnik;

//FUNKCJE

float readTemperature()
{
  return czujnik.temp();
  //return rand() % 20 + 10;
}

//przypisuje odpowiednie piny PWM do odpowiednich kanalow
void init_PWM()
{
  uint32_t f;
  f = ledcSetup(kanal1, freq, resolution);
  Serial.printf("Kanal1: freq %d\n", f);
  f = ledcSetup(kanal2, freq, resolution);
  Serial.printf("Kanal2: freq %d\n", f);
  ledcAttachPin(22, kanal1);
  ledcAttachPin(23, kanal2);
}

// zwroc zakres PWM odpowiadajacy wyjsciu regulatora
// Mapuj wartosc pid_output do zakresu pwm_range_begin-pwm_range_end proporcjonalnie do zakresu 0-max_pid_output/
// 
// PID-range jest wyliczany dynamicznie do najwiekszej wartosci zwracanej przez regulator. Osobno dla dodatnich i ujemnych.
// 
// Np.:
// pwm_range_start = 100
// pwm_range_end   = 200
// pid_output = 150
// max_pid_output = 1000
// 
// Mapowanie przedzialow:
// 
//           PID                                PWM
//
//   min_pid_output : 0                   
// 
//                                             pwm_range_begin = 100
//    
//                                            
//                                             d = ? (odpowiednik 'p')  
//                p : 500                     
//                                             pwm_range_end   = 190
//                                          
//
//
//   max_pid_output : 1000
//
//   Proporcja:
// 
//       p            max_pid_output - min_pid_output
//       _   =    _____________________________________
//       d            pwm_range_end - pwm_range_begin
//
//                 (pwm_range_end - pwm_range_begin) * P
//       d =      _______________________________________
//                    max_pid_output - min_pid_output
// 
//               190 - 100
//       d =  _______________  * 500 =  49.8 
//               1000 - 0
//
//   d trzeba jeszcze "przesunac" o pwm_range_begin, czyli ostatecznie d = d + pwm_range_begin  = 149.9  - czyli jest w polowie zakresu, tak jak PID-output.
//
double pid_output_to_duty(double pid_output, double max_pid_output, double pwm_range_begin, double pwm_range_end )
{  
  double duty = ((pwm_range_end - pwm_range_begin) / max_pid_output) * pid_output;
  duty += pwm_range_begin;
  if ( duty > pwm_range_end )
  {
     duty = pwm_range_end;
  }
   static int print = 0;
  print++;
  if ( print < 100 ) 
  {
    // printf("pid output: %.2f\t(zakres: 0 -\t%.2f) => duty: %.2f\t(zakres: %.2f\t- %.2f)\n", pid_output, max_pid_output, duty, pwm_range_begin, pwm_range_end);
  }  
  return duty;
}

//TODO algorytmy sterowania temperatura
//readTemperature(); - temperatura obecna (float)
//temp_zadana - temperatura zadana
//duty_heatbed, duty_peltier - wypelnienia 
bool sterowanie(int &tryb, int &duty_heatbed, int &duty_peltier)
{
  float temp_aktualna = readTemperature();
    
  //Praca ręczna
  if(tryb == 0)
  {
    //ZABEZPIECZENIE
    if (duty_heatbed < 0) duty_heatbed = 0;
    if (duty_heatbed > pow(2,resolution)-1) duty_heatbed = pow(2,resolution)-1;
    if (duty_peltier < 0) duty_peltier = 0;
    if (duty_peltier > pow(2,resolution)-1) duty_peltier = pow(2,resolution)-1;

    ledcWrite(kanal1, duty_heatbed);
    ledcWrite(kanal2, duty_peltier);

    return true;
  }
  //Dwustanowe
  if(tryb == 1)
  {
    float histereza = 1.0;
    
    if (temp_aktualna < temp_zadana - (histereza/2.0))
    {

      duty_heatbed = 355;
      duty_peltier = 0;
      //Serial.println("Grzeje!");
    }
    if(temp_aktualna > temp_zadana + (histereza/2.0))
    {
      duty_heatbed = 0;
      duty_peltier = 511;
      //Serial.println("Chlodze!");
    }
    
    
    ledcWrite(kanal1, duty_heatbed);
    ledcWrite(kanal2, duty_peltier);
    return true;
  }
  //PID
  if(tryb == 2)
  {    
    double duty = 0;
    // przyrost wartosci:    
    double pid_out = pid.calculate(temp_zadana, temp_aktualna);    

    duty_heatbed = (int) pid_out + min_punkt_grzania;                   
    if ( duty_heatbed < 0 )
    {
       duty_heatbed = 0;
    }    
    static int opusc = 0;
    opusc++;
    if ( true || opusc == 10 )
    {      
       Serial.printf("pid-out: %f, DH:%d, DP: %d\n",  pid_out, duty_heatbed, duty_peltier );   
       opusc=0;
    }    
    ledcWrite(kanal1, duty_heatbed);
    ledcWrite(kanal2, duty_peltier);
    return true;
  }
  //Logika rozmyta
  if(tryb == 3)
  {
    static int last_hb_duty = 0;
    double duty = 0;
    // przyrost wartosci:    
    double pid_out = fpid.calculate(temp_zadana, temp_aktualna);    

    duty_heatbed = (int) pid_out + min_punkt_grzania;                   
    if ( duty_heatbed < 0 )
    {
       duty_heatbed = 0;
    }    
    static int opusc = 0;
    opusc++;
    if ( true || opusc == 10 )
    {
      if ( last_hb_duty != duty_heatbed )
      {
         Serial.printf("fpid-out: %f, DH (zmiana):%d, DP: %d\n",  pid_out, duty_heatbed, duty_peltier );   
      }
      last_hb_duty = duty_heatbed;
      opusc=0;
    }    
    ledcWrite(kanal1, duty_heatbed);
    ledcWrite(kanal2, duty_peltier);
    return true;
  }
  Serial.println("ERROR - Wrong mode set.");
  return false;
}

//sprawdza czy czujnik jest polaczony i odpowiada
bool init_bme280()
{
    status_bme280 = czujnik.begin();//0x76
    if(!status_bme280)
    {
      Serial.println("Could not find a valid BME280 sensor.");
      return false;
    }
  return true;
}
void error_LED(int mrugniecia, int opoznienie = 500)
{
  for (int i = 0; i < mrugniecia; i++)
  {
    delay(opoznienie);
    digitalWrite(2, HIGH);
    delay(opoznienie);
    digitalWrite(2, LOW);
  }
}

bool init_SPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    error_LED(2); // LED mrugnie 2 razy
    return false;
  }
  Serial.println("SPIFFS OK");
  return true;
}

//sprawdza czy istnieją podane pliki
bool files_exist(fs::FS &fs,std::vector<const char*> filePaths)
{
  //z jakiegos powodu w tej funkcji nie dzialaja seriale bez dodania delay(5000) przed wywolaniem funkcji
  Serial.printf("%i files needed\n",filePaths.size());
  const char * sciezka = "";
  for(int i =0; i < filePaths.size(); i++)
  {
    delay(500);
    sciezka = filePaths[i];
    Serial.printf("Checking file: %s\r\n", sciezka);
    File file = fs.open(sciezka);
    if(!file || file.isDirectory() || !file.size() > 0)
    {
      Serial.print("Failed to open file");
      // jak dodadza wsparcie w SPIFFS dla katalogow to wtedy ta funkcja bedzie rozrozniac
      // if(file.isDirectory()) Serial.print(", path leads to a directory.");
     return false;
    }
  }
  Serial.println("All necessary files exist");
  return true;
}

// wczytuje pierwsza linie
String read_line(fs::FS &fs, const char *sciezka)
{
  Serial.printf("Reading file: %s\r\n", sciezka);

  File file = fs.open(sciezka);
  if (!file || file.isDirectory())
  {
    Serial.print("Failed to open file");
    // jak dodadza wsparcie w SPIFFS dla katalogow to wtedy ta funkcja bedzie rozrozniac
    // if(file.isDirectory()) Serial.print(", path leads to a directory.");
    return String();
  }

  String zawartosc;
  while (file.available())
  {
    zawartosc = file.readStringUntil('\n');
    break;
  }
  return zawartosc;
}

// zapisuje pierwsza linie
bool write_line(fs::FS &fs, const char *sciezka, const char *line)
{
  Serial.printf("Writing file: %s\r\n", sciezka);

  File file = fs.open(sciezka, FILE_WRITE);
  if (!file || file.isDirectory())
  {
    Serial.print("Failed to open file");
    // jak dodadza wsparcie w SPIFFS dla katalogow to wtedy ta funkcja bedzie rozrozniac
    // if(file.isDirectory()) Serial.print(", path leads to a directory.");
    return false;
  }
  if (file.print(line))
  {
    Serial.println("Success - file written");
    return true;
  }
  else
  {
    Serial.println("Write failed");
    return false;
  }
}

// proba polaczenia z danych ze zmiennych
bool init_WiFi()
{
  ssid = read_line(SPIFFS, ssidPath);
  pass = read_line(SPIFFS, passPath);
  ip = read_line(SPIFFS, ipPath);
  gateway = read_line(SPIFFS, gatewayPath);

  if (ssid == "" || ip == "")
  {
    Serial.println("Undefined SSID or IP");
    error_LED(3, 200);
    return false;
  }

  WiFi.mode(WIFI_STA);
  localIP.fromString(ip.c_str());
  localGateway.fromString(gateway.c_str());

  if (!WiFi.config(localIP, localGateway, subnet))
  {
    Serial.println("Error configuring static IP address");
    error_LED(3, 200);
    return false;
  }
  WiFi.begin(ssid.c_str(), pass.c_str());
  Serial.println("Connecting to WiFi...");

  currentMillis = millis();
  previousMillis = currentMillis;
  previousRegMillis = currentMillis;

  while (WiFi.status() != WL_CONNECTED)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis >= wifi_timeoutMillis)
    {
      Serial.println("Failed to connect in time.");
      error_LED(3, 200);
      return false;
    }
  }
  Serial.printf("Connected to: %s, as ", ssid.c_str());
  Serial.println(WiFi.localIP());
  return true;
}

// funkcja sprawdzajaca czy jest polaczenie, jesli nie ma, probuje polaczyc sie z ostatnia siecia
void keep_WiFi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Lost connection, trying to reconnect to the last known WiFi.");
    Serial.println(ssid);
    Serial.println(pass);
    Serial.println(ip);
    Serial.println(gateway);
    init_WiFi();
  }
  /*else
    Serial.print("\nWiFi OK: ");
    Serial.print(ssid);*/
}

// funkcja uruchamiająca AP
bool init_AP()
{
  const char *nazwa = "ESP32 Access Point";
  const char *haslo = "123456789";

  Serial.print("Setting up AP (Access Point): ");
  Serial.print(nazwa);
  if (!WiFi.softAP(nazwa, haslo))
  {
    Serial.println("Error creating AP");
    error_LED(4);
    return false;
  }
  Serial.print("\nWiFi-Manager address: ");
  Serial.print(WiFi.softAPIP());

  return true;
}

// WebServer
bool init_WebServer()
{
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", "text/html", false); });

  server.serveStatic("/", SPIFFS, "/");
  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              int params = request->params();
              for(int i=0;i<params;i++)
              {
                AsyncWebParameter* p = request->getParam(i);
                if(p->isPost())
                {
                  if (p->name() == "mqtt") 
                  {
                    mqtt_server = p->value().c_str();
                    Serial.print("MQTT Broker IP to: ");
                    Serial.println(mqtt_server);
                    // Write file to save value
                    write_line(SPIFFS, MQTTPath, mqtt_server.c_str());
                  }
                }
              }
              request->send(200, "text/plain", "MQTT Broker IP set to: " + mqtt_server);
            });
  server.begin();
  return true;
}

// WebServer dla WifiManagera
bool init_APWebServer()
{
  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/wifimanager.html", "text/html"); });
  server.serveStatic("/", SPIFFS, "/");
  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
            {
      int params = request->params();
      for(int i=0;i<params;i++)
      {
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost())
        {
          // HTTP POST ssid value
          if (p->name() == "ssid") 
          {
            ssid = p->value().c_str();
            Serial.print("SSID set to: ");
            Serial.println(ssid);
            // Write file to save value
            write_line(SPIFFS, ssidPath, ssid.c_str());
          }
          // HTTP POST pass value
          if (p->name() == "pass") 
          {
            pass = p->value().c_str();
            Serial.print("Password set to: ");
            Serial.println(pass);
            // Write file to save value
            write_line(SPIFFS, passPath, pass.c_str());
          }
          // HTTP POST ip value
          if (p->name() == "ip") 
          {
            ip = p->value().c_str();
            Serial.print("IP Address set to: ");
            Serial.println(ip);
            // Write file to save value
            write_line(SPIFFS, ipPath, ip.c_str());
          }
          // HTTP POST gateway value
          if (p->name() == "gateway") 
          {
            gateway = p->value().c_str();
            Serial.print("Gateway set to: ");
            Serial.println(gateway);
            // Write file to save value
            write_line(SPIFFS, gatewayPath, gateway.c_str());
          }
          if (p->name() == "mqtt") 
          {
            mqtt_server = p->value().c_str();
            Serial.print("MQTT Broker IP to: ");
            Serial.println(mqtt_server);
            // Write file to save value
            write_line(SPIFFS, MQTTPath, mqtt_server.c_str());
          }
          //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
      }
      request->send(200, "text/plain", "Done. ESP will restart, connect to your router and go to IP address: " + ip);
      delay(3000);
      ESP.restart(); });
  server.begin();
  return true;
}

//funkcja wywolywana po otrzymaniu wiadomosci od brokera MQTT
void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
  String temp_message = "";
  //wyswietlanie otrzymanej wiadomosci
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    temp_message += (char)payload[i];
  }
  Serial.println();

  //obsluga polecen
  if(String(topic) == "esp32/output/mode")
  {
    if(temp_message == "0") tryb = 0; //reczny
    if(temp_message == "1") tryb = 1; //dwustanowe
    if(temp_message == "2")  {
       tryb = 2; //PID
       pid.reset();
    }
    if(temp_message == "3") tryb = 3; //rozmyta
  }

  //odczyt pwm
  if(String(topic) == "esp32/output/pwm1")
  {
    duty_heatbed = temp_message.toInt();
  }
  if(String(topic) == "esp32/output/pwm2")
  {
    duty_peltier = temp_message.toInt();
  }
}

//obsluga polaczenia z brokerem MQTT
bool init_MQTT()
{
  mqtt_server = read_line(SPIFFS, MQTTPath);
  
  if (mqtt_server == "")
  {
    Serial.println("Undefined MQTT Broker IP");
    error_LED(4, 200);
    return false;
  }
  //MQTT
  client.setServer(mqtt_server.c_str(), 1883);

  //SUS - potrafila sie sypac bez przyczyny (naprawde nie wiem czemu teraz nagle dziala a wczesniej nie)
  client.setCallback(mqtt_callback);
  return true;
}

//funckja do ponownego łączenia z brokerem MQTT
void mqtt_reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output/#");
      //client.subscribe("esp32/output/mode");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(18,19); //i2c
  init_PWM();
  // zainicjuj i ustaw pin do LED
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  init_SPIFFS();
  delay(5000);
  if(!files_exist(SPIFFS,filePaths))
  {
    error_LED(2);
    Serial.println("Key files missing.");
  }
  else Serial.println("Files OK");
  init_bme280();
  if (init_WiFi())
  {
    //udane połączenie - tu incjować rzeczy które wymagają wifi
    init_WebServer();
    connected_once = true;
    init_MQTT();
  }
  else
  {
    init_AP();
    init_APWebServer();
  }

}

double acs_to_amp(double RawValue)
{   
   int mVperAmp = 100; // use 100 for 20A Module and 66 for 30A Module   
   int ACSoffset = 2500; 
   double Voltage = 0;
   double Amps = 0;
   Voltage = (RawValue / 1024.0) * 3300; // Gets you mV
   Amps = ((Voltage - ACSoffset) / mVperAmp);
   return Amps;
}

void loop()
{  
  if (connected_once)
  {
    keep_WiFi();
    //MQTT 
    if (!client.connected()) {
      mqtt_reconnect();
    }
    client.loop();
    currentMillis = millis();
    currentMillis2 = millis();
    if(currentMillis - previousMillis > 50)
    {
      char payload[8];
      dtostrf(duty_heatbed,1,1,payload);
      client.publish("esp32/input/pwm1",payload);
      dtostrf(duty_peltier,1,1,payload);
      client.publish("esp32/input/pwm2",payload);
      previousMillis = currentMillis;
    }
    
    //wysylanie danych do serwera
    if(currentMillis2 - previousMillis2 > 1000)
    {
      StaticJsonDocument<300> json;
      json["czas"] = millis();
      json["temperatura"] = readTemperature();
      json["temp_zadana"] = temp_zadana;
      json["id_sterowania"] = tryb;
      // json["pobor"] = analogRead(34) + analogRead(35); //odczytane wartosci z pinow 34 i 35
      json["pobor"] =  acs_to_amp(analogRead(34));  // heatbed przeliczony
      json["heatbed"] = duty_heatbed;
      json["peltier"] = duty_peltier;
      char buffer[300];
      serializeJson(json, buffer);
      client.publish("esp32/input/temperature",buffer);
      previousMillis2 = currentMillis2;
    }
    if(currentMillis - previousRegMillis >= T)
    {
      previousRegMillis = currentMillis;
      if(!sterowanie(tryb, duty_heatbed, duty_peltier)) Serial.println("Wrong mode selected!");
    }
  }
  else
  {
    error_LED(3);
  }
  
  
  //delay(3000);


}