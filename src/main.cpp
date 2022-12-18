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

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv );

    private:
        double _dt;
        double _max_val;
        double _min_val;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
        double _lastT;
};

class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double dt, double max, double min, double Kp, double Kd, double Ki );

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
        ~PID();

    private:
        PIDImpl *pimpl;
};




PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max_val(max),
    _min_val(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
  _lastT = millis();
}

double PIDImpl::calculate( double setpoint, double pv )
{
    double t = millis();
    _dt = t - _lastT;
    _lastT = t;

    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    double output_computed = output;
    // Restrict to max/min
    if( output > _max_val )
        output = _max_val;
    else if( output < _min_val )
        output = _min_val;

    // Save error to previous error
    _pre_error = error;
    //_lastT = t; 
    static int opusc = 0;
    opusc++;
    if ( opusc == 1000 )
    {
       Serial.printf("PID:calc: dt: %.3f, S:%.3f, PV:%.3f, out:%.3f, D:%.3f\n", _dt, setpoint, pv, output_computed, (output / _max_val) * 255 );
       opusc=0;
    }
    

    return output;
}

PIDImpl::~PIDImpl()
{
}

//sterowanie
double temp_zadana = 25.0;
int tryb = 0;
int duty_heatbed = 0;
int duty_peltier = 0;
const int freq = 1000; //czestotliwosc
const int kanal1 = 0; //kanaly (mozna dac pare pinow w jeden kanal i beda mialy identyczny sygnal)
const int kanal2 = 1;
const int resolution = 8; //rozdzielczosc 8bit -> 0 do 255
// regulator pid
// 
const double pid_range = 10000;
// Okres probkowania regulatora [ms]
const double T = 100;
static PID pid (T/1000,   // dt 
                pid_range,  // max
               -pid_range,   // min
                10,   // Kp
                0.0, // Kd
                0.01);  // Ki

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
  ledcSetup(kanal1, freq, resolution);
  ledcSetup(kanal2, freq, resolution);
  ledcAttachPin(22, kanal1);
  ledcAttachPin(23, kanal2);
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

      duty_heatbed = 190;
      duty_peltier = 0;
      //Serial.println("Grzeje!");
    }
    if(temp_aktualna > temp_zadana + (histereza/2.0))
    {
      duty_heatbed = 0;
      duty_peltier = 255;
      //Serial.println("Chlodze!");
    }
    
    
    ledcWrite(kanal1, duty_heatbed);
    ledcWrite(kanal2, duty_peltier);
    return true;
  }
  //PID
  if(tryb == 2)
  {
    // przyrost wartosci:    
    double duty = (pid.calculate(temp_zadana, temp_aktualna) / pid_range) * 255;
    // const int min_duty = 190;
    // const int max_duty = 240;
    if ( duty < 0 )
    {    
      duty_heatbed = 0;
      duty_peltier = (int) -duty;
    }
    else {
      duty_heatbed = duty;
      duty_peltier = (int) 0;
    }
    
    // Serial.printf("DH:%d, DP: %d\n", duty_heatbed, duty_peltier );
    
    ledcWrite(kanal1, duty_heatbed);
    ledcWrite(kanal2, duty_peltier);

    return true;
  }
  //Logika rozmyta
  if(tryb == 3)
  {
    
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
    if(temp_message == "2") tryb = 2; //PID
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
      json["pobor"] = analogRead(34) + analogRead(35); //odczytane wartosci z pinow 34 i 35
      json["heatbed"] = duty_heatbed;
      json["peltier"] = duty_peltier;
      char buffer[300];
      serializeJson(json, buffer);
      client.publish("esp32/input/temperature",buffer);
      previousMillis2 = currentMillis2;
    }
    // if(currentMillis - previousRegMillis >= T)
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