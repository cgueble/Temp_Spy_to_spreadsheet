#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <PubSubClient.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
//#include <Codes_CGU.h>// contient toutes les variables avec les loggins et les codes
#include <C:\Users\chris\OneDrive\Documents\Arduino\libraries\Codes_CGU.h>

//****************Configuration pour Debug****************
const boolean SERIAL_PORT_LOG_ENABLE = true; //true pour avoir la console active et false pour la desactiver ; il faut la desactiver pour l'application car meme pour que "verrou"

//Declaration des compteurs


//Connexion a Internet
/*const char* ssid0 = "YourSSID_0";//type your ssid
  const char* password0 = "YourPWD_0";//type your password
  const char* ssid1 = "YourSSID_1";//type your ssid
  const char* password1 = "YourPWD_1";//type your password
  const char* ssid2 = "YourSSID_2";//type your ssid
  const char* password2 = "YourPWD_2";//type your password
*/
String CurentSSID;
String CurentSSIDTry;
String StringDate = "no date";
String StringTime = "no time";
String Release_Date = "12-02-2018";

//Definition des Outputs
int SensorPowerSupply = 14; //GPOI14 = D5

//Variables de gestion de l'etat de la porte et des mails
const unsigned int localPort = 2390;

boolean Update_needed = true;
long rssi; //pour mesure RSSI

//variables pour le gestion de la date et du temps
const char* ntpServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
unsigned long secsSince1900 = 0;
const unsigned long seventyYears = 2208988800UL;
int Hour;
int Minute;
int Second;
int Day;
int DayofWeek; // Sunday is day 0
int Month;     // Jan is month 0
int Year;      // the Year minus 1900
int WeekDay;

//Variables pour la mesure de temperature
float  TempInt = 0;
float  TempExt = 0;

//Variable pour la gestion de thingspeak
const char* server = "mqtt.thingspeak.com";// Define the ThingSpeak MQTT broker

const unsigned long postingInterval = 1 * 60 * 1L * 1000L;// post data every 1 min
const unsigned long RegularpostingInterval = 10 * 60 * 1L * 1000L;// post data every 10 min
const unsigned long whatchDogValue = 60 * 60 * 1000L;// WhatchDog Value 60min

unsigned long lastConnectionTime = 0; // track the last connection time
unsigned long lastPostTime = 0;// track the last Post time

time_t epoch = 0; // contient
IPAddress ip;
IPAddress timeServerIP;

WiFiClient client;  // Initialize the Wifi client library.
WiFiUDP udp; //A UDP instance to let us send and receive packets over UDP
PubSubClient mqttClient(client); // Initialize the PuBSubClient library

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2 //GPIO2 = D4
#define TEMPERATURE_PRECISION 9

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
//DeviceAddress insideThermometer, outsideThermometer;

DeviceAddress insideThermometer    = { 0x28, 0x0C, 0x01, 0x07, 0xA8, 0x9E, 0x01, 0xBA };//280C0107A89E01BA
DeviceAddress outsideThermometer   = { 0x28, 0xFF, 0xD6, 0x1B, 0xC4, 0x17, 0x05, 0x56 };//28FFD61BC4170556

//************************Debut de setup*************************
void setup() {
  delay(5000);
  if (SERIAL_PORT_LOG_ENABLE) {
    Serial.begin(115200);
    Serial.print("debut de setup. SERIAL_PORT_LOG_ENABLE= ");
    Serial.println(SERIAL_PORT_LOG_ENABLE);
  }
 //Set up PIN in Output
  pinMode(SensorPowerSupply, OUTPUT);
  digitalWrite(SensorPowerSupply, HIGH);// on commence par couper l'alim des capteurs
  
  // Start up the library
  sensors.begin();

  InitSensors();

  //Set up PIN in INPUT


  //Set up PIN in OUTPUT


  while ((WiFi.status() != WL_CONNECTED)) {
    WifiConnexionManager();//Recherche reseau wifi + connexion
    if (SERIAL_PORT_LOG_ENABLE) {
      Serial.print(".");
      delay(1000);
      Serial.print(".");
      delay(1000);
      Serial.print(".\r");
      delay(1000);
      Serial.println("   \r");
    }
  }//Loop connexion until Wifi Connetion is OK

  if (SERIAL_PORT_LOG_ENABLE) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("Use this URL to connect: ");
    Serial.print("http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
  }

  udp.begin(localPort);// Start UDP port for connexion to NTP server

  if (SERIAL_PORT_LOG_ENABLE) {
    Serial.println("Starting UDP");
    Serial.print("Local port: ");
    Serial.println(udp.localPort());
  }

  GetTimeByUDP();
  UpdateTime();
  mqttClient.setServer(server, 1883);// Set the MQTT broker details

  //wifi_status_led_uninstall();

  if (SERIAL_PORT_LOG_ENABLE) {
    Serial.println("End of Setup");
  }
  TemperatureMeasurment();
}// fin set up
//******************end of setup*************************************
//*******************************************************************



//******************start of loop************************************
void loop() {

  //LoopLog
  if(SERIAL_PORT_LOG_ENABLE){
  Serial.print(".\r");
  delay(100);
  Serial.print("0\r\n");
  }

  WifiConnectOwner((char*)ssid1,(char*)password1);
  UpdateTime();
  rssi = WiFi.RSSI();
  if (Year < 2016) { // si l'heure n'est pas configuree C est a dire si l'annee n'est pas bonne, alors on recupere l'heure sur le serveur NTP
    GetTimeByUDP();
    UpdateTime();
  }

  if (WiFi.status() != WL_CONNECTED) {// si la connexion WIFI est perdue, alors on relance le setup
    WiFi.disconnect();
    if (SERIAL_PORT_LOG_ENABLE) {
      Serial.println("Connexion lost ");
    }
    setup();
  }

 // if (millis() - lastPostTime > RegularpostingInterval)
 // {
    digitalWrite(SensorPowerSupply, HIGH);
    Serial.println("power suply sensors ");
    delay(1000);
    InitSensors();
    delay(60000);
    TemperatureMeasurment();
    lastPostTime = millis();
    Update_needed = true;
    digitalWrite(SensorPowerSupply, LOW);
    Serial.println("turn off sensors");
 // }  

    // si un changement de status est detecte, on essaye de mettre a jour le statut thingspeak
  if (Update_needed == true) {
    mqttpublishtry();
    gotoSleep(10);//time to go to sleep in minutes
  }
 


}//*****************END OF LOOP *******************

void gotoSleep(long int SleepDurationMinutes){
  Serial.print("Go to sleep for = ");
  Serial.print(SleepDurationMinutes);
  Serial.println(" minutes");
  SleepDurationMinutes = SleepDurationMinutes * 60 * 1000 * 1000;
  ESP.deepSleep(SleepDurationMinutes);
}

void UpdateTime() {
  StringDate = ""; // reset StringDate value
  StringTime = ""; // reset StringTime value
  Hour = hour();
  Minute = minute();
  Second = second();
  Year = year();
  Month = month();
  Day = day();
  WeekDay = weekday();
//2014-12-31 23:59:59
  if ((Month <= 3) || (Month > 10)) { // on est un mois d'hiver
    if ((Month == 3) && (Day - WeekDay > 24)) { //on est dans le dernier mois d'hiver et aprÃ¨s le dernier dimanche
      Hour = Hour + 2;
      //StringTime += " UTC+2: ";
      if (Hour >= 22){
        Day = Day + 1;
      }
    }
    else { // toute la periode hivernale
      Hour = Hour + 1;
      //StringTime += " UTC+1: ";
        if (Hour >= 23){
        Day = Day + 1;
      }
    }
  }
  else { // on est un mois d'aout
    if ((Month == 10) && (Day - WeekDay > 24)) { //on est dans le dernier mois d'aout et apres le dernier dimanche
      Hour = Hour + 1;
      //StringTime += " UTC+1: ";
      if (Hour >= 23){
        Day = Day + 1;
      }
    }
    else { // toute la periode estivale
      Hour = Hour + 2;
      //StringTime += " UTC+2: ";
      if (Hour >= 22){
        Day = Day + 1;
      }
    }
  }

  //StringDate += "Date: ";
  StringDate += Year;
  StringDate += "-";
    if (Month < 10) {
    StringDate += "-0";
  }
  else {
    StringDate += "/";
  }
  StringDate += Month;
  StringDate += "/";
  
  if (Day < 10) {
    StringDate += "0";
  }
  StringDate += Day;

  

  if (Hour < 10) {
    StringTime += "0";
  }
  StringTime += Hour;
  if (Minute < 10) {
    StringTime += ":0";
  }
  else {
    StringTime += ":";
  }
  StringTime += Minute;
  if (Second < 10) {
    StringTime += ":0";
  }
  else {
    StringTime += ":";
  }
  StringTime += Second;
  if (SERIAL_PORT_LOG_ENABLE) {
    //Serial.print("valeure de StringDate ");
    //Serial.println(StringDate);
    //Serial.print("valeure de StringTime ");
    //Serial.println(StringTime);
  }
}//End of Update Time


void GetTimeByUDP() {
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);


  int cb = udp.parsePacket();

  if (!cb) {
    if (SERIAL_PORT_LOG_ENABLE) {
      Serial.println("no packet yet");
    }
  }
  else {
    if (SERIAL_PORT_LOG_ENABLE) {
      Serial.print("packet received, length=");
      Serial.println(cb);
    }

    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    secsSince1900 = highWord << 16 | lowWord;
    if (SERIAL_PORT_LOG_ENABLE) {
      Serial.print("Seconds since Jan 1 1900 = " );
      Serial.println(secsSince1900);
    }

    // now convert NTP time into everyday time:
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    epoch = secsSince1900 - seventyYears;
    setTime(epoch);

  }
}// end of GetTimeByUDP()



// send an NTP request to the time server at the given address
//unsigned long sendNTPpacket(IPAddress& address)
void sendNTPpacket(IPAddress& address)
{
  if (SERIAL_PORT_LOG_ENABLE) {
    Serial.println("sending NTP packet...");
  }
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void reconnect()
{
  int connect_counter = 20;
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Connect to the MQTT broker
    if (mqttClient.connect(ThingspeakClientID, ThingspeakUserID, ThingspeakUserPwd))
    {
      Serial.println("connected");
    } else
    {
      connect_counter = connect_counter - 1;
      if ((mqttClient.state() == -3) or (connect_counter < 0))
      {
        Serial.println("-3 : MQTT_CONNECTION_LOST - the network connection was broken ... launch setup");
        setup();
      }
      Serial.print("failed, rc=");
      // Print to know why the connection failed
      // See http://pubsubclient.knolleary.net/api.html#state for the failure code and its reason
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying to connect again
      delay(5000);
    }
  }
}

void mqttpublish() {
  String data = String("field1=" + String(TempInt, DEC) + "&field2=" + String(TempExt, DEC) + "&field3=" + String(rssi, DEC));
  // Get the data string length
  int length = data.length();
  char msgBuffer[length];
  char* PublishCmd = "channels/1418978/publish/W5I8JXWTKFVJPEJY";
  // Convert data string to character buffer
  data.toCharArray(msgBuffer, length + 1);
  Serial.println(msgBuffer);
  // Publish data to ThingSpeak. Replace <YOUR-CHANNEL-ID> with your channel ID and <YOUR-CHANNEL-WRITEAPIKEY> with your write API key
  if(mqttClient.publish(PublishCmd, msgBuffer)){
    // note the last connection time
    lastConnectionTime = millis();
  }
}

void mqttpublishtry() {
  if (!mqttClient.connected())
  {
    reconnect();
  }
  // Call the loop continuously to establish connection to the server
  mqttClient.loop();
  // If interval time has passed since the last connection, Publish data to ThingSpeak
//  if (millis() - lastConnectionTime > postingInterval)
//  {
    mqttpublish();
    Update_needed = false;
    delay(1000);
//  }
//    if (SERIAL_PORT_LOG_ENABLE) {
//    Serial.print("Update if (millis() - lastConnectionTime) = ");
//    Serial.print(millis() - lastConnectionTime );
//    Serial.print(" > postingInterval = ");
//    Serial.println(postingInterval );
    
//    }
}

void  WifiConnexionManager() {
  if (SERIAL_PORT_LOG_ENABLE) {
  Serial.println("Begin of WifiConnexionManager");
  }
  int numberOfNetworks = WiFi.scanNetworks();

    if (SERIAL_PORT_LOG_ENABLE) {
      for (int i = 0; i < numberOfNetworks; i++) {
          Serial.print(WiFi.RSSI(i));
          Serial.print("dBm  ");
          Serial.print(" ");
          Serial.print("Network name: ");
          Serial.println(WiFi.SSID(i));
      }
    }
for (int i = 0; i < numberOfNetworks; i++) {    
    if (WiFi.SSID(i) == ssid1) {
      WiFi.begin(ssid1, password1);
      CurentSSID = WiFi.SSID(i);
      WaitConnexion();
      if (SERIAL_PORT_LOG_ENABLE) {
        Serial.print("Wifi trouvé. temtative de connexion a: ");
        Serial.println(ssid1);
      }
      break;
    }

    if (WiFi.SSID(i) == ssid2) {
      WiFi.begin(ssid2, password2);
      CurentSSID = WiFi.SSID(i);
      WaitConnexion();
      if (SERIAL_PORT_LOG_ENABLE) {
        Serial.print("Wifi trouvé. temtative de connexion a: ");
        Serial.println(ssid2);
      }
      break;
    }

    if (WiFi.SSID(i) == ssid0) {
      WiFi.begin(ssid0, password0);
      CurentSSID = WiFi.SSID(i);
      WaitConnexion();
      if (SERIAL_PORT_LOG_ENABLE) {
        Serial.print("Wifi trouvé. temtative de connexion a: ");
        Serial.println(ssid0);
        Serial.print("Wifi.status(): ");
        Serial.println(WiFi.status());
      }
      break;
    }
  }//End FOR
  Serial.println("End of WifiConnexionManager");
}//end WifiConnexionManager


void  WifiConnectOwner(char* SSIDowner_fct, char* passwordowner_fct) {
  if (SERIAL_PORT_LOG_ENABLE) {
      Serial.println("Begin of WifiConnectOwner");
      Serial.println("Already connected to :");
      Serial.println(WiFi.SSID());
  }
  int numberOfNetworks = WiFi.scanNetworks();
  for (int i = 0; i < numberOfNetworks; i++) {
    if ((WiFi.SSID(i) == SSIDowner_fct) && ((String)SSIDowner_fct != CurentSSID)) {
      WiFi.disconnect();
      WiFi.begin(SSIDowner_fct, passwordowner_fct);
      
      if (SERIAL_PORT_LOG_ENABLE) {
        Serial.print("Wifi Owner trouve . temtative de connexion a: ");
        Serial.println(SSIDowner_fct);
      }
      WaitConnexion();
      if ((WiFi.status() == WL_CONNECTED))
      {
        CurentSSID = WiFi.SSID(i);
        if (SERIAL_PORT_LOG_ENABLE) {
          Serial.print("connecte a: ");
          Serial.println(SSIDowner_fct);
        }
        
      }
      break;
    }
  }//End FOR
Serial.println("End of WifiConnectOwner");  
}//end WifiConnectOwner

void WaitConnexion(){
      int cpt=10;
      if (SERIAL_PORT_LOG_ENABLE) {
      Serial.println("Begin of WaitConnexion");
      Serial.print("cpt= ");
      Serial.print(cpt);
      Serial.print("   WiFi.status= ");
      Serial.print(WiFi.status());
      Serial.print("   WL_CONNECTED= ");
      Serial.println(WL_CONNECTED);
      Serial.print("test = (cpt >= 0) && ((WiFi.status() != WL_CONNECTED)) =  ");
      Serial.println((cpt > 0) && ((WiFi.status() != WL_CONNECTED)));
      }
      while((cpt > 0) && ((WiFi.status() != WL_CONNECTED))){
      if (SERIAL_PORT_LOG_ENABLE) {
      Serial.print("cpt= ");
      Serial.print(cpt);
      Serial.print("   WiFi.status= ");
      Serial.print(WiFi.status());
      Serial.print("   WL_CONNECTED= ");
      Serial.println(WL_CONNECTED);
      Serial.print("test = (cpt >= 0) && ((WiFi.status() != WL_CONNECTED)) =  ");
      Serial.println((cpt > 0) && ((WiFi.status() != WL_CONNECTED)));
      }
      cpt = cpt -1;
      delay(1000);
      }
      if (SERIAL_PORT_LOG_ENABLE) {
      Serial.print("cpt= ");
      Serial.print(cpt);
      Serial.print("   WiFi.status= ");
      Serial.print(WiFi.status());
      Serial.print("   WL_CONNECTED= ");
      Serial.println(WL_CONNECTED);
      Serial.print("test = (cpt >= 0) && ((WiFi.status() != WL_CONNECTED)) =  ");
      Serial.println((cpt > 0) && ((WiFi.status() != WL_CONNECTED)));
      }
      if ((WiFi.status()== WL_CONNECTED))
      {
          if (SERIAL_PORT_LOG_ENABLE) {
             Serial.print("connecte a: ");
             Serial.println(WiFi.SSID());
            }
      } 
      if (SERIAL_PORT_LOG_ENABLE) {
      Serial.println("End of WaitConnexion");  
      }   
}// end Waitconnexion

void TemperatureMeasurment(){
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures();
  Serial.println("DONE");

  // print the device information
  printData(insideThermometer);
  printData(outsideThermometer);
    
  TempInt = sensors.getTempCByIndex(0)+6;// Offset de -6°C sur le thermometre "interieure" => compensation de +6
  TempExt = sensors.getTempCByIndex(1);
  
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}

void InitSensors()
{
  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addresses and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).
  //
  // method 1: by index
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0");
  if (!sensors.getAddress(outsideThermometer, 1)) Serial.println("Unable to find address for Device 1");

  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices,
  // or you have already retrieved all of them. It might be a good idea to
  // check the CRC to make sure you didn't get garbage. The order is
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to insideThermometer
  //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");
  // assigns the seconds address found to outsideThermometer
  //if (!oneWire.search(outsideThermometer)) Serial.println("Unable to find address for outsideThermometer");

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

  Serial.print("Device 1 Address: ");
  printAddress(outsideThermometer);
  Serial.println();

  // set the resolution to 9 bit per device
  sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC);
  Serial.println();

  Serial.print("Device 1 Resolution: ");
  Serial.print(sensors.getResolution(outsideThermometer), DEC);
  Serial.println();
  }
