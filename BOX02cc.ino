#include "FS.h"
#include "BluetoothSerial.h"
#include "ELMduino.h"
#include <string.h>
//#include <ArduinoJson.h>
#include <ArduinoHttpClient.h>
// Your GPRS credentials (leave empty, if not needed)
#include <CRC32.h>
#include "Adafruit_FONA.h"
#include <Update.h>
#include "SPIFFS.h"
#include <esp_task_wdt.h>
char inByte;
boolean obd_error_flag;      //Variable for OBD connection error
#define OBD_CMD_RETRIES 3
BluetoothSerial SerialBT;
#define ELM_PORT   SerialBT
#define DEBUG_PORT Serial2
#define REMOVE_BONDED_DEVICES 0
ELM327 myELM327;
RTC_DATA_ATTR    int indicateurrpm = 1;
RTC_DATA_ATTR    int indicateurkph = 2;
RTC_DATA_ATTR    int indicateurengine_load = 3;
RTC_DATA_ATTR    int indicateurengineCoolantTemp = 4;
RTC_DATA_ATTR    int indicateurvin = 5;
RTC_DATA_ATTR    int indicateurdistTravelWithMIL = 6;
RTC_DATA_ATTR    int indicateurniveauCarburant = 7;
RTC_DATA_ATTR    int kilometrage = 8;
RTC_DATA_ATTR    int indicateur3 = 9;
RTC_DATA_ATTR    int indicateurDTC = 10;
RTC_DATA_ATTR    int indicateurposition = 11;
RTC_DATA_ATTR    int indicateur4 = 12;
RTC_DATA_ATTR    int indicateur5 = 13;
RTC_DATA_ATTR    int indicateur6 = 14;
//________________________variables des fonctions_______________________________//


#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR  uint16_t rpm ;
RTC_DATA_ATTR  uint16_t kph ;
RTC_DATA_ATTR  float engine_load ;
RTC_DATA_ATTR float engineCoolantTemp ;
RTC_DATA_ATTR  uint16_t distTravelWithMIL ;
RTC_DATA_ATTR  int pres = 8399 ;
RTC_DATA_ATTR  int  ind2 = 22;
RTC_DATA_ATTR  int  ind3 = 0;
RTC_DATA_ATTR  int  ind4 = 10;
RTC_DATA_ATTR  int  ind5 = 9;
RTC_DATA_ATTR  float  Version_Software = 1.1;
RTC_DATA_ATTR  float Lat      = 0;
RTC_DATA_ATTR  float Long      = 0;
RTC_DATA_ATTR  String DTC1 = "";
RTC_DATA_ATTR  String  DTC2 = "";
RTC_DATA_ATTR  String  DTC3 = "";
RTC_DATA_ATTR  String  DTC4 = "";
RTC_DATA_ATTR  String  DTC5 = "";
RTC_DATA_ATTR  bool initvid = false ;
//RTC_DATA_ATTR  int8_t vin;
RTC_DATA_ATTR  uint8_t addmtemp = -40;
RTC_DATA_ATTR  uint16_t distSinceCodesCleared  ;
RTC_DATA_ATTR  uint32_t vid ;
RTC_DATA_ATTR  float accuracy = 10;
RTC_DATA_ATTR  float consomation_carburant  ;
RTC_DATA_ATTR String DTC ;
RTC_DATA_ATTR String vin ="" ;
int indicateur166 = 15;
int indicateur16 = 16;
int indicateur17 = 17;
int indicateur18 = 18;
int indicateur19 = 19;
int indicateur20 = 20;
const char hostnameOBD[] = "your server"; 
const char hostname[] = "";domain name: example.com, maker.ifttt.com, etc
const char resource[] = " ";         // resource path, for example: /post-data.php
const int  port = 8080;
const int  portOBD = 80;;
String resourceota = String (resource);
String resourceotasend = resourceota.substring(12,resourceota.length()-4);
String OTA ="";
int vin3 ;
String vin4 = "";
String vin5 = "";
String vin6 = "";
String vin7 = "";
//______________________connexion avec elm327_________________________________//
String imiobd = "";
String name = "OBDII";
String MACadd = "176:101:215:94:37:175"; // discovered by serial BT monitor (Android)
uint8_t address[6]  = {0x66, 0x1E, 0x11, 0x12, 0x19, 0xA5}; //{0x00, 0x10, 0xCC, 0x4F, 0x36, 0x03} ; //{0x66, 0x1E, 0x11, 0x12, 0x19, 0xA5};// saif 
String elmmac ="0010CC4F3603";
uint32_t support1_20 ;
uint32_t support21_40;
uint32_t support41_60;
uint32_t support61_80;
String support1 ;
String support2 ;
String support3 ;
String support4  ;
String sendall ;
//_______________________________________GSM config_________________________________________________//

const char apn[]      = "TM"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = ""; // GPRS User
const char gprsPass[] = ""; // GPRS Password
const char simPIN[]   = "";
//__________________________________configuration entre esp32 et sim800l________________________________________//
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
//________________________________________________________________________________//
#define MAX 1000
#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb
#include <Wire.h>
#include <TinyGsmClient.h>
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif  // I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0); // TinyGSM Client for Internet connection
TinyGsmClient client(modem);
//TinyGsmClient client(modem);

HttpClient http_client = HttpClient(client, hostname, port);
HttpClient http_clientOBD = HttpClient(client, hostnameOBD, portOBD);

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00
bool setPowerBoostKeepOn(int en) {
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}
//____________________________________setup_____________________________________________//
void setup()
{
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  SerialMon.begin(115200);  //Initialize serial
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  DEBUG_PORT.begin(115200);
  Serial.println("Aprés OTA");
  Serial.println("Boot number: " + String(bootCount));
  Serial.println("hello bb");
  esp_task_wdt_init(600,true);
esp_task_wdt_add(NULL);
//  elm327();
////  vin = Vin();
//vin = "5555313553441503435338373833136";
//
//  if (vin.length() > 10)
//  {
//  vin3 =vin.substring(0, 7);
//  vin4 =vin.substring(7, 14);
//  vin5 =vin.substring(14, 22);
//  vin6 =vin.substring(22, 27); 
//  vin7 =vin.substring(27, vin.length());
//  }
//  delay(5000);
//  indicateur();
//  //odo();
////  DTC =getDTCs();
//  envoie();

//  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
//  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
//                 " Seconds");
//  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
//  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");
//  Serial.println("Going to sleep now");
//  Serial.flush();
//  esp_deep_sleep_start();
}

void loop()
{
  elm327();
  delay(2000);
//  vin = Vin();
//  String vin1 ;
//  vin1 += vin ;
//  if (vin1.length() > 10)
//  {
//  vin3 =vin1.substring(0, 7).toInt();
//  vin4 =vin1.substring(7, 14).toInt();
//  vin5 =vin1.substring(14, 22).toInt();
//  vin6 =vin1.substring(22, 27).toInt(); 
//  vin7 =vin1.substring(27, vin.length()).toInt();
//  }
//vin = "55553135534E415034353D3837383F3136";
  delay(5000);
  indicateur();
  //odo();
//  DTC =getDTCs();
  envoie();
  esp_task_wdt_reset();
  delay(120000);
}
void printError()
{
  Serial.print("Received: ");
  for (byte i = 0; i < myELM327.recBytes; i++)
    Serial.write(myELM327.payload[i]);
  Serial.println();

  if (myELM327.status == ELM_SUCCESS)
    Serial.println(F("\tELM_SUCCESS"));
  else if (myELM327.status == ELM_NO_RESPONSE)
    Serial.println(F("\tERROR: ELM_NO_RESPONSE"));
  else if (myELM327.status == ELM_BUFFER_OVERFLOW)
    Serial.println(F("\tERROR: ELM_BUFFER_OVERFLOW"));
  else if (myELM327.status == ELM_UNABLE_TO_CONNECT)
    Serial.println(F("\tERROR: ELM_UNABLE_TO_CONNECT"));
  else if (myELM327.status == ELM_NO_DATA)
    Serial.println(F("\tERROR: ELM_NO_DATA"));
  else if (myELM327.status == ELM_STOPPED)
    Serial.println(F("\tERROR: ELM_STOPPED"));
  else if (myELM327.status == ELM_TIMEOUT)
    Serial.println(F("\tERROR: ELM_TIMEOUT"));
  else if (myELM327.status == ELM_TIMEOUT)
    Serial.println(F("\tERROR: ELM_GENERAL_ERROR"));



  delay(100);}

//---------------------------------test-------------------------------------------//
void  odo() 
{
  char recvChars;
  String stro;
  String code;
  boolean prompt;
  int i, retries;
  if (!(obd_error_flag)) {                                       //if no OBD connection error
    prompt = false;
    retries = 0;
    while ((!prompt) && (retries < OBD_CMD_RETRIES)) {            //while no prompt and not reached OBD cmd retries
      ELM_PORT.print("0101");                             //send OBD cmd
      ELM_PORT.print("\r");                                //send cariage return

      while (ELM_PORT.available() <= 0);                   //wait while no data from ELM
      i = 0;
      while ((ELM_PORT.available() > 0) && (!prompt)) {    //while there is data and not prompt
        recvChars = ELM_PORT.read();                        //read from elm
        stro += recvChars;
        i++;
        if (recvChars == 62) prompt = true;                        //if received char is '>' then prompt is true
      }
      //Serial.println(" ");
      retries = retries + 1;                                      //increase retries
      delay(2000);
    }
    if (retries >= OBD_CMD_RETRIES) {                             // if OBD cmd retries reached
      obd_error_flag = true;                                      // obd error flag is true
    }
 
  Serial.println("odometre");
  Serial.println(stro);

}}
//___________________________DTC___________________________________//


String  getDTCs() 
{
  char recvChars;
  String strDTC;
  String code;
  boolean prompt;
  int i, retries;
  if (!(obd_error_flag)) {                                       //if no OBD connection error
    prompt = false;
    retries = 0;
    while ((!prompt) && (retries < OBD_CMD_RETRIES)) {            //while no prompt and not reached OBD cmd retries
      ELM_PORT.print("03");                             //send OBD cmd
      ELM_PORT.print("\r");                                //send cariage return

      while (ELM_PORT.available() <= 0);                   //wait while no data from ELM
      i = 0;
      while ((ELM_PORT.available() > 0) && (!prompt)) {    //while there is data and not prompt
        recvChars = ELM_PORT.read();                        //read from elm
        strDTC += recvChars;
        i++;
        if (recvChars == 62) prompt = true;                        //if received char is '>' then prompt is true
      }
      //Serial.println(" ");
      retries = retries + 1;                                      //increase retries
      delay(2000);
    }
    if (retries >= OBD_CMD_RETRIES) {                             // if OBD cmd retries reached
      obd_error_flag = true;                                      // obd error flag is true
    }
  
  strDTC.replace(" ","");
  strDTC.remove(0,2);
  Serial.println("dtc");
  String dtc2=strDTC;
  String dtc1=strDTC;
   dtc1.substring(0,4);
   dtc2.substring(4,4);
  Serial.println(dtc1);
  Serial.println(dtc2);
return strDTC;

}
}
//________________________________vin_________________________________//

String Vin(){
  char recvChar;
  String vin ;
  boolean prompt;
  int i,retries;
   if (!(obd_error_flag)){                                        //if no OBD connection error
    prompt=false;
    retries=0;
    while((!prompt) && (retries<OBD_CMD_RETRIES)){                //while no prompt and not reached OBD cmd retries
      ELM_PORT.print("0902");                                  //send OBD cmd
      ELM_PORT.print("\r");                                  //send cariage return

      while (ELM_PORT.available() <= 0);                   //wait while no data from ELM
      i=0;
      while ((ELM_PORT.available()>0) && (!prompt)){       //while there is data and not prompt
        recvChar = ELM_PORT.read();                        //read from elm
        vin += recvChar;
        i++;
        if (recvChar==62) prompt=true;                            //if received char is '>' then prompt is true
      }
      //Serial.println(" ");
      retries=retries+1;                                          //increase retries
      delay(2000);
    }
    if (retries>=OBD_CMD_RETRIES) {                               // if OBD cmd retries reached
      obd_error_flag=true;                                        // obd error flag is true
    }
  }
       vin.replace(" ","");
       Serial.println(vin);
       String vin1 ;
       for(int i=3;i<vin.length();i++)
           {if ( vin[i]==':'){vin.remove(i-1,2);}}
       vin.remove(0,3);
       vin1=vin.substring(20,vin.length());
       Serial.println("vin aprés: ");
       Serial.println(vin1);
       String vin2 = vin1.substring(0,vin1.length()-1);
       Serial.println(vin2);

    return vin2;          
}
String hexToAscii( String hex )
{
  uint16_t len = hex.length();
  String ascii = "";
  for ( uint16_t i = 0; i < len; i += 2 )
    ascii += (char)strtol( hex.substring( i, i+2 ).c_str(), NULL, 16 );
  return ascii;
}

void elm327()
{
  //Serial.begin(115200);
  delay(10);
SerialMon.println("connecting to elm327");

  DEBUG_PORT.begin(115200);
  SerialBT.setPin("1234");
  ELM_PORT.begin("OBDII", true); 
  

  if (!ELM_PORT.connect(address))
  {
    DEBUG_PORT.println("Couldn't connect to OBD scanner - Phase 1");
   // while(1);
  }
    //myELM327.initializeELM('5',2000);

  if (!myELM327.begin(ELM_PORT))
  {
    Serial.println("Couldn't connect to OBD scanner - Phase 2");
   //return;
  }
    Serial.println("Connected to ELM327");
}
//_______________________________________________________________________________________________//

void indicateur ()
{

  delay(1000);

  //_______________________________consommation carburant________________________________________//

  float consomation_carburant = myELM327.fuelLevel();
  if (myELM327.status == ELM_SUCCESS)
  {

    Serial.print("fuel level : "); Serial.print(consomation_carburant); Serial.println("%");
  }
  else
    printError();

  //___________________________________RPM__________________________________________________//

  float tempRPM = myELM327.rpm();
  if (myELM327.status == ELM_SUCCESS)
  {
    rpm = (uint32_t)tempRPM;
    Serial.print("RPM: "); Serial.print(rpm); Serial.println("  tr/min");
  }
  else
    printError();


  //_________________________distance effectuée depuis que la MIL est allumée______________________//
  uint16_t temp_distTravelWithMIL = myELM327.distTravelWithMIL();
  if (myELM327.status == ELM_SUCCESS)
  {
    distTravelWithMIL = temp_distTravelWithMIL;
    Serial.print("distTravelWithERROR "); Serial.print(distTravelWithMIL); Serial.println("  Km");
    //la distance est codée sur 2 bits A et B 256A+B résultat entre 0 et 65,535 (sortie codée sur deux bit)
    if (distTravelWithMIL > 0)
    {
      Serial.println("un code defaut est active vous devez reparer votre vehicule");
    }
  }
  else
    printError();

  //__________________________vitesse de la véhicule  ___________________________//
  delay(2000);
  uint16_t vitesse = myELM327.kph();
  if (myELM327.status == ELM_SUCCESS)
  {
    kph = vitesse;
    Serial.print("la vitesse de la vehicule : "); Serial.print(vitesse); Serial.println("  km/h");
  }
  else
    printError();

  //________________________charge moteur____________________//
  delay(2000);
  float calculatedengineload = myELM327.engineLoad();
  if (myELM327.status == ELM_SUCCESS)
  {
    engine_load = calculatedengineload;
    Serial.println("charge moteur calculée : "); Serial.print(engine_load); Serial.println(" %");


  }
  else
  {
    printError();

  }
  //_______________Température liquide de refroidissement _______//

  delay(2000);
  float tempENGINE_COOLANT_TEMP = myELM327.engineCoolantTemp();
  if (myELM327.status == ELM_SUCCESS)
  {
    engineCoolantTemp = tempENGINE_COOLANT_TEMP;
    Serial.print("température du liquide de refroidissement du moteur: "); Serial.print(engineCoolantTemp); Serial.println("  °C");
  }
  else
    printError();

  //________________________________________________date de vidange ______________________________________________________//

  uint16_t temp_distSinceCodesCleared = myELM327.distSinceCodesCleared();
  int j = 1 ;
  if (myELM327.status == ELM_SUCCESS)
  {
    distSinceCodesCleared = temp_distSinceCodesCleared ;
    Serial.print("distTravel_since_code_cleared: "); Serial.print(distSinceCodesCleared); Serial.println("  Km");
    vid = distSinceCodesCleared + 1000 ;
    if ( vid / (j * 10000) >= 1 )
    {
      Serial.println(" vous devez effectuer immediatement votre vidange ");

      if (initvid == true)
      {
        j += 1 ;
        Serial.println("le vidange a ete effectué avec succés");
        initvid = false ;
      }
    }
    pres = j * 10000 - vid ;
    Serial.print("distance restante jusqu'au prochain vidange: "); Serial.print(pres); Serial.println("  Km");


  }

  else
    printError();
    delay(3000);
 //float fuelInjectT = myELM327.fuelInjectTiming();
 
 if (myELM327.status == ELM_SUCCESS)
  {
    //fuelInjectTime = fuelInjectT ;
    uint32_t support1_20=myELM327.supportedPIDs_1_20();
    delay(2000);
    uint32_t support21_40=myELM327.supportedPIDs_21_40();
    delay(2000);
    uint32_t support41_60 =myELM327.supportedPIDs_41_60();
    delay(2000);
    uint32_t support61_80 =myELM327.supportedPIDs_61_80();
    //Serial.print("fuelInjectTiming: "); Serial.print(fuelInjectTime); Serial.println(" ");
    Serial.print(support1_20);
    Serial.print(support21_40);
    Serial.print(support41_60);
    Serial.print(support61_80);
  }
  else
    printError();
}
void envoie() {

  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  modem.init();

  // Set GSM module baud rate and UART pins
  delay(3000);
  SerialMon.println("Initializing modem...");
  Serial.print("Waiting for network...");
  do {
    Serial.println(" fail");
  }
  while ((!modem.waitForNetwork()));
  Serial.println(" OK");
  Serial.print("Connecting to ");
  Serial.print(apn);
  do {
    Serial.println(" fail");
  }
  while ((!modem.gprsConnect(apn, gprsUser, gprsPass)));
  Serial.println(" ok");

  imiobd = modem.getIMEI();
  Serial.println(imiobd);
//    float Lat      = 0;
//    float Lon      = 0;
//    float accuracy = 0;
//  for (int8_t i = 5; i; i--)
//
//  {
//    // Try this five times:
//    SerialMon.println("Requesting current GSM location");
//    if (modem.getGsmLocation(&Lat, &Long, &accuracy))
//    {
//      SerialMon.print("Latitude:");
//      SerialMon.print(String(Lat, 8));
//      SerialMon.print("\tLongitude:");
//      SerialMon.println(String(Long, 8));
//      SerialMon.print("Accuracy:");
//      SerialMon.println(accuracy);
//      //break;
//    }
//    else
//    {
//      SerialMon.println("Couldn't get GSM location, retrying in 1s.");
//      delay(1000L);
//    }
//  }

 support1 =String(support1_20);
 support2  =String(support21_40);
 support3  =String(support41_60);
 support4  =String(support61_80);
  String donneesEnvoyees = "";
//  donneesEnvoyees =
//    String(rpm)    + "," + String(indicateurrpm) + ";"
//    + String(kph)    + "," + String(indicateurkph) + ";"
//    + String(engine_load)    + "," + String(indicateurengine_load) + ";"
//    + String(engineCoolantTemp)    + "," + String(indicateurengineCoolantTemp) + ";"
//    + String(distTravelWithMIL)    + "," + String(indicateurdistTravelWithMIL) + ";"
//    + String(pres)    + "," + String(indicateur1) + ";"
//    + String(distSinceCodesCleared)    + "," + String(indicateur2) + ";"
//    + String(accuracy)    + "," + String(indicateur3) + ";"
//    + String(DTC1) + "," + String(DTC2) + "," + String(DTC3) + "," + String(DTC4) + "," + String(DTC5) + "," + String(indicateurDTC) + ";"
//    + String(Long) + "," + String(Lat) + "," + String(indicateurposition) + ";"
//    + String(0)    + "," + String(indicateur4) + ";"
//    + String(0)    + "," + String(indicateur5) + ";"
//    + String(Version_Software)    + "," + String(indicateur6) ;

  donneesEnvoyees =
    String(rpm)    + "," + String(indicateurrpm) + ";"
    + String(kph)    + "," + String(indicateurkph) + ";"
    + String(engine_load)    + "," + String(indicateurengine_load) + ";"
    + String(engineCoolantTemp)    + "," + String(indicateurengineCoolantTemp) + ";"    
   // + String(vin3) + "," + String(vin4) + "," + String(vin5) + "," + String(vin6) + "," + String(vin7) + "," + String(indicateurvin) + ";"
    + String(distTravelWithMIL)    + "," + String(indicateurdistTravelWithMIL) + ";"
    + String(consomation_carburant)    + "," + String(indicateurniveauCarburant) + ";"
    + String(distSinceCodesCleared)    + "," + String(kilometrage) + ";"
    + String(accuracy)    + "," + String(indicateur3) + ";"
    + String(DTC1) + "," + String(DTC2) + "," + String(DTC3) + "," + String(DTC4) + "," + String(DTC5) + "," + String(indicateurDTC) + ";"
    + String(Long) + "," + String(Lat) + "," + String(indicateurposition) + ";"
    + String(pres)    + "," + String(indicateur4) + ";"
    + String(0)    + "," + String(indicateur5) + ";"
    + String(Version_Software)    + "," + String(indicateur6) + ";"
    + String(resourceotasend)    + "," + String(indicateur166)+ ";"
    + String(elmmac)    + "," + String(indicateur16)+ ";"
    + support1    + "," + String(indicateur17)+ ";"
    + String(support2)    + "," + String(indicateur18)+ ";"
    + String(support3)    + "," + String(indicateur19)+ ";"
    + String(support4)    + "," + String(indicateur20) ;  
  Serial.println("################   Début   ############################");
  sendall = "/obd-0.0.1-SNAPSHOT/mesure/obd/save?mesure=" + imiobd + ";" + donneesEnvoyees ;
  Serial.println(sendall);
  Serial.println("la requette ##################   fin   ##########################");
  String donneesEnvoyeesvin = "";
  donneesEnvoyeesvin = String(vin3) + "," + String(vin4) + "," + String(vin5) + "," + String(vin6) + "," + String(vin7) + "," + String(indicateurvin);
  String sendallv ="";
  sendallv = "/obd-0.0.1-SNAPSHOT/mesure/obd/save?mesure=" + imiobd + ";" + donneesEnvoyeesvin ;
      Serial.println("envoyer vin ");
      Serial.println(sendallv);
  http_client.get(sendallv);
    int status_code = http_client.responseStatusCode();
  String retourServeur = http_client.responseBody();
  Serial.println(status_code);
  Serial.println(retourServeur);
    Serial.println("envoyer indicateur ");

  http_client.get(sendall);
  int status_code1 = http_client.responseStatusCode();
   String retourServeur1 = http_client.responseBody();
  Serial.println(status_code1);
  Serial.println(retourServeur1 );
  http_client.stop();
  modem.gprsDisconnect();
  Serial.println("modem disconnected");

  //OTA
  retourServeur = getValue(retourServeur, ':', 2);
  String(OTA) = getValue(retourServeur, '"', 1);
  Serial.print("OTA : ");
  Serial.println(OTA);

  if ((status_code == 200))
  {
    Serial.println("bon retour");
  }
  else
  {
    Serial.println("mauvais retour");
  }
  //OTA=="1";
  if (String(OTA) == "1")
  {
    Serial.println("BEGIN OTA");
    ota_send();
    //      String(OTA)="0";
    //      envoiFlag();
  }

  //Désactivation modem
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, LOW);
  digitalWrite(MODEM_POWER_ON, LOW);
}

String getValue(String dataJSON, char separator, int index) { //Fonction pour séparer les params du fichier JSON
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = dataJSON.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (dataJSON.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? dataJSON.substring(strIndex[0], strIndex[1]) : "";
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    Serial.println("APOK");
  }
  else
  {
    Serial.println("APX");
  }
}

void readFile(fs::FS &fs, const char *path)
{
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory())
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available())
  {
    Serial.write(file.read());
    delayMicroseconds(100);
  }
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root)
  {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory())
  {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels)
      {
        listDir(fs, file.name(), levels - 1);
      }
    }
    else
    {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void deleteFile(fs::FS &fs, const char *path)
{
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path))
  {
    Serial.println("File deleted");
  }
  else
  {
    Serial.println("Delete failed");
  }
}

void updateFromFS()
{
  File updateBin = SPIFFS.open("/cc.bin");
  if (updateBin)
  {
    if (updateBin.isDirectory())
    {
      Serial.println("fichier n'existe pas ");
      updateBin.close();
      return;
    }

    size_t updateSize = updateBin.size();

    if (updateSize > 0)
    {
      Serial.println("On Va installer La mise à jour ");
      performUpdate(updateBin, updateSize);
    }
    else
    {
      Serial.println("Error, archive Vide");
    }

    updateBin.close();

    // whe finished remove the binary from sd card to indicate end of the process
    //fs.remove("/update.bin");
  }
  else
  {
    Serial.println("No se puede cargar el archivo");
  }
}

void performUpdate(Stream &updateSource, size_t updateSize)
{
  if (Update.begin(updateSize))
  {
    size_t written = Update.writeStream(updateSource);
    if (written == updateSize)
    {
      Serial.println("Escritos : " + String(written) + " successfully");
    }
    else
    {
      Serial.println("Solamente escritos : " + String(written) + "/" + String(updateSize) + ". Retry?");
    }
    if (Update.end())
    {
      Serial.println("OTA realizado!");
      if (Update.isFinished())
      {
        Serial.println("Ota exitoso, reiniciando!");
        ESP.restart();
      }
      else
      {
        Serial.println("Ota no terminó? Algo salió mal!");
      }
    }
    else
    {
      Serial.println("Ocurrió Error #: " + String(Update.getError()));
    }
  }
  else
  {
    Serial.println("Sin espacio suficiente para hacer OTA");
  }
}
void printPercent(uint32_t readLength, uint32_t contentLength) {
  // If we know the total length

  if (contentLength != (uint32_t) - 1) {
    SerialMon.print("\r ");
    SerialMon.print((100.0 * readLength) / contentLength);
    SerialMon.print('%');
  } else {
    SerialMon.println(readLength);
  }
}
void ota_send() {
  Serial.println("BEGIN OTA");

  if (!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  SPIFFS.format();
  listDir(SPIFFS, "/", 0);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  SerialMon.println("Initializing modem...");
  modem.restart();
  //delay(2000);
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  do {
    SerialMon.println(" fail");
  }
  while (!modem.gprsConnect(apn, gprsUser, gprsPass));
  SerialMon.println(" OK");
  //modemInfo();
  //Serial.println(getModemInfo());
  SerialMon.print("Connecting to ");
  SerialMon.print(hostnameOBD);
  SerialMon.println("get method");
  http_clientOBD.get("update152.bin");   // /TTcphL75-./obd152oba0.bin"); //.begin();
  SerialMon.println("test");
  int status_code = http_clientOBD.responseStatusCode();
  int contentLength = http_clientOBD.contentLength();
  SerialMon.println(status_code);
  SerialMon.println("contentLength : ");
  SerialMon.print(contentLength);
  long timeout = millis();
  SerialMon.println(F("Reading response data"));
  timeout = millis();
  uint32_t readLength = 0;
  //CRC32 crc;
  //Update.begin(UPDATE_SIZE_UNKNOWN);
  // uint8_t buff[] = { 0 };
  //int pos = 0; // current position in the buffer
  unsigned long timeElapsed = millis();
  //printPercent(readLength, contentLength);
  //uint32_t len_data=printPercent(readLength);
  File file = SPIFFS.open("/cc.bin", FILE_APPEND);
  while (readLength < contentLength && client.connected() && millis() - timeout < 10000L) {
    int i = 0 ;
    while (client.available()) {
      if (!file.write(char(client.read())))
      {
        Serial.println("file added");
      }
      // uint8_t c = client.read();
      //SerialMon.print((char)c);       // Uncomment this to show data
      //crc.update(c);

      //buff[pos++]= client.read();
      readLength++;
      if (readLength % (contentLength / 13) == 0) {
        printPercent(readLength, contentLength);
      }
      timeout = millis();
    }
  }
  file.close();
  //printPercent(readLength, contentLength);
  //uint32_t len_data= readLength;
  timeElapsed = millis() - timeElapsed;
  SerialMon.println();
  http_clientOBD.stop();
  // Shutdown
  client.stop();
  SerialMon.println(F("Server disconnected"));
  modem.gprsDisconnect();
  SerialMon.println(F("GPRS disconnected"));
  float duration = float(timeElapsed) / 1000;
  //SerialMon.print(sizeof(buff));
  SerialMon.println();
  SerialMon.print("Content-Length: ");   SerialMon.println(contentLength);
  SerialMon.print("Actually read:  ");   SerialMon.println(readLength);
  //SerialMon.print("Calc. CRC32:    0x"); SerialMon.println(crc.finalize(), HEX);
  // SerialMon.print("Known CRC32:    0x"); SerialMon.println(knownCRC32, HEX);
  SerialMon.print("Duration:       ");   SerialMon.print(duration); SerialMon.println("s");
  Serial.println("Juste Pour amuser attend woooooooooooooooooo allez !!!!!!!!!!!! 1 vers 3");
  for (int i = 0; i < 3; i++)
  {
    Serial.print(String(i) + "...");
    delay(1000);
  }

  //readFile(SPIFFS, "/update.bin");

  updateFromFS();
}
