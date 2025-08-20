// globals.h
#ifndef GLOBALS_H
#define GLOBALS_H
    /// compartments global variables 
  #define scanRX 15
// #define scanTX 14
    // Hardware BarSanner(scanRX);   // Receive data pin fom barcode scanner
  #define  BarSanner Serial1

    bool OTADownloading = false;
    // connectivity global variables 
    bool CompCaptive=false;
    bool isCaptiveOn=false;
    static EventGroupHandle_t wifi_event_group;
    const int WIFI_CONNECTED_BIT = BIT0;
    bool AssemblConnect; 
    int restWifiAttempt = 0;
    #define MAX_CLIENTS 4	// ESP32 supports up to 10 but I have not tested it yet
    #define WIFI_CHANNEL 6	// 2.4ghz channel 6 https://en.wikipedia.org/wiki/List_of_WLAN_channels#2.4_GHz_(802.11b/g/n/ax)
    #define MAX_NETWORKS 20 // Define the maximum number of networks to scan
    #define WIFI_TIMEOUT 20
    HTTPClient client;
    bool networkUpdateSent = false;
    bool CollectionCardFlag = false;  
    bool netwoi_Creds=false;
    bool wifiCreds=false;
    bool WifiConnecting = false;
    bool AWSConnecting = false;
    bool ModemConnecting = false;
    bool internetConnected = false;
    String CaptiveSsid = "RenieBin";  // FYI The SSID can't have a space in it.
    const char *CaptivePassword = NULL;  // no password
    const String FALLBACKSSID1 = "Renie 1";
    const String FALLBACKPASS1 = "#RenieTech2023!";
    // const String FALLBACKSSID1 = "Assembly";
    // const String FALLBACKPASS1 = "Renie2022!";
    // const String FALLBACKSSID1 = "Tamim";
    // const String FALLBACKPASS1 = "Tamim123";
    const IPAddress localIP(4, 3, 2, 1);		   // the IP address the web server, Samsung requires the IP to be in public space
    const IPAddress gatewayIP(4, 3, 2, 1);		   // IP address of the network should be the same as the local IP for captive portals
    const IPAddress subnetMask(255, 255, 255, 0);  // no need to change: https://avinetworks.com/glossary/subnet-mask/
    const String localIPURL = "http://4.3.2.1";	 // a string version of the local IP with http, used for redirecting clients to your webpage
    const char* Read_EANs = "/eans?type=";
    const char* Country_EANs = "&country=";

    int16_t numNetworks = 0;
    String storedSSIDs[20]; // Assuming a maximum of 20 networks
    String udpBroadcastAdd = "";
    byte retryCount; 
    String URL;
    const char* ca_cert = \
                        "-----BEGIN CERTIFICATE-----\n" \
                        "MIIEdTCCA12gAwIBAgIJAKcOSkw0grd/MA0GCSqGSIb3DQEBCwUAMGgxCzAJBgNV\n" \
                        "BAYTAlVTMSUwIwYDVQQKExxTdGFyZmllbGQgVGVjaG5vbG9naWVzLCBJbmMuMTIw\n" \
                        "MAYDVQQLEylTdGFyZmllbGQgQ2xhc3MgMiBDZXJ0aWZpY2F0aW9uIEF1dGhvcml0\n" \
                        "eTAeFw0wOTA5MDIwMDAwMDBaFw0zNDA2MjgxNzM5MTZaMIGYMQswCQYDVQQGEwJV\n" \
                        "UzEQMA4GA1UECBMHQXJpem9uYTETMBEGA1UEBxMKU2NvdHRzZGFsZTElMCMGA1UE\n" \
                        "ChMcU3RhcmZpZWxkIFRlY2hub2xvZ2llcywgSW5jLjE7MDkGA1UEAxMyU3RhcmZp\n" \
                        "ZWxkIFNlcnZpY2VzIFJvb3QgQ2VydGlmaWNhdGUgQXV0aG9yaXR5IC0gRzIwggEi\n" \
                        "MA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDVDDrEKvlO4vW+GZdfjohTsR8/\n" \
                        "y8+fIBNtKTrID30892t2OGPZNmCom15cAICyL1l/9of5JUOG52kbUpqQ4XHj2C0N\n" \
                        "Tm/2yEnZtvMaVq4rtnQU68/7JuMauh2WLmo7WJSJR1b/JaCTcFOD2oR0FMNnngRo\n" \
                        "Ot+OQFodSk7PQ5E751bWAHDLUu57fa4657wx+UX2wmDPE1kCK4DMNEffud6QZW0C\n" \
                        "zyyRpqbn3oUYSXxmTqM6bam17jQuug0DuDPfR+uxa40l2ZvOgdFFRjKWcIfeAg5J\n" \
                        "Q4W2bHO7ZOphQazJ1FTfhy/HIrImzJ9ZVGif/L4qL8RVHHVAYBeFAlU5i38FAgMB\n" \
                        "AAGjgfAwge0wDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAYYwHQYDVR0O\n" \
                        "BBYEFJxfAN+qAdcwKziIorhtSpzyEZGDMB8GA1UdIwQYMBaAFL9ft9HO3R+G9FtV\n" \
                        "rNzXEMIOqYjnME8GCCsGAQUFBwEBBEMwQTAcBggrBgEFBQcwAYYQaHR0cDovL28u\n" \
                        "c3MyLnVzLzAhBggrBgEFBQcwAoYVaHR0cDovL3guc3MyLnVzL3guY2VyMCYGA1Ud\n" \
                        "HwQfMB0wG6AZoBeGFWh0dHA6Ly9zLnNzMi51cy9yLmNybDARBgNVHSAECjAIMAYG\n" \
                        "BFUdIAAwDQYJKoZIhvcNAQELBQADggEBACMd44pXyn3pF3lM8R5V/cxTbj5HD9/G\n" \
                        "VfKyBDbtgB9TxF00KGu+x1X8Z+rLP3+QsjPNG1gQggL4+C/1E2DUBc7xgQjB3ad1\n" \
                        "l08YuW3e95ORCLp+QCztweq7dp4zBncdDQh/U90bZKuCJ/Fp1U1ervShw3WnWEQt\n" \
                        "8jxwmKy6abaVd38PMV4s/KCHOkdp8Hlf9BRUpJVeEXgSYCfOn8J3/yNTd126/+pZ\n" \
                        "59vPr5KW7ySaNRB6nJHGDn2Z9j8Z3/VyVOEVqQdZe4O/Ui5GjLIAZHYcSNPYeehu\n" \
                        "VsyuLAOQ1xk4meTKCRlb/weWsKh/NEnfVqn3sF/tM+2MR7cwA130A4w=\n" \
                        "-----END CERTIFICATE-----\n";


    // A7670Modem.h variables 
    bool ModemConnected = false;
    int HourNow = 0;
    int MinuteNow = 0;
    int SecondNow = 0;
    int dayNow = 0;
    int monthNow = 0;
    int yearNow = 0;
    float myTimeZone = 0.0;
   
   HardwareSerial scanner(1);
// address we will assign if dual sensor is present
#define TCAADDR 0x77 // Default I2C address of TCA952DR8A
#define TCAADDR_1 0x70 // Default I2C address of TCA952DR8A
// address we will assign if dual sensor is present
#define defaultAddress 0x29
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define VSensAddress1 0x32
#define VSensAddress2 0x33
#define VSensAddress3 0x34
#define PRINT_DEC_POINTS  3         // decimal points to print
// uint8_t compartmentState[4] = {0, 0, 0, 0}; // Current detection status
uint8_t compartmentState[4] = {0, 0, 0, 0}; // Current detection status
uint8_t EwasteCompartmentState[4] = {0, 0, 0, 0}; // Current detection status
bool sensorFail = false;
// Setup mode for doing reads
Beastdevices_INA3221 ina3221(INA3221_ADDR40_GND);
Adafruit_INA219 ina219(0x45);
ina3221_ch_t INA3221Channels[3] = {INA3221_CH1, INA3221_CH3, INA3221_CH2};
float currentThreshoulds[4]={};
float sampleReadingCurrent = 0.0;
// Adafruit_INA219 ina219;  //Defined INA219_ADDRESS (0x45) // 1000101 (A0+A1=VCC)
//objects for the VL6180X

TCA9548A I2CMux(TCAADDR);
TCA9548A I2CMux_1(TCAADDR_1);

Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox5 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox6 = Adafruit_VL53L0X();


Adafruit_VL53L0X lox7 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox8 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox9 = Adafruit_VL53L0X();

Adafruit_VL53L0X lox10 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox11 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox12 = Adafruit_VL53L0X();
 
// Adafruit_VL6180X lox4 = Adafruit_VL6180X();
// Adafruit_VL6180X lox5 = Adafruit_VL6180X();
// Adafruit_VL53L0X lox6 = Adafruit_VL53L0X();


// Adafruit_VL6180X lox7 = Adafruit_VL6180X();
// Adafruit_VL6180X lox8 = Adafruit_VL6180X();
// Adafruit_VL53L0X lox9 = Adafruit_VL53L0X();

// Adafruit_VL6180X lox10 = Adafruit_VL6180X();
// Adafruit_VL6180X lox11 = Adafruit_VL6180X();
// Adafruit_VL53L0X lox12 = Adafruit_VL53L0X();
 
extern void downloadEANList();

int UpNow,DownNow, state;


float myusage;
const int upperThreshold = 198; // was 195
const int lowerThreshold = 198;// was 176

const int upperHysteresis = 25;
const int lowerHysteresis = 25;

const int differenceThreshold = 25; // was 10
int differenceState = 0; // Initialize difference state

int Statcounter1;
int StatecounterN2;
int StatecounterN1;


bool upperDetected = false;
bool lowerDetected = false;
bool whitelableF = false;
float VDistance=0.0;
float VAvraValue=0.0;
int fakedropcounter;
int bottleCount;
int ewasteCountBig;
int binSize=120;      
int binfullDis=30;
bool isWaitingForCollection = false;
byte bottleNotDetectCount; 
bool triggerCollection = false;

    // awsiotcore variables 

    #define AWS_IOT_ENDPOINT "a29iszikvyvxl5-ats.iot.me-central-1.amazonaws.com" 
    extern const char AWS_CERT_CA[] asm("_binary_certs_ca_pem_start");
    extern const char AWS_CERT_CRT[] asm("_binary_certs_cert_pem_start");
    extern const char AWS_CERT_PRIVATE[] asm("_binary_certs_private_pem_start");

    //// Tobics definitions. 
    String Add_Bottle;
    String Collection_done;
    String CreateIssueTopic;
    String UnknownBottleTopic;
    String FakedropTopic;
    String BarcodeScannedTopic;
    String AWS_IOT_PUB_TOPIC;
    String Scanned;
    String MakeCollection;
    String myServosub;
    String myServoCalibation;
    String myLockSub;
    String myResetBinSub;
    String AWS_IOT_SUB_TOPIC;
    String RegisterandDeploy;
    String SensorFBTopic;
    String ResetWifiTopic;
    
    char SUBtopic[50];
    char PUBtopic[50];
    char ServoSub[50];
    char ServoCalibation[50]; 
    char LockSub[50];
    char ResetWifiSub[50];
    char ResetBinSub[50];
    char ScannedTopic[50];
    char MakeCollectionTopic[50];
    char THINGNAME[12]; 
    char Shadow_Get[50]="$aws/things/{myThingname}/shadow/get";
    char Get_Accept[50]="$aws/things/{myThingname}/shadow/get/accepted";
    char Get_Rejected [50]="$aws/things/{myThingname}/shadow/get/rejected";
    char Shadow_Update[50]="$aws/things/{myThingname}/shadow/update";
    char Update_Accept[50] = "$aws/things/{myThingname}/shadow/update/accepted";
    char Update_Rejected[50] = "$aws/things/{myThingname}/shadow/update/rejected";
    char Update_Delta[50] = "$aws/things/{myThingname}/shadow/update/delta";

    // defininitions of Device shadow objects. 
    const char* IOTDevice_Mode;
    const char* IOTPCB_Version;
    const char* IOTBin_Type;
    const char* IOTCountry;
    const char* IOTMaterial;
    const char* IOTSensors_Type;
    bool IOTRGB;
    const char* IOTICCID;
    const char* IOTBIN_Area;
    const char* IOTWMC;
    const char* IOTQR_Code;
    const char* IOTWifi_SSID;
    const char* IOTWifi_Pass;
    int IOTServoCloseAngle;
    int IOTServoOpenAngle;
    const char *IOTCards;
    String versionName;
    String versionLink;

    uint8_t SensorState[2][3] = {{0, 0, 0}, {0, 0, 0}};

    //variables definisition 
    String IoTEAN;
    bool hbTenable; 
    bool ProvisionDone;
    bool ShadowRegestred;
    int PuBackCounter;
    bool AWSCOnnectivity = false;
    bool CupIncoming = false;
    bool g_OTAFlag=false;
    bool compartIusse[4] = {false, false, false, false};
    String CompartNameIssue = "";
    String cards="";
    String timestamp = "";
    bool binlocaltimeflag = false;
    String CompartmentIssues[4];  // Array to store issues for each compartment
    String ServoIssues[4];  // Array to store issues for each compartment
    bool isEANdownloading=false;

    // systemfunction variables
#define OTA_SUCCESS 0
#define OTA_NO_SPACE 1
#define OTA_NO_CONTENT 2
#define OTA_INCOMPLETE 3
#define OTA_NO_CONNECTION 4
#define OTA_INVALID_CONTENT 5
#define OTA_ERROR 6
    String deviceMode = "";
#define encrypt_length 8
#define SLEEP 0
#define POWER_RESET 1
#define RTC_IO 2
#define TIMER 3
#define StorageSize 100
    String tdesKey = "1937116853248E9436FFDACA";
    byte out[encrypt_length];
    byte hex[encrypt_length];
    byte tdes_key[24];

    uint32_t chipId = 0;
    String ChipID = "";
    String Bin_ID = "";
    // const int8_t gmtoffsethour = +4;//write your country's offset
    const int8_t daylightSavinghour = 0; // write your country's daylight Saving hours
    bool isI2Ciniatlized = false;
    
    const char *ntpServer = "pool.ntp.org";
    // const int  gmtOffset_sec = gmtoffsethour * 3600;
    const int daylightOffset_sec = daylightSavinghour * 3600;
    int gmtOffset_sec;
    String timeNow = "";
    String BinType = "";
    String PCBVersion = "";
    String devCountry = "";
    String Material = "";
    String SensorType = "";
    bool devRGB;
    String devICCID = "";
    String BinArea = "";
    String devWMC = "";
    String QRCode = "";
    String WifiSSID = "";
    String WifiPass = "";
    String UsageValue = "";
    bool RegisterDone;
    String ServoCloseAngle = "";
    String ServoOpenAngle = "";
    bool isSOmodifed = false;
    bool isSCmodifed = false;
    bool SimcomModem = false;
    bool incomingEwaste = false;
    String comp1Name = "", comp2Name = "", comp3Name = "", comp4Name = "";
    String *compName[] = {&comp1Name, &comp2Name, &comp3Name, &comp4Name};
    ////////////////////// Sensors global variable or Input Outputs
    bool isBinFull[4] = {false, false, false, false};
    CRGB *leds = nullptr;
#define BRIGHTNESS 255            // Brightness level (0-255)
#define RGB_CAN_PIN 19            // Pin to which the LED strip is connected
#define RGB_PET_PIN 12            // Pin to which the LED strip is connected
#define RGB_PAPER_PIN 23          // Pin to which the LED strip is connected
#define RGB_Additional_PIN 27     // Pin to which the LED strip is connected
    static bool firstRunDone = false; // Static variable to track if block has executed
    uint16_t p_ledStateCounter;
    uint16_t g_ledStateCounter;
uint8_t NUM_LEDS=16;
#define LED_NUMS 16
size_t ewasteBigTime=0;
    uint8_t servoPins[] = {18, 25,14, 32};
    uint8_t lockPins[] = {17, 26,13, 33};

// #define LED_PIN 2
    bool CloseFlag;

    int16_t SClosePos[4]= { 30, 30, 30, 30};
    // = {0, 0, 0, 0};
    int16_t SOpenPos[4]= {95, 95, 95, 95};
    int16_t savedOPos[4]= {0, 0, 0, 0};
    int16_t savedCPos[4]= {0, 0, 0, 0};
    bool servoLimit[4] = {false, false, false, false};
#define _ANGLELIMIT 180
    // First declare individual LED arrays
    const int MAX_RGB_STRIPS = 4;
    CRGB RGB_Can[LED_NUMS], RGB_Paper[LED_NUMS], RGB_Pet[LED_NUMS], RGB_Additional[LED_NUMS];
    CRGB *RGBcomp[4] = {RGB_Can, RGB_Paper, RGB_Pet, RGB_Additional};

    bool compVaildation(uint8_t i)
    {
      if (*compName[i] != "null" && *compName[i] != "")
        return true;
      else
        return false;
    }
     
    void turnLock(bool status)
    {
      for (uint8_t i = 0; i < 4; i++)
      {
        if (compartmentState[0] || EwasteCompartmentState[1])
        {
          {
            digitalWrite(lockPins[i], status);
            vTaskDelay(5 / portTICK_PERIOD_MS);
          }
        }
      }
    }
    // main global variablse
    int loadingCounter;


#endif