#include "FS.h"
#include "SPIFFS.h"
#include <Preferences.h>

Preferences prefs;

String allEans = "";
bool allEans_read_lock = false;
bool allEans_write_lock = false;

bool cards_read_lock = false;
bool cards_write_lock = false;

void getPref(){
  String prefKey="";
  prefs.begin("configs", false); // begin preferences
  deviceMode = prefs.getString("devMode");
  versionName = prefs.getString("name","V3IoT1.0");
  versionLink = prefs.getString("link");
  CloseFlag = prefs.getBool("ServSavedV");
  // savedOPos = prefs.getInt("servoOpen");
  // savedCPos = prefs.getInt("servoClose");
  for (uint8_t i = 0; i < 1; i++) {
    prefKey = "servoClose" + String(i); 
    SClosePos[i] = prefs.getInt(prefKey.c_str());
    prefKey = "servoOpen" + String(i); 
    SOpenPos[i] = prefs.getInt(prefKey.c_str());//75 is the angle from opening to closing of Lid
  }
  for (uint8_t i = 0; i < 4; i++)
  {
    prefKey = "comp" + String(i + 1);
   if(!i) *compName[i] = prefs.getString(prefKey.c_str(), "ewaste-small");
   if(i==1) *compName[i] = prefs.getString(prefKey.c_str(), "ewaste-big");
  }
  ProvisionDone = prefs.getBool("provisionStatus");
  RegisterDone = prefs.getBool("RegisterStatus");
  ShadowRegestred = prefs.getBool("ShadowRegStatus");
  BinType = prefs.getString("Bin_Type");
  PCBVersion = prefs.getString("PCB_Version");
  devCountry = prefs.getString("Country", "ae");
  Material = prefs.getString("Material");
  bottleCount= prefs.getInt("bot_count");
  ewasteCountBig= prefs.getInt("ewasteBig");
  devICCID = prefs.getString("ICCID");
  BinArea = prefs.getString("BIN_Area");
  // devWMC = prefs.getString("WMC");
  QRCode = prefs.getString("QR_Code");
  WifiSSID = prefs.getString("ssid");
  WifiPass = prefs.getString("password");
  SimcomModem = prefs.getBool("mymodem");
  AssemblConnect=prefs.getBool("Assemble");
  if(!RegisterDone) AssemblConnect=true;
}
/**
 * Logs the current device preferences and settings.
 * This function outputs various configuration details to the log for debugging and verification purposes.
 * Fields marked with comments are currently disabled in the log output.
 */
void PrintPref(){
  mLogger.logInfo("prefs", "Print Prefrenses Status");
  mLogger.logInfo("devMode", deviceMode);
  mLogger.logInfo("versionName", versionName);
  mLogger.logInfo("versionLink", versionLink);
  // mLogger.logInfo("testResults", testResults);
  mLogger.logInfo("CloseFlag", String(CloseFlag));
  // mLogger.logInfo("savedOPos", String(savedOPos));
  // mLogger.logInfo("savedCPos", String(savedCPos));
  mLogger.logInfo("ProvisionDone", String(ProvisionDone));
  mLogger.logInfo("ShadowRegestred", String(ShadowRegestred));
  mLogger.logInfo("BinType", BinType);
  mLogger.logInfo("PCBVersion", PCBVersion);
  mLogger.logInfo("devCountry", devCountry);
  mLogger.logInfo("Material", Material);
  // mLogger.logInfo("SensorType", SensorType);
  // mLogger.logInfo("devRGB", String(devRGB));
  mLogger.logInfo("devICCID", devICCID);
  // mLogger.logInfo("BinArea", BinArea);
  // mLogger.logInfo("devWMC", devWMC);
  mLogger.logInfo("QRCode", QRCode);
  mLogger.logInfo("WifiSSID", WifiSSID);
  mLogger.logInfo("WifiPass", WifiPass);
  mLogger.logInfo("Assemble", String(AssemblConnect));
}
//func to load EANs to memory
String readFromFile(fs::FS &fs, const char * path) {
  File file = fs.open(path);
  String s = "";
  if (!file || file.isDirectory()) {
    //Serial.println("- failed to open file for reading");
    return s;
  }

  //Serial.println("- read from file:");
  while (file.available()) {
    char c = char(file.read());
    if (c != ' ')s += c;
  }
 
  file.close();
  return s;
}

/** function to write EANs to txt file/flash
 * 
 *@param FS name of the class
 *@param fs pointer to variable
 *@param path URL of the file/EAN here
 *@param message description of the operation
*/
void writeFile(fs::FS &fs, const char * path, const char * message) {
  //Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    //Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    //Serial.println("- file written");
  } else {
    //Serial.println("- write failed");
  }
  file.close();
}

/** function to delete the file
 * 
 *@param FS name of the class
 *@param fs pointer to variable
 *@param path URL of the file/EAN here
*/
void deleteFile(fs::FS &fs, const char * path) {
  if (fs.remove(path)) {
    //Serial.println("- file deleted");
  } else {
    //Serial.println("- delete failed");
  }
}

/** function to append EANs to the txt file/flash
 * 
 *@param FS name of the class
 *@param fs pointer to variable
 *@param path URL of the file/EAN here
 *@param message description of the operation
*/
void appendFile(fs::FS &fs, const char * path, String message) {

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("- failed to open file for appending");

    return;
  }
  if (file.print(message)) {
    //    //Serial.println("- message appended");
  } else {
    //Serial.println("- append failed");
  }
  file.close();
}

void filesBegin(){
String path="";
if (!SPIFFS.begin(true)) { //format if mount failed
    mLogger.logWarning("filesBegin", "SPIFFS Mount Failed");
    return;
  }
  // for(uint8_t i = 0; i < 4; i++) {
  //   path = "/" + *compName[i] + ".txt";
  //   if (!SPIFFS.exists(path.c_str())) writeFile(SPIFFS, path.c_str(), "1");
  //   else mLogger.logWarning("filesBegin", *compName[i] + " File already exists");
  //   }
if (!SPIFFS.exists("/cards.txt")) {
    //file doesn't exist
    writeFile(SPIFFS, "/cards.txt", "123456");
  }
  else {
     mLogger.logWarning("filesBegin","File already exists");
    //    readFile(SPIFFS, "/EAN.txt");
  }

  cards = readFromFile(SPIFFS, "/cards.txt");
}

