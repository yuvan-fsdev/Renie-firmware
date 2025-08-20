#define TINY_GSM_MODEM_A7670
HardwareSerial SerialAT(2);
#define TINY_GSM_DEBUG Serial
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define GSM_PIN ""
#include <TinyGsmClient.h>

TinyGsm modem(SerialAT);
TinyGsmClient modemclient(modem);
const char apn[] = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";


void ReconnectModem() {
    mLogger.logMessage("ReconnectModem", "Connection lost. Reconnecting...");
    modem.gprsDisconnect(); // Close current connection, if any
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait a bit before reconnecting
    
    if (modem.gprsConnect(apn, gprsUser, gprsPass)) { // Attempt GPRS reconnection
    mLogger.logMessage("ReconnectModem", "GPRS reconnected successfully.");
        ModemConnected=true;

    } else {
    mLogger.logWarning("ReconnectModem", "Failed to reconnect GPRS. Retrying...");
    }
}

String GetTimeModem(){
  if (modem.getNetworkTime(&yearNow, &monthNow, &dayNow, &HourNow, &MinuteNow, &SecondNow, &myTimeZone)) {
    // Successfully retrieved time from the network
    mLogger.logMessage("GetTimeModem", "Time obtained from network:");
    // Print the date in YYYY-MM-DD format
    mLogger.logMessage("GetTimeModem", "Date: " + String(yearNow) + "-" + (monthNow < 10 ? "0" : "") + String(monthNow) + "-" + (dayNow < 10 ? "0" : "") + String(dayNow));
    // Print the time in HH:MM:SS format
    mLogger.logMessage("GetTimeModem", "Time: " + String(HourNow < 10 ? "0" : "") + String(HourNow) + ":" + (MinuteNow < 10 ? "0" : "") + String(MinuteNow) + ":" + (SecondNow < 10 ? "0" : "") + String(SecondNow));
    // Print the timezone
    mLogger.logMessage("GetTimeModem", String(myTimeZone));  // Print timezone with one decimal place);
    // Calculate the Unix timestamp (epoch time)
    struct tm modemtimeinfo;
    modemtimeinfo.tm_year = yearNow - 1900;  // tm_year is years since 1900
    modemtimeinfo.tm_mon = monthNow - 1;     // tm_mon is 0-based (0 = January)
    modemtimeinfo.tm_mday = dayNow;
    modemtimeinfo.tm_hour = HourNow;
    modemtimeinfo.tm_min = MinuteNow;
    modemtimeinfo.tm_sec = SecondNow;
    modemtimeinfo.tm_isdst = -1;  // Daylight saving time is not in effect

    time_t epochTime = mktime(&modemtimeinfo);  // Convert to epoch time

    // Print the timestamp
    mLogger.logMessage("GetTimeModem", "Timestamp (Unix Epoch): "+ String(epochTime));
    return String(epochTime);
    } else {
        // Failed to retrieve the time
    mLogger.logMessage("GetTimeModem", "Failed to get time from network.");
        return "";
    }
}
void getmodeminfo(){
    mLogger.logMessage("getmodeminfo", "getOperator: "+ modem.getOperator());
    mLogger.logMessage("getmodeminfo", "Sim card getProvider: "+modem.getProvider());
    mLogger.logMessage("getmodeminfo", "getNetworkMode: "+String(modem.getNetworkMode()));
    mLogger.logMessage("getmodeminfo", "getRegistrationStatus: "+String(modem.getRegistrationStatus()));
    mLogger.logMessage("getmodeminfo", "getSimCCID: "+modem.getSimCCID());
    mLogger.logMessage("getmodeminfo", "getSimStatus: "+String(modem.getSimStatus()));
    mLogger.logMessage("getmodeminfo", "getSignalQuality: "+String(modem.getSignalQuality()));
    mLogger.logMessage("getmodeminfo", "getGsmLocation: "+modem.getGsmLocation());
}
void connect_to_modem(void *pvParameters) //task to connect to modem
{
    ModemConnecting=true;
    mLogger.logMessage("Modem A7670E", "Initializing modem...");
    SerialAT.begin(115200, SERIAL_8N1, 34, 4);//Vertical sensor port  16/17
    vTaskDelay(500/portTICK_PERIOD_MS);
    modem.mqtt_end();
    modem.streamClear();
    modem.restart();
    vTaskDelay(500/portTICK_PERIOD_MS);
    mLogger.logMessage("Modem A7670E", "Initializing modem done!");
    String modemInfo = modem.getModemInfo();
    mLogger.logMessage("Modem A7670E info:", modemInfo);
    if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); }
    mLogger.logMessage("Modem A7670E Connecting", "Waiting for network...");
    if (!modem.waitForNetwork()) {
        mLogger.logMessage("Modem A7670E Connecting", "fail, try again after 5s");
        ModemConnecting=false;
        vTaskDelay(5000/portTICK_PERIOD_MS);
        vTaskDelete(NULL);
        return;
    }
    if (modem.isNetworkConnected()) mLogger.logMessage("Modem A7670E Connecting", "network success!");
    mLogger.logMessage("Modem A7670E Connecting", "Connecting To network GPRS...");
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        mLogger.logWarning("Modem A7670E Connecting", "GPRS fail, try again after 5s");
        ModemConnecting=false;
        vTaskDelay(5000/portTICK_PERIOD_MS);
        vTaskDelete(NULL);
        return;
    }
    mLogger.logMessage("Modem A7670E Connecting", "GPRS Connection success!");
    modem.sendAT("AT+CREG=0");
    mLogger.logMessage("connect_to_modem", "getOperator: "+ modem.getOperator());
    mLogger.logMessage("connect_to_modem", "Sim card getProvider: " +modem.getProvider());
    mLogger.logMessage("connect_to_modem", "getNetworkMode: " + String(modem.getNetworkMode()));
    mLogger.logMessage("connect_to_modem", "getRegistrationStatus: " + String(modem.getRegistrationStatus()));
    mLogger.logMessage("connect_to_modem", "getSimCCID: " + modem.getSimCCID());
    mLogger.logMessage("connect_to_modem", "getSimStatus: "+String(modem.getSimStatus()));
    mLogger.logMessage("connect_to_modem", "getSignalQuality: "+ String(modem.getSignalQuality()));
    GetTimeModem();
    // Serial.print("getGsmLocation: ");
    // Serial.println(modem.getGsmLocation());
    // vTaskDelay(2000/portTICK_PERIOD_MS);
    ModemConnected=true;
    mLogger.logMessage("Modem A7670E", "Initializing modem Done !!!!");
    ModemConnecting=false;
    internetConnected=true;
    // xTaskCreate(ModemAWSConnect, "awsmodemcon", 10000, NULL, 2, &aws_task);
    vTaskDelete(NULL);
}
bool isModuleConnected() {
    SerialAT.begin(115200, SERIAL_8N1, 34, 4);//Vertical sensor port  16/17
    vTaskDelay(200/portTICK_PERIOD_MS);

    SerialAT.println("AT");     // Send AT command
    vTaskDelay(200/portTICK_PERIOD_MS);

    if (SerialAT.available()) {
        String response = SerialAT.readString();
        if (response.indexOf("OK") != -1) {
        return true;             // Module is connected
        }
    }
    return false;                // No response, module not connected
    }
// void StartGPS(){
//     Serial.println("Enabling GPS/GNSS/GLONASS");
//     while (!modem.enableGPS()) {
//         Serial.print(".");
//     }
//     Serial.println();
//     Serial.println("GPS Enabled");
//     delay(1000);
//     modem.setGPSBaud(9600);
//     delay(5000);
//     float lat2      = 0;
//     float lon2      = 0;
//     float speed2    = 0;
//     float alt2      = 0;
//     int   vsat2     = 0;
//     int   usat2     = 0;
//     float accuracy2 = 0;
//     int   year2     = 0;
//     int   month2    = 0;
//     int   day2      = 0;
//     int   hour2     = 0;
//     int   min2      = 0;
//     int   sec2      = 0;
//     uint8_t    fixMode   = 0;
//     for (;;) {
//         Serial.println("Requesting current GPS/GNSS/GLONASS location");
//         if (modem.getGPS(&fixMode, &lat2, &lon2, &speed2, &alt2, &vsat2, &usat2, &accuracy2,
//                             &year2, &month2, &day2, &hour2, &min2, &sec2)) {

//             Serial.print("FixMode:"); Serial.println(fixMode);
//             Serial.print("Latitude:"); Serial.print(lat2, 6); Serial.print("\tLongitude:"); Serial.println(lon2, 6);
//             Serial.print("Speed:"); Serial.print(speed2); Serial.print("\tAltitude:"); Serial.println(alt2);
//             Serial.print("Visible Satellites:"); Serial.print(vsat2); Serial.print("\tUsed Satellites:"); Serial.println(usat2);
//             Serial.print("Accuracy:"); Serial.println(accuracy2);

//             Serial.print("Year:"); Serial.print(year2);
//             Serial.print("\tMonth:"); Serial.print(month2);
//             Serial.print("\tDay:"); Serial.println(day2);

//             Serial.print("Hour:"); Serial.print(hour2);
//             Serial.print("\tMinute:"); Serial.print(min2);
//             Serial.print("\tSecond:"); Serial.println(sec2);
//             break;
//         } else {
//             Serial.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.");
//             delay(15000L);
//         }
//     }   
// }
void restartmodem() {
    if (restWifiAttempt > 5) {
        mLogger.logMessage("Modem Restart", "Attempt >5 Restarting the ESP32 now");
        restWifiAttempt=0;
        sleep(5);
        ESP.restart();
    } else {
        restWifiAttempt++;
        mLogger.logMessage("Modem Restart", "Restarting Modem Restart");
        // modem.restart();
        // xTaskCreatePinnedToCore(connect_to_modem, "connect_to_modem", 5000, NULL, 1, &Task1, 1);
    }
}