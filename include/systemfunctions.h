#include <DES.h>
#include <base64.hpp>
#include <Update.h>


DES des;

void ConfigSystemParam(){
  vTaskDelay(7000 / portTICK_PERIOD_MS);
  mLogger.logMessage("SimcomModem","Detecting A7670E Modem...");
  if (isModuleConnected()){
    prefs.begin("configs", false); // begin preferences
    mLogger.logMessage("SimcomModem","A7670E Modem Detected !!!!");
    SimcomModem=true;
    prefs.putBool("mymodem",SimcomModem);
    prefs.end();
  }
  else {
    prefs.begin("configs", false); // begin preferences
    mLogger.logMessage("SimcomModem","Not detected.");
    SimcomModem=false;
    prefs.putBool("mymodem",SimcomModem);
    prefs.end();
  }
  // if (savedOPos == 0)
  // {
  //   SOpenPos = 105;
  //   prefs.putInt("servoOpen", SOpenPos);
  // }
  // else
  // {
  //   SOpenPos = savedOPos;
  // }
  // if (savedCPos == 0 && CloseFlag == LOW)
  // {
  //   SClosePos = 12;
  //   prefs.putInt("servoClose", SClosePos);
  // }
  // else
  // {
  //   SClosePos = savedCPos;
  // }
    // binSize = 90;
    // binfullDis = 20;
    // NUM_LEDS=15;
    leds = new CRGB[NUM_LEDS];
  // if (devCountry==""){
  //   Serial.println("Country is empty, Set to ae");
  //   devCountry="ae";
  //   prefs.putString("Country",devCountry);
  //   gmtOffset_sec = +4 * 3600;// +4 UAE GMT +4 timing 
  // }
  //   else {
  //   gmtOffset_sec = +3 * 3600; // +3 rest of middle east. 
  // }
  if (PCBVersion==""){
    Serial.println("PCBVersion is empty, Set to V3.1");
    PCBVersion="V3.1";
    prefs.putString("PCB_Version",PCBVersion);
  }
  if (WifiSSID=="" && WifiPass==""){
    wifiCreds=false;
  }
  else wifiCreds=true;
  // deviceMode="development";
  if (deviceMode == "staging")
    URL = "https://backendstaging.renie.io";
  else if (deviceMode == "development")
    URL = "https://apidev-bin.renie.io"; // test for anandu it was: else if (deviceMode == "development")URL = "https://backenddev.renie.io";
  else{
    if(devCountry=="ae")URL = "https://api-bin.renie.io"; // https://backend.renie.io
    else if (devCountry=="ksa")URL = "https://api-bin.ksa.renie.io";
    else if (devCountry=="qa") URL = "https://api-bin.renie.qa"; // https://backend-pk.renie.io  
  }
}
bool printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    // mLogger.logMessage("printLocalTime", "Failed to obtain time");
    return false;
  }

  // mLogger.logMessage(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  HourNow = timeinfo.tm_hour;
  MinuteNow = timeinfo.tm_min;
  SecondNow = timeinfo.tm_sec;
  dayNow = timeinfo.tm_mday;
  monthNow = timeinfo.tm_mon + 1;
  yearNow = timeinfo.tm_year - 100 + 2000;
  return true;
}

void StringToHexArray(String input, byte * target, int length) {
  input.getBytes(target, length + 1);
}
String encodeToBase64(byte * hex, int rawLength) {
  char encoded[BASE64::encodeLength(rawLength)];
  BASE64::encode((const uint8_t*)hex, rawLength, encoded);
  return String(encoded);
}

String GetMacAddress(){
  // Get the MAC address for the station interface
  uint8_t mymac[6];
  char binMac[18];  // Format: XX:XX:XX:XX:XX:XX\0
  esp_read_mac(mymac, ESP_MAC_WIFI_STA);
  snprintf(binMac, sizeof(binMac), "%02X:%02X:%02X:%02X:%02X:%02X",
            mymac[0], mymac[1], mymac[2], mymac[3], mymac[4], mymac[5]);
  mLogger.logInfo("GetMacAddress",binMac);
  return String(binMac);
}

void Get_ChipID() {
  StringToHexArray(tdesKey, tdes_key, 24); //prepare the key buffer
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  ChipID = String(chipId);
  Bin_ID = String(chipId);
  mLogger.logInfo("Get_ChipID",Bin_ID);  
  StringToHexArray(ChipID, hex, encrypt_length); //convert chip id to hex
  des.tripleEncrypt(out, hex, tdes_key); //encrypt to ascii/hex_16
  String encodedChipID = encodeToBase64(out, encrypt_length); //encode to base64
  mLogger.logMessage("Get_ChipID", "encodedChipID: " + encodedChipID); 
  ChipID = encodedChipID;
}
byte print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : mLogger.logMessage("print_wakeup_reason", "RTC_IO Wakeup"); return RTC_IO; break;
    case ESP_SLEEP_WAKEUP_EXT1 : mLogger.logMessage("print_wakeup_reason", "RTC_IO Wakeup"); return RTC_IO; break;
    case ESP_SLEEP_WAKEUP_TIMER : mLogger.logMessage("print_wakeup_reason", "TIMER Wakeup"); return TIMER; break;    //case ESP_SLEEP_WAKEUP_TOUCHPAD : mLogger.logMessage("print_wakeup_reason", "print_wakeup_reason","Wakeup caused by touchpad"); break;
    default :
      mLogger.logInfo(String("wakeup_reason"), String(wakeup_reason));
      return POWER_RESET;
  }
  return POWER_RESET;
}
String getUnixTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    mLogger.logMessage("getUnixTimestamp", "Failed to obtain Unix Times");
    return "";
  }
  // mLogger.logMessage(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  time_t unixTime = mktime(&timeinfo);
  return String(unixTime);
}

String getHostName(String url) {
  url = url.substring(8); //remove 'https://'
  url = url.substring(0, url.indexOf('/')); //start to the index of '/' begining of filename
  return url;
}
String getFileName(String url) {
  url = url.substring(8); //remove 'https://'
  url = url.substring(url.indexOf('/')); //start from index of '/' begining of filename
  return url;
}

String getHeaderValue(String header, String headerName) {
  return header.substring(strlen(headerName.c_str()));
}

#include "esp_task_wdt.h" // Include for watchdog management

byte SimComExecOTA(String host, String bin) {
    long contentLength = 0;
    bool isValidContentType = false;

    // Indicate OTA is downloading
    OTADownloading = true;

    mLogger.logMessage("SimComExecOTA", "Connecting to: " + String(host));

    // Validate host and bin inputs
    if (host.isEmpty() || bin.isEmpty()) {
        mLogger.logMessage("SimComExecOTA", "Invalid host or bin path.");
        OTADownloading = false;
        return OTA_NO_CONNECTION;
    }

    // Temporarily disable watchdog for the task
    esp_task_wdt_delete(NULL);

    // Attempt to connect to the host
    if (modemclient.connect(host.c_str(), 80)) {
        mLogger.logMessage("SimComExecOTA", "Connection established to: " + String(host));

        // Send the HTTP GET request
        modemclient.print(String("GET ") + bin + " HTTP/1.1\r\n" +
                          "Host: " + host + "\r\n" +
                          "Cache-Control: no-cache\r\n" +
                          "Connection: close\r\n\r\n");

        unsigned long timeout = millis();
        while (modemclient.available() == 0) {
            if (millis() - timeout > 5000) { // Timeout after 5 seconds
                mLogger.logMessage("SimComExecOTA", "Client Timeout! Could not receive data.");
                modemclient.stop();
                esp_task_wdt_add(NULL); // Re-enable the watchdog
                OTADownloading = false;
                return OTA_NO_CONNECTION;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS); // Yield to other tasks
        }

        // Read headers from the response
        while (modemclient.available()) {
            String line = modemclient.readStringUntil('\n');
            line.trim();
            if (!line.length()) break;

            if (line.startsWith("HTTP/1.1")) {
                if (line.indexOf("200") < 0) {
                    mLogger.logMessage("SimComExecOTA", "Non-200 status code received: " + line);
                    esp_task_wdt_add(NULL); // Re-enable the watchdog
                    OTADownloading = false;
                    return OTA_INVALID_CONTENT;
                }
            }
            if (line.startsWith("Content-Length: ")) {
                contentLength = atol((getHeaderValue(line, "Content-Length: ")).c_str());
                mLogger.logMessage("SimComExecOTA", "Received Content-Length: " + String(contentLength));
            }
            if (line.startsWith("Content-Type: ")) {
                String contentType = getHeaderValue(line, "Content-Type: ");
                mLogger.logMessage("SimComExecOTA", "Received Content-Type: " + contentType);
                if (contentType == "application/octet-stream") {
                    isValidContentType = true;
                } else {
                    esp_task_wdt_add(NULL); // Re-enable the watchdog
                    OTADownloading = false;
                    return OTA_INVALID_CONTENT;
                }
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    } else {
        mLogger.logMessage("SimComExecOTA", "Connection to " + String(host) + " failed. Verify host and network.");
        esp_task_wdt_add(NULL); // Re-enable the watchdog
        OTADownloading = false;
        return OTA_NO_CONNECTION;
    }

    // Log final checks
    mLogger.logMessage("SimComExecOTA", "Content-Length: " + String(contentLength) + ", Valid Content-Type: " + String(isValidContentType));

    if (contentLength && isValidContentType) {
        if (Update.begin(contentLength)) {
            mLogger.logMessage("SimComExecOTA", "Starting OTA update...");

            while (modemclient.available()) {
                uint8_t buffer[256];
                size_t bytesRead = modemclient.read(buffer, sizeof(buffer));
                if (bytesRead > 0) {
                    size_t written = Update.write(buffer, bytesRead);
                    if (written != bytesRead) {
                        mLogger.logMessage("SimComExecOTA", "Write mismatch during OTA update.");
                        esp_task_wdt_add(NULL); // Re-enable the watchdog
                        OTADownloading = false;
                        return OTA_INCOMPLETE;
                    }
                }

                // Reset watchdog and yield
                esp_task_wdt_reset();
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }

            if (Update.end()) {
                if (Update.isFinished()) {
                    mLogger.logMessage("SimComExecOTA", "OTA update completed successfully.");
                    esp_task_wdt_add(NULL); // Re-enable the watchdog
                    OTADownloading = false;
                    return OTA_SUCCESS;
                } else {
                    mLogger.logMessage("SimComExecOTA", "OTA update not finished. Check for issues.");
                    esp_task_wdt_add(NULL); // Re-enable the watchdog
                    OTADownloading = false;
                    return OTA_INCOMPLETE;
                }
            } else {
                mLogger.logMessage("SimComExecOTA", "OTA update error: " + String(Update.getError()));
                esp_task_wdt_add(NULL); // Re-enable the watchdog
                OTADownloading = false;
                return OTA_ERROR;
            }
        } else {
            mLogger.logMessage("SimComExecOTA", "Not enough space for OTA update.");
            esp_task_wdt_add(NULL); // Re-enable the watchdog
            OTADownloading = false;
            return OTA_NO_SPACE;
        }
    } else {
        mLogger.logMessage("SimComExecOTA", "Invalid content or response from server.");
        esp_task_wdt_add(NULL); // Re-enable the watchdog
        OTADownloading = false;
        return OTA_NO_CONTENT;
    }

    esp_task_wdt_add(NULL); // Re-enable the watchdog
    OTADownloading = false;
    return OTA_ERROR;
}
byte execOTA(String host, String bin) {
  long contentLength = 0;
  bool isValidContentType = false;
    OTADownloading = true;
  WiFiClient client;
  
  //Serial.println("Connecting to: " + String(host));
  // Connect to S3
  if (client.connect(host.c_str(), 80)) {
    // Connection Succeed.
    // Fecthing the bin
    Serial.println("Fetching Bin: " + String(bin));

    // Get the contents of the bin file
    client.print(String("GET ") + bin + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Cache-Control: no-cache\r\n" +
                 "Connection: close\r\n\r\n");

    // Check what is being sent
    //    Serial.print(String("GET ") + bin + " HTTP/1.1\r\n" +
    //                 "Host: " + host + "\r\n" +
    //                 "Cache-Control: no-cache\r\n" +
    //                 "Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println("Client Timeout !");
        client.stop();
        OTADownloading = false;
        return OTA_NO_CONNECTION;
      }
    }
    // Once the response is available,
    // check stuff

    while (client.available()) {
      // read line till /n
      String line = client.readStringUntil('\n');
      // remove space, to check if the line is end of headers
      line.trim();

      // if the the line is empty,
      // this is end of headers
      // break the while and feed the
      // remaining `client` to the
      // Update.writeStream();
      if (!line.length()) {
        //headers ended
        break; // and get the OTA started
      }

      // Check if the HTTP Response is 200
      // else break and Exit Update
      if (line.startsWith("HTTP/1.1")) {
        if (line.indexOf("200") < 0) {
          Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
            OTADownloading = false;
          return OTA_INVALID_CONTENT;
        }
      }

      // extract headers here
      // Start with content length
      if (line.startsWith("Content-Length: ")) {
        contentLength = atol((getHeaderValue(line, "Content-Length: ")).c_str());
        Serial.println("Got " + String(contentLength) + " bytes from server");
      }

      // Next, the content type
      if (line.startsWith("Content-Type: ")) {
        String contentType = getHeaderValue(line, "Content-Type: ");
        Serial.println("Got " + contentType + " payload.");
        if (contentType == "application/octet-stream") {
          isValidContentType = true;
        }
        else {
            OTADownloading = false;
            return OTA_INVALID_CONTENT;
        }
      }
    }
  } else {
    // Connect to S3 failed
    // May be try?
    // Probably a choppy network?
    Serial.println("Connection to " + String(host) + " failed. Please check your setup");
    OTADownloading = false;
    return OTA_NO_CONNECTION;
  }

  // Check what is the contentLength and if content type is `application/octet-stream`
  Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

  // check contentLength and content type
  if (contentLength && isValidContentType) {
    // Check if there is enough to OTA Update
    bool canBegin = Update.begin(contentLength);

    // If yes, begin
    if (canBegin) {
      Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
      // No activity would appear on the Serial monitor
      // So be patient. This may take 2 - 5mins to complete
      size_t written = Update.writeStream(client);

      if (written == contentLength) {
        Serial.println("Written : " + String(written) + " successfully");
      } else {
        Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?" );
        // retry??
        // execOTA();
        OTADownloading = false;
        return OTA_INCOMPLETE;
      }

      if (Update.end()) {
        Serial.println("OTA done!");
        if (Update.isFinished()) {
          Serial.println("Update successfully completed. Rebooting.");
          OTADownloading = false;
          return OTA_SUCCESS;
        } else {
          Serial.println("Update not finished? Something went wrong!");
          OTADownloading = false;
          return OTA_INCOMPLETE;
        }
      } else {
        Serial.println("Error Occurred. Error #: " + String(Update.getError()));
        OTADownloading = false;
        return OTA_ERROR;
      }
    } else {
      // not enough space to begin OTA
      // Understand the partitions and
      // space availability
      Serial.println("Not enough space to begin OTA");
      client.flush();
      OTADownloading = false;
      return OTA_NO_SPACE;
    }
  } else {
    Serial.println("There was no content in the response");
    
    client.flush();
    OTADownloading = false;
    return OTA_NO_CONTENT;
  }
  OTADownloading = false;
  return OTA_ERROR;

}


//-----Encryption Functions 

#define AES_KEY "a47b58b8c748b058e9991700aaf3adaf818cbb9e4bce4fc0a5da39441581f199"
#define AES_KEY_SIZE 32 // For AES-256
#define AES_BLOCK_SIZE 16
// AES key and IV (must match the encryption values)
unsigned char aes_key[AES_KEY_SIZE] = {};
unsigned char iv[AES_BLOCK_SIZE] = {};

/**
 * @brief Converts a hexadecimal string to a byte array
 *
 * @param hex_str The hexadecimal string to convert
 * @param byte_array The byte array to store the converted data
 * @param byte_array_size The size of the byte array
 *
 * @note The string length of hex_str must be even and not larger than byte_array_size * 2 
 */
void hexStrToByteArray(const char *hex_str, unsigned char *byte_array, size_t byte_array_size)
{
    size_t hex_str_len = strlen(hex_str);

    // Ensure the string length is even and not larger than the byte array size * 2
    if (hex_str_len % 2 != 0 || hex_str_len / 2 > byte_array_size)
    {
        ESP_LOGE("HEXtoSTR","Invalid hex string length\n");
        return;
    }

    for (size_t i = 0; i < hex_str_len; i += 2)
    {
        // Convert each pair of hex characters into a byte
        char byte_str[3] = {hex_str[i], hex_str[i + 1], '\0'};
        byte_array[i / 2] = (unsigned char)strtol(byte_str, NULL, 16);
    }
}



/**
 * @brief Decrypts a password encrypted with AES-256 in CBC mode
 *
 * @param[in] encrypted_password The encrypted password to decrypt
 * @param[in] encrypted_len The length of the encrypted password
 * @param[out] decrypted_password The decrypted password
 *
 * @note The IV used for decryption must be the same as the one used for encryption
 * @note The decrypted password is stored in-place in the decrypted_password array
 */
void aes_decrypt(unsigned char *encrypted_password, size_t encrypted_len, unsigned char *decrypted_password)
{
    mbedtls_aes_context aes;
    unsigned char iv_copy[AES_BLOCK_SIZE];

    mbedtls_aes_init(&aes);

    // Set the decryption key
    mbedtls_aes_setkey_dec(&aes, aes_key, AES_KEY_SIZE * 8); // AES-256 uses a 256-bit key

    // Make a copy of the IV because mbedTLS modifies the IV during decryption
    memcpy(iv_copy, iv, AES_BLOCK_SIZE);

    // Decrypt the encrypted password in-place
    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, encrypted_len, iv_copy, encrypted_password, decrypted_password);

    // Clean up
    mbedtls_aes_free(&aes);
}

/**
 * @brief Decrypts a password using AES-256 in CBC mode
 *
 * @param[in] strAesKey The AES-256 key to use for decryption
 * @param[in] fullCipher The full cipher text to decrypt
 * @param[out] decryptedPassword The decrypted password
 * @return true if decryption was successful, false otherwise
 *
 * @note The IV used for decryption must be the same as the one used for encryption
 * @note The decrypted password is stored in-place in the decrypted_password array
 * @note Input Password Length must be <=16 characters
 * @note Input CIPHER should have the following format: AES256-<IV>-<CIPHER>
 */
bool decryptPassword(const char* strAesKey, const char* fullCipher, char *decryptedPassword)
{
    // first check if password is Encrypted using Metadata
    char aesPrefix[6];
    strncpy(aesPrefix, fullCipher, 6);
    if (strncmp(aesPrefix, "AES256", 6) != 0)
    {

        // return logic for password
        return false;
    }

    // now start decrypting the password

    char strCipher[65] = {0}; // Assuming a maximum length of 64
    char strIV[33] = {0};     // Assuming a maximum length of 16

    char cipherCopy[100];
    strncpy(cipherCopy, fullCipher, sizeof(cipherCopy));

    // Tokenize by "-" delimiter
    char *token = strtok(cipherCopy, "-");

    // Skip the first part ("AES256")
    token = strtok(NULL, "-");
    if (token != NULL)
    {
        strncpy(strIV, token, 32); // Assuming a maximum of 16 characters for the IV
        strIV[33] = '\0';
    }

    // Get the IV (third part)
    token = strtok(NULL, "-");
    // Get the encrypted data (third part)
    if (token != NULL)
    {
        strncpy(strCipher, token, 64); // Assuming a maximum of 64 characters for the encrypted data
        strCipher[63] = '\0';
    }

    hexStrToByteArray(strAesKey, aes_key, AES_KEY_SIZE);
    hexStrToByteArray(strIV, iv, AES_BLOCK_SIZE);

    unsigned char encrypted_password[16] = {/* The encrypted password byte array */};

    hexStrToByteArray(strCipher, encrypted_password, 16);
    size_t encrypted_len = sizeof(encrypted_password);

    // Buffer for decrypted password
    unsigned char decrypted_password[256];
    memset(decrypted_password, 0, sizeof(decrypted_password));

    aes_decrypt(encrypted_password, encrypted_len, decrypted_password);

    // printf("Decrypted password: %s\n", decrypted_password);

    char lastByte = decrypted_password[encrypted_len - 1];
    //printf("Last byte %d\n", lastByte);
    if (lastByte > 0 && lastByte < 14)
    {
        for (int i = 0; i < encrypted_len - lastByte; i++)
        {
            decryptedPassword[i] = decrypted_password[i];
        }
        decryptedPassword[encrypted_len - lastByte] = '\0';
        return true;
    }
    else
    {
        ESP_LOGE("AES", "Decryption Failed");
        return false;
    }
}

