#include <MQTTClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <SSLClient.h>
SSLClient secureClientModem(&modemclient);
WiFiClientSecure net = WiFiClientSecure();
MQTTClient IoTclient = MQTTClient(16384, 256);
void CollectionIncoming(String ean);


/**
 * Sets MQTT topics based on the provided country code.
 * This function constructs various MQTT topic strings for publishing and subscribing, 
 * prefixing each topic with a base path determined by the given country.
 *
 * @param country The country code used to set the topic paths.
 *                This parameter is a `String` representing the country,
 *                which will be appended to a topic prefix for organized topic naming.
 *                Example: "US", "AE", etc.
 *
 * @note The generated topics are specific to various actions such as adding bottles, 
 *       collection completion, issue creation, and device calibration.
 * 
 * @example
 *   setTopicBasedOnCountry("ae");
 *   // Resulting topics will have the prefix: "Topic/Prod/US/"
 */
void setTopicBasedOnCountry(String country) {
    String prefix = "Topic/Prod/" + country + "/";
    Add_Bottle = prefix + "Add_Bottle";
    Collection_done = prefix + "Collection_done";
    CreateIssueTopic = prefix + "CreateIssue";
    UnknownBottleTopic = prefix + "UnknownBottle";
    FakedropTopic = prefix + "Fake_drop";
    BarcodeScannedTopic = prefix + "Barcode_Scanned";
    AWS_IOT_PUB_TOPIC = prefix + "pub";
    Scanned = prefix + "Scanned";
    MakeCollection = prefix + "Make-Collection";
    myServosub = prefix + "Servo";
    myServoCalibation = prefix + "Calibation";
    myLockSub = prefix + "Lock";
    myResetBinSub = prefix + "ResetBin";
    AWS_IOT_SUB_TOPIC = prefix + "sub";
    RegisterandDeploy = prefix + "Add_Bin";
    SensorFBTopic = prefix + "SensorFeedback";
    ResetWifiTopic = prefix + "ResetWifi";

}
/**
 * Performs an OTA update if a newer firmware version is available.
 * This function compares the current firmware version with the provided OTA version and
 * executes the OTA update process if they differ. It handles multiple retry attempts, 
 * logs the update progress, and updates stored firmware information upon success.
 *
 * @param BinFile The URL or path to the OTA binary file as a `String`.
 *                This file will be downloaded if an update is available.
 * @param Firmware The version name of the new firmware as a `String`.
 *                 This will be compared to the stored firmware version.
 * 
 * @return `true` if the OTA update is successful, `false` if the update fails or no update is available.
 * 
 * @note The function retries the update process up to two times in case of connection or 
 *       other transient issues. Upon a successful update, it stores the new version name 
 *       and link, performs a servo operation, and logs the event before restarting the device.
 *
 * @example
 *   bool success = getOtaUpdate("http://example.com/update.bin", "v1.2.3");
 *   if (success) {
 *     // Proceed with post-update tasks
 *   }
 */
bool getOtaUpdate(String BinFile, String Firmware) {
  int timeout = 0;
  const char *data_file = BinFile.c_str();
  const char *data_name = Firmware.c_str();
  mLogger.logMessage("getOTAUpdate", "Checking OTA update");
  g_OTAFlag=true;
  while (timeout < 2)
  { // check multiuple times
    mLogger.logInfo("Current Prefs Version Name",versionName);
    mLogger.logInfo("Current Prefs Link",versionLink);
    mLogger.logInfo("OTA Version Name",data_name);
    mLogger.logInfo("OTA Version link",data_file);
    if (versionName != String(data_name))
    {
      OTADownloading = true;
      IoTclient.disconnect();

      // ota logic along with error cases
      byte ret;
      if (!SimcomModem){ 
        ret = execOTA(getHostName(data_file), getFileName(data_file));
      }
      else {
        ret = SimComExecOTA(getHostName(data_file), getFileName(data_file));
      }
      switch (ret)
      {
      case OTA_SUCCESS:
        // discoverESP("download complete");
        mLogger.logMessage("execOTA","Download OTA Success !!, updating Prefs ...");
        prefs.begin("configs", false); // begin preferences
        vTaskDelay(50 / portTICK_PERIOD_MS); // storing OTA version
        prefs.putString("name", data_name);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        versionName = prefs.getString("name");
        vTaskDelay(50 / portTICK_PERIOD_MS);

        prefs.putString("link", data_file); // storing OTA link
        vTaskDelay(200 / portTICK_PERIOD_MS);
        versionLink = prefs.getString("link");
        vTaskDelay(50 / portTICK_PERIOD_MS);
        prefs.end();
        for (uint8_t i=0; i<4; i++){
        open_servo(servos[i],SOpenPos[i]);
        }
        for (uint8_t i=0; i<4; i++){
        close_servo(servos[i],SClosePos[i]);
        }
        mLogger.logMessage("execOTA", "Done, Restarting ESP");
        return true;
        // ESP.restart();
        // break;
      case OTA_INCOMPLETE:
        mLogger.logError("execOTA", "OTA_INCOMPLETE");
        g_OTAFlag=false;
        OTADownloading = false;
        return false;
      case OTA_NO_CONNECTION:
        mLogger.logError("execOTA", "OTA_NO_CONNECTION");
        g_OTAFlag=false;
        OTADownloading = false;
        return false;
      }
      // } else {
      //   mLogger.logMessage("getOTAUpdate", "No new OTA update");
      //   return true;
      // }
    }
    // else mLogger.logMessage("getOTAUpdate", client.getString());

    delay(1000); // wait for 1 sec before retrying
    timeout++;
  }
  mLogger.logError("execOTA", "Failed After 2 attemtps.");
  g_OTAFlag=false;
  OTADownloading = false;
  return false;
}
/**
 * Updates the stored card data by fetching new card information from the server.
 * This function saves the new card data to a file in SPIFFS after acquiring a write lock.
 * It also logs the process and displays the updated card data on the Serial monitor.
 *
 * @param CollectionCards A `String` containing the new card data to be stored.
 *                        This data will replace the existing card information.
 * 
 * @note The function waits for a read lock (`cards_read_lock`) to be released before 
 *       writing to prevent data conflicts. After updating, it releases the write lock (`cards_write_lock`).
 *       Logging is handled through `mLogger`, and the updated card data is printed to Serial.
 * 
 * @example
 *   get_cards("CardDataString");
 *   // New card data will be written to /cards.txt and printed to Serial.
 */
void get_cards(String CollectionCards) {
  cards = CollectionCards;
  mLogger.logInfo("get_cards", "updating Collection Cards....");
  cards_write_lock = true;
  deleteFile(SPIFFS, "/cards.txt");
  appendFile(SPIFFS, "/cards.txt", cards);
  mLogger.logInfo("get_cards", "Collection Cards File Updated !!");
  cards = readFromFile(SPIFFS, "/cards.txt");
  mLogger.logInfo("get_cards",cards);
  cards_write_lock = false;
}

/**
 * Reports device state changes to the device shadow by publishing a JSON payload.
 * This function serializes the `shadowInfoChangesMap` key-value pairs and embeds them within
 * a structured JSON payload that represents the reported state. The payload is then published 
 * to the `Shadow_Update` topic.
 *
 * @param shadowInfoChangesMap A `JsonObject` containing key-value pairs representing 
 *                             changes to the device's shadow state. Each pair in this map 
 *                             is added directly to the "Bin_info" section of the payload.
 * 
 * @note The function constructs a JSON document with the structure:
 *       `{ "state": { "reported": { "Bin_info": { ...shadowInfoChangesMap } } } }`.
 *       It serializes and publishes this payload to update the device shadow. 
 *       After publishing, it sets `IsPendingDelta` to `false`.
 * 
 * @example
 *   JsonObject changes;
 *   changes["temperature"] = 25;
 *   changes["status"] = "active";
 *   ReportToDevShadow(changes);
 *   // This will publish a payload with temperature and status under "Bin_info".
 */
void ReportToDevShadow(JsonObject shadowInfoChangesMap)
{
  mLogger.logInfo("ReportToDevShadow", "Report To device Shadow started");
  DynamicJsonDocument reportDoc(1024);
  JsonObject myState = reportDoc.createNestedObject("state");
  String statePayload;
  serializeJson(myState, statePayload);
  JsonObject reportedState = myState.createNestedObject("reported");
  JsonObject binInfo = reportedState.createNestedObject("Bin_info");
  
  // Iterate over the shadowInfoChangesMap and add its key-value pairs to binInfo
  for (JsonPair keyValue : shadowInfoChangesMap)
  {
    // Directly add key-value pairs to binInfo without converting to String
    binInfo[keyValue.key()] = keyValue.value();
  }
  String reportPayload;
  serializeJson(reportDoc, reportPayload);
  mLogger.logInfo("ReportToDevShadow", reportPayload);
  if (IoTclient.publish(Shadow_Update, reportPayload.c_str())) mLogger.logInfo("ReportToDevShadow", "Report To device Shadow Sucsess !!!");
}
/**
 * Restarts the bin by disconnecting from IoT, deleting a specific file, and restarting the ESP.
 * This function handles IoT disconnection, removes the `EAN.txt` file from SPIFFS, verifies the deletion,
 * and then performs a system restart.
 * 
 * @note The function waits briefly after each step to ensure processes complete. 
 *       It checks for successful file deletion and logs the outcome to Serial before restarting.
 *
 * @example
 *   restartBin();
 *   // Disconnects from IoT, deletes `/EAN.txt`, verifies deletion, and restarts the ESP.
 */
void restartBin(){
    mLogger.logWarning("restartBin","Restart the bin and Deleting EANs Files...");
    String compPath="";
    for (uint8_t i=0; i<4; i++){
      mLogger.logWarning("restartBin","inside the for loop");
      compPath= "/"+*compName[i]+".txt";
      if(SPIFFS.begin(true))deleteFile(SPIFFS, compPath.c_str());
      else mLogger.logWarning("SPIFFS","is not ready");
      vTaskDelay(50 / portTICK_PERIOD_MS);
      // Its checking if EAN.txt file exists in SPIFFS
      if(!SPIFFS.exists(compPath.c_str())) mLogger.logWarning("restartBin"," EANs File deleted successfully");
      else mLogger.logWarning("restartBin"," EANs File deleted successfully");
    }
    IoTclient.disconnect();
    vTaskDelay(250 / portTICK_PERIOD_MS);
    mLogger.logWarning("restartBin", " Restarting ESP Now.....");
    ESP.restart();
}
/**
 * This function searches for a placeholder substring within a larger string and replaces it 
 * with a specified replacement string, adjusting the rest of the characters in the string as needed.
 * 
 * @param str The original string, containing the placeholder to be replaced. 
 *            This is a character array with a fixed maximum length.
 * @param maxLength The maximum length of the `str` buffer to prevent overflow.
 * @param placeholder The substring to search for within `str`.
 *                    If found, this will be replaced by `replacement`.
 * @param replacement The string to replace the placeholder with.
 *                    It can be shorter, equal in length, or longer than the placeholder.
 * 
 * @note If `replacement` is longer than `placeholder`, the function ensures there is enough space 
 *       in `str` (using `maxLength`) before performing the replacement. It shifts characters as needed 
 *       to fit the new string, preventing buffer overflows.
 * 
 * @example
 *   char message[50] = "Hello, [name]!";
 *   replacePlaceholder(message, 50, "[name]", "Alice");
 *   // Result: message becomes "Hello, Alice!"
 */
void replacePlaceholder(char* str, int maxLength, const char* placeholder, const char* replacement) {
  char* found = strstr(str, placeholder);  // Find the placeholder in the string

  if (found != NULL) {  // If the placeholder is found
    int placeholderLength = strlen(placeholder);
    int replacementLength = strlen(replacement);
    int shift = replacementLength - placeholderLength;

    if (shift > 0 && strlen(str) + shift < maxLength) {  // Check if there is enough space for the replacement
      // Shift the characters to make space for the replacement
      memmove(found + replacementLength, found + placeholderLength, strlen(found + placeholderLength) + 1);
    } else if (shift < 0) {
      // Shift the characters to make space for the replacement
      memmove(found + replacementLength, found + placeholderLength, strlen(found + placeholderLength) + 1);
    }

    // Copy the replacement into the placeholder's position
    strncpy(found, replacement, replacementLength);
  }
}
/**
 * Prepares the IoT topics by including the device's Thing ID (chip ID) into the relevant topic strings.
 * This function generates unique topic names for subscribing and publishing based on the device's ID 
 * and updates placeholders in the topic names for Shadow and update-related operations.
 *
 * @note This function constructs a series of topic strings, each incorporating the Thing ID (`chipId`),
 *       and stores them in corresponding global variables. The function also replaces placeholders in 
 *       predefined topics (`Shadow_Get`, `Get_Accept`, etc.) with the `Bin_ID`. 
 *       It uses the `replacePlaceholder` function to replace `{myThingname}` with the device's unique ID.
 *
 * @example
 *   PrepairIoTTopics();
 *   // This will prepare the topics with the device-specific Thing ID, and 
 *   // the updated topics will be ready for use in IoT communication.
 */
void PrepairIoTTopics(){// this function to include the thing ID to the Device Shadow topics/ 
  String subscriptionTopic = String(AWS_IOT_SUB_TOPIC) + "_" + Bin_ID;
  String publishTopic = String(AWS_IOT_PUB_TOPIC) + "_" + Bin_ID;
  String subscriptionservo = String(myServosub) + "_" + Bin_ID;
  String subscriptionservoCalibation = String(myServoCalibation) + "_" + Bin_ID; // include the thing ID to Servo Calibation Shadow topics
  String subscriptionreset = String(myResetBinSub) + "_" + Bin_ID;
  String subscripScanned = String(Scanned) + "_" + Bin_ID; // topic to subscribe to get what ever scanned there is. 
  String subscripCollection = String (MakeCollection) + "_" + Bin_ID;
  String subscriptionlck = String(myLockSub) + "_" + Bin_ID;
  String subscriptionResetWifi = String(ResetWifiTopic) + "_" + Bin_ID;
  subscriptionTopic.toCharArray(SUBtopic, 50);
  publishTopic.toCharArray(PUBtopic, 50);
  subscriptionservo.toCharArray(ServoSub,50);
  subscriptionservoCalibation.toCharArray(ServoCalibation,50);
  subscriptionlck.toCharArray(LockSub,50);
  subscriptionreset.toCharArray(ResetBinSub,50);
  subscripScanned.toCharArray(ScannedTopic,50);
  subscripCollection.toCharArray(MakeCollectionTopic,50);
  subscriptionResetWifi.toCharArray(ResetWifiSub,50);
  replacePlaceholder(Shadow_Get,50,"{myThingname}", Bin_ID.c_str());
  replacePlaceholder(Get_Accept, 50, "{myThingname}", Bin_ID.c_str());
  replacePlaceholder(Get_Rejected, 50, "{myThingname}", Bin_ID.c_str());
  replacePlaceholder(Shadow_Update,50,"{myThingname}", Bin_ID.c_str());
  replacePlaceholder(Update_Accept, 50, "{myThingname}", Bin_ID.c_str());
  replacePlaceholder(Update_Rejected, 50, "{myThingname}", Bin_ID.c_str());
  replacePlaceholder(Update_Delta, 50, "{myThingname}", Bin_ID.c_str());
  mLogger.logMessage("PrepairIoTTopics"," Sucsess !");
}
/**
 * Registers the Device Shadow by publishing an initial, empty state with relevant device information.
 * This function is called the first time to initialize the Device Shadow with basic details of the device.
 * It publishes the device's state, including information such as the device's mode, PCB version, type, country,
 * sensors, and other configuration details to the IoT platform. 
 * 
 * @note This function prepares a JSON payload that contains the device's state, with fields like:
 *       - Device Mode
 *       - PCB Version
 *       - Bin Type
 *       - Country
 *       - Sensors Type
 *       - Firmware Version
 *       - OTA Link
 *       - Wi-Fi credentials (SSID and password), etc.
 *       It serializes this data and sends it as a message to the IoT platform via the `Shadow_Update` topic.
 *       The device shadow is initialized with an empty state, and this function is only called once for device registration.
 * 
 * @example
 *   registerDevShadow();
 *   // This will publish an initial Device Shadow state to the cloud with device-specific information.
 */
void registerDevShadow() { 
  String prefKey="";
  prefs.begin("configs", false);
  mLogger.logInfo("registerDevShadow"," Start Registering Device Shadow");
  StaticJsonDocument<1024> doc;
  JsonObject state_reported_Bin_info = doc["state"]["reported"].createNestedObject("Bin_info");
  state_reported_Bin_info["Device_Mode"] = deviceMode;
  state_reported_Bin_info["PCB_Version"] = PCBVersion;
  state_reported_Bin_info["MACaddress"] = GetMacAddress();
  state_reported_Bin_info["Bin_Type"] = BinType;
  state_reported_Bin_info["Country"] = devCountry;
  state_reported_Bin_info["Material"] = Material;
  state_reported_Bin_info["Sensors_Type"] = SensorType;
  state_reported_Bin_info["RGB"] = devRGB;
  state_reported_Bin_info["ICCID"] = "";
  state_reported_Bin_info["BIN_Area"] = "";
  state_reported_Bin_info["WMC"] = "";
  state_reported_Bin_info["Firmware_Version"] = versionName;
  state_reported_Bin_info["OTA_Link"] = versionLink;
  state_reported_Bin_info["Bin_Full"] = "";
  state_reported_Bin_info["Wifi_SSID"] = WifiSSID;
  state_reported_Bin_info["Wifi_Pass"] = WifiPass;
  state_reported_Bin_info["BottleCount"] = "";

  if (compartmentState[0])
  {
    float temUsage = 0.0;
    auto &muxV = compSensors[0].vSensor.muxID == 0 ? I2CMux : I2CMux_1;
    temUsage = getUsage(compSensors[0].vSensor.sensorName, compSensors[0].vSensor.sensor, muxV, compSensors[0].vSensor.channel, 0);
    state_reported_Bin_info["Usage_Small"] = String(temUsage);  
  }
  if (EwasteCompartmentState[1])
  { 
  float temUsage = 0.0;
  temUsage = getUsage(temUsage); // printVertiaclSensorReads(temUsage);
  state_reported_Bin_info["Usage_Big"] = String(temUsage);  
  }
  JsonObject compartments = state_reported_Bin_info.createNestedObject("compartments");
  for(uint8_t i=0; i<4; i++) compartments["comp"+String(i+1)] = *compName[i];

  JsonObject servoCloseAngle = state_reported_Bin_info.createNestedObject("ServoCloseAngle");

  for(uint8_t i=0; i<4; i++){
    if(compartmentState[i])servoCloseAngle[*compName[i]] = SClosePos[i]; 
    else servoCloseAngle[*compName[i]] = "";
  }

  JsonObject servoOpenAngle = state_reported_Bin_info.createNestedObject("ServoOpenAngle");

  for (uint8_t i = 0; i < 4; i++) {
      if (compartmentState[i]) {
          // Assign the value only if compName[i] is valid
          if (compName[i] != nullptr && !compName[i]->isEmpty()) {
              servoOpenAngle[compName[i]->c_str()] = SOpenPos[i]; // Use c_str() for JSON key
          }
      } else {
          // Clear the key value
          if (compName[i] != nullptr && !compName[i]->isEmpty()) {
              servoOpenAngle[compName[i]->c_str()] = nullptr; // Clear value in JsonObject
          }
      }
  }


  state_reported_Bin_info["QR_Code"] = QRCode;
  state_reported_Bin_info["Eans"] = "0";
  JsonArray state_reported_Bin_info_Cards = state_reported_Bin_info.createNestedArray("Cards");
  String jsonString;
  serializeJson(doc, jsonString);

  char charBuf[jsonString.length() + 1];
  jsonString.toCharArray(charBuf, jsonString.length() + 1);
  if (IoTclient.publish(Shadow_Update, charBuf,false,1)) mLogger.logInfo("PrepairIoTTopics","Registering Device sucsess !!!");
  else mLogger.logError("PrepairIoTTopics","Registering Device failed");
}

/**
 * Saves the provided certificate and private key data to the file system.
 * 
 * This function takes a `DynamicJsonDocument` containing the certificate and private key information,
 * extracts the values for the `certificatePem` and `privateKey` fields, and writes them to a file named 
 * `aws.json` in SPIFFS (Serial Peripheral Interface Flash File System).
 * The function also serializes the JSON data to the serial monitor for debugging purposes.
 * 
 * @param doc The `DynamicJsonDocument` containing the certificate and private key fields (`certificatePem` and `privateKey`).
 *            This document is expected to have the required certificate data in JSON format.
 * 
 * @note If the file cannot be opened for writing, an error message is printed to the serial monitor.
 *       The function will overwrite the existing `aws.json` file with the new certificate data.
 * 
 * @example
 *   DynamicJsonDocument certDoc(1024);
 *   certDoc["certificatePem"] = "-----BEGIN CERTIFICATE-----...";
 *   certDoc["privateKey"] = "-----BEGIN PRIVATE KEY-----...";
 *   saveCertificateToFS(certDoc);
 *   // This will save the certificate and private key to the file system as 'aws.json'.
 */
void saveCertificateToFS(DynamicJsonDocument doc) {
  DynamicJsonDocument pem(4000);
  pem["certificatePem"] = doc["certificatePem"];
  pem["privateKey"] = doc["privateKey"];
  File file = SPIFFS.open("/aws.json", "w");
  if (!file)
  {
    mLogger.logError("PrepairIoTTopics","failed to open config file for writing");
  }
  
  serializeJson(pem, Serial);
  serializeJson(pem, file);
  file.close();
}

/**
 * Registers a device (thing) to AWS IoT using a provisioning template and certificate ownership token.
 * 
 * This function generates a registration request with the device's certificate ownership token and
 * serial number, and then publishes the registration request to AWS IoT using the `IoTclient` MQTT client.
 * The request is sent in JSON format to the `$aws/provisioning-templates/ProvisionClaimTemp/provision/json` topic.
 * 
 * @param doc The `DynamicJsonDocument` containing the `certificateOwnershipToken` required for provisioning.
 *            This document should have the `certificateOwnershipToken` field containing the token.
 * 
 * @note The `THINGNAME` is constructed using the device's chip ID (`chipId`), and this value is used as
 *       the `SerialNumber` for the device registration.
 * 
 * @example
 *   DynamicJsonDocument certDoc(1024);
 *   certDoc["certificateOwnershipToken"] = "your_certificate_ownership_token";
 *   registerThing(certDoc);
 *   // This will send the provisioning request to AWS IoT with the provided certificate ownership token.
 */
void registerThing(DynamicJsonDocument doc) {
  mLogger.logInfo("registerThing","Start Registering Thing...");
  const char *certificateOwnershipToken = doc["certificateOwnershipToken"];
  DynamicJsonDocument reqBody(4000);
  reqBody["certificateOwnershipToken"] = certificateOwnershipToken;
  reqBody["parameters"]["SerialNumber"] = Bin_ID;
  char jsonBuffer[4000];
  serializeJson(reqBody, jsonBuffer);
  mLogger.logInfo("registerThing","Sending certificate...");
  IoTclient.publish("$aws/provisioning-templates/ProvisionClaimTemp/provision/json", jsonBuffer);
}

/**
 * @brief Handles incoming MQTT messages based on the topic, and performs various actions based on the payload.
 * 
 * This function listens to different topics and processes the payload accordingly. It handles tasks like 
 * registering certificates, updating device configurations, performing OTA updates, adjusting servo positions, 
 * and reporting device shadow changes.
 *
 * @param topic [in] The topic of the incoming message.
 * @param payload [in] The payload of the incoming message, which contains the data to be processed.
 */
void messageHandler(String &topic, String &payload) {
  mLogger.logInfo("messageHandler","MQTT message Recived");
  mLogger.logMessage("messageHandler Topic=",topic);
  // Create a JSON document
  DynamicJsonDocument shadowInfoChangesDoc(512);
  // Convert the document to a JsonObject
  JsonObject shadowInfoChangesMap = shadowInfoChangesDoc.to<JsonObject>();
  mLogger.logMessage("messageHandler", "Topic - 1");
  if (topic == "$aws/certificates/create/json/accepted")
  {
    DynamicJsonDocument doc(5120);
    deserializeJson(doc, payload);
    saveCertificateToFS(doc);
    registerThing(doc);
  }
  else if (topic == "$aws/provisioning-templates/ProvisionClaimTemp/provision/json/accepted")
  {
    mLogger.logMessage("registerThing","Register things successfully.");
    mLogger.logWarning("registerThing","Restart in 5s.");
    ProvisionDone = true;
    prefs.begin("configs", false);
    prefs.putBool("provisionStatus", ProvisionDone);
    prefs.end();
    delay(100);
    sleep(5);
    AWSConnecting=false;
    ESP.restart();
  }
  // else if (topic == Get_Accept) Serial.println("Shadow Get accepted received");
  else if (topic == Get_Rejected){
    mLogger.logError("Topic",Get_Rejected);
    mLogger.logMessage("Topic,Get_Rejected",payload);
  }
  else if (topic == Update_Accept) 
  {
    mLogger.logMessage("Topic",Update_Accept);
    if (!ShadowRegestred){// if devise shadow is not registred then this update accept feedback from publishing the device shadow first time.
      mLogger.logMessage("Device Shadow","Device Shadow Register is Done");
      ShadowRegestred=true;
      prefs.begin("configs", false);
      prefs.putBool("ShadowRegStatus", ShadowRegestred);
      prefs.end();
    }
  }
  else if (topic == Update_Rejected) Serial.println(payload);

  else if (topic == Update_Delta) { // for device shadow delta only. 
    mLogger.logInfo("topic","Delta received");
    mLogger.logMessage("delta",payload);
    DynamicJsonDocument doc(1024);
    deserializeJson(doc,payload);
    if (doc.containsKey("state")) {
      JsonObject state = doc["state"];
      if(state.containsKey("Bin_info")){
        JsonObject binInfo = state["Bin_info"];
        if (binInfo.containsKey("Cards")){
          JsonArray cards = binInfo["Cards"].as<JsonArray>(); // Extract the array of Cards
          Serial.println("Cards:");
          String cardsString;
          serializeJson(cards, cardsString); // Convert JSON array to string
          IOTCards = cardsString.c_str();    // Convert String to const char*
          get_cards(cardsString);
          shadowInfoChangesMap["Cards"] = cards;
          for (JsonVariant card : cards){
            mLogger.logMessage("Collection Cards Received",card.as<String>());
          }
        }

       
        if (binInfo.containsKey("OTA_Link")){
          String FirmwareBinFile = binInfo["OTA_Link"]; // Extract URL of the firmware 
          String FirmwareVersion = binInfo["Firmware_Version"];
          Serial.println("FirmwareVersion " +FirmwareVersion);
          if (!FirmwareVersion.isEmpty() && FirmwareVersion.startsWith("V3")){
            if (getOtaUpdate(FirmwareBinFile, FirmwareVersion)){
              mLogger.logMessage("getOtaUpdate","OTA Update Done");
              prefs.begin("configs", false);
              vTaskDelay(50 / portTICK_PERIOD_MS);
              prefs.putString("name", FirmwareVersion);
              prefs.putString("link", FirmwareBinFile);
              vTaskDelay(50 / portTICK_PERIOD_MS);
              prefs.end();
              // shadowInfoChangesMap["Material"] = IOTMaterial;
              shadowInfoChangesMap["OTA_Link"] = FirmwareBinFile;
              shadowInfoChangesMap["Firmware_Version"] = FirmwareVersion;
              ReportToDevShadow(shadowInfoChangesMap);
              mLogger.logMessage("getOtaUpdate","Restarting Bin");
              vTaskDelay(2000 / portTICK_PERIOD_MS);
              ESP.restart();
            }
            else return;
          } else return;
        }
        if (binInfo.containsKey("Device_Mode")) {
          IOTDevice_Mode = binInfo["Device_Mode"];
          deviceMode=IOTDevice_Mode;
          prefs.begin("configs", false);
          prefs.putString("devMode", deviceMode);
          prefs.end();
          // execute the device mode so the bin will use it to do its functions. 
          shadowInfoChangesMap["Device_Mode"] = IOTDevice_Mode;
        }
        if (binInfo.containsKey("PCB_Version")) {
          IOTPCB_Version = binInfo["PCB_Version"];
          prefs.begin("configs", false);
          prefs.putString("PCB_Version",IOTPCB_Version);
          prefs.end();
          shadowInfoChangesMap["PCB_Version"] = IOTPCB_Version;
        }
        if (binInfo.containsKey("Bin_Type")) {
          IOTBin_Type = binInfo["Bin_Type"];
          prefs.begin("configs", false);
          prefs.putString("Bin_Type", IOTBin_Type);
          prefs.end();
          shadowInfoChangesMap["Bin_Type"] = IOTBin_Type;
        }
        if (binInfo.containsKey("Country")) {
          IOTCountry = binInfo["Country"];
          prefs.begin("configs", false);
          prefs.remove("Country");
          vTaskDelay(50 / portTICK_PERIOD_MS);
          prefs.putString("Country", IOTCountry);
          prefs.end();
          vTaskDelay(500 / portTICK_PERIOD_MS);
          restartBin();
        }
        if (binInfo.containsKey("Material")) {
          IOTMaterial = binInfo["Material"];
          prefs.begin("configs", false);
          prefs.remove("Material");
          vTaskDelay(50 / portTICK_PERIOD_MS);
          prefs.putString("Material", IOTMaterial);
          prefs.end();
          // shadowInfoChangesMap["Material"] = IOTMaterial;
          vTaskDelay(500 / portTICK_PERIOD_MS);
          restartBin();
        }
        if (binInfo.containsKey("Sensors_Type")) {
          IOTSensors_Type = binInfo["Sensors_Type"];
          prefs.begin("configs", false);
          prefs.putString("Sensors_Type", IOTSensors_Type);
          prefs.end();
          shadowInfoChangesMap["Sensors_Type"] = IOTSensors_Type;
        }
        if (binInfo.containsKey("RGB")) {
          IOTRGB = binInfo["RGB"];
          prefs.begin("configs", false);
          prefs.putBool("RGB",IOTRGB);
          prefs.end();
          // exwcute function to Switch off RGB if required 
          shadowInfoChangesMap["RGB"] = IOTRGB;
        }
        if (binInfo.containsKey("ICCID")) {
          IOTICCID = binInfo["ICCID"];
          prefs.begin("configs", false);
          prefs.putString("RGB",IOTICCID);
          prefs.end();
          shadowInfoChangesMap["ICCID"] = IOTICCID;
        }
        if (binInfo.containsKey("BIN_Area")) {
          IOTBIN_Area = binInfo["BIN_Area"];
          prefs.begin("configs", false);
          prefs.putString("BIN_Area", IOTBIN_Area);
          prefs.end();
          shadowInfoChangesMap["BIN_Area"] = IOTBIN_Area;
        }
        if (binInfo.containsKey("WMC")) {
          IOTWMC = binInfo["WMC"];
          prefs.begin("configs", false);
          prefs.putString("WMC",IOTWMC);
          prefs.end();
          shadowInfoChangesMap["WMC"] = IOTWMC;
        }
        if (binInfo.containsKey("QR_Code")) {
          IOTQR_Code = binInfo["QR_Code"];
          prefs.begin("configs", false);
          prefs.putString("QR_Code",IOTQR_Code);
          prefs.end();
          // Store the QR Code, and report it. OR delete it here and put in getQR function after getting it report it to DS.
          shadowInfoChangesMap["QR_Code"] = IOTQR_Code;
        }
        if (binInfo.containsKey("Wifi_SSID")) {
          IOTWifi_SSID = binInfo["Wifi_SSID"];
          prefs.begin("configs", false);
          prefs.putString("ssid", IOTWifi_SSID);
          prefs.end();
          // Connect to the new Wifi SSID-Pass.
          shadowInfoChangesMap["Wifi_SSID"] = IOTWifi_SSID;
        }
        if (binInfo.containsKey("Wifi_Pass")) {
          IOTWifi_Pass = binInfo["Wifi_Pass"];
          prefs.begin("configs", false);
          prefs.putString("password", IOTWifi_Pass);
          prefs.end();
          //Connect to the new Wifi SSID-Pass.
          shadowInfoChangesMap["Wifi_Pass"] = IOTWifi_Pass;
        }
        if (binInfo.containsKey("compartments"))
        {
          JsonObject iotCompartments = binInfo["compartments"].as<JsonObject>();
          prefs.begin("configs", false);
          String prefkey="";
          for (uint8_t i=0; i<4; i++){
            if(iotCompartments["comp"+String(i+1)].as<String>()!="null"){
              *compName[i]=iotCompartments["comp"+String(i+1)].as<String>();
              shadowInfoChangesMap["compartments"]["comp"+String(i+1)] = *compName[i];
              prefkey = "comp" + String(i+1);
              prefs.putString(prefkey.c_str(), *compName[i]);
            }
          }
          prefs.end();
          for (uint8_t i=0; i<4; i++)
          ReportToDevShadow(shadowInfoChangesMap);
          vTaskDelay(500 / portTICK_PERIOD_MS);
          ESP.restart();
        }
        ReportToDevShadow(shadowInfoChangesMap);
      }
    }
  }
  else if (topic == ServoSub)
  {
    diagnoseSensors();

    String prefKey = "";
    prefs.begin("configs", false);
    mLogger.logMessage("ServoSub", "Testing Servo Open/Close");
    findingCurrrentThreshoulds();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    for (uint8_t i = 0; i < 4; i++)
    {
      Serial.println("comp state: " + String(compartmentState[i]) + " , " + String(compartIusse[i]));
      if (compartmentState[i] || compartIusse[i]) // && !RegisterDone)
        calibrateServo(i);
    }
    for (uint8_t i = 0; i < 4; i++)
    {
      prefKey = "servoClose" + String(i);
      SClosePos[i] = prefs.getInt(prefKey.c_str(), SClosePos[i]);
      prefKey = "servoOpen" + String(i);
      SOpenPos[i] = prefs.getInt(prefKey.c_str(), SClosePos[i] + 75); // 75 is the angle from opening to closing of Lid
      servos[i]->write(SClosePos[i]);
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    servoTesting(); // servo testing
  }
  else if (topic == ServoCalibation)
  {
    Serial.println(payload);
    DynamicJsonDocument doc(256);
    deserializeJson(doc, payload);
    String prefKey="";
    if (doc.containsKey("type") && doc.containsKey("action"))
    {
      String type = doc["type"].as<String>();
      String action = doc["action"].as<String>();
      if(action.indexOf("open")!=-1) isSOmodifed = true;
      if(action.indexOf("close")!=-1) isSCmodifed = true;
      for (uint8_t i = 0; i < 4; i++)
      {
        if (type == *compName[i])
        {
          if (action == "closeP")
          {
            if (SClosePos[i] >= 30)
              SClosePos[i] = 30;
            SClosePos[i] -= 1;
            if (SClosePos[i] <= 0)
              SClosePos[i] = 0;
            close_servo(servos[i], SClosePos[i]);
          }
          else if (action == "closeN")
          {
            SClosePos[i] += 1;
            if (SClosePos[i] >= 40)
              SClosePos[i] = 40;
            close_servo(servos[i], SClosePos[i]);
          }
          else if (action == "openP")
          {
             if (SOpenPos[i] <= 110)
              SOpenPos[i] = 110;
            SOpenPos[i] += 5;
            if (SOpenPos[i] >= 120)
              SOpenPos[i] = 120;
            open_servo(servos[i], SOpenPos[i]);
          }
          else if (action == "openN")
          {
            mLogger.logWarning("openN", "dicrease open recived");
            SOpenPos[i] -= 5;
            if (SOpenPos[i] <= 70)
              SOpenPos[i] = 70;
            open_servo(servos[i], SOpenPos[i]);
          }
          else if (action == "openT")
          {
            mLogger.logWarning("openT", "Test open position");
            open_servo(servos[i], SOpenPos[i]);
          }
          else if (action == "closeT")
          {
            mLogger.logWarning("closeT", "Test close position");
            close_servo(servos[i], SClosePos[i]);
          }
          else if (action == "zero")
          {
            mLogger.logWarning("closeT", "Test close position");
            zero_servo(servos[i]);
          }
          else if (action == "save")
          {
            mLogger.logWarning("ServoCalibation", "Positions Saving...");
            prefs.begin("configs", false);
            if(isSOmodifed){
              prefKey = "servoOpen" + String(i);
              prefs.putInt(prefKey.c_str(), SOpenPos[i]);
              shadowInfoChangesMap["ServoOpenAngle"][*compName[i]] = SOpenPos[i];
              isSOmodifed = false;
               mLogger.logWarning("ServoCalibation", "Open: " + String(prefs.getInt("servoOpen" + char(i))) +"," + String(isSOmodifed));
              }
            if(isSCmodifed){
              prefKey = "servoClose" + String(i);
              prefs.putInt(prefKey.c_str(), SClosePos[i]);
              shadowInfoChangesMap["ServoCloseAngle"][*compName[i]] = SClosePos[i];
              isSCmodifed = false;
              mLogger.logWarning("ServoCalibation", "Close: " + String(prefs.getInt("servoClose" + char(i))) +"," + String(isSCmodifed));
              }
            prefs.end();
            ReportToDevShadow(shadowInfoChangesMap);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            for (uint8_t i = 0; i < 4; i++)
            close_servo(servos[i], SClosePos[i]); // close the servo after calibration is done in case it was opened.
            return;
          }
        }
      }
    }
  }
  else if (topic == LockSub)
  {
    turnLock(HIGH);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    turnLock(LOW);
  }
  else if (topic == MakeCollectionTopic)
  {
    mLogger.logMessage("MakeCollectionTopic", payload);
    IoTEAN = payload;
    CollectionIncoming(IoTEAN);
  }
  else if (topic == ScannedTopic)
  { 
    //  for (uint8_t i = 0; i < 2; i++)
    //   {
    //     if (compartmentState[0] || EwasteCompartmentState[1])
    //     { 
    //       // Material == *compName[i];
    //       mLogger.logInfo("ScannedTopic Material", *compName[i]);
    //       auto &muxV = compSensors[i].vSensor.muxID == 0 ? I2CMux : I2CMux_1;
    //       if(!i)isBinFull[0] = checkBinFull(getUsage(compSensors[i].vSensor.sensorName, compSensors[i].vSensor.sensor, muxV, compSensors[i].vSensor.channel, i), i);
    //       else if (i==1){
    //       float tempUsage=0.00;
    //       isBinFull[1] = checkBinFull(getUsage(tempUsage), i);
    //       mLogger.logInfo("ScannedTopic",String(isBinFull[i]));
    //       }
    //       taskYIELD();
    //     }
    //   }
    mLogger.logMessage("ScannedTopic", payload);
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);
    IoTEAN = doc["ean"].as<String>();
    String curType = doc["type"].as<String>();
    Material = curType;
    if (curType == "ewaste-small")
      materialIncoming(curType);
    else if (curType == "ewaste-big")
    {
      if(isBinFull[1]) {
        mLogger.logInfo("ewaste-big", "is full");
        return;}
      uint8_t i = 0;
      mLogger.logInfo("ewaste-big, handleDropDetection", "Started");
      incomingEwaste = true;
      turn_Lock(lockPins[1], HIGH);
      ewasteBigTime = millis();
      handleDropDetection(i);
      turn_Lock(lockPins[1], LOW);
      incomingEwaste = false;
      // }
    }
    for (uint8_t i = 0; i < 2; i++)
      {
        // if (compVaildation(i))
        if (compartmentState[0] || EwasteCompartmentState[1])
        { 
          // Material == *compName[i];
          auto &muxV = compSensors[i].vSensor.muxID == 0 ? I2CMux : I2CMux_1;
          if(!i){
            isBinFull[0] = checkBinFull(getUsage(compSensors[i].vSensor.sensorName, compSensors[i].vSensor.sensor, muxV, compSensors[i].vSensor.channel, i), i);
            mLogger.logInfo("message handler (ScannedTopic)","Checking Bin Full for comp Small" + String(isBinFull[0]));
          }
          //   else if (i==1){
          // float tempUsage=0.00;
          // isBinFull[1] = checkBinFull(getUsage(tempUsage), i);
          // mLogger.logInfo("message handler (ScannedTopic)","Checking Bin Full for comp Large" + String(isBinFull[i]));
          // }
          taskYIELD();
        }
      }
  }
  else if (topic == ResetBinSub)
  {
    // ClearStorage();
    //     mLogger.logMessage("MakeCollecTopic", payload);
    // IoTEAN = payload;
    // CollectionIncoming(IoTEAN);
    restartBin();
  }
  else if (topic == ResetWifiSub)
  {
    mLogger.logMessage("topic", "ResetWifi");
    prefs.begin("configs", false); // begin preferences
    prefs.putString("ssid", "");
    prefs.putString("password", "");
    prefs.end();
    // reboot
    ESP.restart();
  }
}


/**
 * @brief Establishes a secure connection to AWS IoT using the provided certificate.
 * 
 * This function connects to the AWS IoT endpoint and subscribes to various topics.
 * It configures the connection settings depending on whether a SimcomModem is used or not.
 * It also prepares the device's IoT topics and starts a heartbeat timer.
 *
 * @param cert The certificate data containing the certificate and private key for the device.
 *             The certificate should be a `DynamicJsonDocument` containing:
 *             - `certificatePem`: The device's certificate as a PEM-encoded string.
 *             - `privateKey`: The device's private key as a PEM-encoded string.
 * 
 * @returns {bool} Returns `true` if the connection to AWS IoT is successful and topics are subscribed,
 *                 or `false` if the connection fails.
 * 
 * @note The function initializes the AWS IoT client, subscribes to several topics, and starts a heartbeat timer.
 *       If the device is not connected to AWS IoT within the timeout period, it will return `false`.
 */
bool connectToAWS(DynamicJsonDocument cert)
{
  if(!SimcomModem){
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(cert["certificatePem"]);
    net.setPrivateKey(cert["privateKey"]);
    IoTclient.setKeepAlive(60);
    IoTclient.begin(AWS_IOT_ENDPOINT, 8883,net);
  }
  else {
    secureClientModem.setCACert(AWS_CERT_CA);
    secureClientModem.setCertificate(cert["certificatePem"]);
    secureClientModem.setPrivateKey(cert["privateKey"]);
    IoTclient.setKeepAlive(90);
    IoTclient.begin(AWS_IOT_ENDPOINT, 8883,secureClientModem);
  }
  PrepairIoTTopics();
  IoTclient.onMessage(messageHandler);
  mLogger.logInfo("connectToAWS", "Connecting to AWS IOT.");
  IoTclient.connect(Bin_ID.c_str());
  if (!IoTclient.connected())
  {
    mLogger.logError("connectToAWS", "Timeout!");
    mLogger.logError("IoTclient.connected last error:", String(IoTclient.lastError()));
    AWSConnecting=false;
    return false;
  }
  mLogger.logInfo("connectToAWS", "Connected !!!");
  IoTclient.subscribe(SUBtopic);
  IoTclient.subscribe(ServoSub);
  IoTclient.subscribe(ServoCalibation);
  IoTclient.subscribe(LockSub);
  IoTclient.subscribe(ResetBinSub);
  IoTclient.subscribe(ResetWifiSub);
  IoTclient.subscribe(Get_Accept);
  IoTclient.subscribe(Get_Rejected);
  IoTclient.subscribe(Update_Accept);
  IoTclient.subscribe(Update_Rejected);
  IoTclient.subscribe(Update_Delta);
  IoTclient.subscribe(ScannedTopic);
  IoTclient.subscribe(MakeCollectionTopic);
  if (!ShadowRegestred) registerDevShadow();
  mLogger.logMessage("Subscription topic", SUBtopic);
  mLogger.logMessage("Subscription topic", ServoSub);
  mLogger.logMessage("Subscription topic", ServoCalibation);
  mLogger.logMessage("Subscription topic", LockSub);
  mLogger.logMessage("Subscription topic", ResetBinSub);
  mLogger.logMessage("Subscription topic", ResetWifiSub);
  mLogger.logMessage("Subscription topic", ScannedTopic);
  mLogger.logMessage("Subscription topic", MakeCollectionTopic);
  mLogger.logMessage("Subscription topic", RegisterandDeploy);
  mLogger.logMessage("Subscription topic", "Subscribe To Topics Done !!!!");
  vTaskDelay(500 / portTICK_PERIOD_MS);
  if (IoTclient.publish(PUBtopic,"ping",false,1)) mLogger.logMessage("Ping","Sucsess!");
  registerDevShadow();
  AWSCOnnectivity=true;
  AWSConnecting=false;
  return true;
}
/**
 * @brief Creates and provisions a new certificate for the device on AWS IoT.
 * 
 * This function initializes the AWS IoT connection with the provided certificate and private key,
 * and subscribes to the relevant topics to handle certificate creation and provisioning events.
 * It publishes a request to create a new certificate on the AWS IoT endpoint and monitors the response.
 * If the connection to AWS IoT fails, the ESP32 is restarted.
 *
 * @returns {void} This function does not return any value.
 * 
 * @note The function subscribes to topics related to certificate creation and provisioning.
 *       It will restart the ESP32 if the connection attempt to AWS IoT times out.
 */
void createCertificate()
{
  mLogger.logInfo("createCertificate", "Creating Certifaces Started...");
  AWSConnecting=true;
  if (!SimcomModem){
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);
    IoTclient.setKeepAlive(60);
    IoTclient.begin(AWS_IOT_ENDPOINT, 8883,net);
  }
  else {
    secureClientModem.setCACert(AWS_CERT_CA);
    secureClientModem.setCertificate(AWS_CERT_CRT);
    secureClientModem.setPrivateKey(AWS_CERT_PRIVATE);
    IoTclient.setKeepAlive(60);
    IoTclient.begin(AWS_IOT_ENDPOINT, 8883,secureClientModem);
  }
  IoTclient.onMessage(messageHandler);
  mLogger.logMessage("createCertificate", "Connecting to AWS IOT");

  if (IoTclient.connect(Bin_ID.c_str())) {
    mLogger.logMessage("createCertificate", "Connected To AWS Success !!");
    AWSCOnnectivity=true;
  }
  else {
    mLogger.logError("createCertificate", "Connected To AWS failed !!");
    AWSConnecting=false;
    return;
  }
  mLogger.logMessage("createCertificate", "Connecting to AWS IOT Sucsess !!!");
  IoTclient.subscribe("$aws/certificates/create/json/accepted");
  IoTclient.subscribe("$aws/certificates/create/json/rejected");
  IoTclient.subscribe("$aws/provisioning-templates/ProvisionClaimTemp/provision/json/accepted");
  IoTclient.subscribe("$aws/provisioning-templates/ProvisionClaimTemp/provision/json/rejected");
  mLogger.logMessage("createCertificate", "Create certificate...");
  IoTclient.publish("$aws/certificates/create/json", "");
} 

/**
 * @brief Initiates the process of loading AWS IoT certificates and establishing a connection to AWS.
 * 
 * This function attempts to open a file containing AWS IoT certificates from the SPIFFS filesystem. If the file
 * is successfully opened and the JSON data is correctly deserialized, it checks for the presence of the 
 * "certificatePem" field. If the certificate is found, it attempts to connect to AWS IoT. The function returns 
 * `true` if the connection is successful, and `false` if any step fails.
 * 
 * @returns {bool} `true` if the connection to AWS IoT is successful, `false` otherwise.
 */
bool awsiotstart() {
  AWSConnecting=true;
  mLogger.logInfo("awsiotstart", "Started....");
  mLogger.logMessage("awsiotstart", "loading Certifaces");
  File file = SPIFFS.open("/aws.json", "r");
  mLogger.logMessage("awsiotstart", "SPIFFS.open");
  if (!file) {
    file.close();
    mLogger.logMessage("awsiotstart", "Failed to open file for reading");
    return false;
  }
  mLogger.logMessage("awsiotstart", "file");
  DynamicJsonDocument cert(5120);
  auto deserializeError = deserializeJson(cert, file);
  if (!deserializeError){
    if (cert["certificatePem"]){
      mLogger.logMessage("awsiotstart", "certificatePem success !!");
      if (connectToAWS(cert)) {
        mLogger.logMessage("awsiotstart", " success !!");
        file.close();
        return true;
      }
    }
  }
  file.close();
  mLogger.logMessage("awsiotstart", "file.close");
  mLogger.logError("awsiotstart", " failed !!");
  return false;
}