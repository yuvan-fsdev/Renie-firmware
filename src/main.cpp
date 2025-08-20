/*
  Firmware V3IoT2.05
  Developed by Renie R&D.
  initial Version 3 PCB firmware. 
  Fixed bugs:
  1. OTA Update throw WIfi is restarting or crashing fixed.

  Added Features:
  1. 

  Changes: 
  1. 

  Data Usage:
  1. Not Tested. 

  Test Notes: 
  1. 

  Checked and tested by R&D
  Date: 29/12/2024
*/
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <FastLED.h>
#include <WiFi.h>
#include <Adafruit_VL6180X.h>
#include "Adafruit_VL53L0X.h"
#include <HTTPClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include "TCA9548A.h"
#include <Adafruit_INA219.h>
// #include <INA3221.h>
#include <Beastdevices_INA3221.h>
#include <map>
#include <set>
#include <string>

#include "globals.h"
// #include "sensors.h"
#define DEBUG_LOG true // Set to false to disable logging
#include "Logger.h"
Logger mLogger;
#include "storage.h"
#include "A7670Modem.h"
#include "systemfunctions.h"
#include "inputoutputs.h"
#include "compartments.h"
#include "awsiotcore.h"
#include "connectivity.h"

Compartment comp1, comp2, comp3, comp4;
Compartment *comps[] = {&comp1, &comp2, &comp3, &comp4}; // pointer to compartments
TaskHandle_t taskNetworkManager;                         /*Network Manager Task */
TaskHandle_t taskScankManager;                           /*Scan Manager Task */
TaskHandle_t taskWifiConnect;                            /*Scan connect to Wifi */
TaskHandle_t taskModemConnect;                           /*Scan connect to Modem */
TaskHandle_t taskAWSConnect;                             /*Scan connect to AWS */
void scanManager(void *pvParameters);
void networkManager(void *pvParameters);

void CollectionIncoming(String ean)
{
  firstRunDone = false;
  float minThreshold, maxThreshold;
  float Min[4] = {0.0, 0.0, 0.0, 0.0};
  float Max[4] = {0.0, 0.0, 0.0, 0.0};
  float SmallCompUsage; 
  long TIMEOUT = 13000; // 3K is 1 min, 13K is 5 min
  int count = 0;
  mLogger.logMessage("CollectionIncoming", "Validating Card: " + ean);
  if (!validateCard(ean))
  { // if its invalid card dont proceed much further
    mLogger.logWarning("CollectionIncoming", "Invalid Card");
    return;
  }
  CollectionCardFlag = true;
  yield();
  turnLock(HIGH);
  isWaitingForCollection = true;
  bool collectionDone[2] = {false, false};

  if (compartmentState[0])
  {
    auto &muxV = (compSensors[0].vSensor.muxID == 0) ? I2CMux : I2CMux_1;
    SmallCompUsage = VAvrage(compSensors[0].vSensor.sensorName, compSensors[0].vSensor.sensor, muxV, compSensors[0].vSensor.channel);
    mLogger.logMessage("collectionIncoming", "small comp usage before collection" + String(SmallCompUsage));

    // std::pair<float *, float *> collect = updateThresholds(0);
    // Min[0] = float(collect.first[0]);
    // Max[0] = float(collect.second[0]);
    // mLogger.logMessage("minThreshold_updateThresholds: Min ",  String(0) + "," + String(Min[0]));
    // mLogger.logMessage("maxThreshold_updateThresholds: Max ",  String(0) + "," + String(Max[0]));
  }
  if (EwasteCompartmentState[1])
  {
    std::pair<float *, float *> collect = EWupdateThresholds(1);
    Min[1] = float(collect.first[1]);
    Max[1] = float(collect.second[1]);
    // mLogger.logMessage("minThreshold_updateThresholds: Min ", String(1) + " - " + String(Min[1]));
    // mLogger.logMessage("maxThreshold_updateThresholds: Max ", String(1) + " - " + String(Max[1]));
  }

  while (count < TIMEOUT)
  {
    vTaskDelay(5 / portTICK_PERIOD_MS);
    yield();
    if (AWSCOnnectivity) IoTclient.loop();
    if (count > 600)
      turnLock(LOW);
    count++;
  }
  if (count >= TIMEOUT)
  {
    mLogger.logInfo("CollectionIncoming", "Timeout finished");
    triggerCollection = !triggerCollection;
  }
  else
    mLogger.logInfo("CollectionIncoming", "Collection Card Rescanned");
  isWaitingForCollection = false;
  turnLock(LOW);
  if (compartmentState[0])
  {
    // mLogger.logMessage("minThreshold_2", String(Min));
    // mLogger.logMessage("maxThreshold_2", String(Max));
    collectionDone[0] = checkExitCondition(0, SmallCompUsage);
  }

  if (EwasteCompartmentState[1])
  {
    // mLogger.logMessage("minThreshold_2", String(Min));
    // mLogger.logMessage("maxThreshold_2", String(Max));
    collectionDone[1] = EWcheckExitCondition(1, Min[1], Max[1]);
  }

  if (compartmentState[0])
  {
    isWaitingForCollection = false;
    if (collectionDone[0])
    {
      mLogger.logMessage("CollectionIncoming", "collectionDone[0] Collection Done ---------------");
      // vTaskDelay(100 / portTICK_PERIOD_MS);
      turnLock(LOW);
      bottleCount = 0;
      ewasteCountBig=0;
      prefs.begin("configs", false);
      prefs.putBool("ewasteBig", ewasteCountBig);
      prefs.putBool("bot_count", bottleCount);
      prefs.end();
      prefs.putBool("collection", false);
      Material = *compName[0];
      if (collectionComplete(ean, timestamp))
        CollectionCardFlag = false;
    }
  }
  if (EwasteCompartmentState[1])
  {
    if (collectionDone[1])
    {
      mLogger.logMessage("CollectionIncoming", "collectionDone[1] Collection Done ---------------");
      // vTaskDelay(100 / portTICK_PERIOD_MS);
      turnLock(LOW);
      bottleCount = 0;
      ewasteCountBig=0;
      prefs.begin("configs", false);
      prefs.putBool("ewasteBig", ewasteCountBig);
      prefs.putBool("bot_count", bottleCount);
      prefs.end();
      prefs.putBool("collection", false);
      Material = *compName[1];
      if (collectionComplete(ean, timestamp))
        CollectionCardFlag = false;
      
    }
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
        mLogger.logInfo("Setup","Checking Bin Full for comp Small" + String(isBinFull[0]));
      }
        else if (i==1){
      float tempUsage=0.00;
      isBinFull[1] = checkBinFull(getUsage(tempUsage), i);
      mLogger.logInfo("Setup","Checking Bin Full for comp Large" + String(isBinFull[i]));
      }
      taskYIELD();
    }
  }
  CollectionCardFlag = false;
}

/**
 * @brief Scans and validates EAN codes, directing the process based on the product type.
 *
 * This function continuously scans incoming EAN codes and validates them against
 * predefined compartments (`comp1`, `comp2`, `comp3`). The EAN code is checked to
 * determine which compartment (e.g., pet bottles, cans, etc.) it belongs to.
 * Based on the validation result, the appropriate compartment's handling
 * handles the situation accordingly.
 *
 * The function runs in an infinite loop, which implies it is designed to operate
 * as part of a real-time system or embedded application.
 *
 * @param pvParameters - A pointer to any parameters passed to the task (not used in this implementation).
 *
 * @note This function assumes that `readIncomingEan()` and the `validate` methods
 *       of `comp1`, `comp2`, and `comp3` are properly defined elsewhere in the program.
 */
void scanManager(void *pvParameters)
{
  while (true)
  {
    if (!BarSanner.available())
    {
      float current = 0.000;
      // timestamp = getUnixTimestamp();
      if (CupIncoming)
        continue;
      for (int i = 0; i < 4; i++)
      {   NUM_LEDS=(i==1)?8:16;
        if (compartmentState[i]|| i==1)
        {
          if (isCaptiveOn)
            movingColorEffect(RGBcomp[i], CRGB::DarkCyan, NUM_LEDS);
          else if (incomingEwaste)
          {
            // BigEwbottleDropEffect(RGBcomp[1], NUM_LEDS);
            // movingColorEffect(RGBcomp[1], CRGB::Green, NUM_LEDS);
            if (loadingCounter < 1200)
              loadingEffect(RGBcomp[1], g_ledStateCounter++, CRGB::Green, 5);
               vTaskDelay(300 / portTICK_PERIOD_MS); 
            loadingCounter++;
            if (loadingCounter >= 1200)
              loadingCounter = 0;
            continue;
          }
          else if (CollectionCardFlag)
          {
            collectionEffect();
            yield();
            continue;
          }
          else if (servoLimit[i])
            movingColorEffect(RGBcomp[i], CRGB::DarkBlue, NUM_LEDS);
          else if (isBinFull[i])
          {
            binfulleffect(RGBcomp[i], NUM_LEDS);
            yield();
            continue; // Skip other effects when bin is full
          }
          else if (g_OTAFlag)
            pulsingColorEffect(RGBcomp[i], CRGB::Purple, NUM_LEDS);
          else if (!internetConnected && !AWSCOnnectivity)
            pulsingGreyEffectS(RGBcomp[i], NUM_LEDS);
          else if (internetConnected && !AWSCOnnectivity)
          {
            if (loadingCounter < 1200)
              loadingEffect(RGBcomp[i], g_ledStateCounter++, CRGB::Yellow, NUM_LEDS);
            loadingCounter++;
            if (loadingCounter >= 1200)
              loadingCounter = 0;
          }
          else if (internetConnected && AWSCOnnectivity)
            breathingBrandColors(RGBcomp[i], NUM_LEDS);
        }
        else if (compartIusse[i])
          pulsingColorEffect(RGBcomp[i], CRGB::Orchid,NUM_LEDS);
        // current = (i < 3) ? ina3221.getCurrent(INA3221Channels[i]) : ina219.getCurrent_mA();
        // Serial.println("Current " +String(i) + " , "+String(current));
      }
      // if(ewasteBigTime-180000>millis()){
      //   for(int i = 1; i <4; i++){
      //     turn_Lock(lockPins[i], LOW);
      // }}
      // if(!isEANdownloading && AWSCOnnectivity) ESP.restart();
      yield();
      // vTaskDelay(100 / portTICK_PERIOD_MS);
    continue;
    }
    else
    {
      for (uint8_t i = 0; i < 4; i++)
      {
        if (compartmentState[0])
        {
          // Material == *compName[i];
          mLogger.logInfo("material in scanManager", *compName[i]);
          auto &muxV = compSensors[i].vSensor.muxID == 0 ? I2CMux : I2CMux_1;
          isBinFull[i] = checkBinFull(getUsage(compSensors[i].vSensor.sensorName, compSensors[i].vSensor.sensor, muxV, compSensors[i].vSensor.channel,i), i);
          yield();
        }
      }
      String ean = "";
      ean = readIncomingEan();
      if (isCollectionCard(ean))
      {
        firstRunDone = false;
        float minThreshold, maxThreshold;
        float Min[4] = {0.0, 0.0, 0.0, 0.0};
        float Max[4] = {0.0, 0.0, 0.0, 0.0};
        int TIMEOUT = 3000; // 300  seconds 5 min
        int count = 0;
        // isBinFull = checkBinFull(getUsage());
        mLogger.logMessage("scanManager", "Validating Card: " + ean);
        if (!validateCard(ean))
        { // if its invalid card dont proceed much further
          mLogger.logWarning("scanManager", "Invalid Card");
            bool isValid = false;
          if(comp1.validate(ean) || comp2.validate(ean) || comp3.validate(ean) || comp4.validate(ean))
          {
            triggerCollection = !triggerCollection;
            mLogger.logWarning("isCollectionCard", "Ean scenned");
          }
          continue;
        }
        CollectionCardFlag = true;
        mLogger.logMessage("scanManager", "Card Valid");
        yield();
        turnLock(HIGH);
        isWaitingForCollection = true;
        bool collectionDone[4] = {false, false, false, false};
        mLogger.logMessage("scanManager", "CisWaitingForCollection");
     
          if (compartmentState[0])
          {
            std::pair<float *, float *> collect = updateThresholds(0);
            Min[0] = float(collect.first[0]);
            Max[0] = float(collect.second[0]);
            // auto [minThreshold, maxThreshold] = updateThresholds(i);
            // Min = float(minThreshold[i]);
            // Max = float(maxThreshold[i]);
            mLogger.logMessage("minThreshold_updateThresholds: Min ",  String(0) + "," + String(Min[0]));
            mLogger.logMessage("maxThreshold_updateThresholds: Max ",  String(0) + "," + String(Max[0]));
          }
          if (compartmentState[2])
          {
            std::pair<float *, float *> collect = updateThresholds(2);
            Min[2] = float(collect.first[2]);
            Max[2] = float(collect.second[2]);
            // auto [minThreshold, maxThreshold] = updateThresholds(i);
            // Min = float(minThreshold[i]);
            // Max = float(maxThreshold[i]);
            mLogger.logMessage("minThreshold_updateThresholds: Min ", String(2) + "," + String(Min[2]));
            mLogger.logMessage("maxThreshold_updateThresholds: Max ", String(2) + "," + String(Max[2]));
          }
          
        
        while (count < TIMEOUT && !BarSanner.available())
        {
          collectionEffect();
          vTaskDelay(95 / portTICK_PERIOD_MS);
          yield();
          if (count > 600)
            turnLock(LOW);
          count++;
        }
        if (count >= TIMEOUT)
        {
          mLogger.logInfo("Timed out", "waiting for Second Scan");
          triggerCollection = !triggerCollection;
        }
        else
          mLogger.logInfo("collection", "Collection Card Rescanned");
        isWaitingForCollection = false;
        turnLock(LOW);
        mLogger.logMessage("scanManager", "------------1-------------");
        for (uint8_t i = 0; i < 4; i++)
        {
          if (compVaildation(i))
          {
            // mLogger.logMessage("minThreshold_2", String(Min));
            // mLogger.logMessage("maxThreshold_2", String(Max));
            // collectionDone[i] = checkExitCondition(i, Min[i], Max[i]);
          }
        }
        mLogger.logMessage("scanManager", "-------------2------------");
        for (uint8_t i = 0; i < 4; i++)
        {
          if (compVaildation(i))
          {
            // if (!collectionDone[i]) prefs.putBool("collection", true); // if collection was not fully done but bin was full, add a marker for this collection
            mLogger.logMessage("scanManager", "--------------- putBool ---------------");
            isWaitingForCollection = false;
            if (collectionDone[i])
            {
              mLogger.logMessage("scanManager", "--------------- Collection Done ---------------");
              // vTaskDelay(100 / portTICK_PERIOD_MS);
              turnLock(LOW);
              bottleCount = 0;
              ewasteCountBig=0;
              prefs.begin("configs", false);
              prefs.putBool("ewasteBig", ewasteCountBig);
              prefs.putBool("bot_count", bottleCount);
              prefs.end();
              prefs.putBool("collection", false);
              Material = *compName[i];
              if (collectionComplete(ean, timestamp))
                //   mLogger.logMessage("ScanEAN-collectionComplete", "Storing Collection Card Info");
                // xTaskCreate(task_uploadFromStorage, "StoredUpload", 10000, NULL, 2, &upload_task);
                CollectionCardFlag = false;
            }
          }
        }
        // for (uint8_t i = 0; i < 4; i++)
        // {

        // if (compVaildation(i))
        // {
        //   mLogger.logMessage("scanManager", "compVaildation");
        //   return;
        //   // }
        //   mLogger.logMessage("scanManager", "return");
        // collectionDone[i] = waitforcollection(i);
        //   mLogger.logMessage("scanManager", "--------------- waitforcollection ---------------");
        //   prefs.putString("lastEan", ean);

        //   // prefs.putString("lastEan", ean);
        //   // collectionDone = waitforcollection();
        //   if (!collectionDone[i])
        //     prefs.putBool("collection", true); // if collection was not fully done but bin was full, add a marker for this collection
        //   mLogger.logMessage("scanManager", "--------------- putBool ---------------");
        //   isWaitingForCollection = false;
        //   if (collectionDone[i])
        //   {
        //     mLogger.logMessage("scanManager", "--------------- Collection Done ---------------");
        //     // if(!i)
        //     for(uint8_t i=0;i<4; i++)
        //     turn_Lock(lockPins[i],LOW);
        //     vTaskDelay(100 / portTICK_PERIOD_MS);
        //     // turnLock(LOW);
        //     bottleCount = 0;
        //     prefs.putBool("collection", false);
        //     Material = *compName[i];
        //     if (collectionComplete(ean, timestamp))
        //       //   mLogger.logMessage("ScanEAN-collectionComplete", "Storing Collection Card Info");
        //       // xTaskCreate(task_uploadFromStorage, "StoredUpload", 10000, NULL, 2, &upload_task);
        //       CollectionCardFlag = false;
        //   }
        // }
      }
      if (comp1.validate(ean) && AWSCOnnectivity)
      { // if ean is valid bottle comp1=pet, comp2=can comp3
        if (isBinFull[0])
        {
          mLogger.logWarning("scanManager", " comp 1 Bin is full");
          vTaskDelay(100 / portTICK_PERIOD_MS); // Add delay here
          continue;
        }
        Material = "";
        Material = comp1Name;
        uploadBarcodeScanned(ean, timestamp);
        bool wasteDetected = wasteDetection(ean);
        if (!wasteDetected)
          continue;
      }
      else if (comp2.validate(ean) && AWSCOnnectivity)
      {
        if (isBinFull[1])
        {
          mLogger.logWarning("scanManager", "comp2 Bin is full");
          vTaskDelay(100 / portTICK_PERIOD_MS); // Add delay here
          continue;
        }
        Material = "";
        Material = comp2Name;
        uploadBarcodeScanned(ean, timestamp);
        bool wasteDetected = wasteDetection(ean);
        if (!wasteDetected)
          continue;
      }
      else if (comp3.validate(ean) && AWSCOnnectivity)
      {
        if (isBinFull[2])
        {
          mLogger.logWarning("scanManager", "comp3 Bin is full");
          vTaskDelay(100 / portTICK_PERIOD_MS); // Add delay here
          continue;
        }
        Material = "";
        Material = comp3Name;
        uploadBarcodeScanned(ean, timestamp);
        bool wasteDetected = wasteDetection(ean);
        if (!wasteDetected)
          continue;
      }
      else if (comp4.validate(ean) && AWSCOnnectivity)
      {
        if (isBinFull[3])
        {
          mLogger.logWarning("scanManager", "comp4 Bin is full");
          vTaskDelay(100 / portTICK_PERIOD_MS); // Add delay here
          continue;
        }
        Material = comp4Name;
        uploadBarcodeScanned(ean, timestamp);
        bool wasteDetected = wasteDetection(ean);
        if (!wasteDetected)
          continue;
      }
      else if (validateWhiteLabelEan(ean))
      {
        if (!AWSCOnnectivity)
          continue;
        // Loop condition for special White Label Ean
        // if (isBinFull)
        {
          mLogger.logWarning("scanManager", "whitelable, Bin is full");
          vTaskDelay(100 / portTICK_PERIOD_MS); // Add delay here
          continue;
        }
        // uploadBarcodeScanned(ean, getUnixTimestamp());
        for (uint8_t i = 0; i < 4; i++)
        {
          if (Material == *compName[i])
          {
            open_servo(servos[i], SOpenPos[i]);
          }
        }
        unsigned long waitTimeStamp = millis();
        while ((millis() - waitTimeStamp) < 10000)
        {
          bool bottleDetected;
          if (bottleDetected)
          {
            // bottleDropEffect();
            // turnOffRGB();
            // saveBottleInfo(ean, getUnixTimestamp()); // save bottle info
            waitTimeStamp = millis();
            bottleDetected = false;
          }
          else
          {
            Serial.println("Fake Drop --- 44");
            //  FakeDropEffect();
            //  turnOffRGB();
          }
        }
        for (uint8_t i = 0; i < 4; i++)
        {
          if (Material == *compName[i])
          {
            close_servo(servos[i], SClosePos[i]);
          }
        }
        // breathingBrandColors();
        whitelableF = false;
        // if (!upload_task_active)
        //   xTaskCreate(task_uploadFromStorage, "StoredUpload", 10000, NULL, 2, &upload_task); // start upload task
      }
      else
      {
        if (ean != "")
        {
        }
      }
    }
  }
}
void scanManagerIdle(void *pvParameters)
{
  while (true)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void downloadEANList()
{ //IoTclient.disconnect();
  // delay(100);
  if (comp1.eansLength() < 2 || comp2.eansLength() < 2 || comp3.eansLength() < 2 || comp4.eansLength() < 2)
  { // check the EANs every MQTT topic reset.
    Serial.println("Getting EANS:  " + String(comp1.eansLength()) + " ," + String(comp2.eansLength()) + ", " + String(comp3.eansLength() + ", " + String(comp4.eansLength())));
    Serial.println("checking flag:  " + URL);
    if (comp1.eansLength() < 2)
      comp1.getEans(client, URL, String(ca_cert));
    delay(10);
    if (comp2.eansLength() < 2)
      comp2.getEans(client, URL, String(ca_cert));
    delay(10);
    if (comp3.eansLength() < 2)
      comp3.getEans(client, URL, String(ca_cert));
    delay(10);
    if (comp4.eansLength() < 2)
      comp4.getEans(client, URL, String(ca_cert));
    delay(10);

    comp1.refreshEans();
    comp2.refreshEans();
    comp3.refreshEans();
    comp4.refreshEans();

    Serial.println("After Refresh Eans: " + String(comp1.eansLength()) + " " + String(comp2.eansLength()) + " " + String(comp3.eansLength() + " " + comp4.eansLength()));

    Serial.println("Restatrting ESP Now");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP.restart(); // it will restart again because if not it will not connect to aws iot core.
  }
  else
    Serial.println("Files exists");
}

/**
 * Initializes the compartments and sets up their serial communication.
 *
 * This function retrieves the compartment names and related data from persistent storage,
 * configures serial communication for each compartment, and enables them if valid data is found.
 * Each compartment has its own serial interface and settings.
 *
 * @function
 * @returns {void} This function does not return anything.
 */
void initCompartments()
{
  // Create vectors to store compartment pointers and their names
  // This allows us to handle multiple compartments dynamically
  std::vector<Compartment *> compartments = {&comp1, &comp2, &comp3, &comp4};
  std::vector<String> compartmentNames = {comp1Name, comp2Name, comp3Name, comp4Name};


  // comp1Name = prefs.getString("comp1", ""); // keep empty if not found
  // comp2Name = prefs.getString("comp2", "can");
  // comp3Name = prefs.getString("comp3", "pet");
  // comp4Name = prefs.getString("comp4", "");
  mLogger.logInfo("compartments names: ", *compName[0] + "," + *compName[1] + "," + *compName[2] + "," + *compName[3]);
  devCountry = prefs.getString("Country", "ae");
  // initialize compartments
  // Assuming you have a vector of compartments and compartment names
  // Iterate through all compartments
  for (size_t i = 0; i < compartments.size(); i++)
  {
    // Check if compartment name is valid (not empty, null, or true)
    if (!compartmentNames[i].isEmpty() && compartmentNames[i] != "null" && compartmentNames[i] != "false")
    {
      compartments[i]->setName(compartmentNames[i]);                                               // Set the name for the compartment
      compartments[i]->setFilePath("/" + compartmentNames[i] + ".txt");                            // Set file path in format "/name.txt" (e.g., "/pet.txt")
      compartments[i]->setEanLink("/eans?type=" + compartmentNames[i] + "&country=" + devCountry); // Set EAN link in format "/eans?type=name&country=countryCode" (e.g., "/eans?type=pet&country=AE")
      compartments[i]->enable();                                                                   // Enable the compartment
      vTaskDelay(100 / portTICK_PERIOD_MS);
      mLogger.logInfo("compartments enabled: ", compartmentNames[i]); // Log that the compartment has been enabled
    }
  }
  mLogger.logInfo("WifiConnecting", String(internetConnected));
  // if (internetConnected && RegisterDone)
  //   // downloadEANList();
  prefs.end();
  mLogger.logInfo("compartments names: ", String(internetConnected) + "," + String(RegisterDone));
}
bool needEANs(){
  return ((comp1.eansLength() < 2 || comp2.eansLength() < 2 || comp3.eansLength() < 2 || comp4.eansLength() < 2)&& RegisterDone);
}

void setup()
{
  // put your setup code here, to run once:
  WireDvicesInit(); // initlaze all the wire devices (BarSanner, I2CMux, I2CMux_1, ina3221, ina219)
  Get_ChipID();
  getPref();
  vTaskDelay(50 / portTICK_PERIOD_MS);
  actuatorsBegin();
  PrintPref();
  if (!SPIFFS.begin(true))
  { // format if mount failed
    mLogger.logWarning("filesBegin", "SPIFFS Mount Failed. Please wait....");
  }
// <<<<<<< dev
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  ConfigSystemParam();
  setTopicBasedOnCountry(devCountry);
//   DetectCompartment();
// =======
  // while (true)
  // {  
    // delay(300);
    DetectCompartment();
    EwasteDetectCompartment();
    // printVertiaclSensorReads();
    // handleDropDetection();
  // }
  printSensorReads();
  ina219Init();
  ina3221Init();


  if (!RegisterDone)
  {
    findingCurrrentThreshoulds();
    for (int i = 0; i < 4; i++)
    {
      Serial.println("comp state: " + String(compartmentState[i]) + " , " + String(compartIusse[i]));
      if (compartmentState[i] || compartIusse[i]) // && !RegisterDone)
        calibrateServo(i);
    }
    // servoTesting();
  }
  // initCompartments();
  // isEANdownloading=needEANs();
   mLogger.logMessage("assembly flag : ", String(AssemblConnect) +","+String(wifiCreds) +","+String(isEANdownloading));
  if (!SimcomModem && (wifiCreds || AssemblConnect))

  {
    WiFi.mode(WIFI_STA);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(
        connect_to_wifi,   // Task function
        "connect_to_wifi", // Name of task
        8192,              // Stack size (for network-related tasks, you may need a larger stack)
        NULL,              // Parameter
        2,                 // Priority
        &taskWifiConnect,  // Task handle
        0                  // Core 0
    );
  }
  // mLogger.logMessage("connect_to_wifi", "Test");
  if (Material != "vape" || Material != "battery" || Material.indexOf("paper") != -1)
  {
    xTaskCreatePinnedToCore(
        scanManager,        // Task function
        "taskScankManager", // Name of task
        6144,               // Stack size (adjust as needed)
        NULL,               // Parameter
        3,                  // Priority
        &taskScankManager,  // Task handle
        1                   // Core 1
    );
  }
  else
  {
    xTaskCreatePinnedToCore(
        scanManagerIdle,    // Task function
        "taskScankManager", // Name of task
        3072,               // Stack size (adjust as needed)
        NULL,               // Parameter
        1,                  // Priority
        &taskScankManager,  // Task handle
        1                   // Core 1
    );
  }
  mLogger.logMessage("taskScankManager", "Test");
  xTaskCreatePinnedToCore(
      networkManager,       // Task function
      "taskNetworkManager", // Name of task
      9216,                 // Stack size (for network-related tasks, you may need a larger stack)
      NULL,                 // Parameter
      1,                    // Priority
      &taskNetworkManager,  // Task handle
      0                     // Core 0
  );
  mLogger.logMessage("taskNetworkManager", "Test");
  mLogger.logMessage("Material", Material);

     for (uint8_t i = 0; i < 2; i++)
      {
        // if (compVaildation(i))
        if (compartmentState[0] || EwasteCompartmentState[1])
        { 
          // Material == *compName[i];
          auto &muxV = compSensors[i].vSensor.muxID == 0 ? I2CMux : I2CMux_1;
          if(!i){
            isBinFull[0] = checkBinFull(getUsage(compSensors[i].vSensor.sensorName, compSensors[i].vSensor.sensor, muxV, compSensors[i].vSensor.channel, i), i);
            mLogger.logInfo("Setup","Checking Bin Full for comp Small" + String(isBinFull[0]));
          }
            else if (i==1){
          float tempUsage=0.00;
          isBinFull[1] = checkBinFull(getUsage(tempUsage), i);
          mLogger.logInfo("Setup","Checking Bin Full for comp Large" + String(isBinFull[i]));
          }
          taskYIELD();
        }
      }
}

void networkManager(void *pvParameters)
{
  while (true)
  {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    if (AWSCOnnectivity) IoTclient.loop();
    // if (*compName[0] == "" && *compName[1] == "" && *compName[2] == "" && *compName[3] == "" && !WifiConnecting){
    //   CaptiveSsid += "_" + Bin_ID + "_" + "SetCompart";
    //   CompCaptive=true;
    //   WifiConnecting=true;
    //   startSoftAccessPoint(CaptiveSsid.c_str(), CaptivePassword, localIP, gatewayIP);
    //   setUpDNSServer(dnsServer, localIP);
    //   setUpWebserver(server, localIP); // this one runs wifi creds captive
    //   server.begin();
    //   mLogger.logInfo("Captive Wi-Fi for compartments", "Started");
    //   isCaptiveOn=true; 
    // }
    if (!SimcomModem && !internetConnected && !WifiConnecting && !wifiCreds && ProvisionDone && RegisterDone)
    { // wifi creds captive
      // captive Wifi
      CaptiveSsid += "_" + Bin_ID + "_" + "SetWifi";
      isCaptiveOn=true;
      CompCaptive=false;
      startSoftAccessPoint(CaptiveSsid.c_str(), CaptivePassword, localIP, gatewayIP);
      setUpDNSServer(dnsServer, localIP);
      setUpWebserver(server, localIP); // this one runs wifi creds captive
      server.begin();
      // scan networks for captive Portal
      numNetworks = WiFi.scanComplete();
      if (numNetworks > 0)
      {
        for (int i = 0; i < numNetworks; i++)
        {
          storedSSIDs[i] = WiFi.SSID(i);
        }
      }
      else if (numNetworks == -1)
        ;
      else if (numNetworks == -2)
        WiFi.scanNetworks(true);
      mLogger.logInfo("Captive Wi-Fi", "Started");
      WifiConnecting = true;
    }
    else if (!SimcomModem && WifiConnecting && !wifiCreds && !AssemblConnect){
      dnsServer.processNextRequest();
      vTaskDelay(30 / portTICK_PERIOD_MS);
    }

    else if (SimcomModem && !ModemConnecting && !internetConnected && RegisterDone)
    {
      xTaskCreatePinnedToCore(connect_to_modem, "connect_to_modem", 5000, NULL, 2, &taskModemConnect, 0);
    }
    if (!IoTclient.connected() && internetConnected && !AWSConnecting && !OTADownloading)
    {
      AWSCOnnectivity = false;
      mLogger.logMessage("networkManager", "Aws is Not connected, Connecting started...");
      vTaskDelay(500 / portTICK_PERIOD_MS);
      if (!ProvisionDone)
        createCertificate();
      else
      {
        if (awsiotstart())
          ; // if it connected to aws.
        else
        {
          if (SimcomModem)
            ReconnectModem();
          else
            return;
        }
      }
    }
    if (OTADownloading){
      mLogger.logMessage("OTADownloading","OTA Downloading");
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }
    if (printLocalTime())
      binlocaltimeflag = true;
    else
      binlocaltimeflag = false;
    if (HourNow == 0 && MinuteNow == 0 && binlocaltimeflag)
    {  
      diagnoseSensors();
      eWasteDiagnoseSensors();
      servoTesting();
      for (uint8_t i = 0; i < 4; i++)
      {
        if (compartIusse[i])
        {
          uploadCreateIssue("ToF", CompartmentIssues[i], timestamp);
          compartIusse[i] = false;
        }
        if (servoLimit[i])
        {
          mLogger.logMessage("servoLimit", "servoLimit[i] is false");
          uploadCreateIssue("Servo", ServoIssues[i], timestamp);
          servoLimit[i] = false;
        }
      }
    }
    // if (HourNow == 15 && MinuteNow == 0 && binlocaltimeflag)
    if (HourNow == 15 && MinuteNow == 0 && binlocaltimeflag)
    {
      // mLogger.logMessage("SendSensorFB ", "Trigged");
      for (uint8_t i = 0; i < 4; i++)
      {
        if (compartmentState[i])
        {
          Material == *compName[i];
          auto &muxH = ((compSensors[i].hSensor1.muxID == 0) || compSensors[i].hSensor2.muxID == 0) ? I2CMux : I2CMux_1;
          std::pair<int, int> ranges = getLoxRanges(compSensors[i].hSensor1.sensor, compSensors[i].hSensor2.sensor, muxH, compSensors[i].hSensor1.channel, compSensors[i].hSensor2.channel);
          mLogger.logMessage("SendSensorFB ", "Ranges: " + String(ranges.first) + " " + String(ranges.second));
          SendSensorFB(ranges.first, ranges.second, getUnixTimestamp());
        }
      }
    }
    if (AWSCOnnectivity && ProvisionDone &&!RegisterDone)
    {
      if (RegisterBin()){
        mLogger.logInfo("RegisterBin", "Success !!");
        AssemblConnect=false;
        prefs.begin("configs", false); // begin preferences
        prefs.putBool("Assemble", AssemblConnect);
        prefs.end();
        // if(comp1.eansLength()<2 || comp2.eansLength()<2 || comp3.eansLength()<2 ||comp3.eansLength()<2) ESP.restart();
      }
      else
        mLogger.logInfo("RegisterBin", "failed !!");
    }
    // for (uint8_t i = 0; i < 2; i++)
    // {
    //   // if (compVaildation(i))
    //   if (compartmentState[0] || EwasteCompartmentState[1])
    //   { 
    //     // Material == *compName[i];
    //     mLogger.logInfo("ScannedTopic Material", *compName[i]);
    //     auto &muxV = compSensors[i].vSensor.muxID == 0 ? I2CMux : I2CMux_1;
    //     if(!i)isBinFull[0] = checkBinFull(getUsage(compSensors[i].vSensor.sensorName, compSensors[i].vSensor.sensor, muxV, compSensors[i].vSensor.channel, i), i);
    //     else if (i==1){
    //     float tempUsage=0.00;
    //     isBinFull[1] = checkBinFull(getUsage(tempUsage), i);
    //     mLogger.logInfo("ScannedTopic",String(isBinFull[i]));
    //     }
    //     taskYIELD();
    //   }
    // }
  }
}
void loop()
{
  // vTask Delay to keep running FreeRTOS tasks:
  //Serial.println ("Core ID: " + String(xPortGetCoreID())); //xPortGetCoreID();
  vTaskDelay(500);
}
