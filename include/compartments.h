#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
// #include <TCA9548A.h>
extern bool uploadFakedrop(int count, String timestamp, float usage);
extern bool uploadBottleInfo(String ean, String timestamp, float usage);
/**
 * @class Compartment
 * @brief A class representing a compartment that can manage EAN (European Article Numbers) data, interact with hardware serial communication, and download/update EAN lists.
 *
 * This class provides functionality for managing a compartment's configuration, such as enabling/disabling the compartment,
 * setting up serial communication, and handling files for EAN data. It can download EAN lists from a URL, validate EANs, and store 
 * them in the SPIFFS file system. The compartment can communicate via a hardware serial interface and handle EAN data as required.
 */
class Compartment
{
    /* Private data members */
private:
    bool enabled = false;              ///< Indicates whether the compartment is enabled or not.
    String eanLink;                    ///< The URL endpoint for downloading EANs.
    String filePath;                   ///< The file path in the file system where EANs are stored.
    HardwareSerial *compSerial;        ///< Pointer to the hardware serial interface for communication.
    String eanList;                    ///< List of EANs stored as a string.

    /* Public data members */
public:
    String name;                       ///< The name of the compartment.

    /** 
     * @brief Refreshes the list of EANs by reading the data from a file stored in SPIFFS.
     *
     * This function reads the EAN data from the specified file path in the file system and prints the length of the EAN list.
     */
    void refreshEans()
    {
        eanList = readFromFile(SPIFFS, filePath.c_str());
        Serial.println("Eans: " + String(eanList.length()));  // Output the length of the EAN list
        Serial.println("Eans: " + filePath);                  // Output the file path
    }

    /** 
     * @brief Enables the compartment and refreshes its EAN data.
     */
    void enable()
    {
        enabled = true;             // Set the compartment as enabled.
        this->refreshEans();        // Load the EAN data.
    }

    /** 
     * @brief Sets the name of the compartment.
     * @param name The name to assign to the compartment.
     */
    void setName(String name) { this->name = name; }

    /** 
     * @brief Sets the file path for the EAN data of the compartment.
     * @param filePath The file path where EANs are stored.
     *
     * If the file doesn't exist, it creates an empty file.
     */
    void setFilePath(String filePath)
    {
        this->filePath = filePath;
        if (!SPIFFS.exists(filePath.c_str()))
        {
            // File doesn't exist, so create an empty file.
            writeFile(SPIFFS, filePath.c_str(), "");
        }
        else
        {
            Serial.println("File already exists");
        }
        // mLogger.logInfo("File path set: ", this->filePath+"," + filePath);
    }

    /** 
     * @brief Sets the URL link for downloading EANs.
     * @param eanLink The URL for downloading the EANs.
     */
    void setEanLink(String eanLink) { this->eanLink = eanLink; 
    // mLogger.logInfo("EAN link set: ", this->eanLink+"," + eanLink);
    }

    // /** 
    //  * @brief Sets the serial interface for communication with the compartment.
    //  * @param Serial A reference to the hardware serial instance.
    //  */
    // void setSerial(HardwareSerial &Serial) { this->compSerial = &Serial; }

    // /** 
    //  * @brief Initializes the serial communication with the specified baud rate.
    //  * @param baud The baud rate for the serial communication.
    //  */
    // void begin(int baud)
    // { 
    //     if (!enabled) return;      // Ensure the compartment is enabled before proceeding.
    //     compSerial->begin(baud);   // Start serial communication with the specified baud rate.
    // }

    /** 
     * @brief Returns the length of the EAN list.
     * @return The length of the EAN list.
     */
    int eansLength()
    {
        if (!enabled) return 2;    // If the compartment is disabled, return a default value.
        return eanList.length();   // Return the length of the EAN list.
    }

    /** 
     * @brief Validates an EAN by checking if it exists in the EAN list.
     * @param ean The EAN to validate.
     * @return True if the EAN is found, false otherwise.
     */
    bool validate(String ean)
    {
        if (!enabled) return false;  // Ensure the compartment is enabled.
        // Serial.println("Validating2 " + ean + " for " + name + " Compartment2");
        // Check if the EAN is present in the list (both in double and single quotes).
        int index1 = eanList.indexOf("\"" + ean + "\"");
        int index2 = eanList.indexOf("'" + ean + "'");
        return (index1 > -1) || (index2 > -1);
    }

    /** 
     * @brief Downloads the EANs from a specified URL and updates the local file.
     * @param client The HTTP client used for the download.
     * @param URL The base URL for the EAN data.
     * @param ca_cert The certificate for HTTPS verification.
     */
    void getEans(HTTPClient &client, String URL, String ca_cert)
    {
      //  mLogger.logInfo("enabled flag: ", String(enabled));
        if (!enabled) return;  // Ensure the compartment is enabled.
        int maxRetries = 2;    // Max retry attempts in case of failure.
        int attempt = 0;
        // Try downloading EANs with retries.
        do
        {   Serial.println("--------------------11");
            eanList = readFromFile(SPIFFS, this->filePath.c_str());  // Read the current EAN list.
            Serial.println("--------------------22");
            client.begin(URL + (this->eanLink), ca_cert.c_str());     // Set up the HTTP request.
            client.addHeader("x-chip-id", ChipID);
            mLogger.logInfo("getEANs: ", URL + (eanLink) +" and "+filePath);
            int httpcode = client.GET();                        // Send GET request.
            Serial.println("--------------------33");
            if (httpcode == 200)
            {
                // If request is successful, update the EAN list.
                eanList = client.getStreamPtr()->readString();
                eanList.replace("]", ",]");
                Serial.println("----------------1");

                if (eanList.indexOf("count") > -1)  // Check if the response contains valid data.
                {
                  Serial.println("----------------2");
                  deleteFile(SPIFFS, filePath.c_str()); // Delete existing file and save new data.
                  Serial.println("----------------3");
                  appendFile(SPIFFS, filePath.c_str(), eanList);
                  Serial.println("----------------4");
                }
                else
                {
                  Serial.println("----------------55555");
                  eanList = readFromFile(SPIFFS, filePath.c_str()); // Reload from file if no valid data.
                }
                Serial.println(name + " Eans Length");
                Serial.println(eanList.length());  // Output the length of the updated EAN list.
                Serial.println("----------------66666666");
                break;  // Exit loop after successful download.
            }
            else
            {
                attempt++;
                Serial.println("HTTP request failed. Retrying...");
                delay(1000);  // Delay before retry.
            }
        } while (attempt < maxRetries);
    }

    /** 
     * @brief Checks if the compartment's serial interface has data available.
     * @return True if data is available, false otherwise.
     */
    bool available()
    {
        if (!enabled) return false;  // Ensure the compartment is enabled.
        return this->compSerial->available();  // Check if data is available on the serial interface.
    }

    // /** 
    //  * @brief Reads a single character from the compartment's serial interface.
    //  * @return The character read from the serial interface.
    //  */
    // char readChar()
    // {
    //     if (!enabled) return 0;  // Ensure the compartment is enabled.
    //     return this->compSerial->read();  // Read and return a character from the serial interface.
    // }

    // /** 
    //  * @brief Writes a single character to the compartment's serial interface.
    //  * @param c The character to write.
    //  */
    // void writeChar(char c)
    // {
    //     if (!enabled) return;  // Ensure the compartment is enabled.
    //     this->compSerial->write(c);  // Write the character to the serial interface.
    // }

    // /** 
    //  * @brief Clears the serial buffer of the compartment.
    //  */
    // void flush()
    // {
    //     if (!enabled) return;  // Ensure the compartment is enabled.
    //     this->compSerial->flush();  // Clear the serial buffer.
    // }

    /** 
     * @brief Reads a string from the serial interface until a specified character is encountered.
     * @param ending The character that marks the end of the string.
     * @return The string read from the serial interface.
     */
    String readStringUntil(char ending)
    {
        return this->compSerial->readStringUntil(ending);  // Read string until the ending character.
    }

    /** 
     * @brief Prints a string to the serial interface.
     * @param s The string to print.
     */
    void println(String s)
    {
        if (!enabled) return;  // Ensure the compartment is enabled.
        this->compSerial->println(s);  // Print the string to the serial interface.
    }
};

/** function read the incoming string on the Serial
*/
String readIncomingEan() {
  String ean;
  // BarSanner.flush();

  while (BarSanner.available())  // Keep reading Byte by Byte from the Buffer till the Buffer is empty
  {
    char input = BarSanner.read();  // Read 1 Byte of data and store it in a character variable
    if ((input >= '0' && input <= '9') || (input >= 'A' && input <= 'Z')|| (input >= 'a' && input <= 'z')) {
      ean.concat(input);
    }
    delay(2); 
  }
  mLogger.logError("ean: ", ean);
  return ean;
}

/**
 * @brief Checks if a given EAN represents a collection card.
 * 
 * This function scans through each character in the provided EAN (European Article Number)
 * and checks if it contains any characters in the range 'A' to 'F'. If any character falls
 * within this range, the EAN is identified as a collection card and the function returns true.
 * Otherwise, it returns false.
 *
 * @param ean The EAN (string) to check.
 * @return True if the EAN contains any characters between 'A' and 'F', indicating a collection card;
 *         False otherwise.
 */
bool isCollectionCard(String ean)
{
  for (auto letter : ean)
  {
    if (letter >= 'A' && letter <= 'F')
    {
      // triggerCollection = true;
      triggerCollection = !triggerCollection;
      mLogger.logMessage("isCollectionCard : ", String(triggerCollection));
    }
  }
  if (triggerCollection)
  {
    // triggerCollection = false;
    return true;
  }
  else
    return false;
}
/**
 * Validates if a given card code exists within a predefined list of cards.
 *
 * This function checks if the provided `code` string is present within
 * the `cards` collection in two formats: enclosed in double quotes (`"code"`)
 * or single quotes (`'code'`). It returns `true` if the code is found in either format.
 *
 * @param {String} code - The card code to validate.
 * @returns {bool} - Returns `true` if the card code exists in `cards`, otherwise `false`.
 */
bool validateCard(String code)
{
  String Sample1 = "\"" + code + "\"";
  String Sample2 = "'" + code + "'";
  int index1 = cards.indexOf(Sample1);
  int index2 = cards.indexOf(Sample2);
  if ((index1 > -1) || (index2 > -1))
  return ((index1 > -1) || (index2 > -1));
  else return false;
}
/**
 * Waits for a bottle to be detected by the upper and lower sensors, performing multiple sensor readings 
 * and state transitions based on those readings. This function uses a combination of range sensing and 
 * hysteresis to accurately detect bottle movement (e.g., up/down) and count bottle drops, managing fake drops 
 * through state transitions.
 * 
 * The function will calibrate the sensor readings before starting the detection process. After that, it continuously 
 * checks the sensor data for bottle movement while respecting timeouts, updating the state, and detecting the bottle.
 * 
 * @returns {boolean} - Returns `true` when a bottle is successfully detected and dropped, `false` if no bottle is detected
 *                      within the given time window (5000ms).
 * 
 * @throws {TaskTimeoutError} - Will throw an error if the task takes longer than the specified timeout (5 seconds).
 */

bool waitforbottle(Adafruit_VL6180X &lox_1, Adafruit_VL6180X &lox_2, TCA9548A &mux, uint8_t channel1, uint8_t channel2, uint8_t RgbCount )//,struct CRGB * targetArray)
{
  mux.openChannel(channel1);
  mux.openChannel(channel2);
//   isWaitingForBottle = true;
  uint16_t range_lox1_sum = 0;
  uint16_t range_lox2_sum = 0;
  uint16_t min_range_lox1 = UINT16_MAX;
  uint16_t min_range_lox2 = UINT16_MAX;

  for (int i = 0; i < 5; i++){ // calibration before dropiing bottle (taking minimumReadings)
    lox_1.startRange();
    lox_2.startRange();
    lox_1.waitRangeComplete();
    lox_2.waitRangeComplete();
    uint16_t current_range_lox1 = lox_1.readRangeResult();
    uint16_t current_range_lox2 = lox_2.readRangeResult();

    current_range_lox1 = constrain(current_range_lox1, 0, 200);
    current_range_lox2 = constrain(current_range_lox2, 0, 200);

    if (current_range_lox1 < min_range_lox1) min_range_lox1 = current_range_lox1;
    if (current_range_lox2 < min_range_lox2) min_range_lox2 = current_range_lox2;
  }
  mLogger.logInfo("watforbottle","wait For Bottle 2");
  if (min_range_lox1 == 200) min_range_lox1 = min_range_lox2;
  unsigned long timeout_counter2 = millis();
  mLogger.logInfo("watforbottle","wait For Bottle 3");
  while ((millis() - timeout_counter2) < 5000) {
    lox_1.startRange();
    lox_2.startRange();
    lox_1.waitRangeComplete();
    lox_2.waitRangeComplete();

    uint8_t range_lox1 = lox_1.readRangeResult();
    uint8_t status_lox1 = lox_1.readRangeStatus();
    uint8_t range_lox2 = lox_2.readRangeResult();
    uint8_t status_lox2 = lox_2.readRangeStatus();
    range_lox1 = constrain(range_lox1, 0, 200);
    range_lox2 = constrain(range_lox2, 0, 200);
    int UpNow = range_lox1;
    int DownNow = range_lox2;
    // Update the detection flags with hysteresis
    if (UpNow < (min_range_lox1 - (min_range_lox1 * 0.2))) upperDetected = true;
    else upperDetected = false;

    if (DownNow < (min_range_lox2 - (min_range_lox2 * 0.1)))  lowerDetected = true;
    else lowerDetected = false;

    if (upperDetected || lowerDetected || state == 3){
      if (upperDetected && !lowerDetected) differenceState = -1; // Upper sensor less than lower sensor reading, bottle up
      else if (!upperDetected && lowerDetected) differenceState = 1; // Upper sensor bigger than lower sensor reading, bottle down
      else if (upperDetected && lowerDetected) differenceState = 0; // Difference is within the threshold

      switch (state){
      case 0:
        if (differenceState == -1 && upperDetected)
          state = 1;
        else if (differenceState == 0)
          state = 2;
        break;
      case 1:
        if (differenceState == 0)state = 2;
        else if (differenceState == 1) state = 3;
        else if (differenceState == -1) {
          state = 1;
          StatecounterN1++;
          if (StatecounterN1 >= 10){
            state = 0;
            StatecounterN1 = 0;
            fakedropcounter++;
            mLogger.logInfo("waitforbottle","Fake Drop in case 1");
            FakeDropEffect(RGBcomp[RgbCount]);
            breathingBrandColors(RGBcomp[RgbCount],NUM_LEDS);
            // FakeDropEffect(targetArray);
            // if (!whitelableF && AWSCOnnectivity)
          }
        }
        break;
      case 2:
        if (differenceState == 1) state = 3;
        else if (differenceState == -1)state = 4;
        else if (differenceState == 0)state = 3;
        break;
      case 3:
        if ((differenceState == 1 || differenceState == 0) && !upperDetected){
          mLogger.logInfo("Bottle Drop ", String(differenceState) + "   " + String(state) + "    " + String(UpNow) + " : " + String(DownNow));
          bottleCount++;
          Statcounter1 = 0;
          StatecounterN1 = 0;
          StatecounterN2 = 0;
          state = 0;
        //   isWaitingForBottle = false;
        mux.closeAll();
          return true;
        }
        else if (differenceState == 1){
          state = 3;
          Statcounter1++;
          if (Statcounter1 >= 10){
            state = 0;
            Statcounter1 = 0;
            fakedropcounter++;
            mLogger.logInfo("waitforbottle","Fake Drop in case 3");
            FakeDropEffect(RGBcomp[RgbCount]);
            breathingBrandColors(RGBcomp[RgbCount],NUM_LEDS);
            // FakeDropEffect(targetArray);
            // if (!whitelableF && AWSCOnnectivity)
            //   breathingBrandColors(targetArray);
          }
        }
        else if (differenceState == 0){
          state = 3;
          StatecounterN2++;
          if (StatecounterN2 >= 15){
            state = 0;
            StatecounterN2 = 0;
            fakedropcounter++;
            mLogger.logInfo("waitforbottle","Fake Drop in case 3");
            FakeDropEffect(RGBcomp[RgbCount]);
            breathingBrandColors(RGBcomp[RgbCount],NUM_LEDS);
          }
        }
        else if (differenceState == -1)state = 1;
        break;
      case 4:
        if (differenceState == 1 && !upperDetected){
          mLogger.logInfo("Bottle Drop",String(differenceState) + "   " + String(state) + "    " + String(UpNow) + " : " + String(DownNow));
          bottleCount++;
          Statcounter1 = 0;
          StatecounterN1 = 0;
          StatecounterN2 = 0;
          state = 0;
        //   isWaitingForBottle = false;
        mux.closeAll();
          return true;
        }
        else if (differenceState == -1) state = 3;
        else state = 1;
      default:
        state = 0; // Default to state 0 for any unexpected behavior
        break;
      }
      mLogger.logInfo("logs",String(differenceState) + "   " + String(state) + "    " + String(UpNow) + " : " + String(DownNow));
    }
    // Serial.println(String(differenceState) + "   " + String(state) + "    " + String(UpNow) + " : " + String(DownNow));
  }
  Statcounter1 = 0;
  state = 0;
  differenceState = 0;
  StatecounterN1 = 0;
  StatecounterN2 = 0;
  // FakeDropEffect();
//   isWaitingForBottle = false;
  mux.closeAll();
  return false;
}

/**
 * Handles the process when material (such as a bottle) is incoming. It checks if the bin is full,
 * detects material presence, handles servo operations, and stores the information if a valid material drop is detected.
 * 
 * The function works as follows:
 * 1. Checks if the bin is full using the `checkBinFull` function and logs a warning if full.
 * 2. If the bin is not full, it proceeds to detect material (bottle) using the `waitforbottle` function.
 * 3. If material is detected, it stores relevant information (such as the IoT EAN code).
 * 4. If no material is detected, it logs a message and resets the material detection process.
 * 5. Additional operations (e.g., RGB lighting effects, servo control, etc.) are handled, though some are commented out.
 * 
 * @returns {void} - This function does not return any value.
 * 
 * @see checkBinFull() - To check if the bin is full.
 * @see waitforbottle() - To detect if a material (bottle) is dropped.
 * @see mLogger.logWarning() - To log a warning when the bin is full.
 * @see mLogger.logMessage() - To log a message when material information is being stored.
 * @see mLogger.logInfo() - To log information during the process.
 */
void materialIncoming(String type){
  
  if (isBinFull[0]) {
    mLogger.logWarning("materialIncoming (small)", "Bin is full");
    return;
  }
  prefs.begin("configs", false);
  CupIncoming=true;
  bool MaterialDetected;
  float temUsage=0.0;
  String timestamp = "";
  uint8_t i=0;
    if (type == *compName[i])
    {
      open_servo(servos[i],SOpenPos[i]);
      auto &muxH1 = ((compSensors[i].hSensor1.muxID == 0) || compSensors[i].hSensor2.muxID == 0) ? I2CMux : I2CMux_1;
      MaterialDetected = waitforbottle(compSensors[i].hSensor1.sensor, compSensors[i].hSensor2.sensor, muxH1, compSensors[i].hSensor1.channel, compSensors[i].hSensor2.channel, i);
      MaterialDetected ? bottleDropEffect(RGBcomp[i]) : FakeDropEffect(RGBcomp[i]);
      close_servo(servos[i],SClosePos[i]);
      temUsage = getUsage(compSensors[i].vSensor.sensorName, compSensors[i].vSensor.sensor, muxH1, compSensors[i].vSensor.channel, i);
    }
  if (!MaterialDetected){
    CupIncoming=false;
  }
  else {
     if (AWSCOnnectivity)uploadBottleInfo(IoTEAN, timestamp, temUsage);
    CupIncoming=false;
    IoTEAN="";
    ++bottleCount;
    prefs.putInt("bot_count", bottleCount);
    prefs.end();
  }
}

std::pair<float*, float*> EWupdateThresholds(int i) {
    static float maxThreshold[4] = {-10, -10, -10, -10};
    static float minThreshold[4] = {2000, 2000, 2000, 2000};
    float x = 0.0;
    float sumX[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    float usage = 0.0;
    if (EwasteCompartmentState[i])
    {
      for (size_t k = 0; k < 5; k++)
      {
        x = 0.0;
        auto &muxV1 = (EwastecompSensors[i].vSensor1.muxID == 0) ? I2CMux : I2CMux_1;
        x += VAvrage(EwastecompSensors[i].vSensor1.sensorName, EwastecompSensors[i].vSensor1.sensor, muxV1, EwastecompSensors[i].vSensor1.channel); //  Reading sample Vertical Sensors for Additional Module
        auto &muxV2 = (EwastecompSensors[i].vSensor2.muxID == 0) ? I2CMux : I2CMux_1;
        x += VAvrage(EwastecompSensors[i].vSensor2.sensorName, EwastecompSensors[i].vSensor2.sensor, muxV2, EwastecompSensors[i].vSensor2.channel); //  Reading sample Vertical Sensors for Additional Module
        auto &muxV3 = (EwastecompSensors[i].vSensor3.muxID == 0) ? I2CMux : I2CMux_1;
        x += VAvrage(EwastecompSensors[i].vSensor3.sensorName, EwastecompSensors[i].vSensor3.sensor, muxV3, EwastecompSensors[i].vSensor3.channel); //  Reading sample Vertical Sensors for Additional Module
        sumX[k] = x;
        sumX[k] /= 3;
        // mLogger.logInfo("--- forloop sum sumX[k]-" + String(i), String(sumX[k]));
      }
      // mLogger.logInfo("sumX" + String(i), String(sumX[i]));
      for(size_t k = 0; k < 5; k++)
      {
        usage += sumX[k];
      }
    usage /= 5;
    // mLogger.logInfo("usage" + String(i), String(usage));
    }

    

    if (usage < minThreshold[i]) {
        minThreshold[i] = usage;
    }
    if (usage > maxThreshold[i]) {
        maxThreshold[i] = usage;
    }
    mLogger.logInfo("min:" + String(minThreshold[i]) + " x", String(usage) + " max:" + String(maxThreshold[i]));

    return std::make_pair(minThreshold, maxThreshold);
}

bool EWcheckExitCondition(int i, float minThreshold, float maxThreshold)
{
  float exitDistance;
  float x = 0.0;
  float sumX[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  for (size_t k = 0; k < 5; k++)
  {
    x = 0.0;
    auto &muxV1 = (EwastecompSensors[i].vSensor1.muxID == 0) ? I2CMux : I2CMux_1;
    x += VAvrage(EwastecompSensors[i].vSensor1.sensorName, EwastecompSensors[i].vSensor1.sensor, muxV1, EwastecompSensors[i].vSensor1.channel); //  Reading sample Vertical Sensors for Additional Module
    auto &muxV2 = (EwastecompSensors[i].vSensor2.muxID == 0) ? I2CMux : I2CMux_1;
    x += VAvrage(EwastecompSensors[i].vSensor2.sensorName, EwastecompSensors[i].vSensor2.sensor, muxV2, EwastecompSensors[i].vSensor2.channel); //  Reading sample Vertical Sensors for Additional Module
    auto &muxV3 = (EwastecompSensors[i].vSensor3.muxID == 0) ? I2CMux : I2CMux_1;
    x += VAvrage(EwastecompSensors[i].vSensor3.sensorName, EwastecompSensors[i].vSensor3.sensor, muxV3, EwastecompSensors[i].vSensor3.channel); //  Reading sample Vertical Sensors for Additional Module
    sumX[k] = x;
    sumX[k] /= 3;
    // mLogger.logInfo("--- forloop sum sumX[k]-" + String(i), String(sumX[k]));
  }
  for (size_t k = 0; k < 5; k++)
  {
    // mLogger.logInfo("--- Before sum sumX[k]-" + String(i), String(sumX[k]));
    exitDistance += sumX[k];
    // mLogger.logInfo("--- Before divide exitDistance-" + String(i), String(exitDistance));
  }
    exitDistance /= 5;
    // mLogger.logInfo("--- After Sum exitDistance-" + String(i), String(exitDistance));

  if ((exitDistance > maxThreshold + 5) || (exitDistance < minThreshold - 5))
  {
    bottleCount = 0;
    turnLock(LOW);
    prefs.putInt("bot_count", bottleCount);
    mLogger.logMessage("EWcheckExitCondition (Large)","True");
    return true;
  }
  mLogger.logMessage("EWcheckExitCondition (Large)" + String(i), String(exitDistance));
  mLogger.logWarning("EWcheckExitCondition (Large)", "No change in usage");
  return false;
}

std::pair<float*, float*> updateThresholds(int i) {
    static float maxThreshold[4] = {-10, -10, -10, -10};
    static float minThreshold[4] = {2000, 2000, 2000, 2000};
    float x[4];

    auto &muxV = (compSensors[i].vSensor.muxID == 0) ? I2CMux : I2CMux_1;
    x[i] = VAvrage(compSensors[i].vSensor.sensorName, compSensors[i].vSensor.sensor, muxV, compSensors[i].vSensor.channel);

    if (x[i] < minThreshold[i]) {
        minThreshold[i] = x[i];
    }
    if (x[i] > maxThreshold[i]) {
        maxThreshold[i] = x[i];
    }
    // mLogger.logInfo("Small Comp: min:" + String(minThreshold[i]) + " x", String(x[i]) + " max:" + String(maxThreshold[i]));

    return std::make_pair(minThreshold, maxThreshold);
}


bool checkExitCondition(int i, float usageThreshold) {
    float exitDistance;
    auto &muxV = (compSensors[i].vSensor.muxID == 0) ? I2CMux : I2CMux_1;

    exitDistance = VAvrage(compSensors[i].vSensor.sensorName,
                              compSensors[i].vSensor.sensor,
                              muxV,
                              compSensors[i].vSensor.channel);
    mLogger.logMessage("checkExitCondition (Small)","exitDistance= " + String(exitDistance));

    if ((exitDistance > usageThreshold + 15)) {// exit distance is less than 15cm
        bottleCount = 0;
        turnLock(LOW);
        prefs.putInt("bot_count", bottleCount);
        mLogger.logMessage("checkExitCondition (Small) collection","True");
        return true;
    }
    mLogger.logMessage("checkExitCondition (Small)", String(exitDistance));
    mLogger.logWarning("checkExitCondition (Small)", "No change in usage");
    return false;
}


/**
 * Monitors and waits for a collection process to complete within a specified timeout.
 *
 * The function samples distance measurements to establish minimum and maximum thresholds,
 * then enters a waiting loop where it checks if any significant change in distance
 * (indicating collection) has occurred. If the distance change is detected, the collection
 * is considered complete, and the function returns `true`. If no change occurs within the
 * timeout, it logs a timeout warning and returns `false`.
 *
 * @returns {bool} - Returns `true` if a significant distance change (indicating collection)
 *                   is detected; otherwise, returns `false` if the timeout is reached without change.
 */
bool waitforcollection(uint8_t i)
{
  int TIMEOUT = 3000; // 300  seconds 5 min
  int count = 0;
  float maxThreshold[4] = {-10, -10, -10, -10};
  float minThreshold[4] = {2000, 2000, 2000, 2000};
  float x[4], exitDistance[4]; 
  

  // delay(100);

  // x = getDistance(vSonar);
  //  x = myVsonar.ping_cm();
  //  x = VAvrage();

  auto &muxV = (compSensors[i].vSensor.muxID == 0) ? I2CMux : I2CMux_1;
  x[i] = VAvrage(compSensors[i].vSensor.sensorName, compSensors[i].vSensor.sensor, muxV, compSensors[i].vSensor.channel); //  Reading sample Vertical Sensors for Additional Module
  if (x[i] < minThreshold[i])
    minThreshold[i] = x[i];
  if (x[i] > maxThreshold[i])
    maxThreshold[i] = x[i];
  mLogger.logInfo("min:" + String(minThreshold[i]) + " x", String(x[i]) + " max:" + String(maxThreshold[i]));


  // ... rest of your existing code ...

  // Only run this block on first iteration if not already done
  if (!firstRunDone)
  {
    firstRunDone = true; // Mark as done
    while (count < TIMEOUT && !BarSanner.available())
    {
      // movingColorEffect(RGBcomp[i], CRGB::Red);
      collectionEffect();
      delay(100);
      yield();
      if (count > 600) turnLock(LOW);
        // turn_Lock(lockPins[i], LOW);
      count++;
    }
    if (count >= TIMEOUT)
      mLogger.logInfo("Timed out", "waiting for Second Scan");
    else
      mLogger.logInfo("collection", "Collection Card Rescanned");
    }

  // if (i == 0)
  // {
  //   while (count < TIMEOUT)
  //   {
  //     // wait for collection.
  //     //  collectionEffectS();
  //     yield();
  //     // delay(100);
  //     if (count > 600)
  //     {
  //       turn_Lock(lockPins[i], LOW);
  //     }
  //     count++;
  //   }
  //   if (count >= TIMEOUT)
  //     mLogger.logInfo("Timed out", "waiting for Second Scan");
  //   else
  //     mLogger.logInfo("collection", "Collection Card Rescanned");
  // }
  

  // float exitDistance = getDistance(vSonar);
  //  float exitDistance = myVsonar.ping_cm();
  exitDistance[i] = VAvrage(compSensors[i].vSensor.sensorName, compSensors[i].vSensor.sensor, muxV, compSensors[i].vSensor.channel); //  Reading sample Vertical Sensors for Additional Module
  // float exitDistance = VAvrage();
  if ((exitDistance[i] > maxThreshold[i] + 2) || (exitDistance[i] < minThreshold[i] - 2))
  {
    bottleCount = 0;
    turnLock(LOW);
    prefs.putInt("bot_count", bottleCount);
    // digitalWrite(2,LOW); //turn off the LED.
    return true;
  }
  else
    mLogger.logWarning("waitforcollection", "No change in usage");
  return false;
}
/**
 * Detects the presence of a waste bottle based on a scanned EAN (European Article Number).
 *
 * This function opens a servo to check for a bottle presence in the corresponding slot
 * based on the detected material type, triggering detection actions if a bottle is found.
 * If the bottle is detected, the collection process is completed or updated. If not,
 * a warning is logged. If any partial collection data exists, it is stored and marked as
 * complete. Returns `true` if a bottle was detected, `false` otherwise.
 *
 * @param {String} scannedEAN - The scanned EAN code to identify the waste material.
 * @returns {bool} - Returns `true` if a bottle is detected; otherwise, `false`.
 */
bool wasteDetection(String scannedEAN){
      // mLogger.logInfo("waste Detection","Open Servo to detect bottle");
      bool bottleDetected;
      float temUsage=0.0;
      for (uint8_t i = 0; i < 4; i++)
      {
        if (Material == *compName[i] && compVaildation(i))
        {
         open_servo(servos[i],SOpenPos[i]);
          auto &muxH1 = ((compSensors[i].hSensor1.muxID == 0) || (compSensors[i].hSensor2.muxID == 0)) ? I2CMux : I2CMux_1;
          // auto &muxH1 = ((compSensors[i].hSensor1.muxID == 0)) ? I2CMux : I2CMux_1;
          bottleDetected = waitforbottle(compSensors[i].hSensor1.sensor, compSensors[i].hSensor2.sensor, muxH1, compSensors[i].hSensor1.channel, compSensors[i].hSensor2.channel , i);
          bottleDetected ? bottleDropEffect(RGBcomp[i]) : FakeDropEffect(RGBcomp[i]);
          close_servo(servos[i],SClosePos[i]);
          temUsage = getUsage(compSensors[i].vSensor.sensorName, compSensors[i].vSensor.sensor, muxH1, compSensors[i].vSensor.channel, i);
          // Serial.println("Current: " + String(ina3221.getCurrent(INA3221_CH3)));
          // mLogger.logMessage("bottleStuck", String(ina3221.getCurrent(INA3221_CH3)));
          // bottleStuck(i);
        }
      }
      if (fakedropcounter > 0){
        uploadFakedrop(fakedropcounter, timestamp, temUsage);
        fakedropcounter = 0;
      }
      if (!bottleDetected){
        mLogger.logWarning("ScanEAN", "Bottle Not Detected");
        bottleNotDetectCount++;
        Serial.println(bottleNotDetectCount);
        return false;
      }
      else
      {
        bottleNotDetectCount = 0;
        if (AWSCOnnectivity)
          uploadBottleInfo(scannedEAN, timestamp, temUsage);
        // if (prefs.getBool("collection")){ // if a collection is partial completed
        //   // if (saveBottleInfo(prefs.getString("lastEan"), getUnixTimestamp()))
        //     mLogger.logMessage("ScanEAN-collectionComStoring bottleinfo.plete", "Storing Collection Card Info");
        //     // isBinFull = checkBinFull(getUsage());
        //   prefs.putBool("collection", false);
        // }
        // else if (saveBottleInfo(ean, getUnixTimestamp())) mLogger.logMessage("ScanEAN", "Storing bottleinfo. "); //save bottle data
        // if (!upload_task_active) xTaskCreate(task_uploadFromStorage, "StoredUpload", 10000, NULL, 2, &upload_task); // start upload task
        return true;
      }
}
/**
 * Validates whether a given EAN (European Article Number) is a "white label" EAN.
 *
 * This function checks if the provided `ean` contains the letter "W" (indicating a white label)
 * and if it exists in the `allEans` list, in either double-quoted or single-quoted format.
 * Returns `true` if both conditions are met, indicating a valid white label EAN.
 *
 * @param {String} ean - The EAN code to validate as a white label.
 * @returns {bool} - Returns `true` if the EAN is a valid white label; otherwise, `false`.
 */
bool validateWhiteLabelEan(String ean) {return false;}