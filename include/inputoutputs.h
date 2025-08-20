#include <ESP32Servo.h>
#include <Wire.h>


Servo servoComp1, servoComp2, servoComp3, servoComp4;
Servo* servos[] = {&servoComp1, &servoComp2, &servoComp3, &servoComp4};
// void binfulleffect(struct CRGB *targetArray);
void turnOffRGB(struct CRGB * targetArray);
void loadingColor(uint8_t i, CRGB color, uint8_t ledNum);
void breathingColorEffect(struct CRGB * targetArray, const CRGB &color, uint8_t ledNum);
void movingColorEffect(struct CRGB * targetArray, const CRGB& color, uint8_t ledNum);
extern bool uploadFakedrop(int count, String timestamp, float usage);
extern bool uploadBottleInfo(String ean, String timestamp, float usage);
float printVertiaclSensorReads(float usage);
bool uploadBarcodeScanned(String ean, String timestamp, float usage);
// Define the colors you want to cycle through
CRGB color1 = CRGB(0, 0, 255);   // 227, 22, 98
CRGB color2 = CRGB(128, 0, 128); // 92, 64, 153
CRGB color3 = CRGB(255, 0, 0);   // 71, 101, 230
CRGB sensorsColor = CRGB(0, 0, 255);  //navy blue color
CRGB comp2Cservoolor=CRGB (45, 39, 27); //dark chocolate brown color
CRGB comp3Color=CRGB(50, 50, 50); // deep charchoal grey color
CRGB comp4Color=CRGB(55, 0, 55); //Rich Blackcurrant Purple
// CRGB *compColor[4]={&comp1Color, &comp2Color, &comp3Color, &comp4Color};

/**
 * @brief Controls the lock mechanism by setting the lock pin to the desired status.
 * 
 * @param status Boolean value indicating the lock state. If true, the lock will be engaged; if false, it will be disengaged.
 * 
 * @note This function uses digitalWrite to control the lock pin.
 */
void turn_Lock(int16_t lockPin, bool status) {
  digitalWrite(lockPin, status);
}

/**
 * @brief Opens the servo to the predefined open position.
 * 
 * This function moves the servo to the position specified by `SOpenPos`, which represents the open position for the bin lid.
 * 
 * @note A delay of 300 milliseconds is included to allow the servo to reach the open position before any further commands.
 */
void open_servo(Servo* myServo, int16_t position) {
  mLogger.logMessage("servo open:,", String(position));
  myServo->write(position);
  vTaskDelay(300/portTICK_PERIOD_MS);
}

/**
 * @brief Closes the servo to the predefined closed position.
 * 
 * This function moves the servo to the position specified by `SClosePos`, which represents the closed position for the bin lid.
 * 
 * @note A delay of 300 milliseconds is included to allow the servo to reach the closed position.
 */
void close_servo(Servo* myServo, int16_t position) {
  mLogger.logMessage("servo close:,", String(position));
  myServo->write(position);
  vTaskDelay(300/portTICK_PERIOD_MS);
}
/**
 * @brief Sets the servo to a zero position for installation purposes.
 * 
 * This function moves the servo to a fixed position (115 degrees) to facilitate installing the lid.
 * 
 * @note A delay of 300 milliseconds is included to give the servo time to reach the zero position.
 */
void zero_servo(Servo* myServo) {
  myServo->write(70);    // zero position for servo to install the lid
  vTaskDelay(200/portTICK_PERIOD_MS);
}
/**
 * @brief Initializes the servo and lock components.
 * 
 * This function sets up the servo with a frequency of 50 Hz and attaches it to a specified pin with given pulse widths. It also sets up the lock pin as an output and sets it to the default unlocked state.
 * 
 * @note The function closes the servo by default after initializing to ensure it's in the starting position.
 */
// Modify actuatorsBegin to use pointer dereferencing
void actuatorsBegin() {
  for (uint8_t i=0; i<4; i++){
    if (i==0)  FastLED.addLeds<WS2812B, RGB_CAN_PIN, GRB>(RGB_Can, NUM_LEDS);
    else if(i==1) FastLED.addLeds<WS2812B, RGB_PET_PIN, GRB>(RGB_Pet, 8);
    else if (i==2) FastLED.addLeds<WS2812B, RGB_PAPER_PIN, GRB>(RGB_Paper, 8);
    else  FastLED.addLeds<WS2812B, RGB_Additional_PIN, GRB>(RGB_Additional, 8);
    FastLED.setBrightness(BRIGHTNESS);  // Set brightness
    FastLED.show();
    servos[i]->setPeriodHertz(50);    // standard 50 hz servo
    servos[i]->attach(servoPins[i], 340, 2700); //500, 2400
    pinMode(lockPins[i], OUTPUT);
    digitalWrite(lockPins[i], LOW);
    Serial.println("servo close angle for " + *compName[i] + " is " + String(SClosePos[i]));
    close_servo(servos[i], SClosePos[i]);
    vTaskDelay(10/portTICK_PERIOD_MS);
    // turnOffRGB(RGBcomp[i]);
  }
}

/**
 * Checks if the bin is full based on the current usage percentage.
 * If the usage is greater than or equal to 98%, the bin is considered full.
 * The function will also turn off the RGB LED when the bin is not full.
 * 
 * @param {float} usage - The current usage percentage (0 to 100) of the bin.
 * @returns {boolean} - Returns `true` if the bin is full (usage >= 98%), otherwise returns `false`.
 */
bool checkBinFull(float usage, uint8_t i)
{
  // Serial.println("Is Bin FUll function is called");
  // mLogger.logInfo("checkBinFull() function: usage", String(usage) +" , "+String(i));
  if (usage > 98.0 && bottleCount>5 && !i) return true;
  else if (usage > 98.0 && i==1) return true;
    return false;
}


float VAvrage(String Sensorname, Adafruit_VL53L0X &sensor, TCA9548A &mux, uint8_t channel) {
  // if(sensorFail) return 0;
  
  float VAvgValue = 0.0;
  int validReadings = 0;
  int readingsCount = 5;  // Use 5 readings for averaging
  float readings[5] = {0.00, 0.00, 0.00, 0.00, 0.00};
  mux.openChannel(channel);

  vTaskDelay(1 / portTICK_PERIOD_MS);  // Wait for the measurement to complete
  for (int i = 0; i < readingsCount; i++) {
    VL53L0X_RangingMeasurementData_t measure; // Declare the measure variable
    sensor.rangingTest(&measure, false);  // Start a single shot ranging

    if (measure.RangeStatus != 4 ) {  // Check if the reading is valid
      if((measure.RangeMilliMeter)){readings[validReadings] = (measure.RangeMilliMeter / 10);  // Convert to centimeters
      validReadings++;}
    } else {
      Serial.println("Invalid reading detected");
    }
    // Serial.println("Reading "+String(readings[validReadings]));

    // if (measure.RangeStatus != 4 && measure  .RangeStatus < 840)
    // {                                                            // Check if the reading is valid
    //   if ((measure.RangeMilliMeter / 10.0) < 800){
    //     readings[validReadings] = measure.RangeMilliMeter / 10.0; // Convert to centimeters
    //     validReadings++;
    //   }
    // }
    // else
    // {
    //   Serial.println("Invalid reading detected");
    // }
    // vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  mux.closeChannel(channel);
  if (validReadings > 0) {
    // Sort the readings array
    for (int i = 0; i < validReadings - 1; i++) {
      for (int j = 0; j < validReadings - i - 1; j++) {
        if (readings[j] > readings[j + 1]) {
          float temp = readings[j];
          readings[j] = readings[j + 1];
          readings[j + 1] = temp;
        }
      }
    }

    // Compute the average, possibly excluding outliers
    float sum = 0.0;
    int start = (validReadings > 2) ? 1 : 0;  // If more than 2 readings, exclude the min and max
    int end = (validReadings > 2) ? validReadings - 1 : validReadings;

    for (int i = start; i < end; i++) {
      sum += readings[i];
    }

    VAvgValue = sum / (end - start);
  } else {
    VAvgValue = -1;  // Return a specific value to indicate an error
  }
  // mux.closeChannel(channel);

  // Serial.print(Sensorname);
  // Serial.print(" V range: ");
  // Serial.println(VAvgValue);

  return VAvgValue;
}

/**
 * Calculates the bin usage percentage based on the average distance from a sensor.
 * The distance is measured by the `VAvrage` function. The bin usage is calculated based on how full the bin is, 
 * with `binfullDis` representing the threshold for when the bin is considered full.
 * 
 * The formula calculates usage as a percentage between 0 and 100, where `0` means empty and `100` means full.
 * If an error occurs (e.g., distance sensor failure), the function returns `-1`.
 * 
 * @returns {float} - The current bin usage as a percentage (0 to 100), or `-1` if there was an error in obtaining the distance.
 */
 float getUsage(String Sensorname, Adafruit_VL53L0X & sensor, TCA9548A & mux, uint8_t channel, uint8_t i){
  // { int offset=(bottleCount>65)?85:65;
    float usage = 0.0;
    float distance = VAvrage(Sensorname, sensor, mux, channel);
    // distance +=(offset-bottleCount);
    // Serial.printf("Distance: %3.2f\n" , distance);
  if (distance == -1) {
    // Serial.println("fail vertical sensors");
    usage = -1.0;
    // mLogger.logInfo(String("getUsage() function: avrage distance"), String(distance));
    // mLogger.logInfo(String("usage"), String(usage));
    myusage = usage;
    return usage;
  }
  else if (distance <= 0)
  {
    usage = 0;
    // Serial.println("Distance below Zero.");
    // mLogger.logInfo(String("getUsage() function: avrage distance"), String(distance));
    // mLogger.logInfo(String("usage"), String(usage));
    myusage = usage;
    return usage;
  }
  else
  {
    // usage = (float)(1 - ((distance - 10) / (30 - binfullDis))) * (float)100;
    usage = (float)(1 - ((distance - binfullDis) / (binSize - binfullDis))) * (float) 100;
    mLogger.logInfo(("getUsage Small"), String(usage));
  }
  usage = constrain(usage,0.00,100.00);      
  // mLogger.logInfo(String("getUsage() function: avrage distance"), String(distance));
  mLogger.logInfo(("getUsage Small"), String(usage));
  myusage = usage;
  // Serial.printf("usage: %3.2f\n" , usage);
  return usage;
}

float getUsage(float temUsage)
  {
    float usage = 0.0;
    float distance=printVertiaclSensorReads(usage);
    // Serial.printf("Distance big-ewaste : %3.2f\n" , distance);
  if (distance == -1) {
    // Serial.println("fail vertical sensors");
    usage = -1.0;
    // mLogger.logInfo(String("getUsage() function: avrage distance"), String(distance));
    // mLogger.logInfo(String("usage"), String(usage));
    myusage = usage;
    return usage;
  }
  else if (distance <= 0)
  {
    usage = 0;
    // Serial.println("Distance below Zero.");
    // mLogger.logInfo(String("getUsage() function: avrage distance"), String(distance));
    // mLogger.logInfo(String("usage"), String(usage));
    myusage = usage;
    return usage;
  }
  else
  {
    int size=800,binfullDistace=100;
    if(distance<100) {size=80;binfullDistace=10;}
    else if(distance>1000) {size=7000;binfullDistace=1000;}
    else if(distance>300) {size=800;binfullDistace=100;}
    usage = (float)(1 - ((distance - binfullDistace) / (size - binfullDistace))) * (float)100;
    if (usage >=90.00) usage=100;
  }
  // mLogger.logInfo(("usage big-ewaste"), String(usage));
  usage = constrain(usage,0.00,100.00);      
  // mLogger.logInfo(String("getUsage() function: avrage distance"), String(distance));
  mLogger.logInfo(("usage big-ewaste"), String(usage));
  myusage = usage;
  // Serial.printf("usage: %3.2f\n" , usage);
  return usage;
}

/**
 * @brief Creates a color-filling effect on the LED strip with a specified color.
 * 
 * This function gradually fills the LED strip with the specified color, lighting each LED one-by-one with a delay to create a cascading effect.
 * 
 * @param color The color to set for each LED. It is passed as a constant reference to avoid copying.
 * 
 * @note The function uses a 35-millisecond delay between lighting each LED to control the speed of the filling effect.
 *       Adjust this delay to make the effect faster or slower as desired.
 */
void colorEffect(struct CRGB * targetArray, const CRGB& color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    targetArray[i] = color;
    FastLED.show();
    vTaskDelay(35 / portTICK_PERIOD_MS); // Small delay for smooth animation
  }
}
void turnOffRGB(struct CRGB * targetArray) {
  fill_solid(targetArray, NUM_LEDS, CRGB::Black);
  FastLED.show();
}
void pulsingWhiteEffectF(struct CRGB * targetArray) {
  static uint8_t brightness = 0;

  // Adjust the range of brightness according to your preference
  // Here, we're using the full range from 0 to 255
  brightness = beatsin8(30, 0, 255);

  // Set all LEDs to white with the current brightness
  fill_solid(targetArray, NUM_LEDS, CRGB::White);
  fadeToBlackBy(targetArray, NUM_LEDS, 255 - brightness); // Fade out based on brightness
  FastLED.show();
  vTaskDelay(5 / portTICK_PERIOD_MS); // Small delay for smooth animation
}
/**
 * @brief Triggers a pulsing grey effect on the LED strip.
 * 
 * This function is called when the bin is not full, there is no internet connection,
 * and AWS connectivity is not established. It makes the LEDs pulse in grey to indicate
 * an offline and unconnected state.
 */
void pulsingGreyEffectS(struct CRGB * targetArray, uint8_t ledNum){
 static uint8_t brightness = 0;
  // Adjust the range of brightness according to your preference
  // Here, we're using the full range from 0 to 255
  brightness = beatsin8(10, 0, 255);

  // Set all LEDs to white with the current brightness
  fill_solid(targetArray, ledNum, CRGB::Gray);
  fadeToBlackBy(targetArray, ledNum, 255 - brightness); // Fade out based on brightness
  FastLED.show();
  vTaskDelay(25 / portTICK_PERIOD_MS); // Small delay for smooth animation
}
/**
 * @brief Triggers a loading effect on the LED strip with a specified color.
 * 
 * This function is called when the bin is not full, the device is connected to the internet, 
 * but AWS connectivity is not yet established. It creates a loading effect every second
 * by updating the LED color to indicate that the connection is in progress.
 * 
 * @param counter The current counter for LED state, which increments with each call to cycle through states.
 * @param color The CRGB color used for the loading effect (e.g., CRGB::Yellow).
 */
void loadingEffect(struct CRGB * targetArray, int state, CRGB color, uint8_t ledNum)
{
  int progress = state % ledNum;
  turnOffRGB(targetArray);
  fill_solid(targetArray, progress, color);
  FastLED.show();
  vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay for smooth animation
}
void binfulleffect(struct CRGB *targetArray, uint8_t ledNum)
{
  static uint8_t brightness = 0;

  // Adjust the range of brightness according to your preference
  // Here, we're using the full range from 0 to 255
  brightness = beatsin8(30, 0, 255);

  // Set all LEDs to white with the current brightness
  fill_solid(targetArray, ledNum, CRGB::Red);
  fadeToBlackBy(targetArray, ledNum, 255 - brightness); // Fade out based on brightness
  FastLED.show();
  // delay(5); // Adjust the delay for the speed of pulsing
  vTaskDelay(5 / portTICK_PERIOD_MS);
}
void calibrationeffect(struct CRGB *targetArray, uint8_t ledNum)
{
  static uint8_t brightness = 255;

  // Adjust the range of brightness according to your preference
  // Here, we're using the full range from 0 to 255
  // brightness = beatsin8(30, 0, 255);

  // Set all LEDs to white with the current brightness
  fill_solid(targetArray, ledNum, CRGB::Orange);
  // fadeToBlackBy(targetArray, NUM_LEDS, 255 - brightness); // Fade out based on brightness
  FastLED.show();
  // delay(5); // Adjust the delay for the speed of pulsing
  vTaskDelay(5 / portTICK_PERIOD_MS);
}
void collectionEffect() {
  static uint8_t brightness = 0;

  // Adjust the range of brightness according to your preference
  // Here, we're using the full range from 0 to 255
  brightness = beatsin8(30, 0, 255);
  for (uint8_t i = 0; i < 4; i++)
  {
    if(compartmentState[0] || EwasteCompartmentState[1])
    {
      // Set all LEDs to white with the current brightness
      fill_solid(RGBcomp[i], NUM_LEDS, CRGB::Green);
      fadeToBlackBy(RGBcomp[i], NUM_LEDS, 255 - brightness); // Fade out based on brightness
      FastLED.show();
      // delay(5); // Adjust the delay for the speed of pulsing
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
  }
}
void fadeColorEffect(struct CRGB * targetArray, const CRGB& color, uint8_t ledNum) {
  for (int k = 0; k < 256; k++) {
    for (int i = 0; i < ledNum; i++) {
      targetArray[i].fadeToBlackBy(255 - k);
      targetArray[i] += color;
    }
    FastLED.show();
    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay for smooth animation // Adjust the delay for the speed of filling
  }
}
void turnOffEffect(struct CRGB * targetArray) {
  for (int i = NUM_LEDS; i >= 0; i--) {
    targetArray[i] = CRGB::Black;
    FastLED.show();
    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay for smooth animation // Adjust the delay for the speed of filling
  }
}
void movingColorEffect(struct CRGB * targetArray, const CRGB& color, uint8_t ledNum) {
  for (int i = 0; i <= ledNum; i++) {
    // fill_solid(targetArray, NUM_LEDS, CRGB::Black);
    targetArray[i] = color;
    FastLED.show();
    delay(50);
    // vTaskDelay(50 / portTICK_PERIOD_MS); // Small delay for smooth animation
  }
}
void rainbowEffect(struct CRGB * targetArray) {
  static uint8_t hue = 0;
  for (int i = 0; i < NUM_LEDS; i++) {
    targetArray[i] = CHSV(hue + (i * 255 / NUM_LEDS), 255, 255);
  }
  FastLED.show();
  vTaskDelay(5 / portTICK_PERIOD_MS); // Small delay for smooth animation
  hue++;
}
/**
 * @brief Triggers a breathing effect with brand colors on the LED strip.
 * 
 * This function is called when the bin is not full, the device is connected to the internet,
 * and AWS connectivity is established. The LEDs cycle through brand-specific colors with a 
 * breathing effect to indicate that everything is functioning normally.
 */
void breathingBrandColors(struct CRGB * targetArray, uint8_t ledNum)
{
  static uint8_t hue = 0;
  static int move = 0;
  uint8_t brightness = beatsin8(40, 100, 255); // Breathing effect
  // Define the segment boundaries
  int segment1End = ledNum / 3;
  int segment2End = 2 * ledNum / 3;
  for (int i = 0; i < ledNum; i++)
  {
    float position = (float)(hue + move + i) / 255.0; // Position calculation for the moving effect
    CRGB color;
    if (i < segment1End)
    {
      color = blend(color1, color2, (i * 255 / segment1End));
    }
    else if (i < segment2End)
    {
      color = blend(color2, color3, ((i - segment1End) * 255 / (segment2End - segment1End)));
    }
    else
    {
      color = blend(color3, color1, ((i - segment2End) * 255 / (ledNum - segment2End)));
    }
    targetArray[i] = color;
  }
  FastLED.setBrightness(brightness);
  FastLED.show();
  // Increment the hue and move values for the moving effect
  hue++;
  move++;
  if (move >= 255)
    move = 0;
  vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay for smooth animation
}

void breathingRainbow(struct CRGB * targetArray,uint8_t ledNum) {
  static uint8_t hue = 0;
  uint8_t brightness = sin8(hue);
  fill_rainbow(targetArray, ledNum, hue);
  FastLED.show();
  vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay for smooth animation
  hue++;
}

void breathingColorEffect(struct CRGB * targetArray, const CRGB &color, uint8_t ledNum)
{
 static uint8_t brightness = 0;
  brightness = sin8(brightness);
  fill_solid(targetArray, ledNum, color);
  FastLED.show();
  delay(20);
}
void pulsingRainbow(struct CRGB * targetArray, uint8_t ledNum) {
  static uint8_t hue = 0;
  uint8_t brightness = beatsin8(60, 0, 255);
  fill_rainbow(targetArray, ledNum, hue, 8);
  FastLED.show();
  vTaskDelay(5 / portTICK_PERIOD_MS); // Small delay for smooth animation
  hue++;
}
void pulsingColorEffect(struct CRGB * targetArray, const CRGB& color, uint8_t ledNum) {
  static uint8_t brightness = 0;
  brightness = beatsin8(60, 0, 255);
  fill_solid(targetArray, ledNum, color);
  FastLED.show();
  vTaskDelay(20 / portTICK_PERIOD_MS);
  // delay(20);
}
void bottleDropEffect(struct CRGB * targetArray)
{
  fill_solid(targetArray, NUM_LEDS, CRGB::Green);
  FastLED.show();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  
}
void FakeDropEffect(struct CRGB * targetArray)
{
  fill_solid(targetArray, NUM_LEDS, CRGB::DarkOrange);
  FastLED.show();
  vTaskDelay(300 / portTICK_PERIOD_MS);
}
void BigEwbottleDropEffect(struct CRGB * targetArray, uint8_t ledNum)
{
  fill_solid(targetArray, ledNum, CRGB::Green);
  FastLED.show();
  vTaskDelay(300 / portTICK_PERIOD_MS);
  
}
void BigEwFakeDropEffect(struct CRGB * targetArray)
{
  fill_solid(targetArray, 8, CRGB::DarkOrange);
  FastLED.show();
  vTaskDelay(300 / portTICK_PERIOD_MS);
}

void WireDvicesInit() {
  Serial.begin(115200);
  BarSanner.begin(9600,SERIAL_8N1,scanRX);


  Wire.begin();
  Wire.setClock(100000);
  Wire.setPins(21, 22);

  I2CMux.begin(Wire);   // Wire instance is passed to the library
  I2CMux.closeAll();    // Set a base state which we know (also the default state on power on)
  I2CMux_1.begin(Wire); // Wire instance is passed to the library
  I2CMux_1.closeAll();  // Set a base state which we know (also the default state on power on)

  ina3221.begin();
  ina3221.reset();
  ina3221.setShuntRes(100, 100, 100);

  if (! ina219.begin()) Serial.println("Failed to find INA219 chip");
}


/*-------------------------------------------------------------------------------------------------
                                void ina3221Init()
                                3 channels curent sensor for comparmant 1, 2 , 3    
-------------------------------------------------------------------------------------------------*/
void ina3221Init() {

  // if(!ina3221.begin()) Serial.println("Failed to find ina3221 chip");
  // ina3221.begin();
  // ina3221.reset();
  // // vTaskDelay(1000/portTICK_PERIOD_MS);
  // // Set shunt resistors to 10 mOhm for all channels
  // ina3221.setShuntRes(10, 10, 10);
  float current[3];
  float voltage[3];

  current[0] = ina3221.getCurrent(INA3221_CH1);
  voltage[0] = ina3221.getVoltage(INA3221_CH1);

  current[1] = ina3221.getCurrent(INA3221_CH2);
  voltage[1] = ina3221.getVoltage(INA3221_CH2);

  current[2] = ina3221.getCurrent(INA3221_CH3);
  voltage[2] = ina3221.getVoltage(INA3221_CH3);

  Serial.print("Channel 1: ");
  Serial.print(current[0], PRINT_DEC_POINTS);
  Serial.print("A, ");
  Serial.print(voltage[0], PRINT_DEC_POINTS);
  Serial.println("V");

  Serial.print("Channel 2: ");
  Serial.print(current[2], PRINT_DEC_POINTS);
  Serial.print("A, ");
  Serial.print(voltage[2], PRINT_DEC_POINTS);
  Serial.println("V");

  Serial.print("Channel 3: ");
  Serial.print(current[1], PRINT_DEC_POINTS);
  Serial.print("A, ");
  Serial.print(voltage[1], PRINT_DEC_POINTS);
  Serial.println("V");

  for (uint8_t i = 0; i < 5; i++)
  {
    if (ina3221.getCurrent(INA3221_CH3)>0.0)sampleReadingCurrent += ina3221.getCurrent(INA3221_CH3);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  sampleReadingCurrent /= 5.0;
  mLogger.logMessage("ina3221Init", "Current reading: " + String(sampleReadingCurrent));

  // delay(1000);
}
/*-------------------------------------------------------------------------------------------------
                                void ina219Init()
                                curent sensor for comparmant-4     
-------------------------------------------------------------------------------------------------*/
void ina219Init() 
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  busvoltage = ina219.getBusVoltage_V();
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  current_mA = ina219.getCurrent_mA();
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  power_mW = ina219.getPower_mW();
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.println("");

  // delay(2000);
}

void bottleStuck(uint8_t i)
{  
  
  vTaskDelay(500/portTICK_PERIOD_MS);
  if (i<3)
  {
    while (ina3221.getCurrent(INA3221Channels[i]) > currentThreshoulds[i])
    {
      mLogger.logMessage("bottleStuck-----------------2", String(ina3221.getCurrent(INA3221_CH3)));
      open_servo(servos[i],SOpenPos[i]);
      vTaskDelay(4000 / portTICK_PERIOD_MS);
      close_servo(servos[i],SClosePos[i]);
      vTaskDelay(300/ portTICK_PERIOD_MS);
      mLogger.logMessage("bottleStuck-----------------3", String(ina3221.getCurrent(INA3221_CH3)));
    }
  }
  else if (i==3)
  {
    while (ina219.getCurrent_mA()>currentThreshoulds[i])
    {
      mLogger.logMessage("bottleStuck-----------------2", String(ina3221.getCurrent(INA3221_CH3)));
      open_servo(servos[i],SOpenPos[i]);
      vTaskDelay(4000 / portTICK_PERIOD_MS);
      close_servo(servos[i],SClosePos[i]);
      vTaskDelay(300/ portTICK_PERIOD_MS);
      mLogger.logMessage("bottleStuck-----------------3", String(ina3221.getCurrent(INA3221_CH3)));
    }}
  
}


void findingCurrrentThreshoulds() {
  for(uint8_t i=0; i<4; i++) {
    float readings[7] = {0};
    uint8_t readCount = 0;
    
    if(compartmentState[i] || compartIusse[i]) {
      zero_servo(servos[i]);
      vTaskDelay(700/portTICK_PERIOD_MS);
      
      // Take up to 7 readings
      for(uint8_t j=0; j<7; j++) {
        if(i<3) {
          float current = ina3221.getCurrent(INA3221Channels[i]);
          if(current > 0.000) {
            readings[readCount] = current;
            mLogger.logMessage("Reading " + String(j) + " ina3221 " + String(i), String(readings[readCount]));
            readCount++;
          }
        }
        else if(i==3) {
          float current = ina219.getCurrent_mA();
          if(current > 0.000) {
            readings[readCount] = current;
            mLogger.logMessage("Reading " + String(j) + " ina219 " + String(i), String(readings[readCount]));
            readCount++;
          }
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
      }

      if(readCount >= 2) {
        // Sort the non-zero readings
        for(uint8_t j=0; j<readCount; j++) {
          for(uint8_t k=j+1; k<readCount; k++) {
            if(readings[j] > readings[k]) {
              float temp = readings[j];
              readings[j] = readings[k];
              readings[k] = temp;
            }
          }
        }

        float sum = 0;
        uint8_t startIndex;
        uint8_t validReadings;

        switch(readCount) {
          case 2:
            startIndex = 0;
            validReadings = 2;
            break; 
          case 3:
            startIndex = 1;
            validReadings = 2;
            break;
          default:  // 4 or more readings
            startIndex = 1;
            validReadings = readCount - 1;
            break;
        }

        // Calculate average
        for(uint8_t j=startIndex; j<readCount; j++) {
          sum += readings[j];
        }
        currentThreshoulds[i] = sum/float(validReadings);
        
        mLogger.logMessage("Final threshold " + String(i), String(currentThreshoulds[i]));
      } else {
        // currentThreshoulds[i] = 0;
        mLogger.logMessage("Not enough valid readings for threshold " + String(i), "0");
      }
      
      vTaskDelay(50/portTICK_PERIOD_MS);
      servos[i]->write(SClosePos[i]);
      vTaskDelay(400/portTICK_PERIOD_MS);
    }
  }
}

// void findingCurrrentThreshoulds(){
//   for(uint8_t i=0; i<4; i++){
//     float tempCurrent=0.000;
//     uint8_t divCounter=0;
//     if(compartmentState[i] ||compartIusse[i]){
//     zero_servo(servos[i]);
//     vTaskDelay(700/portTICK_PERIOD_MS);
//     for(uint8_t j=0; j<5; j++){
//     if(ina3221.getCurrent(INA3221Channels[i])>0.000 & i<3){
//         currentThreshoulds[i]+=ina3221.getCurrent(INA3221Channels[i]);
//         divCounter++;
//         mLogger.logMessage("threshould ina3221 " + String(i), String(currentThreshoulds[i]));
//       }
//       else if(i==3 && ina219.getCurrent_mA()>0.000){
//         currentThreshoulds[i]+=currentThreshoulds[i]=ina219.getCurrent_mA(); 
//         divCounter++;
//         mLogger.logMessage("threshould  ina219 " + String(i), String(currentThreshoulds[i]));
//       }
//     vTaskDelay(10/portTICK_PERIOD_MS);
//     }
//     currentThreshoulds[i]=(currentThreshoulds[i]/float(divCounter));
//     mLogger.logMessage("threshould " + String(i), String(currentThreshoulds[i]));
//     }
//     vTaskDelay(50/portTICK_PERIOD_MS);
//     close_servo(servos[i], SClosePos[i]);
//     vTaskDelay(300/portTICK_PERIOD_MS);
//   } 
// }
void calibrateServo(uint8_t i) {
  const uint8_t CALIBRATION_ATTEMPTS = 3;
  uint16_t minAngle = 0xFFFF;  // Start with maximum value
  uint16_t originalPos = SClosePos[i];
  String prefKey="";
  prefKey= "servoClose" + String(i);
  prefs.begin("configs", false);
  
  // Log initial current reading
  if(i < 3) {
    mLogger.logMessage("calibrateServo ina3221 " + String(i), String(ina3221.getCurrent(INA3221Channels[i])));
  } else {
    mLogger.logMessage("calibrateServo ina219 " + String(i), String(ina219.getCurrent_mA()));
  }
  // make sure it hits the end of the closing point, making the current pretty heigher than the threshould already set
  // float current = (i < 3) ? ina3221.getCurrent(INA3221Channels[i]) : ina219.getCurrent_mA();
  // uint8_t idx=0;
  // while(current >currentThreshoulds[i]*1.5){
  // idx+=5;
  servos[i]->write(0);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  // current = (i < 3) ? ina3221.getCurrent(INA3221Channels[i]) : ina219.getCurrent_mA();
  // }
  // Perform calibration attempts
  for(uint8_t attempt = 0; attempt < CALIBRATION_ATTEMPTS; attempt++) {
    bool isCalibrating = true;
    SClosePos[i] = originalPos; // Reset position for each attempt
    
    while(isCalibrating) {
      float current = (i < 3) ? ina3221.getCurrent(INA3221Channels[i]) : ina219.getCurrent_mA();
      
      if(current > currentThreshoulds[i]) {
        SClosePos[i] += 2;
        servos[i]->write(SClosePos[i]);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        current = (i < 3) ? ina3221.getCurrent(INA3221Channels[i]) : ina219.getCurrent_mA();
        
        if(current <= currentThreshoulds[i]) {
          // Update minAngle if current position is smaller
          if(SClosePos[i] < minAngle) {
            minAngle = SClosePos[i];
            mLogger.logMessage("New minimum angle found", String(minAngle));
          }
          isCalibrating = false;
        }
      } else {
        isCalibrating = false;
      }
    }
    // findingCurrrentThreshoulds();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  // Only update and save if we found a valid minimum angle
  if(minAngle != 0xFFFF) {
    SClosePos[i] = minAngle;
    servos[i]->write(SClosePos[i]);
    prefs.putInt(prefKey.c_str(), SClosePos[i]);
    mLogger.logMessage("Final calibration " + String(i), 
                      "Minimum angle: " + String(SClosePos[i]));
  }
  prefs.end();
}
// void calibrateServo(uint8_t i)
// {
//   bool isCalibrating = true;
//   float current;
//   String prefKey="";
//   prefs.begin("configs", false);
//   (i < 3) ? mLogger.logMessage("calibrateServo ina3221 " + String(i), String(ina3221.getCurrent(INA3221Channels[i]))) : mLogger.logMessage("calibrateServo ina219" + String(i), String(ina219.getCurrent_mA()));
//   while (isCalibrating)
//   {
//     // Serial.println("calibrateServo teat 1");
//     current = (i < 3) ? ina3221.getCurrent(INA3221Channels[i]) : ina219.getCurrent_mA();
//     if ((i < 3 && current > currentThreshoulds[i]) || (i == 3 && current > currentThreshoulds[i]))
//     {
//       {
//         // Serial.println("calibrateServo teat 2");
//         SClosePos[i] += 2;
//         servos[i]->write(SClosePos[i]);
//         // close_servo(servos[i], SClosePos[i]);
//         vTaskDelay(30 / portTICK_PERIOD_MS);
//         current = (i < 3) ? ina3221.getCurrent(INA3221Channels[i]) : ina219.getCurrent_mA();
//         if ((i < 3 && current <= currentThreshoulds[i]) || (current <= currentThreshoulds[i] && i == 3))
//         {
//           // Serial.println("calibrateServo teat 3");
//           if (i < 3)
//           { 
//             mLogger.logMessage("calibrateServo ina3221 " + String(i), String(ina3221.getCurrent(INA3221Channels[i])));
//             prefKey="ServoCloseAngle"+String(i);
//             prefs.putInt(prefKey.c_str(), SClosePos[i]);
//             // Serial.println("calibrateServo teat 4");
//           }
//           else if (i == 3)
//           {
//             mLogger.logMessage("calibrateServo ina219 " + String(i), String(ina219.getCurrent_mA()));
//             prefKey="ServoCloseAngle"+String(i);
//             prefs.putInt(prefKey.c_str(), SClosePos[i]);
//             // Serial.println("calibrateServo teat 5");
//           }
//           mLogger.logMessage("calibrateServo " + String(i), "save servo positions----");
//           prefs.begin("configs", false);
//           prefs.putUInt("ServoCloseAngle", SClosePos[i]);
//           prefs.end();
//           isCalibrating = false;
//           // 
//         }
//       }
//     }
//     else isCalibrating = false;
//   }
//   prefs.end();
// }
/*-------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------*/
void servoTesting()
{
  float currentOpen = 0.0, currentClose = 0.0;
  mLogger.logMessage("servoTesting", "Testing servo");
  for (uint8_t i = 0; i < 4; i++)
  {
    if (compartmentState[i])
    {
      currentOpen = (i < 3) ? ina3221.getCurrent(INA3221Channels[i]) : ina219.getCurrent_mA();
      mLogger.logMessage("servoTesting - currentOpen", String(currentOpen));
      vTaskDelay(10 / portTICK_PERIOD_MS);
      for (uint8_t j = 0; j < 5; j++)
      {
        open_servo(servos[i], SOpenPos[i]);
        mLogger.logMessage("servo:,", String(SOpenPos[i]));
        close_servo(servos[i], SClosePos[i]);
        mLogger.logMessage("servo:,", String(SClosePos[i]));
        // vTaskDelay(100 / portTICK_PERIOD_MS);
        // close_servo(servos[i], SClosePos[i]);
        // vTaskDelay(200 / portTICK_PERIOD_MS);
        currentClose += (i < 3) ? ina3221.getCurrent(INA3221Channels[i]) : ina219.getCurrent_mA();
      }
      currentClose /= 5;
      mLogger.logMessage("servoTesting - currentClose", String(currentClose));
      if (currentClose <= currentOpen)
      {
        servoLimit[i] = true;
        mLogger.logMessage("servoTesting - servoLimit" + String(servoLimit[i]), String(currentClose));
        ServoIssues[i] = "Servo " + String(compartmentState[i]) + " on compartment " + String(i + 1) + " has issues";
      }
      else
        servoLimit[i] = false;
      // vTaskDelay(200 / portTICK_PERIOD_MS);
    }
  }

  // return std::make_tuple(servoLimit[0], servoLimit[1], servoLimit[2], servoLimit[3]);
}

// Using it:
// auto [a, b, c, d] = servoTesting();

  /*-------------------------------------------------------------------------------------------------
                                  bool IsloxConnected(byte address)
                                  check if Down or Up TOF sensor are connected
  -------------------------------------------------------------------------------------------------*/
  bool IsloxConnected(uint8_t address)
  {
    uint8_t error;
    Wire.beginTransmission(address); // Begin transmission to the specified address
    error = Wire.endTransmission();  // End transmission and check for an error
    if (error == 0)
    {
      Serial.println("Device found at address 0x" + String(address, HEX));
      return true;
    } else {
    // Serial.println("Device not found at address 0x" + String(address, HEX));
    return false;
  }
}
/*-------------------------------------------------------------------------------------------------
                                initialize_Adafruit_VL6180X
                                initialize Down or Up TOF sensor either     
-------------------------------------------------------------------------------------------------*/
void initializeHorizontalSensor(String Sensorname, Adafruit_VL6180X &sensor, TCA9548A &mux, uint8_t channel, uint8_t address)
{
  mux.openChannel(channel);
  delay(10);
  if (sensor.begin())
  {
    #ifdef DEBUG_LOG 
    Serial.print("H Sensor '");
    Serial.print(Sensorname);
    Serial.print("' on '"); 
    Serial.print(channel);
    Serial.println("' initialized");
    #endif
    sensor.setAddress(address);
    sensorFail = false;
  }
  else
  {
    #ifdef DEBUG_LOG 
    Serial.print("H Sensor '");
    Serial.print(Sensorname);
    Serial.print("' on '");
    Serial.print(channel);
    Serial.println("' fail to initialized !!!");
    #endif
    sensorFail = true;
  }
  mux.closeChannel(channel);

}
/*-------------------------------------------------------------------------------------------------
                                initialize_Adafruit_VL6180X
                                initialize Vertical TOF sensor      
-------------------------------------------------------------------------------------------------*/
void initializeVerticalSensor(String Sensorname, Adafruit_VL53L0X &sensor, TCA9548A &mux, uint8_t channel)
{
  mux.openChannel(channel);
  delay(10);
  if (sensor.begin())
  {
    #ifdef DEBUG_LOG 
    Serial.print("V Sensor '");
    Serial.print(Sensorname);
    Serial.print("' on '");
    Serial.print(channel);
    Serial.println("' initialized");
    #endif
    sensor.setMeasurementTimingBudgetMicroSeconds(50);
    sensorFail = false;
  }
  else
  {
    #ifdef DEBUG_LOG 
    Serial.print("V Sensor '");
    Serial.print(Sensorname);
    Serial.print("' on '");
    Serial.print(channel);
    Serial.println("' fail to initialize !!!");
    #endif
    sensorFail = true;
  }
  mux.closeChannel(channel);
}
/*-------------------------------------------------------------------------------------------------
                                Testing_Adafruit_VL6180X
                                Sampling readings from Down and UP TOF sensor    
-------------------------------------------------------------------------------------------------*/
void Testing_Adafruit_VL6180X(String Sensorname_1, String Sensorname_2, Adafruit_VL6180X &lox_1, Adafruit_VL6180X &lox_2, TCA9548A &mux_1, TCA9548A &mux_2, uint8_t channel1, uint8_t channel2)
{
  if(sensorFail) return;
  mux_1.openChannel(channel1);
  mux_2.openChannel(channel2);
  // delay(500);

  lox_1.startRange();
  lox_1.waitRangeComplete();
  // delay(500);

  lox_2.startRange();
  lox_2.waitRangeComplete();
  // delay(500);

  uint8_t current_range_lox1 = lox_1.readRangeResult();
  uint8_t current_range_lox2 = lox_2.readRangeResult();

    Serial.print(Sensorname_1);
    Serial.print(" Up range value: ");
    Serial.print(current_range_lox1); 
    Serial.print(" -- Up Address: ");
    Serial.println(lox_1.getAddress(), HEX); 

    Serial.print(Sensorname_2);
    Serial.print(" Down range value: ");
    Serial.print(current_range_lox2);
    Serial.print(" -- Down Address: ");
    Serial.println(lox_2.getAddress(), HEX); 

  mux_1.closeAll();
  mux_2.closeAll();
}

/**
 * @file SensorConfig.cpp
 * @brief Configuration and initialization of sensors for a smart bin project.
 *
 * This code defines the structure and initialization logic for the sensors in each
 * compartment of a smart bin. Each compartment contains two horizontal and one
 * vertical sensor. The program checks for sensor detection and stores the detection
 * status in non-volatile storage.
 */

// Define sensor configuration struct for horizontal sensors
struct SensorHorizontalConfig {
  String sensorName;         ///< Name of the sensor
  Adafruit_VL6180X &sensor;  ///< Reference to the horizontal sensor object
  uint8_t muxID;             ///< Multiplexer ID for channel selection
  uint8_t channel;           ///< Channel number on the multiplexer
};

// Define sensor configuration struct for vertical sensors
struct SensorVerticalConfig {
  String sensorName;         ///< Name of the sensor
  Adafruit_VL53L0X &sensor;  ///< Reference to the vertical sensor object
  uint8_t muxID;             ///< Multiplexer ID for channel selection
  uint8_t channel;           ///< Channel number on the multiplexer
};

// Define a compartment struct that groups horizontal and vertical sensors
struct compartmentSensor {
  SensorHorizontalConfig hSensor1; ///< First horizontal sensor
  SensorHorizontalConfig hSensor2; ///< Second horizontal sensor
  SensorVerticalConfig vSensor;    ///< Vertical sensor
};

// Initialize array with compartment configurations
/**
 * @struct compartmentSensor
 * @description Array of VL53L0X sensors configuration for each compartment in the bin.
 * Each row represents one compartment with three sensors (top, middle, bottom).
 * 
 * Structure format:
 * {{"sensorName", sensorObject, multiplexerID, sensorPin}, ...} for each sensor in compartment
 * 
 * Array layout:
 * [0] = Compartment 1: {lox1(top), lox2(middle), lox3(bottom)}
 * [1] = Compartment 2: {lox4(top), lox5(middle), lox6(bottom)}
 * [2] = Compartment 3: {lox7(top), lox8(middle), lox9(bottom)}
 * [3] = Compartment 4: {lox10(top), lox11(middle), lox12(bottom)}
 * 
 * Parameters for each sensor:
 * - sensorName: String identifier for the sensor (e.g., "lox1")
 * - sensorObject: VL53L0X object reference
 * - multiplexerID: ID of the multiplexer (0 or 1) the sensor is connected to
 * - sensorPin: Pin number on the multiplexer (0-7)
 */
compartmentSensor compSensors[] = {
  {{"lox1", lox1, 0, 0}, {"lox2", lox2, 0, 1}, {"lox3", lox3, 0, 2}},
  // {{"lox4", lox4, 1, 0}, {"lox5", lox5, 1, 1}, {"lox6", lox6, 1, 2}},
  // {{"lox7", lox7, 0, 5}, {"lox8", lox8, 0, 6}, {"lox9", lox9, 0, 7}},
  // {{"lox10", lox10, 1, 5}, {"lox11", lox11, 1, 6}, {"lox12", lox12, 1, 7}} 
};

  // {{"lox4", lox4, 1, 0}, {"lox5", lox5, 1, 1}, {"lox6", lox6, 1, 2}},
  // {{"lox7", lox7, 0, 5}, {"lox8", lox8, 0, 6}, {"lox9", lox9, 0, 7}},

  // {{"lox4", lox4, 0, 5}, {"lox5", lox5, 0, 6}, {"lox6", lox6, 0, 7}},
  // {{"lox7", lox7, 1, 0}, {"lox8", lox8, 1, 1}, {"lox9", lox9, 1, 2}},

/**
 * @brief Check if at least one sensor is detected but not all sensors.
 *
 * @param currentState Array of current detection states of sensors.
 * @return true If at least one sensor is detected and others are not.
 * @return false Otherwise.
 */
bool isPartiallyDetected(const uint8_t currentState[3])
{
  bool atLeastOneDetected = false;    // Flag for at least one sensor detected
  bool atLeastOneNotDetected = false; // Flag for at least one sensor not detected

  // Iterate over the currentState array
  for (int i = 0; i < 3; ++i)
  {
    if (currentState[i] == 1)
    {
      atLeastOneDetected = true;
    }
    else
    {
      atLeastOneNotDetected = true;
    }

    // If both conditions are met, return true
    if (atLeastOneDetected && atLeastOneNotDetected)
    {
      return true;
    }
  }

  // Return false if not partially detected
  return false;
}


// Target state for sensor detection (1 indicates expected detection for each sensor)

/**
 * @brief Detecting sensors in each compartment and store detection state.
 *
 * This function iterates through each compartment, checking the connection status
 * of horizontal and vertical sensors. If all sensors in a compartment are detected,
 * it records the detection status in non-volatile storage.
 */
void DetectCompartment()
{
  const uint8_t targetState[3] = {1, 1, 1}; // 3 sensors per compartment

  I2CMux.closeAll();
  I2CMux_1.closeAll();
  // for (int i = 0; i < 4; ++i) {
  uint8_t i = 0;
  uint8_t currentState[3] = {0, 0, 0}; // Current detection status
  // Check first horizontal sensor
  auto &muxH1 = (compSensors[i].hSensor1.muxID == 0) ? I2CMux : I2CMux_1;
  muxH1.openChannel(compSensors[i].hSensor1.channel);
  if (IsloxConnected(defaultAddress) || IsloxConnected(LOX1_ADDRESS))
  {
#ifdef DEBUG_LOG
    Serial.print("H Sensor '");
    Serial.print(compSensors[i].hSensor1.sensorName);
    Serial.print("' on channel ");
    Serial.println(compSensors[i].hSensor1.channel);
#endif
    compSensors[i].hSensor1.sensor.begin();
    compSensors[i].hSensor1.sensor.setAddress(LOX1_ADDRESS);
    currentState[0] = 1; // Mark as detected
  }
  muxH1.closeChannel(compSensors[i].hSensor1.channel);

  // Check second horizontal sensor
  auto &muxH2 = (compSensors[i].hSensor2.muxID == 0) ? I2CMux : I2CMux_1;
  muxH2.openChannel(compSensors[i].hSensor2.channel);
  if (IsloxConnected(defaultAddress) || IsloxConnected(LOX2_ADDRESS))
  {
#ifdef DEBUG_LOG
    Serial.print("H Sensor '");
    Serial.print(compSensors[i].hSensor2.sensorName);
    Serial.print("' on channel ");
    Serial.println(compSensors[i].hSensor2.channel);
#endif
    compSensors[i].hSensor2.sensor.begin();
    compSensors[i].hSensor2.sensor.setAddress(LOX2_ADDRESS);
    currentState[1] = 1; // Mark as detected
  }
  muxH2.closeChannel(compSensors[i].hSensor2.channel);

  // Check vertical sensor
  auto &muxV = (compSensors[i].vSensor.muxID == 0) ? I2CMux : I2CMux_1;
  muxV.openChannel(compSensors[i].vSensor.channel);
  if (IsloxConnected(defaultAddress))
  {
#ifdef DEBUG_LOG
    Serial.print("V Sensor '");
    Serial.print(compSensors[i].vSensor.sensorName);
    Serial.print("' on channel ");
    Serial.println(compSensors[i].vSensor.channel);
#endif
    currentState[2] = 1; // Mark as detected
    compSensors[i].vSensor.sensor.begin();
    compSensors[i].vSensor.sensor.setMeasurementTimingBudgetMicroSeconds(50);
  }
  muxV.closeChannel(compSensors[i].vSensor.channel);

  // Compare current state with target state
  bool compartmentDetected = true;
  for (int j = 0; j < 3; ++j)
  {
    if (currentState[j] != targetState[j])
    {
      compartmentDetected = false;
      break;
    }
  }
  // If all sensors in the compartment are detected
  if (compartmentDetected)
  {
    compartmentState[i] = 1;
#ifdef DEBUG_LOG
    Serial.print("Compartment '");
    Serial.print(i + 1);
    Serial.println("' detected.");
#endif
    // Store detection status based on compartment index
    prefs.begin("configs", false);
    switch (i)
    {
    case 0:
      prefs.putBool("Compart1", true);
      break;
    case 1:
      prefs.putBool("Compart2", true);
      break;
    case 2:
      prefs.putBool("Compart3", true);
      break;
    case 3:
      prefs.putBool("Compart4", true);
      break;
    default:
      prefs.putBool("Compart1", false);
      prefs.putBool("Compart2", false);
      prefs.putBool("Compart3", false);
      prefs.putBool("Compart4", false);
      break;
    }
    prefs.end();
  }
  if (isPartiallyDetected(currentState))
  {
    String SensorNum = "";
    // Handle partial detection case
    Serial.println("Partial detection found - some sensors missing in compartment " + String(i + 1));
    for (int j = 0; j < 3; j++) // Changed 'i' to 'j' to avoid conflict with outer loop
    {
      if (currentState[j] == 0)
      {
        switch (j)
        {
        case 0:
          SensorNum += "[Horizontal up] ";
          break;
        case 1:
          SensorNum += "[Horizontal Down] ";
          break;
        case 2:
          SensorNum += "[Vertical] ";
          break;
        }
      }
    }
    // Store unique message for each compartment
    CompartmentIssues[i] = SensorNum + "on compartment " + String(i + 1) + " has issues";
    mLogger.logMessage("DetectCompartment", CompartmentIssues[i]);
    compartIusse[i] = true;
  }
  // }
}

// Define sensor configuration struct for vertical sensors
struct EwasteSensorVerticalConfig {
  String sensorName;         ///< Name of the sensor
  Adafruit_VL53L0X &sensor;  ///< Reference to the vertical sensor object
  uint8_t muxID;             ///< Multiplexer ID for channel selection
  uint8_t channel;           ///< Channel number on the multiplexer
};

// Define a compartment struct that groups horizontal and vertical sensors
struct EwastecompartmentSensor {
  EwasteSensorVerticalConfig vSensor1; ///< First horizontal sensor
  EwasteSensorVerticalConfig vSensor2; ///< Second horizontal sensor
  EwasteSensorVerticalConfig vSensor3;    ///< Vertical sensor
};

EwastecompartmentSensor EwastecompSensors[] = {
    {{"lox4", lox4, 1, 0}, {"lox5", lox5, 1, 1}, {"lox6", lox6, 1, 2}},
    {{"lox7", lox7, 0, 5}, {"lox8", lox8, 0, 6}, {"lox9", lox9,  0, 7}},
    // {{"lox10", lox10, 1, 5}, {"lox11", lox11, 1, 6}, {"lox12", lox12, 1, 7}}
    };

    // {{"lox1", lox1, 0, 0}, {"lox2", lox2, 0, 1}, {"lox3", lox3, 0, 2}},
    // {{"lox4", lox4, 1, 0}, {"lox5", lox5, 1, 1}, {"lox6", lox6, 1, 2}},
    // {{"lox7", lox7, 0, 5}, {"lox8", lox8, 0, 6}, {"lox9", lox9, 0, 7}},
// Target state for sensor detection (1 indicates expected detection for each sensor)

/**
 * @brief Detecting sensors in each compartment and store detection state.
 *
 * This function iterates through each compartment, checking the connection status
 * of horizontal and vertical sensors. If all sensors in a compartment are detected,
 * it records the detection status in non-volatile storage.
 */
void EwasteDetectCompartment()
{
  const uint8_t targetState[3] = {1, 1, 1}; // 3 sensors per compartment

  I2CMux.closeAll();
  I2CMux_1.closeAll();
  for (int i = 0; i < 2; ++i) { 
  // uint8_t i = 0;
    uint8_t currentState[3] = {0, 0, 0}; // Current detection status

    // Check first horizontal sensor
    auto &muxV1 = (EwastecompSensors[i].vSensor1.muxID == 0) ? I2CMux : I2CMux_1;
    muxV1.openChannel(EwastecompSensors[i].vSensor1.channel);
    if (IsloxConnected(defaultAddress))
    {   
      EwastecompSensors[i].vSensor1.sensor.begin();
      EwastecompSensors[i].vSensor1.sensor.setMeasurementTimingBudgetMicroSeconds(50);
      currentState[0] = 1; // Mark as detected
      SensorState[i][0] = 1;
#ifdef DEBUG_LOG
      Serial.print("E-waste V Sensor '");
      Serial.print(EwastecompSensors[i].vSensor1.sensorName);
      Serial.print("' on channel ");
      Serial.println(EwastecompSensors[i].vSensor1.channel);
#endif
    }
    muxV1.closeAll();
    delay(100);
    // Check second horizontal sensor
    auto &muxV2 = (EwastecompSensors[i].vSensor2.muxID == 0) ? I2CMux : I2CMux_1;
    muxV2.openChannel(EwastecompSensors[i].vSensor2.channel);
    if (IsloxConnected(defaultAddress))
    {
      EwastecompSensors[i].vSensor2.sensor.begin();
      EwastecompSensors[i].vSensor2.sensor.setMeasurementTimingBudgetMicroSeconds(50);
      currentState[1] = 1; // Mark as detected
      SensorState[i][1] = 1;
#ifdef DEBUG_LOG
      Serial.print("E-waste V Sensor '");
      Serial.print(EwastecompSensors[i].vSensor2.sensorName);
      Serial.print("' on channel ");
      Serial.println(EwastecompSensors[i].vSensor2.channel);
#endif
    }
    muxV2.closeAll();
    delay(100);

    // Check vertical sensor
    auto &muxV3 = (EwastecompSensors[i].vSensor3.muxID == 0) ? I2CMux : I2CMux_1;
    muxV3.openChannel(EwastecompSensors[i].vSensor3.channel);
    if (IsloxConnected(defaultAddress))
    {
      EwastecompSensors[i].vSensor3.sensor.begin();
      EwastecompSensors[i].vSensor3.sensor.setMeasurementTimingBudgetMicroSeconds(50);
      currentState[2] = 1; // Mark as detected
      SensorState[i][2] = 1;
#ifdef DEBUG_LOG
      Serial.print("E-waste V Sensor '");
      Serial.print(EwastecompSensors[i].vSensor3.sensorName);
      Serial.print("' on channel ");
      Serial.println(EwastecompSensors[i].vSensor3.channel);
#endif
    }
    muxV3.closeAll();
    // Compare current state with target state
    bool compartmentDetected = true;
    for (int j = 0; j < 3; ++j) {
      if (currentState[j] != targetState[j]) {
        compartmentDetected = false;
        break;
      }
    }
    // If all sensors in the compartment are detected
    if (compartmentDetected)
    {
      EwasteCompartmentState[i] = 1;
      #ifdef DEBUG_LOG
      Serial.print("Compartment '");
      Serial.print(i + 1);
      Serial.println("' detected.");
      #endif
      // Store detection status based on compartment index
      prefs.begin("configs", false);
      switch (i)
      {
      case 0:
        prefs.putBool("Compart1", true);
        break;
      case 1:
        prefs.putBool("Compart2", true);
        break;
      case 2:
        prefs.putBool("Compart3", true);
        break;
      case 3:
        prefs.putBool("Compart4", true);
        break;
      default:
        prefs.putBool("Compart1", false);
        prefs.putBool("Compart2", false);
        prefs.putBool("Compart3", false);
        prefs.putBool("Compart4", false);
        break;
      }
      prefs.end();
    }
    if (isPartiallyDetected(currentState))
    {
      String SensorNum = "";
      // Handle partial detection case
      Serial.println("Partial detection found - some sensors missing in compartment " + String(i + 1));
      for (int j = 0; j < 3; j++) // Changed 'i' to 'j' to avoid conflict with outer loop
      {
        if (currentState[j] == 0)
        {
          switch (j)
          {
          case 0:
            SensorNum += "[Vertical 1] ";
            break;
          case 1:
            SensorNum += "[Vertical 2] ";
            break;
          case 2:
            SensorNum += "[Vertical 3] ";
            break;
          }
        }
      }
      // Store unique message for each compartment
      CompartmentIssues[i+1] = SensorNum + "on compartment " + String(i + 1) + " has issues";
      mLogger.logMessage("DetectCompartment", CompartmentIssues[i+1]);
      compartIusse[i+1] = true;
    }
    for (uint8_t g = 0; g < 3; g++)
    {
      mLogger.logMessage("SensorState " + String(g) + String(": i = ") + String(i) + String(":"), String(SensorState[i][g]));
    }
  }
}

void diagnoseSensors()
{
  const uint8_t targetState[3] = {1, 1, 1}; // 3 sensors per compartment

  I2CMux.closeAll();
  I2CMux_1.closeAll();
  for (int i = 0; i < 4; ++i) {  // Loop through each compartment
    uint8_t currentState[3] = {0, 0, 0}; // Current detection status

    // Check first horizontal sensor
    auto &muxH1 = (compSensors[i].hSensor1.muxID == 0) ? I2CMux : I2CMux_1;
    muxH1.openChannel(compSensors[i].hSensor1.channel);
    if (IsloxConnected(LOX1_ADDRESS)) {
      currentState[0] = 1; // Mark as detected
    }
    muxH1.closeChannel(compSensors[i].hSensor1.channel);
    // Check second horizontal sensor
    auto &muxH2 = (compSensors[i].hSensor2.muxID == 0) ? I2CMux : I2CMux_1;
    muxH2.openChannel(compSensors[i].hSensor2.channel);
    if (IsloxConnected(LOX2_ADDRESS)) {
      currentState[1] = 1; // Mark as detected
    } 
    muxH2.closeChannel(compSensors[i].hSensor2.channel);

    // Check vertical sensor
    auto &muxV = (compSensors[i].vSensor.muxID == 0) ? I2CMux : I2CMux_1;
    muxV.openChannel(compSensors[i].vSensor.channel);
    if (IsloxConnected(defaultAddress)) {

      currentState[2] = 1; // Mark as detected
      compSensors[i].vSensor.sensor.setMeasurementTimingBudgetMicroSeconds(50);
    } 
    muxV.closeChannel(compSensors[i].vSensor.channel);

    // Compare current state with target state
    bool compartmentDetected = true;
    for (int j = 0; j < 3; ++j) {
      if (currentState[j] != targetState[j]) {
        compartmentDetected = false;
        break;
      }
    }
    // If all sensors in the compartment are detected
    if (compartmentDetected)
    {
      compartmentState[i] = 1;
      #ifdef DEBUG_LOG
      Serial.print("Compartment '");
      Serial.print(i + 1);
      Serial.println("' detected.");
      #endif
      // Store detection status based on compartment index
      prefs.begin("configs", false);
      switch (i)
      {
      case 0:
        prefs.putBool("Compart1", true);
        break;
      case 1:
        prefs.putBool("Compart2", true);
        break;
      case 2:
        prefs.putBool("Compart3", true);
        break;
      case 3:
        prefs.putBool("Compart4", true);
        break;
      default:
        prefs.putBool("Compart1", false);
        prefs.putBool("Compart2", false);
        prefs.putBool("Compart3", false);
        prefs.putBool("Compart4", false);
        break;
      }
      prefs.end();
    }
    if (isPartiallyDetected(currentState))
    {
      String SensorNum = "";
      // Handle partial detection case
      Serial.println("Partial detection found - some sensors missing in compartment " + String(i + 1));
      for (int j = 0; j < 3; j++) // Changed 'i' to 'j' to avoid conflict with outer loop
      {
        if (currentState[j] == 0)
        {
          switch (j)
          {
          case 0:
            SensorNum += "[Horizontal up] ";
            break;
          case 1:
            SensorNum += "[Horizontal Down] ";            
            break;
          case 2:
            SensorNum += "[Vertical] ";
            break;
          }
        }
      }
      // Store unique message for each compartment
      CompartmentIssues[i] = SensorNum + "on compartment " + String(i + 1) + " has issues";
      mLogger.logMessage("DetectCompartment", CompartmentIssues[i]);
      compartIusse[i] = true;
    }
  }
}


void eWasteDiagnoseSensors()
{
  const uint8_t targetState[3] = {1, 1, 1}; // 3 sensors per compartment

  I2CMux.closeAll();
  I2CMux_1.closeAll();
  for (int i = 0; i < 2; ++i) {  // Loop through each compartment
    uint8_t currentState[3] = {0, 0, 0}; // Current detection status

    // Check first horizontal sensor
    auto &muxV1 = (EwastecompSensors[i].vSensor1.muxID == 0) ? I2CMux : I2CMux_1;
    muxV1.openChannel(EwastecompSensors[i].vSensor1.channel);
    if (IsloxConnected(defaultAddress))
    {   
      EwastecompSensors[i].vSensor1.sensor.begin();
      EwastecompSensors[i].vSensor1.sensor.setMeasurementTimingBudgetMicroSeconds(50);
      currentState[0] = 1; // Mark as detected
#ifdef DEBUG_LOG
      Serial.print("E-waste V Sensor '");
      Serial.print(EwastecompSensors[i].vSensor1.sensorName);
      Serial.print("' on channel ");
      Serial.println(EwastecompSensors[i].vSensor1.channel);
#endif
    }
    muxV1.closeAll();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // Check second horizontal sensor
    auto &muxV2 = (EwastecompSensors[i].vSensor2.muxID == 0) ? I2CMux : I2CMux_1;
    muxV2.openChannel(EwastecompSensors[i].vSensor2.channel);
    if (IsloxConnected(defaultAddress))
    {
      EwastecompSensors[i].vSensor2.sensor.begin();
      EwastecompSensors[i].vSensor2.sensor.setMeasurementTimingBudgetMicroSeconds(50);
      currentState[1] = 1; // Mark as detected
#ifdef DEBUG_LOG
      Serial.print("E-waste V Sensor '");
      Serial.print(EwastecompSensors[i].vSensor2.sensorName);
      Serial.print("' on channel ");
      Serial.println(EwastecompSensors[i].vSensor2.channel);
#endif
    }
    muxV2.closeAll();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // Check vertical sensor
    auto &muxV3 = (EwastecompSensors[i].vSensor3.muxID == 0) ? I2CMux : I2CMux_1;
    muxV3.openChannel(EwastecompSensors[i].vSensor3.channel);
    if (IsloxConnected(defaultAddress))
    {
      EwastecompSensors[i].vSensor3.sensor.begin();
      EwastecompSensors[i].vSensor3.sensor.setMeasurementTimingBudgetMicroSeconds(50);
      currentState[2] = 1; // Mark as detected
#ifdef DEBUG_LOG
      Serial.print("E-waste V Sensor '");
      Serial.print(EwastecompSensors[i].vSensor3.sensorName);
      Serial.print("' on channel ");
      Serial.println(EwastecompSensors[i].vSensor3.channel);
#endif
    }
    muxV3.openChannel(EwastecompSensors[i].vSensor3.channel);
    // Compare current state with target state
    bool compartmentDetected = true;
    for (int j = 0; j < 3; ++j) {
      if (currentState[j] != targetState[j]) {
        compartmentDetected = false;
        break;
      }
    }
    // If all sensors in the compartment are detected
    if (compartmentDetected)
    {
      EwasteCompartmentState[i] = 1;
      #ifdef DEBUG_LOG
      Serial.print("Compartment '");
      Serial.print(i + 1);
      Serial.println("' detected.");
      #endif
      // Store detection status based on compartment index
      prefs.begin("configs", false);
      switch (i)
      {
      case 0:
        prefs.putBool("Compart1", true);
        break;
      case 1:
        prefs.putBool("Compart2", true);
        break;
      case 2:
        prefs.putBool("Compart3", true);
        break;
      case 3:
        prefs.putBool("Compart4", true);
        break;
      default:
        prefs.putBool("Compart1", false);
        prefs.putBool("Compart2", false);
        prefs.putBool("Compart3", false);
        prefs.putBool("Compart4", false);
        break;
      }
      prefs.end();
    }
    if (isPartiallyDetected(currentState))
    {
      String SensorNum = "";

      // Handle partial detection case
      Serial.println("Partial detection found - some sensors missing in compartment " + String(i + 1));
      for (int j = 0; j < 3; j++) // Changed 'i' to 'j' to avoid conflict with outer loop
      {
        if (currentState[j] == 0)
        {
          switch (j)
          {
          case 0:
            if (i == 0)
              SensorNum += "[TOF Up] ";
            else
              SensorNum += "[TOF Down] ";
            break;
          case 1:
            if (i == 0)
              SensorNum += "[TOF Up] ";
            else
              SensorNum += "[TOF Down] ";
            break;
          case 2:
            if (i == 0)
              SensorNum += "[TOF Up] ";
            else
              SensorNum += "[TOF Down] ";
            break;
          }
        }
      }
      // Store unique message for each compartment
      CompartmentIssues[i + 1] = SensorNum + "on compartment " + String(i + 1) + " has issues";
      mLogger.logMessage("DetectCompartment", CompartmentIssues[i + 1]);
      compartIusse[i + 1] = true;
    }
  }
}

//  }
 void diagnoseBin(){
  // diagnoseSensors();
  servoTesting();
  }

// Gets range measurements from two VL53L0X distance sensors (lox1 and lox2)
// Returns a pair of integers containing the measured distances in millimeters
std::pair<int, int> getLoxRanges(Adafruit_VL6180X &lox_1, Adafruit_VL6180X &lox_2, TCA9548A &mux, uint8_t channel1, uint8_t channel2) {
  // Initialize minimum range variables to maximum possible value
  uint16_t min_range_lox1 = UINT16_MAX;
  uint16_t min_range_lox2 = UINT16_MAX;
  mux.openChannel(channel1);
  mux.openChannel(channel2);
  // Take 5 measurements and keep track of minimum values
  for (int i = 0; i < 5; i++) {
  lox_1.startRange();
  lox_1.waitRangeComplete();

  lox_2.startRange();
  lox_2.waitRangeComplete();

    // Read results from both sensors
    uint16_t current_range_lox1 = lox_1.readRangeResult();
    uint16_t current_range_lox2 = lox_2.readRangeResult();

    // Constrain measurements to 0-200mm range
    current_range_lox1 = constrain(current_range_lox1, 0, 200);
    current_range_lox2 = constrain(current_range_lox2, 0, 200);

    // Update minimum values if current reading is smaller
    if (current_range_lox1 < min_range_lox1) {
      min_range_lox1 = current_range_lox1;
    }

    if (current_range_lox2 < min_range_lox2) {
      min_range_lox2 = current_range_lox2;
    }
  }

  // If lox1 reads maximum range, use lox2's reading instead
  if (min_range_lox1 == 200) {
    min_range_lox1 = min_range_lox2;
  }

  // Take one final measurement from both sensors
  lox_1.startRange();
  lox_1.waitRangeComplete();

  lox_2.startRange();
  lox_2.waitRangeComplete();

  // Read and constrain final measurements
  uint8_t range_lox1 = lox_1.readRangeResult();
  uint8_t range_lox2 = lox_2.readRangeResult();

  range_lox1 = constrain(range_lox1, 0, 200);
  range_lox2 = constrain(range_lox2, 0, 200);
  
  mux.closeAll();
  // Return the final measurements as a pair of integers
  return std::make_pair((int)range_lox1, (int)range_lox2);
}



void printSensorReads()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    // Check if the compartment state is active (1)
    if (compartmentState[i])
    {
      auto &muxH1 = ((compSensors[i].hSensor1.muxID == 0)|| compSensors[i].hSensor2.muxID == 0) ? I2CMux : I2CMux_1;
      Testing_Adafruit_VL6180X(compSensors[i].hSensor1.sensorName, compSensors[i].hSensor2.sensorName, compSensors[i].hSensor1.sensor, compSensors[i].hSensor2.sensor, muxH1, muxH1, compSensors[i].hSensor1.channel, compSensors[i].hSensor2.channel); //  Reading sample Up and Down Sensors for Pet Module
      auto &muxV = (compSensors[i].vSensor.muxID == 0) ? I2CMux : I2CMux_1;
      VAvrage(compSensors[i].vSensor.sensorName , compSensors[i].vSensor.sensor, muxV, compSensors[i].vSensor.channel); //  Reading sample Vertical Sensors for Additional Module
    }
  }
}

float printVertiaclSensorReads(float usage)
{
uint8_t i = 1;
float DivideRate = 0;
  // for (uint8_t i = 0; i < 3; ++i)
  // {
    // Check if the compartment state is active (1)
  //   if (EwasteCompartmentState[i])
  //   { 
  //     auto &muxV1 = (EwastecompSensors[i].vSensor1.muxID == 0) ? I2CMux : I2CMux_1;
  //     usage +=VAvrage(EwastecompSensors[i].vSensor1.sensorName , EwastecompSensors[i].vSensor1.sensor, muxV1, EwastecompSensors[i].vSensor1.channel); //  Reading sample Vertical Sensors for Additional Module
  //     auto &muxV2 = (EwastecompSensors[i].vSensor2.muxID == 0) ? I2CMux : I2CMux_1;
  //     usage+= VAvrage(EwastecompSensors[i].vSensor2.sensorName , EwastecompSensors[i].vSensor2.sensor, muxV2, EwastecompSensors[i].vSensor2.channel); //  Reading sample Vertical Sensors for Additional Module
  //     auto &muxV3 = (EwastecompSensors[i].vSensor3.muxID == 0) ? I2CMux : I2CMux_1;
  //     usage+= VAvrage(EwastecompSensors[i].vSensor3.sensorName , EwastecompSensors[i].vSensor3.sensor, muxV3, EwastecompSensors[i].vSensor3.channel); //  Reading sample Vertical Sensors for Additional Module
  //   }
  // // }
  // return usage/3.0;

// if (EwasteCompartmentState[i])
// {
if (SensorState[i][0])
{
  auto &muxV1 = (EwastecompSensors[i].vSensor1.muxID == 0) ? I2CMux : I2CMux_1;
  usage += VAvrage(EwastecompSensors[i].vSensor1.sensorName, EwastecompSensors[i].vSensor1.sensor, muxV1, EwastecompSensors[i].vSensor1.channel); //  Reading sample Vertical Sensors for Additional Module
  DivideRate++;
  mLogger.logMessage("printVertiaclSensorReads ---1" ,String(usage));
}
if (SensorState[i][1])
{
  auto &muxV2 = (EwastecompSensors[i].vSensor2.muxID == 0) ? I2CMux : I2CMux_1;
  usage += VAvrage(EwastecompSensors[i].vSensor2.sensorName, EwastecompSensors[i].vSensor2.sensor, muxV2, EwastecompSensors[i].vSensor2.channel); //  Reading sample Vertical Sensors for Additional Module
  DivideRate++;
  mLogger.logMessage("printVertiaclSensorReads ---2" ,String(usage));
}
if (SensorState[i][2])
{
  auto &muxV3 = (EwastecompSensors[i].vSensor3.muxID == 0) ? I2CMux : I2CMux_1;
  usage += VAvrage(EwastecompSensors[i].vSensor3.sensorName, EwastecompSensors[i].vSensor3.sensor, muxV3, EwastecompSensors[i].vSensor3.channel); //  Reading sample Vertical Sensors for Additional Module
  DivideRate++;
  mLogger.logMessage("printVertiaclSensorReads ---3" ,String(usage));
}
// }
// }
if(DivideRate<1.00) DivideRate = 1.00;
mLogger.logMessage("printVertiaclSensorReads " +String(usage), "Avrage distance: " + String(usage/DivideRate)+" , "+String(DivideRate));
float usageresult=0.00;
usageresult = usage /DivideRate;
return usageresult;
}

/**
 * @brief Detects objects dropping through the horizontal sensor array
 * @param sensor1 First TOF sensor (top)
 * @param sensor2 Second TOF sensor (middle) 
 * @param sensor3 Third TOF sensor (bottom)
 * @param mux The multiplexer object
 * @param ch1 Channel for sensor1
 * @param ch2 Channel for sensor2 
 * @param ch3 Channel for sensor3
 * @return true if drop detected, false otherwise
 */
bool detectDrop(Adafruit_VL53L0X &sensor1, Adafruit_VL53L0X &sensor2, Adafruit_VL53L0X &sensor3,
                TCA9548A &mux, uint8_t ch1, uint8_t ch2, uint8_t ch3)
{
 
  static const uint16_t THRESHOLD = 700; // Distance threshold in mm to detect object
  static const uint8_t SAMPLE_COUNT = 3; // Number of samples to confirm detection
  bool detectionState = false;           // Tracks progression of drop detection

  uint16_t rangeSensor1Sum = 0, rangeSensor2Sum = 0, rangeSensor3Sum;
  uint16_t minRangeSensor1 = UINT16_MAX, minRangeSensor2 = UINT16_MAX, minRangeSensor3 = UINT16_MAX;

  if (SensorState[0][0] == 1)
  {
    mux.openChannel(ch1);
    VL53L0X_RangingMeasurementData_t measure1;
    sensor1.rangingTest(&measure1, false);
    uint16_t dist1 = measure1.RangeMilliMeter;
    mux.closeChannel(ch1);
    dist1 = constrain(dist1, 0, 760);
    if (dist1 < minRangeSensor1)
      minRangeSensor1 = dist1;
    // vTaskDelay(5/portTICK_PERIOD_MS);
  }
  if (SensorState[0][1] == 1)
  {
    mux.openChannel(ch2);
    VL53L0X_RangingMeasurementData_t measure2;
    sensor2.rangingTest(&measure2, false);
    uint16_t dist2 = measure2.RangeMilliMeter;
    mux.closeChannel(ch2);
    dist2 = constrain(dist2, 0, 760);
    if (dist2 < minRangeSensor2)
      minRangeSensor2 = dist2;
    // vTaskDelay(5/portTICK_PERIOD_MS);
  }
  if (SensorState[0][2] == 1)
  {
    mux.openChannel(ch3);
    VL53L0X_RangingMeasurementData_t measure3;
    sensor3.rangingTest(&measure3, false);
    uint16_t dist3 = measure3.RangeMilliMeter;
    mux.closeChannel(ch3);
    dist3 = constrain(dist3, 0, 760);
    if (dist3 < minRangeSensor3)
      minRangeSensor3 = dist3;
    // vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  mLogger.logMessage("detectDrop", "minRangeSensor1: " + String(minRangeSensor1));
  mLogger.logMessage("detectDrop", "minRangeSensor2: " + String(minRangeSensor2));
  mLogger.logMessage("detectDrop", "minRangeSensor3: " + String(minRangeSensor3));
  int count = 0;
  // unsigned long timeout_counter2 = millis();
  while (true)
  {
    uint16_t reading1 = 0;
    if (SensorState[0][0] == 1)
    {
      mux.openChannel(ch1);
      VL53L0X_RangingMeasurementData_t measure1;
      sensor1.rangingTest(&measure1, false);
      reading1 = measure1.RangeMilliMeter;
      mux.closeChannel(ch1);
      reading1 = constrain(reading1, 0, 760);
      if ((reading1 < (minRangeSensor1 - 110)))
      {
        return true;
      }
      // vTaskDelay(5/portTICK_PERIOD_MS);
    }
    uint16_t reading2 = 0;
    if (SensorState[0][1] == 1)
    {
      mux.openChannel(ch2);
      VL53L0X_RangingMeasurementData_t measure2;
      sensor2.rangingTest(&measure2, false);
      reading2 = measure2.RangeMilliMeter;
      mux.closeChannel(ch2);
      reading2 = constrain(reading2, 0, 760);
      // vTaskDelay(5/portTICK_PERIOD_MS);
      if ((reading2 < (minRangeSensor2 - 110)))
      {
        return true;
      }
    }
    uint16_t reading3 = 0;
    if (SensorState[0][2] == 1)
    {
      mux.openChannel(ch3);
      VL53L0X_RangingMeasurementData_t measure3;
      sensor3.rangingTest(&measure3, false);
      reading3 = measure3.RangeMilliMeter;
      mux.closeChannel(ch3);
      reading3 = constrain(reading3, 0, 760);
      vTaskDelay(5 / portTICK_PERIOD_MS);
      if ((reading3 < (minRangeSensor3 - 110)))
      {
        return true;
      }
    }
    mLogger.logInfo("detect drop", String(reading1) + "   " + String(reading2) + "    " + String(reading3) + " : " + String(detectionState));
    count++;
    mLogger.logMessage("detectDrop", "count: " + String(count));
    if (count > 360)
    {
      return false;
    }
  }
}


// /**
//  * @brief Example usage in main loop
//  */
void handleDropDetection(uint8_t i) {
  // Assuming sensors are properly initialized and connected to mux
  float temUsage = 0.0;
  uploadBarcodeScanned(IoTEAN, timestamp, 11.11);

  auto &mux = (EwastecompSensors[i].vSensor2.muxID == 0) ? I2CMux : I2CMux_1;
  if (detectDrop(
          EwastecompSensors[i].vSensor1.sensor,
          EwastecompSensors[i].vSensor2.sensor,
          EwastecompSensors[i].vSensor3.sensor, mux,
          EwastecompSensors[i].vSensor1.channel,
          EwastecompSensors[i].vSensor2.channel,
          EwastecompSensors[i].vSensor3.channel))
  {
    mLogger.logMessage("handleDropDetection", "Drop detected");
    vTaskDelay(5000/portTICK_PERIOD_MS);
    temUsage = getUsage(temUsage); // printVertiaclSensorReads(temUsage);
    uploadBottleInfo(IoTEAN, timestamp, temUsage);
    ewasteCountBig++;
    auto &muxV = compSensors[i].vSensor.muxID == 0 ? I2CMux : I2CMux_1;
    isBinFull[1] = checkBinFull(temUsage, 1);
    mLogger.logMessage("handleDropDetection", "Check Bin Full (Large) " + String(isBinFull[1]));
    prefs.begin("configs", false);
    prefs.putBool("ewasteBig", ewasteCountBig);
    prefs.end();
  }
  else
  {
    mLogger.logMessage("handleDropDetection", "Drop not detected");
  }
}

