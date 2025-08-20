#include "esp_wifi.h"
#include "esp_event.h"
// #include "files.h"
#include "esp_log.h"
// #include "awsiotcore.h"
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Ping.h>

DNSServer dnsServer;
AsyncWebServer server(80);
WiFiClient mywifiClient;
// Event group to handle WiFi connection events

extern void initCompartments();

bool testInternetConnection()
{
  IPAddress Testip(8, 8, 8, 8); // Google DNS
  if (Ping.ping(Testip))
  {
    return true;
  }
  return false;
}
/**
 * @brief Checks if the ESP32 is connected to a Wi-Fi access point (AP).
 *
 * This function queries the ESP32's Wi-Fi connection status by attempting to retrieve
 * information about the currently connected AP. If the AP information is successfully retrieved,
 * it indicates that the ESP32 is connected to Wi-Fi.
 *
 * @returns {bool} Returns `true` if the ESP32 is connected to a Wi-Fi AP,
 *                 and `false` if it is not connected.
 *
 * @note The function uses `esp_wifi_sta_get_ap_info()` to check the connection status.
 *       It yields the task to ensure proper operation when used in a multitasking environment.
 */
bool isWiFiConnected()
{
  wifi_ap_record_t ap_info;
  // Check if the ESP32 is connected to an AP
  yield();
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
  {
    // If AP info is successfully retrieved, the ESP32 is connected
    return true;
  }
  return false;
}
String compartmentOptions = "\
<option value='pet'>pet</option>\
<option value='can'>can</option>\
<option value='paper'>paper</option>\
<option value='vape'>vape</option>\
<option value='battery'>battery</option>\
<option value='pet:::can'>pet:::can</option>\
<option value='tpk:::paper'>tpk:::paper</option>\
<option value='tpk'>tpk</option>\
<option value='ewaste'>ewaste</option>\
<option value='glass'>glass</option>";


String countryOptions = "\
<option value='ae'>ae</option>\
<option value='ksa'>ksa</option>\
<option value='qa'>qa</option>";

String wifiHtml() {
    String html = "<!DOCTYPE html><html lang='en'>\
    <head>\
      <meta charset='UTF-8'>\
      <meta name='viewport' content='width=device-width, initial-scale=1.0'>\
      <title>WiFi Connection</title>\
      <style>\
        body { background-color: white; padding: 1rem; font-family: Arial, sans-serif; }\
        .container { max-width: 320px; margin: 0 auto; }\
        input { width: 100%; padding: 0.5rem; border-radius: 10px; border: 1px solid #ccc; margin-bottom: 1rem; }\
        button { width: 100%; padding: 1rem; border-radius: 10px; background: linear-gradient(to right, #4765E6, #5C4099, #E31662); color: white; font-weight: bold; border: none; }\
      </style>\
    </head>\
    <body>\
      <div class='container'>\
        <form action='/connect' method='POST'>\
          <label for='wifi-name'>Wi-Fi name</label>\
          <input type='text' id='wifi-name' name='ssid' placeholder='Enter WiFi name' required>\
          <label for='wifi-password'>Wi-Fi password</label>\
          <input type='text' id='wifi-password' name='password' placeholder='Enter WiFi password' required>\
          <button type='submit'>Connect</button>\
        </form>\
      </div>\
    </body></html>";

    return html;
}
/**
 * @brief Generates an HTML page for Wi-Fi connection.
 *
 * This function generates a basic HTML page with a form that allows the user to
 * input their Wi-Fi network name (SSID) and password. The form submits the input
 * data to a specified endpoint (`/connect`) using a POST request.
 * The page is styled with CSS for a simple and responsive design.
 *
 * @returns {String} Returns a string containing the generated HTML page.
 *
 * @note The HTML form includes two input fields for the Wi-Fi SSID and password,
 *       along with a submit button. The form submits the data to the `/connect`
 *       endpoint for further processing.
 */
String compHtml(uint8_t compartmentState[], const String &compartmentOptions, const String &countryOptions)
{
  String html = "<!DOCTYPE html><html lang='en'>\
    <head>\
      <meta charset='UTF-8'>\
      <meta name='viewport' content='width=device-width, initial-scale=1.0'>\
      <title>Device Configuration</title>\
      <style>\
        body { background-color: white; padding: 1rem; font-family: Arial, sans-serif; }\
        .container { max-width: 320px; margin: 0 auto; }\
        select, button { width: 100%; padding: 0.5rem; border-radius: 10px; border: 1px solid #ccc; margin-bottom: 1rem; }\
        button { background: linear-gradient(to right, #4765E6, #5C4099, #E31662); color: white; font-weight: bold; border: none; }\
      </style>\
    </head>\
    <body>\
      <div class='container'>\
        <form action='/connect' method='POST'>";

  // Loop to create compartments
  for (uint8_t i = 0; i < 4; i++)
  {
    if (compartmentState[i]) 
    html += "<label for='compartment-" + String(i+1) + "'>Compartment " + String(i+1) + "</label>\
          <select id='compartment-" + String(i+1) + "' name='compartment" + String(i+1) + "' required>\n" +
            compartmentOptions + "</select>\n";
  }

  // Add the country selection list
  html += "<label for='devCountry'>Device Country</label>\
          <select id='devCountry' name='Country' required>\n" +
          countryOptions + "</select>\
          <button type='submit'>Save Configuration</button>\
        </form>";
    
  for (uint8_t i = 1; i <= 4; i++){
    html+="<p>"+ CompartmentIssues[i]+ "</p>";
  }
  
  html+="</div>\
    </body></html>";

  return html;
}


/**
 * @brief Sets up a DNS server to respond to all DNS queries with a specified IP address.
 *
 * This function configures the DNS server to return a custom IP address for any requested domain.
 * It sets the Time-To-Live (TTL) for DNS responses to 3600 seconds and starts the DNS server
 * on port 53. The DNS server responds to all queries with the provided `localIP`.
 *
 * @param {DNSServer} dnsServer A reference to the `DNSServer` object that will handle DNS queries.
 * @param {IPAddress} localIP The IP address that will be returned for all DNS queries.
 *
 * @returns {void} This function does not return any value.
 *
 * @note The DNS server processes requests at a fixed interval defined by `DNS_INTERVAL` (30 ms).
 */
void setUpDNSServer(DNSServer &dnsServer, const IPAddress &localIP)
{
// Define the DNS interval in milliseconds between processing DNS requests
#define DNS_INTERVAL 30

  // Set the TTL for DNS response and start the DNS server
  dnsServer.setTTL(3600);
  dnsServer.start(53, "*", localIP);
}

/**
 * @brief Configures and starts the ESP32 as a Wi-Fi Soft Access Point (AP).
 *
 * This function sets up the ESP32 as a Wi-Fi access point with a given SSID, password,
 * and IP configuration. It also defines the maximum number of clients allowed to connect
 * to the access point, the Wi-Fi channel, and the subnet mask. The AP will use the specified
 * local IP and gateway IP. Additionally, the function disables AMPDU RX to fix a compatibility
 * issue with Android devices.
 *
 * @param {const char*} ssid The SSID (network name) for the access point.
 * @param {const char*} password The password for the access point (optional; can be NULL for open networks).
 * @param {const IPAddress&} localIP The local IP address to be assigned to the access point.
 * @param {const IPAddress&} gatewayIP The gateway IP address for the access point's network.
 *
 * @returns {void} This function does not return any value.
 *
 * @note The Wi-Fi access point is configured with a subnet mask of `255.255.255.0` and operates on channel 6.
 *       The maximum number of clients that can connect to the AP is defined as `MAX_CLIENTS` (4 clients).
 */
void startSoftAccessPoint(const char *ssid, const char *password, const IPAddress &localIP, const IPAddress &gatewayIP)
{
  // Define the maximum number of clients that can connect to the server
#define MAX_CLIENTS 4
// Define the WiFi channel to be used (channel 6 in this case)
#define WIFI_CHANNEL 6
  WiFi.mode(WIFI_OFF);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  // Set the WiFi mode to access point and station
  WiFi.mode(WIFI_MODE_APSTA);
  // Define the subnet mask for the WiFi network
  const IPAddress subnetMask(255, 255, 255, 0);

  // Configure the soft access point with a specific IP and subnet mask
  WiFi.softAPConfig(localIP, gatewayIP, subnetMask);

  // Start the soft access point with the given ssid, password, channel, max number of clients
  WiFi.softAP(ssid, password, WIFI_CHANNEL, 0, MAX_CLIENTS);

  // Disable AMPDU RX on the ESP32 WiFi to fix a bug on Android
  esp_wifi_stop();
  esp_wifi_deinit();
  wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
  my_config.ampdu_rx_enable = false;
  esp_wifi_init(&my_config);
  vTaskDelay(50 / portTICK_PERIOD_MS); // Add a small delay
  esp_wifi_start();
  vTaskDelay(100 / portTICK_PERIOD_MS); // Add a small delay
}

/**
 * @brief Configures the webserver to handle various requests, including captive portal redirects
 *        and serving an HTML page for Wi-Fi credentials input.
 *
 * This function sets up a web server using the `AsyncWebServer` library. It handles specific routes
 * that are commonly used by devices to detect if they are connected to the internet, such as
 * `/generate_204`, `/connecttest.txt`, and other common captive portal detection requests. The
 * server also serves a basic HTML page for Wi-Fi configuration and handles the POST request to
 * store the Wi-Fi credentials.
 *
 * @param {AsyncWebServer&} server The web server instance to configure and handle requests.
 * @param {const IPAddress&} localIP The local IP address to use for captive portal redirects.
 *
 * @returns {void} This function does not return any value.
 *
 * @note The function sets up routes to handle captive portal requests from various operating systems
 *       (Windows, Android, iOS, Firefox) and serves a basic HTML page to enter Wi-Fi credentials.
 *       After the credentials are submitted, the device attempts to connect to the specified Wi-Fi.
 */
void setUpWebserver(AsyncWebServer &server, const IPAddress &localIP)
{
  //======================== Webserver ========================
  // WARNING IOS (and maybe macos) WILL NOT POP UP IF IT CONTAINS THE WORD "Success" https://www.esp8266.com/viewtopic.php?f=34&t=4398
  // SAFARI (IOS) IS STUPID, G-ZIPPED FILES CAN'T END IN .GZ https://github.com/homieiot/homie-esp8266/issues/476 this is fixed by the webserver serve static function.
  // SAFARI (IOS) there is a 128KB limit to the size of the HTML. The HTML can reference external resources/images that bring the total over 128KB
  // SAFARI (IOS) popup browser has some severe limitations (javascript disabled, cookies disabled)

  // Required
  server.on("/connecttest.txt", [](AsyncWebServerRequest *request)
            { request->redirect("http://logout.net"); }); // windows 11 captive portal workaround
  server.on("/wpad.dat", [](AsyncWebServerRequest *request)
            { request->send(404); }); // Honestly don't understand what this is but a 404 stops win 10 keep calling this repeatedly and panicking the esp32 :)

  // Background responses: Probably not all are Required, but some are. Others might speed things up?
  // A Tier (commonly used by modern systems)
  server.on("/generate_204", [](AsyncWebServerRequest *request)
            { request->redirect(localIPURL); }); // android captive portal redirect
  server.on("/redirect", [](AsyncWebServerRequest *request)
            { request->redirect(localIPURL); }); // microsoft redirect
  server.on("/hotspot-detect.html", [](AsyncWebServerRequest *request)
            { request->redirect(localIPURL); }); // apple call home
  server.on("/canonical.html", [](AsyncWebServerRequest *request)
            { request->redirect(localIPURL); }); // firefox captive portal call home
  server.on("/success.txt", [](AsyncWebServerRequest *request)
            { request->send(200); }); // firefox captive portal call home
  server.on("/ncsi.txt", [](AsyncWebServerRequest *request)
            { request->redirect(localIPURL); }); // windows call home

  // B Tier (uncommon)
  //  server.on("/chrome-variations/seed",[](AsyncWebServerRequest *request){request->send(200);}); //chrome captive portal call home
  //  server.on("/service/update2/json",[](AsyncWebServerRequest *request){request->send(200);}); //firefox?
  //  server.on("/chat",[](AsyncWebServerRequest *request){request->send(404);}); //No stop asking Whatsapp, there is no internet connection
  //  server.on("/startpage",[](AsyncWebServerRequest *request){request->redirect(localIPURL);});

  // return 404 to webpage icon
  server.on("/favicon.ico", [](AsyncWebServerRequest *request)
            { request->send(404); }); // webpage icon

  // Serve Basic HTML Page
  server.on("/", HTTP_ANY, [](AsyncWebServerRequest *request){
    String htmlFile;
   
    
    if(CompCaptive){
      Serial.println("Serial comps " +String(compartmentState[0])+ "," +String(compartmentState[1])+ "," +String(compartmentState[2])+ "," + String(compartmentState[3]));
      htmlFile=compHtml(compartmentState,compartmentOptions,countryOptions);
      Serial.println("captive is opened for the compartments");
    }
    else htmlFile=wifiHtml();
    
		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", htmlFile);
		response->addHeader("Cache-Control", "public,max-age=31536000");  // save this file to cache for 1 year (unless you refresh)
		request->send(response);
		Serial.println("Served Basic HTML Page"); });

  
  server.on("/connect", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    String ssid,password, countryCode;
    // Get the SSID from the request
    if (request->hasParam("ssid", true)) {
        ssid = request->getParam("ssid", true)->value();
    }

    // Get the password from the request
    if (request->hasParam("password", true)) {
        password = request->getParam("password", true)->value();
    }
    if (request->hasParam("Country", true)) {
        countryCode = request->getParam("Country", true)->value();
    }
    // Get the compartment names from the request
    for (uint8_t i = 0; i < 4; i++) {
        String paramName = "compartment" + String(i + 1);
        if (request->hasParam(paramName, true)) {
            *compName[i] = request->getParam(paramName, true)->value();
        }
    }
    String response = "<!DOCTYPE html><html><body><h1>Connecting to...</h1>";
    String prefKey="";
    // Store SSID and password in preferences
      Serial.println("succseffully Stored WIFI Creds.");
      prefs.begin("configs", false); // begin preferences
      prefs.putString("ssid", ssid);
      prefs.putString("password", password);
      WifiSSID=ssid;
      WifiPass=password;
      wifiCreds=false;
      prefs.end();
      // Send a response to the client
      response += "<p>SSID: " + ssid + "</p>";
      response += "<p>Password: " + password + "</p>";
      response += "<p>The page will close shortly..., if didnt close you can close it</p>";
      response += "<script>setTimeout(function() { window.location.href = '/close'; }, 5000);</script>"; // Redirect after 2 seconds
      response += "</body></html>";
      request->send(200, "text/html", response);
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      ESP.restart();

     });

  // the catch all
  server.onNotFound([](AsyncWebServerRequest *request)
                    {
		request->redirect(localIPURL);
		Serial.print("onnotfound ");
		Serial.print(request->host());	// This gives some insight into whatever was being requested on the serial monitor
		Serial.print(" ");
		Serial.print(request->url());
		Serial.print(" sent redirect to " + localIPURL + "\n"); });

  server.begin();
}

/**
 * @brief Configures the ESP32 to connect to a fallback Wi-Fi network.
 *
 * This function configures the Wi-Fi interface to connect to a predefined fallback Wi-Fi network
 * (specified by the `FALLBACKSSID1` and `FALLBACKPASS1` constants). It logs a message indicating
 * that the fallback Wi-Fi network is being used and sets the Wi-Fi configuration for the station
 * mode (STA). The device will then attempt to connect to the fallback Wi-Fi network.
 *
 * @returns {void} This function does not return any value.
 *
 * @note This function is intended to be used when the device fails to connect to the primary Wi-Fi
 *       network, ensuring a backup connection attempt to a fallback network.
 */
void fallbackWifi()
{
  mLogger.logMessage("fallbackWifi", "Using fallback wifi");
  wifi_config_t wifi_config = {};
  strcpy((char *)wifi_config.sta.ssid, FALLBACKSSID1.c_str());
  strcpy((char *)wifi_config.sta.password, FALLBACKPASS1.c_str());
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_connect();
}

void connect_to_wifi(void *pvParameters)
{
  WifiConnecting = true;
  mLogger.logMessage("connect_to_wifi", "Connecting to WiFi");
  // Initialize WiFi with station mode
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  mLogger.logMessage("connect_to_wifi", "wifi_init_config_t");
  esp_wifi_init(&cfg);
  mLogger.logMessage("connect_to_wifi", "esp_wifi_init(&cfg)");

  esp_wifi_set_mode(WIFI_STA);
  mLogger.logMessage("connect_to_wifi", "esp_wifi_set_mode");
  vTaskDelay(500 / portTICK_PERIOD_MS); // Allow some time for stopping
  if (esp_wifi_start() != ESP_OK)
  {
    mLogger.logError("connect_to_wifi", "Failed to start WiFi");
    WifiConnecting = false;
    vTaskDelete(NULL);
    return;
  }
  mLogger.logMessage("connect_to_wifi", "esp_wifi_start");
  // Start scanning for networks
  int16_t num_networks = WiFi.scanNetworks();
  mLogger.logMessage("connect_to_wifi", "scanNetworks " + String(num_networks));
  if (num_networks == -1)
  {
    mLogger.logError("connect_to_wifi", "WiFi scan failed");
    WifiConnecting = false;
    vTaskDelete(NULL);
    return;
  }
  mLogger.logMessage("connect_to_wifi", "num_networks");
  // Get stored WiFi credentials
  prefs.begin("configs", false);
  String myWifi = prefs.getString("ssid");
  String myPass = prefs.getString("password");
  prefs.end();

  //load and Decrypt the password
  char decryptedPassword[16]={};
  bool decryptionResult = decryptPassword(AES_KEY, myPass.c_str(), decryptedPassword);
    

  String target_ssid = "";
  String target_password = "";

  // Log available networks
  for (int i = 0; i < num_networks; i++)
  {
    String found_ssid = WiFi.SSID(i);
    mLogger.logMessage("connect_to_wifi", "Available SSID: " + found_ssid);
  }
  if (myWifi != ""){
    for (int i = 0; i < num_networks; i++)
    {
      String found_ssid = WiFi.SSID(i);
      
      //This section will only connect to 4G-UFI SSIDs
      // -------------------------
      if (found_ssid.startsWith("4G-UFI"))
      { // Check if SSID starts with stored SSID
        mLogger.logMessage("connect_to_wifi", " startsWith " + found_ssid);
        target_ssid = found_ssid;


        if (decryptionResult) target_password = String(decryptedPassword);
        else target_password = myPass;
        break;
      }
      //-------------------------


      //This section will connect to the saved SSID matching the stored SSID
      else if (found_ssid == myWifi){
        mLogger.logMessage("connect_to_wifi", " WiFi Found :  " + found_ssid);
        target_ssid = found_ssid;
      
        if (decryptionResult) target_password = String(decryptedPassword);
        else target_password = myPass;
        break;
      }

      


    }
  }
  mLogger.logMessage("connect_to_wifi", "target_ssid" + target_ssid);

  // If no saved SSID found, attempt fallback SSID
  if (target_ssid == "")
  {
    mLogger.logMessage("connect_to_wifi", "No matching SSID found, attempting fallback SSID...");
    for (int i = 0; i < num_networks; i++)
    {
      String found_ssid = WiFi.SSID(i);
      if (found_ssid.equals(FALLBACKSSID1.c_str()))
      {
        target_ssid = found_ssid;
        target_password = FALLBACKPASS1;
        mLogger.logMessage("connect_to_wifi", "fallback target_password " + target_password);
        break;
      }
    }
  }

  // If still no SSID, exit
  if (target_ssid == "")
  {
    mLogger.logError("connect_to_wifi", "Failed to find any matching SSID. Exiting.");
    WifiConnecting = false;
    vTaskDelete(NULL);
    return;
  }

  mLogger.logMessage("connect_to_wifi", "Attempting to connect to " + target_ssid);
  mLogger.logMessage("connect_to_wifi", "password " + target_password);

  // Set WiFi configuration and attempt to connect
  wifi_config_t wifi_config = {};
  strcpy((char *)wifi_config.sta.ssid, target_ssid.c_str());
  strcpy((char *)wifi_config.sta.password, target_password.c_str());
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  // Attempt to connect
  esp_wifi_connect();
  // Wait for connection to be established
  unsigned long start = millis();
  while ((WiFi.status() != WL_CONNECTED) && (millis() - start) < (WIFI_TIMEOUT * 1000))
  {
    vTaskDelay(200);
  }
  // Check if connection was successful
  if (WiFi.status() != WL_CONNECTED)
  {
    mLogger.logError("connect_to_wifi", "Connection failed, Exit");
  }
  else
  {
    mLogger.logMessage("connect_to_wifi", "Connected !! To : " + target_ssid);
    // ping in case its required
    // if (testInternetConnection()) mLogger.logMessage("testInternetConnection","Ping DOne!!");
    // else mLogger.logError("testInternetConnection","Ping Failed!!");
    internetConnected = true;
  }
  Serial.println("-------5");
  WifiConnecting = false;
  mLogger.logMessage("connect_to_wifi", "END");
  if (internetConnected && RegisterDone)downloadEANList();
  // initCompartments();
  Serial.println("-------6");
  vTaskDelete(NULL);
}

/**
 * @function restatrWifi
 * @description Restarts the Wi-Fi connection. If the number of Wi-Fi restart attempts exceeds 5, the ESP32 will be restarted.
 * Otherwise, it will attempt to restart the Wi-Fi connection by stopping the current connection and attempting to reconnect.
 *
 * @returns {void} This function does not return any value.
 */
void restatrWifi()
{
  if (restWifiAttempt > 5)
  {
    mLogger.logMessage("restartWifi", "Attempt >5 Restarting the ESP32 now");
    sleep(5);
    ESP.restart();
  }
  else
  {
    restWifiAttempt++;
    mLogger.logMessage("restartWifi", "Restarting wifi");
    esp_wifi_stop(); // Turn off Wi-Fi
                     // xTaskCreate(connect_to_wifi, "connect_to_wifi", 5000, NULL, 1, &Task1);
  }
}

/**
 * @function collectionComplete
 * @description Publishes a message to the IoT server indicating the completion of a collection.
 * If the timestamp is provided, it includes the timestamp in the message. It also checks for a PUBACK response to confirm the message was sent successfully.
 * If the PUBACK is not received, it stores the information offline.
 *
 * @param {String} uuid - The unique identifier for the collection.
 * @param {String} timestamp - The timestamp of the collection (can be empty).
 * @returns {boolean} Returns `true` if the collection completion message was successfully sent, `false` if it failed and the message was stored offline.
 */
bool collectionComplete(String uuid, String timestamp)
{
  String iotBody;
  if (timestamp.toInt() != 0)
    iotBody = "{\"chip_id\":\"" + Bin_ID + "\",\"type\":\"" + Material + "\",\"uuid\": \"" + uuid + "\",\"timestamp\":\"" + timestamp + "\"}";
  else
    iotBody = "{\"chip_id\":\"" + Bin_ID + "\",\"type\":\"" + Material + "\", \"uuid\": \"" + uuid + "\"}";
  mLogger.logMessage("IoT CollectionComplete", iotBody);
  IoTclient.publish(Collection_done, iotBody.c_str(), false, 1);
  if (IoTclient.lastPacketID() != PuBackCounter)
  {
    Serial.println("PUBACK received!");
    mLogger.logMessage("IoT collectionComplete", "Collection Complete message Sent");
    PuBackCounter = IoTclient.lastPacketID();
    return true;
  }
  else
  {
    mLogger.logWarning("IoT Collection Complete", "Failed to send collection complete message");
    // Store Ean
    // if (saveBottleInfo(uuid, timestamp)) mLogger.logMessage("IoT collectionComplete", "Offline, storing Collection Card Info");
    // else mLogger.logError("collectionComplete", "Error Storing Card");
    return false;
  }
}

/**
 * @function uploadBottleInfo
 * @description Publishes a message to the IoT server with information about a bottle, including the EAN (European Article Number), usage, and optionally a timestamp.
 * It waits for a PUBACK response to confirm the message was sent successfully. If the PUBACK is not received, it stores the information offline.
 *
 * @param {String} ean - The European Article Number (EAN) of the bottle.
 * @param {String} timestamp - The timestamp of the bottle usage (optional).
 * @param {float} usage - The current usage or amount of usage of the bottle.
 * @returns {boolean} Returns `true` if the bottle information was successfully uploaded, `false` if it failed and the message was stored offline.
 */
bool uploadBottleInfo(String ean, String timestamp, float usage)
{
  String iotBody;
  if (timestamp.toInt() == 0)
    iotBody = "{\"chip_id\":\"" + Bin_ID + "\",\"type\":\"" + Material + "\",\"ean\":\"" + ean + "\",\"currentUsage\":" + String(usage) + "}";
  else
    iotBody = "{\"chip_id\":\"" + Bin_ID + "\",\"type\":\"" + Material + "\",\"ean\":\"" + ean + "\",\"currentUsage\":" + String(usage) + ",\"timestamp\": \"" + timestamp + "\"}";
  mLogger.logMessage("IoT uploadBottleInfo", iotBody);
  IoTclient.publish(Add_Bottle, iotBody.c_str(), false, 1);
  Serial.println(PuBackCounter);
  if (IoTclient.lastPacketID() != PuBackCounter && IoTclient.lastPacketID() > 5)
  {
    Serial.println(IoTclient.lastPacketID());
    Serial.println("PUBACK received!");
    mLogger.logMessage("IoT uploadBottleInfo", "Add points successful...");
    PuBackCounter = IoTclient.lastPacketID();
    return true;
  }
  else
  {
    mLogger.logWarning("IoT uploadBottleInfo", " failed to add points ");
    // Store Ean
    // if (saveBottleInfo(ean, timestamp)) mLogger.logMessage("IoT uploadBottleInfo" , "Offline, storing bottleinfo");
    // else mLogger.logWarning("IoT uploadBottleInfo", "Storage Full");
  }
  return false;
}

// /**
//  * @function uploadFakedrop
//  * @description Publishes a message to the IoT server containing fake drop information, including the drop count, usage, and optionally a timestamp.
//  * It waits for a PUBACK response to confirm the message was sent successfully. If the PUBACK is not received, it returns `false`.
//  *
//  * @param {int} count - The number of fake drops.
//  * @param {String} timestamp - The timestamp of the fake drop event (optional).
//  * @param {float} usage - The current usage or amount associated with the fake drop.
//  * @returns {boolean} Returns `true` if the fake drop information was successfully uploaded, `false` if the message was not acknowledged.
//  */
// bool uploadFakedrop(int count, String timestamp, float usage) {
//   String iotBody;
//   if (timestamp.toInt() == 0) iotBody = "{\"chip_id\":\"" + Bin_ID + "\",\"type\":\"" + Material + "\",\"count\":\"" + String(count) + "\",\"currentUsage\":" + String(usage) + "}";
//   else iotBody = "{\"chip_id\":\"" + Bin_ID + "\",\"type\":\"" + Material + "\",\"count\":\"" + String(count) + "\",\"currentUsage\":" + String(usage) + ",\"timestamp\": \"" + timestamp + "\"}";
//   mLogger.logMessage("IoT uploadFakedrop", iotBody);
//   IoTclient.publish(FakedropTopic,iotBody.c_str(),false,1);
//   if (IoTclient.lastPacketID() != 0){
//     Serial.println("PUBACK received!");
//     mLogger.logMessage("IoT uploadFakedrop", "Add points successful...");
//     return true;
//   }
//   return false;
// }

/**
 * @function uploadFakedrop
 * @description Publishes a message to the IoT server containing fake drop information, including the drop count, usage, and optionally a timestamp.
 * It waits for a PUBACK response to confirm the message was sent successfully. If the PUBACK is not received, it returns `false`.
 *
 * @param {int} count - The number of fake drops.
 * @param {String} timestamp - The timestamp of the fake drop event (optional).
 * @param {float} usage - The current usage or amount associated with the fake drop.
 * @returns {boolean} Returns `true` if the fake drop information was successfully uploaded, `false` if the message was not acknowledged.
 */
bool uploadFakedrop(int count, String timestamp, float usage)
{
  StaticJsonDocument<200> doc;
  doc["chip_id"] = Bin_ID;
  doc["type"] = Material;
  doc["count"] = String(count);
  doc["currentUsage"] = String(usage);
  if (timestamp.toInt() != 0)
    doc["timestamp"] = timestamp;
  // Serialize JSON and publish to IoT client
  String iotBody;
  serializeJson(doc, iotBody);
  mLogger.logMessage("IoT uploadFakedrop", iotBody);
  IoTclient.publish(FakedropTopic, iotBody.c_str(), false, 1);

  if (IoTclient.lastPacketID() != 0)
  {
    Serial.println("PUBACK received!");
    mLogger.logMessage("IoT uploadFakedrop", "Add points successful...");
    return true;
  }
  return false;
}

/**
 * @function uploadBarcodeScanned
 * @description Publishes a message to the IoT server with barcode scan information, including the EAN (barcode) and optionally a timestamp.
 * The function waits for a PUBACK response to confirm the message was successfully uploaded. If the PUBACK is not received, it returns `false`.
 *
 * @param {String} ean - The EAN (barcode) value to be uploaded.
 * @param {String} timestamp - The timestamp of the barcode scan (optional).
 * @returns {boolean} Returns `true` if the barcode scan information was successfully uploaded, `false` if the message was not acknowledged.
 */
bool uploadBarcodeScanned(String ean, String timestamp, float usage = 0.0)
{
  String iotBody;
  if (timestamp.toInt() == 0)
    iotBody = "{\"chip_id\":\"" + Bin_ID + "\",\"usage\":\"" + usage + "\",\"type\":\"" + Material + "\",\"ean\":\"" + ean + "\"}";
  else
    iotBody = "{\"chip_id\":\"" + Bin_ID + "\",\"usage\":\"" + usage + "\",\"type\":\"" + Material + "\",\"ean\":\"" + ean + "\",\"timestamp\":\"" + timestamp + "\"}";
  mLogger.logMessage("IoT BarcodeScannedTopic", iotBody);
  IoTclient.publish(BarcodeScannedTopic, iotBody.c_str(), false, 1);
  if (IoTclient.lastPacketID() != 0)
  {
    Serial.println("PUBACK received!");
    mLogger.logMessage("IoT uploadBarcodeScanned", "Successfully Uploaded...");
    return true;
  }
  return false;
}

/**
 * Uploads an issue to the IoT system via MQTT
 * @param IssueType The type/category of the issue being reported
 * @param description Details about the issue
 * @param timestamp Optional timestamp of when the issue occurred (pass "0" to omit)
 * @return true if publish was acknowledged, false otherwise
 *
 * Creates a JSON payload with device and issue information and publishes it to
 * the CreateIssueTopic MQTT topic with QoS 1. The payload includes chip_id,
 * material type, issue type, description and optional timestamp.
 */
bool uploadCreateIssue(String IssueType, String description, String timestamp)
{

  StaticJsonDocument<200> doc;
  doc["chip_id"] = chipId;
  doc["type"] = Material;
  doc["IssueType"] = IssueType;
  doc["description"] = description;
  if (timestamp.toInt() != 0)
    doc["timestamp"] = timestamp;
  // Serialize JSON and publish to IoT client
  String iotBody;
  serializeJson(doc, iotBody);
  mLogger.logMessage("CreateIssueTopic", iotBody);
  IoTclient.publish(CreateIssueTopic, iotBody.c_str(), false, 1);
  if (IoTclient.lastPacketID() != 0)
  {
    Serial.println("PUBACK received!");
    mLogger.logMessage("CreateIssueTopic", "Add issue successfully");
    return true;
  }
  return false;
}
/**
 * Sends sensor feedback data to IoT client
 * @param {int} UpperRange - Upper range sensor value
 * @param {int} LowerRange - Lower range sensor value
 * @param {String} timestamp - Timestamp of the reading
 */
void SendSensorFB(int UpperRange, int LowerRange, String timestamp)
{
  // Create JSON document with 200 bytes capacity
  StaticJsonDocument<200> doc;
  doc["chip_id"] = chipId;
  doc["sensor_up"] = UpperRange;
  doc["sensor_down"] = LowerRange;
  if (timestamp.toInt() != 0)
    doc["timestamp"] = timestamp;
  // Serialize JSON and publish to IoT client
  String iotBody;
  serializeJson(doc, iotBody);
  IoTclient.publish(SensorFBTopic, iotBody.c_str(), false, 1);
  mLogger.logMessage("Sensor Feedback", iotBody.c_str());

  // Check if publish was successful
  if (IoTclient.lastPacketID() != 0)
  {
    Serial.println("PUBACK received!");
    mLogger.logMessage("IoT uploadSensorFB", "Upload successful...");
  }
  else
  {
    mLogger.logWarning("IoT uploadSensorFB", " failed to Upload ");
  }
}

/**
 * Registers a bin by creating a JSON payload with required and optional details and publishes it to an MQTT topic.
 *
 * @returns {bool} - Returns `true` if the bin registration is successful and a PUBACK is received; otherwise, `false`.
 *
 * @description
 * The function prepares a JSON document with both required and optional fields for registering a bin. It publishes
 * the JSON payload to the MQTT topic defined by `RegisterandDeploy`. If a PUBACK is received, the registration is
 * considered successful.
 */
bool RegisterBin()
{
  DynamicJsonDocument jsonDoc(256);
  // Required fields
  jsonDoc["chip_id"] = Bin_ID;
  jsonDoc["ssid"] = WifiSSID;
  jsonDoc["password"] = WifiPass;
  jsonDoc["variant"] = BinType;
  jsonDoc["firmware"] = versionName;
  jsonDoc["sim_card"] = devICCID;

  // Add compartment array and mandatory Comp1, plus optional compartments
  JsonArray compartmentArray = jsonDoc.createNestedArray("compartment");
  for (uint8_t i = 0; i < 4; i++)
    if (*compName[i] != "false")
      compartmentArray.add(*compName[i]); // Comp1 is mandatory
  // Serialize JSON to String for MQTT payload
  String iotBody;
  serializeJson(jsonDoc, iotBody);
  if (IoTclient.publish(RegisterandDeploy, iotBody.c_str(), false, 1)) {
    mLogger.logMessage("IoT RegisterBin Topic", RegisterandDeploy);
    mLogger.logMessage("IoT RegisterBin Payload", iotBody);
  }
  mLogger.logMessage("IoT RegisterBin Topic", RegisterandDeploy);
  mLogger.logMessage("IoT RegisterBin Payload", iotBody);
  if (IoTclient.lastPacketID() != 0)
  {
    Serial.println("PUBACK received!");
    mLogger.logMessage("IoT RegisterBin", "Successfully Uploaded...");
    RegisterDone = true;
    prefs.begin("configs", false); // begin preferences
    prefs.putBool("RegisterStatus", RegisterDone);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    prefs.end();
    return true;
  }
  return false;
}