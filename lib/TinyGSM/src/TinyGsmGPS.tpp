/**
 * @file       TinyGsmGPS.tpp
 * @author     Volodymyr Shymanskyy
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
 * @date       Nov 2016
 */

#ifndef SRC_TINYGSMGPS_H_
#define SRC_TINYGSMGPS_H_

#include "TinyGsmCommon.h"

#define TINY_GSM_MODEM_HAS_GPS

#define GSM_MODEM_AUX_POWER     (127)
template <class modemType>
class TinyGsmGPS {
 public:
  /*
   * GPS/GNSS/GLONASS location functions
   */ 
  //power_en_pin: modem gpio number
  bool enableGPS(int8_t power_en_pin = -1,uint8_t enable_level = 1) {
    return thisModem().enableGPSImpl(power_en_pin,enable_level);
  }
  bool disableGPS(int8_t power_en_pin =-1,uint8_t disbale_level = 0) {
    return thisModem().disableGPSImpl(power_en_pin , disbale_level);
  }
  bool isEnableGPS() {
    return thisModem().isEnableGPSImpl();
  }
  String getGPSraw() {
    return thisModem().getGPSrawImpl();
  }
  bool enableAGPS(){
    return thisModem().enableAGPSImpl();
  }
  bool getGPS(uint8_t *status,float* lat, float* lon, float* speed = 0, float* alt = 0,
              int* vsat = 0, int* usat = 0, float* accuracy = 0, int* year = 0,
              int* month = 0, int* day = 0, int* hour = 0, int* minute = 0,
              int* second = 0) {
    return thisModem().getGPSImpl(status,lat, lon, speed, alt, vsat, usat, accuracy,
                                  year, month, day, hour, minute, second);
  }
  bool getGPSTime(int* year, int* month, int* day, int* hour, int* minute,
                  int* second) {
    float lat = 0;
    float lon = 0;
    uint8_t status;
    return thisModem().getGPSImpl(&status,&lat, &lon, 0, 0, 0, 0, 0, year, month, day,
                                  hour, minute, second);
  }

  bool setGPSBaud(uint32_t baud){
    return thisModem().setGPSBaudImpl(baud);
  }

  bool setGPSMode(uint8_t mode){
    return thisModem().setGPSModeImpl(mode);
  }

  bool setGPSOutputRate(uint8_t rate_hz){
    return thisModem().setGPSOutputRateImpl(rate_hz);
  }

  bool enableNMEA(){
    return thisModem().enableNMEAImpl();
  }

  bool disableNMEA(){
    return thisModem().disableNMEAImpl();
  }

  bool configNMEASentence(bool CGA,bool GLL,bool GSA,bool GSV,bool RMC,bool VTG = 0,bool ZDA = 0,bool ANT = 0){
    return thisModem().configNMEASentenceImpl( CGA, GLL, GSA, GSV, RMC, VTG, ZDA, ANT);
  }
  /*
   * CRTP Helper
   */
 protected:
  inline const modemType& thisModem() const {
    return static_cast<const modemType&>(*this);
  }
  inline modemType& thisModem() {
    return static_cast<modemType&>(*this);
  }
 protected:
  bool enableGPSImpl(int8_t power_en_pin ,uint8_t enable_level) {
    if(power_en_pin!= -1){
      sendAT("+CGDRT=",power_en_pin,",1");
      waitResponse();
      sendAT("+CGSETV=",power_en_pin,",",enable_level);
      waitResponse();
    }
    sendAT(GF("+CGNSSPWR=1"));  
    if (waitResponse(10000UL, GF("+CGNSSPWR: READY!")) != 1) { return false; }
    return true;
  }

  bool disableGPSImpl(int8_t power_en_pin ,uint8_t disbale_level) {
    if(power_en_pin!= -1){
      sendAT("+CGSETV=",power_en_pin,",",disbale_level);
      waitResponse();
      sendAT("+CGDRT=",power_en_pin,",0");
      waitResponse();
    }
    sendAT(GF("+CGNSSPWR=0"));  
    if (waitResponse() != 1) { return false; }
    return true;
  }

  bool isEnableGPSImpl(){
    sendAT(GF("+CGNSSPWR?"));
    if (waitResponse("+CGNSSPWR:") != 1) { return false; }
    // +CGNSSPWR:<GNSS_Power_status>,<AP_Flash_status>,<GNSS_dynamic_load>
    return 1 == streamGetIntBefore(','); 
  }

  bool enableAGPSImpl(){
    sendAT(GF("+CGNSSPWR?"));
    if (waitResponse("+CGNSSPWR:") != 1) { return false; }
    // +CGNSSPWR:<GNSS_Power_status>,<AP_Flash_status>,<GNSS_dynamic_load>
    if(1 == streamGetIntBefore(',')){
      sendAT("+CAGPS");
      if (waitResponse(30000UL,"+AGPS:") != 1) { return false; }
      String res = stream.readStringUntil('\n');
      if(res.startsWith(" success.")){
        return true;
      }
    }
    return false;
  }

  // get the RAW GPS output
  String getGPSrawImpl() {
    sendAT(GF("+CGNSSINFO"));
    if (waitResponse(GF(GSM_NL "+CGNSSINFO:")) != 1) { return ""; }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  // get GPS informations
  bool getGPSImpl(uint8_t *status,float* lat, float* lon, float* speed = 0, float* alt = 0,
                  int* vsat = 0, int* usat = 0, float* accuracy = 0,
                  int* year = 0, int* month = 0, int* day = 0, int* hour = 0,
                  int* minute = 0, int* second = 0) {
    sendAT(GF("+CGNSSINFO"));
    if (waitResponse(GF(GSM_NL "+CGNSSINFO:")) != 1) { return false; }

    uint8_t fixMode = streamGetIntBefore(',');  // mode 2=2D Fix or 3=3DFix
                                                // TODO(?) Can 1 be returned
    if (fixMode == 1 || fixMode == 2 || fixMode == 3) {
      // init variables
      float ilat = 0;
      char  north;
      float ilon = 0;
      char  east;
      float ispeed       = 0;
      float ialt         = 0;
      int   ivsat        = 0;
      int   iusat        = 0;
      float iaccuracy    = 0;
      int   iyear        = 0;
      int   imonth       = 0;
      int   iday         = 0;
      int   ihour        = 0;
      int   imin         = 0;
      float secondWithSS = 0;

      streamSkipUntil(',');               // GPS satellite valid numbers
      streamSkipUntil(',');               // GLONASS satellite valid numbers
      streamSkipUntil(',');               // skip dump , A7670
      streamSkipUntil(',');               // BEIDOU satellite valid numbers
      ilat  = streamGetFloatBefore(',');  // Latitude in ddmm.mmmmmm
      north =  stream.read();              // N/S Indicator, N=north or S=south
      streamSkipUntil(',');
      ilon = streamGetFloatBefore(',');  // Longitude in ddmm.mmmmmm
      east =  stream.read();              // E/W Indicator, E=east or W=west
      streamSkipUntil(',');

      // Date. Output format is ddmmyy
      iday   = streamGetIntLength(2);    // Two digit day
      imonth = streamGetIntLength(2);    // Two digit month
      iyear  = streamGetIntBefore(',');  // Two digit year

      // UTC Time. Output format is hhmmss.s
      ihour = streamGetIntLength(2);  // Two digit hour
      imin  = streamGetIntLength(2);  // Two digit minute
      secondWithSS =
      streamGetFloatBefore(',');  // 4 digit second with subseconds

      ialt   = streamGetFloatBefore(',');  // MSL Altitude. Unit is meters
      ispeed = streamGetFloatBefore(',');  // Speed Over Ground. Unit is knots.
      streamSkipUntil(',');                // Course Over Ground. Degrees.
      streamSkipUntil(',');  // After set, will report GPS every x seconds
      iaccuracy = streamGetFloatBefore(',');  // Position Dilution Of Precision
      streamSkipUntil(',');   // Horizontal Dilution Of Precision
      streamSkipUntil(',');   // Vertical Dilution Of Precision
      streamSkipUntil('\n');  // TODO(?) is one more field reported??
      if (status){
          *status = fixMode;
      }
      // Set pointers
      if (lat != NULL){
          *lat = (ilat) * (north == 'N' ? 1 : -1);
      }
      if (lon != NULL){
          *lon = (ilon) * (east == 'E' ? 1 : -1);
      }
      if (speed != NULL) *speed = ispeed;
      if (alt != NULL) *alt = ialt;
      if (vsat != NULL) *vsat = ivsat;
      if (usat != NULL) *usat = iusat;
      if (accuracy != NULL) *accuracy = iaccuracy;
      if (iyear < 2000) iyear += 2000;
      if (year != NULL) *year = iyear;
      if (month != NULL) *month = imonth;
      if (day != NULL) *day = iday;
      if (hour != NULL) *hour = ihour;
      if (minute != NULL) *minute = imin;
      if (second != NULL) *second = static_cast<int>(secondWithSS);

      waitResponse();
      return true;
    }
    waitResponse();
    return false;
  }

  bool setGPSBaudImpl(uint32_t baud){
    sendAT("+CGNSSIPR=",baud);
    return waitResponse(1000L) == 1;
  }

  bool setGPSModeImpl(uint8_t mode){
      sendAT("+CGNSSMODE=",mode);
      return waitResponse(1000L) == 1;
  }

  bool setGPSOutputRateImpl(uint8_t rate_hz){
      sendAT("+CGPSNMEARATE=",rate_hz);
      return waitResponse(1000L) == 1;
  }

  bool enableNMEAImpl(){
      sendAT("+CGNSSTST=1");
      waitResponse(1000L);
    // Select the output port for NMEA sentence
      sendAT("+CGNSSPORTSWITCH=0,1");
      return waitResponse(1000L) == 1;
  }

  bool disableNMEAImpl(){
      sendAT("+CGNSSTST=0");
      waitResponse(1000L);
    // Select the output port for NMEA sentence
      sendAT("+CGNSSPORTSWITCH=1,0");
      return waitResponse(1000L) == 1;
  }

  bool configNMEASentenceImpl(bool CGA,bool GLL,bool GSA,bool GSV,bool RMC,bool VTG,bool ZDA,bool ANT){
      char buffer[32];
      snprintf(buffer,32,"%u,%u,%u,%u,%u,%u,%u,0", CGA, GLL, GSA, GSV, RMC, VTG, ZDA);
      sendAT("+CGNSSNMEA=",buffer);
      return waitResponse(1000L) == 1;
  }


  /*
  * GPS/GNSS/GLONASS location functions
  */

  // bool    enableGPSImpl(int8_t power_en_pin ,uint8_t enable_level) TINY_GSM_ATTR_NOT_IMPLEMENTED;
  // bool    disableGPSImpl(int8_t power_en_pin ,uint8_t disbale_level) TINY_GSM_ATTR_NOT_IMPLEMENTED;
  // bool    isEnableGPSImpl() TINY_GSM_ATTR_NOT_IMPLEMENTED;
  // bool    enableAGPSImpl() TINY_GSM_ATTR_NOT_IMPLEMENTED;
  // String  getGPSrawImpl() TINY_GSM_ATTR_NOT_IMPLEMENTED;
  // bool    getGPSImpl(uint8_t *status,float* lat, float* lon, float* speed = 0, float* alt = 0,
  //                    int* vsat = 0, int* usat = 0, float* accuracy = 0,
  //                    int* year = 0, int* month = 0, int* day = 0, int* hour = 0,
  //                    int* minute = 0,
  //                    int* second = 0) TINY_GSM_ATTR_NOT_IMPLEMENTED;
  // bool    setGPSBaudImpl(uint32_t baud)TINY_GSM_ATTR_NOT_IMPLEMENTED;
  // bool    setGPSModeImpl(uint8_t mode)TINY_GSM_ATTR_NOT_IMPLEMENTED;
  // bool    setGPSOutputRateImpl(uint8_t rate_hz)TINY_GSM_ATTR_NOT_IMPLEMENTED;
  // bool    enableNMEAImpl()TINY_GSM_ATTR_NOT_IMPLEMENTED;
  // bool    disableNMEAImpl()TINY_GSM_ATTR_NOT_IMPLEMENTED;
  // bool    configNMEASentenceImpl()TINY_GSM_ATTR_NOT_IMPLEMENTED;
};


#endif  // SRC_TINYGSMGPS_H_
