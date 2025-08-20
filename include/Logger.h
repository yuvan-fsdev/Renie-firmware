class Logger {
public:
  void logMessage(String function, String message) {
    #if DEBUG_LOG
    Serial.println("{" + function + "}{\"message\":\"" + message + "\"}");
    #endif
  }

  void logWarning(String function, String warning) {
    #if DEBUG_LOG
    Serial.println("{" + function + "}{\"warning\":\"" + warning + "\"}");
    #endif
  }

  void logInfo(String info, String value) {
    #if DEBUG_LOG
    Serial.println("{\"" + info + "\":\"" + value + "\"}");
    #endif
  }

  void logError(String function, String error) {
    #if DEBUG_LOG
    Serial.println("{" + function + "}{\"error\":\"" + error + "\"}");
    #endif
  }
};
