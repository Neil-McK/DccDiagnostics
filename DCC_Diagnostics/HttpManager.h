#ifndef httpmanager_h
#define httpmanager_h

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>

class HttpManagerClass {
  public:
    void begin(const char *ssid, const char *password, const char *dnsName);
    void setBuffer(char *buffer);
    void process();

  private:  
    bool connected = false; 
};

// Singleton Object
extern HttpManagerClass HttpManager;

#endif
