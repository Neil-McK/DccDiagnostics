/*
 * HttpManager class definition - include file.
 * 
 * Encapsulates the WiFI and Web Server functionality of the DCC inspector program.
 * 
 */

#ifndef httpmanager_h
#define httpmanager_h

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>

class HttpManagerClass {
  public:
    // Function to initialise the object, connect to WiFi and start HTTP server.
    void begin(const char *ssid, const char *password, const char *dnsName);
    // Function to provide a buffer of dynamic data to the server.
    void setBuffer(char *buffer);
    // Function to be called in loop() function to keep things going.
    void process();

  private:  
    // Flag whether connected or not.
    bool connected = false; 
} /* class HttpManagerClass */;

// Singleton class instance.
extern HttpManagerClass HttpManager;

#endif
