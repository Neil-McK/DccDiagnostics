/*
 * HttpManager class definition - methods and static data.
 * 
 * Encapsulates the WiFI and Web Server functionality of the DCC inspector program.
 * 
 */

#if defined(ESP32) || defined(ESP8266)

#include "HttpManager.h"

// Buffer pointer reference.  This is the buffer of dynamic text sent to the web server client.
static char *bufferPointer = 0;
// Web server object.
static WebServer server;

// Function to handle request for the root web page (http://server/).
//  It sends a static web page that includes an updating IFRAME where the dynamic data
//  will be displayed.
static void handleRoot() {
  digitalWrite(LED_BUILTIN, 1);
  String temp = 
    "<html>\r\n"
      "<script>\r\n"
        "window.setInterval('reloadIFrame();', 1000);\r\n"
        "function reloadIFrame() {\r\n"
          "document.getElementById('data').src='/data';\r\n"
        "}\r\n"
      "</script>\r\n"
      "<head>\r\n"
        "<title>DCC Diagnostics - Bit analysis</title>\r\n"
        "<style>\r\n"
          "body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\r\n"
        "</style>\r\n"
      "</head>\r\n"
      "<body>\r\n"
        "<h1>DCC Diagnostics</h1>\r\n"
        "<p>Diagnostic information from ESP32 server:</p>"
        "<p><iframe id=\"data\" src=\"/data\" height=\"400\" width=\"200\"></iframe></p>\r\n"
      "</body>\r\n"
    "</html>";
  server.send(200, "text/html", temp);
  digitalWrite(LED_BUILTIN, 0);
}

// Function to handle the request for dynamic data (http://server/data).
static void handleBitData() {
  digitalWrite(LED_BUILTIN, 1);
  if (bufferPointer) 
    server.send(200, "text/plain", bufferPointer);
  else
    server.send(200, "text/plain", "");
  digitalWrite(LED_BUILTIN, 0);
}

// Function to handle any other requests.  Returns "404 Not Found".
static void handleNotFound() {
  digitalWrite(LED_BUILTIN, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
  digitalWrite(LED_BUILTIN, 0);
}

// Function to initialise the object.  It connects to the WiFi access point, 
//  registers an mDNS name (e.g. ESP.local) and sets up the web server.
void HttpManagerClass::begin(const char *ssid, const char *password, const char *dnsName) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  Serial.print(F("Connecting to WiFi \""));
  Serial.print(ssid);
  Serial.print("\" ");
  int retries = 20; // try for 10 seconds
  while (WiFi.status() != WL_CONNECTED && --retries > 0) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("\r\nCannot connect"));
    return;
  }

  connected = true;
  
  Serial.println();
  Serial.print(F("Connected, IP address: "));
  Serial.println(WiFi.localIP());

  if (MDNS.begin(dnsName)) {
    Serial.println(F("MDNS responder started"));
  }
  
  server.on("/", handleRoot);
  server.on("/data", handleBitData);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println(F("HTTP server started"));
}

// Function to set the pointer to dynamic data to be sent to the
//  web client on request.
void HttpManagerClass::setBuffer(char *bp) {
  bufferPointer = bp;
}

// Function to be called from the loop() function to do all the
//  regular processing related to the class.
void HttpManagerClass::process() {
  if (connected)
    server.handleClient();
}

// Declare singleton class instance.
HttpManagerClass HttpManager;

#endif
