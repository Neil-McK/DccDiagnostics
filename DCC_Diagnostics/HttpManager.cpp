#if defined(ESP32) || defined(ESP8266)

#include "HttpManager.h"

static char *bufferPointer = 0;
static WebServer server;

static void handleRoot() {
  digitalWrite(LED_BUILTIN, 1);
  String temp = 
    "<html>\r\n"
      "<script>\r\n"
      "  window.setInterval('reloadIFrame();', 1000);\r\n"
      "  function reloadIFrame() {\r\n"
      "    document.getElementById('bitData').src='/bitData';\r\n"
//      "    document.getElementById('packetData').src='/packetData';\r\n"
      "  }\r\n"
      "</script>\r\n"
      "<head>\r\n"
      "  <title>DCC Diagnostics - Bit analysis</title>\r\n"
      "  <style>\r\n"
      "    body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\r\n"
      "  </style>\r\n"
      "</head>\r\n"
      "<body>\r\n"
      "  <h1>DCC Diagnostics</h1>\r\n"
      "  <p>Information regarding bit quality:</p>"
      "  <p><iframe id=\"bitData\" src=\"/bitData\"></iframe></p>\r\n"
//      "  <p>Information regarding packet quality:</p>"
//      "  <p><iframe id=\"packetData\" src=\"/packetData\"></iframe></p>\r\n"
      "</body>\r\n"
    "</html>";

  server.send(200, "text/html", temp);
  digitalWrite(LED_BUILTIN, 0);
}

static void handleBitData() {
  digitalWrite(LED_BUILTIN, 1);
  if (bufferPointer) 
    server.send(200, "text/plain", bufferPointer);
  else
    server.send(200, "text/plain", "");
  digitalWrite(LED_BUILTIN, 0);
}

//static void handlePacketData() {
//  digitalWrite(LED_BUILTIN, 1);
//  server.send(200, "text/plain", "To be implemented");
//  digitalWrite(LED_BUILTIN, 0);
//}

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

void HttpManagerClass::begin(const char *ssid, const char *password, const char *dnsName) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  int retries = 20; // try for 10 seconds
  while (WiFi.status() != WL_CONNECTED && --retries > 0) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.print("Cannot connect to ");
    Serial.println(ssid);
    return;
  }

  connected = true;
  
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin(dnsName)) {
    Serial.println("MDNS responder started");
  }
  
  server.on("/", handleRoot);
  server.on("/bitData", handleBitData);
  //server.on("/packetData", handlePacketData);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void HttpManagerClass::setBuffer(char *bp) {
  bufferPointer = bp;
}

void HttpManagerClass::process() {
  if (connected)
    server.handleClient();
}

// Declare singleton object
HttpManagerClass HttpManager;

#endif
