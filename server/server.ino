/*
   Copyright (c) 2015, Majenko Technologies
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

 * * Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

 * * Redistributions in binary form must reproduce the above copyright notice, this
     list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

 * * Neither the name of Majenko Technologies nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#if defined(ESP8266)

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <FS.h>
typedef ESP8266WebServer WebServer_t;

#elif defined(ESP32)

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <Wire.h>
#include <SPIFFS.h>
typedef WebServer WebServer_t;
#define LED_BUILTIN 2

#else

#error "Supported boards: NodeMCU, ESP32"

#endif

#ifndef STASSID
#define STASSID "wingServer"
#define STAPSK  "123456789"
#endif

const char InoId=8;
const char *ssid = STASSID;
const char *password = STAPSK;
WebServer_t server(80);

const int led = LED_BUILTIN;

void updateGains(){
  digitalWrite(led,0);
  String message="";
  if (server.hasArg("kp")){
    message+="kp found\n";
    float kp=server.arg("kp").toFloat();
    Wire.beginTransmission(InoId);
    Wire.write(byte('p'));
    Wire.write((byte *)&kp,sizeof(float));
    Wire.endTransmission();
  }
  else{
    message+="kp not found\n";
  }
  if (server.hasArg("ki")){
    message+="ki found\n";
    float ki=server.arg("ki").toFloat();
    Wire.beginTransmission(InoId);
    Wire.write(byte('i'));
    Wire.write((byte *)&ki,sizeof(float));
    Wire.endTransmission();
  }
  else{
    message+="ki not found\n";
  }
  if (server.hasArg("kd")){
    message+="kd found\n";
    float kd=server.arg("kd").toFloat();
    Wire.beginTransmission(InoId);
    Wire.write(byte('d'));
    Wire.write((byte *)&kd,sizeof(float));
    Wire.endTransmission();
  }
  else{
    message+="kd not found\n";
  }
  server.send(200, "text/plain", message);
  digitalWrite(led,1);
}

void handleNotFound() {
  digitalWrite(led, 0);
  String uri=server.uri();
  Serial.println(uri);
  if (SPIFFS.exists(uri)) {
    File f=SPIFFS.open(uri,"r");
    String mime=getMIMEType(uri);
    server.streamFile<File>(f, mime);
    f.close();
    return;
  }
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
  digitalWrite(led, 1);
}

void drawGraph() {
  String out;
  out.reserve(2600);
  char temp[70];
  out += "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"400\" height=\"150\">\n";
  out += "<rect width=\"400\" height=\"150\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
  out += "<g stroke=\"black\">\n";
  int y = rand() % 130;
  for (int x = 10; x < 390; x += 10) {
    int y2 = rand() % 130;
    sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke-width=\"1\" />\n", x, 140 - y, x + 10, 140 - y2);
    out += temp;
    y = y2;
  }
  out += "</g>\n</svg>\n";

  server.send(200, "image/svg+xml", out);
}

String getMIMEType(String filename){
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  return "text/plain";
}

struct WingData {
  uint16_t pos;
  float cs1;
  float cs2;
}wingData; //has to total less than or equal to 32 bytes long

struct VehicleData{
  uint8_t kph;
}vehicleData; //the 32 byte limit also holds here

void setup(void) {
  pinMode(led, OUTPUT);
  digitalWrite(led,1);
  Serial.begin(115200);
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println("");

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", [](){
    File f=SPIFFS.open("/index.html","r");
    server.streamFile<File>(f, "text/html");
    f.close();
  });
  server.on("/index.html", [](){
    File f=SPIFFS.open("/index.html","r");
    server.streamFile<File>(f, "text/html");
    f.close();
  });
  server.on("/index.js", [](){
    File f=SPIFFS.open("/index.js","r");
    server.streamFile<File>(f, "text/js");
    f.close();
  });
  server.on("/test.svg", drawGraph);
  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });
  server.on("/action.php",updateGains);
  server.on("/events",[](){
    WiFiClient client=server.client();
    if (client){
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/event-stream;charset=UTF-8");
      client.println("Connection: close");               // the connection will be closed after completion of the response
      client.println("Access-Control-Allow-Origin: *");  // allow any connection. We don't want Arduino to host all of the website ;-)
      client.println("Cache-Control: no-cache");         // refresh the page automatically every 5 sec
      client.println();
      client.flush();
      client.println("event: wingSensorsUpdate");                 // this name could be anything, really.
      client.print("data: {");
      
      Wire.requestFrom(InoId,sizeof(WingData));
      int idx=0;
      while(Wire.available()){
        ((char *)&wingData)[idx++]=Wire.read();
      }
      Wire.requestFrom(InoId,sizeof(VehicleData));
      int idx=0;
      while(Wire.available()){
        ((char *)&vehicleData)[idx++]=Wire.read();
      }
      
      client.print("\"speed\":"+String(vehicleData.kph));
      client.print(",\"cs1\":"+String(wingData.cs1));
      client.print(",\"cs2\":"+String(wingData.cs2));
      client.println("}");
      client.println();
      client.flush();
    }
  });
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
  
  Wire.begin();
}

void loop(void) {
  server.handleClient();
}
