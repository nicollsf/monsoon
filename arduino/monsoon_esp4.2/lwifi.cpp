// Fred Nicolls, March 2024

#include <Arduino.h>
#include "lwifi.h"
#include "system.h"
#include "ota.h"


#define SCAN_PERIOD 30000
#define SCAN_DURATION 30000
unsigned long lastScanMillis = 0;
#define CONNECT_PERIOD 30000
unsigned long lastConnMillis = 0;
int bestssid = -1;

unsigned long lastConnRep = 0;


int wifi_getbestssid()
{
  int bestssid = -1;

  // List discovered wifi ssids
  int n = WiFi.scanComplete();
  if( n==0 ) {
    Serial.println("no networks found");
    return(-1);
  }
  Serial.println("wifi_getbestssid(): networks found=" + String(n));
  for( int i=0; i<n; i++ ) {
    Serial.println(String(i) + ": " + WiFi.SSID(i) + " (" + WiFi.RSSI(i) + ")");
  }

  // Find strongest network in permitted list
  float rssimax = -100000;
  for( int i=0; i<numssids; i++ ) {
    for( int j=0; j<n; j++ ) {
      if( String(WiFi.SSID(j))==ssids[i] ) {
        if( bestssid==-1 || WiFi.RSSI(j)>rssimax ) {
          bestssid = i;
          rssimax = WiFi.RSSI(j);
        }
      }
    }
  }
  
  return(bestssid);
}


void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info){
  int wifilog = 2;  
  unsigned long currentMillis = millis();
  if( wifilog>=1 ) Serial.println("WiFiEvent: received " + String(WiFi.eventName(event)));
  
  switch( event ) {
    case ARDUINO_EVENT_WIFI_READY: 
      Serial.println("WiFiEvent(READY): WiFi interface ready");
      //WiFi.mode(WIFI_STA);
      break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
      if( wifilog>=1 ) Serial.println("WiFiEvent(SCAN_DONE): Completed scan for access points");

      // Get index of strongest network
      bestssid = wifi_getbestssid();
      if( bestssid>=0 ) Serial.println("Best SSID is " + ssids[bestssid]);
      
      WiFi.scanDelete();  // forget scanned networks
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      if( wifilog>=2 ) Serial.println("WiFiEvent(STA_START): WiFi client started");
      if( wifilog>=2 ) Serial.println("WiFiEvent(STA_START): calling WiFi.scanNetworks(true)");
      WiFi.disconnect();
      if( millis()-lastScanMillis>SCAN_PERIOD ) {
        lastScanMillis = millis();
        WiFi.scanNetworks(true);
      }
      break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
      if( wifilog>=2 ) Serial.println("WiFiEvent(STA_STOP): WiFi clients stopped");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      if( wifilog>=2 ) Serial.println("WiFiEvent(STA_CONNECTED): Connected to access point");
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      if( wifilog>=2 ) Serial.println("WiFiEvent(STA_DISCONNECTED): Disconnected from WiFi access point");
      if( wifilog>=2 ) Serial.print("WiFiEvent(STA_DISCONNECTED): reason=");
      if( wifilog>=2 ) Serial.println(WiFi.disconnectReasonName((wifi_err_reason_t)info.wifi_sta_disconnected.reason));

      switch( info.wifi_sta_disconnected.reason ) {
        case WIFI_REASON_NO_AP_FOUND:
          if( currentMillis-lastScanMillis>SCAN_DURATION ) {
            if( wifilog>=2 ) Serial.println("WiFiEvent(REASON_NO_AP_FOUND): calling WiFi.scanNetworks(true)");
            WiFi.disconnect();
            if( millis()-lastScanMillis>SCAN_PERIOD ) {
              lastScanMillis = millis();
              WiFi.scanNetworks(true);
            }
      }
          break;
      }

      break;
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
      if( wifilog>=1 ) Serial.println("WiFiEvent(STA_AUTHMODE_CHANGE): Authentication mode of access point has changed");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      if( wifilog>=1 ) Serial.print("WiFiEvent(STA_GOT_IP): Obtained IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      if( wifilog>=1 ) Serial.println("WiFiEvent(STA_LOST_IP): Lost IP address and IP address is reset to 0");
      break;
    case ARDUINO_EVENT_WPS_ER_SUCCESS:
      if( wifilog>=1 ) Serial.println("WiFiEvent(WPS_ER_SUCCESS): WiFi Protected Setup (WPS): succeeded in enrollee mode");
      break;
    case ARDUINO_EVENT_WPS_ER_FAILED:
      if( wifilog>=1 ) Serial.println("WiFiEvent(WPS_ER_FAILED): WiFi Protected Setup (WPS): failed in enrollee mode");
      break;
    case ARDUINO_EVENT_WPS_ER_TIMEOUT:
      if( wifilog>=1 ) Serial.println("WiFiEvent(WPS_ER_TIMEOUT): WiFi Protected Setup (WPS): timeout in enrollee mode");
      break;
    case ARDUINO_EVENT_WPS_ER_PIN:
      if( wifilog>=1 ) Serial.println("WiFiEvent(WPS_ER_PIN): WiFi Protected Setup (WPS): pin code in enrollee mode");
      break;
    case ARDUINO_EVENT_WIFI_AP_START:
      if( wifilog>=1 ) Serial.println("WiFiEvent(AP_START): WiFi access point started");
      break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
      if( wifilog>=1 ) Serial.println("WiFiEvent(AP_STOP): WiFi access point stopped");
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      if( wifilog>=1 ) Serial.println("WiFiEvent(AP_STACONNECTED): Client connected");
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      if( wifilog>=1 ) Serial.println("WiFiEvent(AP_STADISCONNECTED): Client disconnected");
      break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
      if( wifilog>=1 ) Serial.println("WiFiEvent(AP_STAIPASSIGNED): Assigned IP address to client");
      break;
    case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
      if( wifilog>=1 ) Serial.println("WiFiEvent(AP_PROBEREQRECVED): Received probe request");
      break;
    case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
      if( wifilog>=1 ) Serial.println("WiFiEvent(AP_GOT_IP6): AP IPv6 is preferred");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
      if( wifilog>=1 ) Serial.println("WiFiEvent(STA_GOT_IP6): STA IPv6 is preferred");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP6:
      if( wifilog>=1 ) Serial.println("WiFiEvent(ETH_GOT_IP6): Ethernet IPv6 is preferred");
      break;
    case ARDUINO_EVENT_ETH_START:
      if( wifilog>=1 ) Serial.println("WiFiEvent(ETH_START): Ethernet started");
      break;
    case ARDUINO_EVENT_ETH_STOP:
      if( wifilog>=1 ) Serial.println("WiFiEvent(ETH_STOP): Ethernet stopped");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      if( wifilog>=1 ) Serial.println("WiFiEvent(ETH_CONNECTED): Ethernet connected");
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      if( wifilog>=1 ) Serial.println("WiFiEvent(ETH_DISCONNECTED): Ethernet disconnected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      if( wifilog>=1 ) Serial.println("WiFiEvent(ETH_GOT_IP): Obtained IP address");
      break;
    default: break;
  }
}


void setup_wifi() 
{
  WiFi.onEvent(WiFiEvent);  // register wifi event handler
  
  // Start wifi
  bestssid = -1;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("esp32-monsoon"); // Set custom hostname for DHCP
  WiFi.disconnect();
  delay(100);
}


void loop_wifi() 
{
  unsigned long currentMillis = millis();

  // switch( WiFi.status() ) {
  //   case WL_STOPPED:  Serial.println("WL_STOPPED");  break;
  //   case WL_CONNECTED:  Serial.println("WL_CONNECTED");  break;
  //   case WL_NO_SHIELD:  Serial.println("WL_NO_SHIELD");  break;
  //   case WL_IDLE_STATUS:  Serial.println("WL_IDLE_STATUS");  break;
  //   case WL_CONNECT_FAILED:  Serial.println("WL_CONNECT_FAILED");  break;
  //   case WL_NO_SSID_AVAIL:  Serial.println("WL_NO_SSID_AVAIL");  break; 
  //   case WL_SCAN_COMPLETED:  Serial.println("WL_SCAN_COMPLETED");  break; 
  //   case WL_CONNECTION_LOST:  Serial.println("WL_CONNECTION_LOST");  break;
  //   case WL_DISCONNECTED:  Serial.println("WL_DISCONNECTED");  break;  
  //   default:  Serial.println("WiFi.status() returned " + String(WiFi.status()));
  // }

  int status = WiFi.status();
  if( status==WL_STOPPED || status==WL_DISCONNECTED ) {
    if( bestssid==-1 ) {
      if( currentMillis-lastScanMillis>SCAN_PERIOD ) {
        Serial.println("Calling WiFi.scanNetworks(true)");
        lastScanMillis = millis();
        WiFi.scanNetworks(true);
      }
    }
    else {
      lastConnMillis = millis();
      Serial.println("Calling WiFi.begin for " + ssids[bestssid]);
      WiFi.begin(ssids[bestssid].c_str(), ssidpasses[bestssid].c_str());  
      bestssid = -1; // Reset to call WiFi.begin only once per scan
    }
  }

  // Timeout so trigger reconnect of wifi to best network
  if( WiFi.status()!=WL_CONNECTED && currentMillis-lastConnMillis>CONNECT_PERIOD ) bestssid = -1;

  return;
}