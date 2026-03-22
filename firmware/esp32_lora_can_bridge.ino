/*
 * ESP32 LoRa to CAN Bridge (MCP2515)
 * 
 * Receives target GPS data via LoRa and forwards it to the Antenna Tracker RPi via CAN bus.
 * Protocol matches 'can_bridge_node.cpp'.
 */

#include <SPI.h>
#include <mcp2515.h>
#include <LoRa.h>

// Pins for MCP2515
#define CAN_CS_PIN 5
#define CAN_INT_PIN 4

// Pins for LoRa (Standard ESP32 LoRa pins)
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

MCP2515 mcp2515(CAN_CS_PIN);

struct can_frame canMsgGps;
struct can_frame canMsgStatus;

void setup() {
  Serial.begin(115200);

  // Initialize MCP2515
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  // Initialize LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(915E6)) { // Adjust frequency as needed (433E6, 868E6, 915E6)
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  Serial.println("ESP32 LoRa-CAN Bridge Ready.");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Expected LoRa Packet: 
    // [lat_int32][lon_int32][alt_int16][status_uint8] (Total 11 bytes)
    
    if (packetSize >= 11) {
      int32_t lat, lon;
      int16_t alt;
      uint8_t status;
      
      byte buffer[11];
      for (int i = 0; i < 11; i++) {
        buffer[i] = LoRa.read();
      }
      
      memcpy(&lat, &buffer[0], 4);
      memcpy(&lon, &buffer[4], 4);
      memcpy(&alt, &buffer[8], 2);
      status = buffer[10];

      // Forward GPS to CAN (ID 0x200)
      canMsgGps.can_id  = 0x200;
      canMsgGps.can_dlc = 8;
      memcpy(&canMsgGps.data[0], &lat, 4);
      memcpy(&canMsgGps.data[4], &lon, 4);
      mcp2515.sendMessage(&canMsgGps);

      // Forward Status to CAN (ID 0x201)
      canMsgStatus.can_id  = 0x201;
      canMsgStatus.can_dlc = 5;
      int16_t rssi = (int16_t)LoRa.packetRssi();
      memcpy(&canMsgStatus.data[0], &alt, 2);
      memcpy(&canMsgStatus.data[2], &rssi, 2);
      canMsgStatus.data[4] = status; // link_quality
      mcp2515.sendMessage(&canMsgStatus);

      Serial.printf("Forwarded GPS: Lat=%d, Lon=%d, RSSI=%d\n", lat, lon, rssi);
    }
  }
}
