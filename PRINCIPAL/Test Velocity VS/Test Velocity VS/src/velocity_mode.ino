/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial2
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 23; // DYNAMIXEL Shield DIR PIN
#define RXD2 16
#define TXD2 17

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// This namespace is required to use Control table item names
using namespace ControlTableItem;

void scanIDs(uint8_t maxID = 20) {
  DEBUG_SERIAL.println("\n[SCAN ID] Recherche de Dynamixel...");
  bool foundAny = false;
  for (uint8_t id = 1; id <= maxID; id++) {
    DEBUG_SERIAL.print("  ID ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" -> ");
    if (dxl.ping(id)) {
      DEBUG_SERIAL.println("TROUVÉ");
      foundAny = true;
    } else {
      DEBUG_SERIAL.println("absent");
    }
    delay(30);
  }
  if (!foundAny) {
    DEBUG_SERIAL.print("  Aucun Dynamixel trouvé entre 1 et ");
    DEBUG_SERIAL.println(maxID);
  }
}

void setup() {
  DEBUG_SERIAL.begin(115200);
  delay(500);
  DXL_SERIAL.begin(57600, SERIAL_8N1, RXD2, TXD2);
  delay(500);

  DEBUG_SERIAL.println("\n=== DYNAMIXEL DEBUG ===");

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.print("Ping ID ");
  DEBUG_SERIAL.print(DXL_ID);
  DEBUG_SERIAL.print(" : ");
  bool pingOK = dxl.ping(DXL_ID);
  DEBUG_SERIAL.println(pingOK ? "OK" : "FAIL");

  if (!pingOK) {
    scanIDs(20);
  }

  DEBUG_SERIAL.print("Torque OFF ID ");
  DEBUG_SERIAL.print(DXL_ID);
  DEBUG_SERIAL.print(" : ");
  bool torqueOffOK = dxl.torqueOff(DXL_ID);
  DEBUG_SERIAL.println(torqueOffOK ? "OK" : "FAIL");

  DEBUG_SERIAL.print("Set OPCODE VELOCITY : ");
  bool modeOK = dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  DEBUG_SERIAL.println(modeOK ? "OK" : "FAIL");

  DEBUG_SERIAL.print("Torque ON ID ");
  DEBUG_SERIAL.print(DXL_ID);
  DEBUG_SERIAL.print(" : ");
  bool torqueOnOK = dxl.torqueOn(DXL_ID);
  DEBUG_SERIAL.println(torqueOnOK ? "OK" : "FAIL");

  DEBUG_SERIAL.println("\n=== FIN SETUP ===\n");
}

void loop() {
  dxl.setGoalVelocity(DXL_ID, 50, UNIT_PERCENT);
  delay(500);

  DEBUG_SERIAL.print("Present Velocity (raw) : ");
  DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID));

  DEBUG_SERIAL.print("Present Velocity (RPM) : ");
  DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID, UNIT_RPM));

  DEBUG_SERIAL.println("---");
  delay(1000);
}
