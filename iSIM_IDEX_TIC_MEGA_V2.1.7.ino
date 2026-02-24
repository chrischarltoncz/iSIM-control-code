/*
 * IDEX HT2425 Valve + Pololu Tic T500 Stepper Controller
 * Arduino Mega - Controls two IDEX valves + one stepper motor via UART
 *
 * V2.1.7 changes:
 * - fixed an issue were there was unintended motor motion when a movement was not commanded via USB
 *
 * Hardware Connections:
 * ---------------------
 * IDEX Valve 1 (Serial1 - pins 18/19):
 *   - Arduino TX1 (pin 18) -> IDEX J2-10 (RXD)
 *   - Arduino RX1 (pin 19) -> IDEX J2-9 (TXD)
 *   - Arduino GND -> IDEX J2-4 (GND)
 *
 * IDEX Valve 2 (Serial2 - pins 16/17):
 *   - Arduino TX2 (pin 16) -> IDEX J2-10 (RXD)
 *   - Arduino RX2 (pin 17) -> IDEX J2-9 (TXD)
 *   - Arduino GND -> IDEX J2-4 (GND)
 *
 * Pololu Tic T500 Stepper (Serial3 - pins 14/15):
 *   - Arduino TX3 (pin 14) -> Tic RX
 *   - Arduino RX3 (pin 15) -> Tic TX
 *   - Arduino GND -> Tic GND
 *
 * PC Commands (via USB at 115200 baud):
 * -------------------------------------
 * VALVE COMMANDS:
 *   1P<n>   - Move valve 1 to position n (1-24)
 *   2P<n>   - Move valve 2 to position n (1-24)
 *   1S      - Query valve 1 status
 *   2S      - Query valve 2 status
 *   1M      - Home valve 1
 *   2M      - Home valve 2
 *   1E      - Read valve 1 error code
 *   2E      - Read valve 2 error code
 *
 * STEPPER COMMANDS:
 *   SV<n>   - Set stepper velocity (microsteps/10000s, negative=reverse)
 *             At 1/2 microstep mode (default): 2,000,000 = 100 full steps/sec
 *   SP<n>   - Set stepper target position (microsteps)
 *             At 1/2 microstep mode (default): 400 microsteps = 1 revolution
 *   SS      - Stop stepper (halt and hold)
 *   SO      - Stepper ON (energize + exit safe start)
 *   SF      - Stepper OFF (de-energize)
 *   SR      - Read stepper status
 *   SC      - Set current position to 0
 *   SM<n>   - Set max speed (microsteps/10000s)
 *   SA<n>   - Set max acceleration (microsteps/100s^2)
 *
 *   ?       - Show help
 *
 * STEP MODE vs VELOCITY REFERENCE (200-step/rev motor):
 * -------------------------------------------------------
 *   Mode 0 (Full):  200 steps/rev,  SV1000000 = 100 full steps/sec
 *   Mode 1 (1/2):   400 steps/rev,  SV1000000 =  50 full steps/sec  <-- DEFAULT
 *   Mode 2 (1/4):   800 steps/rev,  SV1000000 =  25 full steps/sec
 *   Mode 3 (1/8):  1600 steps/rev,  SV1000000 = 12.5 full steps/sec
 */

#include <Arduino.h>

// ==================== Keepalive ====================
unsigned long lastTicKeepalive = 0;
#define TIC_KEEPALIVE_MS 500
bool ticMotorActive = false;  // Only send keepalive when motor is intentionally running

// ==================== Configuration ====================
#define USB_BAUD         115200
#define VALVE_BAUD       19200
#define TIC_BAUD         9600
#define RESPONSE_TIMEOUT 3000
#define CMD_BUFFER_SIZE  32
#define MAX_POSITIONS    24

// ==================== Tic T500 Hard-Coded Settings ====================
// Step mode: 0=Full, 1=1/2, 2=1/4, 3=1/8 (T500 supports up to 1/8)
// NOTE: All SV and SP values are in MICROSTEPS.
//   At mode 1 (1/2): a 200-step/rev motor needs 400 microsteps/rev.
//   Velocity formula: full_steps_per_sec = SV_value / 10000 / (1 << TIC_STEP_MODE)
#define TIC_STEP_MODE    1   // 0=Full, 1=1/2, 2=1/4, 3=1/8 --> currently 1/2 microstepping

// Current limit code for T500 (0-32). See table below:
//   Code  mA       Code  mA       Code  mA
//   0     0        11    1281     22    2131
//   1     1        12    1368     23    2207
//   2     174      13    1452     24    2285
//   3     343      14    1532     25    2366
//   4     495      15    1611     26    2451
//   5     634      16    1687     27    2540
//   6     762      17    1762     28    2634
//   7     880      18    1835     29    2734
//   8     990      19    1909     30    2843
//   9     1092     20    1982     31    2962
//   10    1189     21    2056     32    3093
#define TIC_CURRENT_LIMIT  9   // Code 7 = 880 mA

// Serial port aliases
#define PC_SERIAL     Serial
#define VALVE1_SERIAL Serial1
#define VALVE2_SERIAL Serial2
#define TIC_SERIAL    Serial3

// IDEX Protocol Constants
#define IDEX_CR   0x0D
#define IDEX_BUSY 0x2A

// Tic Command Bytes (Compact Protocol)
#define TIC_CMD_SET_TARGET_POSITION  0xE0
#define TIC_CMD_SET_TARGET_VELOCITY  0xE3
#define TIC_CMD_HALT_AND_HOLD        0x89
#define TIC_CMD_HALT_AND_SET_POS     0xEC
#define TIC_CMD_DEENERGIZE           0x86
#define TIC_CMD_ENERGIZE             0x85
#define TIC_CMD_EXIT_SAFE_START      0x83
#define TIC_CMD_GET_VARIABLE         0xA1
#define TIC_CMD_SET_MAX_SPEED        0xE6
#define TIC_CMD_SET_MAX_ACCEL        0xEA
#define TIC_CMD_SET_MAX_DECEL        0xE9
#define TIC_CMD_SET_STEP_MODE        0x94
#define TIC_CMD_SET_CURRENT_LIMIT    0x91

// Tic Variable Offsets
#define TIC_VAR_OPERATION_STATE  0x00
#define TIC_VAR_MISC_FLAGS       0x01
#define TIC_VAR_ERROR_STATUS     0x02
#define TIC_VAR_CURRENT_POSITION 0x22
#define TIC_VAR_CURRENT_VELOCITY 0x26
#define TIC_VAR_TARGET_POSITION  0x0A
#define TIC_VAR_TARGET_VELOCITY  0x0E

char cmdBuffer[CMD_BUFFER_SIZE];
int  cmdIndex = 0;

// ==================== Function Prototypes ====================
void processCommand(char* cmd);
// IDEX
int         sendValveCommand(HardwareSerial& port, char cmd, int value, bool hasValue);
void        moveValve(int valveNum, int position);
void        queryStatus(int valveNum);
void        homeValve(int valveNum);
void        readError(int valveNum);
HardwareSerial& getValveSerial(int valveNum);
const char* getErrorDescription(int errCode);
// Tic
void    ticSendCommand(uint8_t cmd);
void    ticSendCommand7Bit(uint8_t cmd, uint8_t data);
void    ticSendCommand32Bit(uint8_t cmd, int32_t data);
int32_t ticGetVariable32(uint8_t offset);
int16_t ticGetVariable16(uint8_t offset);   // FIX: was declared but never implemented
uint8_t ticGetVariable8(uint8_t offset);
void    ticSetVelocity(int32_t velocity);
void    ticSetPosition(int32_t position);
void    ticStop();
void    ticEnergize();
void    ticDeenergize();
void    ticSetMaxSpeed(uint32_t speed);
void    ticSetMaxAccel(uint32_t accel);
void    ticSetCurrentPosition(int32_t position);
void    ticReadStatus();
void    ticSetStepMode(uint8_t mode);
void    ticSetCurrentLimit(uint8_t code);
void    printHelp();

// ==================== Setup ====================
void setup() {
  PC_SERIAL.begin(USB_BAUD);
  while (!PC_SERIAL && millis() < 3000);

  VALVE1_SERIAL.begin(VALVE_BAUD, SERIAL_8N1);
  VALVE2_SERIAL.begin(VALVE_BAUD, SERIAL_8N1);
  TIC_SERIAL.begin(TIC_BAUD, SERIAL_8N1);

  delay(100);

  ticSetStepMode(TIC_STEP_MODE);
  ticSetCurrentLimit(TIC_CURRENT_LIMIT);

  PC_SERIAL.println(F("\n============================================="));
  PC_SERIAL.println(F("IDEX HT2425 + Tic T500 Controller"));
  PC_SERIAL.println(F("============================================="));
  PC_SERIAL.println(F("Valve 1: Serial1 (TX1=18, RX1=19) @ 19200"));
  PC_SERIAL.println(F("Valve 2: Serial2 (TX2=16, RX2=17) @ 19200"));
  PC_SERIAL.println(F("Stepper: Serial3 (TX3=14, RX3=15) @ 9600"));
  PC_SERIAL.print(F("  Step mode: 1/"));
  PC_SERIAL.print(1 << TIC_STEP_MODE);
  PC_SERIAL.println(F(" microstepping"));
  PC_SERIAL.print(F("  Microsteps/rev (200-step motor): "));
  PC_SERIAL.println(200 * (1 << TIC_STEP_MODE));
  PC_SERIAL.print(F("  Current limit code: "));
  PC_SERIAL.print(TIC_CURRENT_LIMIT);
  PC_SERIAL.println(F(" (880 mA)"));
  PC_SERIAL.println(F("Type '?' for help"));
  PC_SERIAL.println(F("Ready.\n"));
}

// ==================== Main Loop ====================
void loop() {
  while (PC_SERIAL.available()) {
    char c = PC_SERIAL.read();
    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        processCommand(cmdBuffer);
        cmdIndex = 0;
      }
    } else if (cmdIndex < CMD_BUFFER_SIZE - 1) {
      cmdBuffer[cmdIndex++] = toupper(c);
    }
  }

  // Keepalive to prevent Tic command timeout (must be < 1000 ms)
  // Only fires when motor is intentionally active - prevents stale target resuming on reboot
  if (ticMotorActive && millis() - lastTicKeepalive >= TIC_KEEPALIVE_MS) {
    ticSendCommand(TIC_CMD_EXIT_SAFE_START);
    lastTicKeepalive = millis();
  }
}

// ==================== Command Parser ====================
void processCommand(char* cmd) {
  if (strlen(cmd) == 0) return;

  PC_SERIAL.print(F("\n> "));
  PC_SERIAL.println(cmd);

  if (cmd[0] == '?') {
    printHelp();
    return;
  }

  // Stepper commands start with 'S'
  if (cmd[0] == 'S') {
    switch (cmd[1]) {
      case 'V': ticSetVelocity(atol(&cmd[2]));       break;
      case 'P': ticSetPosition(atol(&cmd[2]));       break;
      case 'S': ticStop();                           break;
      case 'O': ticEnergize();                       break;
      case 'F': ticDeenergize();                     break;
      case 'R': ticReadStatus();                     break;
      case 'C': ticSetCurrentPosition(0);            break;
      case 'M': ticSetMaxSpeed(atol(&cmd[2]));       break;
      case 'A': ticSetMaxAccel(atol(&cmd[2]));       break;
      default:
        PC_SERIAL.println(F("ERROR: Unknown stepper command"));
        break;
    }
    return;
  }

  // Valve commands start with '1' or '2'
  int valveNum = cmd[0] - '0';
  if (valveNum < 1 || valveNum > 2) {
    PC_SERIAL.println(F("ERROR: Use 1, 2 for valves or S for stepper"));
    return;
  }

  switch (cmd[1]) {
    case 'P': {
      int pos = atoi(&cmd[2]);
      if (pos < 1 || pos > MAX_POSITIONS) {
        PC_SERIAL.print(F("ERROR: Position must be 1-"));
        PC_SERIAL.println(MAX_POSITIONS);
        return;
      }
      moveValve(valveNum, pos);
      break;
    }
    case 'S': queryStatus(valveNum); break;
    case 'M': homeValve(valveNum);   break;
    case 'E': readError(valveNum);   break;
    default:
      PC_SERIAL.println(F("ERROR: Unknown valve command"));
      break;
  }
}

// ==================== IDEX Valve Functions ====================
HardwareSerial& getValveSerial(int valveNum) {
  return (valveNum == 1) ? VALVE1_SERIAL : VALVE2_SERIAL;
}

int sendValveCommand(HardwareSerial& port, char cmd, int value, bool hasValue) {
  while (port.available()) port.read();

  if (hasValue) {
    char packet[5];
    sprintf(packet, "%c%02X", cmd, value & 0xFF);
    port.print(packet);
    port.write(IDEX_CR);
  } else {
    port.write(cmd);
    port.write(IDEX_CR);
  }
  port.flush();

  unsigned long startTime = millis();
  char response[8];
  int  respIdx = 0;

  while (millis() - startTime < RESPONSE_TIMEOUT) {
    if (port.available()) {
      char c = port.read();
      if (c == IDEX_BUSY) {
        startTime = millis();
        delay(50);
        continue;
      }
      if (c == IDEX_CR) {
        if (respIdx == 0) return -3; // ACK only
        response[respIdx] = '\0';
        return (int)strtol(response, NULL, 16);
      }
      if (respIdx < 6) response[respIdx++] = c;
    }
  }
  return -1; // Timeout
}

void moveValve(int valveNum, int position) {
  HardwareSerial& port = getValveSerial(valveNum);
  PC_SERIAL.print(F("Moving valve "));
  PC_SERIAL.print(valveNum);
  PC_SERIAL.print(F(" to position "));
  PC_SERIAL.println(position);

  int result = sendValveCommand(port, 'P', position, true);
  if (result == -3) {
    PC_SERIAL.println(F("OK: Move accepted"));
  } else if (result == -1) {
    PC_SERIAL.println(F("ERROR: No response"));
  } else {
    PC_SERIAL.print(F("Response: 0x"));
    PC_SERIAL.println(result, HEX);
  }
}

void queryStatus(int valveNum) {
  HardwareSerial& port = getValveSerial(valveNum);
  int result = sendValveCommand(port, 'S', 0, false);

  if (result >= 1 && result <= MAX_POSITIONS) {
    PC_SERIAL.print(F("Position: "));
    PC_SERIAL.println(result);
  } else if (result > MAX_POSITIONS) {
    PC_SERIAL.print(F("ERROR: 0x"));
    PC_SERIAL.print(result, HEX);
    PC_SERIAL.print(F(" - "));
    PC_SERIAL.println(getErrorDescription(result));
  } else {
    PC_SERIAL.println(F("No response"));
  }
}

void homeValve(int valveNum) {
  HardwareSerial& port = getValveSerial(valveNum);
  int result = sendValveCommand(port, 'M', 0, false);
  PC_SERIAL.println(result == -3 ? F("OK: Home accepted") : F("No response"));
}

void readError(int valveNum) {
  HardwareSerial& port = getValveSerial(valveNum);
  int result = sendValveCommand(port, 'E', 0, false);
  if (result >= 0) {
    PC_SERIAL.print(F("Error: 0x"));
    PC_SERIAL.print(result, HEX);
    PC_SERIAL.print(F(" - "));
    PC_SERIAL.println(result == 0 ? "None" : getErrorDescription(result));
  }
}

const char* getErrorDescription(int errCode) {
  switch (errCode) {
    case 0x63: return "Valve failure";
    case 0x58: return "NVM error";
    case 0x4D: return "Config error";
    case 0x42: return "Position error";
    case 0x37: return "Integrity error";
    case 0x2C: return "CRC error";
    default:   return "Unknown";
  }
}

// ==================== Tic Low-Level Comms ====================

// Quick command (no data)
void ticSendCommand(uint8_t cmd) {
  TIC_SERIAL.write(cmd);
}

// 7-bit write command (cmd + 1 data byte, MSB always 0)
void ticSendCommand7Bit(uint8_t cmd, uint8_t data) {
  TIC_SERIAL.write(cmd);
  TIC_SERIAL.write(data & 0x7F);
}

// 32-bit write command (Pololu compact protocol)
// Byte layout: cmd | MSBs-byte | b0 | b1 | b2 | b3
// Each data byte is 7-bit; bit 7 of each is packed into the MSBs byte.
void ticSendCommand32Bit(uint8_t cmd, int32_t data) {
  uint32_t val = (uint32_t)data;
  TIC_SERIAL.write(cmd);
  TIC_SERIAL.write((uint8_t)(
    ((val >>  7) & 0x01) |   // bit7 of byte0 -> MSBs bit0
    ((val >> 14) & 0x02) |   // bit7 of byte1 -> MSBs bit1
    ((val >> 21) & 0x04) |   // bit7 of byte2 -> MSBs bit2
    ((val >> 28) & 0x08)     // bit7 of byte3 -> MSBs bit3
  ));
  TIC_SERIAL.write((uint8_t)(val >>  0) & 0x7F);
  TIC_SERIAL.write((uint8_t)(val >>  8) & 0x7F);
  TIC_SERIAL.write((uint8_t)(val >> 16) & 0x7F);
  TIC_SERIAL.write((uint8_t)(val >> 24) & 0x7F);
}

// Read 32-bit variable from Tic (4-byte little-endian response)
int32_t ticGetVariable32(uint8_t offset) {
  while (TIC_SERIAL.available()) TIC_SERIAL.read();

  TIC_SERIAL.write(TIC_CMD_GET_VARIABLE);
  TIC_SERIAL.write(offset);
  TIC_SERIAL.write((uint8_t)4);

  unsigned long start = millis();
  while (TIC_SERIAL.available() < 4 && millis() - start < 100);

  if (TIC_SERIAL.available() >= 4) {
    uint8_t b0 = TIC_SERIAL.read();
    uint8_t b1 = TIC_SERIAL.read();
    uint8_t b2 = TIC_SERIAL.read();
    uint8_t b3 = TIC_SERIAL.read();
    return (int32_t)((uint32_t)b0 |
                     ((uint32_t)b1 << 8)  |
                     ((uint32_t)b2 << 16) |
                     ((uint32_t)b3 << 24));
  }
  return 0;
}

// FIX: Read 16-bit variable from Tic (2-byte little-endian response)
// Was declared in prototypes but never implemented in v2.1.5.
int16_t ticGetVariable16(uint8_t offset) {
  while (TIC_SERIAL.available()) TIC_SERIAL.read();

  TIC_SERIAL.write(TIC_CMD_GET_VARIABLE);
  TIC_SERIAL.write(offset);
  TIC_SERIAL.write((uint8_t)2);

  unsigned long start = millis();
  while (TIC_SERIAL.available() < 2 && millis() - start < 100);

  if (TIC_SERIAL.available() >= 2) {
    uint8_t b0 = TIC_SERIAL.read();
    uint8_t b1 = TIC_SERIAL.read();
    return (int16_t)((uint16_t)b0 | ((uint16_t)b1 << 8));
  }
  return 0;
}

// Read 8-bit variable from Tic (1-byte response)
uint8_t ticGetVariable8(uint8_t offset) {
  while (TIC_SERIAL.available()) TIC_SERIAL.read();

  TIC_SERIAL.write(TIC_CMD_GET_VARIABLE);
  TIC_SERIAL.write(offset);
  TIC_SERIAL.write((uint8_t)1);

  unsigned long start = millis();
  while (!TIC_SERIAL.available() && millis() - start < 100);

  if (TIC_SERIAL.available()) return TIC_SERIAL.read();
  return 0;
}

// ==================== Tic High-Level Commands ====================

// Set target velocity (microsteps / 10,000 s)
// Positive = forward, Negative = reverse
// Full steps/sec = velocity / 10000 / (1 << TIC_STEP_MODE)
//   e.g. at 1/2 mode: SV2000000 -> 2000000/10000/2 = 100 full steps/sec
void ticSetVelocity(int32_t velocity) {
  PC_SERIAL.print(F("Setting velocity: "));
  PC_SERIAL.print(velocity);
  PC_SERIAL.print(F(" usteps/10000s  (~"));
  // Compute and display equivalent full steps/sec for clarity
  int32_t fullStepsPerSec = velocity / 10000L / (1 << TIC_STEP_MODE);
  PC_SERIAL.print(fullStepsPerSec);
  PC_SERIAL.println(F(" full steps/sec)"));

  ticSendCommand(TIC_CMD_ENERGIZE);
  ticSendCommand(TIC_CMD_EXIT_SAFE_START);
  ticSendCommand32Bit(TIC_CMD_SET_TARGET_VELOCITY, velocity);

  ticMotorActive = (velocity != 0);
  lastTicKeepalive = millis();

  PC_SERIAL.println(F("OK"));
}

// Set target position (microsteps from position 0)
// At 1/2 microstep mode, 400 microsteps = 1 revolution (200-step motor)
void ticSetPosition(int32_t position) {
  PC_SERIAL.print(F("Setting target position: "));
  PC_SERIAL.print(position);
  PC_SERIAL.println(F(" usteps"));

  ticSendCommand(TIC_CMD_ENERGIZE);
  ticSendCommand(TIC_CMD_EXIT_SAFE_START);
  ticSendCommand32Bit(TIC_CMD_SET_TARGET_POSITION, position);

  ticMotorActive = true;
  lastTicKeepalive = millis();

  PC_SERIAL.println(F("OK"));
}

// Stop motor (halt and hold current position)
void ticStop() {
  PC_SERIAL.println(F("Stopping stepper..."));
  ticSendCommand(TIC_CMD_HALT_AND_HOLD);
  ticMotorActive = false;
  PC_SERIAL.println(F("OK: Motor stopped and holding"));
}

// Energize motor and exit safe start
// HALT_AND_HOLD is sent before EXIT_SAFE_START to clear any stale velocity/position
// target that may have been left from a previous session, preventing unintended movement.
void ticEnergize() {
  PC_SERIAL.println(F("Energizing stepper..."));
  ticSendCommand(TIC_CMD_ENERGIZE);
  ticSendCommand(TIC_CMD_HALT_AND_HOLD);   // clear stale target before releasing safe start
  ticSendCommand(TIC_CMD_EXIT_SAFE_START);
  PC_SERIAL.println(F("OK: Motor energized and holding"));
}

// De-energize motor (coast to stop)
void ticDeenergize() {
  PC_SERIAL.println(F("De-energizing stepper..."));
  ticSendCommand(TIC_CMD_DEENERGIZE);
  ticMotorActive = false;
  PC_SERIAL.println(F("OK: Motor de-energized"));
}

// Set max speed (microsteps / 10,000 s) -- same units as velocity
void ticSetMaxSpeed(uint32_t speed) {
  PC_SERIAL.print(F("Setting max speed: "));
  PC_SERIAL.print(speed);
  PC_SERIAL.print(F(" usteps/10000s  (~"));
  PC_SERIAL.print(speed / 10000UL / (1 << TIC_STEP_MODE));
  PC_SERIAL.println(F(" full steps/sec)"));
  ticSendCommand32Bit(TIC_CMD_SET_MAX_SPEED, (int32_t)speed);
  PC_SERIAL.println(F("OK"));
}

// Set max acceleration (and deceleration) in microsteps / 100 s^2
void ticSetMaxAccel(uint32_t accel) {
  PC_SERIAL.print(F("Setting max accel/decel: "));
  PC_SERIAL.print(accel);
  PC_SERIAL.println(F(" usteps/100s^2"));
  ticSendCommand32Bit(TIC_CMD_SET_MAX_ACCEL, (int32_t)accel);
  ticSendCommand32Bit(TIC_CMD_SET_MAX_DECEL, (int32_t)accel);
  PC_SERIAL.println(F("OK"));
}

// Halt motor and set its internal position register
void ticSetCurrentPosition(int32_t position) {
  PC_SERIAL.print(F("Setting current position to: "));
  PC_SERIAL.print(position);
  PC_SERIAL.println(F(" usteps"));
  ticSendCommand32Bit(TIC_CMD_HALT_AND_SET_POS, position);
  PC_SERIAL.println(F("OK"));
}

// Set step mode (7-bit write 0x94)
// T500 valid: 0=Full, 1=1/2, 2=1/4, 3=1/8
void ticSetStepMode(uint8_t mode) {
  ticSendCommand7Bit(TIC_CMD_SET_STEP_MODE, mode);
}

// Set current limit (7-bit write 0x91)
// T500 codes 0-32 (see table in defines section)
void ticSetCurrentLimit(uint8_t code) {
  ticSendCommand7Bit(TIC_CMD_SET_CURRENT_LIMIT, code);
}

// Read and display full stepper status
void ticReadStatus() {
  PC_SERIAL.println(F("--- Stepper Status ---"));

  uint8_t opState = ticGetVariable8(TIC_VAR_OPERATION_STATE);
  PC_SERIAL.print(F("  Operation state: "));
  switch (opState) {
    case 0:  PC_SERIAL.println(F("Reset"));               break;
    case 2:  PC_SERIAL.println(F("De-energized"));        break;
    case 4:  PC_SERIAL.println(F("Soft error"));          break;
    case 6:  PC_SERIAL.println(F("Waiting for ERR line")); break;
    case 8:  PC_SERIAL.println(F("Starting up"));         break;
    case 10: PC_SERIAL.println(F("Normal"));              break;
    default: PC_SERIAL.println(opState);                  break;
  }

  int32_t curPos = ticGetVariable32(TIC_VAR_CURRENT_POSITION);
  PC_SERIAL.print(F("  Current position: "));
  PC_SERIAL.print(curPos);
  PC_SERIAL.println(F(" usteps"));

  int32_t targPos = ticGetVariable32(TIC_VAR_TARGET_POSITION);
  PC_SERIAL.print(F("  Target position:  "));
  PC_SERIAL.print(targPos);
  PC_SERIAL.println(F(" usteps"));

  int32_t curVel = ticGetVariable32(TIC_VAR_CURRENT_VELOCITY);
  PC_SERIAL.print(F("  Current velocity: "));
  PC_SERIAL.print(curVel);
  PC_SERIAL.print(F(" usteps/10000s  (~"));
  PC_SERIAL.print(curVel / 10000L / (1 << TIC_STEP_MODE));
  PC_SERIAL.println(F(" full steps/sec)"));

  PC_SERIAL.print(F("  Step mode:        1/"));
  PC_SERIAL.println(1 << TIC_STEP_MODE);
  PC_SERIAL.println(F("---------------------"));
}

// ==================== Help ====================
void printHelp() {
  PC_SERIAL.println(F("\n============ Command Reference ============"));
  PC_SERIAL.println(F(""));
  PC_SERIAL.println(F("VALVE COMMANDS (1=valve1, 2=valve2):"));
  PC_SERIAL.println(F("  1P<n>    Move valve 1 to position (1-24)"));
  PC_SERIAL.println(F("  2P<n>    Move valve 2 to position (1-24)"));
  PC_SERIAL.println(F("  1S / 2S  Query valve status"));
  PC_SERIAL.println(F("  1M / 2M  Home valve"));
  PC_SERIAL.println(F("  1E / 2E  Read error code"));
  PC_SERIAL.println(F(""));
  PC_SERIAL.println(F("STEPPER COMMANDS:"));
  PC_SERIAL.println(F("  SO       Stepper ON (energize + exit safe start)"));
  PC_SERIAL.println(F("  SF       Stepper OFF (de-energize / coast)"));
  PC_SERIAL.println(F("  SS       Stop (halt and hold position)"));
  PC_SERIAL.println(F("  SR       Read stepper status"));
  PC_SERIAL.println(F("  SC       Set current position to 0"));
  PC_SERIAL.println(F(""));
  PC_SERIAL.println(F("  SV<n>    Set velocity (microsteps / 10000 s)"));
  PC_SERIAL.println(F("           Positive=forward, Negative=reverse"));
  PC_SERIAL.println(F("           At 1/2 microstep mode (current default):"));
  PC_SERIAL.println(F("             SV2000000  =  100 full steps/sec fwd"));
  PC_SERIAL.println(F("             SV-1000000 =   50 full steps/sec rev"));
  PC_SERIAL.println(F("             SV0        = decelerate to stop"));
  PC_SERIAL.println(F(""));
  PC_SERIAL.println(F("  SP<n>    Set target position (microsteps from 0)"));
  PC_SERIAL.println(F("           At 1/2 microstep mode (current default):"));
  PC_SERIAL.println(F("             400 usteps = 1 revolution (200-step motor)"));
  PC_SERIAL.println(F("             SP400  = move to 1 rev from home"));
  PC_SERIAL.println(F("             SP-400 = move to 1 rev reverse from home"));
  PC_SERIAL.println(F(""));
  PC_SERIAL.println(F("  SM<n>    Set max speed (microsteps / 10000 s)"));
  PC_SERIAL.println(F("  SA<n>    Set max acceleration (microsteps / 100 s^2)"));
  PC_SERIAL.println(F(""));
  PC_SERIAL.println(F("STEP MODE vs FULL STEPS/SEC (200-step/rev motor):"));
  PC_SERIAL.println(F("  Mode 0 (Full):  SV1000000 = 100 full steps/sec"));
  PC_SERIAL.println(F("  Mode 1 (1/2):   SV1000000 =  50 full steps/sec  <--DEFAULT"));
  PC_SERIAL.println(F("  Mode 2 (1/4):   SV1000000 =  25 full steps/sec"));
  PC_SERIAL.println(F("  Mode 3 (1/8):   SV1000000 =12.5 full steps/sec"));
  PC_SERIAL.println(F(""));
  PC_SERIAL.println(F("EXAMPLES:"));
  PC_SERIAL.println(F("  1P5       Move valve 1 to position 5"));
  PC_SERIAL.println(F("  SO        Turn stepper on"));
  PC_SERIAL.println(F("  SV2000000 Run forward at 100 full steps/sec"));
  PC_SERIAL.println(F("  SV-2000000 Run reverse at 100 full steps/sec"));
  PC_SERIAL.println(F("  SV0       Decelerate to stop"));
  PC_SERIAL.println(F("  SS        Emergency stop (hold position)"));
  PC_SERIAL.println(F("  SP800     Move to 2 revolutions from home"));
  PC_SERIAL.println(F("============================================\n"));
}