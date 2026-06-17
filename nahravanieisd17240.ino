#include <SPI.h>

// SPI: D13=SCK, D12=MISO, D11=MOSI, D10=SS
#define ISD_SS  10
// INT z ISD17240 (aktivne LOW, open-drain) -> pripojit na D6 + vnutorny pull-up
#define ISD_INT 6

const int btnRec  = 2;
const int btnPlay = 3;
const int btnRedo = 4;
const int btnTrim = 5;

const uint32_t SPI_SPEED = 10000; // max 1 MHz podla datasheet
const uint16_t MEM_START = 0x196; // fixna obsadena cast 0x010-0x195 sa nikdy nemaže
const uint16_t MEM_END   = 0x78F; // ISD17240 @ 8 kHz
const uint8_t  SR1_RDY   = 0x01;
const uint8_t  SR1_REC   = 0x08;
const uint8_t  SR0_CMD_ERR = 0x01;
const uint8_t  SR0_PU    = 0x04;
const uint8_t  DEVID_ISD17240 = 0xE0; // CHIPID 11100 v bitoch 7:3

uint16_t currentAddress = MEM_START;
uint16_t history[100];
uint16_t endAddresses[100];
int recordCount = 0;
bool isRecording = false;
uint16_t recordingStart = MEM_START;

bool lastRecState  = HIGH;
bool lastPlayState = HIGH;
bool lastTrimState = HIGH;
bool lastRedoState = HIGH;
unsigned long lastButtonChange = 0;
const unsigned long DEBOUNCE_MS = 40;

struct Status {
  uint8_t sr0_lo;
  uint8_t sr0_hi;
  uint8_t sr1;
};

uint8_t xfer(uint8_t data) {
  delayMicroseconds(50);
  uint8_t r = SPI.transfer(data);
  delayMicroseconds(50);
  return r;
}

void selectISD() {
  digitalWrite(ISD_SS, HIGH);
  delayMicroseconds(100);
  digitalWrite(ISD_SS, LOW);
  delayMicroseconds(100);
}

void deselectISD() {
  delayMicroseconds(100);
  digitalWrite(ISD_SS, HIGH);
  delayMicroseconds(100);
}

void sendSimpleCommand(uint8_t opcode) {
  selectISD();
  xfer(opcode);
  deselectISD();
}

void powerUp() { sendSimpleCommand(0x01); }
void stopISD() { sendSimpleCommand(0x02); }
void resetISD() { sendSimpleCommand(0x03); }
void clrInt() { sendSimpleCommand(0x04); }

Status readStatus() {
  selectISD();
  xfer(0x05);
  Status s;
  s.sr0_lo = xfer(0x00);
  s.sr0_hi = xfer(0x00);
  s.sr1    = xfer(0x00);
  deselectISD();
  return s;
}

uint16_t readCurrentRowAddress() {
  Status s = readStatus();
  return ((uint16_t)s.sr0_hi << 3) | ((s.sr0_lo >> 5) & 0x07);
}

uint8_t readDeviceId() {
  selectISD();
  xfer(0x09);
  uint8_t id = xfer(0x00);
  deselectISD();
  return id;
}

uint16_t readAddressPointer(uint8_t opcode) {
  selectISD();
  xfer(opcode);
  uint8_t lo = xfer(0x00);
  uint8_t hi = xfer(0x00) & 0x07;
  xfer(0x00);
  deselectISD();
  return ((uint16_t)hi << 8) | lo;
}

uint16_t readRecPointer() {
  return readAddressPointer(0x08);
}

bool cmdError(const Status &s) {
  return (s.sr0_lo & SR0_CMD_ERR) != 0;
}

bool poweredUp(const Status &s) {
  return (s.sr0_lo & SR0_PU) != 0;
}

bool readyNow() {
  Status s = readStatus();
  return (s.sr1 & SR1_RDY) != 0;
}

void dumpStatus(const __FlashStringHelper *label) {
  Status s = readStatus();
  Serial.print(label);
  Serial.print(F(" SR0=0x"));
  Serial.print(s.sr0_hi, HEX);
  Serial.print(s.sr0_lo, HEX);
  Serial.print(F(" SR1=0x"));
  Serial.print(s.sr1, HEX);
  Serial.print(F(" INT="));
  Serial.print(digitalRead(ISD_INT) == LOW ? F("L") : F("H"));
  Serial.print(F(" PU="));
  Serial.print(poweredUp(s) ? F("1") : F("0"));
  Serial.print(F(" CMD_ERR="));
  Serial.print(cmdError(s) ? F("1") : F("0"));
  Serial.print(F(" RDY="));
  Serial.println((s.sr1 & SR1_RDY) ? F("1") : F("0"));
}

bool waitForReady(unsigned long timeoutMs = 6000) {
  unsigned long startWait = millis();
  while (millis() - startWait < timeoutMs) {
    if (readyNow()) return true;
    delay(2);
  }
  Serial.println(F("WARN: ISD timeout RDY"));
  dumpStatus(F("  stav"));
  return false;
}

// Datasheet 11.7: po SET_REC/SET_PLAY/STOP sledovat INT (aktivne LOW)
bool waitForIntLow(unsigned long timeoutMs = 5000) {
  unsigned long startWait = millis();
  while (digitalRead(ISD_INT) == HIGH && millis() - startWait < timeoutMs) {
    delay(1);
  }
  return digitalRead(ISD_INT) == LOW;
}

// Pred odoslanim SET_* prikazu musi byt RDY=1
bool ensureDeviceReady() {
  Status s = readStatus();
  if ((s.sr1 & SR1_RDY) && !cmdError(s)) return true;

  clrInt();
  if (waitForReady(4000)) return true;

  stopISD();
  clrInt();
  return waitForReady(4000);
}

// Po SET_REC/SET_PLAY: INT klesne po prijati prikazu, alebo SR1 ukaze REC/PLAY
bool waitForSetCommandAccepted(unsigned long timeoutMs = 5000) {
  unsigned long startWait = millis();
  while (millis() - startWait < timeoutMs) {
    if (digitalRead(ISD_INT) == LOW) return true;
    Status s = readStatus();
    if ((s.sr1 & (SR1_REC | 0x04)) != 0 && !cmdError(s)) return true;
    delay(1);
  }
  Serial.println(F("WARN: ISD timeout INT pri SET"));
  dumpStatus(F("  stav"));
  return false;
}

// Po STOP/ERASE: INT klesne po dokonceni, potom RDY=1
bool waitForOperationEnd(unsigned long timeoutMs = 20000) {
  if (!waitForIntLow(timeoutMs)) {
    Serial.println(F("WARN: ISD timeout INT"));
    dumpStatus(F("  stav"));
  }
  return waitForReady(3000);
}

void setupAPC_ANA_AUD() {
  // APC=0x0440: SPI_FT, AUD vystup, max hlasitost, AnaIn cesta
  selectISD();
  xfer(0x65);
  xfer(0x40);
  xfer(0x04);
  deselectISD();
}

void sendSetCommand(uint8_t opcode, uint16_t start, uint16_t end) {
  start = constrain(start, MEM_START, MEM_END);
  end = constrain(end, start, MEM_END);

  selectISD();
  xfer(opcode);
  xfer(0x00);
  xfer(start & 0xFF);
  xfer((start >> 8) & 0x07);
  xfer(end & 0xFF);
  xfer((end >> 8) & 0x07);
  xfer(0x00);
  deselectISD();
}

void setPlay(uint16_t start, uint16_t end) {
  sendSetCommand(0x80, start, end);
}

void setRec(uint16_t start, uint16_t end) {
  sendSetCommand(0x81, start, end);
}

void setErase(uint16_t start, uint16_t end) {
  if (start < MEM_START) start = MEM_START;
  if (end < start) return;
  sendSetCommand(0x82, start, end);
}

// Maze len dynamicku cast (0x196-0x78F), fixna pamat 0x010-0x195 zostane
void eraseDynamicRegion() {
  setErase(MEM_START, MEM_END);
}

bool pressedEdge(int pin, bool &lastState) {
  bool state = digitalRead(pin);
  bool edge = (lastState == HIGH && state == LOW &&
               (millis() - lastButtonChange) > DEBOUNCE_MS);
  if (state != lastState) {
    lastState = state;
    lastButtonChange = millis();
  }
  return edge;
}

void printRecordingAddress(uint8_t index, uint16_t start, uint16_t end) {
  Serial.print(F("Nahravka "));
  Serial.print(index);
  Serial.print(F(" start=0x"));
  Serial.print(start, HEX);
  Serial.print(F(" koniec=0x"));
  Serial.println(end, HEX);
}

void startRecording() {
  if (recordCount >= 100) {
    Serial.println(F("REC: plny zoznam"));
    return;
  }
  if (currentAddress >= MEM_END) {
    Serial.println(F("REC: koniec pamate"));
    return;
  }

  recordingStart = currentAddress;
  history[recordCount] = recordingStart;
  Serial.print(F("REC START addr=0x"));
  Serial.println(recordingStart, HEX);

  if (!ensureDeviceReady()) {
    Serial.println(F("REC: cip nie je pripraveny"));
    dumpStatus(F("  stav"));
    return;
  }

  clrInt();
  setRec(recordingStart, MEM_END);
  if (!waitForSetCommandAccepted()) {
    Serial.println(F("REC: chyba start"));
    return;
  }
  clrInt();
  isRecording = true;
  Serial.println(F("REC OK"));
}

void finishRecording() {
  stopISD();
  waitForOperationEnd();

  uint16_t rawEnd = readRecPointer();
  if (rawEnd <= recordingStart || rawEnd > MEM_END) {
    rawEnd = recordingStart + 1;
  }

  uint16_t playEnd = constrain(rawEnd, recordingStart, MEM_END);

  endAddresses[recordCount] = playEnd;
  currentAddress = playEnd + 1;
  if (currentAddress > MEM_END) currentAddress = MEM_END;

  recordCount++;
  isRecording = false;
  clrInt();

  Serial.print(F("REC STOP raw=0x"));
  Serial.print(rawEnd, HEX);
  Serial.print(F(" "));
  printRecordingAddress(recordCount, recordingStart, playEnd);
}

void eraseAndRedoLast() {
  if (recordCount <= 0 || isRecording) return;

  uint16_t start = history[recordCount - 1];
  uint16_t end   = endAddresses[recordCount - 1];

  Serial.print(F("REDO ERASE 0x"));
  Serial.print(start, HEX);
  Serial.print(F("-0x"));
  Serial.println(end, HEX);

  setErase(start, end);
  waitForOperationEnd(20000);
  clrInt();

  recordCount--;
  currentAddress = start;

  Serial.print(F("REDO addr=0x"));
  Serial.println(currentAddress, HEX);
}

void trimLastRecording() {
  if (recordCount <= 0 || isRecording) return;

  uint16_t start    = history[recordCount - 1];
  uint16_t oldEnd   = endAddresses[recordCount - 1];
  if (oldEnd <= start) return;

  uint16_t newEnd = oldEnd - 1;
  uint16_t eraseFrom = newEnd + 1;

  Serial.print(F("TRIM ERASE 0x"));
  Serial.print(eraseFrom, HEX);
  Serial.print(F("-0x"));
  Serial.println(oldEnd, HEX);

  setErase(eraseFrom, oldEnd);
  waitForOperationEnd(20000);
  clrInt();

  endAddresses[recordCount - 1] = newEnd;
  currentAddress = newEnd + 1;

  Serial.print(F("TRIM koniec=0x"));
  Serial.println(newEnd, HEX);

  setPlay(start, newEnd);
  waitForSetCommandAccepted();
  clrInt();
}

void setup() {
  Serial.begin(9600);
  pinMode(ISD_SS, OUTPUT);
  digitalWrite(ISD_SS, HIGH);
  pinMode(ISD_INT, INPUT_PULLUP);

  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_SPEED, LSBFIRST, SPI_MODE3));

  pinMode(btnRec, INPUT_PULLUP);
  pinMode(btnPlay, INPUT_PULLUP);
  pinMode(btnRedo, INPUT_PULLUP);
  pinMode(btnTrim, INPUT_PULLUP);

  Serial.println(F("--- START SYSTEMU ---"));
  resetISD();
  delay(10);
  powerUp();
  waitForReady();

  uint8_t devId = readDeviceId();
  Serial.print(F("DEVID=0x"));
  Serial.println(devId, HEX);
  if ((devId & 0xF8) != DEVID_ISD17240) {
    Serial.println(F("WARN: neocekavany chip (ocakavane ISD17240)"));
  }

  if (digitalRead(btnRec) == LOW) {
    Serial.println(F("MAZANIE DYNAMICKEJ PAMATE 0x196-0x78F..."));
    eraseDynamicRegion();
    waitForOperationEnd(20000);
    clrInt();
    currentAddress = MEM_START;
    recordCount = 0;
    Serial.println(F("MAZANIE OK"));
    while (digitalRead(btnRec) == LOW) delay(5);
  }

  setupAPC_ANA_AUD();
  waitForReady();
  stopISD();
  clrInt();

  Serial.print(F("Pamat: 0x"));
  Serial.print(MEM_START, HEX);
  Serial.print(F("-0x"));
  Serial.println(MEM_END, HEX);
  Serial.println(F("INT pin: pripoj ISD INT -> Arduino D6"));
  Serial.println(F("--- SYSTEM PRIPRAVENY ---"));
}

void loop() {
  if (pressedEdge(btnRec, lastRecState)) {
    if (isRecording) {
      finishRecording();
    } else {
      startRecording();
    }
  }

  if (!isRecording && pressedEdge(btnTrim, lastTrimState)) {
    trimLastRecording();
  }

  if (!isRecording && pressedEdge(btnPlay, lastPlayState)) {
    if (recordCount > 0) {
      uint16_t s = history[recordCount - 1];
      uint16_t e = endAddresses[recordCount - 1];
      Serial.print(F("PLAY 0x"));
      Serial.print(s, HEX);
      Serial.print(F("-0x"));
      Serial.println(e, HEX);

      setPlay(s, e);
      waitForSetCommandAccepted();
      clrInt();
      Serial.println(F("PLAY OK"));
    }
  }

  if (!isRecording && digitalRead(btnRedo) == LOW && lastRedoState == HIGH) {
    lastRedoState = LOW;
    unsigned long pressStartTime = millis();
    bool longPressTriggered = false;

    while (digitalRead(btnRedo) == LOW) {
      if (!longPressTriggered && (millis() - pressStartTime > 3000)) {
        Serial.println(F("--- ZOZNAM ADRES ---"));
        for (int i = 0; i < recordCount; i++) {
          printRecordingAddress(i + 1, history[i], endAddresses[i]);
        }
        Serial.println(F("-------------------"));
        longPressTriggered = true;
      }
      delay(10);
    }

    if (!longPressTriggered) {
      eraseAndRedoLast();
    }
    lastButtonChange = millis();
  }
  if (digitalRead(btnRedo) == HIGH) lastRedoState = HIGH;
}
