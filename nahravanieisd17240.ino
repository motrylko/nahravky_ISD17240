#include <SPI.h>

// SPI: D13=SCK, D12=MISO, D11=MOSI, D10=SS
#define ISD_SS  10
// INT z ISD17240 (aktivne LOW, open-drain) -> pripojit na D6 + vnutorny pull-up
#define ISD_INT 6

const int btnRec  = 2;
const int btnPlay = 3;
const int btnRedo = 4;
const int btnTrim = 5;

const uint32_t SPI_SPEED = 1000000; // overene s ISD1700 kniznicou; max 1 MHz podla datasheet
const uint16_t MEM_START = 0x196; // fixna obsadena cast 0x010-0x195 sa nikdy nemaže
const uint16_t MEM_END   = 0x3FF; // horna adresa pouzita funkcnym ISD1700 sketchom
const uint8_t  SR1_RDY   = 0x01;
const uint8_t  DEVID_ISD17240 = 0xE0; // CHIPID 11100 v bitoch 7:3
const unsigned long ROW_DURATION_MS = 125; // ponechany vypocet dlzky podla casu

uint16_t currentAddress = MEM_START;
uint16_t history[100];
uint16_t endAddresses[100];
int recordCount = 0;
bool isRecording = false;
uint16_t recordingStart = MEM_START;
unsigned long recordingStartMillis = 0;

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
  xfer(0x00);
  deselectISD();
}

void powerUp() { sendSimpleCommand(0x01); }
void stopISD() { sendSimpleCommand(0x02); }
void resetISD() { sendSimpleCommand(0x03); }
void clrInt() { sendSimpleCommand(0x04); }

Status readStatus() {
  selectISD();
  Status s;
  s.sr0_lo = xfer(0x05);
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
  xfer(0x00);
  uint8_t id = xfer(0x00);
  deselectISD();
  return id;
}

bool readyNow() {
  Status s = readStatus();
  return (s.sr1 & SR1_RDY) != 0;
}

bool waitForReady(unsigned long timeoutMs = 6000) {
  unsigned long startWait = millis();
  while (millis() - startWait < timeoutMs) {
    if (readyNow()) return true;
    delay(2);
  }
  Serial.println(F("WARN: ISD timeout RDY"));
  return false;
}

// INT=LOW signalizuje dokoncenu operaciu (po pripojeni INT na D6)
bool waitForOperation(unsigned long timeoutMs = 10000) {
  unsigned long startWait = millis();
  while (digitalRead(ISD_INT) == HIGH && millis() - startWait < 500) {
    delay(1);
  }
  if (digitalRead(ISD_INT) == LOW) {
    while (millis() - startWait < timeoutMs) {
      if (readyNow()) return true;
      delay(2);
    }
    Serial.println(F("WARN: ISD timeout INT+RDY"));
    return false;
  }
  return waitForReady(timeoutMs);
}

void setupAPC_ANA_AUD() {
  // SET_APC = 0x000B je rovnake nastavenie ako vo fungujucom teste ISD1700.
  // Posielame ho priamo rovnakou sekvenciou CS/SPI ako referencny sketch,
  // aby sa audio cesta ANA -> AUD/SPK otvorila hned po zapnuti aj pocas REC.
  digitalWrite(ISD_SS, LOW);
  delayMicroseconds(20);
  SPI.transfer(0x65);
  SPI.transfer(0x0B);
  SPI.transfer(0x00);
  digitalWrite(ISD_SS, HIGH);
}

void sendSetCommand(uint8_t opcode, uint16_t start, uint16_t end) {
  start = constrain(start, MEM_START, MEM_END);
  end = constrain(end, start, MEM_END);

  // Rovnaka priama 7-bajtova SET_PLAY/SET_REC/SET_ERASE sekvencia ako vo funkcnom kode.
  // Nepouzivame tu selectISD()/xfer(), aby CS a medzibajtove casovanie zostalo co najblizsie referencii.
  digitalWrite(ISD_SS, LOW);
  delayMicroseconds(20);
  SPI.transfer(opcode);
  SPI.transfer(0x00);
  SPI.transfer(start & 0xFF);
  SPI.transfer((start >> 8) & 0xFF);
  SPI.transfer(end & 0xFF);
  SPI.transfer((end >> 8) & 0xFF);
  SPI.transfer(0x00);
  digitalWrite(ISD_SS, HIGH);
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
  recordingStartMillis = millis();
  history[recordCount] = recordingStart;
  Serial.print(F("REC START addr=0x"));
  Serial.println(recordingStart, HEX);

  setupAPC_ANA_AUD();
  waitForReady();
  setRec(recordingStart, MEM_END);
  // Pri SET_REC nesmieme cakat na koniec operacie: cip je BUSY pocas celeho nahravania.
  // Pauza 300 ms je prevzata z funkcneho sketchu po prikaze REC.
  delay(300);
  isRecording = true;
}

void finishRecording() {
  stopISD();
  waitForOperation();

  unsigned long recordedMs = millis() - recordingStartMillis;
  uint16_t rows = (recordedMs + ROW_DURATION_MS - 1) / ROW_DURATION_MS;
  if (rows < 1) rows = 1;
  uint16_t rawEnd = constrain(recordingStart + rows, recordingStart + 1, MEM_END);

  uint16_t playEnd = rawEnd;

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
  waitForOperation(20000);
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
  waitForOperation(20000);
  clrInt();

  endAddresses[recordCount - 1] = newEnd;
  currentAddress = newEnd + 1;

  Serial.print(F("TRIM koniec=0x"));
  Serial.println(newEnd, HEX);

  setPlay(start, newEnd);
  waitForOperation();
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
  delay(50); // TPUD podla datasheet @ 8 kHz
  waitForReady();
  setupAPC_ANA_AUD();
  waitForReady();

  uint8_t devId = readDeviceId();
  Serial.print(F("DEVID=0x"));
  Serial.println(devId, HEX);
  if ((devId & 0xF8) != DEVID_ISD17240) {
    Serial.println(F("WARN: neocekavany chip (ocakavane ISD17240)"));
  }

  if (digitalRead(btnRec) == LOW) {
    Serial.println(F("MAZANIE DYNAMICKEJ PAMATE 0x196-0x3FF..."));
    eraseDynamicRegion();
    waitForOperation(20000);
    clrInt();
    currentAddress = MEM_START;
    recordCount = 0;
    Serial.println(F("MAZANIE OK"));
    while (digitalRead(btnRec) == LOW) delay(5);
  }

  setupAPC_ANA_AUD();
  waitForReady();
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

      setupAPC_ANA_AUD();
      waitForReady();
      setPlay(s, e);
      waitForOperation();
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
