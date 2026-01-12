/*
  eFlesh Board Code
  By: Venkatesh P
  Date: August 1, 2025
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Credits: Adapted from (ReSkin (https://reskin.dev) and AnySkin (https://any-skin.github.io) repos) - maintained by Raunaq Bhirangi, Tess Hellebrekers
  Library: Heavily based on original MLX90393 library from Theodore Yapo (https://github.com/tedyapo/arduino-MLX90393)
  Library: https://github.com/tesshellebrekers/arduino-MLX90393  (required)
*/

#include <Wire.h>
#include <MLX90393.h>

#define Serial SERIAL_PORT_USBVIRTUAL  // use default Serial if your board doesn't define this

// MLX90393 objects and data buffers
static const uint8_t NUM_SENSORS = 5;
MLX90393 mlx[NUM_SENSORS];
MLX90393::txyz dataBuf[NUM_SENSORS];

// Known sequences
const uint8_t TARGETS_ALL_CONSEC[NUM_SENSORS] = {0x0C, 0x0D, 0x0E, 0x0F, 0x10};
const uint8_t TARGETS_WHITE_SET[NUM_SENSORS]   = {0x0C, 0x10, 0x11, 0x12, 0x13};

// Forward decls
void scanI2C(uint8_t* found, uint8_t& count);
void chooseOrderedAddresses(const uint8_t* found, uint8_t count, uint8_t* ordered, uint8_t& orderedCount);
bool hasExactSet(const uint8_t* found, uint8_t count, const uint8_t* pattern);
void sortAscending(uint8_t* arr, uint8_t n);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(5); }

  Wire.begin();
  Wire.setClock(400000);
  delay(10);

  uint8_t found[16] = {0};
  uint8_t foundCount = 0;
  scanI2C(found, foundCount);

  Serial.println(F("I2C scan complete."));
  Serial.print(F("Found MLX candidates: "));
  for (uint8_t i = 0; i < foundCount; ++i) {
    Serial.print("0x"); Serial.print(found[i], HEX); Serial.print(" ");
  }
  Serial.println();

  uint8_t ordered[NUM_SENSORS] = {0};
  uint8_t orderedCount = 0;
  chooseOrderedAddresses(found, foundCount, ordered, orderedCount);

  if (orderedCount != NUM_SENSORS) {
    Serial.println(F("[WARN] Did not find exactly 5 target MLX addresses. Proceeding with what was found."));
  }

  // Initialize sensors in the chosen order
  for (uint8_t i = 0; i < orderedCount; ++i) {
    byte status = mlx[i].begin(ordered[i], -1, Wire);
    Serial.print(F("Init MLX[")); Serial.print(i); Serial.print(F("] @0x"));
    Serial.print(ordered[i], HEX);
    Serial.print(F(" status=")); Serial.println(status, HEX);

    // Configs
    mlx[i].setGainSel(0x1);
    mlx[i].setResolution(0x2, 0x2, 0x2);
    mlx[i].setDigitalFiltering(0x4);

    // Start burst mode (Temp + X + Y + Z) = 0xF
    mlx[i].startBurst(0xF);
  }

  // If fewer than 5 were found, zero-fill the remaining buffers so the binary record stays a fixed length if you need it
  for (uint8_t i = orderedCount; i < NUM_SENSORS; ++i) {
    dataBuf[i] = {0, 0, 0, 0};
  }

  Serial.print(F("Streaming order: "));
  for (uint8_t i = 0; i < orderedCount; ++i) {
    Serial.print("0x"); Serial.print(ordered[i], HEX); Serial.print(" ");
  }
  Serial.println();
}

void loop() {
  // Read in the same order the sensors were initialized
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    mlx[i].readBurstData(dataBuf[i]);  // if that index wasn't initialized (no device), the buffer remains whatever it was
  }

  // Write binary records in-order
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    Serial.write((uint8_t*)&dataBuf[i], sizeof(dataBuf[i]));
  }
  Serial.println();

  // adjust for desired sample rate
  delayMicroseconds(500);
}

// Scan all I2C addresses to find MLX90393 sensors
void scanI2C(uint8_t* found, uint8_t& count) {
  count = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      found[count++] = addr;
      if (count >= 16) break;
    }
  }
}

void chooseOrderedAddresses(const uint8_t* found, uint8_t count, uint8_t* ordered, uint8_t& orderedCount) {
  orderedCount = 0;

  // Copy to a temp for set checks (exact set of 5)
  if (count >= NUM_SENSORS) {
    // Check for the two exact sets
    if (hasExactSet(found, count, TARGETS_ALL_CONSEC)) {
      for (uint8_t i = 0; i < NUM_SENSORS; ++i) ordered[i] = TARGETS_ALL_CONSEC[i];
      orderedCount = NUM_SENSORS;
      return;
    }
    if (hasExactSet(found, count, TARGETS_WHITE_SET)) {
      // custom order: C,11,12,13,10
      ordered[0] = 0x0C; ordered[1] = 0x11; ordered[2] = 0x12; ordered[3] = 0x13; ordered[4] = 0x10;
      orderedCount = NUM_SENSORS;
      return;
    }
  }

  uint8_t tmp[16];
  uint8_t n = count;
  if (n > 16) n = 16;
  for (uint8_t i = 0; i < n; ++i) tmp[i] = found[i];
  sortAscending(tmp, n);

  orderedCount = (n >= NUM_SENSORS) ? NUM_SENSORS : n;
  for (uint8_t i = 0; i < orderedCount; ++i) ordered[i] = tmp[i];
}

bool hasExactSet(const uint8_t* found, uint8_t count, const uint8_t* pattern) {
  uint8_t hits = 0;
  for (uint8_t p = 0; p < NUM_SENSORS; ++p) {
    bool present = false;
    for (uint8_t i = 0; i < count; ++i) {
      if (found[i] == pattern[p]) { present = true; break; }
    }
    if (present) ++hits;
  }
  return (hits == NUM_SENSORS);
}

void sortAscending(uint8_t* arr, uint8_t n) {
  for (uint8_t i = 0; i < n; ++i) {
    for (uint8_t j = i + 1; j < n; ++j) {
      if (arr[j] < arr[i]) {
        uint8_t t = arr[i]; arr[i] = arr[j]; arr[j] = t;
      }
    }
  }
}
