/**
 * @file EEPROM.h
 * @brief Flash-emulated EEPROM for Air105 — Arduino-compatible API
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Uses the last 64 KB of the Air105's internal 4 MB SPI NOR flash
 * (SPI address 0x3F0000, memory-mapped at 0x013F0000) as a persistent
 * byte store with a standard Arduino EEPROM interface.
 *
 * Usage:
 *   EEPROM.begin(size);         // size <= 65536
 *   val = EEPROM.read(addr);
 *   EEPROM.write(addr, val);    // writes to RAM buffer only
 *   EEPROM.commit();            // erases sector + reprograms from buffer
 *   EEPROM.end();               // commit + free buffer
 *
 * Internal flash erase/program runs from RAM with interrupts disabled.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef EEPROM_h
#define EEPROM_h

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/*  EERef — proxy returned by operator[] for read/write syntax        */
/* ------------------------------------------------------------------ */
class EEPROMClass;

class EERef {
public:
    EERef(EEPROMClass &ee, int idx) : _ee(ee), _idx(idx) {}
    operator uint8_t() const;               /* read  */
    EERef &operator=(uint8_t val);          /* write */
    EERef &operator=(const EERef &ref);
    EERef &operator+=(uint8_t val);
    EERef &operator-=(uint8_t val);
    EERef &operator*=(uint8_t val);
    EERef &operator/=(uint8_t val);
    EERef &operator^=(uint8_t val);
    EERef &operator%=(uint8_t val);
    EERef &operator&=(uint8_t val);
    EERef &operator|=(uint8_t val);
private:
    EEPROMClass &_ee;
    int _idx;
};

/* ------------------------------------------------------------------ */
/*  EEPROMClass                                                       */
/* ------------------------------------------------------------------ */
class EEPROMClass {
public:
    EEPROMClass() : _data(nullptr), _size(0), _dirty(false) {}

    /**
     * @brief Initialise EEPROM emulation.
     * @param size  Number of bytes to emulate (1..65536, default 4096).
     * @return true on success.
     */
    bool begin(size_t size = 4096);

    /** Read one byte (returns 0xFF if uninitialised / out-of-range). */
    uint8_t read(int address);

    /** Write one byte to the RAM buffer (call commit() to persist). */
    void write(int address, uint8_t value);

    /** Write only if the value differs (avoids unnecessary commit). */
    void update(int address, uint8_t value);

    /** Erase flash sectors and reprogram from the RAM buffer. */
    bool commit(void);

    /** Commit and release the RAM buffer. */
    void end(void);

    /** Number of emulated bytes. */
    size_t length(void) const { return _size; }

    /** operator[] for convenient read/write. */
    EERef operator[](int idx) { return EERef(*this, idx); }

    /** Read an arbitrary type from EEPROM. */
    template<typename T>
    T &get(int address, T &t) {
        if (_data && address >= 0 && (size_t)(address + sizeof(T)) <= _size)
            memcpy(&t, _data + address, sizeof(T));
        return t;
    }

    /** Write an arbitrary type to the RAM buffer. */
    template<typename T>
    const T &put(int address, const T &t) {
        if (_data && address >= 0 && (size_t)(address + sizeof(T)) <= _size) {
            if (memcmp(_data + address, &t, sizeof(T)) != 0) {
                memcpy(_data + address, &t, sizeof(T));
                _dirty = true;
            }
        }
        return t;
    }

private:
    uint8_t *_data;     /* RAM shadow buffer                          */
    size_t   _size;     /* requested emulation size (≤ 65536)         */
    bool     _dirty;    /* true if buffer differs from flash          */
};

extern EEPROMClass EEPROM;

#endif /* EEPROM_h */
