/**
 * @file EEPROM.cpp
 * @brief Flash-emulated EEPROM for Air105 — implementation
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * The Air105's 4 MB internal flash is a SPI NOR die accessed through the
 * QSPI controller (QSPI_TypeDef at 0x400A2000).  The last 64 KB
 * (SPI address 0x3F0000, memory-mapped at 0x013F0000) is reserved for
 * EEPROM emulation (16 × 4 KB sectors).
 *
 * Flash programming protocol:
 *   1.  Disable interrupts (code must NOT fetch from flash).
 *   2.  Acquire QSPI from cache via FCU_CMD.ACCESS_REQ/ACK.
 *   3.  Send SPI NOR commands through FCU_CMD (Write-Enable, Sector-Erase,
 *       Page-Program, Read-Status) — all in single-wire SPI mode (BUS_MODE=0).
 *   4.  Release QSPI, invalidate cache, re-enable interrupts.
 *
 * ALL code that runs between acquire/release is placed in RAM via the
 * .RamFunc linker section (copied from flash to SRAM by startup).
 *
 * CMD_FORMAT encoding (4-bit field at FCU_CMD[7:4]):
 *   0x0  CMD only                (e.g. Write-Enable  0x06)
 *   0x1  CMD + read-data         (e.g. Read-Status   0x05)
 *   0x2  CMD + address           (e.g. Sector-Erase  0x20)
 *   0x6  CMD + address + write   (e.g. Page-Program  0x02)
 *
 * SPDX-License-Identifier: MIT
 */

#include "EEPROM.h"
#include "air105.h"
#include <stdlib.h>

/* ------------------------------------------------------------------ */
/*  Flash geometry                                                     */
/* ------------------------------------------------------------------ */
#define EEPROM_FLASH_SIZE    65536U   /* 64 KB reserved region       */
#define EEPROM_SECTOR_SIZE   4096U    /* SPI NOR erase granularity    */
#define EEPROM_PAGE_SIZE     256U     /* SPI NOR program granularity  */

/* SPI NOR address of the reserved EEPROM region (last 64 KB of 4 MB) */
#define EEPROM_SPI_ADDR      0x3F0000U

/* Memory-mapped address (for direct reads through the cache) */
#define EEPROM_MEM_ADDR      (AIR105_FLASH_BASE + EEPROM_SPI_ADDR)

/* ------------------------------------------------------------------ */
/*  SPI NOR opcodes                                                    */
/* ------------------------------------------------------------------ */
#define NOR_CMD_WREN         0x06U   /* Write Enable                  */
#define NOR_CMD_RDSR         0x05U   /* Read Status Register 1        */
#define NOR_CMD_SE           0x20U   /* Sector Erase (4 KB)           */
#define NOR_CMD_PP           0x02U   /* Page Program (≤256 B)         */

/* ------------------------------------------------------------------ */
/*  CMD_FORMAT helpers  (value already shifted into bits [7:4])        */
/* ------------------------------------------------------------------ */
#define FMT_CMD_ONLY         (0x0U << 4)   /* opcode only             */
#define FMT_CMD_RD           (0x1U << 4)   /* opcode + read data      */
#define FMT_CMD_ADDR         (0x2U << 4)   /* opcode + 3-byte addr    */
#define FMT_CMD_ADDR_WR      (0x6U << 4)   /* opcode + addr + write   */

/* ------------------------------------------------------------------ */
/*  Shorthand for register bits                                        */
/* ------------------------------------------------------------------ */
#define FCU_REQ              QUADSPI_FCU_CMD_ACCESS_REQ
#define FCU_ACK              QUADSPI_FCU_CMD_ACCESS_ACK
#define FCU_DONE             QUADSPI_FCU_CMD_DONE
#define FIFO_FLUSH           (QUADSPI_FIFO_CNTL_TFFH | QUADSPI_FIFO_CNTL_RFFH)
#define FIFO_TX_FULL         QUADSPI_FIFO_CNTL_TFFL

/* ------------------------------------------------------------------ */
/*  RAM-resident flash helpers                                         */
/*                                                                     */
/*  Every function between _qspiAcquire() and _qspiRelease() MUST     */
/*  live in SRAM so that the CPU never tries to fetch from flash       */
/*  while we hold the QSPI bus.                                        */
/* ------------------------------------------------------------------ */
#define RAMFUNC __attribute__((section(".RamFunc"), noinline, long_call))

/* Build a FCU_CMD word: opcode in [31:24], format in [7:4], keep REQ */
#define FCU_WORD(opcode, fmt)  \
    (((uint32_t)(opcode) << 24) | (uint32_t)(fmt) | FCU_REQ)

/* ---- acquire / release ------------------------------------------ */

static RAMFUNC void _qspiAcquire(void)
{
    QSPI->FCU_CMD = FCU_REQ;
    while (!(QSPI->FCU_CMD & FCU_ACK)) { /* spin */ }
}

static RAMFUNC void _qspiRelease(void)
{
    QSPI->FCU_CMD = 0;   /* clear ACCESS_REQ → cache resumes */
}

/* ---- atomic SPI NOR commands (must be called while acquired) ---- */

static RAMFUNC void _norWriteEnable(void)
{
    QSPI->FIFO_CNTL = FIFO_FLUSH;
    QSPI->FCU_CMD   = FCU_WORD(NOR_CMD_WREN, FMT_CMD_ONLY);
    while (!(QSPI->FCU_CMD & FCU_DONE)) { /* spin */ }
}

static RAMFUNC uint8_t _norReadStatus(void)
{
    QSPI->FIFO_CNTL = FIFO_FLUSH;
    QSPI->BYTE_NUM  = (1U << 16);        /* RD_BYTE = 1               */
    QSPI->FCU_CMD   = FCU_WORD(NOR_CMD_RDSR, FMT_CMD_RD);
    while (!(QSPI->FCU_CMD & FCU_DONE)) { /* spin */ }
    return (uint8_t)(QSPI->RD_FIFO & 0xFFU);
}

static RAMFUNC void _norWaitReady(void)
{
    while (_norReadStatus() & 0x01U) { /* WIP bit */ }
}

static RAMFUNC void _norEraseSector(uint32_t spiAddr)
{
    _norWriteEnable();
    QSPI->FIFO_CNTL = FIFO_FLUSH;
    QSPI->ADDRES    = (spiAddr << 8);    /* addr in [31:8], M8=0      */
    QSPI->FCU_CMD   = FCU_WORD(NOR_CMD_SE, FMT_CMD_ADDR);
    while (!(QSPI->FCU_CMD & FCU_DONE)) { /* spin */ }
    _norWaitReady();                       /* erase takes ~60-400 ms   */
}

static RAMFUNC void _norProgramPage(uint32_t spiAddr,
                                    const uint8_t *src,
                                    uint32_t len)
{
    _norWriteEnable();

    QSPI->FIFO_CNTL = FIFO_FLUSH;
    QSPI->ADDRES    = (spiAddr << 8);
    QSPI->BYTE_NUM  = len;               /* WR_BYTE = len             */
    QSPI->FCU_CMD   = FCU_WORD(NOR_CMD_PP, FMT_CMD_ADDR_WR);

    /* Stream data into TX FIFO — 32-bit writes, up to 4 bytes each */
    uint32_t i = 0;
    while (i < len) {
        /* Wait for space in the 16-entry TX FIFO */
        while (QSPI->FIFO_CNTL & FIFO_TX_FULL) { /* spin */ }

        uint32_t word = 0;
        uint32_t chunk = (len - i);
        if (chunk > 4) chunk = 4;
        for (uint32_t j = 0; j < chunk; j++)
            word |= ((uint32_t)src[i + j]) << (j * 8);
        QSPI->WR_FIFO = word;
        i += chunk;
    }

    while (!(QSPI->FCU_CMD & FCU_DONE)) { /* spin */ }
    _norWaitReady();                       /* program takes ~0.4-3 ms  */
}

/* Maximum number of 4 KB sectors in the 64 KB EEPROM region */
#define EEPROM_MAX_SECTORS   (EEPROM_FLASH_SIZE / EEPROM_SECTOR_SIZE)  /* 16 */

/* ---- top-level commit (called with IRQs disabled) --------------- */
/*  sectorMask: bit N = 1 means sector N needs erase + reprogram     */

static RAMFUNC bool _flashCommitRAM(uint32_t spiAddr,
                                    const uint8_t *buf,
                                    uint32_t len,
                                    uint32_t sectorMask)
{
    _qspiAcquire();

    uint32_t nSectors = (len + EEPROM_SECTOR_SIZE - 1) / EEPROM_SECTOR_SIZE;

    for (uint32_t s = 0; s < nSectors; s++) {
        if (!(sectorMask & (1U << s)))
            continue;   /* this sector is unchanged — skip */

        uint32_t secOff  = s * EEPROM_SECTOR_SIZE;
        uint32_t secAddr = spiAddr + secOff;

        /* 1. Erase this 4 KB sector */
        _norEraseSector(secAddr);

        /* 2. Program the sector in 256-byte pages */
        uint32_t secEnd = secOff + EEPROM_SECTOR_SIZE;
        if (secEnd > len) secEnd = len;

        for (uint32_t off = secOff; off < secEnd; off += EEPROM_PAGE_SIZE) {
            uint32_t plen = EEPROM_PAGE_SIZE;
            if (off + plen > secEnd) plen = secEnd - off;

            /* Skip fully-erased pages (all 0xFF) — saves time */
            bool blank = true;
            for (uint32_t k = 0; k < plen; k++) {
                if (buf[off + k] != 0xFF) { blank = false; break; }
            }
            if (!blank)
                _norProgramPage(spiAddr + off, buf + off, plen);
        }
    }

    _qspiRelease();
    return true;
}

/* ================================================================== */
/*  EERef                                                              */
/* ================================================================== */

EERef::operator uint8_t() const            { return _ee.read(_idx); }
EERef &EERef::operator=(uint8_t val)       { _ee.write(_idx, val); return *this; }
EERef &EERef::operator=(const EERef &ref)  { return *this = (uint8_t)ref; }
EERef &EERef::operator+=(uint8_t v)        { return *this = (uint8_t)*this + v; }
EERef &EERef::operator-=(uint8_t v)        { return *this = (uint8_t)*this - v; }
EERef &EERef::operator*=(uint8_t v)        { return *this = (uint8_t)*this * v; }
EERef &EERef::operator/=(uint8_t v)        { return *this = (uint8_t)*this / v; }
EERef &EERef::operator^=(uint8_t v)        { return *this = (uint8_t)*this ^ v; }
EERef &EERef::operator%=(uint8_t v)        { return *this = (uint8_t)*this % v; }
EERef &EERef::operator&=(uint8_t v)        { return *this = (uint8_t)*this & v; }
EERef &EERef::operator|=(uint8_t v)        { return *this = (uint8_t)*this | v; }

/* ================================================================== */
/*  EEPROMClass                                                        */
/* ================================================================== */

bool EEPROMClass::begin(size_t size)
{
    if (size == 0 || size > EEPROM_FLASH_SIZE)
        size = EEPROM_FLASH_SIZE;

    if (_data) free(_data);

    _data = (uint8_t *)malloc(size);
    if (!_data) return false;

    _size  = size;
    _dirty = false;

    /* Copy current flash content into the RAM buffer */
    const volatile uint8_t *flash = (const volatile uint8_t *)EEPROM_MEM_ADDR;
    for (size_t i = 0; i < size; i++)
        _data[i] = flash[i];

    return true;
}

uint8_t EEPROMClass::read(int address)
{
    if (!_data || address < 0 || (size_t)address >= _size)
        return 0xFF;
    return _data[address];
}

void EEPROMClass::write(int address, uint8_t value)
{
    if (!_data || address < 0 || (size_t)address >= _size)
        return;
    if (_data[address] != value) {
        _data[address] = value;
        _dirty = true;
    }
}

void EEPROMClass::update(int address, uint8_t value)
{
    write(address, value);   /* write() already checks for change */
}

bool EEPROMClass::commit(void)
{
    if (!_data || !_dirty)
        return true;           /* nothing to do */

    /* Compare each 4 KB sector against flash to find which changed.
       This runs with IRQs still enabled (reading via memory-mapped flash). */
    const volatile uint8_t *flash = (const volatile uint8_t *)EEPROM_MEM_ADDR;
    uint32_t sectorMask = 0;
    uint32_t nSectors = (_size + EEPROM_SECTOR_SIZE - 1) / EEPROM_SECTOR_SIZE;
    for (uint32_t s = 0; s < nSectors; s++) {
        uint32_t base = s * EEPROM_SECTOR_SIZE;
        uint32_t end  = base + EEPROM_SECTOR_SIZE;
        if (end > _size) end = _size;
        for (uint32_t i = base; i < end; i++) {
            if (_data[i] != flash[i]) {
                sectorMask |= (1U << s);
                break;
            }
        }
    }

    if (sectorMask == 0) {
        _dirty = false;
        return true;           /* all sectors already match */
    }

    /* Disable interrupts — no code may fetch from flash */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DSB();

    bool ok = _flashCommitRAM(EEPROM_SPI_ADDR, _data, _size, sectorMask);

    /* Re-enable interrupts */
    __DSB();
    __ISB();
    __set_PRIMASK(primask);

    if (ok) {
        /* Verify: compare RAM buffer against memory-mapped flash */
        const volatile uint8_t *flash =
            (const volatile uint8_t *)EEPROM_MEM_ADDR;
        for (size_t i = 0; i < _size; i++) {
            if (flash[i] != _data[i]) {
                ok = false;
                break;
            }
        }
    }

    if (ok) _dirty = false;
    return ok;
}

void EEPROMClass::end(void)
{
    if (_data) {
        commit();
        free(_data);
        _data  = nullptr;
        _size  = 0;
        _dirty = false;
    }
}

/* Global instance */
EEPROMClass EEPROM;
