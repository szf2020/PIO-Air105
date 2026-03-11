/**
 * @file SPI.cpp
 * @brief Arduino SPI implementation for Air105
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Direct register-level driver for the DesignWare SSI controller.
 * Master-mode only, polling-based (no IRQ / DMA).
 *
 * Supported instances (all SPI_TypeDef based):
 *   SPIM0 — PB12=SCK, PB13=CS, PB14=MOSI, PB15=MISO (AF0)
 *   SPIM1 — PA6=SCK,  PA7=CS,  PA8=MOSI,  PA9=MISO  (AF3)
 *   SPIM2 — PB2=SCK,  PB3=CS,  PB4=MOSI,  PB5=MISO  (AF0)
 *
 * Clock formula (from vendor core_spi.c):
 *   PCLK = SystemCoreClock / 4
 *   Divider = PCLK / desired_speed   (must be even, >= 2)
 *   Actual clock = PCLK / divider
 *
 * SPDX-License-Identifier: MIT
 */

#include "SPI.h"
#include "Arduino.h"

/* ================================================================== */
/*  Default instance — SPIM2 on PB2/PB3/PB4/PB5 (AF0)                */
/* ================================================================== */
SPIClass SPI(SPIM2, PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_SS, 0);

/* ---- TX/RX FIFO depth for standard SPI masters (SPIM0/1/2) ---- */
#define SPIM_FIFO_DEPTH  16

/* ================================================================== */
/*  Constructor                                                       */
/* ================================================================== */

SPIClass::SPIClass(SPI_TypeDef *spi,
                   uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin,
                   uint8_t ssPin, uint8_t afValue)
    : _spi(spi),
      _sckPin(sckPin), _misoPin(misoPin), _mosiPin(mosiPin),
      _ssPin(ssPin), _afValue(afValue),
      _bitOrder(MSBFIRST), _dataMode(SPI_MODE0),
      _clockFreq(4000000),
      _begun(false)
{}

/* ================================================================== */
/*  Pin mux helper (same approach as Wire.cpp)                        */
/* ================================================================== */

void SPIClass::_setIomux(uint8_t pin, uint8_t func)
{
    uint8_t  port  = pin >> 4;
    uint8_t  bit   = pin & 0x0F;
    uint32_t shift = bit * 2;
    uint32_t mask  = ~(0x03UL << shift);
    uint32_t val   = (uint32_t)func << shift;
    GPIO->ALT[port] = (GPIO->ALT[port] & mask) | val;
}

/* ================================================================== */
/*  Pin configuration                                                 */
/* ================================================================== */

void SPIClass::_configurePins()
{
    /* Route SCK, MOSI, MISO to the SPI alternate function */
    _setIomux(_sckPin,  _afValue);
    _setIomux(_mosiPin, _afValue);
    _setIomux(_misoPin, _afValue);

    /* Switch SS/CS pin to GPIO mode (AF1) so the hardware slave-select
     * does not interfere.  The user drives CS manually via digitalWrite(). */
    _setIomux(_ssPin, 1);   /* AF1 = GPIO mode on Air105 */

    /* Configure SS as GPIO output HIGH (deasserted) */
    GPIO_TypeDef *port = &GPIO_GROUP[_ssPin >> 4];
    uint16_t bitmask = (uint16_t)(1U << (_ssPin & 0x0F));
    port->BSRR = bitmask;       /* drive high (deassert) */
    port->PUE &= ~bitmask;      /* no pull-up needed */
    port->OEN &= ~bitmask;      /* OEN=0 → output */
}

/* ================================================================== */
/*  Apply SPI mode / speed to registers                               */
/* ================================================================== */

void SPIClass::_applyConfig(uint32_t clock, uint8_t dataMode)
{
    /* ---- Disable SPI while reconfiguring ---- */
    _spi->SSIENR = 0;
    _spi->SER    = 0;
    _spi->IMR    = 0;     /* no interrupts */
    _spi->DMACR  = 0;     /* no DMA */

    /* ---- CTRLR0: 8-bit, Motorola SPI, CPOL/CPHA ---- */
    uint16_t ctrl = 0x0007;  /* DFS[3:0] = 7 → 8-bit frame */

    switch (dataMode) {
        default:
        case SPI_MODE0: /* CPOL=0, CPHA=0 */
            break;
        case SPI_MODE1: /* CPOL=0, CPHA=1 */
            ctrl |= SPI_CTRLR0_SCPH;
            break;
        case SPI_MODE2: /* CPOL=1, CPHA=0 */
            ctrl |= SPI_CTRLR0_SCPOL;
            break;
        case SPI_MODE3: /* CPOL=1, CPHA=1 */
            ctrl |= SPI_CTRLR0_SCPOL | SPI_CTRLR0_SCPH;
            break;
    }
    _spi->CTRLR0 = ctrl;

    /* ---- BAUDR: clock divider ----
     * PCLK = SystemCoreClock / 4
     * Actual SPI clock = PCLK / BAUDR
     * BAUDR must be even and >= 2 */
    uint32_t pclk = SystemCoreClock >> 2;
    uint32_t div  = pclk / clock;
    if (div < 2) div = 2;
    if (div & 1) div++;     /* must be even */
    _spi->BAUDR = div;

    /* ---- FIFO thresholds ---- */
    _spi->TXFTLR = 0;
    _spi->RXFTLR = 0;

    /* ---- DMA levels (unused, but init to sane defaults) ---- */
    _spi->DMATDLR = 7;
    _spi->DMARDLR = 0;

    /* ---- Enable SPI ---- */
    _spi->SSIENR = 1;
}

/* ================================================================== */
/*  begin / end                                                       */
/* ================================================================== */

void SPIClass::begin()
{
    if (_begun) return;
    _begun = true;

    /* If this is SPIM0, ensure master mode (clear slave-enable bit) */
    if (_spi == SPIM0) {
        SYSCTRL->PHER_CTRL &= ~SYSCTRL_PHER_CTRL_SPI0_SLV_EN;
    }

    /* Soft-reset the SPI peripheral via SYSCTRL */
    if (_spi == SPIM0) {
        SYSCTRL->SOFT_RST1 = SYSCTRL_APBPeriph_SPI0;
        while (SYSCTRL->SOFT_RST1 & SYSCTRL_APBPeriph_SPI0) {}
    } else if (_spi == SPIM1) {
        SYSCTRL->SOFT_RST1 = SYSCTRL_APBPeriph_SPI1;
        while (SYSCTRL->SOFT_RST1 & SYSCTRL_APBPeriph_SPI1) {}
    } else if (_spi == SPIM2) {
        SYSCTRL->SOFT_RST1 = SYSCTRL_APBPeriph_SPI2;
        while (SYSCTRL->SOFT_RST1 & SYSCTRL_APBPeriph_SPI2) {}
    }

    /* Configure GPIO pin mux */
    _configurePins();

    /* Apply default configuration */
    _applyConfig(_clockFreq, _dataMode);
}

void SPIClass::end()
{
    if (!_begun) return;
    _begun = false;

    /* Disable the SPI engine */
    _spi->SSIENR = 0;
    _spi->SER    = 0;

    /* Restore all four pins to GPIO mode (AF1) */
    _setIomux(_sckPin,  1);
    _setIomux(_mosiPin, 1);
    _setIomux(_misoPin, 1);
    /* SS already in GPIO mode from begin() */
}

/* ================================================================== */
/*  Transaction API                                                   */
/* ================================================================== */

void SPIClass::beginTransaction(SPISettings settings)
{
    if (!_begun) begin();

    /* Reconfigure only if settings changed */
    if (settings._clock != _clockFreq || settings._dataMode != _dataMode) {
        _clockFreq = settings._clock;
        _dataMode  = settings._dataMode;
        _applyConfig(_clockFreq, _dataMode);
    }
    _bitOrder = settings._bitOrder;
}

void SPIClass::endTransaction()
{
    /* Nothing to do — CS is user-managed */
}

/* ================================================================== */
/*  Byte reversal (for LSBFIRST software emulation)                   */
/* ================================================================== */

uint8_t SPIClass::_reverseByte(uint8_t b)
{
    b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);
    b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);
    b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);
    return b;
}

/* ================================================================== */
/*  transfer — single byte                                            */
/* ================================================================== */

uint8_t SPIClass::transfer(uint8_t data)
{
    if (!_begun) begin();

    if (_bitOrder == LSBFIRST) data = _reverseByte(data);

    /* Deselect → load TX FIFO → select (starts clocking) */
    _spi->SER = 0;
    _spi->DR  = data;
    _spi->SER = 1;

    /* Wait for the received byte */
    while (!_spi->RXFLR) {}
    uint8_t rx = (uint8_t)_spi->DR;

    _spi->SER = 0;

    if (_bitOrder == LSBFIRST) rx = _reverseByte(rx);
    return rx;
}

/* ================================================================== */
/*  transfer16 — two bytes, respecting bit order                      */
/* ================================================================== */

uint16_t SPIClass::transfer16(uint16_t data)
{
    uint8_t hi, lo;

    if (_bitOrder == MSBFIRST) {
        hi = transfer((uint8_t)(data >> 8));
        lo = transfer((uint8_t)(data & 0xFF));
        return ((uint16_t)hi << 8) | lo;
    } else {
        lo = transfer((uint8_t)(data & 0xFF));
        hi = transfer((uint8_t)(data >> 8));
        return ((uint16_t)hi << 8) | lo;
    }
}

/* ================================================================== */
/*  transfer(buf, count) — bulk in-place exchange via FIFO            */
/* ================================================================== */

void SPIClass::transfer(void *buf, size_t count)
{
    if (!buf || count == 0) return;
    if (!_begun) begin();

    uint8_t *p = (uint8_t *)buf;

    _spi->SER = 0;   /* deselect before filling FIFO */

    /* Pre-fill the TX FIFO (up to FIFO depth) */
    uint32_t txIdx  = 0;
    uint32_t rxIdx  = 0;
    uint32_t preFill = (count > SPIM_FIFO_DEPTH) ? SPIM_FIFO_DEPTH : count;

    for (uint32_t i = 0; i < preFill; i++) {
        uint8_t d = (_bitOrder == LSBFIRST) ? _reverseByte(p[i]) : p[i];
        _spi->DR = d;
        txIdx++;
    }

    _spi->SER = 1;   /* assert slave-select → clocking begins */

    /* Stream: read RX FIFO, push more TX as space opens */
    while (rxIdx < count) {
        /* Drain available RX bytes */
        while (_spi->RXFLR && rxIdx < count) {
            uint8_t rx = (uint8_t)_spi->DR;
            if (_bitOrder == LSBFIRST) rx = _reverseByte(rx);
            p[rxIdx++] = rx;
        }
        /* Push more TX bytes if FIFO has room */
        while (txIdx < count && (_spi->SR & SPI_SR_TFNF)) {
            uint8_t d = (_bitOrder == LSBFIRST) ? _reverseByte(p[txIdx]) : p[txIdx];
            _spi->DR = d;
            txIdx++;
        }
    }

    _spi->SER = 0;   /* deselect */
}

/* ================================================================== */
/*  Legacy API                                                        */
/* ================================================================== */

void SPIClass::setBitOrder(uint8_t bitOrder)
{
    _bitOrder = bitOrder;
}

void SPIClass::setDataMode(uint8_t dataMode)
{
    _dataMode = dataMode;
    if (_begun) _applyConfig(_clockFreq, _dataMode);
}

void SPIClass::setClockDivider(uint8_t clockDiv)
{
    /* Interpret divider relative to PCLK (= SystemCoreClock / 4) */
    uint32_t pclk = SystemCoreClock >> 2;
    _clockFreq = pclk / clockDiv;
    if (_begun) _applyConfig(_clockFreq, _dataMode);
}

/* ================================================================== */
/*  STM32duino pin-parameterized begin/transaction API                 */
/* ================================================================== */

void SPIClass::begin(uint32_t ssPin)
{
    if (ssPin != CS_PIN_CONTROLLED_BY_USER) {
        _ssPin = (uint8_t)ssPin;
    }
    begin();
}

void SPIClass::beginTransaction(uint32_t ssPin, SPISettings settings)
{
    beginTransaction(settings);
    if (ssPin != CS_PIN_CONTROLLED_BY_USER) {
        digitalWrite((uint8_t)ssPin, LOW);
    }
}

void SPIClass::endTransaction(uint32_t ssPin)
{
    if (ssPin != CS_PIN_CONTROLLED_BY_USER) {
        digitalWrite((uint8_t)ssPin, HIGH);
    }
    endTransaction();
}

/* ================================================================== */
/*  STM32duino pin-parameterized transfer                             */
/* ================================================================== */

uint8_t SPIClass::transfer(uint32_t ssPin, uint8_t data, SPITransferMode mode)
{
    if (ssPin != CS_PIN_CONTROLLED_BY_USER) {
        digitalWrite((uint8_t)ssPin, LOW);
    }

    uint8_t rx = transfer(data);

    if (mode == SPI_LAST && ssPin != CS_PIN_CONTROLLED_BY_USER) {
        digitalWrite((uint8_t)ssPin, HIGH);
    }
    return rx;
}

uint16_t SPIClass::transfer16(uint32_t ssPin, uint16_t data, SPITransferMode mode)
{
    if (ssPin != CS_PIN_CONTROLLED_BY_USER) {
        digitalWrite((uint8_t)ssPin, LOW);
    }

    uint16_t rx = transfer16(data);

    if (mode == SPI_LAST && ssPin != CS_PIN_CONTROLLED_BY_USER) {
        digitalWrite((uint8_t)ssPin, HIGH);
    }
    return rx;
}

void SPIClass::transfer(uint32_t ssPin, void *buf, size_t count, SPITransferMode mode)
{
    if (ssPin != CS_PIN_CONTROLLED_BY_USER) {
        digitalWrite((uint8_t)ssPin, LOW);
    }

    transfer(buf, count);

    if (mode == SPI_LAST && ssPin != CS_PIN_CONTROLLED_BY_USER) {
        digitalWrite((uint8_t)ssPin, HIGH);
    }
}

/* ================================================================== */
/*  STM32duino pin reconfiguration                                    */
/* ================================================================== */

void SPIClass::setMISO(uint32_t miso) { _misoPin = (uint8_t)miso; }
void SPIClass::setMOSI(uint32_t mosi) { _mosiPin = (uint8_t)mosi; }
void SPIClass::setSCLK(uint32_t sclk) { _sckPin  = (uint8_t)sclk; }
void SPIClass::setSSEL(uint32_t ssel) { _ssPin   = (uint8_t)ssel; }

/* ================================================================== */
/*  Interrupt guarding (stubs — bare-metal, no RTOS)                  */
/* ================================================================== */

void SPIClass::usingInterrupt(int interruptNumber)    { (void)interruptNumber; }
void SPIClass::notUsingInterrupt(int interruptNumber) { (void)interruptNumber; }

/* ================================================================== */
/*  Deprecated stubs                                                  */
/* ================================================================== */

void SPIClass::attachInterrupt() {}
void SPIClass::detachInterrupt() {}

/* ================================================================== */
/*  DMA helpers                                                       */
/* ================================================================== */

uint8_t SPIClass::_dmaRequestTx() const
{
    if (_spi == SPIM0) return DMA_REQUEST_SPI0_TX;
    if (_spi == SPIM1) return DMA_REQUEST_SPI1_TX;
    return DMA_REQUEST_SPI2_TX;
}

uint8_t SPIClass::_dmaRequestRx() const
{
    if (_spi == SPIM0) return DMA_REQUEST_SPI0_RX;
    if (_spi == SPIM1) return DMA_REQUEST_SPI1_RX;
    return DMA_REQUEST_SPI2_RX;
}

/* ================================================================== */
/*  SPI DMA transfer — full duplex                                    */
/* ================================================================== */

bool SPIClass::transferDMA(const void *txBuf, void *rxBuf, size_t count)
{
    if (count == 0) return true;
    if (!txBuf && !rxBuf) return false;
    if (!_begun) begin();

    /* Static dummy byte for TX-only (sends 0xFF) or RX-discard */
    static const uint8_t dummyTx = 0xFF;
    static uint8_t dummyRx;

    /* Allocate two DMA channels: one for TX, one for RX */
    int8_t chTx = DMAControl.allocateChannel();
    if (chTx < 0) return false;

    int8_t chRx = DMAControl.allocateChannel();
    if (chRx < 0) {
        DMAControl.freeChannel(chTx);
        return false;
    }

    /* Configure TX channel */
    DMAControl.configure(chTx,
                         _dmaRequestTx(),
                         DMA_MEMORY_TO_PERIPH,
                         DMA_PINC_DISABLE,                     /* SPI DR is fixed */
                         txBuf ? DMA_MINC_ENABLE : DMA_MINC_DISABLE, /* inc if real buf */
                         DMA_PDATAALIGN_BYTE,
                         DMA_MDATAALIGN_BYTE,
                         DMA_PRIORITY_MEDIUM);

    /* Configure RX channel */
    DMAControl.configure(chRx,
                         _dmaRequestRx(),
                         DMA_PERIPH_TO_MEMORY,
                         DMA_PINC_DISABLE,                     /* SPI DR is fixed */
                         rxBuf ? DMA_MINC_ENABLE : DMA_MINC_DISABLE,
                         DMA_PDATAALIGN_BYTE,
                         DMA_MDATAALIGN_BYTE,
                         DMA_PRIORITY_HIGH);

    bool ok = true;
    size_t offset = 0;

    while (offset < count) {
        /* DMA max block is 4095; chunk if needed */
        uint32_t chunk = count - offset;
        if (chunk > DMA_MAX_BLOCK_SIZE) chunk = DMA_MAX_BLOCK_SIZE;

        /* Prepare SPI for DMA: disable, set thresholds, enable DMA signals */
        _spi->SSIENR = 0;
        _spi->DMACR  = 0;
        _spi->DMATDLR = 7;   /* TX DMA request when FIFO <= 7 entries */
        _spi->DMARDLR = 0;   /* RX DMA request when FIFO >= 1 entry */
        _spi->DMACR  = SPI_DMACR_TDMAE | SPI_DMACR_RDMAE;
        _spi->SSIENR = 1;

        /* Start RX DMA first (so it's ready to drain) */
        const void *rxAddr = rxBuf ? ((const uint8_t *)rxBuf + offset) : (void *)&dummyRx;
        DMAControl.start(chRx, (uint32_t)&_spi->DR, (void *)rxAddr, chunk);

        /* Start TX DMA */
        const void *txAddr = txBuf ? ((const uint8_t *)txBuf + offset) : (const void *)&dummyTx;
        DMAControl.start(chTx, (uint32_t)&_spi->DR, (void *)txAddr, chunk);

        /* Assert slave select — starts clocking */
        _spi->SER = 1;

        /* Wait for both channels to complete (RX finishes last in full-duplex) */
        if (!DMAControl.poll(chRx, 5000) || !DMAControl.poll(chTx, 5000)) {
            DMAControl.abort(chTx);
            DMAControl.abort(chRx);
            ok = false;
            break;
        }

        offset += chunk;
    }

    /* Deselect and disable DMA mode on SPI */
    _spi->SER = 0;
    _spi->SSIENR = 0;
    _spi->DMACR  = 0;
    _spi->SSIENR = 1;

    DMAControl.freeChannel(chTx);
    DMAControl.freeChannel(chRx);

    return ok;
}

/* ================================================================== */
/*  SPI DMA write — TX-only (discard RX)                              */
/* ================================================================== */

bool SPIClass::writeDMA(const void *buf, size_t count)
{
    if (!buf || count == 0) return (count == 0);
    return transferDMA(buf, nullptr, count);
}

/* ================================================================== */
/*  SPI DMA read — RX-only (send 0xFF dummy)                         */
/* ================================================================== */

bool SPIClass::readDMA(void *buf, size_t count)
{
    if (!buf || count == 0) return (count == 0);
    return transferDMA(nullptr, buf, count);
}
