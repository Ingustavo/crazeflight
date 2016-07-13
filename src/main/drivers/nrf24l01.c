/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

// This file is copied with modifications from project Deviation,
// see http://deviationtx.com

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#include "system.h"
#include "gpio.h"
#include "nrf24l01.h"
#include "bus_spi.h"

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

#define DISABLE_NRF24       GPIO_SetBits(NRF24_CS_GPIO, NRF24_CS_PIN)
#define ENABLE_NRF24        GPIO_ResetBits(NRF24_CS_GPIO, NRF24_CS_PIN)

#define NRF_24_CE_HIGH      GPIO_SetBits(NRF24_CE_GPIO, NRF24_CE_PIN)
#define NRF_24_CE_LOW       GPIO_ResetBits(NRF24_CE_GPIO, NRF24_CE_PIN)

static uint8_t rf_setup;

void NRF24L01_SpiInit(void)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

#ifdef STM32F303xC
    // CS as output
    RCC_AHBPeriphClockCmd(NRF24_CS_GPIO_CLK_PERIPHERAL, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = NRF24_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(NRF24_CS_GPIO, &GPIO_InitStructure);
    // CE as OUTPUT
    RCC_AHBPeriphClockCmd(NRF24_CE_GPIO_CLK_PERIPHERAL, ENABLE);

    GPIO_InitStructure.GPIO_Pin = NRF24_CE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(NRF24_CE_GPIO, &GPIO_InitStructure);
#endif

#ifdef STM32F10X
    // CS as output
    RCC_APB2PeriphClockCmd(NRF24_CS_GPIO_CLK_PERIPHERAL, ENABLE);

    gpio_config_t gpio;
    gpio.mode = Mode_Out_PP;
    gpio.pin = NRF24_CS_PIN;
    gpio.speed = Speed_50MHz;

    gpioInit(NRF24_CS_GPIO, &gpio);
    // CE as output
    RCC_APB2PeriphClockCmd(NRF24_CE_GPIO_CLK_PERIPHERAL, ENABLE);

    gpio.mode = Mode_Out_PP;
    gpio.pin = NRF24_CE_PIN;
    gpio.speed = Speed_50MHz;

    gpioInit(NRF24_CE_GPIO, &gpio);
    // TODO: NRF24_IRQ as input
#endif

    DISABLE_NRF24;
    NRF_24_CE_LOW;

    spiSetDivisor(NRF24_SPI_INSTANCE, SPI_9MHZ_CLOCK_DIVIDER);

    hardwareInitialised = true;
}

void NRF24L01_Initialize()
{
    rf_setup = 0x0F;
}

uint8_t NRF24L01_WriteReg(uint8_t reg, uint8_t data)
{
    ENABLE_NRF24;
    spiTransferByte(NRF24_SPI_INSTANCE, W_REGISTER | (REGISTER_MASK & reg));
    spiTransferByte(NRF24_SPI_INSTANCE, data);
    DISABLE_NRF24;
    return true;
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
    ENABLE_NRF24;
    spiTransferByte(NRF24_SPI_INSTANCE, R_REGISTER | (REGISTER_MASK & reg));
    uint8_t data = spiTransferByte(NRF24_SPI_INSTANCE, 0xFF);
    DISABLE_NRF24;
    return data;
}

uint8_t NRF24L01_WriteRegisterMulti(uint8_t reg, const uint8_t data[], uint8_t length)
{
    ENABLE_NRF24;
    uint8_t res = spiTransferByte(NRF24_SPI_INSTANCE, W_REGISTER | ( REGISTER_MASK & reg));
    for (uint8_t i = 0; i < length; i++)
    {
        spiTransferByte(NRF24_SPI_INSTANCE, data[i]);
    }
    DISABLE_NRF24;
    return res;
}

uint8_t NRF24L01_WritePayload(uint8_t *data, uint8_t length)
{
    ENABLE_NRF24;
    uint8_t res = spiTransferByte(NRF24_SPI_INSTANCE, W_TX_PAYLOAD);
    for (uint8_t i = 0; i < length; i++)
    {
        spiTransferByte(NRF24_SPI_INSTANCE, data[i]);
    }
    DISABLE_NRF24;
    return res;
}

uint8_t NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t data[], uint8_t length)
{
    ENABLE_NRF24;
    uint8_t res = spiTransferByte(NRF24_SPI_INSTANCE, R_REGISTER | (REGISTER_MASK & reg));
    for(uint8_t i = 0; i < length; i++)
    {
        data[i] = spiTransferByte(NRF24_SPI_INSTANCE, 0xFF);
    }
    DISABLE_NRF24;
    return res;
}

uint8_t NRF24L01_ReadPayload(uint8_t *data, uint8_t length)
{
    ENABLE_NRF24;
    uint8_t res = spiTransferByte(NRF24_SPI_INSTANCE, R_RX_PAYLOAD);
    for(uint8_t i = 0; i < length; i++)
    {
        data[i] = spiTransferByte(NRF24_SPI_INSTANCE, 0xFF);
    }
    DISABLE_NRF24;
    return res;
}

static uint8_t Strobe(uint8_t state)
{
    ENABLE_NRF24;
    uint8_t res = spiTransferByte(NRF24_SPI_INSTANCE, state);
    DISABLE_NRF24;
    return res;
}

uint8_t NRF24L01_FlushTx()
{
    return Strobe(FLUSH_TX);
}

uint8_t NRF24L01_FlushRx()
{
    return Strobe(FLUSH_RX);
}

uint8_t NRF24L01_Activate(uint8_t code)
{
    ENABLE_NRF24;
    uint8_t res = spiTransferByte(NRF24_SPI_INSTANCE, ACTIVATE);
    spiTransferByte(NRF24_SPI_INSTANCE, code);
    DISABLE_NRF24;
    return res;
}

uint8_t NRF24L01_SetBitrate(uint8_t bitrate)
{
    // Note that bitrate 250kbps (and bit RF_DR_LOW) is valid only
    // for nRF24L01+. There is no way to programmatically tell it from
    // older version, nRF24L01, but the older is practically phased out
    // by Nordic, so we assume that we deal with with modern version.

    // Bit 0 goes to RF_DR_HIGH, bit 1 - to RF_DR_LOW
    rf_setup = (rf_setup & 0xD7) | ((bitrate & 0x02) << 4) | ((bitrate & 0x01) << 3);
    return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}

uint8_t NRF24L01_SetPower(uint8_t power)
{
    if(power > 3)
        power = 3;
    // Power is in range 0..3 for nRF24L01
    rf_setup = (rf_setup & 0xF9) | ((power & 0x03) << 1);
    return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}

void NRF24L01_SetTxRxMode(enum TXRX_State mode)
{
    if(mode == TX_EN) {
        NRF_24_CE_LOW;
        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to TX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP));
        delayMicroseconds(130);
        NRF_24_CE_HIGH;
    } else if (mode == RX_EN) {
        NRF_24_CE_LOW;
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);        // reset the flag(s)
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);        // switch to RX mode
        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to RX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP)
                                            | (1 << NRF24L01_00_PRIM_RX));
        delayMicroseconds(130);
        NRF_24_CE_HIGH;
    } else {
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)); //PowerDown
        NRF_24_CE_LOW;
    }
}

bool NRF24L01_Reset()
{
    NRF24L01_SetTxRxMode(TXRX_OFF);
    return NRF24L01_ReadReg(0x07) == 0x0E;
}

