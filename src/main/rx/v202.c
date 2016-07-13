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

// this file is copied with modifications from bradwii for jd385
// see https://github.com/hackocopter/bradwii-jd385

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>
#include "debug.h"
#include "common/utils.h"
#include "drivers/system.h"
#include "io/rc_controls.h"

#include "rx/rx.h"
#include "rx/v202.h"

#include "drivers/nrf24l01.h"

#define V2X2_PAYLOAD_SIZE 16
#define V2X2_NFREQCHANNELS 16
#define TXIDSIZE 3
#define SUPPORTED_RC_CHANNELS 8

enum {
    V2X2_FLAG_CAMERA = 0x01, // also automatic Missile Launcher and Hoist in one direction
    V2X2_FLAG_VIDEO  = 0x02, // also Sprayer, Bubbler, Missile Launcher(1), and Hoist in the other dir.
    V2X2_FLAG_FLIP   = 0x04,
    V2X2_FLAG_UNK9   = 0x08,
    V2X2_FLAG_LED    = 0x10,
    V2X2_FLAG_UNK10  = 0x20,
    V2X2_FLAG_BIND   = 0xC0
};

enum {
    PHASE_NOT_BOUND = 0,
    PHASE_BOUND
};

#define BV(x) (1 << (x))

// This is frequency hopping table for V202 protocol
// The table is the first 4 rows of 32 frequency hopping
// patterns, all other rows are derived from the first 4.
// For some reason the protocol avoids channels, dividing
// by 16 and replaces them by subtracting 3 from the channel
// number in this case.
// The pattern is defined by 5 least significant bits of
// sum of 3 bytes comprising TX id
static const uint8_t v2x2_freq_hopping[][V2X2_NFREQCHANNELS] = {
 { 0x27, 0x1B, 0x39, 0x28, 0x24, 0x22, 0x2E, 0x36,
   0x19, 0x21, 0x29, 0x14, 0x1E, 0x12, 0x2D, 0x18 }, //  00
 { 0x2E, 0x33, 0x25, 0x38, 0x19, 0x12, 0x18, 0x16,
   0x2A, 0x1C, 0x1F, 0x37, 0x2F, 0x23, 0x34, 0x10 }, //  01
 { 0x11, 0x1A, 0x35, 0x24, 0x28, 0x18, 0x25, 0x2A,
   0x32, 0x2C, 0x14, 0x27, 0x36, 0x34, 0x1C, 0x17 }, //  02
 { 0x22, 0x27, 0x17, 0x39, 0x34, 0x28, 0x2B, 0x1D,
   0x18, 0x2A, 0x21, 0x38, 0x10, 0x26, 0x20, 0x1F }  //  03
};

static uint8_t freqhopping[V2X2_NFREQCHANNELS];
static uint8_t rf_channels[V2X2_NFREQCHANNELS];
static uint8_t packet[V2X2_PAYLOAD_SIZE];
static uint8_t rf_ch_num;
static uint8_t bind_phase;
static uint32_t packet_timer;
static uint8_t txid[TXIDSIZE];
static uint32_t rx_timeout;
static uint32_t valid_packets;
static uint16_t chan_data[SUPPORTED_RC_CHANNELS];
unsigned char v2x2_channelindex[] = {THROTTLE,YAW,PITCH,ROLL,AUX1,AUX2,AUX3,AUX4};

void v2x2_set_tx_id(uint8_t *id)
{
    uint8_t sum;
    txid[0] = id[0];
    txid[1] = id[1];
    txid[2] = id[2];
    sum = id[0] + id[1] + id[2];

    // Base row is defined by lowest 2 bits
    const uint8_t *fh_row = v2x2_freq_hopping[sum & 0x03];
    // Higher 3 bits define increment to corresponding row
    uint8_t increment = (sum & 0x1e) >> 2;
    for (int i = 0; i < V2X2_NFREQCHANNELS; ++i) {
        uint8_t val = fh_row[i] + increment;
        // Strange avoidance of channels divisible by 16
        freqhopping[i] = (val & 0x0f) ? val : val - 3;
    }
}

static void prepare_to_bind(void)
{
    packet_timer = micros();
    for (int i = 0; i < V2X2_NFREQCHANNELS; ++i) {
        rf_channels[i] = v2x2_freq_hopping[0][i];
    }
    rx_timeout = 1000L;
}

static void switch_channel(void)
{
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_channels[rf_ch_num]);
    if (++rf_ch_num >= V2X2_NFREQCHANNELS) rf_ch_num = 0;
}

void initV202rx(void)
{
    NRF24L01_Initialize();

    // 2-bytes CRC, radio off
    uint8_t config = BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PRIM_RX);

    NRF24L01_WriteReg(NRF24L01_00_CONFIG, config);
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0xFF); // 4ms retransmit t/o, 15 tries
    NRF24L01_SetBitrate(NRF24L01_BR_250K);
    NRF24L01_SetPower(3);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, V2X2_PAYLOAD_SIZE);  // bytes of data payload for pipe 0
    NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here
    uint8_t rx_tx_addr[] = {0x66, 0x88, 0x68, 0x68, 0x68};
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);

    delay(50);

    NRF24L01_FlushTx();
    NRF24L01_FlushRx();

    rf_ch_num = 0;

    // Turn radio power on
    config |= BV(NRF24L01_00_PWR_UP);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, config);
    NRF24L01_SetTxRxMode(RX_EN);

    bind_phase = PHASE_NOT_BOUND;
    prepare_to_bind();

    switch_channel();
}

static void set_bound()
{
    for (int i = 0; i < V2X2_NFREQCHANNELS; ++i) {
        rf_channels[i] = freqhopping[i];
    }
    //nfreqchannels = fhsize;
    rx_timeout = 1000L; // find the channel as fast as possible
}

static void decode_bind_packet(uint8_t *packet)
{
    if ((packet[14] & V2X2_FLAG_BIND) == V2X2_FLAG_BIND) {
        // Fill out usersettings with bound protocol parameters
        v2x2_set_tx_id(&packet[7]);
        // Read usersettings into current values
        bind_phase = PHASE_BOUND;
        set_bound();
    }
}

// Returns whether the data was successfully decoded
static bool decode_packet(uint8_t *packet, uint16_t *data)
{
    if(bind_phase != PHASE_BOUND) {
        decode_bind_packet(packet);
    }
    else {
        // Decode packet
        if ((packet[14] & V2X2_FLAG_BIND) == V2X2_FLAG_BIND) {
            return false;
        }
        if (packet[7] != txid[0] ||
            packet[8] != txid[1] ||
            packet[9] != txid[2])
        {
            return false;
        }
        // Restore regular interval
        rx_timeout = 10000L; // 4ms interval, duplicate packets, (8ms unique) + 25%
        // TREA order in packet to MultiWii order is handled by
        // correct assignment to channelindex
        // Throttle 0..255 to 1000..2000
        data[v2x2_channelindex[0]] = ((uint16_t)packet[0]) * 1000 / 255 + 1000;
        for (int i = 1; i < 4; ++i) {
            uint8_t a = packet[i];
            data[v2x2_channelindex[i]] = ((uint16_t)(a < 0x80 ? 0x7f - a : a)) * 1000 / 255 + 1000;
        }
        uint8_t flags[] = {V2X2_FLAG_LED, V2X2_FLAG_FLIP, V2X2_FLAG_CAMERA, V2X2_FLAG_VIDEO}; // two more unknown bits
        for (int i = 4; i < 8; ++i) {
            data[v2x2_channelindex[i]] = 1000 + ((packet[14] & flags[i-4]) ? 1000 : 0);
        }
        packet_timer = micros();
        if (++valid_packets > 50) bind_phase = PHASE_BOUND;
        return true;
    }
    return false;
}

bool readrx(void)
{
    uint16_t data[8];

    if (!(NRF24L01_ReadReg(NRF24L01_07_STATUS) & BV(NRF24L01_07_RX_DR))) {
        uint32_t t = micros() - packet_timer;
        if (t > rx_timeout) {
            switch_channel();
            packet_timer = micros();
        }
        return false;
    }
    packet_timer = micros();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BV(NRF24L01_07_RX_DR));
    NRF24L01_ReadPayload(packet, V2X2_PAYLOAD_SIZE);
    NRF24L01_FlushRx();
    switch_channel();
    if (!decode_packet(packet, data))
        return false;
    for(int i=0; i<SUPPORTED_RC_CHANNELS; i++)
        chan_data[i] = data[i];
    return true;
}

bool v202DataReceived()
{
    return readrx();
}

static uint16_t v202ReadRawRC(rxRuntimeConfig_t *rxRuntimeConfigPtr, uint8_t channel)
{
    UNUSED(rxRuntimeConfigPtr);
    return chan_data[channel];
}

bool v202ProtocolInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    UNUSED(rxConfig); // todo: init 250k or 1mbps protocol
    NRF24L01_SpiInit();
    NRF24L01_Reset();
    initV202rx();
    rxRuntimeConfig->channelCount = SUPPORTED_RC_CHANNELS;
    *callback = v202ReadRawRC;
    return true;
}
