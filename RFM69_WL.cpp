// **********************************************************************************
// Automatic Transmit Power Control class derived from RFM69 library.
// **********************************************************************************
// Copyright Thomas Studwell (2014,2015), James Willcox (2016)
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <RFM69_WL.h>
#include <RFM69registers.h>
#include <SPI.h>

#define ENCRYPT_KEY_LENGTH 16

#define BURST_CYCLE_PADDING_MS 50

// #define RFM69_WL_DEBUG

volatile uint16_t RFM69_WL::LISTEN_BURST_REMAINING_MS = 0;


//=============================================================================
// initialize() - some extra initialization before calling base class
//=============================================================================
bool RFM69_WL::initialize(uint8_t freqBand, uint8_t address, uint8_t networkID)
{
  if (!RFM69::initialize(freqBand, address, networkID)) {
    return false;
  }

  abortListenMode();

  return true;
}

void RFM69_WL::receiveBegin()
{
  RFM69::receiveBegin();
  LISTEN_BURST_REMAINING_MS = 0;
}

//=============================================================================
// abortListenMode() - exit listen mode and nothing else
//=============================================================================
void RFM69_WL::abortListenMode()
{
  writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTENABORT | RF_OPMODE_STANDBY);
  writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_STANDBY);
  setMode(RF69_MODE_STANDBY);
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
}

static uint32_t getUsForResolution(byte resolution)
{
  switch (resolution) {
    case RF_LISTEN1_RESOL_RX_64:
    case RF_LISTEN1_RESOL_IDLE_64:
      return 64;
    case RF_LISTEN1_RESOL_RX_4100:
    case RF_LISTEN1_RESOL_IDLE_4100:
      return 4100;
    case RF_LISTEN1_RESOL_RX_262000:
    case RF_LISTEN1_RESOL_IDLE_262000:
      return 262000;
    default:
      // Whoops
      return 0;
  }
}

static uint32_t getCoefForResolution(byte resolution, uint32_t duration)
{
  uint32_t resolDuration = getUsForResolution(resolution);

  uint32_t result = duration / resolDuration;

  // If the next-higher coefficient is closer, use that
  if (abs(duration - ((result + 1) * resolDuration)) <
      abs(duration - (result * resolDuration)))
  {
    return result + 1;
  }

  return result;
}

static bool chooseResolutionAndCoef(byte *resolutions, uint32_t duration, byte& resolOut, byte& coefOut)
{
  for (int i = 0; resolutions[i]; i++) {
    uint32_t coef = getCoefForResolution(resolutions[i], duration);
    if (coef < 256) {
      coefOut = coef;
      resolOut = resolutions[i];
      return true;
    }
  }

  // out of range
  return false;
}

bool RFM69_WL::setListenDurations(uint32_t& rxDuration, uint32_t& idleDuration)
{
  byte rxResolutions[] = { RF_LISTEN1_RESOL_RX_64, RF_LISTEN1_RESOL_RX_4100, RF_LISTEN1_RESOL_RX_262000, 0 };
  byte idleResolutions[] = { RF_LISTEN1_RESOL_IDLE_64, RF_LISTEN1_RESOL_IDLE_4100, RF_LISTEN1_RESOL_IDLE_262000, 0 };

#ifdef RFM69_WL_DEBUG
  Serial.print("Attempting to match RX duration ");
  Serial.println(rxDuration, DEC);

  Serial.print("Attempting to match idle duration ");
  Serial.println(idleDuration);
#endif

  if (!chooseResolutionAndCoef(rxResolutions, rxDuration, _rxListenResolution, _rxListenCoef)) {
    return false;
  }

  if (!chooseResolutionAndCoef(idleResolutions, idleDuration, _idleListenResolution, _idleListenCoef)) {
    return false;
  }

  rxDuration = getUsForResolution(_rxListenResolution) * _rxListenCoef;
  idleDuration = getUsForResolution(_idleListenResolution) * _idleListenCoef;
  _listenCycleDurationMs = (rxDuration + idleDuration) / 1000;

#ifdef RFM69_WL_DEBUG
  Serial.print("RX resolution ");
  Serial.println(_rxListenResolution, DEC);
  Serial.print("RX coefficient ");
  Serial.println(_rxListenCoef, DEC);
  Serial.print("RX duration ");
  Serial.println(getUsForResolution(_rxListenResolution) * (uint32_t)_rxListenCoef, DEC);

  Serial.print("Idle resolution ");
  Serial.println(_idleListenResolution, DEC);
  Serial.print("Idle coefficient ");
  Serial.println(_idleListenCoef, DEC);
  Serial.print("Idle duration ");
  Serial.println(getUsForResolution(_idleListenResolution) * (uint32_t)_idleListenCoef, DEC);
#endif

  return true;
}

void RFM69_WL::getListenDurations(uint32_t &rxDuration, uint32_t &idleDuration)
{
  rxDuration = getUsForResolution(_rxListenResolution) * _rxListenCoef;
  idleDuration = getUsForResolution(_idleListenResolution) * _idleListenCoef;
}

//=============================================================================
// static pointer to 'this' needed by irq handler
//=============================================================================
static RFM69_WL*  pRadio;

//=============================================================================
// irq handler, simply calls listenIrq method so internal methods can be accessed easily
//=============================================================================
static void irq()
{
  pRadio->listenIrq();
}

void RFM69_WL::resetListenReceive(void)
{
  DATALEN = 0;
  SENDERID = 0;
  TARGETID = 0;
  PAYLOADLEN = 0;
  ACK_REQUESTED = 0;
  ACK_RECEIVED = 0;
  LISTEN_BURST_REMAINING_MS = 0;
}

//=============================================================================
// listenIrq() - only called by listen irq handler
//=============================================================================
void RFM69_WL::listenIrq(void)
{
  if (DATALEN != 0) return;

  resetListenReceive();

  noInterrupts();
  select();

  union                        // union to simplify addressing of long and short parts of time offset
  {
    uint32_t l;
    uint8_t  b[4];
  } burstRemaining;

  burstRemaining.l = 0;

  SPI.transfer(REG_FIFO & 0x7F);
  PAYLOADLEN = SPI.transfer(0);
  PAYLOADLEN = PAYLOADLEN > 66 ? 66 : PAYLOADLEN; // precaution
  TARGETID = SPI.transfer(0);
  if(!(_promiscuousMode || TARGETID == _address || TARGETID == RF69_BROADCAST_ADDR) // match this node's address, or broadcast address or anything in promiscuous mode
     || PAYLOADLEN < 4) // address situation could receive packets that are malformed and don't fit this library's extra fields
  {
    resetListenReceive();
    goto out;
  }

  SENDERID = SPI.transfer(0);
  burstRemaining.b[0] =  SPI.transfer(0);  // and get the time remaining
  burstRemaining.b[1] =  SPI.transfer(0);

  // We've read the target, sender id, and the burst time remaining for a total of 4 bytes
  DATALEN = PAYLOADLEN - 4;

  LISTEN_BURST_REMAINING_MS = burstRemaining.l;

  // if (LISTEN_BURST_REMAINING_MS > (_listenCycleDurationMs + BURST_CYCLE_PADDING_MS)) {
  //   // This happened to me with a presumably bad radio, but still a good sanity check
  //   resetListenReceive();
  //   return;
  // }

  for (uint8_t i = 0; i < DATALEN; i++) {
    DATA[i] = SPI.transfer(0);
  }

  if (DATALEN < RF69_MAX_DATA_LEN)
    DATA[DATALEN] = 0; // add null at end of string

out:
  unselect();
  interrupts();
}

void RFM69_WL::startListening(void)
{
  pRadio = this;

  abortListenMode();
  resetListenReceive();

  detachInterrupt(RF69_IRQ_NUM);
  attachInterrupt(RF69_IRQ_NUM, irq, RISING);
  setMode(RF69_MODE_STANDBY);

  _listenTransaction.pushReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);

  _listenTransaction.pushReg(REG_FRFMSB, readReg(REG_FRFMSB) + 1);
  _listenTransaction.pushReg(REG_FRFLSB, readReg(REG_FRFLSB));      // MUST write to LSB to affect change!

  if (_isHighSpeed) {
    _listenTransaction.pushReg(REG_BITRATEMSB, RF_BITRATEMSB_200000);
    _listenTransaction.pushReg(REG_BITRATELSB, RF_BITRATELSB_200000);
    _listenTransaction.pushReg(REG_FDEVMSB, RF_FDEVMSB_300000);
    _listenTransaction.pushReg(REG_FDEVLSB, RF_FDEVLSB_300000);
    _listenTransaction.pushReg(REG_RXBW, RF_RXBW_DCCFREQ_000 | RF_RXBW_MANT_20 | RF_RXBW_EXP_0);
  }

  // Force LNA to the highest gain
  _listenTransaction.pushReg(REG_LNA, (readReg(REG_LNA) & ~0x3) | RF_LNA_GAINSELECT_MAX);

  _listenTransaction.pushReg(REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_WHITENING | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON);

  uint8_t aesConfig = readReg(REG_PACKETCONFIG2) & RF_PACKET2_AES_ON;
  _listenTransaction.pushReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | aesConfig);
  _listenTransaction.pushReg(REG_SYNCVALUE1, 0x5A);
  _listenTransaction.pushReg(REG_SYNCVALUE2, 0x5A);
  _listenTransaction.pushReg(REG_LISTEN1, _rxListenResolution | _idleListenResolution |
                             RF_LISTEN1_CRITERIA_RSSI | RF_LISTEN1_END_10);
  _listenTransaction.pushReg(REG_LISTEN2, _idleListenCoef);
  _listenTransaction.pushReg(REG_LISTEN3, _rxListenCoef);

  _listenTransaction.pushReg(REG_RSSITHRESH, _rssiThreshold);
  _listenTransaction.pushReg(REG_RXTIMEOUT2, 75);

  // We want to handle these separately from the configuration above
  writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_STANDBY);
  writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_ON  | RF_OPMODE_STANDBY);
}

bool RFM69_WL::endListening(void)
{
  detachInterrupt(RF69_IRQ_NUM);
  attachInterrupt(RF69_IRQ_NUM, RFM69::isr0, RISING);

  _listenTransaction.revert();
  abortListenMode();

  return LISTEN_BURST_REMAINING_MS > 0 && DATALEN > 0;
}

// We send:
// Payload size
// Target node
// Sender node
// Two bytes for the burst time remaining
#define BURST_MSG_HEADER_SIZE 5


void RFM69_WL::sendBurst(uint8_t targetNode, void* buffer, uint8_t size, uint16_t durationMs)
{
  detachInterrupt(RF69_IRQ_NUM);
  setMode(RF69_MODE_STANDBY);

  RegisterTransaction trans(this);
  trans.pushReg(REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_WHITENING | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON );

  uint8_t aesConfig = readReg(REG_PACKETCONFIG2) & RF_PACKET2_AES_ON;
  trans.pushReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | aesConfig);

  // We don't want the radio to start transmitting until we fill the FIFO with our entire message, so
  // so set the FIFO threshold to one less than the message size. The radio processes the FIFO when
  // the threshold has one more than the threshold value.
  trans.pushReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | (size + BURST_MSG_HEADER_SIZE - 1));

  trans.pushReg(REG_SYNCVALUE1, 0x5A);
  trans.pushReg(REG_SYNCVALUE2, 0x5A);

  if (_isHighSpeed) {
    trans.pushReg(REG_BITRATEMSB, RF_BITRATEMSB_200000);
    trans.pushReg(REG_BITRATELSB, RF_BITRATELSB_200000);
    trans.pushReg(REG_FDEVMSB, RF_FDEVMSB_300000);
    trans.pushReg(REG_FDEVLSB, RF_FDEVLSB_300000);
    trans.pushReg(REG_RXBW, RF_RXBW_DCCFREQ_000 | RF_RXBW_MANT_20 | RF_RXBW_EXP_0);
  }

  trans.pushReg(REG_FRFMSB, readReg(REG_FRFMSB) + 1);
  trans.pushReg(REG_FRFLSB, readReg(REG_FRFLSB));      // MUST write to LSB to affect change!

  union // union to simplify addressing of long and short parts of time offset
  {
    int32_t l;
    uint8_t b[4];
  } timeRemaining;

  if (durationMs == 0) {
    // No duration specified, use a default based on the cycle duration
    durationMs = _listenCycleDurationMs + BURST_CYCLE_PADDING_MS;
  }

  timeRemaining.l = durationMs;

#ifdef RFM69_WL_DEBUG
  Serial.print("Sending burst for ");
  Serial.print(durationMs, DEC);
  Serial.println(" ms");
#endif

  setMode(RF69_MODE_TX);

  uint32_t startTime = millis();
  while(timeRemaining.l > 0) {
    // make sure packet is sent before putting more into the FIFO
    while ((readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFONOTEMPTY) != 0) {
      timeRemaining.l = (int32_t)durationMs - (millis() - startTime);
      if (timeRemaining.l < 0) {
        // We probably got stuck here, bail out
        break;
      }
    }

    noInterrupts();
    // write to FIFO
    select();
    SPI.transfer(REG_FIFO | 0x80);
    SPI.transfer(size + 4);      // two bytes for target and sender node, two bytes for the burst time remaining
    SPI.transfer(targetNode);
    SPI.transfer(_address);

    // We send the burst time remaining with the packet so the receiver knows how long to wait before trying to reply
    SPI.transfer(timeRemaining.b[0]);
    SPI.transfer(timeRemaining.b[1]);

    for (uint8_t i = 0; i < size; i++) {
      SPI.transfer(((uint8_t*) buffer)[i]);
    }

    unselect();
    interrupts();

    timeRemaining.l = (int32_t)durationMs - (millis() - startTime);
  }

  trans.revert();

  setMode(RF69_MODE_STANDBY);
  attachInterrupt(RF69_IRQ_NUM, RFM69::isr0, RISING);
}


