// **********************************************************************************
// Automatic Transmit Power Control class derived from RFM69 library.
// **********************************************************************************
// Copyright Thomas Studwell (2014,2015)
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
#ifndef RFM69_WL_h
#define RFM69_WL_h

#include "RFM69.h"

// By default, receive for 256uS in listen mode and idle for ~1s
#define  DEFAULT_LISTEN_RX_US 256
#define  DEFAULT_LISTEN_IDLE_US 1000000

class RFM69_WL: public RFM69 {
  public:
    // When we receive a packet in listen mode, this is the time left in the sender's burst.
    // You need to wait at least this long before trying to reply.
    static volatile uint16_t LISTEN_BURST_REMAINING_MS;

    RFM69_WL(uint8_t slaveSelectPin=RF69_SPI_CS, uint8_t interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false, uint8_t interruptNum=RF69_IRQ_NUM)
      : RFM69(slaveSelectPin, interruptPin, isRFM69HW, interruptNum)
      , _isHighSpeed(true)
      , _haveEncryptKey(false)
    {
      uint32_t rxDuration = DEFAULT_LISTEN_RX_US;
      uint32_t idleDuration = DEFAULT_LISTEN_IDLE_US;
      setListenDurations(rxDuration, idleDuration);
    }

    bool initialize(uint8_t freqBand, uint8_t address, uint8_t networkID=1);

    bool highSpeedListen(void) {
      return _isHighSpeed;
    }

    void setHighSpeedListen(bool highSpeed) {
      _isHighSpeed = highSpeed;
    }

    // Encryption is disabled in listen mode, and therefore when sending bursts as well.
    // We override this to save the key in order to restore it when returning to normal operation.
    void encrypt(const char* key);

    //=======================================================================
    // New methods to handle listen mode & burst wake up
    //=======================================================================

    // rx and idle duration in microseconds
    bool setListenDurations(uint32_t& rxDuration, uint32_t& idleDuration);

    // The values passed to setListenDurations() may be slightly different to accomodate
    // what is allowed by the radio. This function returns the actual values used.
    void getListenDurations(uint32_t& rxDuration, uint32_t& idleDuration);

    void startListening(void);
    void endListening(void);

    uint32_t listenCycleDurationUs() {
      return _listenCycleDurationUs;
    }

    // This repeatedly sends the message to the target node for the duration
    // of an entire listen cycle. The amount of time remaining in the burst
    // is transmitted to the receiver, and it is expected that the receiver
    // wait for the burst to end before attempting a reply.
    // See LISTEN_BURST_REMAINING_MS above.
    void sendBurst(uint8_t targetNode, void* buffer, uint8_t size);

    void listenIrq(void);

  protected:
    virtual void receiveBegin() override;

    void abortListenMode(void);
    bool reinitRadio(void);
    void resetListenReceive();

    bool _isHighSpeed;
    bool _haveEncryptKey;
    char _encryptKey[16];

    // Save these so we can reinitialize the radio after sending a burst
    // or exiting listen mode.
    uint8_t _freqBand;
    uint8_t _networkID;

    byte _rxListenCoef;
    byte _rxListenResolution;

    byte _idleListenCoef;
    byte _idleListenResolution;

    uint32_t _listenCycleDurationUs;
};

#endif
