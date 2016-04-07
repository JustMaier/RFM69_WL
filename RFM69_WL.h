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
#define DEFAULT_LISTEN_RX_US 256
#define DEFAULT_LISTEN_IDLE_US 1000000
#define DEFAULT_RSSI_THRESHOLD 170
#define REGISTER_TRANSACTION_CAPACITY 32

class RFM69_WL: public RFM69 {
  public:
    // When we receive a packet in listen mode, this is the time left in the sender's burst.
    // You need to wait at least this long before trying to reply.
    static volatile uint16_t LISTEN_BURST_REMAINING_MS;

    RFM69_WL(uint8_t slaveSelectPin=RF69_SPI_CS, uint8_t interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false, uint8_t interruptNum=RF69_IRQ_NUM)
      : RFM69(slaveSelectPin, interruptPin, isRFM69HW, interruptNum)
      , _isHighSpeed(true)
      , _rssiThreshold(DEFAULT_RSSI_THRESHOLD)
      , _listenTransaction(RegisterTransaction(this))
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

    // RSSI threshold used in listen mode
    uint8_t rssiThreshold(void) {
      return _rssiThreshold;
    }

    void setRssiThreshold(uint8_t threshold) {
      _rssiThreshold = threshold;
    }

    //=======================================================================
    // New methods to handle listen mode & burst wake up
    //=======================================================================

    // rx and idle duration in microseconds
    bool setListenDurations(uint32_t& rxDuration, uint32_t& idleDuration);

    // The values passed to setListenDurations() may be slightly different to accomodate
    // what is allowed by the radio. This function returns the actual values used.
    void getListenDurations(uint32_t& rxDuration, uint32_t& idleDuration);

    void startListening(void);

    // Return to normal operation, and returns true if a message was received
    bool endListening(void);

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
    class RegisterTransaction
    {
    public:
      RegisterTransaction(RFM69_WL* radio): _radio(radio), _length(0) {
        memset(_pairs, 0, sizeof(RegisterPair) * REGISTER_TRANSACTION_CAPACITY);
      }

      bool pushReg(uint8_t reg, uint8_t value) {
        if (_length == REGISTER_TRANSACTION_CAPACITY) {
          return false;
        }

        uint8_t oldValue = _radio->readReg(reg);
        _radio->writeReg(reg, value);
        _pairs[_length++] = RegisterPair(reg, oldValue);
        return true;
      }

      void revert(void) {
        // Pop all of the registers in the original order
        for (uint8_t i = 0; i < _length; i++) {
          _radio->writeReg(_pairs[i].reg, _pairs[i].value);
        }

        _length = 0;
      }

      ~RegisterTransaction() {
        revert();
      }

    protected:
      struct RegisterPair {
        RegisterPair() : reg(0), value(0) {}
        RegisterPair(uint8_t reg, uint8_t value) : reg(reg), value(value) {}

        uint8_t reg;
        uint8_t value;
      };

      RFM69_WL* _radio;
      RegisterPair _pairs[REGISTER_TRANSACTION_CAPACITY];
      uint8_t _length;
    };

    friend class RegisterTransaction;

    virtual void receiveBegin() override;

    void abortListenMode(void);
    void resetListenReceive();

    bool _isHighSpeed;
    uint8_t _rssiThreshold;

    RegisterTransaction _listenTransaction;

    byte _rxListenCoef;
    byte _rxListenResolution;

    byte _idleListenCoef;
    byte _idleListenResolution;

    uint32_t _listenCycleDurationUs;
};

#endif
