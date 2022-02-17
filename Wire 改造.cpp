extern "C" {
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "utility/twi.h"
}

#include "Wire.h"

// 初期化 //////////////////////////////////////////////////

uint8_t TwoWire::rxBuffer[BUFFER_LENGTH];
uint8_t TwoWire::rxBufferIndex = 0;
uint8_t TwoWire::rxBufferLength = 0;

uint8_t TwoWire::txAddress = 0;
uint8_t TwoWire::txBuffer[BUFFER_LENGTH];
uint8_t TwoWire::txBufferIndex = 0;
uint8_t TwoWire::txBufferLength = 0;

uint8_t TwoWire::transmitting = 0;
void (*TwoWire::user_onRequest)(void);
void (*TwoWire::user_onReceive)(int);

// Constructors ////////////////////////////////////////////////////////////////

TwoWire::TwoWire() {}

// Public Methods //////////////////////////////////////////////////////////////

void TwoWire::begin(void) {
  rxBufferIndex = 0;
  rxBufferLength = 0;

  txBufferIndex = 0;
  txBufferLength = 0;

  twi_init();
  twi_attachSlaveTxEvent(onRequestService);  // default callback must exist
  twi_attachSlaveRxEvent(onReceiveService);  // default callback must exist
}

void TwoWire::begin(uint8_t address) {
  begin();
  twi_setAddress(address);
}

void TwoWire::begin(int address) { begin((uint8_t)address); }

void TwoWire::end(void) { twi_disable(); }

void TwoWire::setClock(uint32_t clock) { twi_setFrequency(clock); }

void TwoWire::setWireTimeout(uint32_t timeout, bool reset_with_timeout) {
  twi_setTimeoutInMicros(timeout, reset_with_timeout);
}

//タイムアウトした時のフラグらしい
bool TwoWire::getWireTimeoutFlag(void) {
  return (twi_manageTimeoutFlag(false));
}

//上のフラグのクリア
void TwoWire::clearWireTimeoutFlag(void) { twi_manageTimeoutFlag(true); }

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity,
                             uint32_t iaddress, uint8_t isize,
                             uint8_t sendStop) {
  if (isize > 0) {
    /*内部アドレスを送信する。このモードでは、いくつかのデバイスの内部レジスタにアクセスするために、
    繰り返しスタートを送信することができます。この機能は、他のプロセッサ上のハードウェアTWIモジュール
    （例えばDueのTWI_IADRおよびTWI_MMRレジスタ）によって実行されます。???*/

    beginTransmission(address);

    // the maximum size of internal address is 3 bytes
    if (isize > 3) {
      isize = 3;
    }

    // write internal register address - most significant byte first
    while (isize-- > 0) write((uint8_t)(iaddress >> (isize * 8)));
    endTransmission(false);
  }

  // clamp to buffer length
  if (quantity > BUFFER_LENGTH) {
    quantity = BUFFER_LENGTH;
  }
  // perform blocking read into buffer
  uint8_t read = twi_readFrom(address, rxBuffer, quantity, sendStop);
  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = read;

  return read;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity,
                             uint8_t sendStop) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint32_t)0,
                     (uint8_t)0, (uint8_t)sendStop);
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}

void TwoWire::beginTransmission(uint8_t address) {
  // indicate that we are transmitting
  transmitting = 1;
  // set address of targeted slave
  txAddress = address;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
}

void TwoWire::beginTransmission(int address) {
  beginTransmission((uint8_t)address);
}

//
//	Originally, 'endTransmission' was an f(void) function.
//	It has been modified to take one parameter indicating
//	whether or not a STOP should be performed on the bus.
//	Calling endTransmission(false) allows a sketch to
//	perform a repeated start.
//
//	WARNING: Nothing in the library keeps track of whether
//	the bus tenure has been properly ended with a STOP. It
//	is very possible to leave the bus in a hung state if
//	no call to endTransmission(true) is made. Some I2C
//	devices will behave oddly if they do not see a STOP.
//
uint8_t TwoWire::endTransmission(uint8_t sendStop) {
  // transmit buffer (blocking)
  uint8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1, sendStop);
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
  // indicate that we are done transmitting
  transmitting = 0;
  return ret;
}

//	This provides backwards compatibility with the original
//	definition, and expected behaviour, of endTransmission
//
uint8_t TwoWire::endTransmission(void) { return endTransmission(true); }

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t TwoWire::write(uint8_t data) {
  if (transmitting) {
    // in master transmitter mode
    // don't bother if buffer is full
    if (txBufferLength >= BUFFER_LENGTH) {
      setWriteError();
      return 0;
    }
    // put byte in tx buffer
    txBuffer[txBufferIndex] = data;
    ++txBufferIndex;
    // update amount in buffer
    txBufferLength = txBufferIndex;
  } else {
    // in slave send mode
    // reply to master
    twi_transmit(&data, 1);
  }
  return 1;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t TwoWire::write(const uint8_t* data, size_t quantity) {
  if (transmitting) {
    // in master transmitter mode
    for (size_t i = 0; i < quantity; ++i) {
      write(data[i]);
    }
  } else {
    // in slave send mode
    // reply to master
    twi_transmit(data, quantity);
  }
  return quantity;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::available(void) { return rxBufferLength - rxBufferIndex; }

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::read(void) {
  int value = -1;

  // get each successive byte on each call
  if (rxBufferIndex < rxBufferLength) {
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::peek(void) {
  int value = -1;

  if (rxBufferIndex < rxBufferLength) {
    value = rxBuffer[rxBufferIndex];
  }

  return value;
}

void TwoWire::flush(void) {
  // XXX: to be implemented.
}

//受信これstatic
void TwoWire::onReceiveService(uint8_t* inBytes, int numBytes) {
  // コールバック関数を設定しない野郎用
  if (!user_onReceive) {
    return;
  }

  //受信バッファがマスターのrequestFrom()によって使用されている場合無視。
  if (rxBufferIndex < rxBufferLength) {
    return;
  }
  // copy twi rx buffer into local read buffer
  // this enables new reads to happen in parallel ???
  for (uint8_t i = 0; i < numBytes; ++i) {
    rxBuffer[i] = inBytes[i];
  }
  // set rx iterator vars ???
  rxBufferIndex = 0;
  rxBufferLength = numBytes;
  // alert user program
  user_onReceive(numBytes);
}

// behind the scenes function that is called when data is requested
void TwoWire::onRequestService(void) {
  if (!user_onRequest) {
    return;
  }

  // reset tx buffer iterator vars
  // !!! this will kill any pending pre-master sendTo() activity
  txBufferIndex = 0;
  txBufferLength = 0;
  // alert user program
  user_onRequest();
}

// sets function called on slave write
void TwoWire::onReceive(void (*function)(int)) { user_onReceive = function; }

// sets function called on slave read
void TwoWire::onRequest(void (*function)(void)) { user_onRequest = function; }

// Preinstantiate Objects //////////////////////////////////////////////////////

TwoWire Wire = TwoWire();
