/*
 * LoRa.c
 *
 *  Created on: 08-Jul-2019
 *      Author: PRASHANT KURREY
 */


#include <LoRa.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "driverlib/ssi.h"


/* Example/Board Header files */
//#include "Board.h"

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255

#define DOSET (*((volatile unsigned long *)(0x400220D0)))
#define DOUT (*((volatile unsigned long *)(0x40022090)))
#define DSCLR (*((volatile unsigned long *)(0x400220A0)))


 //UART_Handle uart;
 //UART_Params uartParams;
uint8_t
SSIData( uint8_t data )
{   uint8_t getdata;
   // HWREG(SSI0_BASE+SSI_O_CR1) = SSI_CR1_SSE;
   // DSCLR=0x00000800;
   while(!(HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_TFE));
   HWREG(SSI0_BASE+SSI_O_DR) = data;
  // HWREG(SSI0_BASE + SSI_O_SR) |=SSI_SR_BSY;
   while((HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_BSY));
   getdata=HWREG(SSI0_BASE+SSI_O_DR);
//   DOUT=0x00000800;
  // HWREG(SSI0_BASE+SSI_O_CR1) &= ~(SSI_CR1_SSE);
   return getdata;
}

void
SSIConfigSet()
{  HWREG(SSI0_BASE+SSI_O_CR1) = 0x00000000;
//__delay_cycles(2400);
HWREG(SSI0_BASE+SSI_O_CPSR)= 0x00000001;
    HWREG(SSI0_BASE+SSI_O_CR0) = 0x00001807;
  //  __delay_cycles(2400);
     HWREG(SSI0_BASE+SSI_O_CR1) =0x00000002;

}



void end()
{
  // put in sleep mode
    LoRasleep();

  // stop SPI
  //_spi->end();
}

int beginPacket(int implicitHeader)
{
  // put in standby mode
  idle();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  // reset FIFO address and paload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);

  return 1;
}

int endPacket()
{
  // put in TX mode
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  // wait for TX done
  while((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
   // yield();
   //   __delay_cycles(2400);
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

  return 1;
}

int parsePacket(int size)
{
  int packetLength = 0;
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    idle();
  } else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    // put in single RX mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packetLength;
}

int packetRssi()
{
  //return (readRegister(REG_PKT_RSSI_VALUE) - 164);//(_frequency < 868E6 ? 164 : 157));
    return 164;
}

float LoRa_packetSnr()
{
  return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

/*long LoRa_packetFrequencyError()
{
  int32_t freqError = 0;
  freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & B111);
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

  if (readRegister(REG_FREQ_ERROR_MSB) & B1000) { // Sign bit is on
     freqError -= 524288; // B1000'0000'0000'0000'0000
  }

  const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
  const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

  return static_cast<long>(fError);
}
*/




int writedata(const uint8_t *buffer, size_t size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  };

  int i=0;
  // write data
  for ( ;i < size ; i++)
  {
    writeRegister(REG_FIFO, buffer[i]);
  };

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

int LoRawrite(uint8_t byte)
{   size_t size = sizeof(byte);
    //int currentLength = readRegister(REG_PAYLOAD_LENGTH);
     //writeRegister(REG_FIFO, byte);
      //writeRegister(REG_PAYLOAD_LENGTH, 2);

      //return currentLength;
 return writedata(&byte, size);
}

int available()
{
  return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int read()
{
  if (!available()) {
    return -1;
  }

  _packetIndex++;

  return readRegister(REG_FIFO);
}

int peek()
{
  if (!available()) {
    return -1;
  }

  // store current FIFO address
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

  // read
  uint8_t b = readRegister(REG_FIFO);

  // restore FIFO address
  writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

  return b;
}

void flush()
{
}

/*#ifndef ARDUINO_SAMD_MKRWAN1300
void onReceive(void(*callback)(int))
{
  _onReceive = callback;

  if (callback) {
    pinMode(_dio0, INPUT);

    writeRegister(REG_DIO_MAPPING_1, 0x00);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    attachInterrupt(digitalPinToInterrupt(_dio0), onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif
  }
}

void receive(int size)
{
  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}
#endif
*/

void idle()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRasleep()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void setTxPower(int level)
{
  /*if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }

    writeRegister(REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST*/
    if (level < 2) {
      level = 2;
    } else if (level >= 17) {
      level = 17;
    }

    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));

}

void setFrequency(long frequency)
{
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int getSpreadingFactor()
{
  return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

void setSpreadingFactor(int sf)
{
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
  setLdoFlag();
}

long getSignalBandwidth()
{
  uint8_t bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);
  switch (bw) {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3;
  }
}

void setSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
  setLdoFlag();
}

void setLdoFlag()
{
  // Section 4.1.1.5
  long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

  // Section 4.1.1.6
  bool ldoOn = symbolDuration > 16;

  uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
  //bitWrite(config3, 3, ldoOn);
  writeRegister(REG_MODEM_CONFIG_3, config3);
}

void setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void setPreambleLength(long length)
{
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void setSyncWord(int sw)
{
  writeRegister(REG_SYNC_WORD, sw);
}

void enableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void disableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

int random()
{
  return readRegister(REG_RSSI_WIDEBAND);
}
/*void setPins(int ss, int reset, int dio0)
{
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
}*/

/*void setSPI(SPIClass& spi)
{
  _spi = &spi;
}

void setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    out.print("0x");
    out.print(i, HEX);
    out.print(": 0x");
    out.println(readRegister(i), HEX);
  }
}*/

void explicitHeaderMode()
{
  _implicitHeaderMode = 0;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void implicitHeaderMode()
{
  _implicitHeaderMode = 1;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void handleDio0Rise()
{
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    if (_onReceive) {
      _onReceive(packetLength);
    }

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);
  }
}

uint8_t readRegister(uint8_t address)
{
  return singleTransfer(address & 0x7f, 0x00);
}

void writeRegister(uint8_t address, uint8_t value)
{
  singleTransfer(address | 0x80, value);
}

uint8_t singleTransfer(uint8_t address, uint8_t value)
{
  uint8_t response;
  DSCLR =0x00000880;
  SSIData(address);
  response = SSIData(value);
  DOUT |=0x00000880;
 /* digitalWrite(_ss, LOW);


  _spi->beginTransaction(_spiSettings);
  _spi->transfer(address);
  response = _spi->transfer(value);
  _spi->endTransaction();

  //digitalWrite(_ss, HIGH);
  //DOUT|=0x00000800;
*/
  return response;
}

void onDio0Rise()
{
  handleDio0Rise();
}

//LoRaClass LoRa;
int LoRabegin(long frequency)
{
  //pinMode(_ss, OUTPUT);

  // start SPI
//_spi->begin();
  SSIConfigSet();

  // check version
  uint8_t version = readRegister(REG_VERSION);

  if (version != 0x12) {
    return 0;
  }
  //uint8_t version = readRegister(0x43);
  // put in sleep mode
  LoRasleep();

  // set frequency
  setFrequency(frequency);

  // set base addresses
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

  // set LNA boost
  //writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);
  writeRegister(REG_LNA, 0x63);

  // set auto AGC
  writeRegister(REG_MODEM_CONFIG_3, 0x00);

  // set output power to 17 dBm
  setTxPower(17);

  // put in standby mode
  idle();

  return 1;
}
