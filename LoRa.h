/*
 * LoRa.h
 *
 *  Created on: 08-Jul-2019
 *      Author: PRASHANT KURREY
 */

#ifndef LORA_H_
#define LORA_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "driverlib/ssi.h"
#include "driverlib/GPIO.h"

/*#ifdef ARDUINO_SAMD_MKRWAN1300
#define LORA_DEFAULT_SPI           SPI1
#define LORA_DEFAULT_SPI_FREQUENCY 250000
#define LORA_DEFAULT_SS_PIN        LORA_IRQ_DUMB
#define LORA_DEFAULT_RESET_PIN     -1
#define LORA_DEFAULT_DIO0_PIN      -1
#else
#define LORA_DEFAULT_SPI           SPI
#define LORA_DEFAULT_SPI_FREQUENCY 8E6
#define LORA_DEFAULT_SS_PIN        10
#define LORA_DEFAULT_RESET_PIN     9
#define LORA_DEFAULT_DIO0_PIN      2
#endif
*/
#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1

//class LoRaClass : public Stream {
//public:
  //LoRaClass();

  int LoRabegin(long frequency);
  void end();

  int beginPacket(int implicitHeader);
  int endPacket();

  int parsePacket(int size);
  int packetRssi();
  float packetSnr();
  long packetFrequencyError();

  // from Print
  //virtual
   int LoRawrite(uint8_t byte);
   int writedata(const uint8_t *buffer, size_t size);

  // from Stream
   int available();
   int read();
   int peek();
   void flush();

/*#ifndef ARDUINO_SAMD_MKRWAN1300
  void onReceive(void(*callback)(int));

  void receive(int size = 0);
#endif*/
  void idle();
  void LoRasleep();

  void setTxPower(int level);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void enableCrc();
  void disableCrc();

  // deprecated
  void enableCrc();
  void disableCrc();

 // byte random();

  //void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
  //void setSPI(SPIClass& spi);
  //void setSPIFrequency(uint32_t frequency);

  //void dumpRegisters(Stream& out);

  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();

  int getSpreadingFactor();
  long getSignalBandwidth();

  void setLdoFlag();

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

  static void onDio0Rise();
  //SPISettings _spiSettings;
  //SPIClass* _spi;
  int _ss;
  int _reset;
  int _dio0;
  long _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  void (*_onReceive)(int);


//extern LoRaClass LoRa;



#endif /* LORA_H_ */
