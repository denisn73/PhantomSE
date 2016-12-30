
#ifdef USE_AVRPROG
#include <SPI.h>

byte program(const byte b1, const byte b2 = 0, const byte b3 = 0, const byte b4 = 0);
byte program(const byte b1, const byte b2, const byte b3, const byte b4);
void writeFlash (unsigned long addr, const byte data);
byte readFlash(unsigned long addr);
void commitPage(unsigned long addr, bool showMessage = false);
void showProgress(unsigned long progressCount = 0);
void showHex(const byte b, const boolean newline = false, const boolean show0x = true);

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// Signatures **************************************************************************//

// structure to hold signature and other relevant data about each chip
typedef struct {
   byte sig[3];                 // chip signature
   char desc[14];               // fixed array size keeps chip names in PROGMEM
   unsigned long flashSize;     // how big the flash is (bytes)
   unsigned int baseBootSize;   // base bootloader size (others are multiples of 2/4/8)
   unsigned long pageSize;      // flash programming page size (bytes)
   byte fuseWithBootloaderSize; // ie. one of: lowFuse, highFuse, extFuse
   bool timedWrites;            // true if pollUntilReady won't work by polling the chip
} signatureType;

const unsigned long kb = 1024;
const byte NO_FUSE = 0xFF;

// copy of fuses/lock bytes found for this processor
byte fuses [5];

// meaning of positions in above array
enum {
      lowFuse,
      highFuse,
      extFuse,
      lockByte,
      calibrationByte };

// see Atmega datasheets
const signatureType signatures[] PROGMEM = {
//     signature        description   flash size   bootloader  flash  fuse     timed
//                                                     size    page    to      writes
//                                                             size   change
  // Attiny84 family
  { { 0x1E, 0x91, 0x0B }, "ATtiny24",   2 * kb,           0,   32,   NO_FUSE,  false },
  { { 0x1E, 0x92, 0x07 }, "ATtiny44",   4 * kb,           0,   64,   NO_FUSE,  false },
  { { 0x1E, 0x93, 0x0C }, "ATtiny84",   8 * kb,           0,   64,   NO_FUSE,  false },
  // Attiny85 family
  { { 0x1E, 0x91, 0x08 }, "ATtiny25",   2 * kb,           0,   32,   NO_FUSE,  false },
  { { 0x1E, 0x92, 0x06 }, "ATtiny45",   4 * kb,           0,   64,   NO_FUSE,  false },
  { { 0x1E, 0x93, 0x0B }, "ATtiny85",   8 * kb,           0,   64,   NO_FUSE,  false },
  // Atmega328 family
  { { 0x1E, 0x92, 0x0A }, "ATmega48PA",   4 * kb,         0,    64,  NO_FUSE,  false },
  { { 0x1E, 0x93, 0x0F }, "ATmega88PA",   8 * kb,       256,   128,  extFuse,  false },
  { { 0x1E, 0x94, 0x0B }, "ATmega168PA", 16 * kb,       256,   128,  extFuse,  false },
  { { 0x1E, 0x95, 0x0F }, "ATmega328P",  32 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x95, 0x14 }, "ATmega328",   32 * kb,       512,   128,  highFuse, false },
  // Atmega644 family
  { { 0x1E, 0x94, 0x0A }, "ATmega164P",   16 * kb,      256,   128,  highFuse, false },
  { { 0x1E, 0x95, 0x08 }, "ATmega324P",   32 * kb,      512,   128,  highFuse, false },
  { { 0x1E, 0x96, 0x0A }, "ATmega644P",   64 * kb,   1 * kb,   256,  highFuse, false },
  // Atmega2560 family
  { { 0x1E, 0x96, 0x08 }, "ATmega640",    64 * kb,   1 * kb,   256,  highFuse, false },
  { { 0x1E, 0x97, 0x03 }, "ATmega1280",  128 * kb,   1 * kb,   256,  highFuse, false },
  { { 0x1E, 0x97, 0x04 }, "ATmega1281",  128 * kb,   1 * kb,   256,  highFuse, false },
  { { 0x1E, 0x98, 0x01 }, "ATmega2560",  256 * kb,   1 * kb,   256,  highFuse, false },
  { { 0x1E, 0x98, 0x02 }, "ATmega2561",  256 * kb,   1 * kb,   256,  highFuse, false },
  // AT90USB family
  { { 0x1E, 0x93, 0x82 }, "At90USB82",    8 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x94, 0x82 }, "At90USB162",  16 * kb,       512,   128,  highFuse, false },
  // Atmega32U2 family
  { { 0x1E, 0x93, 0x89 }, "ATmega8U2",    8 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x94, 0x89 }, "ATmega16U2",  16 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x95, 0x8A }, "ATmega32U2",  32 * kb,       512,   128,  highFuse, false },
  // Atmega32U4 family -  (datasheet is wrong about flash page size being 128 words)
  { { 0x1E, 0x94, 0x88 }, "ATmega16U4",  16 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x95, 0x87 }, "ATmega32U4",  32 * kb,       512,   128,  highFuse, false },
  // ATmega1284P family
  { { 0x1E, 0x97, 0x05 }, "ATmega1284P", 128 * kb,   1 * kb,   256,  highFuse, false },
  // ATtiny4313 family
  { { 0x1E, 0x91, 0x0A }, "ATtiny2313A",   2 * kb,        0,    32,  NO_FUSE,  false },
  { { 0x1E, 0x92, 0x0D }, "ATtiny4313",    4 * kb,        0,    64,  NO_FUSE,  false },
  // ATtiny13 family
  { { 0x1E, 0x90, 0x07 }, "ATtiny13A",     1 * kb,        0,    32,  NO_FUSE,  false },
   // Atmega8A family
  { { 0x1E, 0x93, 0x07 }, "ATmega8A",      8 * kb,      256,    64,  highFuse, true },
  // ATmega64rfr2 family
  { { 0x1E, 0xA6, 0x02 }, "ATmega64rfr2",  256 * kb, 1 * kb,   256,  highFuse, false },
  { { 0x1E, 0xA7, 0x02 }, "ATmega128rfr2", 256 * kb, 1 * kb,   256,  highFuse, false },
  { { 0x1E, 0xA8, 0x02 }, "ATmega256rfr2", 256 * kb, 1 * kb,   256,  highFuse, false },
};  // end of signatures

// programming commands to send via SPI to the chip ************************************//
enum {
    progamEnable = 0xAC,
    // writes are preceded by progamEnable
    chipErase = 0x80,
    writeLockByte = 0xE0,
    writeLowFuseByte = 0xA0,
    writeHighFuseByte = 0xA8,
    writeExtendedFuseByte = 0xA4,
    pollReady = 0xF0,
    programAcknowledge = 0x53,
    readSignatureByte = 0x30,
    readCalibrationByte = 0x38,
    readLowFuseByte = 0x50,       readLowFuseByteArg2 = 0x00,
    readExtendedFuseByte = 0x50,  readExtendedFuseByteArg2 = 0x08,
    readHighFuseByte = 0x58,      readHighFuseByteArg2 = 0x08,
    readLockByte = 0x58,          readLockByteArg2 = 0x00,
    readProgramMemory = 0x20,
    writeProgramMemory = 0x4C,
    loadExtendedAddressByte = 0x4D,
    loadProgramMemory = 0x40,
};  // end of enum

enum {
    hexDataRecord,                   // 00
    hexEndOfFile,                    // 01
    hexExtendedSegmentAddressRecord, // 02
    hexStartSegmentAddressRecord,    // 03
    hexExtendedLinearAddressRecord,  // 04
    hexStartLinearAddressRecord      // 05
};  // types of record in .hex file

// actions to take
enum {
    checkFile,
    verifyFlash,
    writeToFlash,
};

unsigned long lowestAddress;
unsigned long highestAddress;
unsigned long extendedAddress;
unsigned long bytesWritten;
unsigned int  lineCount;
bool          gotEndOfFile;

const unsigned int ENTER_PROGRAMMING_ATTEMPTS = AVRISP_ATTEMPTS;
// if signature found in signature table, this is its index
unsigned int errors;
int foundSig = -1;
bool was_programming = false;
byte lastAddressMSB = 0;
// copy of current signature entry for matching processor
signatureType currentSignature;
unsigned int progressBarCount;
unsigned long pagesize;
unsigned long pagemask;
unsigned long oldPage;
const unsigned long NO_PAGE = 0xFFFFFFFF;
// number of items in an array
#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

byte processorsCount = NUMITEMS(signatures);

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// Programming_Utils *******************************************************************//

bool wasAVRprogramming() {
  bool state = was_programming;
  was_programming = false;
  return state;
}

// show one progress symbol, wrap at 64 characters
void showProgress(unsigned long progressCount) {
  if( (progressBarCount++ % 64 == 0) || (progressCount==currentSignature.flashSize) ) {
    progressCount = ((progressCount+1) * 100) / currentSignature.flashSize;
    if(progressCount>0) {
      printDebug(" [ ");
      printDebug(progressCount);
      printlnDebug("% ]");
    } else printlnDebug();
    yield();
  }
  printDebug("#");  // progress bar
}  // end of showProgress

// clear entire temporary page to 0xFF in case we don't write to all of it 
void clearPage() {
  unsigned int len = currentSignature.pageSize;
  for(unsigned int i = 0; i < len; i++) writeFlash(i, 0xFF);
} // end of clearPage

// write data to temporary buffer, ready for committing  
void writeData(const unsigned long addr, const byte * pData, const int length) {
  for(int i = 0; i < length; i++) { // write each byte
    unsigned long thisPage = (addr + i) & pagemask;
    // page changed? commit old one
    if(thisPage != oldPage && oldPage != NO_PAGE) commitPage(oldPage);
    oldPage = thisPage;               // now this is the current page    
    writeFlash (addr + i, pData [i]); // put byte into work buffer
  }  // end of for
}  // end of writeData

void verifyData (const unsigned long addr, const byte * pData, const int length) {
  // check each byte
  for (int i = 0; i < length; i++) {
    unsigned long thisPage = (addr + i) & pagemask;
    // page changed? show progress
    if(thisPage != oldPage && oldPage != NO_PAGE) showProgress();
    oldPage = thisPage; // now this is the current page
    byte found = readFlash (addr + i);
    byte expected = pData [i];
    if (found != expected) {
      if (errors <= 3) {
        printDebug("Verification error at address ");
        printDebug(addr + i, HEX);
        printDebug(". Got: ");
        showHex(found);
        printDebug(" Expected: ");
        showHex(expected, true);
      }  // end of haven't shown 100 errors yet
      if (errors == 3) printDebug("... ");
      errors++;
    }  // end if error
  }  // end of for
  if(errors) {
    printDebug(errors);
    printlnDebug(" errors");
  }
}  // end of verifyData

// show a byte in hex with leading zero and optional newline
void showHex(const byte b, const boolean newline, const boolean show0x) {
  if(show0x) printDebug("0x");
  // try to avoid using sprintf
  char buf [4];
  buf [0] = ((b >> 4) & 0x0F) | '0';
  buf [1] = (b & 0x0F) | '0';
  buf [2] = ' ';
  buf [3] = 0;
  if (buf [0] > '9') buf [0] += 7;
  if (buf [1] > '9') buf [1] += 7;
  printDebug(buf);
  if(newline) printlnDebug();
}  // end of showHex

// convert two hex characters into a byte
// returns true if error, false if OK
bool hexConv (char * (& pStr), byte & b, byte index) {
  if(!isxdigit (pStr[0]) || !isxdigit (pStr[1])) {
    printDebug("Invalid hex digits [");
    printDebug(index);
    printDebug("] : ");
    printDebug(pStr[0]);
    printlnDebug(pStr[1]);
    return true;
  } // end not hex
  b = *pStr++ - '0';
  if (b > 9) b -= 7;
  // high-order nybble
  b <<= 4;
  byte b1 = *pStr++ - '0';
  if (b1 > 9) b1 -= 7;
  b |= b1;
  return false;  // OK
}  // end of hexConv

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// ICSP_Utils **************************************************************************//
// which program instruction writes which fuse
const byte fuseCommands[4] = { writeLowFuseByte, writeHighFuseByte, writeExtendedFuseByte, writeLockByte };
// execute one programming instruction ... b1 is command, b2, b3, b4 are arguments
// processor may return a result on the 4th transfer, this is returned.
byte program(const byte b1, const byte b2, const byte b3, const byte b4) {
  SPI.transfer(b1);
  SPI.transfer(b2);
  SPI.transfer(b3);
  return SPI.transfer(b4);
} // end of program

// read a byte from flash memory
byte readFlash(unsigned long addr) {
  byte high = (addr & 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address
  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if(MSB != lastAddressMSB) {
    program (loadExtendedAddressByte, 0, MSB);
    lastAddressMSB = MSB;
  }  // end if different MSB
  return program(readProgramMemory | high, highByte (addr), lowByte (addr));
} // end of readFlash

// write a byte to the flash memory buffer (ready for committing)
void writeFlash (unsigned long addr, const byte data) {
  byte high = (addr & 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address
  program(loadProgramMemory | high, 0, lowByte (addr), data);
} // end of writeFlash

// poll the target device until it is ready to be programmed
void pollUntilReady() {
  if(currentSignature.timedWrites) delay(10); // at least 2 x WD_FLASH which is 4.5 mS
  else {
    while((program(pollReady) & 1) == 1) {}   // wait till ready
  }  // end of if
}  // end of pollUntilReady

byte readFuse(const byte which) {
  switch (which) {
    case lowFuse:         return program(readLowFuseByte, readLowFuseByteArg2);
    case highFuse:        return program(readHighFuseByte, readHighFuseByteArg2);
    case extFuse:         return program(readExtendedFuseByte, readExtendedFuseByteArg2);
    case lockByte:        return program(readLockByte, readLockByteArg2);
    case calibrationByte: return program(readCalibrationByte);
  }  // end of switch
 return 0;
}  // end of readFuse

// write specified value to specified fuse/lock byte
void writeFuse (const byte newValue, const byte whichFuse) {
  if(newValue == 0) return;  // ignore
  program(progamEnable, fuseCommands[whichFuse], 0, newValue);
  pollUntilReady();
}  // end of writeFuse

void readSignature(byte sig[3])  {
  for(byte i = 0; i < 3; i++) sig[i] = program(readSignatureByte, 0, i);
  // make sure extended address is zero to match lastAddressMSB variable
  program(loadExtendedAddressByte, 0, 0);
  lastAddressMSB = 0;
}  // end of readSignature

// commit page to flash memory
void commitPage(unsigned long addr, bool showMessage) {
  if(showMessage) {
    printDebug("Committing page starting at 0x");
    if(addr<16) printDebug("0");
    printlnDebug(addr, HEX);
  } else showProgress();
  addr >>= 1;  // turn into word address
  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if(MSB != lastAddressMSB) {
    program(loadExtendedAddressByte, 0, MSB);
    lastAddressMSB = MSB;
  }  // end if different MSB
  program(writeProgramMemory, highByte (addr), lowByte (addr));
  pollUntilReady();
  clearPage();  // clear ready for next page full
}  // end of commitPage

void eraseMemory() {
  printDebug("Erasing chip... ");
  program(progamEnable, chipErase);   // erase it
  delay(20);  // for Atmega8
  pollUntilReady();
  clearPage();  // clear temporary page
  printlnDebug("OK!");
}  // end of eraseMemory

inline bool _resetLevel(bool reset_state) { return reset_state == false; }

void setReset(bool rst) {
  digitalWrite(AVRISP_RESET_PIN, _resetLevel(rst));
}

// put chip into programming mode
bool startProgramming() {
  byte confirm;
  unsigned int timeout = 0;
  printDebug("Attempting to enter ICSP programming mode... ");
  do { delay(100); // regrouping pause
    was_programming = true;
    pinMode(AVRISP_RESET_PIN, OUTPUT);
    setReset(true);
    SPI.begin();
    SPI.setFrequency(AVRISP_SPI_FREQ);
    SPI.setHwCs(false);
    SPI.transfer(0x00);
    digitalWrite(AVRISP_RESET_PIN, _resetLevel(false));
    delayMicroseconds(50);
    digitalWrite(AVRISP_RESET_PIN, _resetLevel(true));
    delay(30);
    //spi_transaction(progamEnable, programAcknowledge, 0x00, 0x00);
    //pmode = 1;
    SPI.transfer(progamEnable);
    SPI.transfer(programAcknowledge);
    confirm = SPI.transfer(0);
    SPI.transfer(0);
    if(confirm != programAcknowledge) {
      printDebug(".");
      if(timeout++ >= ENTER_PROGRAMMING_ATTEMPTS) {
        printlnDebug("Failed! Double-check wiring!");
        return false;
      } // end of too many attempts
    } // end of not entered programming mode
  } while (confirm != programAcknowledge);
  printlnDebug("OK!");
  return true;
}

void stopProgramming() {
  pinMode(AVRISP_RESET_PIN, INPUT);
  digitalWrite(AVRISP_RESET_PIN, HIGH);
  SPI.end();
  setReset(true);
  delay(10);
  setReset(false);
  //pmode = 0;
  printlnDebug("Programming mode off.");
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// Flash process hex line **************************************************************//
bool processLine(char * pLine, byte _size, const byte action) {
  byte maxHexData = 40;
  byte sizeHexData = (_size - 1) / 2;
  byte hexBuffer[sizeHexData];
  int bytesInLine = 0;
  if(*pLine++ != ':') {
     printlnDebug("Line does not start with ':' character."); 
     return false;  // error
  }
  if(action == checkFile) { if(lineCount++ % 40 == 0) showProgress(); }
  // convert entire line from ASCII into binary
  while(isxdigit(*pLine) && bytesInLine<sizeHexData) {
    if(bytesInLine >= maxHexData) { // can't fit?
      printlnDebug("Line too long to process.");
      return false;
    } // end if too long
    if(hexConv(pLine, hexBuffer[bytesInLine], bytesInLine)) {
      for(byte i=0; i<bytesInLine; i++) showHex(hexBuffer[i], false, false);
      printlnDebug();
      return false;
    }
    bytesInLine++;
  }
  if(bytesInLine < 5) {
    printlnDebug("Line too short.");
    return false;  
  }
  // sumcheck it  
  byte sumCheck = 0;
  for(int i = 0; i < (bytesInLine-1); i++) sumCheck += hexBuffer[i];
  // 2's complement
  sumCheck = ~sumCheck + 1;
  // check sumcheck
  if(sumCheck != hexBuffer[bytesInLine - 1]) {
    printDebug("Sumcheck error. Expected: ");
    showHex(sumCheck, true, false);
    printDebug(", got: ");
    showHex(hexBuffer[bytesInLine - 1], true, true);
    for(byte i=0; i<bytesInLine; i++) showHex(hexBuffer[i], false, false);
    printlnDebug();
    return false;
  }
  // length of data (eg. how much to write to memory)
  byte len = hexBuffer[0];
  // the data length should be the number of bytes, less
  // length / address(2) / transaction type / sumcheck
  if(len != (bytesInLine - 5)) {
    printDebug("Line not expected length. Expected ");
    printDebug(len, DEC);
    printDebug(" bytes, got ");
    printDebug(bytesInLine - 5, DEC);
    printlnDebug(" bytes.");
    return false;
  }
  unsigned long addrH = hexBuffer[1];
  unsigned long addrL = hexBuffer[2];
  unsigned long addr = addrL | (addrH << 8);
  byte recType = hexBuffer[3];
  switch (recType) {
    // stuff to be written to memory
    case hexDataRecord:
      lowestAddress  = _min(lowestAddress, addr + extendedAddress);
      highestAddress = _max(lowestAddress, addr + extendedAddress + len - 1);
      bytesWritten += len;    
      switch(action) {
        case checkFile:  // nothing much to do, we do the checks anyway
          break;
        case verifyFlash:
          verifyData(addr + extendedAddress, &hexBuffer [4], len);
          break;        
        case writeToFlash:
          writeData(addr + extendedAddress, &hexBuffer[4], len);
          break;      
        } // end of switch on action
      break;    
    // end of data
    case hexEndOfFile:  gotEndOfFile = true; break;    
    // we are setting the high-order byte of the address
    case hexExtendedSegmentAddressRecord:
      extendedAddress = ((unsigned long) hexBuffer [4]) << 12;
      break;      
    // ignore these, who cares?
    case hexStartSegmentAddressRecord:
    case hexExtendedLinearAddressRecord:
    case hexStartLinearAddressRecord:
    break;      
    default:  
      printDebug("Cannot handle record type: ");
      printlnDebug(recType, DEC);
      return false;
  }
  return true;
}  // end of processLine

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// Main programm utils *****************************************************************//

void getSignature() {
  foundSig = -1;
  byte sig[3];
  printDebug("Signature = ");
  readSignature(sig);
  for(byte i = 0; i < 3; i++) showHex(sig[i]);
  printlnDebug();
  for(unsigned int j = 0; j < NUMITEMS(signatures); j++) {
    memcpy_P (&currentSignature, &signatures [j], sizeof currentSignature);
    if(memcmp(sig, currentSignature.sig, sizeof sig) == 0) {
      foundSig = j;
      printDebug("Processor = ");
      printlnDebug(currentSignature.desc);
      printDebug("Flash memory size = ");
      printDebug(currentSignature.flashSize, DEC);
      printlnDebug(" bytes.");
      return;
    }  // end of signature found
  }  // end of for each signature
  printlnDebug("Unrecogized signature.");
}  // end of getSignature

void readFlashContents() {
  printlnDebug("[Read flash start]");
  progressBarCount = 0;
  pagesize = currentSignature.pageSize;
  pagemask = ~(pagesize - 1);
  oldPage = NO_PAGE;
  byte lastMSBwritten = 0;
  byte memBuf [16];
  bool allFF;
  byte sumCheck;
  char linebuf [50];
  unsigned int i;
  for(unsigned long address = 0; address < currentSignature.flashSize; address += sizeof memBuf) {
    unsigned long thisPage = address & pagemask;
    // page changed? show progress
    if(thisPage != oldPage && oldPage != NO_PAGE) showProgress(address+1);
    // now this is the current page
    oldPage = thisPage;
    // don't write lines that are all 0xFF
    allFF = true;    
    for(i = 0; i < sizeof memBuf; i++) {
      memBuf [i] = readFlash (address + i); 
      if(memBuf [i] != 0xFF) allFF = false;
    }  // end of reading 16 bytes
    if(allFF) continue;
    byte MSB = address >> 16;
    if(MSB != lastMSBwritten) {
      sumCheck = 2 + 2 + (MSB << 4);
      sumCheck = ~sumCheck + 1;      
      // hexExtendedSegmentAddressRecord (02)
      //printfDebug(linebuf, ":02000002%02X00%02X\r\n", MSB << 4, sumCheck);    
      //myFile.print (linebuf);
      lastMSBwritten = MSB;
    }  // end if different MSB
    sumCheck = 16 + lowByte (address) + highByte (address);
    //printfDebug(linebuf, ":10%04X00", (unsigned int) address & 0xFFFF);
    for(i = 0; i < sizeof memBuf; i++) {
      //printfDebug(&linebuf [(i * 2) + 9] , "%02X",  memBuf [i]);
      sumCheck += memBuf [i];
    }  // end of reading 16 bytes
    // 2's complement
    sumCheck = ~sumCheck + 1;
    // append sumcheck
    //printfDebug(&linebuf [(sizeof memBuf * 2) + 9] , "%02X\r\n",  sumCheck);
    yield();
  } // end of reading flash
  showProgress(currentSignature.flashSize);
  //printlnDebug();  // finish off progress bar
  printlnDebug("[Read flash end]");
}

#endif
