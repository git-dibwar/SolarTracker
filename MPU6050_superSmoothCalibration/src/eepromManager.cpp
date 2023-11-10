#include "eepromManager.h"
#include "C:\Users\Narza\Documents\Arduino\hardware\arduino\avr\libraries\EEPROM\src\EEPROM.h"

int eepromManager::readIntFromEEPROM(int address) {
    int val;
    val = EEPROM.read(address);
    val |= EEPROM.read(address + 1) << 8;
    return val;
}



void eepromManager::writeIntToEEPROM(int address, int value) {
    EEPROM.update(address, value & 0xff); //write lower byte
    EEPROM.update(address + 1, value >> 8); //upper byte
}

//Offset getters
int eepromManager::getXGyroOffset()
{
    return readIntFromEEPROM(AddrxGyroE);
}

int eepromManager::getYGyroOffset()
{
    return readIntFromEEPROM(AddryGyroE);
}

int eepromManager::getZGyroOffset()
{
    return readIntFromEEPROM(AddrzGyroE);

}

int eepromManager::getXAccelOffset()
{
    return readIntFromEEPROM(AddrxAccelE);

}

int eepromManager::getYAccelOffset()
{
    return readIntFromEEPROM(AddryAccelE);

}

int eepromManager::getZAccelOffset()
{
    return readIntFromEEPROM(AddrzAccelE);

}

int eepromManager::setXGyroOffset(int os)
{
    writeIntToEEPROM(AddrxGyroE, os);
}

int eepromManager::setYGyroOffset(int os)
{
    writeIntToEEPROM(AddryGyroE, os);

}

int eepromManager::setZGyroOffset(int os)
{
    writeIntToEEPROM(AddrzGyroE, os);

}

int eepromManager::setXAccelOffset(int os)
{
    writeIntToEEPROM(AddrxAccelE, os);

}

int eepromManager::setYAccelOffset(int os)
{
    writeIntToEEPROM(AddryAccelE, os);
}

int eepromManager::setZAccelOffset(int os)
{
    writeIntToEEPROM(AddrzAccelE, os);
}

//Calib flag
bool eepromManager::getCalibFlag() {
    return (readIntFromEEPROM(CalibFlagAddr) == CalibFlagTrue);
}
void eepromManager::setCalibFlag()
{
    writeIntToEEPROM(CalibFlagAddr, CalibFlagTrue);

}

void eepromManager::resetCalibFlag()
{
    writeIntToEEPROM(CalibFlagAddr, CalibFlagFalse);
}
