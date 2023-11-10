#pragma once
#include "IeepromMPU.h"

class eepromManager: public IeepromMPU
{
	enum OffsetAddr {
		AddrxGyroE = 2,
		AddryGyroE = 4,
		AddrzGyroE = 6,
		AddrxAccelE = 8,
		AddryAccelE = 10,
		AddrzAccelE = 12
		
	};
	const int CalibFlagAddr = 0;
	const int CalibFlagTrue = 77;
	const int CalibFlagFalse = 0;

	int readIntFromEEPROM(int address);
	void writeIntToEEPROM(int addresss, int value);

	// Inherited via IeepromMPU
	virtual int getXGyroOffset() override;
	virtual int getYGyroOffset() override;
	virtual int getZGyroOffset() override;
	virtual int getXAccelOffset() override;
	virtual int getYAccelOffset() override;
	virtual int getZAccelOffset() override;

	virtual int setXGyroOffset(int os) override;
	virtual int setYGyroOffset(int os) override;
	virtual int setZGyroOffset(int os) override;
	virtual int setXAccelOffset(int os) override;
	virtual int setYAccelOffset(int os) override;
	virtual int setZAccelOffset(int os) override;

	virtual bool getCalibFlag() override;
	virtual void setCalibFlag() override;
	virtual void resetCalibFlag() override;

};

