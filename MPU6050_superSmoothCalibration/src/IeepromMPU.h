#pragma once
class IeepromMPU
{
public: 
	//Offset getters
	virtual int getXGyroOffset() = 0;
	virtual int getYGyroOffset() = 0;
	virtual int getZGyroOffset() = 0;
	virtual int getXAccelOffset() = 0;
	virtual int getYAccelOffset() = 0;
	virtual int getZAccelOffset() = 0;

	//Offset setters
	virtual int setXGyroOffset(int os) = 0;
	virtual int setYGyroOffset(int os) = 0;
	virtual int setZGyroOffset(int os) = 0;
	virtual int setXAccelOffset(int os) = 0;
	virtual int setYAccelOffset(int os) = 0;
	virtual int setZAccelOffset(int os) = 0;

	//Calib flag
	virtual bool getCalibFlag() = 0;
	virtual void setCalibFlag() = 0; //Sets it to true value
	virtual void resetCalibFlag() = 0; //Sets it otfalse value
};

