#pragma once
class IMPU
{
public:
	virtual void Init() = 0;
	virtual void getYawPitchRoll(float& y, float& p, float& r) = 0;

	//Calib
	virtual void Calibrate() = 0;
	virtual bool isCalibrated() = 0;
	virtual void resetCalibFlag() = 0;
};

