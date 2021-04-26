#include "BME280_Sensor.h"
#define _VERBOSE_

int BME280_RegSet(u8 RegAddr, u8 RegMask, u8 RegData){
	u8 controlData, sendArray[2];
	int byteCount;
	byteCount = XIic_Send(XPAR_AXI_IIC_0_BASEADDR, BME280_IIC_ADDR, &RegAddr, 1, XIIC_REPEATED_START);
	byteCount = XIic_Recv(XPAR_AXI_IIC_0_BASEADDR, BME280_IIC_ADDR, &controlData, 1, XIIC_STOP);
	if(byteCount != 1){
		print("bmeregset bytecount != 1");
		return XST_FAILURE;
	}
	controlData &= RegMask;
	controlData |= RegData;
	sendArray[0] = RegAddr;
	sendArray[1] = controlData;
	byteCount = XIic_Send(XPAR_AXI_IIC_0_BASEADDR, BME280_IIC_ADDR, &sendArray, 2, XIIC_STOP);
	if(byteCount != 2){
		print("bmeregset bytecount != 2");
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}


float FloatAltitudeMeters(float currPres, float refPres)
{
	float heightOutput = 0;
	heightOutput = ((float)-44330.77)*(pow((currPres/refPres), 0.190263) - (float)1); //Corrected, see issue 30
	return heightOutput;
}


float setRefPresFromAlt(float currPres, float currAlt){
	float referencePres;
	referencePres = currPres / pow((1 + (-0.000022558 * currAlt)), 5.255882647);
	return referencePres;
}


int BME280_Initialize(){
	int Status;
	// u8 measureAddr = BME280_MEASUREMENTS_REG;
    u8 ctrlRegAddr;
    int ByteCount;
	u8 retData;

	Status = BME280_RegSet(BME280_RST_REG,
			~(BME280_RESET_MASK),
			(BME280_SOFT_RESET));

#ifdef _VERBOSE_
	if (Status != XST_SUCCESS) {
		print("FAILURE: BME280 Reset()\n");
	}
	else{
		print("SUCCESS: BME280 Reset()\n");
	}
#endif
	if (Status != XST_SUCCESS)  {
		 return XST_FAILURE;
	}

	/*
	* Set barometer standby time and hardware filter amount
	*/
	Status = BME280_RegSet(BME280_CONFIG_REG,
			~(BME280_CONFIG_T_SB_MASK | BME280_CONFIG_FILTER_MASK),
			(BME280_T_STANDBY_0_5_MS | BME280_FILTER_COEFF_8));

#ifdef _VERBOSE_
	if (Status != XST_SUCCESS) {
		print("FAILURE: BME280 Config Reg Set()\n");
	}
	else{
		print("SUCCESS: BME280 Config Reg Set()\n");
	}
#endif
	if (Status != XST_SUCCESS)  {
		 return XST_FAILURE;
	}

	/*
	* Set barometer humidity oversample amount
	*/
	Status = BME280_RegSet(BME280_CTRL_HUMIDITY_REG,
			~(BME280_CTRL_HUM_MASK),
			BME280_H_OVERSAMPLE_X8);

#ifdef _VERBOSE_
	if (Status != XST_SUCCESS) {
		print("FAILURE: BME280 Hum Ctrl Reg Set\n");
	}
	else{
		print("SUCCESS: BME280 Hum Ctrl Reg Set\n");
	}
#endif
	if (Status != XST_SUCCESS)  {
		 return XST_FAILURE;
	}

	/*
	* Set barometer mode, pressure and temperature oversample amounts
	*/
	Status = BME280_RegSet(BME280_CTRL_MEAS_REG,
			~(BME280_CTRL_MEAS_MODE_MASK | BME280_CTRL_MEAS_OSRS_P_MASK | BME280_CTRL_MEAS_OSRS_T_MASK),
			(BME280_CTRL_MEAS_MODE_NORMAL | BME280_P_OVERSAMPLE_X8 | BME280_T_OVERSAMPLE_X8));

#ifdef _VERBOSE_
	if (Status != XST_SUCCESS) {
		print("FAILURE: BME280 set ctrl_meas\n");
	}
	else{
		print("SUCCESS: BME280 set ctrl_meas\n");
	}
#endif
	if (Status != XST_SUCCESS)  {
		 return XST_FAILURE;
	}

	ctrlRegAddr = BME280_CTRL_MEAS_REG;
	ByteCount = XIic_Send(XPAR_AXI_IIC_0_BASEADDR, BME280_IIC_ADDR, &ctrlRegAddr, 1, XIIC_REPEATED_START);
	ByteCount = XIic_Recv(XPAR_AXI_IIC_0_BASEADDR, BME280_IIC_ADDR, &retData, 1, XIIC_STOP);

	return XST_SUCCESS;
}
