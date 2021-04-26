//******************************************************************************
//	Engineer:		Christopher Jones
//	Project:		FPRock
//	Date:			03/09/2021
//	Description:	Header file for BME280_Sensor.c
//					Includes register address definitions,
//					function prototypes,
//					etc.
//
//******************************************************************************

#ifndef BME280_SENSOR_H
#define BME280_SENSOR_H

#include "xparameters.h"
#include "math.h"
#include "xil_types.h"
#include "xiic_l.h"
#include "xgpio.h"

//Sensor IIC Address:
//#define BME280_IIC_ADDR 0x77 //111011x
#define BME280_IIC_ADDR 0x76 //111011x

#define BME280_CTRL_MEAS_ADDR 0xF4
#define BME280_CTRL_MEAS_MODE 0x03

//Sensor modes:
#define MODE_SLEEP 0b00
#define MODE_FORCED 0b01
#define MODE_NORMAL 0b11

//Register names:
#define BME280_DIG_T1_LSB_REG			0x88
#define BME280_DIG_T1_MSB_REG			0x89
#define BME280_DIG_T2_LSB_REG			0x8A
#define BME280_DIG_T2_MSB_REG			0x8B
#define BME280_DIG_T3_LSB_REG			0x8C
#define BME280_DIG_T3_MSB_REG			0x8D
#define BME280_DIG_P1_LSB_REG			0x8E
#define BME280_DIG_P1_MSB_REG			0x8F
#define BME280_DIG_P2_LSB_REG			0x90
#define BME280_DIG_P2_MSB_REG			0x91
#define BME280_DIG_P3_LSB_REG			0x92
#define BME280_DIG_P3_MSB_REG			0x93
#define BME280_DIG_P4_LSB_REG			0x94
#define BME280_DIG_P4_MSB_REG			0x95
#define BME280_DIG_P5_LSB_REG			0x96
#define BME280_DIG_P5_MSB_REG			0x97
#define BME280_DIG_P6_LSB_REG			0x98
#define BME280_DIG_P6_MSB_REG			0x99
#define BME280_DIG_P7_LSB_REG			0x9A
#define BME280_DIG_P7_MSB_REG			0x9B
#define BME280_DIG_P8_LSB_REG			0x9C
#define BME280_DIG_P8_MSB_REG			0x9D
#define BME280_DIG_P9_LSB_REG			0x9E
#define BME280_DIG_P9_MSB_REG			0x9F
#define BME280_DIG_H1_REG				0xA1
#define BME280_CHIP_ID_REG				0xD0 //Chip ID
#define BME280_RST_REG					0xE0 //Softreset Reg
#define BME280_DIG_H2_LSB_REG			0xE1
#define BME280_DIG_H2_MSB_REG			0xE2
#define BME280_DIG_H3_REG				0xE3
#define BME280_DIG_H4_MSB_REG			0xE4
#define BME280_DIG_H4_LSB_REG			0xE5
#define BME280_DIG_H5_MSB_REG			0xE6
#define BME280_DIG_H6_REG				0xE7
#define BME280_CTRL_HUMIDITY_REG		0xF2 //Ctrl Humidity Reg
#define BME280_STAT_REG					0xF3 //Status Reg
#define BME280_CTRL_MEAS_REG			0xF4 //Ctrl Measure Reg
#define BME280_CONFIG_REG				0xF5 //Configuration Reg
#define BME280_MEASUREMENTS_REG			0xF7 //Measurements register start
#define BME280_PRESSURE_MSB_REG			0xF7 //Pressure MSB
#define BME280_PRESSURE_LSB_REG			0xF8 //Pressure LSB
#define BME280_PRESSURE_XLSB_REG		0xF9 //Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG		0xFA //Temperature MSB
#define BME280_TEMPERATURE_LSB_REG		0xFB //Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG		0xFC //Temperature XLSB
#define BME280_HUMIDITY_MSB_REG			0xFD //Humidity MSB
#define BME280_HUMIDITY_LSB_REG			0xFE //Humidity LSB

//Register Isolation Masks:
#define BME280_RESET_MASK				0xFF
#define BME280_CTRL_HUM_MASK			0x07
#define BME280_STATUS_MASK				0x09
#define BME280_CTRL_MEAS_MODE_MASK		0x03
#define BME280_CTRL_MEAS_OSRS_P_MASK	0x1C
#define BME280_CTRL_MEAS_OSRS_T_MASK	0xE0
#define BME280_CONFIG_SPI3W_EN_MASK		0x03
#define BME280_CONFIG_FILTER_MASK		0x1C
#define BME280_CONFIG_T_SB_MASK			0xE0

//Config Register Values
#define BME280_SOFT_RESET				0xB6

#define BME280_CTRL_MEAS_MODE_SLEEP		0x00
#define BME280_CTRL_MEAS_MODE_FORCED	0x01
#define BME280_CTRL_MEAS_MODE_NORMAL	0x03

#define BME280_P_OVERSAMPLE_X0			0x00
#define BME280_P_OVERSAMPLE_X1			0x04
#define BME280_P_OVERSAMPLE_X2			0x08
#define BME280_P_OVERSAMPLE_X4			0x0C
#define BME280_P_OVERSAMPLE_X8			0x10
#define BME280_P_OVERSAMPLE_X16			0x14

#define BME280_T_OVERSAMPLE_X0			0x00
#define BME280_T_OVERSAMPLE_X1			0x20
#define BME280_T_OVERSAMPLE_X2			0x40
#define BME280_T_OVERSAMPLE_X4			0x60
#define BME280_T_OVERSAMPLE_X8			0x80
#define BME280_T_OVERSAMPLE_X16			0xA0

#define BME280_H_OVERSAMPLE_X0			0x00
#define BME280_H_OVERSAMPLE_X1			0x01
#define BME280_H_OVERSAMPLE_X2			0x02
#define BME280_H_OVERSAMPLE_X4			0x03
#define BME280_H_OVERSAMPLE_X8			0x04
#define BME280_H_OVERSAMPLE_X16			0x05

#define BME280_T_STANDBY_0_5_MS			0x00
#define BME280_T_STANDBY_62_5_MS		0x20
#define BME280_T_STANDBY_125_0_MS		0x40
#define BME280_T_STANDBY_250_0_MS		0x60
#define BME280_T_STANDBY_500_0_MS		0x80
#define BME280_T_STANDBY_1000_0_MS		0xA0
#define BME280_T_STANDBY_10_0_MS		0xC0
#define BME280_T_STANDBY_20_0_MS		0xE0

#define BME280_FILTER_COEFF_OFF			0x00
#define BME280_FILTER_COEFF_2			0x04
#define BME280_FILTER_COEFF_4			0x08
#define BME280_FILTER_COEFF_8			0x0C
#define BME280_FILTER_COEFF_16			0x10

int BME280_RegSet(u8 RegAddr, u8 RegMask, u8 RegData);

float FloatAltitudeMeters(float currPres, float refPres);

float setRefPresFromAlt(float currPres, float currAlt);

#endif
