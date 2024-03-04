/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;


void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceINA219State.i2cAddress			= i2cAddress;
	deviceINA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload)
{
	uint8_t		payloadByte[2], commandByte[1];
	i2c_status_t	returnValue;

	switch (deviceRegister)
	{
		case 0x00: case 0x05:
		{
			/* Calibration Register */
			break;
		}

		default:
		{
			/* Otherwise these arent valid registers to write to*/
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
			.address       = deviceINA219State.i2cAddress,
			.baudRate_kbps = gWarpI2cBaudRateKbps};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	warpEnableI2Cpins();

	commandByte[0] = deviceRegister;
	payloadByte[0] = (payload >> 8) & 0xFF; /* MSB first */
	payloadByte[1] = payload & 0xFF;        /* LSB */
	returnValue    = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		2,
		1000);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t cmdBuf[1] = {0xFF};
	i2c_status_t status1, status2;

	i2c_device_t slave =
		{
			.address       = deviceINA219State.i2cAddress,
			.baudRate_kbps = gWarpI2cBaudRateKbps};

	USED(numberOfBytes);

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	warpEnableI2Cpins();

	cmdBuf[0] = deviceRegister;

	status1 = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t*)deviceINA219State.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status1 != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}


	return kWarpStatusOK;
}

void
printSensorDataINA219(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Current, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			/*
			 *	Multiplying by 100 to convert to actual current in uA
			 */
			warpPrint(" %d,", (readSensorRegisterValueCombined * 100));
		}
	}

	i2cReadStatus |= readSensorRegisterINA219(kWarpSensorOutputRegisterINA219ShuntVoltage, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			/*
			 * Multiplying by 10 to give actual voltage in uV 
			 */
			warpPrint(" %d,", (readSensorRegisterValueCombined * 10));
		}
	}

	i2cReadStatus |= readSensorRegisterINA219(kWarpSensorOutputRegisterINA219LoadVoltage, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			/*
			 * Bitshifting left by 3 and multiplying by 4 to give actual voltage
			 */
			warpPrint(" %d,", 4*(readSensorRegisterValueCombined >>3));
		}
	}

	i2cReadStatus |= readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Power, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			/*
			 *	Multiplying by 100 to convert to actual power in W??
			 */
			warpPrint(" %d,", (readSensorRegisterValueCombined * 100));
		}
	}
}

void 
printCurrentMeasurementINA219(uint16_t shuntResistance, uint16_t samplesToAverage){

	// configuring sampling ratio 

	uint16_t conversionTime = 1000;
	uint8_t adcSetting = 0b1000;
	WarpStatus i2cWriteStatus;

	switch (samplesToAverage){
		case 1:{
			warpPrint("Averaging over 1 sample");
			conversionTime = 532;
			adcSetting = 0b1000;
			break;
		}
		case 2:{
			warpPrint("Averaging over 2 samples");
			conversionTime = 1060;
			adcSetting = 0b1001;
			break;
		}
		case 4:{
			warpPrint("Averaging over 4 samples");
			conversionTime = 2130;
			adcSetting = 0b1010;
			break;
		}
		case 8:{
			warpPrint("Averaging over 8 samples");
			conversionTime = 4260;
			adcSetting = 0b1011;
			break;
		}
		case 16:{
			warpPrint("Averaging over 16 samples");
			conversionTime = 8510;
			adcSetting = 0b1100;
			break;
		}
		case 32:{
			warpPrint("Averaging over 32 samples");
			conversionTime = 17020;
			adcSetting = 0b1101;
			break;
		}
		case 64:{
			warpPrint("Averaging over 64 samples");
			conversionTime = 34050;
			adcSetting = 0b1110;
			break;
		}
		case 128:{
			warpPrint("Averaging over 128 samples");
			conversionTime = 68100;
			adcSetting = 0b1111;
			break;
		}
		default:{
			warpPrint("Incorrect number of samples");
			return;
		}
	}


	//								 xxxxxxxxxSADCxxx
	uint16_t configRegisterValue = 0b0011100110000101;
	configRegisterValue 		|= adcSetting << 3;
	warpPrint(".\nSetting Config Register to 0x%04x. \n", configRegisterValue);

	i2cWriteStatus = writeSensorRegisterINA219(
		kWarpSensorConfigurationRegisterINA219, /* Configuration register	*/
		(configRegisterValue)
	);

	warpPrint("Waiting %d us for conversion. ", conversionTime);
	OSA_TimeDelay(conversionTime);
	warpPrint("Reading Registers: \n \n");

	

	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219ShuntVoltage, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" Current Measurement Failed ");
	}
	else{
		/*
			*	Dividing by shuntResistance (in mOhm) to convert to actual current in uA
			*/
		warpPrint("Shunt Voltage Register Value: 0x%02x%02x \n", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		warpPrint("         Shunt Voltage Value: %d  uV\n", (readSensorRegisterValueCombined * 10));
		warpPrint("          Calculated Current: %d uA \n", (readSensorRegisterValueCombined  * 10 * 1000 / shuntResistance));
	
	}
}
void
csvCurrentMeasurementINA219(uint16_t shuntResistance, uint16_t nReadings){

	// configuring sampling ratio 

	WarpStatus i2cWriteStatus;

	uint16_t conversionTime = 532;

	uint16_t configRegisterValue = 0b0011100110011111; // Both bus and shunt voltage set to read without sampling 
	warpPrint(".\nSetting Config Register to 0x%04x. \n", configRegisterValue);

	i2cWriteStatus = writeSensorRegisterINA219(
		kWarpSensorConfigurationRegisterINA219, /* Configuration register	*/
		(configRegisterValue)
	);

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	warpPrint("Load Current /uA, Load Voltage /mV, time /ms,\n");

	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;

	uint32_t startTime = OSA_TimeGetMsec();

	for(uint16_t tmp=0; tmp<nReadings; tmp++){

		// First measure shunt voltage (IE current)

		i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219ShuntVoltage, 2 /* numberOfBytes */);
		readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
		readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
		readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

		if (i2cReadStatus != kWarpStatusOK)
		{
			warpPrint("-,");
		}
		else{
			/*
				*	Dividing by shuntResistance (in mOhm) to convert to actual current in uA
				*/
			warpPrint("%d,", (readSensorRegisterValueCombined  * 10 * 1000 / shuntResistance));
		}

		// Then measure bus voltage

		i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219LoadVoltage, 2 /* numberOfBytes */);
		readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
		readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
		readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

		if (i2cReadStatus != kWarpStatusOK)
		{
			warpPrint("-,");
		}
		else{
			/*
			 * Bitshifting left by 3 and multiplying by 4 to give actual voltage
			 */
			warpPrint(" %d,", 4*(readSensorRegisterValueCombined >>3));
		}
		warpPrint("%d,", OSA_TimeGetMsec() - startTime);
		warpPrint("\n");
	}
}
