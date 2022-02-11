/***************************************************************************//**
 *   @file   AD7190.c
 *   @brief  Implementation of AD7190 Driver.
 *   @author DNechita (Dan.Nechita@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 903
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "AD7190.h"      // AD7190 definitions.
#include "main.h"       // AD7190 definitions.

 Ad7190_Def  reader;
/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue - Data value to write.
 * @param bytesNumber - Number of bytes to be written.
 * @param modifyCS - Allows Chip Select to be modified.
 *
 * @return none.
*******************************************************************************/
	unsigned long reg[10];
	
void sensor_init(void)
{
	reader.spi = &hspi1;
  reader.sensor.PT100_oil.select = false;
	reader.sensor.PT100_water.select = false;
  reader.sensor.PT100_steam.select = false;
  reader.sensor.pressure.select = false;
 //**********************************************************//
	 // Give the Pins and Ports Name  
	reader.sensor.PT100_oil.port = PT100_STEAM_GPIO_Port;
	reader.sensor.PT100_oil.pin = PT100_STEAM_Pin;
	reader.sensor.PT100_water.port = PT100_WATER_GPIO_Port; 
  reader.sensor.PT100_water.pin = PT100_WATER_Pin;
	reader.sensor.PT100_steam.port =  PT100_OIL_GPIO_Port;
  reader.sensor.PT100_steam.pin = PT100_OIL_Pin;	
	
	//*******************************************************//
	
		//FIRST OF ALL THE PINS ARE "1"
	HAL_GPIO_WritePin(reader.sensor.PT100_oil.port , reader.sensor.PT100_oil.pin , GPIO_PIN_SET);
	HAL_GPIO_WritePin(reader.sensor.PT100_water.port , reader.sensor.PT100_water.pin , GPIO_PIN_SET);
	HAL_GPIO_WritePin(reader.sensor.PT100_steam.port , reader.sensor.PT100_steam.pin , GPIO_PIN_SET);
	
 //**********************************************************//	
	
	AD7190_Init();
	int i = 0;
	 //Read the registers value From AD7190
	reg[i++] = AD7190_GetRegisterValue(AD7190_REG_STAT, 1, 1);
	reg[i++] = AD7190_GetRegisterValue(AD7190_REG_MODE, 3, 1);
	reg[i++] = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
	reg[i++] = AD7190_GetRegisterValue(AD7190_REG_DATA, 3, 1);
	reg[i++] = AD7190_GetRegisterValue(AD7190_REG_ID, 1, 1);
	reg[i++] = AD7190_GetRegisterValue(AD7190_REG_GPOCON, 1, 1);
	reg[i++] = AD7190_GetRegisterValue(AD7190_REG_OFFSET, 3, 1);
	reg[i++] = AD7190_GetRegisterValue(AD7190_REG_FULLSCALE, 3, 1); 
  AD7190_RangeSetup(0, 0);
     //Select the Channle for read the voltage 	
			//*******************************************//
	   unsigned long command = 0x0;
	 
	   command = AD7190_MODE_SEL(AD7190_MODE_CONT) |  AD7190_MODE_RATE(0x00) | AD7190_MODE_CLKSRC(AD7190_CLK_INT); 
		
		 AD7190_SetRegisterValue(AD7190_REG_MODE, command, 3, 0); // CS is not modified.	
}
void Ad7190_Handler(void)
{
  if(reader.sensor.PT100_oil.select)
	{
		if(!reader.sensor.PT100_oil.enable_channle)
		{
			reader.sensor.PT100_oil.enable_channle = true;
			AD7190_ChannelSelect(AD7190_CH_AIN3P_AINCOM);
			reader.sensor.PT100_oil.tickstart = HAL_GetTick();
		}
		if(HAL_GetTick() - reader.sensor.PT100_oil.tickstart > 100)
		{
		 HAL_GPIO_WritePin(reader.sensor.PT100_oil.port , reader.sensor.PT100_oil.pin , GPIO_PIN_RESET);
	   uint32_t value = GetInput();
	//****************************** For Calcute The Voltage ********************************
			reader.sensor.PT100_oil.value = (float)((value - 0x800000) * vref_adc)/0x7fffff;				
			//******************************** For Calcute the Resistance ***********************
			reader.sensor.PT100_oil.Resistance = (reader.sensor.PT100_oil.value * Rref) / (vrefpt - reader.sensor.PT100_oil.value);	
		  //******************* For Calcute the Temprature **********************************
			reader.sensor.PT100_oil.select = false;
			reader.sensor.PT100_water.select = true;
			reader.sensor.PT100_steam.select = false;
			reader.sensor.pressure.select = false;
			reader.sensor.PT100_water.enable_channle = false;

		}
	}
	
	if(reader.sensor.PT100_water.select)
	{
		if(!reader.sensor.PT100_water.enable_channle)
		{
			reader.sensor.PT100_water.enable_channle = true;
			AD7190_ChannelSelect(AD7190_CH_AIN2P_AINCOM);
			reader.sensor.PT100_water.tickstart = HAL_GetTick();
		}
		if(HAL_GetTick() - reader.sensor.PT100_water.tickstart>100)
		{
			HAL_GPIO_WritePin(reader.sensor.PT100_water.port ,reader.sensor.PT100_water.pin , GPIO_PIN_RESET);
			uint32_t value_2 = GetInput();
			//****************************** For Calcute The Voltage ********************************
			reader.sensor.PT100_water.value = (float)((value_2 - 0x800000) * vref_adc)/0x7fffff;				
			//******************************** For Calcute the Resistance ***********************
			reader.sensor.PT100_water.Resistance = (reader.sensor.PT100_water.value * Rref) / (vrefpt - reader.sensor.PT100_water.value);	
		  //******************* For Calcute the Temprature **********************************
			reader.sensor.PT100_water.select = false;
			reader.sensor.PT100_oil.select = false;
			reader.sensor.PT100_steam.select = false;
			reader.sensor.pressure.select = false;
			reader.sensor.PT100_steam.select = true;
			reader.sensor.PT100_steam.enable_channle = false;
		}
	}
  if(reader.sensor.PT100_steam.select)
	 {
		 if(!reader.sensor.PT100_steam.enable_channle)
		 {
			 reader.sensor.PT100_steam.enable_channle = true;
			 AD7190_ChannelSelect(AD7190_CH_AIN1P_AINCOM);
			 reader.sensor.PT100_steam.tickstart = HAL_GetTick();				 
		 }
		 if(HAL_GetTick() - reader.sensor.PT100_steam.tickstart > 100)
		 {
			HAL_GPIO_WritePin(reader.sensor.PT100_steam.port , reader.sensor.PT100_steam.pin , GPIO_PIN_RESET);
			 uint32_t value_3 = GetInput();
			//****************************** For Calcute The Voltage ********************************
			reader.sensor.PT100_steam.value = (float)((value_3 - 0x800000) * vref_adc)/0x7fffff;				
			//******************************** For Calcute the Resistance ***********************
			reader.sensor.PT100_steam.Resistance = (reader.sensor.PT100_steam.value * Rref) / (vrefpt - reader.sensor.PT100_steam.value);	
			//******************* For Calcute the Temprature **********************************
			reader.sensor.PT100_water.select = false;
			reader.sensor.PT100_oil.select = false;
			reader.sensor.PT100_steam.select = false;
			reader.sensor.pressure.select = false;
			reader.sensor.pressure.select = true;
			reader.sensor.pressure.enable_channle = false;
		 }
	}
	if(reader.sensor.pressure.select)
	{
			if(!reader.sensor.pressure.enable_channle)
			{
				reader.sensor.pressure.enable_channle = true;
				AD7190_ChannelSelect(AD7190_CH_AIN4P_AINCOM);
				reader.sensor.pressure.tickstart = 	HAL_GetTick();
			}
		  if(HAL_GetTick() - reader.sensor.pressure.tickstart > 100)
			{			
				uint32_t value_4 = GetInput();
				reader.sensor.pressure.value = (float)((value_4 - 0x800000) * vref_adc)/0x7fffff;
				reader.sensor.PT100_water.select = false;
				reader.sensor.PT100_oil.select = false;
				reader.sensor.PT100_steam.select = false;
				reader.sensor.pressure.select = false;				
       reader.sensor.PT100_oil.select = true;	
				reader.sensor.PT100_oil.enable_channle = false;
			}					
		}
	}
long GetInput(void)
{
		unsigned long command = 0x0;
    long regData = 0x0;
    AD7190_WaitRdyGoLow();  
	regData = AD7190_GetRegisterValue(AD7190_REG_DATA, 3 , 0);
    ADI_PART_CS_HIGH;
	
    return regData;
}
unsigned char SPI_Read(unsigned char slaveDeviceId,unsigned char* data,unsigned char bytesNumber)
{
    /* Add your code here. */
	HAL_SPI_Receive(reader.spi,data,bytesNumber,0xffff);
}
/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char SPI_Write(unsigned char slaveDeviceId,unsigned char* data, unsigned char bytesNumber)
{
    /* Add your code here. */
	int cnt = 0;
	HAL_SPI_Transmit(reader.spi,data,bytesNumber,0xffff);
}
void AD7190_SetRegisterValue(unsigned char registerAddress,unsigned long registerValue,unsigned char bytesNumber,unsigned char modifyCS)
{
    unsigned char writeCommand[5] = {0, 0, 0, 0, 0};
    unsigned char* dataPointer    = (unsigned char*)&registerValue;
    unsigned char bytesNr         = bytesNumber;
    
    writeCommand[0] = AD7190_COMM_WRITE |
                      AD7190_COMM_ADDR(registerAddress);
    while(bytesNr > 0)
    {
        writeCommand[bytesNr] = *dataPointer;
        dataPointer ++;
        bytesNr --;
    }
		ADI_PART_CS_LOW;
			
    SPI_Write(AD7190_SLAVE_ID * modifyCS, writeCommand, 4);

		ADI_PART_CS_HIGH;
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 * @param bytesNumber - Number of bytes that will be read.
 * @param modifyCS    - Allows Chip Select to be modified.
 *
 * @return buffer - Value of the register.
*******************************************************************************/
unsigned long AD7190_GetRegisterValue(unsigned char registerAddress, unsigned char bytesNumber,unsigned char modifyCS)
{
    unsigned char registerWord[5] = {0, 0, 0, 0, 0}; 
    unsigned long buffer          = 0x0;
    unsigned char i               = 0;
    
    registerWord[0] = AD7190_COMM_READ |
                      AD7190_COMM_ADDR(registerAddress);
		ADI_PART_CS_LOW;

    SPI_Read(AD7190_SLAVE_ID * modifyCS, registerWord, bytesNumber + 1);
		ADI_PART_CS_HIGH;
    
		switch(bytesNumber)
		{
			case 1:
				buffer = registerWord[1];
				break;
			case 2:
				buffer = (int)(registerWord[1])*0x100 + (int)registerWord[2];
				break;
			case 3:
				buffer = (int)(registerWord[1])*0x10000 + (int)registerWord[2]*0x100 +  (int)registerWord[3];
				break;
		}
    
    return buffer;
}

/***************************************************************************//**
 * @brief Checks if the AD7190 part is present.
 *
 * @return status - Indicates if the part is present or not.
*******************************************************************************/
    unsigned char regVal = 0;
unsigned char AD7190_Init(void)
{
    unsigned char status = 1;
    
    
    AD7190_Reset();
    /* Allow at least 500 us before accessing any of the on-chip registers. */
    HAL_Delay(1000);
    regVal = AD7190_GetRegisterValue(AD7190_REG_ID, 1, 1);
    if( (regVal & AD7190_ID_MASK) != ID_AD7190)
    {
        status = 0;
    }
    return status ;
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @return none.
*******************************************************************************/
void AD7190_Reset(void)
{
    unsigned char registerWord[10];
    
    registerWord[0] = 0x01;
    registerWord[1] = 0xFF;
    registerWord[2] = 0xFF;
    registerWord[3] = 0xFF;
    registerWord[4] = 0xFF;
    registerWord[5] = 0xFF;
    registerWord[6] = 0xFF;
    SPI_Write(AD7190_SLAVE_ID, registerWord, 7); 
	
}
/***************************************************************************//**
 * @brief Set device to idle or power-down.
 *
 * @param pwrMode - Selects idle mode or power-down mode.
 *                  Example: 0 - power-down
 *                           1 - idle
 *
 * @return none.
*******************************************************************************/
void AD7190_SetPower(unsigned char pwrMode)
{
     unsigned long oldPwrMode = 0x0;
     unsigned long newPwrMode = 0x0; 
 
     oldPwrMode = AD7190_GetRegisterValue(AD7190_REG_MODE, 3, 1);
     oldPwrMode &= ~(AD7190_MODE_SEL(0x7));
     newPwrMode = oldPwrMode | 
                  AD7190_MODE_SEL((pwrMode * (AD7190_MODE_IDLE)) |
                                  (!pwrMode * (AD7190_MODE_PWRDN)));
     AD7190_SetRegisterValue(AD7190_REG_MODE, newPwrMode, 3, 1);
}

/***************************************************************************//**
 * @brief Waits for RDY pin to go low.
 *
 * @return none.
*******************************************************************************/
void AD7190_WaitRdyGoLow(void)
{
    unsigned long timeOutCnt = 0xFFFFF;
}
void AD7190_SetBiPolar(char uniPolar)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   
     
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_UNIPOLAR);
	  if(uniPolar)
		{
			 newRegValue = oldRegValue | AD7190_CONF_UNIPOLAR;   
		}
		else
		{
			newRegValue = oldRegValue;
		}
		
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
}


/***************************************************************************//**
 * @brief Selects the channel to be enabled.
 *
 * @param channel - Selects a channel.
 *  
 * @return none.
*******************************************************************************/
void AD7190_ChannelSelect(unsigned short channel)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   
     
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));
    newRegValue = oldRegValue | AD7190_CONF_CHAN(1 << channel);   
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
}

/***************************************************************************//**
 * @brief Performs the given calibration to the specified channel.
 *
 * @param mode - Calibration type.
 * @param channel - Channel to be calibrated.
 *
 * @return none.
*******************************************************************************/
void AD7190_Calibrate(unsigned char mode, unsigned char channel)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;
    
    AD7190_ChannelSelect(channel);
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_MODE, 3, 1);
    oldRegValue &= ~AD7190_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7190_MODE_SEL(mode);
  HAL_GPIO_WritePin(SPI_CS_PIN_GPIO_Port , SPI_CS_PIN_Pin , GPIO_PIN_SET);  
	ADI_PART_CS_LOW; 
    AD7190_SetRegisterValue(AD7190_REG_MODE, newRegValue, 3, 0); // CS is not modified.
    AD7190_WaitRdyGoLow();
    ADI_PART_CS_HIGH;
}

/***************************************************************************//**
 * @brief Selects the polarity of the conversion and the ADC input range.
 *
 * @param polarity - Polarity select bit. 
                     Example: 0 - bipolar operation is selected.
                              1 - unipolar operation is selected.
* @param range - Gain select bits. These bits are written by the user to select 
                 the ADC input range.     
 *
 * @return none.
*******************************************************************************/
void AD7190_RangeSetup(unsigned char polarity, unsigned char range)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;
    
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF,3, 1);
    oldRegValue &= ~(AD7190_CONF_UNIPOLAR |
                     AD7190_CONF_GAIN(0x7));
    newRegValue = oldRegValue | 
                  (polarity * AD7190_CONF_UNIPOLAR) |
                  AD7190_CONF_GAIN(range); 
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
}

/***************************************************************************//**
 * @brief Returns the result of a single conversion.
 *
 * @return regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
unsigned long AD7190_SingleConversion(void)
{
    unsigned long command = 0x0;
    unsigned long regData = 0x0;
 
    command = AD7190_MODE_SEL(AD7190_MODE_SINGLE) |  AD7190_MODE_RATE(0x060) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) ;   
	
    ADI_PART_CS_LOW;
    AD7190_SetRegisterValue(AD7190_REG_MODE, command, 3, 0); // CS is not modified.
    AD7190_WaitRdyGoLow();
    regData = AD7190_GetRegisterValue(AD7190_REG_DATA, 3, 0);
    ADI_PART_CS_HIGH;
    
    return regData;
}

/***************************************************************************//**
 * @brief Returns the average of several conversion results.
 *
 * @return samplesAverage - The average of the conversion results.
*******************************************************************************/
unsigned long AD7190_ContinuousReadAvg(unsigned char sampleNumber)
{
    unsigned long samplesAverage = 0x0;
    unsigned char count = 0x0;
    unsigned long command = 0x0;
    
    command = AD7190_MODE_SEL(AD7190_MODE_CONT)|AD7190_MODE_CLKSRC(AD7190_CLK_INT)|AD7190_MODE_RATE(0x060);
    ADI_PART_CS_LOW;
    AD7190_SetRegisterValue(AD7190_REG_MODE, command, 3, 0); // CS is not modified.
    for(count = 0;count < sampleNumber;count ++)
    {
        AD7190_WaitRdyGoLow();
        samplesAverage += AD7190_GetRegisterValue(AD7190_REG_DATA, 3, 0); // CS is not modified.
    }
    ADI_PART_CS_HIGH;
    samplesAverage = samplesAverage / sampleNumber;
    
    return samplesAverage ;
}

/***************************************************************************//**
 * @brief Read data from temperature sensor and converts it to Celsius degrees.
 *
 * @return temperature - Celsius degrees.
*******************************************************************************/
unsigned long AD7190_TemperatureRead(void)
{
    unsigned char temperature = 0x0;
    unsigned long dataReg = 0x0;
   
    AD7190_RangeSetup(0, AD7190_CONF_GAIN_1);
    AD7190_ChannelSelect(AD7190_CH_TEMP_SENSOR);
    dataReg = AD7190_SingleConversion();
    dataReg -= 0x800000;
    dataReg /= 2815;   // Kelvin Temperature
    dataReg -= 273;    //Celsius Temperature
    temperature = (unsigned long) dataReg;
    
    return temperature;
}




