/**************************************************************************/
/*! 
    @file     tsl2561.c
    @author   K. Townsend (microBuilder.eu)
    modified for STM32 by JC Toussaint Phelma
	
    @brief    Drivers for the TAOS TSL2561 I2C digital luminosity sensor

    @section DESCRIPTION

    The TSL2561 is a 16-bit digital luminosity sensor the approximates
    the human eye's response to light.  It contains one broadband
    photodiode that measures visible plus infrared light (channel 0)
    and one infrared photodiode (channel 1).

    @section EXAMPLE

    @code
    #include "tsl2561.h"
    ...
    uint16_t broadband, ir;
    uint32_t lux;

    // Initialise luminosity sensor
    TSL2561_Init(&hi2c1);

    // Optional ... default setting is 400ms with no gain
    // Set timing to 101ms with no gain
    TSL2561_SetTiming(&hi2c1, TSL2561_INTEGRATIONTIME_101MS, TSL2561_GAIN_0X);

    // Check luminosity level and calculate lux
    TSL2561_GetLuminosity(&hi2c1, &broadband, &ir);
    lux = TSL2561_CalculateLux(broadband, ir);
    printf("Broadband: %u, IR: %u, Lux: %d %s", broadband, ir, lux, CFG_PRINTF_NEWLINE);

    @endcode

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2010, microBuilder SARL
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include "tsl2561.h"

static uint8_t _tsl2561_Initialised = false;
static tsl2561_IntegrationTime_t _tsl2561_IntegrationTime = TSL2561_INTEGRATIONTIME_402MS;
static tsl2561_Gain_t _tsl2561_Gain = TSL2561_GAIN_0X;

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	char msg[256];
	sprintf(msg,"%s,%d", file,line);
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}


/*!
 * @brief sends a command to the TSL2561 sensor
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  cmd unsigned integer to be sent
 * @retval None
 */
void TSL2561_WriteCmd (I2C_HandleTypeDef *hi2c, uint8_t cmd)
{
	while(HAL_I2C_Master_Transmit (hi2c, TSL2561_ADDRESS, &cmd, 1, 1000)!= HAL_OK)
	  {
	    /* Error_Handler() function is called when Timout error occurs */
		_Error_Handler(__FILE__, __LINE__);
	  }
}


/*!
 * @brief Writes a value into an 8-bit register of the TSL2561 sensor
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  reg Internal memory address
 * @param  value unsigned integer to be written
 * @retval None
 */
void TSL2561_Write8 (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
	if (HAL_I2C_Mem_Write(hi2c, TSL2561_ADDRESS, (uint16_t) reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);
}

/*!
 * @brief reads a 16 bit value from an 8-bit register of the TSL2561 sensor
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  reg Internal memory address
 * @param  value unsigned integer to be written
 * @retval None
 */
void TSL2561_Read16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *value)
{
	uint8_t buffer[2] = {0};

	/* Send register address and Receive 2 bytes */
	if (HAL_I2C_Mem_Read(hi2c, TSL2561_ADDRESS, (uint16_t) reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *) buffer, 2, 1000) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

 // Shift values to create properly formed integer (low byte first)
		*value = (buffer[1] << 8) + buffer[0];
}


/*!
 * @brief Enables the device
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */

void TSL2561_Enable(I2C_HandleTypeDef *hi2c)
{
  if (!_tsl2561_Initialised) TSL2561_Init(hi2c);

  // Enable the device by setting the control bit to 0x03
  TSL2561_Write8(hi2c, TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
}

/*!
 * @brief  Disables the device (putting it in lower power sleep mode)
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */

void TSL2561_Disable(I2C_HandleTypeDef *hi2c)
{
  if (!_tsl2561_Initialised) TSL2561_Init(hi2c);

  // Turn the device off to save power
  TSL2561_Write8(hi2c, TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
}

/*!
 * @brief  Initialises the I2C block
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */

void TSL2561_Init(I2C_HandleTypeDef *hi2c)
{
/*
  // Initialise I2C
  if (i2cInit(I2CMASTER) == false)
  {
    return TSL2561_ERROR_I2CINIT;    // Fatal error 
  }
*/
  
  _tsl2561_Initialised = true;

  // Set default integration time and gain
  TSL2561_SetTiming(hi2c, _tsl2561_IntegrationTime, _tsl2561_Gain);

  // Note: by default, the device is in power down mode on bootup
}

/*!
 * @brief  Sets the integration time and gain (controls sensitivity)
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param integration time
 * @param gain
 * @retval None
 */

void TSL2561_SetTiming(I2C_HandleTypeDef *hi2c, tsl2561_IntegrationTime_t integration, tsl2561_Gain_t gain)
{
  if (!_tsl2561_Initialised) TSL2561_Init(hi2c);

  // Enable the device by setting the control bit to 0x03
  TSL2561_Enable(hi2c);
  
  // Turn the device off to save power
  TSL2561_Write8(hi2c, TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, integration | gain);  
 
  // Update value placeholders
  _tsl2561_IntegrationTime = integration;
  _tsl2561_Gain = gain;

  // Turn the device off to save power
  TSL2561_Disable(hi2c);
}

/*!
 * @brief  Reads the luminosity on both channels from the TSL2561
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param uint16_t pointer to visible and infrared contribution
 * @param uint16_t pointer to infrared one only
 * @retval None
 */

void TSL2561_GetLuminosity (I2C_HandleTypeDef *hi2c, uint16_t *broadband, uint16_t *ir)
{
  if (!_tsl2561_Initialised) TSL2561_Init(hi2c);

  // Enable the device by setting the control bit to 0x03
  TSL2561_Enable(hi2c);
 
  // Wait x ms for ADC to complete
  switch (_tsl2561_IntegrationTime)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      HAL_Delay(14);
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      HAL_Delay(102);
      break;
    default:
      HAL_Delay(400);
      break;
  }

  // Reads two byte value from channel 0 (visible + infrared)
  TSL2561_Read16(hi2c, TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW, broadband);

  // Reads two byte value from channel 1 (infrared)
  TSL2561_Read16(hi2c, TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW, ir);

  // Turn the device off to save power
  TSL2561_Disable(hi2c);
}

/*!
 * @brief  Calculates LUX from the supplied ch0 (broadband) and ch1 (IR) readings
 * @param uint16_t channel0
 * @param uint16_t channel1
 * @retval uint32_t LUX value
 */

uint32_t TSL2561_CalculateLux(uint16_t ch0, uint16_t ch1)
{
  unsigned long chScale;
  unsigned long channel1;
  unsigned long channel0;

  switch (_tsl2561_IntegrationTime)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      chScale = TSL2561_LUX_CHSCALE_TINT0;
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      chScale = TSL2561_LUX_CHSCALE_TINT1;
      break;
    default: // No scaling ... integration time = 402ms
      chScale = (1 << TSL2561_LUX_CHSCALE);
      break;
  }

  // Scale for gain (1x or 16x)
  if (!_tsl2561_Gain) chScale = chScale << 4;

  // scale the channel values
  channel0 = (ch0 * chScale) >> TSL2561_LUX_CHSCALE;
  channel1 = (ch1 * chScale) >> TSL2561_LUX_CHSCALE;

  // find the ratio of the channel values (Channel1/Channel0)
  unsigned long ratio1 = 0;
  if (channel0 != 0) ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;

  // round the ratio value
  unsigned long ratio = (ratio1 + 1) >> 1;

  unsigned int b, m;

#ifdef TSL2561_PACKAGE_CS
  if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1C))
    {b=TSL2561_LUX_B1C; m=TSL2561_LUX_M1C;}
  else if (ratio <= TSL2561_LUX_K2C)
    {b=TSL2561_LUX_B2C; m=TSL2561_LUX_M2C;}
  else if (ratio <= TSL2561_LUX_K3C)
    {b=TSL2561_LUX_B3C; m=TSL2561_LUX_M3C;}
  else if (ratio <= TSL2561_LUX_K4C)
    {b=TSL2561_LUX_B4C; m=TSL2561_LUX_M4C;}
  else if (ratio <= TSL2561_LUX_K5C)
    {b=TSL2561_LUX_B5C; m=TSL2561_LUX_M5C;}
  else if (ratio <= TSL2561_LUX_K6C)
    {b=TSL2561_LUX_B6C; m=TSL2561_LUX_M6C;}
  else if (ratio <= TSL2561_LUX_K7C)
    {b=TSL2561_LUX_B7C; m=TSL2561_LUX_M7C;}
  else if (ratio > TSL2561_LUX_K8C)
    {b=TSL2561_LUX_B8C; m=TSL2561_LUX_M8C;}
#else
  if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T))
    {b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
  else if (ratio <= TSL2561_LUX_K2T)
    {b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
  else if (ratio <= TSL2561_LUX_K3T)
    {b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
  else if (ratio <= TSL2561_LUX_K4T)
    {b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
  else if (ratio <= TSL2561_LUX_K5T)
    {b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
  else if (ratio <= TSL2561_LUX_K6T)
    {b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
  else if (ratio <= TSL2561_LUX_K7T)
    {b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
  else if (ratio > TSL2561_LUX_K8T)
    {b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}
#endif

  unsigned long temp;
  temp = ((channel0 * b) - (channel1 * m));

  // do not allow negative lux value
  if (temp < 0) temp = 0;

  // round lsb (2^(LUX_SCALE-1))
  temp += (1 << (TSL2561_LUX_LUXSCALE-1));

  // strip off fractional portion
  uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;

  // Signal I2C had no errors
  return lux;
}
