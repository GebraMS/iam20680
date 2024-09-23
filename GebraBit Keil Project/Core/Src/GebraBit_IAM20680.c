/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Mehrdad Zeinali
 * ________________________________________________________________________________________________________
 */
 
#include	"GebraBit_IAM20680.h"

extern SPI_HandleTypeDef hspi1;
	
/*=========================================================================================================================================
 * @brief     Read data from spacial register.
 * @param     regAddr Register Address of IAM20680
 * @param     data    Pointer to Variable that register value is saved .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t	GB_IAM20680_Read_Reg_Data ( uint8_t regAddr, uint8_t* data)
{	
	uint8_t txBuf[2] = {regAddr|0x80 , 0x00}; //Read operation: set the 8th-bit to 1.
	uint8_t rxBuf[2];
	HAL_StatusTypeDef stat = HAL_ERROR ;
	//GB_IAM20680_Bank_Selection(regBank);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	stat = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	if (stat == HAL_OK)
	{
		*data = rxBuf[1];
	}
	return stat;
}
/*========================================================================================================================================= 
 * @brief     Read data from spacial bits of a register.
 * @param     regAddr     Register Address of IAM20680 .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to read(1 to 8) 
 * @param     data        Pointer to Variable that register Bits value is saved .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_IAM20680_Read_Reg_Bits (uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t* data)
{
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;
	
	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}

	if (GB_IAM20680_Read_Reg_Data( regAddr, &tempData) == HAL_OK)
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1); //formula for making a broom of 1&0 for gathering desired bits
		tempData &= mask; // zero all non-important bits in data
		tempData >>= (start_bit - len + 1); //shift data to zero position
		*data = tempData;
		status = HAL_OK;
	}
	else
	{
		status = HAL_ERROR;
		*data = 0;
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Read multiple data from first spacial register address.
 * @param     regAddr First Register Address of IAM20680 that reading multiple data start from this address
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     byteQuantity Quantity of data that we want to read .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_IAM20680_Burst_Read(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)
{
	uint8_t *pTxBuf;
	uint8_t *pRxBuf;
//	uint8_t pTxBuf[513];
//	uint8_t pRxBuf[513];
	uint8_t status = HAL_ERROR;
	pTxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1)); // reason of "+1" is for register address that comes in first byte
	pRxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1));
	memset(pTxBuf, 0, (byteQuantity + 1)*sizeof(uint8_t));

	pTxBuf[0] = regAddr | 0x80; //Read operation: set the 8th-bit to 1.

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, pTxBuf, pRxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	if (status == HAL_OK)
	{
		memcpy(data, &pRxBuf[1], byteQuantity*sizeof(uint8_t)); //here we dont have "+1" beacause we don't need first byte that was register data , we just need DATA itself
	}
	free(pTxBuf);
	free(pRxBuf);
	return status;
}
/*=========================================================================================================================================
 * @brief     Write data to spacial register.
 * @param     regAddr Register Address of IAM20680
 * @param     data    Value that will be writen to register .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_IAM20680_Write_Reg_Data(uint8_t regAddr, uint8_t data)
{
	uint8_t txBuf[2] = {regAddr|0x00 , data}; //Write operation: set the 8th-bit to 0
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	return status;	
}

/*=========================================================================================================================================
 * @brief     Write data to spacial bits of a register.
 * @param     regAddr     Register Address of IAM20680 .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to write(1 to 8) 
 * @param     data        Value that will be writen to register bits .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_IAM20680_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data)
{
	uint8_t txBuf[2];
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;
	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}
	if (GB_IAM20680_Read_Reg_Data( regAddr,  &tempData) == HAL_OK)	
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		data <<= (start_bit - len + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		tempData &= ~(mask); // zero all important bits in existing byte
		tempData |= data; // combine data with existing byte

		txBuf[0] = regAddr;
		txBuf[1] = tempData;
	
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
		while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Write value to Multiple register address.
 * @param     regAddr First Register Address of IAM20680 that writing multiple data start from this address
 * @param     data    Pointer to Variable that multiple data are writen from .
 * @param     byteQuantity Quantity of data that we want to write .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_IAM20680_Burst_Write		( uint8_t regAddr, uint8_t *data, 	uint16_t byteQuantity)
{
	uint8_t txBuf[byteQuantity + 1]; // +1 is for register address that is 1 byte
	uint8_t rxBuf[byteQuantity + 1];
	uint8_t status = HAL_ERROR;
	txBuf[0] = regAddr | 0x00; //Write operation: set the 8th-bit to 0.
	memcpy(txBuf+1, data, byteQuantity); // +1 is for set the address of data from [1]th position of array

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	return status;
}

/*=========================================================================================================================================
 * @brief     Reset IAM20680
 * @param     IAM20680   IAM20680 Struct RESET  variable
 * @param     IAM20680   IAM20680 Struct
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IAM20680_Soft_Reset ( GebraBit_IAM20680 * IAM20680 )
{
	GB_IAM20680_Write_Reg_Bits(IAM20680_USER_CTRL, START_MSB_BIT_AT_0,BIT_LENGTH_1 , 1);
	do 
	 {
		GB_IAM20680_Write_Reg_Bits(IAM20680_PWR_MGMT_1, START_MSB_BIT_AT_7,BIT_LENGTH_1 , 1);
		HAL_Delay(100);
		GB_IAM20680_Read_Reg_Bits (IAM20680_PWR_MGMT_1, START_MSB_BIT_AT_7,BIT_LENGTH_1, &IAM20680->RESET); 
		if ( IAM20680->RESET == DONE )
			break;
	 }while(1);
}
/*=========================================================================================================================================
 * @brief     Get Who am I Register Value From Sensor
 * @param     IAM20680     IAM20680 Struct WHO_AM_I variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void	GB_IAM20680_Who_am_I(GebraBit_IAM20680 * IAM20680)
{
	GB_IAM20680_Read_Reg_Data( IAM20680_WHO_AM_I,&IAM20680->WHO_AM_I);
}	

/*=========================================================================================================================================
 * @brief     Select SPI 4 Wire as interface
 * @param     IAM20680   IAM20680 Struct INTERFACE  variable
 * @param     spisel Determines SPI 4 Wire as interface or not(IS_SPI or NOT_SPI ) 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IAM20680_Select_SPI4_Interface(GebraBit_IAM20680 * IAM20680 , IAM20680_Interface spisel)
{
 GB_IAM20680_Write_Reg_Bits( IAM20680_USER_CTRL, START_MSB_BIT_AT_4, BIT_LENGTH_1 , spisel);
 IAM20680->INTERFACE = spisel ; 
}
/*=========================================================================================================================================
 * @brief     SET IAM20680 Sleep or Awake
 * @param     IAM20680   IAM20680 Struct IS_IAM20680_Sleep  variable
 * @param     working   Determines IAM20680_AWAKE or IAM20680_SLEEP
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Sleep_Awake (GebraBit_IAM20680 * IAM20680, IAM20680_Sleep  working  ) 
{
  GB_IAM20680_Write_Reg_Bits (IAM20680_PWR_MGMT_1 , START_MSB_BIT_AT_6, BIT_LENGTH_1 , working);
  IAM20680->IS_IAM20680_SLEEP = working ;
}

/*=========================================================================================================================================
 * @brief     Set IAM20680 Accelerometer Power Mode
 * @param     IAM20680   IAM20680 Struct ACCEL_POWER_MODE  variable
 * @param     pmode        Determines IAM20680 Accelerometer Power Mode in IAM20680_LOW_NOISE or IAM20680_LOW_POWER
 * @return    Nothing
 ========================================================================================================================================*/ 
 void GB_IAM20680_ACCEL_Power_Mode(GebraBit_IAM20680* IAM20680 ,IAM20680_Power_Mode pmode)
{
	GB_IAM20680_Write_Reg_Bits (IAM20680_PWR_MGMT_1 , START_MSB_BIT_AT_5, BIT_LENGTH_1 , pmode);
	IAM20680->ACCEL_POWER_MODE = pmode ;
}
/*=========================================================================================================================================
 * @brief     Set IAM20680 Gyroscope Power Mode
 * @param     IAM20680   IAM20680 Struct GYRO_POWER_MODE  variable
 * @param     pmode        Determines IAM20680 Gyroscope Power Mode in IAM20680_LOW_NOISE or IAM20680_LOW_POWER
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_GYRO_Power_Mode(GebraBit_IAM20680* IAM20680 ,IAM20680_Power_Mode pmode)
{
	GB_IAM20680_Write_Reg_Bits (IAM20680_LP_MODE_CFG , START_MSB_BIT_AT_7, BIT_LENGTH_1 , pmode);
	IAM20680->GYRO_POWER_MODE = pmode ;
}
/*=========================================================================================================================================
 * @brief     Set IAM20680 Clock Source
 * @param     IAM20680   IAM20680 Struct CLOCK_SOURCE  variable
 * @param     clk    Determines between INTERNAL_20MHZ_OSCILLATOR , AUTO_SELECT and CLOCK_STOP
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Set_Clock_Source(GebraBit_IAM20680 * IAM20680 , IAM20680_CLK clk)
{ 
 GB_IAM20680_Write_Reg_Bits( IAM20680_PWR_MGMT_1, START_MSB_BIT_AT_2, BIT_LENGTH_3 , clk);
 IAM20680->CLOCK_SOURCE = clk ;
} 
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Temperature Sensor
 * @param     IAM20680   IAM20680 Struct TEMPERATURE  variable
 * @param     temp     Determines DISABLE or ENABLE Temperature Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Temperature(GebraBit_IAM20680* IAM20680 ,IAM20680_Ability temp)
{
	GB_IAM20680_Write_Reg_Bits (IAM20680_PWR_MGMT_1 , START_MSB_BIT_AT_3, BIT_LENGTH_1 , !temp);
  IAM20680->TEMPERATURE = temp ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Accelerometer Sensor
 * @param     IAM20680   IAM20680 Struct ACCEL  variable
 * @param     accel     Determines SENSOR_DISABLE or SENSOR_ENABLE Accelerometer Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_IAM20680_Accelerometer(GebraBit_IAM20680 * IAM20680 , IAM20680_Sensor accel)
{
	GB_IAM20680_Write_Reg_Bits (IAM20680_PWR_MGMT_2 , START_MSB_BIT_AT_5, BIT_LENGTH_3 , accel);
  IAM20680->ACCEL = accel ; 
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Gyroscope Sensor
 * @param     IAM20680   IAM20680 Struct GYRO  variable
 * @param     gyro     Determines SENSOR_DISABLE or SENSOR_ENABLE Gyroscope Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Gyroscope(GebraBit_IAM20680 * IAM20680 , IAM20680_Sensor gyro)
{
	GB_IAM20680_Write_Reg_Bits (IAM20680_PWR_MGMT_2 , START_MSB_BIT_AT_2, BIT_LENGTH_3 , gyro);
  IAM20680->GYRO = gyro ; 
}
/*=========================================================================================================================================
 * @brief     Configure hardware interrupt pin (INT) 
 * @param     IAM20680  IAM20680 struct INT_PIN_LEVEL , INT_PIN_TYPE and INT_PIN_LATCH  variables
 * @param     level   ACTIVE_HIGH or  ACTIVE_LOW 
 * @param     type    PUSH_PULL   or  OPEN_DRAIN
 * @param     latch   _50_US      or  HELD_STATUS_CLEAR
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IAM20680_Set_INT_Pin(GebraBit_IAM20680 * IAM20680 , IAM20680_INT_Level level ,IAM20680_INT_Type type , IAM20680_Latch_Type latch )
{
  GB_IAM20680_Write_Reg_Bits( IAM20680_INT_PIN_CFG, START_MSB_BIT_AT_7, BIT_LENGTH_1 , level);
	GB_IAM20680_Write_Reg_Bits( IAM20680_INT_PIN_CFG, START_MSB_BIT_AT_6, BIT_LENGTH_1 , type);
	GB_IAM20680_Write_Reg_Bits( IAM20680_INT_PIN_CFG, START_MSB_BIT_AT_5, BIT_LENGTH_1 , latch);
	IAM20680->INT_PIN_LEVEL = level ; 
	IAM20680->INT_PIN_TYPE  = type  ;
	IAM20680->INT_PIN_LATCH = latch ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE FIFO Overflow Interrupt
 * @param     IAM20680   IAM20680 Struct FIFO_OVERFLOW_INT  variable
 * @param     data_ovf_int    Determines  FIFO Overflow Interrupt Disable or Enable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_FIFO_Overflow_Interrupt(GebraBit_IAM20680 * IAM20680 , IAM20680_Ability data_ovf_int)
{
	GB_IAM20680_Write_Reg_Bits(IAM20680_INT_ENABLE, START_MSB_BIT_AT_4, BIT_LENGTH_1 , data_ovf_int);
	IAM20680->FIFO_OVERFLOW_INT = data_ovf_int  ; 
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Data Ready Interrupt
 * @param     IAM20680   IAM20680 Struct DATA_READY_INT  variable
 * @param     data_ready_int    Determines Data Ready Interrupt Disable or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Data_Ready_Interrupt(GebraBit_IAM20680 * IAM20680 , IAM20680_Ability data_ready_int)
{
	GB_IAM20680_Write_Reg_Bits(IAM20680_INT_ENABLE, START_MSB_BIT_AT_0, BIT_LENGTH_1 , data_ready_int);
	IAM20680->DATA_READY_INT = data_ready_int  ; 
}
/*=========================================================================================================================================
 * @brief     Check if FIFO Overflow
 * @param     IAM20680   Store FIFO OVERFLOW status onIAM20680 Struct FIFO_OVERFLOW  variable
 * @return    FIFO_IS_NOT_OVERFLOW or FIFO_IS_OVERFLOW
 ========================================================================================================================================*/ 
IAM20680_FIFO_Overflow GB_IAM20680_Check_FIFO_Overflow(GebraBit_IAM20680 * IAM20680)
{
  GB_IAM20680_Read_Reg_Bits(IAM20680_INT_STATUS, START_MSB_BIT_AT_4, BIT_LENGTH_1 , &IAM20680->FIFO_OVERFLOW);
	return IAM20680->FIFO_OVERFLOW;
}
/*=========================================================================================================================================
 * @brief     Check if Data is ready
 * @param     IAM20680    Store data ready status on IAM20680 Struct DATA_STATUS variable
 * @return    IS_Ready or IS_NOT_Ready
 ========================================================================================================================================*/ 
IAM20680_Preparation GB_IAM20680_Check_Data_Preparation(GebraBit_IAM20680 * IAM20680)
{
  GB_IAM20680_Read_Reg_Bits(IAM20680_INT_STATUS, START_MSB_BIT_AT_0, BIT_LENGTH_1 , &IAM20680->DATA_STATUS); 
	return IAM20680->DATA_STATUS;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Access Serial Interface To FIFO
 * @param     IAM20680  IAM20680 struct INTERFACE_ACCESS_FIFO  variable
 * @param     interface_access_fifo    DISABLE or ENABLE
 * @return    Nothing
 ========================================================================================================================================*/ 
/*
M403Z 
*/
void GB_IAM20680_Access_Serial_Interface_To_FIFO(GebraBit_IAM20680 * IAM20680 , IAM20680_Ability interface_access_fifo) 
{ 
	GB_IAM20680_Write_Reg_Bits (IAM20680_USER_CTRL , START_MSB_BIT_AT_6, BIT_LENGTH_1,  interface_access_fifo);
  IAM20680->INTERFACE_ACCESS_FIFO = interface_access_fifo ;  
}

/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE accelerometer to be written on FIFO
 * @param     IAM20680  IAM20680 struct ACCEL_TO_FIFO  variable  
 * @param     accel_fifo Determines accelerometer write on fifo
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Write_ACCEL_FIFO(GebraBit_IAM20680 * IAM20680 , IAM20680_Ability accel_fifo )
{
   GB_IAM20680_Write_Reg_Bits (IAM20680_FIFO_EN, START_MSB_BIT_AT_3, BIT_LENGTH_1,accel_fifo); 
	 IAM20680->ACCEL_TO_FIFO = accel_fifo ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Gyroscope to be written on FIFO
 * @param     IAM20680  IAM20680 struct GYRO_TO_FIFO  variable  
 * @param     gyro_fifo  Determines Gyroscope write on fifo
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Write_GYRO_FIFO(GebraBit_IAM20680 * IAM20680 , IAM20680_Ability gyro_fifo )
{
   GB_IAM20680_Write_Reg_Bits (IAM20680_FIFO_EN, START_MSB_BIT_AT_6, BIT_LENGTH_3,(uint8_t)(gyro_fifo*7)); ///*7 to make b111 to enable xyz
	 IAM20680->GYRO_TO_FIFO = gyro_fifo ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Temperature to be written on FIFO
 * @param     IAM20680  IAM20680 struct TEMP_TO_FIFO  variable 
 * @param     temp_fifo  Determines Temperature write on fifo
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_IAM20680_Write_TEMP_FIFO(GebraBit_IAM20680 * IAM20680 , IAM20680_Ability temp_fifo )
{
   GB_IAM20680_Write_Reg_Bits (IAM20680_FIFO_EN, START_MSB_BIT_AT_7, BIT_LENGTH_1,temp_fifo); 
	 IAM20680->TEMP_TO_FIFO = temp_fifo ;
}
/*=========================================================================================================================================
 * @brief     Set FIFO MODE
 * @param     IAM20680  IAM20680 struct FIFO_MODE  variable 
 * @param     fifo_mode     Determines FIFO MODE BYPASS ,  STREAM_TO_FIFO , STOP_ON_FULL_FIFO_SNAPSHOT
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_FIFO_Mode(GebraBit_IAM20680 * IAM20680 , IAM20680_FIFO_Mode fifo_mode )
{
  GB_IAM20680_Write_Reg_Bits (IAM20680_CONFIG,START_MSB_BIT_AT_6, BIT_LENGTH_1, fifo_mode); 
  IAM20680->FIFO_MODE = fifo_mode;
}
/*=========================================================================================================================================
 * @brief     Set FIFO reset.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_FIFO_Reset(void ) 
{
  GB_IAM20680_Write_Reg_Bits (IAM20680_USER_CTRL, START_MSB_BIT_AT_2, BIT_LENGTH_1, 1); 
}
/*=========================================================================================================================================
 * @brief     Get FIFO Count  
 * @param     IAM20680   IAM20680 struct  FIFO_COUNT variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_GET_FIFO_Count (GebraBit_IAM20680 * IAM20680 ) 
{
	uint8_t count_h , count_l;
  GB_IAM20680_Read_Reg_Data( IAM20680_FIFO_COUNTH, &count_h); 
	GB_IAM20680_Read_Reg_Data( IAM20680_FIFO_COUNTL, &count_l );
	IAM20680->FIFO_COUNT = (uint16_t)((count_h << 8) | count_l);////13_Bit
}
/*=========================================================================================================================================
 * @brief     Read Data Directly from FIFO
 * @param     IAM20680  IAM20680 struct FIFO_DATA variable
 * @param     qty    Determine hoe many Data Byte to read
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Read_FIFO(GebraBit_IAM20680 * IAM20680 , uint16_t qty)  
{
  GB_IAM20680_Burst_Read( IAM20680_FIFO_R_W,IAM20680->FIFO_DATA, qty);
}
/*=========================================================================================================================================
 * @brief     Set Gyroscope Full Scale Range and select Gyroscope SCALE FACTOR
 * @param     IAM20680   IAM20680 Struct GYRO_FULL_SCALE and GYRO_SCALE_FACTOR variable
 * @param     fs         Determines Full Scale Range among FS_250_DPS , FS_500_DPS , FS_1000_DPS , FS_2000_DPS
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IAM20680_GYRO_Full_Scale ( GebraBit_IAM20680 * IAM20680 , IAM20680_Gyro_Fs_Sel fs ) 
{
  GB_IAM20680_Write_Reg_Bits (IAM20680_GYRO_CONFIG , START_MSB_BIT_AT_4, BIT_LENGTH_2, fs);
	IAM20680->GYRO_FULL_SCALE = fs ; 
	switch(fs)
	 {
	  case FS_250_DPS:
		IAM20680->GYRO_SCALE_FACTOR = SCALE_FACTOR_131_LSB_DPS ;
		IAM20680->PRECISE_GYRO_SF   =  131 ;
    break;
		case FS_500_DPS:
		IAM20680->GYRO_SCALE_FACTOR = SCALE_FACTOR_65p5_LSB_DPS ;
		IAM20680->PRECISE_GYRO_SF   =  65.5 ;
    break;	
		case FS_1000_DPS:
		IAM20680->GYRO_SCALE_FACTOR = SCALE_FACTOR_32p8_LSB_DPS ;
		IAM20680->PRECISE_GYRO_SF   =  32.8 ;
    break;	
		case FS_2000_DPS:
		IAM20680->GYRO_SCALE_FACTOR = SCALE_FACTOR_16p4_LSB_DPS ;
		IAM20680->PRECISE_GYRO_SF   =  16.4 ;
    break;			
		default:
		IAM20680->GYRO_SCALE_FACTOR = SCALE_FACTOR_131_LSB_DPS ;
    IAM20680->PRECISE_GYRO_SF   =  131 ;		
	 }
}
/*=========================================================================================================================================
 * @brief     Enable Or Bypass GYRO and Temperature Low Pass Filter
 * @param     IAM20680     IAM20680 Struct GYRO_FCHOICEB variable
 * @param     bypass       Determines BYPASS_DLPF_FCHOICEB or ENABLE_DLPF_FCHOICEB
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_IAM20680_GYRO_TEMP_Low_Pass_Filter  (GebraBit_IAM20680 * IAM20680 ,  IAM20680_FCHOICEB bypass )
{
	if ( bypass == BYPASS_DLPF_FCHOICEB  )
		IAM20680->GYRO_SAMPLE_RATE = _32_KHz ;
	GB_IAM20680_Write_Reg_Bits(IAM20680_GYRO_CONFIG, START_MSB_BIT_AT_1, BIT_LENGTH_2, (uint8_t) (bypass*3));//2 Bits : 11
	IAM20680->GYRO_FCHOICEB =bypass ;   
}
/*=========================================================================================================================================
 * @brief     Set GYRO and Temperature Low Pass Filter value
 * @param     IAM20680     IAM20680 Struct GYRO_TEMP_DLPF variable
 * @param     dlpf         Low Pass Filter value 
 * @return    Nothing
 ========================================================================================================================================*/ 		
void GB_IAM20680_GYRO_TEMP_Low_Pass_Filter_Value  (GebraBit_IAM20680 * IAM20680 , IAM20680_GYRO_TEMP_DLPF dlpf )
{
	  GB_IAM20680_Write_Reg_Bits(IAM20680_CONFIG , START_MSB_BIT_AT_2, BIT_LENGTH_3,  dlpf);
	  IAM20680->GYRO_TEMP_DLPF =  dlpf ;
}
/*=========================================================================================================================================
 * @brief     Set  GYRO Averaging Filter
 * @param     IAM20680  IAM20680 Struct GYRO_AVERAGING_FILTER variable
 * @param     avg       Averaging value
 * @return    Nothing 
 ========================================================================================================================================*/ 		
void GB_IAM20680_GYRO_LP_Averaging_Filter  (GebraBit_IAM20680 * IAM20680 , IAM20680_GYRO_Averaging_Filter avg )
{
	  GB_IAM20680_GYRO_TEMP_Low_Pass_Filter(IAM20680,ENABLE_DLPF_FCHOICEB);
	  GB_IAM20680_Write_Reg_Bits(IAM20680_LP_MODE_CFG , START_MSB_BIT_AT_6, BIT_LENGTH_3,  avg);
	  IAM20680->GYRO_AVERAGING_FILTER =  avg ;
}
/*=========================================================================================================================================
 * @brief     Set Accelerometer Full Scale Range and select sensor SCALE FACTOR
 * @param     IAM20680   IAM20680 struct ACCEL_FULL_SCALE and ACCEL_SCALE_FACTOR variable
 * @param     fs         Determines Full Scale Range among 2g , 4g , 8g , 16g , 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IAM20680_ACCEL_Full_Scale ( GebraBit_IAM20680 * IAM20680 , IAM20680_Accel_Fs_Sel fs ) 
{
  GB_IAM20680_Write_Reg_Bits( IAM20680_ACCEL_CONFIG, START_MSB_BIT_AT_4, BIT_LENGTH_2 , fs);
	IAM20680->ACCEL_FULL_SCALE =  fs ;
	switch(fs)
	 {
	  case FULL_SCALE_2g:
		IAM20680->ACCEL_SCALE_FACTOR = SCALE_FACTOR_16384_LSB_g ;
    break;
		case FULL_SCALE_4g:
		IAM20680->ACCEL_SCALE_FACTOR = SCALE_FACTOR_8192_LSB_g ;
    break;	
		case FULL_SCALE_8g:
		IAM20680->ACCEL_SCALE_FACTOR = SCALE_FACTOR_4096_LSB_g ;
    break;	
		case FULL_SCALE_16g: 
		IAM20680->ACCEL_SCALE_FACTOR = SCALE_FACTOR_2048_LSB_g ;
    break;			
		default:
		IAM20680->ACCEL_SCALE_FACTOR = SCALE_FACTOR_16384_LSB_g ;		
	 }
}

/*=========================================================================================================================================
 * @brief     Enable Or Bypass Accelerometer Low Pass Filter
 * @param     IAM20680     IAM20680 Struct ACCEL_FCHOICEB variable
 * @param     bypass       Determines ENABLE_DLPF_FCHOICEB or BYPASS_DLPF_FCHOICEB
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_IAM20680_ACCEL_Low_Pass_Filter  (GebraBit_IAM20680 * IAM20680 ,  IAM20680_FCHOICEB bypass )
{
	if ( bypass == BYPASS_DLPF_FCHOICEB  )
		IAM20680->ACCEL_SAMPLE_RATE = _4_KHz ;
	GB_IAM20680_Write_Reg_Bits(IAM20680_ACCEL_CONFIG2 , START_MSB_BIT_AT_3, BIT_LENGTH_1,  bypass);
	IAM20680->ACCEL_FCHOICEB =bypass ;   
}
/*=========================================================================================================================================
 * @brief     Set Accelerometer Low Pass Filter value
 * @param     IAM20680     IAM20680 Struct ACCEL_DLPF variable
 * @param     dlpf     Low Pass Filter value 
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_IAM20680_ACCEL_Low_Pass_Filter_Value  (GebraBit_IAM20680 * IAM20680 , IAM20680_ACCEL_DLPF dlpf )
{
	  GB_IAM20680_Write_Reg_Bits(IAM20680_ACCEL_CONFIG2, START_MSB_BIT_AT_2, BIT_LENGTH_3,  dlpf);
	  IAM20680->ACCEL_DLPF =  dlpf ;
}
/*=========================================================================================================================================
 * @brief     Set  Accelerometer Averaging Filter
 * @param     IAM20680  IAM20680 Struct ACCEL_AVERAGING_FILTER variable
 * @param     avg       Averaging value
 * @return    Nothing 
 ========================================================================================================================================*/ 		
void GB_IAM20680_ACCEL_LP_Averaging_Filter  (GebraBit_IAM20680 * IAM20680 , IAM20680_ACCEL_Averaging_Filter avg )
{   
	  GB_IAM20680_ACCEL_Low_Pass_Filter(IAM20680,ENABLE_DLPF_FCHOICEB);
	  GB_IAM20680_ACCEL_Low_Pass_Filter_Value(IAM20680,IAM20680_ACCEL_DLPF_420);
	  GB_IAM20680_Write_Reg_Bits(IAM20680_ACCEL_CONFIG2 , START_MSB_BIT_AT_5, BIT_LENGTH_2,  avg);
	  IAM20680->ACCEL_AVERAGING_FILTER =  avg ;
}
/*=========================================================================================================================================
 * @brief     Set Sensor Output Sample Rate that controls  data output rate, FIFO sample rate
 * @param     IAM20680   IAM20680 struct ACCEL_SAMPLE_RATE and ACCEL_SAMPLE_DEVIDE variable
 * @param     rate_hz    Sample Rate in Hz
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Output_Sample_Rate (GebraBit_IAM20680 * IAM20680 , uint16_t rate_hz)
{
	uint8_t  gfchoice , gdlpf ;
	GB_IAM20680_Read_Reg_Bits(IAM20680_GYRO_CONFIG, START_MSB_BIT_AT_1, BIT_LENGTH_2,&gfchoice );
	GB_IAM20680_Read_Reg_Bits(IAM20680_CONFIG , START_MSB_BIT_AT_2, BIT_LENGTH_3,  &gdlpf);
	if((gfchoice==0)&&(0<gdlpf)&&(gdlpf<7))
	{
		IAM20680->INTERNAL_SAMPLE_RATE = _1_KHz ;
		IAM20680->SAMPLE_RATE = rate_hz ; 
    IAM20680->SAMPLE_DEVIDE=(IAM20680->INTERNAL_SAMPLE_RATE/rate_hz)-1;  
		GB_IAM20680_Write_Reg_Data( IAM20680_SMPLRT_DIV ,IAM20680->SAMPLE_DEVIDE ); 
	}
	else if((gfchoice==0)&&(1>gdlpf)&&(gdlpf>6))
	{
	  IAM20680->GYRO_SAMPLE_RATE = _8_KHz  ;
	}
}
/*=========================================================================================================================================
 * @brief     Configure FIFO
 * @param     IAM20680       IAM20680 Struct FIFO variable
 * @param     fifo           Configure IAM20680 FIFO according it is FIFO_DISABLE or FIFO_ENABLE 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_FIFO_Configuration ( GebraBit_IAM20680 * IAM20680 , IAM20680_FIFO_Ability fifo  )
{
	IAM20680->FIFO_PACKET_QTY = PACKET_QTY_IN_FULL_FIFO ;  
	if( fifo==Enable )  
	{
		IAM20680->FIFO = FIFO_ENABLE  ;
		GB_IAM20680_FIFO_Reset();
		GB_IAM20680_Access_Serial_Interface_To_FIFO( IAM20680 , Enable );
		GB_IAM20680_FIFO_Mode ( IAM20680 , STOP_ON_FULL_FIFO_SNAPSHOT );
		GB_IAM20680_Write_GYRO_FIFO ( IAM20680 , Enable );
	  GB_IAM20680_Write_ACCEL_FIFO( IAM20680 , Enable );
		GB_IAM20680_Write_TEMP_FIFO ( IAM20680 , Enable );
		GB_IAM20680_FIFO_Overflow_Interrupt( IAM20680 ,Enable ) ;
	}
	else if ( fifo == Disable )
	{
		IAM20680->FIFO = FIFO_DISABLE  ;
		GB_IAM20680_FIFO_Overflow_Interrupt( IAM20680 ,Disable ) ;
		GB_IAM20680_Write_GYRO_FIFO ( IAM20680 , Disable );
	  GB_IAM20680_Write_ACCEL_FIFO( IAM20680 , Disable );
		GB_IAM20680_Write_TEMP_FIFO ( IAM20680 , Disable );
		GB_IAM20680_Access_Serial_Interface_To_FIFO( IAM20680 , Disable );
		GB_IAM20680_FIFO_Reset( );
	}
}
/*=========================================================================================================================================
 * @brief     Set IAM20680 Power Management
 * @param     IAM20680   IAM20680 Struct ACCEL_POWER_MODE and GYRO_POWER_MODE  variable
 * @param     pmode        Determines IAM20680 Accelerometer Power Mode in IAM20680_LOW_NOISE or IAM20680_LOW_POWER or IAM20680_SLEEP_OFF
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Set_Power_Management(GebraBit_IAM20680 * IAM20680 , IAM20680_Power_Mode pmode) 
{	
	
 GB_IAM20680_Temperature(IAM20680 , Enable );
 GB_IAM20680_Accelerometer(IAM20680 , SENSOR_ENABLE );
 GB_IAM20680_Gyroscope(IAM20680 , SENSOR_ENABLE );
 if(pmode==IAM20680_LOW_POWER)
 {
	GB_IAM20680_Sleep_Awake(IAM20680 , IAM20680_AWAKE );
  GB_IAM20680_GYRO_Power_Mode (IAM20680 , IAM20680_LOW_POWER );
  GB_IAM20680_ACCEL_Power_Mode(IAM20680 , IAM20680_LOW_POWER );		 
 }
  else if(pmode==IAM20680_LOW_NOISE)
 {
	GB_IAM20680_Sleep_Awake(IAM20680 , IAM20680_AWAKE );
  GB_IAM20680_GYRO_Power_Mode (IAM20680 , IAM20680_LOW_NOISE );
  GB_IAM20680_ACCEL_Power_Mode(IAM20680 , IAM20680_LOW_NOISE );	  
 }
 else if (pmode==IAM20680_SLEEP_OFF)
 {
	IAM20680->ACCEL_POWER_MODE = IAM20680_SLEEP_OFF ;
  IAM20680->GYRO_POWER_MODE = IAM20680_SLEEP_OFF ;
	GB_IAM20680_Temperature(IAM20680 , Disable );
	GB_IAM20680_Accelerometer(IAM20680 , SENSOR_DISABLE );
	GB_IAM20680_Gyroscope(IAM20680 , SENSOR_DISABLE );
	GB_IAM20680_Sleep_Awake(IAM20680 , IAM20680_SLEEP );
 }
 HAL_Delay(1);
}

/*=========================================================================================================================================
 * @brief     initialize IAM20680
 * @param     IAM20680     initialize IAM20680 according  
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_initialize( GebraBit_IAM20680 * IAM20680 )
{
  HAL_Delay(3);
  GB_IAM20680_Who_am_I(IAM20680);
	GB_IAM20680_Soft_Reset(IAM20680);
	GB_IAM20680_Select_SPI4_Interface(IAM20680 , IS_SPI);
	GB_IAM20680_Set_Power_Management( IAM20680 , IAM20680_LOW_NOISE );
	GB_IAM20680_Set_Clock_Source( IAM20680 , AUTO_SELECT );
	GB_IAM20680_GYRO_TEMP_Low_Pass_Filter (IAM20680,BYPASS_DLPF_FCHOICEB);
	GB_IAM20680_ACCEL_Low_Pass_Filter(IAM20680,BYPASS_DLPF_FCHOICEB);
	//GB_IAM20680_GYRO_TEMP_Low_Pass_Filter_Value (IAM20680,IAM20680_GYRO_TEMP_DLPF_92);
	//GB_IAM20680_ACCEL_Low_Pass_Filter_Value(IAM20680,IAM20680_ACCEL_DLPF_99);
	GB_IAM20680_FIFO_Configuration ( IAM20680 ,FIFO_DISABLE ) ;
	GB_IAM20680_Set_INT_Pin( IAM20680 , ACTIVE_LOW  , OPEN_DRAIN  ,  HELD_STATUS_CLEAR );
	GB_IAM20680_Data_Ready_Interrupt( IAM20680 ,Enable ) ;
}
/*=========================================================================================================================================
 * @brief     Configure IAM20680
 * @param     IAM20680  Configure IAM20680 according 
 * @param     fifo           Configure IAM20680 FIFO according it is FIFO_DISABLE or FIFO_ENABLE 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Configuration(GebraBit_IAM20680 * IAM20680, IAM20680_FIFO_Ability fifo)
{
	//GB_IAM20680_Output_Sample_Rate(IAM20680,SAMPLE_RATE_ODR_HZ );
	GB_IAM20680_GYRO_Full_Scale ( IAM20680 ,FS_1000_DPS ) ;
	GB_IAM20680_ACCEL_Full_Scale( IAM20680 ,FULL_SCALE_4g ) ; 
	GB_IAM20680_FIFO_Configuration ( IAM20680 ,fifo ) ;
	HAL_Delay(20);	
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Temprature from Register 
 * @param     IAM20680  store Raw Data Of Temprature in GebraBit_IAM20680 Staruct REGISTER_RAW_TEMP
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_Temp_Register_Raw_Data(GebraBit_IAM20680 * IAM20680)
{
	uint8_t temp_msb , temp_lsb;
  GB_IAM20680_Read_Reg_Data(IAM20680_TEMP_OUT_H , &temp_msb);
	GB_IAM20680_Read_Reg_Data(IAM20680_TEMP_OUT_L, &temp_lsb);
	IAM20680->REGISTER_RAW_TEMP = (int16_t)((temp_msb << 8) | temp_lsb);
}

/*=========================================================================================================================================
 * @brief     Get Valid Data Of Temprature Base on Datasheet Formula 
 * @param     IAM20680  store Valid Data Of Temprature in GebraBit_IAM20680 Staruct VALID_TEMP_DATA
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_Temp_Valid_Data(GebraBit_IAM20680 * IAM20680)
{ 
  IAM20680->VALID_TEMP_DATA =(IAM20680->REGISTER_RAW_TEMP / 326.8 ) + 25-ROOM_TEMPERATURE_OFFSET;///25 - 8 PCS OFSET!!!
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of X Axis GYRO from Register 
 * @param     IAM20680  store Raw Data Of X Axis GYRO DATA in GebraBit_IAM20680 Staruct REGISTER_RAW_GYRO_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_GYRO_X_Register_Raw_DATA(GebraBit_IAM20680 * IAM20680)
{
	uint8_t gyrox_msb , gyrox_lsb;
  GB_IAM20680_Read_Reg_Data( IAM20680_GYRO_XOUT_H, &gyrox_msb);
	GB_IAM20680_Read_Reg_Data( IAM20680_GYRO_XOUT_L, &gyrox_lsb );
	IAM20680->REGISTER_RAW_GYRO_X = (int16_t)((gyrox_msb << 8) | gyrox_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Y Axis GYRO from Register 
 * @param     IAM20680  store Raw Data Of Y Axis GYRO DATA in GebraBit_IAM20680 Staruct REGISTER_RAW_GYRO_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_GYRO_Y_Register_Raw_DATA(GebraBit_IAM20680 * IAM20680)
{
	uint8_t gyroy_msb , gyroy_lsb;
  GB_IAM20680_Read_Reg_Data( IAM20680_GYRO_YOUT_H, &gyroy_msb);
	GB_IAM20680_Read_Reg_Data( IAM20680_GYRO_YOUT_L, &gyroy_lsb );
	IAM20680->REGISTER_RAW_GYRO_Y = (int16_t)((gyroy_msb << 8) | gyroy_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Z Axis GYRO from Register 
 * @param     IAM20680  store Raw Data Of Z Axis GYRO DATA in GebraBit_IAM20680 Staruct REGISTER_RAW_GYRO_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_GYRO_Z_Register_Raw_DATA(GebraBit_IAM20680 * IAM20680)
{
	uint8_t gyroz_msb , gyroz_lsb;
  GB_IAM20680_Read_Reg_Data( IAM20680_GYRO_ZOUT_H, &gyroz_msb);
	GB_IAM20680_Read_Reg_Data( IAM20680_GYRO_ZOUT_L, &gyroz_lsb );
	IAM20680->REGISTER_RAW_GYRO_Z = (int16_t)((gyroz_msb << 8) | gyroz_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of X Axis GYRO Base on GebraBit_IAM20680 Staruct SCALE_FACTOR 
 * @param     IAM20680  store Valid Data Of X Axis GYRO in GebraBit_IAM20680 Staruct VALID_GYRO_DATA_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_GYRO_DATA_X_Valid_Data(GebraBit_IAM20680 * IAM20680)
{
  IAM20680->VALID_GYRO_DATA_X =(IAM20680->REGISTER_RAW_GYRO_X /IAM20680->PRECISE_GYRO_SF);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Y Axis GYRO Base on GebraBit_IAM20680 Staruct SCALE_FACTOR 
 * @param     IAM20680  store Valid Data Of Y Axis GYRO in GebraBit_IAM20680 Staruct VALID_GYRO_DATA_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_GYRO_DATA_Y_Valid_Data(GebraBit_IAM20680 * IAM20680)
{
  IAM20680->VALID_GYRO_DATA_Y =(IAM20680->REGISTER_RAW_GYRO_Y /IAM20680->PRECISE_GYRO_SF);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Z Axis GYRO Base on GebraBit_IAM20680 Staruct SCALE_FACTOR 
 * @param     IAM20680  store Valid Data Of Z Axis GYRO in GebraBit_IAM20680 Staruct VALID_GYRO_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_GYRO_DATA_Z_Valid_Data(GebraBit_IAM20680 * IAM20680)
{
  IAM20680->VALID_GYRO_DATA_Z =(IAM20680->REGISTER_RAW_GYRO_Z /IAM20680->PRECISE_GYRO_SF);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of X Axis ACCEL from Register 
 * @param     IAM20680  store Raw Data Of X Axis ACCEL DATA in GebraBit_IAM20680 Staruct REGISTER_RAW_ACCEL_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_ACCEL_X_Register_Raw_DATA(GebraBit_IAM20680 * IAM20680)
{
	uint8_t accelx_msb , acclx_lsb;
  GB_IAM20680_Read_Reg_Data( IAM20680_ACCEL_XOUT_H, &accelx_msb);
	GB_IAM20680_Read_Reg_Data( IAM20680_ACCEL_XOUT_L, &acclx_lsb );
	IAM20680->REGISTER_RAW_ACCEL_X = (int16_t)((accelx_msb << 8) | acclx_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Y Axis ACCEL from Register 
 * @param     IAM20680  store Raw Data Of Y Axis ACCEL DATA in GebraBit_IAM20680 Staruct REGISTER_RAW_ACCEL_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_ACCEL_Y_Register_Raw_DATA(GebraBit_IAM20680 * IAM20680)
{
	uint8_t accely_msb , accly_lsb;
  GB_IAM20680_Read_Reg_Data( IAM20680_ACCEL_YOUT_H, &accely_msb);
	GB_IAM20680_Read_Reg_Data( IAM20680_ACCEL_YOUT_L, &accly_lsb );
	IAM20680->REGISTER_RAW_ACCEL_Y = (int16_t)((accely_msb << 8) | accly_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Z Axis ACCEL from Register 
 * @param     IAM20680  store Raw Data Of Z Axis ACCEL DATA in GebraBit_IAM20680 Staruct REGISTER_RAW_ACCEL_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_ACCEL_Z_Register_Raw_DATA(GebraBit_IAM20680 * IAM20680)
{
	uint8_t accelz_msb , acclz_lsb;
  GB_IAM20680_Read_Reg_Data( IAM20680_ACCEL_ZOUT_H, &accelz_msb);
	GB_IAM20680_Read_Reg_Data( IAM20680_ACCEL_ZOUT_L, &acclz_lsb );
	IAM20680->REGISTER_RAW_ACCEL_Z = (int16_t)((accelz_msb << 8) | acclz_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of X Axis ACCEL Base on GebraBit_IAM20680 Staruct SCALE_FACTOR 
 * @param     IAM20680  store Valid Data Of X Axis ACCEL in GebraBit_IAM20680 Staruct VALID_ACCEL_DATA_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_ACCEL_DATA_X_Valid_Data(GebraBit_IAM20680 * IAM20680)
{
	float scale_factor = IAM20680->ACCEL_SCALE_FACTOR;
  IAM20680->VALID_ACCEL_DATA_X =(IAM20680->REGISTER_RAW_ACCEL_X /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Y Axis ACCEL Base on GebraBit_IAM20680 Staruct SCALE_FACTOR 
 * @param     IAM20680  store Valid Data Of Y Axis ACCEL in GebraBit_IAM20680 Staruct VALID_ACCEL_DATA_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_ACCEL_DATA_Y_Valid_Data(GebraBit_IAM20680 * IAM20680)
{
	float scale_factor = IAM20680->ACCEL_SCALE_FACTOR;
  IAM20680->VALID_ACCEL_DATA_Y =(IAM20680->REGISTER_RAW_ACCEL_Y /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Z Axis ACCEL Base on GebraBit_IAM20680 Staruct SCALE_FACTOR 
 * @param     IAM20680  store Valid Data Of Z Axis ACCEL in GebraBit_IAM20680 Staruct VALID_ACCEL_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_ACCEL_DATA_Z_Valid_Data(GebraBit_IAM20680 * IAM20680)
{
	float scale_factor = IAM20680->ACCEL_SCALE_FACTOR;
  IAM20680->VALID_ACCEL_DATA_Z =(IAM20680->REGISTER_RAW_ACCEL_Z /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Temprature Directly 
 * @param     IAM20680       GebraBit_IAM20680 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_Temperature(GebraBit_IAM20680 * IAM20680)
{
  GB_IAM20680_Get_Temp_Register_Raw_Data  (IAM20680);
	GB_IAM20680_Get_Temp_Valid_Data(IAM20680);
}
/*=========================================================================================================================================
 * @brief     Get XYZ GYROSCOPE Directly 
 * @param     IAM20680       GebraBit_IAM20680 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_XYZ_GYROSCOPE(GebraBit_IAM20680 * IAM20680)
{
	GB_IAM20680_Get_GYRO_X_Register_Raw_DATA(IAM20680);
	GB_IAM20680_Get_GYRO_DATA_X_Valid_Data(IAM20680);
	GB_IAM20680_Get_GYRO_Y_Register_Raw_DATA(IAM20680);
	GB_IAM20680_Get_GYRO_DATA_Y_Valid_Data(IAM20680);
	GB_IAM20680_Get_GYRO_Z_Register_Raw_DATA(IAM20680);
	GB_IAM20680_Get_GYRO_DATA_Z_Valid_Data(IAM20680);
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION Directly 
 * @param     IAM20680       GebraBit_IAM20680 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_XYZ_ACCELERATION(GebraBit_IAM20680 * IAM20680)
{
	GB_IAM20680_Get_ACCEL_X_Register_Raw_DATA(IAM20680);
	GB_IAM20680_Get_ACCEL_DATA_X_Valid_Data(IAM20680);
	GB_IAM20680_Get_ACCEL_Y_Register_Raw_DATA(IAM20680);
	GB_IAM20680_Get_ACCEL_DATA_Y_Valid_Data(IAM20680);
	GB_IAM20680_Get_ACCEL_Z_Register_Raw_DATA(IAM20680);
	GB_IAM20680_Get_ACCEL_DATA_Z_Valid_Data(IAM20680);
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION and GYROSCOPE and Temprature Directly From Registers
 * @param     IAM20680       GebraBit_IAM20680 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_ACCEL_GYRO_TEMP_From_Registers(GebraBit_IAM20680 * IAM20680)
{
  if (IS_Ready==GB_IAM20680_Check_Data_Preparation(IAM20680))
	 {
		 IAM20680->GET_DATA =  FROM_REGISTER ; 
	   GB_IAM20680_Get_Temperature( IAM20680 );
	   GB_IAM20680_Get_XYZ_ACCELERATION( IAM20680);
		 GB_IAM20680_Get_XYZ_GYROSCOPE( IAM20680);
	 }
}
/*=========================================================================================================================================
 * @brief     Separate XYZ ACCELERATION , GYROSCOPE and Temprature Data From FIFO and caculate Valid data
 * @param     IAM20680  store Valid Data Of XYZ ACCEL Axis and temp from FIFO TO GebraBit_IAM20680 Staruct VALID_FIFO_DATA_X , VALID_FIFO_DATA_Y ,VALID_FIFO_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_FIFO_Data_Partition_ACCEL_GYRO_XYZ_TEMP(GebraBit_IAM20680 * IAM20680)
{
	uint16_t i,offset=0;
  float accel_scale_factor = IAM20680->ACCEL_SCALE_FACTOR;
	if ( (IAM20680->TEMP_TO_FIFO == Enable ) && ( IAM20680->ACCEL_TO_FIFO == Enable ) && ( IAM20680->GYRO_TO_FIFO == Enable ) )
	{
	 for ( i = 0 ; i < (PACKET_QTY_IN_FULL_FIFO-1) ; i++ )	
		{
			IAM20680->VALID_FIFO_ACCEL_X[i] = ((int16_t)( (IAM20680->FIFO_DATA[offset] << 8) | IAM20680->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2; 
			IAM20680->VALID_FIFO_ACCEL_Y[i] = ((int16_t)( (IAM20680->FIFO_DATA[offset] << 8) | IAM20680->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			IAM20680->VALID_FIFO_ACCEL_Z[i] = ((int16_t)( (IAM20680->FIFO_DATA[offset] << 8) | IAM20680->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			IAM20680->VALID_FIFO_TEMP[i]    = (((int16_t)( (IAM20680->FIFO_DATA[offset] << 8)| IAM20680->FIFO_DATA[offset+1]))/ 326.8) + 25-ROOM_TEMPERATURE_OFFSET ;
			offset += 2;
			IAM20680->VALID_FIFO_GYRO_X[i]  = ((int16_t)( (IAM20680->FIFO_DATA[offset] << 8) | IAM20680->FIFO_DATA[offset+1]))/IAM20680->PRECISE_GYRO_SF ;
			offset += 2; 
			IAM20680->VALID_FIFO_GYRO_Y[i]  = ((int16_t)( (IAM20680->FIFO_DATA[offset] << 8) | IAM20680->FIFO_DATA[offset+1]))/IAM20680->PRECISE_GYRO_SF ;
			offset += 2;
			IAM20680->VALID_FIFO_GYRO_Z[i]  = ((int16_t)( (IAM20680->FIFO_DATA[offset] << 8) | IAM20680->FIFO_DATA[offset+1]))/IAM20680->PRECISE_GYRO_SF ;
			offset += 2;
		}
			IAM20680->VALID_FIFO_ACCEL_X[36] = ((int16_t)( (IAM20680->FIFO_DATA[offset] << 8) | IAM20680->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2; 
			IAM20680->VALID_FIFO_ACCEL_Y[36] = ((int16_t)( (IAM20680->FIFO_DATA[offset] << 8) | IAM20680->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			IAM20680->VALID_FIFO_ACCEL_Z[36] = ((int16_t)( (IAM20680->FIFO_DATA[offset] << 8) | IAM20680->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			IAM20680->VALID_FIFO_TEMP[36]    = (((int16_t)( (IAM20680->FIFO_DATA[offset] << 8)| IAM20680->FIFO_DATA[offset+1]))/ 326.8) + 25-ROOM_TEMPERATURE_OFFSET ;
	}
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION , GYRO and Temprature From FIFO
 * @param     IAM20680       GebraBit_IAM20680 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_ACCEL_GYRO_TEMP_From_FIFO(GebraBit_IAM20680 * IAM20680)
{
	  

	if (IS_Ready==GB_IAM20680_Check_Data_Preparation(IAM20680))
	{
		  if (FIFO_IS_OVERFLOW==GB_IAM20680_Check_FIFO_Overflow(IAM20680))
		  {
        
				GB_IAM20680_Read_FIFO(IAM20680,FIFO_DATA_BUFFER_SIZE);
				GB_IAM20680_GET_FIFO_Count(IAM20680);
				GB_ICM20649_FIFO_Data_Partition_ACCEL_GYRO_XYZ_TEMP(IAM20680); 
        //memset(IAM20680->FIFO_DATA , 0, FIFO_DATA_BUFFER_SIZE*sizeof(uint8_t));				
				GB_IAM20680_FIFO_Reset();
				IAM20680->GET_DATA =  FROM_FIFO ;
		  } 
	}	
}
/*=========================================================================================================================================
 * @brief     Get Data From IAM20680
 * @param     IAM20680       GebraBit_IAM20680 Staruct
 * @param     get_data       Determine Method of reading data from sensoe : FROM_REGISTER or FROM_FIFO
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IAM20680_Get_Data(GebraBit_IAM20680 * IAM20680 , IAM20680_Get_DATA get_data)
{
 if( (get_data == FROM_REGISTER)&&(IAM20680->FIFO == Disable) )
	 GB_IAM20680_Get_ACCEL_GYRO_TEMP_From_Registers(IAM20680);
 else if ((get_data == FROM_FIFO)&&(IAM20680->FIFO == Enable)) 
	GB_IAM20680_Get_ACCEL_GYRO_TEMP_From_FIFO(IAM20680); 
}
/*----------------------------------------------------------------------------------------------------------------------------------------*
 *                                                                      End                                                               *
 *----------------------------------------------------------------------------------------------------------------------------------------*/


/*
	GB_IAM20680_Read_Reg_Data( 0x0 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x01 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x02 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x0D ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x0E ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x0F ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x13 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x14 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x15 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x16 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x17 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x18 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x19 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x1A ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x1B ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x1C ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x1D ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x1E ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x1F ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x23 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x36 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x37 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x38 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x39 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x3A ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x3B ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x3C ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x3D ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x3E ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x3F ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x40 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x41 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x42 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x43 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x44 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x45 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x46 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x47 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x48 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x68 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x69 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x6A ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x6B ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x6C ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x72 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x73 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x74 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x75 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x77 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x78 ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x7A ,&IAM20680_Module.Register_Cache1);	
	GB_IAM20680_Read_Reg_Data( 0x7B ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x7D ,&IAM20680_Module.Register_Cache1);
	GB_IAM20680_Read_Reg_Data( 0x7E ,&IAM20680_Module.Register_Cache1);
	*/

















