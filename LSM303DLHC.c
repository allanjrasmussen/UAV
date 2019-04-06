/*
 * LSM303DLHC.c
 *
 *  Created on: 20. mar. 2019
 *      Author: ajr
 */

#include "LSM303DLHC.h"


#define ABS(x)         (x < 0) ? (-x) : x

#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
#define PI                         (float)     3.14159265f

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */



/**
  * @brief  Set LSM303DLHC Mag Initialization.
  * @param  LSM303DLHC_InitStruct: pointer to a LSM303DLHC_MagInitTypeDef structure
  *         that contains the configuration setting for the LSM303DLHC.
  * @retval None
  */
void MagInit(I2C_HandleTypeDef *hi2c, Mag_InitTypeDef *InitStruct)
{
  uint8_t cra_regm = 0x00, crb_regm = 0x00, mr_regm = 0x00;

  /* Configure the low level interface ---------------------------------------*/
  // LSM303DLHC_LowLevel_Init();

  /* Configure MEMS: temp and Data rate */
  cra_regm |= (uint8_t) (InitStruct->Temperature_Sensor | InitStruct->MagOutput_DataRate);

  /* Configure MEMS: full Scale */
  crb_regm |= (uint8_t) (InitStruct->MagFull_Scale);

  /* Configure MEMS: working mode */
  mr_regm |= (uint8_t) (InitStruct->Working_Mode);

  /* Write value to Mag MEMS CRA_REG regsister */
  HAL_I2C_Mem_Write(hi2c, MAG_I2C_ADDRESS, CRA_REG_M, 1, &cra_regm, 1, 100);

  /* Write value to Mag MEMS CRB_REG regsister */
  HAL_I2C_Mem_Write(hi2c, MAG_I2C_ADDRESS, CRB_REG_M, 1, &crb_regm, 1, 100);

  /* Write value to Mag MEMS MR_REG regsister */
  HAL_I2C_Mem_Write(hi2c, MAG_I2C_ADDRESS, MR_REG_M, 1, &mr_regm, 1, 100);
}


/** @defgroup STM32F3_DISCOVERY_LSM303DLHC_Private_Functions
  * @{
  */

/**
  * @brief  Set LSM303DLHC Initialization.
  * @param  LSM303DLHC_InitStruct: pointer to a LSM303DLHC_InitTypeDef structure
  *         that contains the configuration setting for the LSM303DLHC.
  * @retval None
  */
void AccInit(I2C_HandleTypeDef *hi2c, Acc_InitTypeDef *InitStruct)

{
  uint8_t ctrl1 = 0x00, ctrl4 = 0x00;

  /* Configure the low level interface ---------------------------------------*/
 // LSM303DLHC_LowLevel_Init();


  /* Configure MEMS: data rate, power mode, full scale and axes */
  ctrl1 |= (uint8_t) (InitStruct->Power_Mode | InitStruct->AccOutput_DataRate | \
                    InitStruct->Axes_Enable);

  ctrl4 |= (uint8_t) (InitStruct->BlockData_Update | InitStruct->Endianness | \
                    InitStruct->AccFull_Scale|InitStruct->High_Resolution);


  /**
    * @brief  Write an amount of data in blocking mode to a specific memory address
    * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
    *                the configuration information for the specified I2C.
    * @param  DevAddress Target device address: The device 7 bits address value
    *         in datasheet must be shifted to the left before calling the interface
    * @param  MemAddress Internal memory address
    * @param  MemAddSize Size of internal memory address
    * @param  pData Pointer to data buffer
    * @param  Size Amount of data to be sent
    * @param  Timeout Timeout duration
    * @retval HAL status
    * HAL_StatusTypeDef HAL_I2C_Mem_Write
    * (I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
    * uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
    */
  /* Write value to ACC MEMS CTRL_REG1 regsister */
  HAL_I2C_Mem_Write(hi2c, ACC_I2C_ADDRESS, CTRL_REG1_A, 1, &ctrl1, 1, 100);

  /* Write value to ACC MEMS CTRL_REG4 regsister */
  HAL_I2C_Mem_Write(hi2c, ACC_I2C_ADDRESS, CTRL_REG4_A, 1, &ctrl4, 1, 100);
}


/**
  * @brief  Set High Pass Filter Modality
  * @param  LSM303DLHC_FilterStruct: pointer to a LSM303DLHC_FilterConfigTypeDef structure
  *         that contains the configuration setting for the LSM303DLHC.
  * @retval None
  */
void AccFilterConfig(I2C_HandleTypeDef *hi2c, Acc_FilterConfigTypeDef *FilterStruct)
{
  uint8_t tmpreg;


  /**
    * @brief  Read an amount of data in blocking mode from a specific memory address
    * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
    *                the configuration information for the specified I2C.
    * @param  DevAddress Target device address: The device 7 bits address value
    *         in datasheet must be shifted to the left before calling the interface
    * @param  MemAddress Internal memory address
    * @param  MemAddSize Size of internal memory address
    * @param  pData Pointer to data buffer
    * @param  Size Amount of data to be sent
    * @param  Timeout Timeout duration
    * @retval HAL status
    * HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
    * uint16_t MemAddress, uint16_t MemAddSize,
    * uint8_t *pData, uint16_t Size, uint32_t Timeout)
    */


  /* Read CTRL_REG2 register */
  HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, CTRL_REG2_A, 1, &tmpreg, 1, 100);

  tmpreg &= 0x0C;

  /* Configure MEMS: mode, cutoff frquency, Filter status, Click, AOI1 and AOI2 */
  tmpreg |= (uint8_t) (FilterStruct->HighPassFilter_Mode_Selection |\
                      FilterStruct->HighPassFilter_CutOff_Frequency|\
                      FilterStruct->HighPassFilter_AOI1|\
                      FilterStruct->HighPassFilter_AOI2);

  /* Write value to ACC MEMS CTRL_REG2 regsister */
  HAL_I2C_Mem_Write(hi2c, ACC_I2C_ADDRESS,CTRL_REG2_A, 1, &tmpreg, 1,100);
}



 void CompassConfig(I2C_HandleTypeDef *hi2c)
{
  Mag_InitTypeDef InitStructure;
  Acc_InitTypeDef Acc_InitStructure;
  Acc_FilterConfigTypeDef Filter_InitStructure;

  /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate*/
  InitStructure.Temperature_Sensor = TEMPSENSOR_DISABLE;
  InitStructure.MagOutput_DataRate = ODR_30_HZ ;
  InitStructure.MagFull_Scale = FS_8_1_GA;
  InitStructure.Working_Mode = CONTINUOS_CONVERSION;
  MagInit(hi2c, &InitStructure);

   /* Fill the accelerometer structure*/
  Acc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
  Acc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
  Acc_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE;
  Acc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
  Acc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
  Acc_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
  Acc_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE;
  /* Configure the accelerometer main parameters*/
  AccInit(hi2c, &Acc_InitStructure);

  /* Fill the accelerometer LPF structure*/
  Filter_InitStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
  Filter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
  Filter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
  Filter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

  /* Configure the accelerometer LPF main parameters*/
  AccFilterConfig(hi2c, &Filter_InitStructure);
}


 /**
 * @brief Read LSM303DLHC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit rappresentation)
 * @param pnData: pointer to float buffer where to store data
 * @retval None
 */
 void CompassReadAcc(I2C_HandleTypeDef *hi2c, float* pfData)
 {
  // int16_t pnRawData[3];
   uint8_t ctrl4;
   uint8_t ctrl5;
   uint8_t buffer[6], cDivider;
   uint8_t i = 0;
   float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;

   /* Read the register content */
   //HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, CTRL_REG4_A, 1, ctrlx, 2, 100);
   HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, CTRL_REG4_A, I2C_MEMADD_SIZE_8BIT, &ctrl4, 1, 100);
  // HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, CTRL_REG5_A, I2C_MEMADD_SIZE_8BIT, &ctrl5, 1, 100);
   //HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, CTRL_REG4_A, 1, ctrlx+1, 1, 100);
   //HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, OUT_X_L_A, 1, buffer, 6, 100);
   uint8_t dataReady = 0;
     do
     {
    	 HAL_Delay(2);
  	   HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, STATUS_REG_A, I2C_MEMADD_SIZE_8BIT, &dataReady, 1, 1000);
     }while(dataReady && 0x08 == 0);

   HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, OUT_X_H_A, I2C_MEMADD_SIZE_8BIT, buffer, 1, 1000);
   HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, OUT_X_L_A, I2C_MEMADD_SIZE_8BIT, buffer+1, 1, 1000);
   HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, OUT_Y_H_A, I2C_MEMADD_SIZE_8BIT, buffer+2, 1, 1000);
   HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, OUT_Y_L_A, I2C_MEMADD_SIZE_8BIT, buffer+3, 1, 1000);
   HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, OUT_Z_H_A, I2C_MEMADD_SIZE_8BIT, buffer+4, 1, 1000);
   HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, OUT_Z_L_A, I2C_MEMADD_SIZE_8BIT, buffer+5, 1, 1000);
/*
   if(ctrl5 & 0x40)
     cDivider=64;
   else
     cDivider=16;
*/
   /* check in the control register4 the data alignment*/
  // if(!(ctrl4 & 0x40) || (ctrl5 & 0x40)) /* Little Endian Mode or FIFO mode */
  // {
  //   for(i=0; i<3; i++)
  //   {
  //     pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i])/(cDivider);
  //   }
  // }
  // else /* Big Endian Mode */
  // {
  //   for(i=0; i<3; i++)
  //     pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
  // }
   /* Read the register content */
  // HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDRESS, CTRL_REG4_A, 1, ctrlx, 2, 100);


 //  if(ctrl5&0x40)
  // {
     /* FIFO mode */
//     LSM_Acc_Sensitivity = 0.25;
  // }
 //  else
  // {
     /* normal mode */
     /* switch the sensitivity value set in the CRTL4*/
     switch(ctrl4 & 0x30)
     {
     case LSM303DLHC_FULLSCALE_2G:
       LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
       break;
     case LSM303DLHC_FULLSCALE_4G:
       LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
       break;
     case LSM303DLHC_FULLSCALE_8G:
       LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
       break;
     case LSM303DLHC_FULLSCALE_16G:
       LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
       break;
     }
  // }

   /* Obtain the mg value for the three axis */
  // for(i=0; i<3; i++)
  // {
  //   pfData[i]=(float)pnRawData[i]/LSM_Acc_Sensitivity;
  // }
   //float divider = 1/(float)cDivider;
  // for(i=0; i<3; i++)
   //   {
	   //int16_t a = ((uint16_t)buffer[2*i] << 8) + buffer[2*i+1];
	  // float b = (float)a/((float)cDivider);
      //  pfData[i]= (float)b/(LSM_Acc_Sensitivity);
   //     pfData[i]=((float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])))/16800;//LSM_Acc_Sensitivity;
   //   }

   for(i = 0; i < 3; i++)
   {
	   int8_t sign = 1;
	   if(buffer[2 * i] && 0x80) sign = -1;
	   //int16_t hByte = (buffer[2 * i] & 127 ) << 8;
	   int16_t hByte = buffer[2 * i] << 8;
	   hByte += buffer[2 *i + 1];
	   float tal = (float)hByte/17039;//32768;
	   tal *= sign;
	   pfData[i] = tal;

   }
 }


 /**
   * @brief  calculate the magnetic field Magn.
 * @param  pfData: pointer to the data out
   * @retval None
   */
 void CompassReadMag (I2C_HandleTypeDef *hi2c, float* pfData)
 {
   static uint8_t buffer[6] = {0};
   uint8_t CTRLB = 0;
   uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
   uint8_t i =0;
   HAL_I2C_Mem_Read(hi2c, MAG_I2C_ADDRESS, CRB_REG_M, I2C_MEMADD_SIZE_8BIT, &CTRLB, 1, 100);

   uint8_t dataReady = 0;
	do
	{
	 HAL_Delay(2);
	   HAL_I2C_Mem_Read(hi2c, MAG_I2C_ADDRESS, SR_REG_M, I2C_MEMADD_SIZE_8BIT, &dataReady, 1, 100);
	}while(dataReady && 0x01 == 0);
   HAL_I2C_Mem_Read(hi2c, MAG_I2C_ADDRESS, OUT_X_H_M, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);
   HAL_I2C_Mem_Read(hi2c, MAG_I2C_ADDRESS, OUT_X_L_M, I2C_MEMADD_SIZE_8BIT, buffer+1, 1, 100);
   HAL_I2C_Mem_Read(hi2c, MAG_I2C_ADDRESS, OUT_Y_H_M, I2C_MEMADD_SIZE_8BIT, buffer+2, 1, 100);
   HAL_I2C_Mem_Read(hi2c, MAG_I2C_ADDRESS, OUT_Y_L_M, I2C_MEMADD_SIZE_8BIT, buffer+3, 1, 100);
   HAL_I2C_Mem_Read(hi2c, MAG_I2C_ADDRESS, OUT_Z_H_M, I2C_MEMADD_SIZE_8BIT, buffer+4, 1, 100);
   HAL_I2C_Mem_Read(hi2c, MAG_I2C_ADDRESS, OUT_Z_L_M, I2C_MEMADD_SIZE_8BIT, buffer+5, 1, 100);
   /* Switch the sensitivity set in the CRTLB*/
   switch(CTRLB & 0xE0)
   {
   case FS_1_3_GA:
     Magn_Sensitivity_XY = M_SENSITIVITY_XY_1_3Ga;
     Magn_Sensitivity_Z = M_SENSITIVITY_Z_1_3Ga;
     break;
   case FS_1_9_GA:
     Magn_Sensitivity_XY = M_SENSITIVITY_XY_1_9Ga;
     Magn_Sensitivity_Z = M_SENSITIVITY_Z_1_9Ga;
     break;
   case FS_2_5_GA:
     Magn_Sensitivity_XY = M_SENSITIVITY_XY_2_5Ga;
     Magn_Sensitivity_Z = M_SENSITIVITY_Z_2_5Ga;
     break;
   case FS_4_0_GA:
     Magn_Sensitivity_XY = M_SENSITIVITY_XY_4Ga;
     Magn_Sensitivity_Z = M_SENSITIVITY_Z_4Ga;
     break;
   case FS_4_7_GA:
     Magn_Sensitivity_XY = M_SENSITIVITY_XY_4_7Ga;
     Magn_Sensitivity_Z = M_SENSITIVITY_Z_4_7Ga;
     break;
   case FS_5_6_GA:
     Magn_Sensitivity_XY = M_SENSITIVITY_XY_5_6Ga;
     Magn_Sensitivity_Z = M_SENSITIVITY_Z_5_6Ga;
     break;
   case FS_8_1_GA:
     Magn_Sensitivity_XY = M_SENSITIVITY_XY_8_1Ga;
     Magn_Sensitivity_Z = M_SENSITIVITY_Z_8_1Ga;
     break;
   }

   for(i=0; i<2; i++)
   {
     pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/Magn_Sensitivity_XY;
   }
   pfData[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_Z;
 }

