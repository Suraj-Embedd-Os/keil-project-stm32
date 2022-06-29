/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern CRC_HandleTypeDef hcrc;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define DUART   &huart3
#define CUART		&huart2

#define BL_ACK 					0xA5
#define BL_NACK 				0x7F
#define BL_VERSION      0x10

#define crc_varified_pass 0
#define crc_varified_fail 1

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void bootloader_jump_to_user_code(void);
void bootloader_uart_read_data(void);
/* USER CODE END Private defines */

void bootloader_get_ver(uint8_t *pBuffer);
void bootloader_get_help(uint8_t *pBuffer);
void bootloader_get_cid(uint8_t *pBuffer);
void bootloader_get_rdp_status(uint8_t *pBuffer);
void bootloader_jump_to_addr(uint8_t *pBuffer);
void bootloader_do_flash_erase(uint8_t *pBuffer);
void bootloader_do_mem_write(uint8_t *pBuffer);
void bootloader_do_en_r_w_protect(uint8_t *pBuffer);
void bootloader_get_mem_read(uint8_t *pBuffer);
void bootloader_get_read_sector_status(uint8_t *pBuffer);
void bootloader_get_otp_read(uint8_t *pBuffer);
void bootloader_do_dis_r_w_protect(uint8_t *pBuffer);



typedef enum{
	BL_GET_VER						=0x51,
	BL_GET_HELP						=0x52,
	BL_GET_CID						=0x53,
	BL_GET_RDP_STATUS			=0x54,
	BL_GO_TO_ADDR					=0x55,
	BL_FLASH_ERASE				=0x56,
	BL_MEM_WRITE					=0x57,
	BL_EN_R_W_PROTECT			=0x58,
	BL_MEM_READ						=0x59,
	BL_READ_SECTOR_STATUS	=0x5A,
	BL_OTP_READ						=0x5B,
	BL_DIS_R_W_PROTECT		=0x5C
	
}command_t;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
