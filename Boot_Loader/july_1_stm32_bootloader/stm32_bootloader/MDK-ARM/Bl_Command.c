
#include "main.h"
#include "stm32f4xx.h"
#include <string.h> 

extern  void printmsg(char *format,...);
uint8_t SUPPORTED_CMD[] = {
	BL_GET_VER,
	BL_GET_HELP,
	BL_GET_CID,
	BL_GET_RDP_STATUS,
	BL_GO_TO_ADDR,
	BL_FLASH_ERASE,
	BL_MEM_WRITE,
	BL_EN_R_W_PROTECT,
	BL_MEM_READ,
	BL_READ_SECTOR_STATUS,
	BL_OTP_READ,
	BL_DIS_R_W_PROTECT	
};

void uart_send_to_host(uint8_t *pBuffer,uint32_t len)
{
	HAL_UART_Transmit(CUART,pBuffer,len,HAL_MAX_DELAY);
}
	
void bootloader_send_ack(uint8_t command_code,uint8_t follow_len)
{
	uint8_t ack_buff[2];
	ack_buff[0]=BL_ACK;
	ack_buff[1]=follow_len;
	
	HAL_UART_Transmit(CUART,ack_buff,sizeof(ack_buff),HAL_MAX_DELAY);
}
void bootloader_send_nack(void)
{
	uint8_t ack_buff=BL_NACK;
	HAL_UART_Transmit(CUART,&ack_buff,sizeof(ack_buff),HAL_MAX_DELAY);
}

uint8_t bootloader_verify_crc(uint8_t *pData,uint32_t len,uint32_t crc_host)
{
	uint32_t uwCRCvalue=0xff;
	for(uint32_t i=0;i<len;++i)
	{
		uint32_t i_data=pData[i];
		uwCRCvalue=HAL_CRC_Accumulate(&hcrc,&i_data,1);
	}
	 /* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(&hcrc);
	if(uwCRCvalue==crc_host)
	{
		return crc_varified_pass;
	}
	 return  crc_varified_fail;
	
}
 uint8_t bootloader_version()
  {
	return BL_VERSION;
  }

void bootloader_get_ver(uint8_t *pBuffer)
{
  	
	 uint8_t bl_version;
	
	printmsg("bootloader get version\n\r");
	
	uint32_t command_lenght=pBuffer[0]+1;
	uint32_t crc_host=*((uint32_t*)(pBuffer+command_lenght-4));
	
	//verify crc checksum
	if(!bootloader_verify_crc(&pBuffer[0],command_lenght-4,crc_host))
	   {
			 //
			 printmsg("checksum sucess\n\r");
			 bootloader_send_ack(pBuffer[0],1);
			 bl_version=bootloader_version();
			 printmsg("Bl version : %d %x\n\r",bl_version,bl_version);
			 uart_send_to_host(&bl_version,sizeof(bl_version));
	   }
		 else
		 {
			  printmsg("checksum failed\n\r");
			  bootloader_send_nack();
			 
		 }
}


void bootloader_get_help(uint8_t *pBuffer)
{
	
	 uint8_t bl_version;
	
	printmsg("bootloader help\n\r");
	
	uint32_t command_lenght=pBuffer[0]+1;
	uint32_t crc_host=*((uint32_t*)(pBuffer+command_lenght-4));
	
	if(!bootloader_verify_crc(&pBuffer[0],command_lenght-4,crc_host))
	   {
			 //
			 printmsg("checksum sucess\n\r");	
			  bootloader_send_ack(pBuffer[0],sizeof(SUPPORTED_CMD));
			  uart_send_to_host(SUPPORTED_CMD,sizeof(SUPPORTED_CMD));
		
	  }
		 else
		 {
			 printmsg("checksum failed\n\r");
			 bootloader_send_nack(); 
			 
		 }
}

uint16_t bootloader_cid()
{
	return (DBGMCU->IDCODE & 0xfff);
}
void bootloader_get_cid(uint8_t *pBuffer)
{
		 uint16_t bl_cid;
	
	printmsg("bootloader get cid\n\r");
	
	uint32_t command_lenght=pBuffer[0]+1;
	uint32_t crc_host=*((uint32_t*)(pBuffer+command_lenght-4));
	
	if(!bootloader_verify_crc(&pBuffer[0],command_lenght-4,crc_host))
	   {
			 //
			 printmsg("checksum sucess\n\r");	
			  bootloader_send_ack(pBuffer[0],2);
			 
			  bl_cid=bootloader_cid();
			 
			  printmsg("bl_cid is %d ,%x\n\r",bl_cid,bl_cid);	
			  uart_send_to_host((uint8_t*)&bl_cid,sizeof(bl_cid));
		
	  }
		 else
		 {
			 printmsg("checksum failed\n\r");
			 bootloader_send_nack(); 
			 
		 }
	
}
uint8_t bootloader_rdp_status()
{
	uint8_t status;
	volatile uint32_t *pob_add=(uint32_t *)0x1fffc000;
	status=(uint8_t)(*pob_add>>8);
	return status;
}
void bootloader_get_rdp_status(uint8_t *pBuffer)
{
	
	uint8_t bl_rdp_status;
	
	printmsg("bootloader get rdp status\n\r");
	
	uint32_t command_lenght=pBuffer[0]+1;
	uint32_t crc_host=*((uint32_t*)(pBuffer+command_lenght-4));
	
	if(!bootloader_verify_crc(&pBuffer[0],command_lenght-4,crc_host))
	   {
			 //
			 printmsg("checksum sucess\n\r");	
			  bootloader_send_ack(pBuffer[0],1);
			 
			  bl_rdp_status=bootloader_rdp_status();
			 
			  printmsg("bl_rdp_status is %d ,%x\n\r",bl_rdp_status,bl_rdp_status);	
			  uart_send_to_host((uint8_t*)&bl_rdp_status,sizeof(bl_rdp_status));
		
	  }
		 else
		 {
			 printmsg("checksum failed\n\r");
			 bootloader_send_nack(); 
			 
		 }
}
void bootloader_jump_to_addr(uint8_t *pBuffer)
{
	
	
}
void bootloader_do_flash_erase(uint8_t *pBuffer)
{
	
	
}
void bootloader_do_mem_write(uint8_t *pBuffer)
{
	
	
}
void bootloader_do_en_r_w_protect(uint8_t *pBuffer)
{
	
	
}
void bootloader_get_mem_read(uint8_t *pBuffer)
{
	
	
}
void bootloader_get_read_sector_status(uint8_t *pBuffer)
{
	
	
}
void bootloader_get_otp_read(uint8_t *pBuffer)
{
	
	
}
void bootloader_do_dis_r_w_protect(uint8_t *pBuffer)
{
	
	
}
















