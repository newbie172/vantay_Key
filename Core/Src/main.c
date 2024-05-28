/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <LCD1602.h>
#include "dwt_delay.h"
#include "stdio.h"
#include "finger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Khai báo các bien
#define Flash_Address     	0x0801FC00
#define TIME     	5000
uint8_t key;
uint8_t d=0,a=0,lansai=0,c,kp,dem;
uint8_t data[6] ; 
uint8_t newdata[6]={'1','2','3','4','5','6'} ;
uint8_t newpass[6]={'1','1','1','1','1','1'} ;
uint8_t Rst[6]={'1','2','3','4','5','6'} ;
uint8_t user[6] ;
uint8_t user1[6] ;
uint32_t time_cho;
char mess[10];
extern char ReceivedData[100];
uint8_t receiver[20];
uint8_t dem;
extern uint8_t pID;
int tmp;
uint8_t ID=0;
int i;
uint8_t Rx_data[1];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//Khoi tao GPIO, UART
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint16_t Read_flash(uint32_t adr)//doc flash
{
	uint16_t *Pntr =(uint16_t *)adr;
	return(*Pntr);
}
//----------------------------------------
void Unlock_flash(void)//mo khoa flash
{
	FLASH->KEYR=0x45670123;
	FLASH->KEYR=0xCDEF89AB;
}
//-----------------------------------------
void Lock_Flash(void)//khoa flash
{
	FLASH->CR=0x00000080;
}
//------------------------------------------
void Erase_Flash (uint32_t adr)//xoa flash
{
	FLASH->CR|=0x00000002;
	FLASH->AR=adr;
	FLASH->CR|=0x00000040;
	while((FLASH->SR&0x00000001));
	FLASH->CR &= ~0x00000042;
}
//------------------------------------------
void Write_Flash (uint32_t adr, uint16_t data)//ghi len flash
{
	FLASH->CR|=0x00000001;
	*(__IO uint16_t*)adr= data;
  while((FLASH->SR&0x00000001));
}
void FlashSave_data(uint32_t adr, uint16_t data)//luu data len flash
{
	//Unlock_flash();
	Erase_Flash(adr);
	Write_Flash(adr, data);
	//Lock_Flash();
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void home ()// man hinh home khi vua mo lcd
{
	lcd_init();
	lcd_clear();
	lcd_send_string ("     Press *   ");
	d=0;
	HAL_Delay(1000);
	
	
}
void beep( uint8_t solan)//phat tieng coi tuy thuoc vao so lan 
{

	while(solan--)
	{
		HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,1);
		HAL_Delay(200);
		HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,0);
		HAL_Delay(150);
	}
}
void dongmocua(void)//ham dong mo cua 
{
	lcd_clear();
	beep(1);
	lcd_send_string("--** CUA MO **--");
	lcd_put_cur(1,2);
	lcd_send_string("XIN MOI VAO");
	HAL_GPIO_WritePin(chotkhoa_GPIO_Port,chotkhoa_Pin,GPIO_PIN_SET);

	HAL_Delay(TIME);
	lcd_clear();
	HAL_Delay(30);
	lcd_send_string("-----******-----");
	lcd_put_cur(1,2);
	lcd_send_string(" CUA DA DONG");
	HAL_Delay(100);
	HAL_GPIO_WritePin(chotkhoa_GPIO_Port,chotkhoa_Pin,GPIO_PIN_RESET);//khi cua da dong thi reset so lan nhap sai pass
	lansai=0;
	home();
}
void restore(void)//Ham khoi phuc lai bo nho flash
{
	int i=0;
  int addr=Flash_Address;
	for(i=0;i<6;i++)
	{
		user[i]=Read_flash(addr);
		addr=addr+4;
	}
	Lock_Flash();
}
void Store (void)// luu tru flash
{
	Unlock_flash();
	int i=0;
  int addr=Flash_Address;
	for(i=0;i<6;i++)
	{
		FlashSave_data(addr,data[i]); 
		addr=addr+4;
	}
	Lock_Flash();
}
char read_keypad (void) //Ham doc ban phim
{
	/* Make ROW 1 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (HANG1_GPIO_Port, HANG1_Pin, GPIO_PIN_RESET);  //Pull the R1 low
	HAL_GPIO_WritePin (HANG2_GPIO_Port, HANG2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (HANG3_GPIO_Port, HANG3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (HANG4_GPIO_Port, HANG4_Pin, GPIO_PIN_SET);  // Pull the R4 High
	
	if (!(HAL_GPIO_ReadPin (COT1_GPIO_Port, COT1_Pin)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (COT1_GPIO_Port, COT1_Pin)));   // wait till the button is pressed
		return '1';
	}
	
	if (!(HAL_GPIO_ReadPin (COT2_GPIO_Port, COT2_Pin)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (COT2_GPIO_Port, COT2_Pin)));   // wait till the button is pressed
		return '2';
	}
	
	if (!(HAL_GPIO_ReadPin (COT3_GPIO_Port, COT3_Pin)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (COT3_GPIO_Port, COT3_Pin)));   // wait till the button is pressed
		return '3';
	}
	
	if (!(HAL_GPIO_ReadPin (COT4_GPIO_Port, COT4_Pin)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (COT4_GPIO_Port, COT4_Pin)));   // wait till the button is pressed
		return 'A';
	}
	
	/* Make ROW 2 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (HANG1_GPIO_Port, HANG1_Pin, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (HANG2_GPIO_Port, HANG2_Pin, GPIO_PIN_RESET);  // Pull the R2 High
	HAL_GPIO_WritePin (HANG3_GPIO_Port, HANG3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (HANG4_GPIO_Port, HANG4_Pin, GPIO_PIN_SET);  // Pull the R4 High
	
	if (!(HAL_GPIO_ReadPin (COT1_GPIO_Port, COT1_Pin)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (COT1_GPIO_Port, COT1_Pin)));   // wait till the button is pressed
		return '4';
	}
	
	if (!(HAL_GPIO_ReadPin (COT2_GPIO_Port, COT2_Pin)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (COT2_GPIO_Port, COT2_Pin)));   // wait till the button is pressed
		return '5';
	}
	
	if (!(HAL_GPIO_ReadPin (COT3_GPIO_Port, COT3_Pin)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (COT3_GPIO_Port, COT3_Pin)));   // wait till the button is pressed
		return '6';
	}
	
	if (!(HAL_GPIO_ReadPin (COT4_GPIO_Port, COT4_Pin)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (COT4_GPIO_Port, COT4_Pin)));   // wait till the button is pressed
		return 'B';
	}
	
	
	/* Make ROW 3 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (HANG1_GPIO_Port, HANG1_Pin, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (HANG2_GPIO_Port, HANG2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (HANG3_GPIO_Port, HANG3_Pin, GPIO_PIN_RESET);  // Pull the R3 High
	HAL_GPIO_WritePin (HANG4_GPIO_Port, HANG4_Pin, GPIO_PIN_SET);  // Pull the R4 High
	
	if (!(HAL_GPIO_ReadPin (COT1_GPIO_Port, COT1_Pin)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (COT1_GPIO_Port, COT1_Pin)));   // wait till the button is pressed
		return '7';
	}
	
	if (!(HAL_GPIO_ReadPin (COT2_GPIO_Port, COT2_Pin)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (COT2_GPIO_Port, COT2_Pin)));   // wait till the button is pressed
		return '8';
	}
	
	if (!(HAL_GPIO_ReadPin (COT3_GPIO_Port, COT3_Pin)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (COT3_GPIO_Port, COT3_Pin)));   // wait till the button is pressed
		return '9';
	}
	
	if (!(HAL_GPIO_ReadPin (COT4_GPIO_Port, COT4_Pin)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (COT4_GPIO_Port, COT4_Pin)));   // wait till the button is pressed
		return 'C';
	}
	
		
	/* Make ROW 4 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (HANG1_GPIO_Port, HANG1_Pin, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (HANG2_GPIO_Port, HANG2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (HANG3_GPIO_Port, HANG3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (HANG4_GPIO_Port, HANG4_Pin, GPIO_PIN_RESET);  // Pull the R4 High
	
	if (!(HAL_GPIO_ReadPin (COT1_GPIO_Port, COT1_Pin)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (COT1_GPIO_Port, COT1_Pin)));   // wait till the button is pressed
		return '*';
	}
	
	if (!(HAL_GPIO_ReadPin (COT2_GPIO_Port, COT2_Pin)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (COT2_GPIO_Port, COT2_Pin)));   // wait till the button is pressed
		return '0';
	}
	
	if (!(HAL_GPIO_ReadPin (COT3_GPIO_Port, COT3_Pin)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (COT3_GPIO_Port, COT3_Pin)));   // wait till the button is pressed
		return '#';
	}
	
	if (!(HAL_GPIO_ReadPin (COT4_GPIO_Port, COT4_Pin)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (COT4_GPIO_Port, COT4_Pin)));   // wait till the button is pressed
		return 'D';
	}
	return 0;
}

void quet_key(void)//ham quet ban phim
{
		kp = 0;
		do
		{
			kp = read_keypad();
		}	
		while(!kp);
		
		data[d] = kp;
		HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_SET);
		lcd_put_cur(1,0+d);
		lcd_send_data(data[d]);
		HAL_Delay(220);
		HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_RESET);
		d++;
}
void quet_key1(void)//ham quet ban phim
{
		kp = 0;
		do
		{
			kp = read_keypad();
		}	
		while(!kp);
		
		data[d] = kp;
		if(data[d]==	'C')	
		{
			d=0,a=-1;
			lcd_clear();
			lcd_send_string("Moi Nhap MK");
		}
		else
		{
			HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_SET);
			lcd_put_cur(1,0+d);
			if(data[d]!='#')
				lcd_send_data('*');
			else
			lcd_send_data(data[d]);
			HAL_Delay(50);
			HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_RESET);
			d++;
		}
}

//---------- them van tay---------------
void add_finger()
{
	vitri2:
	while(1)
	{
		collect_finger();
		lcd_clear();
		lcd_put_cur(0,0);
		lcd_send_string("  Them Van Tay!!     ");
		lcd_put_cur(1,0);
		lcd_send_string("Dat Van Tay!!     ");
		HAL_Delay(1000);
	// dat tay vao
		lcd_put_cur(1,0);
		lcd_send_string("Reading finger...!!     ");
		tmp=0xff;
		while(tmp!=0x00){
			collect_finger();
			collect_finger();	
			tmp= collect_finger();					
		}
		tmp=0xff;
		lcd_put_cur(1,0);
		lcd_send_string("Remove Finger!!   ");HAL_Delay(100);
		lcd_put_cur(1,0);
		lcd_send_string("Processing Finger!!   ");
		tmp=0xff;
		while(tmp!=0x00){			
		tmp=img2tz(0x01);	
		}
		lcd_put_cur(1,0);
		lcd_send_string("dat lai van tay !!   ");HAL_Delay(100);
		lcd_put_cur(1,0);
		lcd_send_string("Reading finger...!!     ");
		tmp=0xff;
		while(tmp!=0x00)	{
			collect_finger();	
			collect_finger();	
			tmp=collect_finger();
		}
		lcd_put_cur(1,0);
		lcd_send_string("Remove Finger!!   ");HAL_Delay(100);
		tmp=0xff;
		lcd_put_cur(1,0);
		lcd_send_string("Processing Finger!!   ");
		while(tmp!=0x00)	{tmp=img2tz(0x02);}
		tmp=0xff;
		// kiem tra 2 buff co trung nhau khong
		while(tmp!=0x00)
		{
			tmp=match();	//HAL_Delay(100);
			if(tmp==0x08||tmp==0x01)
			{
				// loi, lam lai
					lcd_put_cur(1,0);
					lcd_send_string("LOI, Lam Lai!!   ");HAL_Delay(1500);
				goto vitri2;
			}
		}
		tmp=0xff;
		while(tmp!=0x00){tmp=regmodel();HAL_Delay(100);}
		tmp=0xff;
		while(tmp!=0x00){tmp=store(ID);HAL_Delay(100);}			// luu id
		lcd_put_cur(1,0);
		lcd_send_string("  Save Finger!    ");
		beep(5);

				/***************** DA LUU XONG**************************/
		HAL_Delay(1500);
		tmp=0xff;	
		break;
	}
}
						//----------end them van tay---------------
void read_finger()
{
/**************************BEgin Doc van tay*****************************/
	tmp=0xff;
	time_cho=HAL_GetTick();
	while(tmp!=0x00){
		tmp=collect_finger();
		if(HAL_GetTick()-time_cho>=1600) {
		time_cho=HAL_GetTick();
		return;}
		
	}
	tmp=0xff;
	if(tmp!=0x00){tmp=img2tz(0x01);}
	tmp=0xff;
	tmp=search();
	if(tmp==0x00)
	{
		tmp=0xff;	// co van tay
		lcd_put_cur(1,0);
		lcd_send_string("Mo Cua!");
		sprintf(mess," #id = %c  ",pID);
		lcd_send_string(mess);
		dongmocua();
	}
	if(tmp==0x09)	// khong co van tay
	{
		tmp=0xff;
		lcd_put_cur(1,0);
		lcd_send_string(" Van Tay Sai!!     ");beep(5); HAL_Delay(1000);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	DWT_Init();
	lcd_init();
	lcd_send_string(" Khoa Van Tay");
	lcd_put_cur(1,1);
	lcd_send_string("DATN HaUI 2024");
	HAL_Delay(1000);
	lcd_clear();
	lcd_send_string ("     Press *     ");
	restore();
	HAL_UART_Receive_IT(&huart1, (uint8_t*)Rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		switch(read_keypad())//doc ban phim
		{
			case '*' :
				{
					HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_SET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_RESET);
					lcd_clear();
					HAL_Delay(30);
					lcd_send_string("Moi Nhap MK");
					HAL_Delay(500);
					for(a=0;a<6;a++)
					{
						quet_key1();
					}
					d=0;
					if(data[0] == '*' && data[1] == '#' && data [2] == '*' && data[3] == '#' && data[4] == '*' && data[5] == '#')
					{
						lcd_clear();
						lcd_send_string("Nhap MK Cu");
						for(a=0;a<6;a++)
						{
							quet_key();
						}
						d=0;
						if(data[0] == newdata[0] && data[1] == newdata[1] && data [2] == newdata[2]
							&& data[3] == newdata[3] && data[4] == newdata[4] && data[5] == newdata[5])
						{
							
							lcd_clear();
							lcd_send_string("Nhap MK Moi");
							for(a=0;a<6;a++)
							{
								quet_key();
							}
							d=0;
							Store();
							HAL_Delay(50);
							lcd_clear();
							lcd_send_string("Doi MK Thanh Cong");
							beep(2);
							restore();
							home();
						}
						else goto vitri1;
					}
				else if (data[0] == newdata[0] && data[1] == newdata[1] && data [2] == newdata[2]
					&& data[3] == newdata[3] && data[4] == newdata[4] && data[5] == newdata[5])
				{
					dongmocua();
				}
				else if (data[0] == newpass[0] && data[1] == newpass[1] && data [2] == newpass[2]
					&& data[3] == newpass[3] && data[4] == newpass[4] && data[5] == newpass[5])
				{
					dongmocua();
				}
				else 
				{
					vitri1:
						lansai++;
						lcd_clear();
						HAL_Delay(30);
						lcd_put_cur(0,0);
						lcd_send_string("-----******-----");
						lcd_put_cur(1,0);
						lcd_send_string("MK KHONG DUNG");
						beep(5);						
						if (lansai==3)
						{
							lcd_clear();
							lcd_put_cur(0,0);
							lcd_send_string("--SAI QUA NHIEU-- ");
							lcd_put_cur(1,0);
							lcd_send_string("XIN THU LAI");
							dem=10;
							time_cho=HAL_GetTick();
							while(1)
							{
									if(HAL_GetTick()-time_cho==1000)
									{
										dem--;
										time_cho=HAL_GetTick();
										sprintf(mess,"%d ", dem);
										lcd_put_cur(1,14);
										lcd_send_string(mess);
									}
									if (dem==0)	break;
							}	
							home();								
						}
						else if(lansai==4)
						{
							lcd_clear();
							HAL_Delay(30);
							lcd_put_cur(0,0);
							lcd_send_string("--SAI QUA NHIEU-- ");
							lcd_put_cur(1,0);
							lcd_send_string("XIN THU LAI");
							dem=20;
							time_cho=HAL_GetTick();
							while(1)
							{
									if(HAL_GetTick()-time_cho==1000)
									{
										dem--;
										time_cho=HAL_GetTick();
										sprintf(mess,"%d ", dem);
										lcd_put_cur(1,14);
										lcd_send_string(mess);
									}
									if (dem==0)	break;
							}		
						home();							
						}
						else if(lansai>=5)
						{
							while(1){
								lcd_clear();
								lcd_put_cur(0,0);
								lcd_send_string("--Khoa Mat Khau-- ");
								lcd_put_cur(1,0);
								lcd_send_string("Moi Nhap Van Tay");
								read_finger();
							}
						}
				}
				break;
			}
			case 'D' :											///  reset pass mk mac dinh 123456
			{
				lcd_clear();
				lcd_send_string("    Advanced");
				for(a=0;a<6;a++)
				{
					quet_key();
				}
				d=0;
				if(data[0] == 'D' && data[1] == 'C' && data [2] == 'D' && data[3] == 'C' && data[4] == 'D' && data[5] == 'C')
				{
					lcd_clear();
					lcd_put_cur(0,0);
					lcd_send_string("RESET PASSWORD ");
					Unlock_flash();
					int i=0;
					int addr=Flash_Address;
					for(i=0;i<6;i++)
					{
						FlashSave_data(addr,Rst[i]); 
						addr=addr+4;
					}
					Lock_Flash();
					HAL_Delay(2000);
					empty();
					NVIC_SystemReset();
				}
				else 
				{
					home();
				}
			}
			case 'A':
			{
				lcd_clear();
				lcd_send_string("Nhap MK");				// nhap mat khau de them van tay
				for(a=0;a<6;a++)
				{
					quet_key();
				}
				d=0;
				if(data[0] == newpass[0] && data[1] == newpass[1] && data [2] == newpass[2]
					&& data[3] == newpass[3] && data[4] == newpass[4] && data[5] == newpass[5])
				{
					lcd_clear();
					lcd_send_string("Them van tay");
					lcd_put_cur(1,0);
					lcd_send_string("ID = ");
					d= 5;
					for(a=0;a<1;a++)
					{
						quet_key();
					}
					ID=data[5];
					HAL_Delay(500);
					add_finger();
				}
				else
				{
					goto vitri1;
				}
				home();
				break;
			}
			case 'B':																							// xoa van tay
			{
				lcd_clear();
				lcd_send_string("Nhap MK");				// nhap mat khau de them van tay
				for(a=0;a<6;a++)
				{
					quet_key();
				}
				d=0;
				if(data[0] == newpass[0] && data[1] == newpass[1] && data [2] == newpass[2]
					&& data[3] == newpass[3] && data[4] == newpass[4] && data[5] == newpass[5])
				{
					lcd_clear();
					lcd_send_string("Xoa van tay");
					lcd_put_cur(1,0);
					lcd_send_string("ID = ");
					d= 5;
					for(a=0;a<1;a++)
					{
						quet_key();
					}
					ID=data[5];
					HAL_Delay(10);
					del(ID);
					lcd_put_cur(1,0);
					lcd_send_string("  Da Xoa      ");
					HAL_Delay(200);
				}
				else
					goto vitri1;
				home();
				break;
			}
			default:																		// mac dinh chay van t
			{
				if(Rx_data[0] == 'a'){
						lcd_clear();
						lcd_send_string("--** CUA MO **--");
						lcd_put_cur(1,2);
						lcd_send_string("XIN MOI VAO");
						HAL_GPIO_WritePin(chotkhoa_GPIO_Port,chotkhoa_Pin,GPIO_PIN_SET);
				}
				else if(Rx_data[0] == 'z'){
					lcd_clear();
					HAL_Delay(30);
					lcd_send_string("-----******-----");
					lcd_put_cur(1,2);
					lcd_send_string(" CUA DA DONG");
					HAL_Delay(2000);
					HAL_GPIO_WritePin(chotkhoa_GPIO_Port,chotkhoa_Pin,GPIO_PIN_RESET);
					home();
				}
				else{
					read_finger();
					lcd_clear();
					lcd_send_string ("     Press *     ");
				}
				//if(HAL_GPIO_ReadPin(BTN_GPIO_Port,BTN_Pin)==0)	dongmocua();
				break;
			}
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HANG4_Pin|HANG3_Pin|HANG2_Pin|HANG1_Pin
                          |EN_Pin|buzzer_Pin|RW_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D6_Pin|D7_Pin|D5_Pin|D4_Pin
                          |chotkhoa_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : COT4_Pin */
  GPIO_InitStruct.Pin = COT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(COT4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : COT3_Pin */
  GPIO_InitStruct.Pin = COT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COT3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COT2_Pin COT1_Pin */
  GPIO_InitStruct.Pin = COT2_Pin|COT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HANG4_Pin HANG3_Pin HANG2_Pin HANG1_Pin */
  GPIO_InitStruct.Pin = HANG4_Pin|HANG3_Pin|HANG2_Pin|HANG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D6_Pin D7_Pin D5_Pin D4_Pin
                           chotkhoa_Pin */
  GPIO_InitStruct.Pin = D6_Pin|D7_Pin|D5_Pin|D4_Pin
                          |chotkhoa_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin buzzer_Pin RW_Pin RS_Pin */
  GPIO_InitStruct.Pin = EN_Pin|buzzer_Pin|RW_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart->Instance == USART1){
		HAL_UART_Receive_IT(&huart1, (uint8_t*)Rx_data, 1);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
