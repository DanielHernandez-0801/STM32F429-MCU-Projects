/****** 
1.The Fist Extern button (named extBtn1)  connected to PC1 (canot use PB1, for 429i-DISCO ,pb1 is used by LCD), ExtBtn1_Config()  //
		2014: canot use pin PB1, for 429i-DISCO ,pb1 is used by LCD. if use this pin, always interrupt by itself
					can not use pin PA1, used by gyro. if use this pin, never interrupt
					pd1----WILL ACT AS PC13, To trigger the RTC timestamp event
					....ONLY PC1 CAN BE USED TO FIRE EXTI1 !!!!
2. the Second external button (extBtn2) may connect to pin PD2.  ExtBtn2_Config() --The pin PB2 on the board have trouble.
    when connect extern button to the pin PB2, the voltage at this pin is not 3V as it is supposed to be, it is 0.3V, why?
		so changed to use pin PD2.
	
	 
		PA2: NOT OK. (USED BY LCD??)
		PB2: ??????
		PC2: ok, BUT sometimes (every 5 times around), press pc2 will trigger exti1, which is configured to use PC1. (is it because of using internal pull up pin config?)
		      however, press PC1 does not affect exti 2. sometimes press PC2 will also affect time stamp (PC13)
		PD2: OK,     
		PE2:  OK  (PE3, PE4 PE5 , seems has no other AF function, according to the table in manual for discovery board)
		PF2: NOT OK. (although PF2 is used by SDRAM, it affects LCD. press it, LCD will flick and displayed chars change to garbage)
		PG2: OK
		
	3. RTC and RTC_Alarm have been configured.
 
*/


/* Includes ------------------------------------------------------------------*/
#include "main.h"


/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define COLUMN(x) ((x) * (((sFONT *)BSP_LCD_GetFont())->Width))    //see font.h, for defining LINE(X)



/* Private macro -------------------------------------------------------------*/

 

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef  I2c3_Handle;

RTC_HandleTypeDef RTCHandle;
RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStructure;



HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 


//memory location to write to in the device
__IO uint16_t memLocation = 0x000A; //pick any location within range

// Variables
int time[4];
int date[4];
int time1[4];
int time2[4];
uint8_t display_old = 0;
uint8_t display_full = 0;
uint8_t changing = 0;
uint16_t inc = 1;
uint8_t count = 0;


char lcd_buffer[14];



void RTC_Config(void);
void RTC_AlarmAConfig(void);



void ExtBtn1_Config(void);  //for the first External button
void ExtBtn2_Config(void);

// Helper Functions
void GetTime(RTC_HandleTypeDef *hrtc);
void GetDate(RTC_HandleTypeDef *hrtc);
void DisplayTime(int *t, int LCDrow);
void DisplayDate(int *d, int LCDrow);
void logTime();
void Display_Past_Two();
void clearScreen();
void SetDateAndTime();
uint8_t buttonHeld(GPIO_TypeDef *port, uint16_t pin);
 

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	
	/* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
	
	 /* Configure the system clock to 180 MHz */
  SystemClock_Config();
	
	
	//Init Systic interrupt so can use HAL_Delay() function 
	HAL_InitTick(0x0000); // set systick's priority to the highest.
                        
  
	
	//Configure LED3 and LED4 ======================================

	BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);

	//configure the USER button as exti mode. 
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);   // BSP_functions in stm32f429i_discovery.c
	


	//Use this line, so do not need extiline0_config()
	//configure external push-buttons and interrupts
	
	ExtBtn1_Config();  //for the first External button
	ExtBtn2_Config();
	
	
	
	
//-----------Init LCD 
	BSP_LCD_Init();
	//BSP_LCD_LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address);
	BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);   //LCD_FRAME_BUFFER, defined as 0xD0000000 in _discovery_lcd.h
														// the LayerIndex may be 0 and 1. if is 2, then the LCD is dark.
	//BSP_LCD_SelectLayer(uint32_t LayerIndex);
	BSP_LCD_SelectLayer(0);
	//BSP_LCD_SetLayerVisible(0, ENABLE); //do not need this line.
	BSP_LCD_Clear(LCD_COLOR_WHITE);  //need this line, otherwise, the screen is dark	
	BSP_LCD_DisplayOn();
 
	BSP_LCD_SetFont(&Font20);  //the default font,  LCD_DEFAULT_FONT, which is defined in _lcd.h, is Font24
	
	
		LCD_DisplayString(3, 2, (uint8_t *)"MT3TA4 LAB 3");
		//LCD_DisplayString(6, 2, (uint8_t *)"Testing EEPROM....");
		
		//HAL_Delay(1000);   //display for 1 second
		
		//BSP_LCD_Clear(LCD_COLOR_WHITE);
		//BSP_LCD_ClearStringLine(5);
		//BSP_LCD_ClearStringLine(6);
		//BSP_LCD_ClearStringLine(7);
		
		
		
	//	LCD_DisplayInt(7, 6, 123456789);
	//	LCD_DisplayFloat(8, 6, 12.3456789, 4);


	
	//configure real-time clock
	RTC_Config();
	RTC_AlarmAConfig();
	
	
//Init I2C for EEPROM		
	I2C_Init(&I2c3_Handle);
	//I2C_ByteWrite(&I2c3_Handle,EEPROM_ADDRESS, memLocation , 0);  //Uncommment and run to reset logged times
	inc = 1 + 4*I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation); // First location in EEPROM keeps track of how many entries we have Eg. EEPROM = [2,17,3,19,1,...] --> 2 time entries total, first time entry is 17:03:19 PM
																																		// (memLocation+inc) now points to location after last recorded value on EEPROM
	
	//Initialize time and date
	clearScreen();
	GetDate(&RTCHandle);
	GetTime(&RTCHandle);
	DisplayTime(time, 8);


//*********************Testing I2C EEPROM------------------
  //the following variables are for testging I2C_EEPROM

	uint8_t data1 =0x67,  data2=0x68;
	uint8_t readData=0x00;
	char AA[34]= "efghijklmnopqstuvefghijklmnopqstuv";
	uint8_t * bufferdata=(uint8_t *)AA;	
	int i;
	uint8_t readMatch=1;
	uint32_t EE_status;
/*
	EE_status=I2C_ByteWrite(&I2c3_Handle,EEPROM_ADDRESS, memLocation , data1);
	if (EE_status==HAL_OK)
			LCD_DisplayString(0, 0, (uint8_t *)"w data1 OK");
	else
			LCD_DisplayString(0, 0, (uint8_t *)"w data1 failed");
	EE_status=I2C_ByteWrite(&I2c3_Handle,EEPROM_ADDRESS, memLocation+1 , data2);
	if (EE_status==HAL_OK)
			LCD_DisplayString(1, 0, (uint8_t *)"w data2 OK");
	else
			LCD_DisplayString(1, 0, (uint8_t *)"w data2 failed");
	
	
	
	readData=I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation);
	if (data1 == readData) {
			LCD_DisplayString(2, 0, (uint8_t *)"r data1 success");
	}else{
			LCD_DisplayString(2, 0, (uint8_t *)"r data1 mismatch");
	}	
	LCD_DisplayInt(3, 14, readData);
	
	readData=I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation+1);
	if (data2 == readData) {
			LCD_DisplayString(4, 0, (uint8_t *)"r data2 success");
	}else{
			LCD_DisplayString(4, 0, (uint8_t *)"r data2 mismatch");
	}	
	LCD_DisplayInt(5, 14, readData);
	EE_status=I2C_BufferWrite(&I2c3_Handle,EEPROM_ADDRESS, memLocation, bufferdata, 34);
	if (EE_status==HAL_OK)
		LCD_DisplayString(6, 0, (uint8_t *)"w buffer OK");
	else
		LCD_DisplayString(6, 0, (uint8_t *)"W buffer failed");
	// Each letter is 2 bytes
	for (i=0;i<=33;i++) { 
			readData=I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation+i);
			HAL_Delay(5);   // just for display effect. for EEPROM read, do not need dalay		
		//BUT :  if here delay longer time, the floowing display will have trouble,???
	
			BSP_LCD_DisplayChar(COLUMN(i%16),LINE(8+ 2*(int)(i/16)), (char) readData);	
			BSP_LCD_DisplayChar(COLUMN(i%16),LINE(9+ 2*(int)(i/16)),  bufferdata[i]);
			if (bufferdata[i]!=readData)
					readMatch=0;
	}
	if (readMatch==0)
		LCD_DisplayString(15, 0, (uint8_t *)"r buffer mismatch");
	else 
		LCD_DisplayString(15, 0, (uint8_t *)"r buffer success");
*/

//******************************testing I2C EEPROM*****************************/


  /* Infinite loop */

while (1)
  {			
		
  } //end of while

}  // end of main

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  Configures EXTI Line0 (connected to PA0 pin) in interrupt mode
  * @param  None
  * @retval None
  */




/**
 * Use this function to configure the GPIO to handle input from
 * external pushbuttons and configure them so that you will handle
 * them through external interrupts.
 */
void ExtBtn1_Config(void)     // for GPIO C pin 1
// can only use PA0, PB0... to PA4, PB4 .... because only  only  EXTI0, ...EXTI4,on which the 
	//mentioned pins are mapped to, are connected INDIVIDUALLY to NVIC. the others are grouped! 
		//see stm32f4xx.h, there is EXTI0_IRQn...EXTI4_IRQn, EXTI15_10_IRQn defined
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_1;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void ExtBtn2_Config(void){  //**********PD2.***********
 
	GPIO_InitTypeDef   GPIO_InitStructure;
	
	/* Enable GPIOD clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* Configure PD2 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	
	
	
	
	
}


void RTC_Config(void) {
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	
	/****************
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RTC_AlarmTypeDef RTC_AlarmStructure;
	****************/
	
	//1: Enable the RTC domain access (enable wirte access to the RTC )
			//1.1: Enable the Power Controller (PWR) APB1 interface clock:
        __HAL_RCC_PWR_CLK_ENABLE(); 
			//1.2:  Enable access to RTC domain 
				HAL_PWR_EnableBkUpAccess();
			//1.3: Select the RTC clock source
				__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSI);  //RCC_RTCCLKSOURCE_LSI is defined in hal_rcc.h
	       // according to P9 of AN3371 Application Note, LSI's accuracy is not suitable for RTC application!!!! 
					//can not use LSE!!!---LSE is not available, at leaset not available for stm32f407 board.
				//****"Without parts at X3, C16, C27, and removing SB15 and SB16, the LSE is not going to tick or come ready"*****.
			//1.4: Enable RTC Clock
			__HAL_RCC_RTC_ENABLE();   //enable RTC
			
	
			//1.5  Enable LSI
			__HAL_RCC_LSI_ENABLE();   //need to enable the LSI !!!
																//defined in _rcc.c
			while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)==RESET) {} //defind in rcc.c
	
			// for the above steps, please see the CubeHal UM1725, p616, section "Backup Domain Access" 	
				
				
				
	//2.  Configure the RTC Prescaler (Asynchronous and Synchronous) and RTC hour 
        
				RTCHandle.Instance = RTC;
				RTCHandle.Init.HourFormat = RTC_HOURFORMAT12_PM;
				//RTC time base frequency =LSE/((AsynchPreDiv+1)*(SynchPreDiv+1))=1Hz
				//see the AN3371 Application Note: if LSE=32.768K, PreDiv_A=127, Prediv_S=255
				//    if LSI=32K, PreDiv_A=127, Prediv_S=249
				//also in the note: LSI accuracy is not suitable for calendar application!!!!!! 
				RTCHandle.Init.AsynchPrediv = 127; //if using LSE: Asyn=127, Asyn=255: 
				RTCHandle.Init.SynchPrediv = 249;  //if using LSI(32Khz): Asyn=127, Asyn=249: 
				// but the UM1725 says: to set the Asyn Prescaler a higher value can mimimize power comsumption
				
				RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
				RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
				RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
				
				//HAL_RTC_Init(); 
				if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
				{
						LCD_DisplayString(1, 0, (uint8_t *)"RTC Init Error!");
				}
	
	//3. init the time and date
				RTC_DateStructure.Year = 93;
				RTC_DateStructure.Month = RTC_MONTH_AUGUST;
				RTC_DateStructure.Date = 18; //if use RTC_FORMAT_BCD, NEED TO SET IT AS 0x18 for the 18th.
				RTC_DateStructure.WeekDay = RTC_WEEKDAY_FRIDAY; //???  if the real weekday is not correct for the given date, still set as 
																												//what is specified here.
				
				if(HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better 
															//before, must set in BCD format and read in BIN format!!
				{
					LCD_DisplayString(2, 0, (uint8_t *)"Date Init Error!");
				} 
  
  
				RTC_TimeStructure.Hours = 5;  
				RTC_TimeStructure.Minutes = 30; //if use RTC_FORMAT_BCD, NEED TO SET IT AS 0x19
				RTC_TimeStructure.Seconds = 13;
				RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_PM;
				RTC_TimeStructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				RTC_TimeStructure.StoreOperation = RTC_STOREOPERATION_RESET;//?????/
				
				if(HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better
																																					//before, must set in BCD format and read in BIN format!!
				{
					LCD_DisplayString(3, 0, (uint8_t *)"TIME Init Error!");
				}
  
			//Writes a data in a RTC Backup data Register0   --why need this line?
			//HAL_RTCEx_BKUPWrite(&RTCHandle,RTC_BKP_DR0,0x32F2);   

	
			//The RTC Resynchronization mode is write protected, use the
			//__HAL_RTC_WRITEPROTECTION_DISABLE() befor calling this function.
			__HAL_RTC_WRITEPROTECTION_DISABLE(&RTCHandle);
			//wait for RTC APB registers synchronisation
			HAL_RTC_WaitForSynchro(&RTCHandle);
			//__HAL_RTC_WRITEPROTECTION_ENABLE(&RTCHandle);				
	 
				
				
			__HAL_RTC_TAMPER1_DISABLE(&RTCHandle);
			__HAL_RTC_TAMPER2_DISABLE(&RTCHandle);	
				//Optionally, a tamper event can cause a timestamp to be recorded. ---P802 of RM0090
				//Timestamp on tamper event
				//With TAMPTS set to ?1 , any tamper event causes a timestamp to occur. In this case, either
				//the TSF bit or the TSOVF bit are set in RTC_ISR, in the same manner as if a normal
				//timestamp event occurs. The affected tamper flag register (TAMP1F, TAMP2F) is set at the
				//same time that TSF or TSOVF is set. ---P802, about Tamper detection
				//-------that is why need to disable this two tamper interrupts. Before disable these two, when program start, there is always a timestamp interrupt.
				//----also, these two disable function can not be put in the TSConfig().---put there will make  the program freezed when start. the possible reason is
				//-----one the RTC is configured, changing the control register again need to lock and unlock RTC and disable write protection.---See Alarm disable/Enable 
				//---function.
				
			HAL_RTC_WaitForSynchro(&RTCHandle);	
			//To read the calendar through the shadow registers after Calendar initialization,
			//		calendar update or after wake-up from low power modes the software must first clear
			//the RSF flag. The software must then wait until it is set again before reading the
			//calendar, which means that the calendar registers have been correctly copied into the
			//RTC_TR and RTC_DR shadow registers.The HAL_RTC_WaitForSynchro() function
			//implements the above software sequence (RSF clear and RSF check).	
}


void RTC_AlarmAConfig(void)
{
	RTC_AlarmTypeDef RTC_Alarm_Structure;

	RTC_Alarm_Structure.Alarm = RTC_ALARM_A;
  RTC_Alarm_Structure.AlarmMask = RTC_ALARMMASK_ALL;
				// See reference manual. especially 
				//p11-12 of AN3371 Application Note.
				// this mask mean alarm occurs every second.
				//if MaskAll, the other 3 fieds of the AlarmStructure do not need to be initiated
				//the other three fieds are: RTC_AlarmTime(for when to occur), 
				//RTC_AlarmDateWeekDaySel (to use RTC_AlarmDateWeekDaySel_Date or RTC_AlarmDateWeekDaySel_WeekDay
				//RTC_AlarmDateWeekDay (0-31, or RTC_Weekday_Monday, RTC_Weekday_Tuesday...., depends on the value of AlarmDateWeekDaySel)	
	//RTC_Alarm_Structure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
  //RTC_Alarm_Structure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  //RTC_Alarm_Structure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
		   //RTC_ALARMSUBSECONDMASK_ALL --> All Alarm SS fields are masked. 
        //There is no comparison on sub seconds for Alarm 
			
  //RTC_Alarm_Structure.AlarmTime.Hours = 0x02;
  //RTC_Alarm_Structure.AlarmTime.Minutes = 0x20;
  //RTC_Alarm_Structure.AlarmTime.Seconds = 0x30;
  //RTC_Alarm_Structure.AlarmTime.SubSeconds = 0x56;
  
  if(HAL_RTC_SetAlarm_IT(&RTCHandle,&RTC_Alarm_Structure,RTC_FORMAT_BCD) != HAL_OK)
  {
			LCD_DisplayString(4, 0, (uint8_t *)"Alarm setup Error!");
  } 
	
	RTC_AlarmA_IT_Enable(&RTCHandle);

	
	//Enable the RTC Alarm interrupt
	//__HAL_RTC_ALARM_ENABLE_IT(&RTCHandle,RTC_IT_ALRA);   //already in function HAL_RTC_SetAlarm_IT()
	
	//Enable the RTC ALARMA peripheral.
	//__HAL_RTC_ALARMA_ENABLE(&RTCHandle);  //already in function HAL_RTC_SetAlarm_IT()
	
	//__HAL_RTC_ALARM_CLEAR_FLAG(&RTCHandle, RTC_FLAG_ALRAF); //need it? !!!!, without it, sometimes(SOMETIMES, when first time to use the alarm interrupt)
																			//the interrupt handler will not work!!! 		
	

	
		//need to set/enable the NVIC for RTC_Alarm_IRQn!!!!
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);   
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0x00, 0);  //not important
	
				
	
}


HAL_StatusTypeDef  RTC_AlarmA_IT_Disable(RTC_HandleTypeDef *hrtc) 
{ 
 	// Process Locked  
	__HAL_LOCK(hrtc);
  
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_DISABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_DISABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  
}


HAL_StatusTypeDef  RTC_AlarmA_IT_Enable(RTC_HandleTypeDef *hrtc) 
{	
	// Process Locked  
	__HAL_LOCK(hrtc);	
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_ENABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  

}







void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		while (*ptr!=NULL)
    {
				BSP_LCD_DisplayChar(COLUMN(ColumnNumber),LINE(LineNumber), *ptr); //new version of this function need Xpos first. so COLUMN() is the first para.
				ColumnNumber++;
			 //to avoid wrapping on the same line and replacing chars 
				if ((ColumnNumber+1)*(((sFONT *)BSP_LCD_GetFont())->Width)>=BSP_LCD_GetXSize() ){
					ColumnNumber=0;
					LineNumber++;
				}
					
				ptr++;
		}
}

void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		sprintf(lcd_buffer,"%d",Number);
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		
		sprintf(lcd_buffer,"%.*f",DigitAfterDecimalPoint, Number);  //6 digits after decimal point, this is also the default setting for Keil uVision 4.74 environment.
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}



/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
  if(GPIO_Pin == KEY_BUTTON_PIN)  //GPIO_PIN_0
  {
		//Log current time
		uint8_t held = buttonHeld(GPIOA, KEY_BUTTON_PIN);
		if (!changing) {
			if(held){									//Display full date and time
				clearScreen();
				DisplayTime(time, 8);
				DisplayDate(date, 7);
				display_full = 1;
			}
			else{
				logTime();
			}
		}
  }
	
	
	if(GPIO_Pin == GPIO_PIN_1)
  {
		
		if (!display_old && !changing && !display_full) {
													//Display last two saved times
				clearScreen();
				Display_Past_Two();
				display_old = 1;
		} 
		else if (changing) {
			//If we enter this if statement, we are changing the time and date
			switch(count) {
				case 0: //Weekday
					++date[count];
					if(date[count] > 7) date[count] = 1;
					break;
				case 1: //Month
					++date[count];
					if(date[count] > 12) date[count] = 1;
					break;
				case 2: //Day
					++date[count];
					if(date[count] > 31) date[count] = 1;
					break;
				case 3: //Year
					++date[count];
					if(date[count] > 2038) date[count] = 1901;
					break;
				case 4: //Hour
					++time[count-4];
					if(time[count-4] > 12) time[count-4] = 1;
					break;
				case 5: //Minutes
					++time[count-4];
					if(time[count-4] > 60) time[count-4] = 0;
					break;
				case 6: //Seconds
					++time[count-4];
				  if(time[count-4] > 60) time[count-4] = 0;
					break;
				case 7: //AM/PM
					time[count-4] = (time[count-4] == 1) ? 0 : 1;
					break;
			}
			DisplayTime(time, 9);
			DisplayDate(date, 7);
			
		} 
		else {
			display_old = 0;
			display_full = 0;
			clearScreen();  //Clear screen from displaying the past 2 times, resumes periodic display of time through HAL_RTC_AlarmAEventCallback
			GetTime(&RTCHandle);
			DisplayTime(time, 8);
		}
		
			
	}  //end of PIN_1

	if(GPIO_Pin == GPIO_PIN_2)
  {
		if (!display_old && !changing && !display_full) {		// Enter time/date edit mode
			changing = 1;
			clearScreen();
			GetDate(&RTCHandle);
			GetTime(&RTCHandle);
			DisplayTime(time, 9);
			DisplayDate(date, 7);
		} else if (changing) {
			++count;													// Move to next variable to edit
			if (count > 7) {									// Save all changes and exit edit mode
				SetDateAndTime();
				count = 0;
				changing = 0;
				clearScreen();
				display_full = 0;
				display_old = 0;
			}
		}
			
	} //end of if PIN_2	
	
	
}

// Alarm A goes off every second
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
 
	//RTC_TimeShow();
	
	//Displaying the time
	if (!display_old && !changing) {
		GetTime(hrtc);
		DisplayTime(time, 8);
	}
	
}


//Helper Functions --------------------------------------------------------------------------------------------------

// Inputs time into time[4] variabl
void GetTime(RTC_HandleTypeDef *hrtc) {
	HAL_RTC_GetTime(hrtc, &RTC_TimeStructure,  RTC_FORMAT_BIN);
	time[0] = RTC_TimeStructure.Hours;   //hour
	time[1] = RTC_TimeStructure.Minutes; //minutes
	time[2] = RTC_TimeStructure.Seconds; //seconds
	if (RTC_TimeStructure.TimeFormat == RTC_HOURFORMAT12_AM) {
		time[3] = 0; //AM
	} else if (RTC_TimeStructure.TimeFormat == RTC_HOURFORMAT12_PM) {
		time[3] = 1; //PM
	}
}


// Inputs date into date[4] variabl
void GetDate(RTC_HandleTypeDef *hrtc) {
	HAL_RTC_GetDate(hrtc, &RTC_DateStructure,  RTC_FORMAT_BIN);
	
	//Weekday
	switch(RTC_DateStructure.WeekDay) {
		case RTC_WEEKDAY_MONDAY:
			date[0] = 1;
			break;
		case RTC_WEEKDAY_TUESDAY:
			date[0] = 2;
			break;
	  case RTC_WEEKDAY_WEDNESDAY:
			date[0] = 3;
			break;
	  case RTC_WEEKDAY_THURSDAY:
			date[0] = 4;
			break;
		case RTC_WEEKDAY_FRIDAY:
			date[0] = 5;
			break;
		case RTC_WEEKDAY_SATURDAY:
			date[0] = 6;
			break;
		case RTC_WEEKDAY_SUNDAY:
			date[0] = 7;
			break;
	}
	
	//Month 
	switch(RTC_DateStructure.Month) {
		case RTC_MONTH_JANUARY:
			date[1] = 1;
			break;
		case RTC_MONTH_FEBRUARY:
			date[1] = 2;
			break;
	  case RTC_MONTH_MARCH:
			date[1] = 3;
			break;
	  case RTC_MONTH_APRIL:
			date[1] = 4;
			break;
		case RTC_MONTH_MAY:
			date[1] = 5;
			break;
		case RTC_MONTH_JUNE:
			date[1] = 6;
			break;
		case RTC_MONTH_JULY:
			date[1] = 7;
			break;
		case RTC_MONTH_AUGUST:
			date[1] = 8;
			break;
	  case RTC_MONTH_SEPTEMBER:
			date[1] = 9;
			break;
		case RTC_MONTH_OCTOBER:
			date[1] =10;
			break;
		case RTC_MONTH_NOVEMBER:
			date[1] = 11;
			break;
		default:
			date[1] = 12;
			break;
	}
	
	//Day
	date[2] = RTC_DateStructure.Date;
	
	//Year
	date[3] = RTC_DateStructure.Year + 1901;
}


//Displays time held in t[4] variable
void DisplayTime(int *t, int LCDrow) {
	
	//hours
	if (t[0] < 10) {
		LCD_DisplayInt(LCDrow, 3, 0);
		LCD_DisplayInt(LCDrow, 4, t[0]);
	} else {
		LCD_DisplayInt(LCDrow, 3, t[0]);
	}
	LCD_DisplayString(LCDrow, 5, (uint8_t *) ":");
	
	//minutes
	if (t[1] < 10) {
		LCD_DisplayInt(LCDrow, 6, 0);
		LCD_DisplayInt(LCDrow, 7, t[1]);
	} else {
		LCD_DisplayInt(LCDrow, 6, t[1]);
	}
	LCD_DisplayString(LCDrow, 8, (uint8_t *) ":");
	
	//seconds
	if (t[2] < 10) {
		LCD_DisplayInt(LCDrow, 9, 0);
		LCD_DisplayInt(LCDrow, 10, t[2]);
	} else {
		LCD_DisplayInt(LCDrow, 9, t[2]);
	}
	
	if (t[3] == 0) {
		LCD_DisplayString(LCDrow, 12, (uint8_t *)"AM");
	} else {
		LCD_DisplayString(LCDrow, 12, (uint8_t *)"PM");
	}
	
}


//Display date held in d[4] variable
void DisplayDate(int *d, int LCDrow) {
	
	BSP_LCD_ClearStringLine(LCDrow-2);
	BSP_LCD_ClearStringLine(LCDrow);
	
	//Weekday
	switch(d[0]) {
		case 1:
			LCD_DisplayString(LCDrow-2, 4, (uint8_t *)"Monday");
			break;
		case 2:
			LCD_DisplayString(LCDrow-2, 4, (uint8_t *)"Tuesday");
			break;
		case 3:
			LCD_DisplayString(LCDrow-2, 4, (uint8_t *)"Wednesday");
			break;
		case 4:
			LCD_DisplayString(LCDrow-2, 4, (uint8_t *)"Thursday");
			break;
		case 5:
			LCD_DisplayString(LCDrow-2, 4, (uint8_t *)"Friday");
			break;
		case 6:
			LCD_DisplayString(LCDrow-2, 4, (uint8_t *)"Saturday");
			break;
		case 7:
			LCD_DisplayString(LCDrow-2, 4, (uint8_t *)"Sunday");
			break;
	}
	
	//Month
	switch(d[1]) {
		case 1:
			LCD_DisplayString(LCDrow, 2, (uint8_t *)"Jan");
			break;
		case 2:
			LCD_DisplayString(LCDrow, 2, (uint8_t *)"Feb");
			break;
	  case 3:
			LCD_DisplayString(LCDrow, 2, (uint8_t *)"Mar");
			break;
	  case 4:
			LCD_DisplayString(LCDrow, 2, (uint8_t *)"Apr");
			break;
		case 5:
			LCD_DisplayString(LCDrow, 2, (uint8_t *)"May");
			break;
		case 6:
			LCD_DisplayString(LCDrow, 2, (uint8_t *)"Jun");
			break;
		case 7:
			LCD_DisplayString(LCDrow, 2, (uint8_t *)"Jul");
			break;
		case 8:
			LCD_DisplayString(LCDrow, 2, (uint8_t *)"Aug");
			break;
	  case 9:
			LCD_DisplayString(LCDrow, 2, (uint8_t *)"Sep");
			break;
		case 10:
			LCD_DisplayString(LCDrow, 2, (uint8_t *)"Oct");
			break;
		case 11:
			LCD_DisplayString(LCDrow, 2, (uint8_t *)"Nov");
			break;
		case 12:
			LCD_DisplayString(LCDrow, 2, (uint8_t *)"Dec");
			break;
	}
	
	//Day
	if (d[2] < 10) {
		LCD_DisplayString(LCDrow, 6, (uint8_t *)" ");
		LCD_DisplayInt(LCDrow, 7, d[2]);
	} else {
		LCD_DisplayInt(LCDrow, 6, d[2]);
	}
	LCD_DisplayString(LCDrow, 8, (uint8_t *)",");
	
	//Year
	LCD_DisplayInt(LCDrow, 10, d[3]);
	
}


// Log current RTC time in EEPROM
void logTime() {
	GetTime(&RTCHandle);
	int timesLogged = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation);
	I2C_ByteWrite(&I2c3_Handle,EEPROM_ADDRESS, memLocation, timesLogged + 1);
		
	I2C_ByteWrite(&I2c3_Handle,EEPROM_ADDRESS, memLocation + inc, time[0]); //hours
	++inc;
	I2C_ByteWrite(&I2c3_Handle,EEPROM_ADDRESS, memLocation + inc, time[1]); //minutes
	++inc;
	I2C_ByteWrite(&I2c3_Handle,EEPROM_ADDRESS, memLocation + inc, time[2]); //seconds
	++inc;
	if (time[3] == 0) {
		I2C_ByteWrite(&I2c3_Handle,EEPROM_ADDRESS, memLocation + inc, time[3]); //AM
	} else {
		I2C_ByteWrite(&I2c3_Handle,EEPROM_ADDRESS, memLocation + inc, time[3]); //PM
	}
	++inc;
}


//Display past two saved times
void Display_Past_Two() {
	BSP_LCD_ClearStringLine(8);  //Clear screen
	int timesLogged = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation);
	
	int currInc = 1;
	if (timesLogged == 0) {
		LCD_DisplayString(6, 2, (uint8_t *)"No saved times");
	} else if (timesLogged == 1) {
		
		time1[0] = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation + currInc); //hours
		++currInc;
		time1[1] = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation + currInc); //minutes
		++currInc;
		time1[2] = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation + currInc); //seconds
		++currInc;
		time1[3] = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation + currInc); //seconds
		++currInc;
		
		DisplayTime(time1, 6);
		
	} else {
		
		currInc = inc - 8;
		time1[0] = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation + currInc); //hours
		++currInc;
		time1[1] = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation + currInc); //minutes
		++currInc;
		time1[2] = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation + currInc); //seconds
		++currInc;
		time1[3] = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation + currInc); //seconds
		++currInc;
		
		time2[0] = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation + currInc); //hours
		++currInc;
		time2[1] = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation + currInc); //minutes
		++currInc;
		time2[2] = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation + currInc); //seconds
		++currInc;
		time2[3] = I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation + currInc); //seconds
		++currInc;
		
		DisplayTime(time1, 6);
		DisplayTime(time2, 8);
		
	}
}

// Set the date and time of RTC
void SetDateAndTime() {
	
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	
  __HAL_RCC_PWR_CLK_ENABLE(); 
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSI);
	__HAL_RCC_RTC_ENABLE();   
	__HAL_RCC_LSI_ENABLE();   
	while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)==RESET) {} 	

  RTCHandle.Instance = RTC;
	RTCHandle.Init.HourFormat = RTC_HOURFORMAT12_PM;
	RTCHandle.Init.AsynchPrediv = 127; 
	RTCHandle.Init.SynchPrediv = 249;  
	RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
	RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
				
	if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
	{
			LCD_DisplayString(1, 0, (uint8_t *)"RTC Init Error!");
	}
	
	
	// << Date >>
	
	//Weekday
	switch(date[0]) {
		case 1:
			RTC_DateStructure.WeekDay = RTC_WEEKDAY_MONDAY;
			break;
		case 2:
			RTC_DateStructure.WeekDay = RTC_WEEKDAY_TUESDAY;
			break;
		case 3:
			RTC_DateStructure.WeekDay = RTC_WEEKDAY_WEDNESDAY;
			break;
		case 4:
			RTC_DateStructure.WeekDay = RTC_WEEKDAY_THURSDAY;
			break;
		case 5:
			RTC_DateStructure.WeekDay = RTC_WEEKDAY_FRIDAY;
			break;
		case 6:
			RTC_DateStructure.WeekDay = RTC_WEEKDAY_SATURDAY;
			break;
		case 7:
			RTC_DateStructure.WeekDay = RTC_WEEKDAY_SUNDAY;
			break;
	}
	
	//Month
	switch(date[1]) {
		case 1:
			RTC_DateStructure.Month = RTC_MONTH_JANUARY;
			break;
		case 2:
			RTC_DateStructure.Month = RTC_MONTH_FEBRUARY;
			break;
		case 3:
			RTC_DateStructure.Month = RTC_MONTH_MARCH;
			break;
		case 4:
			RTC_DateStructure.Month = RTC_MONTH_APRIL;
			break;
		case 5:
			RTC_DateStructure.Month = RTC_MONTH_MAY;
			break;
		case 6:
			RTC_DateStructure.Month = RTC_MONTH_JUNE;
			break;
		case 7:
			RTC_DateStructure.Month = RTC_MONTH_JULY;
			break;
		case 8:
			RTC_DateStructure.Month = RTC_MONTH_AUGUST;
			break;
		case 9:
			RTC_DateStructure.Month = RTC_MONTH_SEPTEMBER;
			break;
		case 10:
			RTC_DateStructure.Month = RTC_MONTH_OCTOBER;
			break;
		case 11:
			RTC_DateStructure.Month = RTC_MONTH_NOVEMBER;
			break;
		case 12:
			RTC_DateStructure.Month = RTC_MONTH_DECEMBER;
			break;
	}
	
	//Day
	RTC_DateStructure.Date = date[2]; 
	
	//Year
	RTC_DateStructure.Year = date[3] - 1901;
	
	
	if(HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN) != HAL_OK)   
	{
		LCD_DisplayString(2, 0, (uint8_t *)"Date Init Error!");
	} 												
  
	// << Time >>
	
	RTC_TimeStructure.Hours = time[0] + 1;  //Hour
	RTC_TimeStructure.Minutes = time[1];    //Minutes
	RTC_TimeStructure.Seconds = time[2];    //Seconds
	RTC_TimeStructure.TimeFormat = (time[3] == 1) ? RTC_HOURFORMAT12_PM : RTC_HOURFORMAT12_AM;  //AM/PM
				
	if(HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN) != HAL_OK) 
	{
		LCD_DisplayString(3, 0, (uint8_t *)"TIME Init Error!");
	}
  
}

// Determines if button on specified pin was held for more than 0.3 seconds
uint8_t buttonHeld(GPIO_TypeDef *port, uint16_t pin){
	uint32_t start_time = HAL_GetTick();
	uint32_t hold_time = 0;
	uint32_t poll_time = 0;
	uint8_t status;
	while(hold_time < 300){
			status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
			hold_time += status*(HAL_GetTick() - start_time);
			poll_time += (1-status)*(HAL_GetTick() - start_time);
			if(poll_time > 100){
				return 0;
			}
			start_time = HAL_GetTick();
	}
	LCD_DisplayInt(12,0,poll_time);
	LCD_DisplayInt(13,0,hold_time);
	return 1;
}


//Clear the screen
void clearScreen() {
	for (int i = 4; i <= 14; ++i) {
		BSP_LCD_ClearStringLine(i);
	}
}

//Helper Functions --------------------------------------------------------------------------------------------------


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}





#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/