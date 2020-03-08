
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdlib.h>
#include "sd_hal_mpu6050.h"
#include <math.h>
/* USER CODE BEGIN Includes */
#include "MY_NRF24.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern uint8_t Rx_str[200];
extern uint8_t Rx_index;
int duty1=450,duty2=450,duty3=450,duty4=450,duty;
int min1=400,min2=400,min3=400,min4=400; // muc thap nhat cho dong co chay la 48 - 61 52 48    B6 B7 B8 B9
int max1=800,max2=800,max3=800,max4=800;
char Tx_str[200]="12;";
uint8_t Tx_index=3;
uint8_t TX_BUF[5] = {'a','b','c',';','d'};
extern uint8_t rx_flag;
char *Std;
int loop=0;
// for mpu6050
SD_MPU6050 mpu1;
  float g_x=0,g_y=0,g_z=0,a_x=0,a_y=0,a_z=0;
	float g_ax=0,g_ay=0,g_az=0,a_ax=0,a_ay=0,a_az=0;
	float xx=0,yy=0,zz=0,axx=0,ayy=0,azz=0;
	float roll=0,pitch=0,yaw=0 ;
	float A=0.999,B=0.96,dt=0, old_time=0;

// Variables for PID
float pid_p_roll=1.0;
float pid_i_roll=0.0;
float pid_d_roll=0.0;

float pid_p_pitch=1.0;
float pid_i_pitch=0.0;
float pid_d_pitch=0.0;

float pid_p_yaw=4.3;
float pid_i_yaw=0.02;
float pid_d_yaw=0.00;

float set_point_roll=0;
float set_point_pitch=0;
float set_point_yaw=0;

float error_roll=0;
float error_pitch=0;
float error_yaw=0;

float last_error_roll=0;
float last_error_pitch=0;
float last_error_yaw=0;

float inter_roll=0;
float inter_pitch=0;
float inter_yaw=0;

float output_roll=0;
float output_pitch=0;
float output_yaw=0;

float input_roll=0;
float input_pitch=0;
float input_yaw=0;

int chanel_send=1;
int throut=48;
int start=0;
float er=0;
//47 60 47 47 tang 1 muc nua la chay
//b6 b7  b8   b9

//d48 61 52 48; muc toi thieu de chay


//duty1= 60 duty2 = 73 duty3= 64 duty4 = 60
//duty1= 60 duty2 = 73 duty3= 64 duty4 = 60
//duty1= 61 duty2 = 74 duty3= 65 duty4 = 61
//duty1= 61 duty2 = 74 duty3= 65 duty4 = 61
//duty1= 61 duty2 = 74 duty3= 65 duty4 = 61
//duty1= 61 duty2 = 74 duty3= 65 duty4 = 61
//duty1= 62 duty2 = 75 duty3= 66 duty4 = 62
//duty1= 61 duty2 = 74 duty3= 65 duty4 = 61
//duty1= 60 duty2 = 73 duty3= 64 duty4 = 60

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void NRF_Sendstring()
	{
		HAL_Delay(1);
		Tx_index=strlen(Tx_str);
			//	LED_TGL;
		for(int i=0;i<Tx_index;i+=5)
		{
			for(int j=0;j<5;j++)
			{
				if((i+j)<Tx_index)
				TX_BUF[j]=Tx_str[i+j];
				else
					TX_BUF[j]=' ';
			}		
			NRF24L01_Send(TX_BUF);
		HAL_Delay(10);
		}
	}

void SetPmw()
{	
	if(duty1<min1)
		duty1=min1;
	if(duty2<min2)
		duty2=min2;
	if(duty3<min3)
		duty3=min3;
	if(duty4<min4)
		duty4=min4;
	
	if(duty1>max1)
		duty1=max1;
	if(duty2>max2)
		duty2=max2;
	if(duty3>max3)
		duty3=max3;
	if(duty4>max4)
		duty4=max4;
}
void calculate_pid()
{
	// Roll
	
	error_roll=round(input_roll)-set_point_roll;
	er=last_error_roll;
	inter_roll+=pid_i_roll*error_roll*dt;
	if(inter_roll>900)
		inter_roll=900;
	if(inter_roll<-900)
		inter_roll=-900;
	if(dt!=0.0)
	{
		output_roll=pid_p_roll*error_roll+inter_roll+pid_d_roll*(error_roll-last_error_roll)/dt;
		last_error_roll=error_roll;
	}
	//Pitch
	error_pitch=round(input_pitch)-set_point_pitch;
	inter_pitch+=error_pitch*dt;
	if(inter_pitch>900)
		inter_pitch=900;
	if(inter_pitch<-900)
		inter_pitch=-900;
	if(dt!=0.0)
	{
		output_pitch=pid_p_pitch*error_roll+pid_i_pitch*inter_pitch+pid_d_pitch*(error_pitch-last_error_pitch)/dt;
		last_error_pitch=error_pitch;
	}
	// Yaw
//	error_roll=roll-set_point_roll;
//	inter_roll+=error_roll*dt;
//	if(inter_roll>400)
//		inter_roll=400;
//	if(inter_roll<-400)
//		inter_roll=-400;
//	if(dt!=0.0)
//	{
//		output_roll=pid_p_roll*error_roll+inter_roll+(error_roll-last_error_roll)/dt;
//		last_error_roll=error_roll;
//	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	SD_MPU6050_Result result ;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	NRF24_ini();
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,duty1);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,duty2);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,duty3);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,duty4);
	LED_OFF;
	
	LED_OFF;
	sprintf(Tx_str,"Set NRF ok;");
	NRF_Sendstring();
	result = SD_MPU6050_Init(&hi2c2,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_8G,SD_MPU6050_Gyroscope_500s );
	  HAL_Delay(500);

	  if(result == SD_MPU6050_Result_Ok)
	  {
			sprintf(Tx_str,"MPU WORK FINE;");
			NRF_Sendstring();
		}
	  else
	  {
			sprintf(Tx_str,"MPU DON'T WORK;");
			NRF_Sendstring();
	  }
		
		old_time=HAL_GetTick();
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) // duty 2 la kv 1000
  {

	
		loop++;
		if(loop>=10)
		{
			switch(chanel_send)
				{
				case 1:
			sprintf(Tx_str,"d1: %d: d2 :%d: d3: %d: d4 : %d;",duty1, duty2, duty3, duty4);
					break;
				case 2:
			sprintf(Tx_str,"P: %f: R: %f :Y: %f: dt: %f;",pitch, roll, yaw,dt);
					break;
				case 3:
			sprintf(Tx_str,"in pitch: %.2f:in roll: %.2f ;",input_pitch, input_roll);
				break;
				case 4:
			sprintf(Tx_str,"out pitch: %.2f:out roll: %.2f :error: %.2f:er: %.2f;",output_pitch, output_roll,error_roll,er);
				break;
				}
			NRF_Sendstring();
		}
		
		if(rx_flag==1) // 
		{	
			HAL_Delay(10);
			if(Rx_str[0]=='d') // set duty
			{
				Rx_str[0]=' ';
				Std=(char*)Rx_str;
			duty =(int) strtod(Std,&Std);
				if(duty>min1 && duty<max1)
				{
					duty1=duty;
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,duty1);
					duty =(int) strtod(Std,&Std);
				}
				if(duty>min2 && duty<max2)
				{
					duty2=duty;
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,duty2);
					duty =(int) strtod(Std,&Std);
				}
				if(duty>min3 && duty<max3)
				{
					duty3=duty;
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,duty3);
					duty =(int) strtod(Std,&Std);
				}
				if(duty>min4 && duty<max4)
				{
					duty4=duty;
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,duty4);
				}
				sprintf(Tx_str,"Apply new duty;");
				NRF_Sendstring();
			}
			
			if(Rx_str[0]=='o') // set min
			{
				Rx_str[0]=' ';
				Std=(char*)Rx_str;
				min1 =(int) strtod(Std,&Std);
				min2 =(int) strtod(Std,&Std);
				min3 =(int) strtod(Std,&Std);
				min4 =(int) strtod(Std,&Std);
				sprintf(Tx_str,"min1= %d min2 = %d min3= %d min4 = %d;",min1, min2, min3, min4);
				NRF_Sendstring();
				
			}
			if(Rx_str[0]=='p')  // set pid for pitch
			{
				Rx_str[0]=' ';
				Std=(char*)Rx_str;
				pid_p_pitch = strtod(Std,&Std);
				pid_i_pitch = strtod(Std,&Std);
				pid_d_pitch = strtod(Std,&Std);
				sprintf(Tx_str,"PID= %f PID = %f PID= %f;",pid_p_pitch, pid_i_pitch, pid_d_pitch);
				NRF_Sendstring();
			}
			if(Rx_str[0]=='r')  // set pid for roll
			{
				Rx_str[0]=' ';
				Std=(char*)Rx_str;
				pid_p_roll = strtod(Std,&Std);
				pid_i_roll = strtod(Std,&Std);
				pid_d_roll = strtod(Std,&Std);
				sprintf(Tx_str,"PID= %f PID = %f PID= %f;",pid_p_roll, pid_i_roll, pid_d_roll);
				NRF_Sendstring();
			}
			if(Rx_str[0]=='b') // begin
			{
				duty1=min1-1;
				duty2=min2-1;
				duty3=min3-1;
				duty4=min4-1;
				SetPmw();
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,duty1);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,duty2);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,duty3);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,duty4);
			}
			if(Rx_str[0]=='e') // begin
			{
				if(start==0)
					{
						sprintf(Tx_str,"Enable Calibration;");
						NRF_Sendstring();
						start=1;
						}
					else
						{
							sprintf(Tx_str,"Disable Calibration;");
						NRF_Sendstring();
						start=0;
							}
			}
			if(Rx_str[0]=='s') //set point
			{
				Rx_str[0]=' ';
				Std=(char*)Rx_str;
				set_point_pitch = strtod(Std,&Std);
				set_point_roll =strtod(Std,&Std);
				set_point_yaw =strtod(Std,&Std);
				inter_roll=0;
				inter_pitch=0;
				sprintf(Tx_str,"set_pitch= %f set_roll = %f set_yaw= %f;",set_point_pitch, set_point_roll, set_point_yaw);
				NRF_Sendstring();
			}
			if(Rx_str[0]=='v') //set point
			{
				set_point_pitch = round(input_pitch);
				set_point_roll =round(input_roll);
				set_point_yaw =round(input_yaw);
				inter_roll=0;
				inter_pitch=0;
				sprintf(Tx_str,"set_pitch= %f set_roll = %f set_yaw= %f;",set_point_pitch, set_point_roll, set_point_yaw);
				NRF_Sendstring();
			}
			if(Rx_str[0]=='c')// change chanel send
			{
				if(chanel_send==4)
					chanel_send=1;
				else
					chanel_send++;
			}
			if(Rx_str[0]=='t')// change chanel send
			{
				Rx_str[0]=' ';
				Std=(char*)Rx_str;
				throut = (int)strtod(Std,&Std);
				sprintf(Tx_str,"Apply new throut= %d;",throut);
				NRF_Sendstring();
			}
			if(Rx_str[0]=='m')// change chanel send
			{
				Rx_str[0]=' ';
				Std=(char*)Rx_str;
				max1 =(int) strtod(Std,&Std);
				max2 =(int) strtod(Std,&Std);
				max3 =(int) strtod(Std,&Std);
				max4 =(int) strtod(Std,&Std);
				sprintf(Tx_str,"max1= %d max2 = %d max3= %d max4 = %d;",max1, max2, max3, max4);
				NRF_Sendstring();
			}
			if(Rx_str[0]=='h') //set point
			{
				if(duty1<max1)
				duty1++;
				if(duty2<max2)
				duty2++;
				if(duty3<max3)
				duty3++;
				if(duty4<max4)
				duty4++;
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,duty1);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,duty2);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,duty3);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,duty4);
			}
			if(Rx_str[0]=='l') //set point
			{
				if(duty1>min1)
				duty1--;
				if(duty2>min2)
				duty2--;
				if(duty3>min3)
				duty3--;
				if(duty4>min4)
				duty4--;
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,duty1);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,duty2);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,duty3);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,duty4);
			}
			for(int i=0;i<Rx_index;i++)
					Rx_str[i]=0;
				Rx_index=0;
			rx_flag=0;
		}
		
		
		SD_MPU6050_ReadAll(&hi2c2,&mpu1);
			a_x = ((float)mpu1.Accelerometer_X-axx)*mpu1.Acce_Mult;
			a_y = ((float)mpu1.Accelerometer_Y-ayy)*mpu1.Acce_Mult;
			a_z = ((float)mpu1.Accelerometer_Z-azz)*mpu1.Acce_Mult;
			g_x = ((float)mpu1.Gyroscope_X-xx)*mpu1.Gyro_Mult;
			g_y = ((float)mpu1.Gyroscope_Y-yy)*mpu1.Gyro_Mult; 
			g_z = ((float)mpu1.Gyroscope_Z-zz)*mpu1.Gyro_Mult;
			
			roll=atan2(a_x,sqrt(a_y*a_y+a_z*a_z))*180/3.1412;
			pitch=atan2(a_y,sqrt(a_x*a_x+a_z*a_z))*180/3.1412;
			yaw=atan2(sqrt(a_y*a_y+a_x*a_x),a_z)*180/3.1412;
		
			dt=(HAL_GetTick()-old_time)/1000.0;
			old_time=HAL_GetTick();
			roll=A*(roll+g_x*dt)+(1-A)*roll;
			pitch=A*(pitch+g_y*dt)+(1-A)*pitch;
			yaw=A*(yaw+g_z*dt)+(1-A)*yaw;
		input_roll=0.7*input_roll+0.3*roll;
	input_pitch=0.7*input_pitch+0.3*pitch;
		if(start==1)
			{
		calculate_pid(); //calculate pid controler
			
		duty1=throut-(int)output_roll;
		duty3=throut+(int)output_roll;
//		duty2=throut+output_roll;
//		duty4=throut-output_roll;
		SetPmw();
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,duty1);
//				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,duty2);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,duty3);
//				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,duty4);
				}
	//	LED_TGL;
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 128;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin== GPIO_PIN_3)
  {
    IRQ_Callback();
  }
  else
  {
    __NOP();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
