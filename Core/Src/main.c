/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "constant.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
	volatile uint16_t adc[2] = {0,}; // � ��� ��� ������ ������� ������ �� ���� ���������

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum Work_DC_DC {OFF_DC, LOW_GND, CALIBRATE_ADC_DC, VOLTAG_DC, CURRENT_DC, POWER_DC};	//���� ������� ���������������� DC/DC
enum Work_DC_DC type_work_dc_dc = CALIBRATE_ADC_DC;		//����� ������ DC/DC ��� ��������� ������� - ���������� ���
enum Work_DC_DC return_type_work_dc_dc = LOW_GND;		//����� ������ DC/DC ����� ���������� ��� ��� ����, ���� ���������� �������� �� ��� ������
uint32_t duty_cycle = 0;	//�������� �� ����������� PWM ������ ���������� ������ ������, ������� ������. ���� ���������� ���


void delay(uint32_t n)  {

  while(n>0) n--;
}
//**************
//������ ������� ������� ���
uint16_t pwm_in_count_control = 0;				
uint16_t pwm_in_flag_max_power = 0;		//���� ������ ������� ��������� ������������ ��������
uint16_t cap_pwm_fall = 0;
uint16_t cap_pwm_rise = 0;
uint16_t cap_pwm_in_old_rise = 0;
uint16_t cap_pwm_in_long_hight = 0;
uint16_t cap_pwm_in_long_period = 0;
uint32_t cap_power_in_scale = 0;	//������������������ �������� ������� ���������� �� ��������
uint32_t cap_v_in_scale = 0;	//������������������ �������� ������� ���������� �� ����������
uint32_t cap_i_in_scale = 0;	//������������������ �������� ������� ���������� �� ����

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) // ������ �� �������
{
	if(htim->Instance == TIM17)	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)	{// RISE/FALL
			//������ ������ �� �����, ��� ����������� ���������� ���������� ������. ��������� ����� ����� �� ��������� ����������!!! - GPB9?
			if (GPIOB->IDR & 0x0200)	{
				cap_pwm_rise = HAL_TIM_ReadCapturedValue(&htim17, TIM_CHANNEL_1); // ������ �������� � �������� �������/���������;
				cap_pwm_in_long_period = cap_pwm_rise - cap_pwm_in_old_rise;
				cap_pwm_in_old_rise = cap_pwm_rise;
			}
			else	{
				cap_pwm_fall = HAL_TIM_ReadCapturedValue(&htim17, TIM_CHANNEL_1); // ������ �������� � �������� �������/���������;;
				cap_pwm_in_long_hight = cap_pwm_fall - cap_pwm_rise;
			}
			//��������, ��� DC/DC �� ��������� � ������ ���������� ���
			if (type_work_dc_dc != CALIBRATE_ADC_DC)	{
				//�������� �� ������� �������� ������� ��� ��� ���������� �� �������� (���������� ������� ���������� ��� � ����� �� ������������ ����������, ��� ������������ �������� � �������)
				if ((cap_pwm_in_long_period >= PWM_IN_CONTROL_PERIOD_POWER_MIN) && (cap_pwm_in_long_period <= PWM_IN_CONTROL_PERIOD_POWER_MAX))	{
					//����� �������� �������� ����������� ��� ����������
					pwm_in_count_control = 0;
					//�������� �� ������� �������� ��� �� ����������
					if ((cap_pwm_in_long_hight >= PWM_IN_MIN) && (cap_pwm_in_long_hight <= PWM_IN_MAX))	{
						cap_power_in_scale = (cap_pwm_in_long_hight - PWM_IN_MIN) * PWM_TO_POWER_SCALE;
						type_work_dc_dc = POWER_DC;	//�������� ����� ������������ ��������
					}
					else	{
						cap_power_in_scale = 0; //��������� ������������ ��������
						type_work_dc_dc = POWER_DC;	//�������� ����� ������������ ��������
					}
					//�������� �� ��������� ������ ���������� ���
					if ((cap_pwm_in_long_hight >= PWM_IN_CALIBRATION_MIN) && (cap_pwm_in_long_hight <= PWM_IN_CALIBRATION_MAX))	{
						return_type_work_dc_dc = POWER_DC;	//��������� ��� �������� �� ������ ���������� ����� ������������ ��������
						type_work_dc_dc = CALIBRATE_ADC_DC;	//�������� ����� ��������� ���
					}
					//�������� �� ��������� ������ ������������ ��������
					if ((cap_pwm_in_long_hight >= PWM_IN_MAX_POWER_MIN) && (cap_pwm_in_long_hight <= PWM_IN_MAX_POWER_MAX))		{
						cap_v_in_scale = MAX_V_PILTIE;	//������� ������������ �������� �� ���������� ��� �������
						type_work_dc_dc = VOLTAG_DC;	//�������� ����� ������������ ����������
					}
				}
					//�������� �� ������� �������� ��� ��� ���������� ������������� ����������
				else if ((cap_pwm_in_long_period >= PWM_IN_CONTROL_PERIOD_VOLTAG_MIN) && (cap_pwm_in_long_period <= PWM_IN_CONTROL_PERIOD_VOLTAG_MAX))	{
					//����� �������� �������� ����������� ��� ���������� �����������
					pwm_in_count_control = 0;
					//�������� �� ������� �������� ��� �� ���������� �����������
					if ((cap_pwm_in_long_hight >= PWM_IN_MIN) && (cap_pwm_in_long_hight <= PWM_IN_MAX))	{
						cap_v_in_scale = (cap_pwm_in_long_hight - PWM_IN_MIN) * PWM_TO_VOLTAG_SCALE;
						type_work_dc_dc = VOLTAG_DC;	//�������� ����� ������������ ����������
					}
					else	{
						cap_v_in_scale = 0; //��������� ������������ ��������
						type_work_dc_dc = VOLTAG_DC;	//�������� ����� ������������ ����������
					}
					//�������� �� ��������� ������ ���������� ���
					if ((cap_pwm_in_long_hight >= PWM_IN_CALIBRATION_MIN) && (cap_pwm_in_long_hight <= PWM_IN_CALIBRATION_MAX))	{
						return_type_work_dc_dc = VOLTAG_DC;	//��������� ��� �������� �� ������ ���������� ����� ������������ ���������
						type_work_dc_dc = CALIBRATE_ADC_DC;	//�������� ����� ��������� ���
					}
				}
				//�������� �� ������� �������� ��� ��� ���������� ������������� ����
				else if ((cap_pwm_in_long_period >= PWM_IN_CONTROL_PERIOD_CURRENT_MIN) && (cap_pwm_in_long_period <= PWM_IN_CONTROL_PERIOD_CURRENT_MAX))	{
					//����� �������� �������� ����������� ��� ���������� �����
					pwm_in_count_control = 0;
					//�������� �� ������� �������� ��� �� ���������� �����
					if ((cap_pwm_in_long_hight >= PWM_IN_MIN) && (cap_pwm_in_long_hight <= PWM_IN_MAX))	{
						cap_i_in_scale = (cap_pwm_in_long_hight - PWM_IN_MIN) * PWM_TO_CURRENT_SCALE;
						type_work_dc_dc = CURRENT_DC;	//�������� ����� ������������ ����
					}
					else	{
						cap_i_in_scale = 0; //��������� ������������ ��������
						type_work_dc_dc = CURRENT_DC;	//�������� ����� ������������ ����
					}
					//�������� �� ��������� ������ ���������� ���
					if ((cap_pwm_in_long_hight >= PWM_IN_CALIBRATION_MIN) && (cap_pwm_in_long_hight <= PWM_IN_CALIBRATION_MAX))	{
						return_type_work_dc_dc = CURRENT_DC;	//��������� ��� �������� �� ������ ���������� ����� ������������ ����
						type_work_dc_dc = CALIBRATE_ADC_DC;	//�������� ����� ��������� ���
					}
				}
				//������� ��� �� �������� �� ������� �� � ���� �� ����������
				else
					type_work_dc_dc = OFF_DC;	//����� ������ ��� - ��������
			}	
		}
	}
}

//**************
//������ ������� ���������� ���
uint64_t count_64_1 = 0;
uint64_t count_64_2 = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// GPIOA->ODR |= 0x20;//��� ��������� �� �������!!!
  if(htim->Instance == TIM1) //��������� ����� ������ ������ ������
   {

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_cycle);
		//�������� ������ ���
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 2); // �������� ���
   }
}
//**************
//������ ���. ��� ���������� ��������� ������ ������ � ������ ����������� DC/DC
uint32_t flag_adc_calibrate = 1; //������� ����� ��������� ���������� ���
uint32_t count_start_adc = 0;			//
uint32_t count_stop_adc = 0;

uint32_t i_adc;
uint32_t i_adc_offset = 0;	//����������� ����������� �������� 
uint32_t i_adc_filtr;
uint32_t i_adc_filtr_normal;
uint32_t v_adc;
uint32_t v_adc_filtr;
uint32_t v_adc_offset = 0;	//����������� ����������� �������� 
uint32_t v_adc_filtr_normal;


//���������� ���������� ��������
uint32_t pwr_adc;
int32_t pwr_uprav = 0;
int64_t pwr_uprav_filtr = 0;
int32_t pwr_uprav_filtr_out = 0;
uint32_t pwr_duty_cycle = 0;
uint32_t pwr_ustavka = 0;
int32_t pwr_kp = 12;
int32_t pwr_kd = 12;
int32_t pwr_ki = 0;
int32_t pwr_error = 0;
int32_t pwr_old = 0;
int32_t pwr_velocity = 0;
int32_t pwr_i = 0;
int64_t pwr_i64 = 0;
//���������� ���������� ����������
int32_t v_uprav = 0;
int32_t v_uprav_filtr = 0;
int32_t v_uprav_filtr_out = 0;
uint32_t v_duty_cycle = 0;
uint32_t v_ustavka = 0;
int32_t v_kp = 5;
int32_t v_kd = 30;
int32_t v_ki = 3;
int32_t v_error = 0;
int32_t v_old = 0;
int32_t v_velocity = 0;
int32_t v_integrator = 0;
int64_t v_i64 = 0;
//���������� ���������� ����
int32_t i_uprav = 0;
int32_t i_uprav_filtr = 0;
int64_t i_uprav_filtr_out = 0;
uint32_t i_duty_cycle = 0;
uint32_t i_ustavka = 0;
int32_t i_kp = 5;
int32_t i_kd = 30;
int32_t i_ki = 3;
int32_t i_error = 0;
int32_t i_old = 0;
int32_t i_velocity = 0;
int32_t i_integrator = 0;
int64_t i_i64 = 0;


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)	{

	if(hadc->Instance == ADC1)	{ //���������, ��� ����� �  ACD1
		//����������, � ������ ������ ��� ������� �������, � ������� ������ �������, ������������ ��� � ������� �������
		HAL_ADC_Stop_DMA(&hadc1);
		
		//�������� �� ����� ������ DC/DC
		switch (type_work_dc_dc)	{
			case LOW_GND:
				//��� �������� ����������, ��������� ��������, ������ ���������� ������ ������. 
				duty_cycle = PWM_ZERO;	//��������� ���
			break;
//����� ������ ���������� ��� DC/DC			******************************************************************************
			case CALIBRATE_ADC_DC:	//����� ���������� ���
				count_start_adc++;	//������ �������� ���������� ���
				if (count_start_adc <= COUNT_ADC_CALIBRATE_START)
					duty_cycle = PWM_ZERO;	//��������� ���
				else {
					//������ �������� ���������� �����������
					v_adc_offset += adc[0];
					i_adc_offset += adc[1];
					//�������� �� ���������� ������ ��������� ���������� ���
					if (count_start_adc >= COUNT_ADC_CALIBRATE)	{
						count_start_adc = 0;	//����� �������� ���������� ��� ��� ���������� �����
						i_adc_offset = i_adc_offset >> 14;
						v_adc_offset = v_adc_offset >> 14;
						type_work_dc_dc = return_type_work_dc_dc;	//����� ����������, ������� � ����������� �����
					}
				}
			break;
//����� ������ ������������ �������� DC/DC			******************************************************************************
			case POWER_DC:
				//��������� ���������� ����� ��� ���� � ���������� ��� 25 ��� ������ ������ �������� 100 ��
				/*v_adc = v_adc - (v_adc >> 8) + (adc[0] <<12);
				v_adc_filtr = v_adc >> 20;
				i_adc = i_adc - (i_adc >> 8) + (adc[1] << 12);
				i_adc_filtr = i_adc >> 20;*/
				//��������� ���������� ����� ��� ���� � ���������� ��� 25 ��� ������ ������ �������� 390 ��
				/*v_adc = v_adc - (v_adc >> 6) + (adc[0] << 14);
				v_adc_filtr = v_adc >> 20;
				i_adc = i_adc - (i_adc >> 6) + (adc[1] << 14);
				i_adc_filtr = i_adc >> 20;*/
				//��������� ���������� ����� ��� ���� � ���������� ��� 25 ��� ������ ������ �������� 1000 ��
				v_adc = v_adc - (v_adc >> 4) + (adc[0] << 16);
				v_adc_filtr = v_adc >> 20;
				i_adc = i_adc - (i_adc >> 4) + (adc[1] << 16);
				i_adc_filtr = i_adc >> 20;

				//��������� ������ �������� ��� ���������� ��� ������� �����������
				v_old = v_adc_filtr_normal;
				//��������� �������� ����������
				if (v_adc_filtr < v_adc_offset)
					v_adc_filtr_normal = 0;
				else
					v_adc_filtr_normal = v_adc_filtr - v_adc_offset;
				//��������� ������ �������� ��� ���� ��� ������� �����������
				i_old = i_adc_filtr_normal;
				//��������� �������� ��������� ����
				if (i_adc_filtr < i_adc_offset)
					i_adc_filtr_normal = 0;
				else
					i_adc_filtr_normal = i_adc_filtr - i_adc_offset;

				//������ �������� �� ������� (DC/DC)
				pwr_old = pwr_adc;
				pwr_adc = v_adc_filtr_normal * i_adc_filtr_normal;
			
				//������ ����������� ���������� 
				pwr_ustavka = cap_power_in_scale;		//�������� ��������� ��������
				pwr_error = pwr_ustavka - pwr_adc;	//������ ������
				
				pwr_velocity = pwr_adc - pwr_old;		//������ ������ ��������(�����������)
				
				pwr_i64 += pwr_error;
				if ((pwr_i64 >> 32) > 250000)	{
						pwr_i64 = 250000;
						pwr_i64 = pwr_i64 << 32;
				}
				if ((pwr_i64 >> 32) < -250000)	{
						pwr_i64 = -250000;
						pwr_i64 = pwr_i64 << 32;
				}
				pwr_i = pwr_i64 >> 16;
				
				pwr_uprav = pwr_ustavka + ((pwr_error * pwr_kp) >> 8) - pwr_velocity * pwr_kd + ((pwr_i * pwr_ki));
				//���������� ���������� 195 ��
				pwr_uprav_filtr = pwr_uprav_filtr - (pwr_uprav_filtr >> 8) + (((int64_t)pwr_uprav) << 12);
				pwr_uprav_filtr_out = pwr_uprav_filtr >> 20;

				//�������� ������������ ��������
				if (pwr_uprav < 0)
					pwr_duty_cycle = PWR_MIN;
				else	{
					//������������ ����� ������ ���������� � ��������� �� ������������ ���������� ���������� - ��������� ��� -> ���
					//��� (����������) �������� �� ��� (���) - �������� ��������. ������� ��� � ������������ ��������� � ��������.
					//��� ������������ � ������� ���, ��������� �� ��������� ��������, ������� ���������� ������ � ��������� ��������� �������� �� ���������� �����������.
					pwr_duty_cycle = (sqrt(pwr_uprav_filtr_out)*100)/ 445;
					if (pwr_duty_cycle < PWR_MIN)
						pwr_duty_cycle = PWR_MIN;
				}
				//����������� ���������
				if (pwr_duty_cycle > PWR_LIMIT)
					duty_cycle = PWR_LIMIT;
				else
					duty_cycle = pwr_duty_cycle;
				break;
//����� ������ ������������ ���� DC/DC			******************************************************************************
				case CURRENT_DC:
					//��������� ���������� ����� ��� ���� � ���������� ��� 25 ��� ������ ������ �������� 1000 ��
					v_adc = v_adc - (v_adc >> 4) + (adc[0] << 16);
					v_adc_filtr = v_adc >> 20;
					i_adc = i_adc - (i_adc >> 4) + (adc[1] << 16);
					i_adc_filtr = i_adc >> 20;
				
					//��������� ������ �������� ��� ���������� ��� ������� �����������
					v_old = v_adc_filtr_normal;
					//��������� �������� ����������
					if (v_adc_filtr < v_adc_offset)
						v_adc_filtr_normal = 0;
					else
						v_adc_filtr_normal = v_adc_filtr - v_adc_offset;
					//��������� ������ �������� ��� ���� ��� ������� �����������
					i_old = i_adc_filtr_normal;
					//��������� �������� ��������� ����
					if (i_adc_filtr < i_adc_offset)
						i_adc_filtr_normal = 0;
					else
						i_adc_filtr_normal = i_adc_filtr - i_adc_offset;
				
					//������ ���������� �� ����
					i_ustavka = cap_i_in_scale;		//�������� ��������� ��������
					i_error = i_ustavka - i_adc_filtr_normal;//������ ������

					i_velocity = i_adc_filtr_normal - i_old;		//������ ������ �������� (�����������)
					//������ ����������� � ��������� �� �����������
					i_integrator += i_error;										
/*����������� I_INTEGR_MAX �� ����������� �����������*/					if (i_integrator > I_INTEGR_MAX)	{
						i_integrator = I_INTEGR_MAX;
					}
					if (i_integrator < -I_INTEGR_MAX)	{
							i_integrator = -I_INTEGR_MAX;
					}
				
					i_uprav = i_ustavka + i_error * i_kp - i_velocity * i_kd + ((i_integrator >> 16) * i_ki);
					
					//���������� ���������� 195 ��
					i_uprav_filtr = i_uprav_filtr - (i_uprav_filtr >> 8) + (i_uprav << 8);
					i_uprav_filtr_out = i_uprav_filtr >> 16;
					//�������� ������������ ��������
					if (i_uprav_filtr_out < 0)
						i_duty_cycle = MIN_POWER_PWM;
					else	{
					//��������� ��������� �������� �� ���������� �����������.
/*���������� ���������� ������� �� ������������� ��������. ��������� ���������!!!*/					i_duty_cycle = (i_uprav_filtr_out * 100)/ 380;
					if (i_duty_cycle < PWR_MIN)
						i_duty_cycle = PWR_MIN;
					}
					//����������� ���������
					if (i_duty_cycle > MAX_POWER_PWM)
						duty_cycle = MAX_POWER_PWM;
					else
						duty_cycle = i_duty_cycle;
				break;
//����� ������ ������������ ���������� DC/DC			******************************************************************************
				case VOLTAG_DC:
					//��������� ���������� ����� ��� ���� � ���������� ��� 25 ��� ������ ������ �������� 195 ��
					/*v_adc = v_adc - (v_adc >> 7) + (adc[0] << 13);
					v_adc_filtr = v_adc >> 20;
					i_adc = i_adc - (i_adc >> 7) + (adc[1] << 13);
					i_adc_filtr = i_adc >> 20;*/
					//��������� ���������� ����� ��� ���� � ���������� ��� 25 ��� ������ ������ �������� 390 ��
					/*v_adc = v_adc - (v_adc >> 6) + (adc[0] << 14);
					v_adc_filtr = v_adc >> 20;
					i_adc = i_adc - (i_adc >> 6) + (adc[1] << 14);
					i_adc_filtr = i_adc >> 20;*/
					//��������� ���������� ����� ��� ���� � ���������� ��� 25 ��� ������ ������ �������� 1000 ��
					v_adc = v_adc - (v_adc >> 4) + (adc[0] << 16);
					v_adc_filtr = v_adc >> 20;
					i_adc = i_adc - (i_adc >> 4) + (adc[1] << 16);
					i_adc_filtr = i_adc >> 20;
				
					//��������� ������ �������� ��� ���������� ��� ������� �����������
					v_old = v_adc_filtr_normal;
					//��������� �������� ����������
					if (v_adc_filtr < v_adc_offset)
						v_adc_filtr_normal = 0;
					else
						v_adc_filtr_normal = v_adc_filtr - v_adc_offset;
					//��������� ������ �������� ��� ���� ��� ������� �����������
					i_old = i_adc_filtr_normal;
					//��������� �������� ��������� ����
					if (i_adc_filtr < i_adc_offset)
						i_adc_filtr_normal = 0;
					else
						i_adc_filtr_normal = i_adc_filtr - i_adc_offset;
				
					//������ ���������� �� ����������
					v_ustavka = cap_v_in_scale;		//�������� ��������� ��������
					v_error = v_ustavka - v_adc_filtr_normal;//������ ������

					v_velocity = v_adc_filtr_normal - v_old;		//������ ������ �������� (�����������)
					//������ ����������� � ��������� �� �����������
					v_integrator += v_error;										
					if (v_integrator > V_INTEGR_MAX)	{
						v_integrator = V_INTEGR_MAX;
					}
					if (v_integrator < -V_INTEGR_MAX)	{
							v_integrator = -V_INTEGR_MAX;
					}
				
					v_uprav = v_ustavka + v_error * v_kp - v_velocity * v_kd + ((v_integrator >> 16) * v_ki);
					
					//���������� ���������� 195 ��
					v_uprav_filtr = v_uprav_filtr - (v_uprav_filtr >> 8) + (v_uprav << 8);
					v_uprav_filtr_out = v_uprav_filtr >> 16;
					//�������� ������������ ��������
					if (v_uprav_filtr_out < 0)
						v_duty_cycle = MIN_POWER_PWM;
					else	{
					//��������� ��������� �������� �� ���������� �����������.
					v_duty_cycle = (v_uprav_filtr_out * 100)/ 570;
					if (v_duty_cycle < PWR_MIN)
						v_duty_cycle = PWR_MIN;
					}
					//����������� ���������
					if (v_duty_cycle > MAX_POWER_PWM)
						duty_cycle = MAX_POWER_PWM;
					else
						duty_cycle = v_duty_cycle;
				break;
				default:
					//��� �������� ����������, ��������� ��������, ������ ���������� ������ ������. 
					duty_cycle = PWM_ZERO;	//��������� ���
				}
					
			//�������� ���������� �������� ������� ��� - ��� ���������� � ������ ���������� �������� ���
			pwm_in_count_control++;
			if (pwm_in_count_control >= PWM_IN_COUNT_CONTROL_MAX)	{
				duty_cycle = 0;	//��������� ��� ��� ��� ����������� ������� ������ ���������� ���
				pwm_in_count_control = PWM_IN_COUNT_CONTROL_MAX;
			}
			
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
  MX_TIM1_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	//������������� UART
	//HAL_UART_Receive_IT(&huart2,(uint8_t*) recive_str_buffer, 1);

	//������ ���
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 2); // �������� ���
	//������ ��������
__HAL_TIM_CLEAR_FLAG(&htim1, TIM_SR_UIF); // ������� ����
  HAL_TIM_Base_Start_IT(&htim1);

	//��������� PWM
  HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	
	//������ ������� ������� ��� �������� ����������
	HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 639;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 3;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 17;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 6;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim17, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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

