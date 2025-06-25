/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEARTBEAT_ID 0x3B0
#define ADC_ID 0x3B1
#define ADC_ID1 0x3B2
#define ADC_ID2 0x3B3
#define ADC_ID3 0x3B4
#define EMERGENCY_ID 0x3B5
#define ANW_ID 0x2B1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader_adc;
CAN_TxHeaderTypeDef TxHeader_adc1;
CAN_TxHeaderTypeDef TxHeader_adc2;
CAN_TxHeaderTypeDef TxHeader_adc3;
CAN_TxHeaderTypeDef TxHeader_anw;
CAN_TxHeaderTypeDef TxHeader_heartbeat;
CAN_TxHeaderTypeDef TxHeader_emergency;

uint32_t TxMailBox;
uint8_t TxData_adc[8];
uint8_t TxData_adc1[4];
uint8_t TxData_adc2[4];
uint8_t TxData_adc3[2];
uint8_t TxData_anw[2];
uint8_t TxData_heartbeat[8];
uint8_t TxData_emergency[8];
uint8_t RxData[8];

uint32_t adc_buff[9];
uint16_t value_adc[9];
uint16_t adcSpiBuffer[3];
uint16_t txSpiData;
uint16_t rxSpiData;

uint16_t adc1, adc2, adc3, adc4, adc5, adc6, adc7, adc8, adc9, adc10, adc11, adc12;
uint8_t tempDataFlag, pressDataFlag, fuelDataFlag, rpmDataFlag, battDataFlag,pwmStartFlag,escReadyFlag;
int16_t ect, oilTemp, oilPress, fuelPress, battVolt, rpm;
uint16_t ectTh[4] = {60, 90, 100, 110};// rangos a partir de los cuales cambia el duty
uint16_t oilTh[4] = {80, 100, 120, 130};// rangos a partir de los cuales cambia el duty
uint16_t battTh[3] = {1100, 1125, 1150};// rangos a partir de los cuales cambia el duty
uint16_t dutyFanNill = 48; //es un 4% de duty --> un poco menos de 1ms  time_high = (CCR/AAR)*time_period (f = 50Hz)
uint16_t dutyPumpNill = 10;//duty de la bomba sin funcionar (mirar datasheet) (f = 150hz)
uint16_t dutyFanEctTh[3] = {40, 50, 60}; //poner de 10 en 10 (no 75 )
uint16_t dutyPumpEctTh[3] = {60, 70, 90};
uint16_t dutyFanOilTh[3] = {10, 20, 30}; //poner de 10 en 10 (no 75 )
uint16_t dutyPumpOilTh[3] = {60, 70, 90};
uint8_t battVoltFlagDone[3];
uint16_t battVoltBuffer[10] = {1300, 1300, 1300, 1300, 1300, 1300, 1300, 1300, 1300, 1300};
uint16_t fuelPumpCurrentBuffer[100];
uint16_t fuelPressBuffer[100];
uint16_t battVoltAverage, fuelPumpCurrentAverage, instFuelConsumption, fuelPressAverage;

uint8_t canResetEcuFlag;
uint8_t resetCounter;
uint16_t txSpiData;
uint16_t rxSpiData;
uint8_t ectEmergencyFlag, oilEmergencyFlag, fuelPumpEmergencyFlag, battVoltEmergencyFlag;
uint8_t send = 0;
uint8_t heartbeatFlag = 0;


#define CS_LOW()  HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET);  // Ajusta según tu hardware
#define CS_HIGH() HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);
#define V12NpOff() HAL_GPIO_WritePin(V12_NP_Signal_GPIO_Port, V12_NP_Signal_Pin, RESET);
#define V12NpOn() HAL_GPIO_WritePin(V12_NP_Signal_GPIO_Port, V12_NP_Signal_Pin, SET);


void translateDuty(uint16_t *buffer, uint8_t bufferSize){
	for (uint8_t i = 0; i < bufferSize; i++) {
	        buffer[i] = 50 + ((buffer[i] / 10) * 5);
	    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {

		Error_Handler();
	}

	if (RxHeader.StdId == 0x3A1) {
			tempDataFlag = 1;
			ect = (RxData[3] << 8) | RxData[2];
			ect = ect - 50;
			if(ect < 0){
				ect = 0;
			}
			oilTemp = (RxData[5] << 8) | RxData[4];
			oilTemp = oilTemp - 50;
			if(oilTemp < 0){
				oilTemp = 0;
			}
	}
	if (RxHeader.StdId == 0x3A2){
			pressDataFlag = 1;
			fuelPress = (RxData[1] << 8) | RxData[0];
			oilPress = (RxData[3] << 8) | RxData[2];

	}
	if (RxHeader.StdId == 0x3A3){
			fuelDataFlag = 1;
			instFuelConsumption = (RxData[3] << 8) | RxData[2];
	}
	if (RxHeader.StdId == 0x3A4){
			rpmDataFlag = 1;
			rpm = (RxData[6] << 8) | RxData[5];
	}
	if (RxHeader.StdId == 0x3A5){
			battDataFlag = 1;
			battVolt = (RxData[1] << 8) | RxData[0];

	}
	if (RxHeader.StdId == 0x092){
		if (RxData[0] == 0x02)	{
			canResetEcuFlag = 1;
		}
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	value_adc[0] = (uint16_t) adc_buff[0];	//Los valores pasan de 32 a 16 bits
	value_adc[1] = (uint16_t) adc_buff[1];
	value_adc[2] = (uint16_t) adc_buff[2];
	value_adc[3] = (uint16_t) adc_buff[3];
	value_adc[4] = (uint16_t) adc_buff[4];
	value_adc[5] = (uint16_t) adc_buff[5];
	value_adc[6] = (uint16_t) adc_buff[6];
	value_adc[7] = (uint16_t) adc_buff[7];
	value_adc[8] = (uint16_t) adc_buff[8];
}
void tempActions(){
	tempDataFlag = 0;
	if((ect > ectTh[0])&&(escReadyFlag)){
		//TIM2->CCR3 = dutyFanEctTh[0];// estos dos están apagados para que solo la bomba se encienda en el primer EctTh
		//TIM2->CCR4 = dutyFanEctTh[0];
		TIM16->CCR1 = dutyPumpEctTh[0];

		if(ect >= ectTh[1]){
			TIM2->CCR3 = dutyFanEctTh[0];
			TIM2->CCR4 = dutyFanEctTh[0];
			TIM16->CCR1 = dutyPumpEctTh[1];

			if(ect > ectTh[2]){
				TIM2->CCR3 = dutyFanEctTh[1];
				TIM2->CCR4 = dutyFanEctTh[1];
				TIM16->CCR1 = dutyPumpEctTh[1];

				if(ect > ectTh[3]){
					ectEmergencyFlag = 1;
					TIM2->CCR3 = dutyFanEctTh[2];
					TIM2->CCR4 = dutyFanEctTh[2];
					TIM16->CCR1 = dutyPumpEctTh[2];
				}
			}
		}else{
			TIM2->CCR3 = dutyFanNill;//para que se apaguen a 90
			TIM2->CCR4 = dutyFanNill;
		}
	}else{
		TIM2->CCR3 = dutyFanNill;//Si no entra a esta condición que mande pwms de apagado
		TIM2->CCR4 = dutyFanNill;
		TIM16->CCR1 = dutyPumpNill;
	}
	if((oilTemp > oilTh[0])&&(escReadyFlag)){
		TIM3->CCR1 = dutyFanEctTh[0];
		TIM3->CCR2 = dutyFanEctTh[0];
		TIM17->CCR1 = dutyPumpOilTh[0];

		if(oilTemp > oilTh[1]){
			TIM3->CCR1 = dutyFanEctTh[1];
			TIM3->CCR2 = dutyFanEctTh[1];
			TIM17->CCR1 = dutyPumpOilTh[1];

			if(oilTemp > oilTh[2]){
				TIM3->CCR1 = dutyFanEctTh[2];
				TIM3->CCR2 = dutyFanEctTh[2];
				TIM17->CCR1 = dutyPumpOilTh[2];

				if(oilTemp > oilTh[3]){
					oilEmergencyFlag = 1;
				}
			}
		}
	}else{
		TIM3->CCR1 = dutyFanNill;
		TIM3->CCR2 = dutyFanNill;
		TIM17->CCR1 = dutyPumpNill;
	}
}
void sendCan(){
	send = 0;

	TxData_adc[0] = (adc8 >> 8) & 0xFF;
	TxData_adc[1] = adc8 & 0xFF;
	TxData_adc[2] = (adc2 >> 8) & 0xFF;
	TxData_adc[3] = adc2 & 0xFF;
	TxData_adc[4] = (adc3 >> 8) & 0xFF;
	TxData_adc[5] = adc3 & 0xFF;
	TxData_adc[6] = (adc4 >> 8) & 0xFF;
	TxData_adc[7] = adc4 & 0xFF;

	HAL_CAN_AddTxMessage(&hcan, &TxHeader_adc, TxData_adc, &TxMailBox);

	TxData_adc1[0] = (adc5 >> 8) & 0xFF;
	TxData_adc1[1] = adc5 & 0xFF;
	TxData_adc1[2] = (adc1 >> 8) & 0xFF;
	TxData_adc1[3] = adc1 & 0xFF;

	HAL_CAN_AddTxMessage(&hcan, &TxHeader_adc1, TxData_adc1, &TxMailBox);

	TxData_adc2[0] = (adc6 >> 8) & 0xFF;
	TxData_adc2[1] = adc6 & 0xFF;
	TxData_adc2[2] = (adc9 >> 8) & 0xFF;
	TxData_adc2[3] = adc9 & 0xFF;

	HAL_CAN_AddTxMessage(&hcan, &TxHeader_adc2, TxData_adc2, &TxMailBox);

	if(fuelPumpEmergencyFlag){
		TxData_emergency[0] = 1;
	}
	if(ectEmergencyFlag){
		TxData_emergency[1] = 1;
	}
	if(oilEmergencyFlag){
		TxData_emergency[2] = 1;
	}
	if(battVoltEmergencyFlag){
		TxData_emergency[3] = 1;
	}
	HAL_CAN_AddTxMessage(&hcan, &TxHeader_emergency, TxData_emergency, &TxMailBox);

}
void heartbeat(){
	TxData_heartbeat[0] = 4;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader_heartbeat, TxData_heartbeat,&TxMailBox);
	heartbeatFlag = 0;

}
void mapeoADC(){
	adc1 = ((value_adc[0] * (3300 / 4095)) - 330) * (1 /132)*1000; // ALTERNATOR la trasnformacion ya en mv y de momento, resultado en A
	adc2 = ((value_adc[1] * (3300 / 4095)) - 330) * (1 /264)*1000;; // WPL
	adc3 = ((value_adc[2] * (3300 / 4095)) - 330) * (1 / 264)*1000;; // WPR
	adc4 = ((value_adc[3] * (3300 / 4095)) - 330) * (1 / 264)*1000;; // F1R
	adc5 = ((value_adc[4] * (3300 / 4095)) - 330) * (1 / 264)*1000;; // F2R
	adc6 = ((value_adc[5] * (3300 / 4095)) - 260) * (1 / 264)*1000;; // F1L
	adc7 = ((value_adc[6] * (3300 / 4095)) - 260) * (1 / 264)*1000;; // F2L
	adc8 = ((value_adc[7] * (3300 / 4095)) - 260) * (10 / 88)*1000;; // 12VNP
	adc9 = ((value_adc[8] * (3300 / 4095) - 500) * (1 / 10))*1000;; // calibración del sensor 0.01V/ºC
	adc10 = ((adcSpiBuffer[0] * (3303 / 4095)) - 260) * (1 / 264); //Injection
	adc11 = ((adcSpiBuffer[1] * (3300 / 4095)) - 260) * (1 /264)*1000;; // Fuel Pump
	adc12 = ((adcSpiBuffer[2] * (3300 / 4095)) - 270) * (1 / 264)*1000;; // Ignition
}
void battControl(){
	battDataFlag = 0;
	uint8_t arrayLength = (sizeof(dutyFanEctTh)/sizeof(dutyFanEctTh[0]));
	if((battVoltAverage < battTh[2])&&(battVoltFlagDone[0] == 0)){
		battVoltFlagDone[0] = 1;
		for(uint8_t i=0; (i<arrayLength); i++){
			dutyFanEctTh[i] = dutyFanEctTh[i]-5;
			dutyFanOilTh[i] = dutyFanOilTh[i]-5;
		}
	}
	if((battVoltAverage < battTh[1])&&(battVoltFlagDone[1] == 0)){
		battVoltFlagDone[1] = 1;
		for(uint8_t i=0; (i<arrayLength); i++){
			dutyFanEctTh[i] = dutyFanEctTh[i]-7;
			dutyFanOilTh[i] = dutyFanOilTh[i]-7;

		}
	}
	if(battVoltAverage < battTh[0]){
		V12NpOff();
		if(battVoltFlagDone[2] == 0){
			battVoltFlagDone[2] = 1;
			battVoltEmergencyFlag = 1;
			for(uint8_t i=0; (i<arrayLength); i++){
				dutyFanEctTh[i] = dutyFanEctTh[i]-10;
				dutyFanOilTh[i] = dutyFanOilTh[i]-10;
			}
		}
	}else{
		V12NpOn();
	}
}
void canResetEcu(){
	HAL_GPIO_WritePin(Ecu_Signal_GPIO_Port, Ecu_Signal_Pin, RESET);
	if(resetCounter >= 2){
		canResetEcuFlag = 0;
		resetCounter = 0;
		HAL_GPIO_WritePin(Ecu_Signal_GPIO_Port, Ecu_Signal_Pin, SET);
	}

}
void Read_All_ADC_Channels() {
    uint8_t numChannels = 3;
    uint16_t dummyRead;  // Variable para la primera lectura incorrecta

    // 1ª vuelta: se configuran los canales pero los datos leídos no son válidos aún
    for (uint8_t i = 0; i < numChannels; i++) {
        txSpiData = (i & 0x07) << 12;  // Configurar el canal en el mensaje de SPI
        CS_LOW();
        HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&txSpiData, (uint8_t*)&dummyRead, 1, HAL_MAX_DELAY);
        CS_HIGH();
    }

    HAL_Delay(1);  // Breve pausa para asegurar estabilidad

    // 2ª vuelta: ahora sí obtenemos los datos correctos
    for (uint8_t i = 0; i < numChannels; i++) {
        txSpiData = (i & 0x07) << 12;  // Configurar el canal en el mensaje de SPI
        rxSpiData = 0;

        CS_LOW();
        HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&txSpiData, (uint8_t*)&rxSpiData, 1, HAL_MAX_DELAY);
        CS_HIGH();

        adcSpiBuffer[i] = rxSpiData & 0x0FFF;  // Extraer solo los 12 bits de datos del ADC
    }
}
void currentChecking(){

}
uint16_t getBufferAverage(uint16_t *buffer, uint8_t bufferSize) {
    uint32_t sum = 0;

    for (uint8_t i = 0; i < bufferSize; i++) {
        sum += buffer[i];
    }

    return (sum + (bufferSize / 2)) / bufferSize;  // Redondeo clásico
}
void fillBuffer(uint16_t *buffer, uint16_t bufferSize, uint16_t newValue) {
    for (uint8_t i = 0; i < bufferSize - 1; i++) {
        buffer[i] = buffer[i + 1];  // Mueve los valores a la izquierda
    }
    buffer[bufferSize - 1] = newValue;  // Inserta el nuevo valor al final
}
void fuelPumpProtection(){
	fillBuffer(fuelPumpCurrentBuffer, 100, adc11);
	fuelPumpCurrentAverage = getBufferAverage(fuelPumpCurrentBuffer, 100);
	fillBuffer(fuelPressBuffer, 100, fuelPress);
	fuelPressAverage = getBufferAverage(fuelPressBuffer, 100);
	if((fuelPumpCurrentAverage > 65000)&&(fuelPumpEmergencyFlag == 0)){
		fuelPumpEmergencyFlag = 1;
	}
	if((fuelPressAverage < 200)&&(rpm > 1000)&&(fuelPumpEmergencyFlag == 0)){
		fuelPumpEmergencyFlag = 1;
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_CAN_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc, adc_buff, 9); //Inicia el DMA se le pasa el ADC, la variable donde guardar los datos y el numero de canales
  HAL_ADC_Start_IT(&hadc); //Se inicia la interrupcion de fin de conversion del ADC en el "Set-up"
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim14);
  TIM2->CCR3 = dutyFanNill;
  TIM2->CCR4 = dutyFanNill;
  TIM3->CCR1 = dutyFanNill;
  TIM3->CCR2 = dutyFanNill;
  TIM16->CCR1 = dutyPumpNill;
  TIM17->CCR1 = dutyPumpNill;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);


	TxHeader_adc.DLC = 8;
	TxHeader_adc.ExtId = 0;
	TxHeader_adc.IDE = CAN_ID_STD;
	TxHeader_adc.RTR = CAN_RTR_DATA;
	TxHeader_adc.StdId = ADC_ID;
	TxHeader_adc.TransmitGlobalTime = DISABLE;

	TxHeader_adc1.DLC = 4;
	TxHeader_adc1.ExtId = 0;
	TxHeader_adc1.IDE = CAN_ID_STD;
	TxHeader_adc1.RTR = CAN_RTR_DATA;
	TxHeader_adc1.StdId = ADC_ID1;
	TxHeader_adc1.TransmitGlobalTime = DISABLE;

	TxHeader_adc2.DLC = 4;
	TxHeader_adc2.ExtId = 0;
	TxHeader_adc2.IDE = CAN_ID_STD;
	TxHeader_adc2.RTR = CAN_RTR_DATA;
	TxHeader_adc2.StdId = ADC_ID2;
	TxHeader_adc2.TransmitGlobalTime = DISABLE;

	TxHeader_adc3.DLC = 2; //AQUI ESTABA EL ERROR; ESO ESTABA COMO TxHeader_adc2
	TxHeader_adc3.ExtId = 0;
	TxHeader_adc3.IDE = CAN_ID_STD;
	TxHeader_adc3.RTR = CAN_RTR_DATA;
	TxHeader_adc3.StdId = ADC_ID3;
	TxHeader_adc3.TransmitGlobalTime = DISABLE;

	TxHeader_anw.DLC = 2;
	TxHeader_anw.ExtId = 0;
	TxHeader_anw.IDE = CAN_ID_STD;
	TxHeader_anw.RTR = CAN_RTR_DATA;
	TxHeader_anw.StdId = ANW_ID;
	TxHeader_anw.TransmitGlobalTime = DISABLE;

	TxHeader_heartbeat.DLC = 1;
	TxHeader_heartbeat.ExtId = 0;
	TxHeader_heartbeat.IDE = CAN_ID_STD;
	TxHeader_heartbeat.RTR = CAN_RTR_DATA;
	TxHeader_heartbeat.StdId = HEARTBEAT_ID;
	TxHeader_heartbeat.TransmitGlobalTime = DISABLE;

	TxHeader_emergency.DLC = 8;
	TxHeader_emergency.ExtId = 0;
	TxHeader_emergency.IDE = CAN_ID_STD;
	TxHeader_emergency.RTR = CAN_RTR_DATA;
	TxHeader_emergency.StdId = EMERGENCY_ID;
	TxHeader_emergency.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();

	}


	translateDuty(dutyFanEctTh, 3);
	translateDuty(dutyFanOilTh, 3);
	V12NpOn();
	HAL_GPIO_WritePin(WPL_Signal_GPIO_Port, WPL_Signal_Pin, SET);
	HAL_GPIO_WritePin(F1L_Signal_GPIO_Port, F1L_Signal_Pin, SET);
	HAL_GPIO_WritePin(F2L_Signal_GPIO_Port, F2L_Signal_Pin, SET);
	HAL_GPIO_WritePin(WPR_Signal_GPIO_Port, WPR_Signal_Pin, SET);
	HAL_GPIO_WritePin(F1R_Signal_GPIO_Port, F1R_Signal_Pin, SET);
	HAL_GPIO_WritePin(F2R_Signal_GPIO_Port, F2R_Signal_Pin, SET);
	TIM2->CCR3 = dutyFanNill;
	TIM2->CCR4 = dutyFanNill;
	TIM16->CCR1 = dutyPumpNill;
	TIM3->CCR1 = dutyFanNill;
	TIM3->CCR2 = dutyFanNill;
	TIM17->CCR1 = dutyPumpNill;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  mapeoADC();
	  if(tempDataFlag){
		  tempActions();
	  }
	  if(send){
		  sendCan();
		  Read_All_ADC_Channels();
	  }
	  if(heartbeatFlag > 5 ){
		  heartbeat();
	  }
	  if(battDataFlag){
		  fillBuffer(battVoltBuffer, 10, battVolt);
		  battVoltAverage= getBufferAverage(battVoltBuffer, 10);
		  battControl();
	  }
	  if(canResetEcuFlag){
		  canResetEcu();
	  }else{
		  HAL_GPIO_WritePin(Ecu_Signal_GPIO_Port, Ecu_Signal_Pin, SET);
	  }
	  if(pressDataFlag){
		  fuelPumpProtection();
	  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	CAN_FilterTypeDef canfilterconfig;

//	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
//	canfilterconfig.FilterBank = 10;
//	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//	canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
//	canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;
//	canfilterconfig.FilterIdHigh = 0x1B1 << 5;
//	canfilterconfig.FilterIdLow = 0x3A1 << 5;
//	canfilterconfig.SlaveStartFilterBank = 0;
	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 10;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;  // Modo enmascarado
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT; // Filtro de 32 bits

	// Aceptar todos los IDs: ID = 0x00000000, Máscara = 0x00000000
	canfilterconfig.FilterIdHigh = 0x0000;
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0x0000;
	canfilterconfig.FilterMaskIdLow = 0x0000;

	canfilterconfig.SlaveStartFilterBank = 0;

	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 48000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 959;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 959;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 4000; // ponerlo para 2/4 segundos
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 48000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 3170;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 3170;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 100;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Ecu_Signal_GPIO_Port, Ecu_Signal_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, V12_NP_Signal_Pin|CS_PIN_Pin|F1R_Signal_Pin|F2L_Signal_Pin
                          |F1L_Signal_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, WPR_Signal_Pin|WPL_Signal_Pin|Reset_Pin|F2R_Signal_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Ecu_Signal_Pin */
  GPIO_InitStruct.Pin = Ecu_Signal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Ecu_Signal_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : V12_NP_Signal_Pin CS_PIN_Pin F1R_Signal_Pin F2L_Signal_Pin
                           F1L_Signal_Pin */
  GPIO_InitStruct.Pin = V12_NP_Signal_Pin|CS_PIN_Pin|F1R_Signal_Pin|F2L_Signal_Pin
                          |F1L_Signal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : WPR_Signal_Pin WPL_Signal_Pin Reset_Pin F2R_Signal_Pin */
  GPIO_InitStruct.Pin = WPR_Signal_Pin|WPL_Signal_Pin|Reset_Pin|F2R_Signal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		send = 1;
		heartbeatFlag = heartbeatFlag+1;
		if(canResetEcuFlag == 1){
			resetCounter = resetCounter+1;
		}
	}
	if (htim->Instance == TIM14) {
			pwmStartFlag = 1;
			escReadyFlag = 1;
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
