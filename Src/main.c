
/**
	******************************************************************************
	* @file					 : main.c
	* @brief					: Main program body
	******************************************************************************
	** This notice applies to any and all portions of this file
	* that are not between comment pairs USER CODE BEGIN and
	* USER CODE END. Other portions of this file, whether 
	* inserted by the user or by software development tools
	* are owned by their respective copyright owners.
	*
	* COPYRIGHT(c) 2019 STMicroelectronics
	*
	* Redistribution and use in source and binary forms, with or without modification,
	* are permitted provided that the following conditions are met:
	*	 1. Redistributions of source code must retain the above copyright notice,
	*			this list of conditions and the following disclaimer.
	*	 2. Redistributions in binary form must reproduce the above copyright notice,
	*			this list of conditions and the following disclaimer in the documentation
	*			and/or other materials provided with the distribution.
	*	 3. Neither the name of STMicroelectronics nor the names of its contributors
	*			may be used to endorse or promote products derived from this software
	*			without specific prior written permission.
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
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "biquad.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/

// uart_buf is the buffer for receiving MIDI data byte-by-byte.
#define UART_CAP 64
volatile uint8_t uart_buf[UART_CAP];
volatile int uart_head, uart_tail;

// Global DMA/DAC flags.
// dac_wait is set to 0 when either half of the DMA/DAC buffer has finished
// being outputted.
// dac_lower indicates which half of the buffer was last written.
volatile bool dac_wait = false;
volatile bool dac_lower = false;

typedef struct {
	uint8_t number, velocity;
	int counter, period;
	int envelope_counter;
	int lfo_counter;
} Note;

#define SINE_RES 1024
float sine[SINE_RES];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_UART5_Init(void);
static void MX_GFXSIMULATOR_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static inline float compute_sample(int fs, Note *note, float modulation);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static inline float compute_sample(int fs, Note *note, float modulation) {

	int lfo_period = roundf(fs / (1.0f + 9.0f * modulation));
	float lfo = sine[(int) (SINE_RES * (float) note->lfo_counter / lfo_period)];
	note->lfo_counter = (note->lfo_counter + 1) % lfo_period;

	// Sawtooth.
	float phase = (float) note->counter / note->period;
	note->counter = (note->counter+1) % note->period;
	return phase * (1.0f - lfo*modulation);

	// FM synth.
	//float phase = (float) note->counter / note->period;
	//note->counter = (note->counter+1) % note->period;
	//phase += sine[(int) (phase * SINE_RES)] * modulation;
	//phase = fmodf(phase + 1.0f, 1.0f);
	//phase += sine[(int) (phase * SINE_RES)] * modulation;
	//phase = fmodf(2.0f*phase + 1.0f, 1.0f);
	//return 0.5f + 0.5f*sine[(int) (phase * SINE_RES)];

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// Increment uart_tail and request the next byte.
	int new_tail = (uart_tail+1) % UART_CAP;
	if (new_tail != uart_head) {
		uart_tail = new_tail;
		HAL_UART_Receive_IT(&huart5, (uint8_t *) &uart_buf[uart_tail], 1);
	}
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdma) {
	if (dac_wait == false) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	dac_lower = true;
	dac_wait = false;
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdma) {
	if (dac_wait == false) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	dac_lower = false;
	dac_wait = false;
}

/* USER CODE END 0 */

/**
	* @brief	The application entry point.
	*
	* @retval None
	*/
int main(void)
{
	/* USER CODE BEGIN 1 */

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
	MX_DMA_Init();
	MX_DAC_Init();
	MX_TIM6_Init();
	MX_UART5_Init();
	MX_GFXSIMULATOR_Init();
	/* USER CODE BEGIN 2 */

	const int fs = roundf(108e6 / (htim6.Init.Period + 1));

	// Initialize the DAC DMA buffer.
	int dac_size = 2048;
	uint16_t *dac_buf = malloc(dac_size * sizeof(uint16_t));
	if (dac_buf == NULL) _Error_Handler(__FILE__, __LINE__);

	// DMA.
	HAL_TIM_Base_Start(&htim6);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *) dac_buf, dac_size, DAC_ALIGN_12B_R);

	// UART interrupts.
	HAL_UART_Receive_IT(&huart5, (uint8_t *) &uart_buf[uart_head], 1);

	// Precomputed table for exponential pitch bending.
	float pow2[128];
	for (int i = 0; i < 128; i++) {
		pow2[i] = powf(2.0f, (i - 64) / 64.0f);
	}

	// Precomputed note frequencies.
	float note_frequencies[128];
	for (int i = 0; i < 128; i++) {
		note_frequencies[i] = 55.0f * powf(2.0f, (i - 33)/12.0f);
	}

	// Precompute the sine wave.
	for (int i = 0; i < SINE_RES; i++) {
		sine[i] = sinf(2.0f*M_PI * (float) i/SINE_RES);
	}

	// Read bytes from UART and accumulate complete MIDI events in midi_buf.
#define MIDI_CAP 8
	uint8_t midi_buf[MIDI_CAP];
	int midi_len = 0;

	// Active notes are tracked in the notes array.
#define NOTES_CAP 16
	Note notes[NOTES_CAP];
	int notes_len = 0;

	// We also track the volume.
	float volume = 1.0f;
	float pitch_bend = 1.0f;

	// Constants for a simplified sound envelope.
	const int attack = 0.0f * fs;
	const int release = 0.5f * fs;

	// Low pass filter.
	BiquadFilter low_pass;
	biquad_filter_low_pass(&low_pass, fs, 1500.0f, 1.0f);

	// Modulation.
	float modulation = 0.0f;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

		// Wait for a DAC interrupt to reset dac_wait.
		dac_wait = true;
		while (dac_wait) __WFI();

		for (; uart_head != uart_tail; uart_head = (uart_head+1) % UART_CAP) {

			// Reset midi_len if it's a new event.
			if (uart_buf[uart_head] >> 7) midi_len = 0;

			// Push byte onto midi_buf.
			if (midi_len < MIDI_CAP) midi_buf[midi_len] = uart_buf[uart_head];
			midi_len++;

			// Parse events.
			// All the events we care about are 3 bytes long.
			if (midi_len != 3) continue;

			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);

			uint8_t type = midi_buf[0] >> 4;
			if (type == 0x09) {

				// Note on event.

				uint8_t number = midi_buf[1],
				      velocity = midi_buf[2];

				// MIDI note 10 doesn't read correctly. SO weird.
				// I hardcoded a fix.
				static bool toggle = true;
				if (number == 13 && velocity == 10) {
					number = 10;
					if (toggle) velocity = 128;
					else        velocity = 0;
					toggle = !toggle;
				}

				if (velocity == 0) {
					// Find the note in the array and set the velocity to zero.
					for (int i = 0; i < notes_len; i++) {
						if (notes[i].number == number && notes[i].velocity > 0) {
							notes[i].velocity = 0;
							// Update the envelope_counter to account for an unfinished attack stage.
							if (notes[i].envelope_counter < attack) {
								notes[i].envelope_counter = release * ((float) (attack - notes[i].envelope_counter) / attack);
							} else {
								notes[i].envelope_counter = 0;
							}
							break;
						}
					}
				} else {
					// If there's space, push the note onto the array.
					if (notes_len < NOTES_CAP && number < 128) {
						notes[notes_len].number = number;
						notes[notes_len].velocity = velocity;
						float f = pitch_bend * note_frequencies[number];
						notes[notes_len].period = roundf(fs / f);
						notes[notes_len].counter = 0;
						notes[notes_len].envelope_counter = 0;
						notes[notes_len].lfo_counter = 0;
						notes_len++;
					}
				}
			} else if (type == 0x08) {

				// Note off event.

				// Find the note in the array and set the velocity to zero.
				uint8_t number = midi_buf[1];
				for (int i = 0; i < notes_len; i++) {
					if (notes[i].number == number && notes[i].velocity > 0) {
						notes[i].velocity = 0;
						notes[i].envelope_counter = 0;
					}
				}

			} else if (type == 0x0b) {
				if (midi_buf[1] == 0x07) {
					// Volume change.
					if (midi_buf[2] < 128) {
						volume = log2f(midi_buf[2]) / 7.0f;
					}
				} else if (midi_buf[1] == 0x01) {
					// Modulation.
					if (midi_buf[2] < 128) {
						modulation = midi_buf[2] / 128.0f;
						//release = 4.0f * fs * (midi_buf[2] / 128.0f);
					}
				}
			} else if (type == 0xe) {
				// Pitch bending.
				if (midi_buf[2] < 128) {
					pitch_bend = pow2[midi_buf[2]];
					for (int i = 0; i < notes_len; i++) {
						float f = pitch_bend * note_frequencies[notes[i].number];
						notes[i].period = roundf(fs / f);
					}
				}
			}

		}

		// Use dac_lower to figure out with half of the buffer to operate on.
		uint16_t *dac_ptr = dac_lower ? &dac_buf[0] : &dac_buf[dac_size/2];

		// Synthesize the signal.
		float out[dac_size/2];
		memset(out, 0, sizeof(out));

		// LFO.
		//float lfo[dac_size/2];
		//float lfo_frequency = 8.0f;
		//int lfo_period = roundf(fs / lfo_frequency);
		//static int lfo_counter;
		//for (int i = 0; i < dac_size/2; i++) {
		//	lfo[i] = sine[(int) ((float) lfo_counter / lfo_period * SINE_RES)];
		//	lfo_counter = (lfo_counter+1) % lfo_period;
		//}

		// For each note..
		for (int i = 0; i < notes_len; i++) {

			// Synthesize the signal.
			if (notes[i].velocity > 0) {

				int j;
				for (j = 0; j < dac_size/2 && notes[i].envelope_counter < attack; j++, notes[i].envelope_counter++) {
					float envelope = (float) notes[i].envelope_counter / attack;
					out[j] += envelope * compute_sample(fs, &notes[i], modulation);
				}
				for (; j < dac_size/2; j++) {
					out[j] += compute_sample(fs, &notes[i], modulation);
				}

			} else {

				for (int j = 0; j < dac_size/2 && notes[i].envelope_counter < release; j++, notes[i].envelope_counter++) {
					float envelope = (float) (release - notes[i].envelope_counter) / release;
					out[j] += envelope * compute_sample(fs, &notes[i], modulation);
				}

				// If the envelope is finished, remove the note from the array with the ol' swap-and-pop.
				if (notes[i].envelope_counter == release) {
					notes[i] = notes[notes_len-1];
					notes_len--;
					i--;
				}

			}

		}

		// Copy out into dac_ptr.
		for (int j = 0; j < dac_size/2; j++) {
			dac_ptr[j] = (2 << 12) * biquad_filter_process(&low_pass, out[j]) * volume / NOTES_CAP;
		}

		// Heartbeat LED.
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

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
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

		/**Configure the main internal regulator output voltage 
		*/
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

		/**Initializes the CPU, AHB and APB busses clocks 
		*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

		/**Activate the Over-Drive mode 
		*/
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

		/**Initializes the CPU, AHB and APB busses clocks 
		*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
															|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
	PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/* DAC init function */
static void MX_DAC_Init(void)
{

	DAC_ChannelConfTypeDef sConfig;

		/**DAC Initialization 
		*/
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

		/**DAC channel OUT1 config 
		*/
	sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* GFXSIMULATOR init function */
static void MX_GFXSIMULATOR_Init(void)
{

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;

	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 0;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 2047;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

	huart5.Instance = UART5;
	huart5.Init.BaudRate = 31250;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart5) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
	* Enable DMA controller clock
	*/
static void MX_DMA_Init(void) 
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pins : PB0 PB14 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
	* @brief	This function is executed in case of error occurrence.
	* @param	file: The file name as string.
	* @param	line: The line in file as a number.
	* @retval None
	*/
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	while(1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef	USE_FULL_ASSERT
/**
	* @brief	Reports the name of the source file and the source line number
	*				 where the assert_param error has occurred.
	* @param	file: pointer to the source file name
	* @param	line: assert_param error line source number
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
