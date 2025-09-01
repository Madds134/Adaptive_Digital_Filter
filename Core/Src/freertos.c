#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "arm_math.h"
#include "fir_coeffs.h"
#include "semphr.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* Global Defines */
// Peripheral handles
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
// RTOS handles
static TaskHandle_t samplerTaskHandle  = NULL;
static TaskHandle_t filterTaskHandle   = NULL;
static TaskHandle_t potTaskHandle      = NULL;
static TaskHandle_t	uartTaskHandle	   = NULL;

// ADC & DMA
#define ADC_BUF_LEN     1024u // Must be even for half-transfer
__attribute__((aligned(4))) static uint16_t adcBuf[ADC_BUF_LEN];
// Flag for Which half the ISR says is ready
static volatile uint8_t activeHalf = 0;

// UART streaming
#define TX_BLOCK_SAMPLES	128u	// Samples per UART group
#define TX_POOL_BLOCKS		6u		// Number of reusable buffers
#define TX_QUEUE_DEPTH		8u		// How many block can wait for UART

typedef struct {
	float32_t *buf;		// Pointer to buffer holding the samples
	uint16_t count;		// Valid samples in the buffer
} TxBlock;

static QueueHandle_t	txQueue			= NULL; // Filled blocks -> UARTTask
static QueueHandle_t	freePool		= NULL; // Empty buffers -> FilterTask
static SemaphoreHandle_t xUartMutex		= NULL; // Serialize UART

__attribute__((aligned(4))) static float32_t txPool[TX_POOL_BLOCKS][TX_BLOCK_SAMPLES];

/* Potentiometer */
#define POT_TASK_PERIOD_MS   100
#define POT_ADC                  (&hadc1)
#define POT_ADC_CHANNEL          ADC_CHANNEL_1
// Small hysteresis to avoid conflicting coefficient updates
#define COEF_CHANGE_EPS          0.02f   // 2%

/* DSP buffers/state */
#define BLOCK_SIZE      (ADC_BUF_LEN / 2)
static float32_t inBlock[BLOCK_SIZE];
static float32_t outBlock[BLOCK_SIZE];

// Filter specs
#define FS_HZ       1000.0f  // Processing sampling rate
// Cutoff range
#define MIN_FC_HZ   2.0f
#define MAX_FC_HZ   400.0f
// Butterworth-like default Q
#define Q_DEFAULT   0.70710678f

// One DF1 biquad stage
// Scalable defines
#define NUM_BIQUAD_STAGES        1
#define BIQ_COEF_LEN_PER_STAGE   5   // {b0, b1, b2, a1, a2}
#define BIQUAD_STATE_LEN         (4 * NUM_BIQUAD_STAGES)

// PotTask & Biquad
static SemaphoreHandle_t gBiquadMutex = NULL;
static arm_biquad_casd_df1_inst_f32 gBiq; // Instance of filter
static float32_t gBiquadCoef[NUM_BIQUAD_STAGES * BIQ_COEF_LEN_PER_STAGE]; // Coefficients
static float32_t gBiquadState[BIQUAD_STATE_LEN]; // States
static void   PotTask(void *arg);
static uint16_t Pot_ReadRaw12_Injected(void);
static float Pot_MapToCutoffHz(uint16_t raw12);
static void Biquad_LP_RBJ(float fs, float fc, float Q, float *co);
static void DSP_InitBiquadLocked(float fs, float fc, float Q);

/**
 * @brief Thread-safe, is a blocking UART transmit
 */
static void UART_Write(const uint8_t *data, uint16_t len)
{
	xSemaphoreTake(xUartMutex, portMAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);
	xSemaphoreGive(xUartMutex);
}

/**
 * @brief Format a floating-point value as a CSV line
 * @param v	Floating-point value to format
 * @param dst Destination character buffer to receive the formatted string
 * @param dst_size Size of the destination buffer in bytes
 */
static inline uint16_t fmt_csv_line(float32_t v, char *dst, size_t dst_size)
{
    int n = snprintf(dst, dst_size, "%.6f\r\n", (double)v);
    return (n > 0) ? (uint16_t)n : 0;
}

/*
 * @brief Task that starts the DMA, then suspends the task
 */
static void SamplerTask(void *arg)
{
    // Ensure half-transfer & transfer-complete IT are enabled
    __HAL_DMA_ENABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(hadc1.DMA_Handle, DMA_IT_TC);

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, ADC_BUF_LEN) != HAL_OK)
    {
        for(;;) {}
    }

    // Start the 1kHz trigger for ADC
    HAL_TIM_Base_Start(&htim3);

    // Nothing else to do; DMA runs in background and ISRs will trigger filterTask
    vTaskSuspend(NULL);
}

/**
 * @brief ISR for when DMA half transfer is complete, when the HT interrupt is fired
 * @details DMA handoff to the RTOS inside the ISR
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        BaseType_t hpw = pdFALSE; // Flag for if high priority task was woken
        activeHalf = 0;
        vTaskNotifyGiveFromISR(filterTaskHandle, &hpw);
        portYIELD_FROM_ISR(hpw); // Request context switch if task woken
    }
}

/**
 * @brief ISR for when DMA transfer is complete, when the TC interrupt is fired
 * @details DMA handoff to the RTOS inside the ISR
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        BaseType_t hpw = pdFALSE; // Flag for if high priority task was woken
        activeHalf = 1;
        vTaskNotifyGiveFromISR(filterTaskHandle, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

/*
 * @brief FilterTask: block on notifications, process the ready half
 */
static void FilterTask(void *arg)
{
    for (;;)
    {
        // Block until either half or full interrupt occurs
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Snapshot which half the ISR said is ready
        uint32_t start = (activeHalf == 0) ? 0 : (ADC_BUF_LEN / 2);
        uint32_t count = ADC_BUF_LEN / 2;

        // Convert ADC samples to floats
        for (uint32_t i = 0; i < count; i++)
        {
            inBlock[i] = (float32_t)adcBuf[start + i];
        }

        // Run the filter, protected under a mutex
        xSemaphoreTake(gBiquadMutex, portMAX_DELAY);
        arm_biquad_cascade_df1_f32(&gBiq, inBlock, outBlock, count);
        xSemaphoreGive(gBiquadMutex);

        // Debug toggle to see if the processing is happening
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        // hand off filtered samples to UARTTask in chunks
        uint32_t produced = count;
        uint32_t idx = 0;

        // Loop until handed off all samples
        while (produced)
        {
            float32_t *bufPtr = NULL;
            // Block here if the UART is behind
            xQueueReceive(freePool, &bufPtr, portMAX_DELAY);

            const uint16_t chunk = (produced > TX_BLOCK_SAMPLES)
                                   ? TX_BLOCK_SAMPLES
                                   : (uint16_t)produced;
            // Copy that slice from output array into the buffer
            memcpy(bufPtr, &outBlock[idx], (size_t)chunk * sizeof(float32_t));

            // Hand off to UART task
            TxBlock blk = { .buf = bufPtr, .count = chunk };
            xQueueSend(txQueue, &blk, portMAX_DELAY);

            produced -= chunk;
            idx      += chunk;
        }

        // Drain any queued notifications
        UBaseType_t pending = ulTaskNotifyTake(pdTRUE, 0);
        while (pending--)
        {
            start = (activeHalf == 0) ? 0 : (ADC_BUF_LEN / 2);
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
    }
}

// Low-priority task: dequeue blocks, format CSV, transmit, return buffer to pool
static void UARTTask(void *arg)
{
	(void)arg;
	// CSV header
	const char *hdr = "# filtered_samples\r\n";
	UART_Write((const uint8_t*)hdr, (uint16_t)strlen(hdr));

	char line[24]; // Temporary ASCII for one formatted float
	uint8_t	batch[1024]; // Buffer to batch lines into a single UART write
	uint16_t used = 0; // Number of valid bytes in the batch

	for(;;)
	{
		TxBlock blk;
		// if producer doesn't send a block
		if(xQueueReceive(txQueue, &blk, portMAX_DELAY) != pdTRUE)
		{
			continue;
		}
		// Loop through each valid sample in the block
		for(uint16_t i = 0; i < blk.count; i++)
		{
			uint16_t w = fmt_csv_line(blk.buf[i], line, sizeof(line));
			// Skip sample if it is 0
			if(!w)
			{
				continue;
			}
			// If batch has no room
			if(used + w > sizeof(batch))
			{
				// Flush batch
				UART_Write(batch, used);
				used = 0;
			}
			// Append formatted line to the batch
			memcpy(&batch[used], line, w);
			used += w;
		}
		// Send any leftovers
		if(used)
		{
			UART_Write(batch, used);
			used = 0;
		}

		// Return the buffer to the free pool
		(void)xQueueSend(freePool, &blk.buf, portMAX_DELAY);
	}
}

/**
 * @brief Reads the Potentiometer value from the injected ADC on PA1
 * @return 12-bit raw ADC value
 */
static uint16_t Pot_ReadRaw12_Injected(void)
{
    HAL_ADCEx_InjectedStart(POT_ADC); // Start injected ADC conversion
    (void)HAL_ADCEx_InjectedPollForConversion(POT_ADC, 2); // wait up to 2ms for completion
    uint32_t v = HAL_ADCEx_InjectedGetValue(POT_ADC, ADC_INJECTED_RANK_1); // Read converted ADC value
    HAL_ADCEx_InjectedStop(POT_ADC); // Stop conversion
    return (uint16_t)(v & 0x0FFFu); // Mask lower 12 bits since 12 bit ADC
}
/**
 * @brief Take 12-bit raw ADC reading from a potentiometer and map it to a cuttoff frequency
 */
static float Pot_MapToCutoffHz(uint16_t raw12)
{
    const float t = (float)raw12 / 4095.0f; // Normalize raw value into [0.0, 1.0]

    // Log-uniform mapping
    const float lnMin = logf(MIN_FC_HZ);
    const float lnMax = logf(MAX_FC_HZ);
    // Perform interpolation and get it back to hertz
    float fc = expf(lnMin + t * (lnMax - lnMin));

    // Safety clamps
    if (fc < 1.0f) fc = 1.0f; // Don't allow less than 1Hz
    const float nyq_guard = 0.45f * FS_HZ; // Nyquist frequency quard
    if (fc > nyq_guard) fc = nyq_guard; // Don't allow more than 450Hz
    return fc;
}

/**
 * @brief Compute coefficients for a 2nd order low-pass biquad filter using RBJ cookbook formulas
 * @param[in] fs	sampling rate in Hz (must be nonnegative)
 * @param[in] fc	desired cutoff frequency in hertz
 * @param[in] Q		Quality factor
 * @param[out] co	pointer to array of coefficients
 */
static void Biquad_LP_RBJ(float fs, float fc, float Q, float *co)
{
	// Clamp the cut-off frequencys
    if (fc < 1.0f) fc = 1.0f;
    if (fc > 0.40f * fs)  fc = 0.40f * fs;
    if (Q   < 0.05f)      Q   = 0.05f; // Avoid extreme resonance
    // Get constants for the cookbook RBJ equations
    const float w0 = 2.0f * (float)M_PI * (fc / fs);
    const float c  = cosf(w0);
    const float s  = sinf(w0);
    const float a  = s / (2.0f * Q);
    // Calculating the coefficents
    float b0 = (1.0f - c) * 0.5f;
    float b1 = (1.0f - c);
    float b2 = (1.0f - c) * 0.5f;
    float a0 = (1.0f + a);
    float a1 = (-2.0f * c);
    float a2 = (1.0f - a);
    // Normalize by a0
    b0 /= a0;  b1 /= a0;  b2 /= a0;
    a1 /= a0;  a2 /= a0;

    // CMSIS DF1 expects {b0,b1,b2,a1,a2} with negated a1,a2 in the array
    co[0] = b0;
    co[1] = b1;
    co[2] = b2;
    co[3] = -a1; // Store negated
    co[4] = -a2; // Store negated
}
/**
 * @brief Initialize and configure the biquad filter instance
 * @details This function recalculates the coefficients of the filter.
 * @param fs Sampling frequency in Hz
 * @param fc Cutoff frequency in Hz
 * @param Q Quality factor
 */
static void DSP_InitBiquadLocked(float fs, float fc, float Q)
{
	// Only one task at a time updates the filter
    xSemaphoreTake(gBiquadMutex, portMAX_DELAY);
    // Generate the filter coefficients
    float c[5];
    Biquad_LP_RBJ(fs, fc, Q, c);
    // Copy coefficients into global array
    for (int i = 0; i < 5; i++)
        gBiquadCoef[i] = c[i];
    // Reset filter state, clears delay line
    memset(gBiquadState, 0, sizeof(gBiquadState));
    // Initiliaze filter instance
    arm_biquad_cascade_df1_init_f32(&gBiq, NUM_BIQUAD_STAGES, gBiquadCoef, gBiquadState);
    // Release Mutex
    xSemaphoreGive(gBiquadMutex);
}
/**
 * @brief FreeRTOS task to update the biquad filter cutoff from a potentiometer
 */
static void PotTask(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(POT_TASK_PERIOD_MS); // Convert period into RTOS ticks
    TickType_t next = xTaskGetTickCount(); // Get current tick count

    float fc_prev = 0.0f; // Previous cut off frequency

    for (;;)
    {
        vTaskDelayUntil(&next, period); // Delay task for 100ms
        uint16_t raw = Pot_ReadRaw12_Injected(); // Read ADC value
        float fc = Pot_MapToCutoffHz(raw); // Get cutoff frequency from that data

        const float base = (fc_prev > 1e-3f) ? fc_prev : fc; // If has changed
        if (fabsf(fc - fc_prev) < COEF_CHANGE_EPS * base)
            continue; // ignore tiny changes
        fc_prev = fc; // Save new cut off frequency
        // Lock biquad mutex
        xSemaphoreTake(gBiquadMutex, portMAX_DELAY);
        // Compute new coefficients for new cutoff
        float c[5];
        Biquad_LP_RBJ(FS_HZ, fc, Q_DEFAULT, c);
        for (int i = 0; i < 5; i++)
            gBiquadCoef[i] = c[i];
        // Reinitialize the biquad filter wit new coefficients and existing state
        arm_biquad_cascade_df1_init_f32(&gBiq, NUM_BIQUAD_STAGES, gBiquadCoef, gBiquadState);
        xSemaphoreGive(gBiquadMutex);
    }
}

/**
 * @brief Initalize all RTOS tasks, synchronization objects, and DSP
 */
void RTOS_init(void)
{
	// Create filter and sampler task
    xTaskCreate(SamplerTask, "Sampler", 256, NULL, tskIDLE_PRIORITY + 2, &samplerTaskHandle);
    xTaskCreate(FilterTask, "Filter",  512, NULL, tskIDLE_PRIORITY + 3, &filterTaskHandle);

    // Biquad mutex + initial configuration of filter
    gBiquadMutex = xSemaphoreCreateMutex();
    configASSERT(gBiquadMutex != NULL);
    DSP_InitBiquadLocked(FS_HZ, 100.0f, Q_DEFAULT);

    // Create potentiometer task
    xTaskCreate(PotTask, "PotTask", 256, NULL, tskIDLE_PRIORITY + 2, &potTaskHandle);

    // UART mutex + Queue
    xUartMutex = xSemaphoreCreateMutex();
    configASSERT(xUartMutex != NULL);
    txQueue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(TxBlock));
    freePool = xQueueCreate(TX_POOL_BLOCKS, sizeof(float32_t*));
    configASSERT(txQueue && freePool);

    // Send free pool with pointers to the reusable buffers
    // So each producer can immediately get a buffer
    for(uint32_t i = 0; i < TX_POOL_BLOCKS; i++)
    {
    	float32_t *p = &txPool[i][0];
    	xQueueSend(freePool, &p, portMAX_DELAY);
    }
    // UART task (lowest priority)
    xTaskCreate(UARTTask, "UART", 512, NULL, tskIDLE_PRIORITY + 1, &uartTaskHandle);

}
/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}


/* Unit test functions for components of the system*/

/* UART tests */
/**
 * @brief Verify the peripheral is functioning as intended
 */
void UART_test_peripheral(void)
{
	const char *message = "UART peripheral test\r\n";
	if(HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100) != HAL_OK)
	{
		//printf("UART Peripheral test failed (transmit error)\r\n"); // Print to SWV
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED ON
	}	else
	{
		//printf("UART Peripheral test passed (transmit success)\r\n"); // Print to SWV
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED OFF
	}

}

/**
 * @brief Verify's the CSV formatting
 */
void UART_test_format(void)
{
	char line[24];
	int16_t testVal = 1234;

	int length = snprintf(line, sizeof(line), "%d\r\n", testVal);

	if( length <= 0 || length >= (int)sizeof(line))
	{
		//printf("UART format test failed(snprintf error/overflow\r\n");
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else if(strcmp(line, "1234\r\n") == 0)
	{
		//printf("UART format test passed (Got '%s')\r\n", line);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
	else
	{
		//printf("UART_Test_Format: FAIL (Got '%s')\r\n", line);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
}
/**
 * @brief Test a formatted block and verify the count of the block
 */
void UART_test_BlockTransmit(void)
{
	char buf[64];
	int len = snprintf(buf, sizeof(buf), "BLOCK_TEST,%u,%u\r\n", 1, 2);

    if (len <= 0) {
        //printf("UART_Test_BlockTransmit: FAIL (format error)\r\n");
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        return;
    }

    if (HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100) != HAL_OK) {
        //printf("UART_Test_BlockTransmit: FAIL (Transmit error)\r\n");
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    } else {
        //printf("UART_Test_BlockTransmit: PASS (Sent '%s')\r\n", buf);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}
/**
 * @brief Tests all UART unit tests
 */
void test_UART(void)
{
	UART_test_peripheral();
	UART_test_format();
	UART_test_BlockTransmit();
}

/** DSP Unit tests */
/**
 * @brief Verify Potentiometer mapping produces valid cutoff range
 */
void test_PotMap(void)
{
	float fc0 = Pot_MapToCutoffHz(0);
	float fcMax = Pot_MapToCutoffHz(4095);

	if(!(fc0 >= 1.0f) || !(fcMax <= (0.45f * FS_HZ)))
	{
		//printf("Pot map test failed (fc0=%.2f, fcMax=%.2f)\r\n", fc0, fcMax);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else
	{
		//printf("Pot map test passed (fc0=%.2f, fcMax=%.2f)\r\n", fc0, fcMax);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}
/**
 * @brief Verify RBJ LP filter coefficients are valid and symmetric
 */
void DSP_test_RBJ_Coeffs(void)
{
	float co[5];
	Biquad_LP_RBJ(1000.0f, 100.0f, 0.7071f, co);

	if(!isfinite(co[0]) || fabsf(co[0]-co[2]) > 1e-4f)
	{
		//printf("RBJ coeff test failed (b0=%.4f, b2=%.4f)\r\n", co[0], co[2]);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else
	{
		//printf("RBJ coeff test passed (b0=%.4f, b2=%.4f)\r\n", co[0], co[2]);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}
/**
 * @brief Verify FreeRTOS queue order and overflow behavior
 */
void RTOS_test_Queue(void)
{
	QueueHandle_t q = xQueueCreate(2, sizeof(int));
	if(q == NULL)
	{
		//printf("Queue test failed (create error)\r\n");
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		return;
	}

	int a=1,b=2,c=3,out;
	BaseType_t r1 = xQueueSend(q,&a,0);
	BaseType_t r2 = xQueueSend(q,&b,0);
	BaseType_t r3 = xQueueSend(q,&c,0);

	BaseType_t ok1 = xQueueReceive(q,&out,0);
	BaseType_t ok2 = xQueueReceive(q,&out,0);

	vQueueDelete(q);

	if(r1!=pdPASS || r2!=pdPASS || r3!=errQUEUE_FULL || ok1!=pdPASS || ok2!=pdPASS)
	{
		//printf("Queue test failed (send/recv error)\r\n");
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else
	{
		//printf("Queue test passed\r\n");
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}
/**
 * @brief Runs all unit tests on the DSP portions
 */
void DSP_test(void)
{
	RTOS_test_Queue();
	DSP_test_RBJ_Coeffs();
	test_PotMap();
}
