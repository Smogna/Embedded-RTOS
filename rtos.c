#include "main.h"
#include "stm32f1xx_hal.h"

// ---------- Define Task Structure ----------
typedef struct {
    int state;
    unsigned long period;
    unsigned long elapsedTime;
    int (*Function)(int);
} task;

// ---------- Function Declarations ----------
void SystemClock_Config(void);
void InitGPIO(void);
void TimerSet(unsigned long ms);
void TimerOn(void);
void TimerISR(void);

// ---------- Scheduler Constants ----------
const unsigned int numTasks = 2;
const unsigned long period = 100;           // Base tick (ms)
const unsigned long periodBlinkLED = 1500;
const unsigned long periodThreeLED = 500;

task tasks[2];

// ---------- Task States ----------
enum BL_states { BL0, BL1 };
int BlinkLED(int state);

enum TL_states { TL0, TL1, TL2 };
int ThreeLED(int state);

// ---------- Virtual Port Output ----------
uint8_t outputs_B = 0;

// ---------- HAL Handles ----------
TIM_HandleTypeDef htim2;

// ---------- Init GPIO ----------
void InitGPIO(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PA0, PA5, PA6, PA7 -> Output LEDs
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PC13 -> Onboard LED (active LOW)
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Set initial states
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED OFF
}

// ---------- Output Update Function ----------
void UpdateOutputs(uint8_t B) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (B & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (B & (1 << 5)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (B & (1 << 6)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (B & (1 << 7)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ---------- Timer ISR ----------
void TimerISR(void) {
    for (unsigned char i = 0; i < numTasks; i++) {
        if (tasks[i].elapsedTime >= tasks[i].period) {
            tasks[i].state = tasks[i].Function(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += period;
    }
}

// ---------- HAL SysTick Callback ----------
void HAL_SYSTICK_Callback(void) {
    TimerISR(); // Called every SysTick interrupt (every 1 ms)
}

// ---------- HAL Setup ----------
void TimerSet(unsigned long ms) {
    // SysTick already runs at 1 ms tick (from HAL_Init)
    // So we don’t need to configure it manually.
    // Just use the callback every ms.
}

void TimerOn(void) {
    // HAL handles SysTick automatically
    // We can just rely on HAL_SYSTICK_Callback()
}

// ---------- Task Implementations ----------
int BlinkLED(int state) {
    switch (state) {
        case BL0:
            outputs_B &= ~0x01; // Clear bit 0
            UpdateOutputs(outputs_B);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED ON
            state = BL1;
            break;

        case BL1:
            outputs_B |= 0x01;  // Set bit 0
            UpdateOutputs(outputs_B);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // LED OFF
            state = BL0;
            break;
    }
    return state;
}

int ThreeLED(int state) {
    switch (state) {
        case TL0:
            outputs_B = (outputs_B & 0x01) | 0x80;
            UpdateOutputs(outputs_B);
            state = TL1;
            break;
        case TL1:
            outputs_B = (outputs_B & 0x01) | 0x40;
            UpdateOutputs(outputs_B);
            state = TL2;
            break;
        case TL2:
            outputs_B = (outputs_B & 0x01) | 0x20;
            UpdateOutputs(outputs_B);
            state = TL0;
            break;
    }
    return state;
}

// ---------- Main ----------
int main(void) {
    HAL_Init();
    SystemClock_Config();
    InitGPIO();

    // Setup tasks
    tasks[0].state = BL0;
    tasks[0].period = periodBlinkLED;
    tasks[0].elapsedTime = tasks[0].period;
    tasks[0].Function = &BlinkLED;

    tasks[1].state = TL0;
    tasks[1].period = periodThreeLED;
    tasks[1].elapsedTime = tasks[1].period;
    tasks[1].Function = &ThreeLED;

    TimerSet(period);
    TimerOn();

    while (1) {
        // Idle loop (tasks handled in SysTick interrupt)
    }
}

// ---------- Basic System Clock Config ----------
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the CPU, AHB and APB busses clocks */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /** Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}
