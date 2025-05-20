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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_ADC 4
#define MIN_BUFF 20
#define MAX_BUFF 80
#define KEY_INPUT 50
#define SAMPLE_ADC 64
#define MAX_ANGLE 5.25
#define STEP_DEGREE 0.75
#define STAGE_DELAY 100
#define MIN_QPD_X 10
#define MIN_QPD_Y 10
#define MIN_QPD_TOTAL 3000
#define MAX_KEY_INPUT 20000
#define CHECK_LED_OF GPIOB->BSRR = 0x10000000
#define CHECK_LED_ON GPIOB->BSRR = 0x00001000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum
{
    IDLE,
    INIT_X,
    INIT_Y,
    WAIT_X_OR,
    WAIT_Y_OR,
    CHECK_X_TP,
    CHECK_Y_TP,
    MOVE_X,
    MOVE_Y,
    WAIT_X_PA,
    WAIT_Y_PA,
    CALC_ANGLE,
    MOVE_DONE
} STAGE_STATE;

typedef enum
{
    QPD_ERR,
    QPD_LOW,
    QPD_HIGH
} QUAD_PD_STATE;

typedef enum
{
    STOP_MOVE,
    SEARCH_MOVE,
    CENTER_MOVE
} FCR100_STATE;

STAGE_STATE STG_STATE = IDLE, BKUP_STG_STATE = IDLE;
QUAD_PD_STATE QPD_STATE = QPD_ERR;
FCR100_STATE FCR_STATE = STOP_MOVE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Check_Auto_Manual(void);
void Check_Overrun_Uart(void);
void ADC_Process(void);
void Process_QPD(void);
void Process_Stage(void);
void Stage_Init(UART_HandleTypeDef *huart, const uint8_t *rxbuff);
void Stage_Moving(UART_HandleTypeDef *huart, const uint8_t *rxbuff);
void Stage_Calculate_Moving_Angle(void);
void Key_Input_Check(void);
void Send_Txbuff(UART_HandleTypeDef *huart, char *data);
bool Stage_Response_Check(const uint8_t *rxbuff, const char *compare);
bool Stage_Position_Check(const uint8_t *rxbuff, const char *compare);
void Send_Data_To_User(void);
void Send_Data_To_PC(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool ADC_Flag = false, INIT_CPLT = false, INIT_KEY = false,
     SEARCH_CPLT = false, CENTER_CPLT = false, AUTO = false, RUN = false;
char pc_buff[MAX_BUFF], stage_buff[MAX_BUFF], usr_buff[MAX_BUFF];
uint8_t rx_stdata, rx_pcdata, rx_xdata, rx_ydata,
    stnum, xnum, ynum, pcnum, stage_dir, SUB_INFO_1, SUB_INFO_2,
    rx_xbuff[MIN_BUFF], rx_ybuff[MIN_BUFF], rx_pcbuff[MIN_BUFF], rx_stbuff[MIN_BUFF];
uint16_t adc_value[SAMPLE_ADC * NUM_ADC], avg_adc[NUM_ADC], cal_adc[NUM_ADC],
    run_sw, stop_sw, init_sw,
    adc_cnt, sys_cnt, pc_cnt,
    stage_cnt, stage_step = 1, stg_delay = STAGE_DELAY,
               stage_move_cnt;
uint32_t cal_sum_adc[NUM_ADC], sum_adc[NUM_ADC];
int32_t qpd_y, qpd_x, qpd_total, sum_avg_adc;
double x_angle = 0.0, y_angle = 0.0, target_x_angle = 0.0, target_y_angle = 0.0,
       dmin = 0.0, dmax = 0.0, corr_x_angle = 0.0, corr_y_angle = 0.0;
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
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_UART4_Init();
    MX_UART5_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_DAC_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    /* USER CODE BEGIN 2 */

    // Check Mode(Auto|Manual)
    Check_Auto_Manual();

    // ADC Setup
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, SAMPLE_ADC * NUM_ADC);

    // Timer Interrupt Setup
    HAL_TIM_Base_Start_IT(&htim3);

    // UART Rx Interrupt Setup
    HAL_UART_Receive_IT(&huart1, &rx_stdata, 1);
    HAL_UART_Receive_IT(&huart2, &rx_pcdata, 1);
    HAL_UART_Receive_IT(&huart4, &rx_ydata, 1);
    HAL_UART_Receive_IT(&huart5, &rx_xdata, 1);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        // ADC Process
        ADC_Process();

        // Uart over-run check
        Check_Overrun_Uart();

        // Process QPD X-Y
        Process_QPD();

        // Find light source by moving Stage X-Y
        Process_Stage();

        // Send Data to PC
        Send_Data_To_PC();

        // Send Data to User
        Send_Data_To_User();
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
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Check Key(Run, Stop, Init)
    Key_Input_Check();

    // ADC
    ADC_Flag = true;

    // Stage Communication Delay
    if (stg_delay > 0)
        stg_delay--;

    // Send PC Data
    pc_cnt++;

    // Send User Data
    sys_cnt++;
}

void Check_Auto_Manual(void)
{
    if (GPIOC->IDR & 0x00000040) // Check Mode Auto-Manual(PC6)
    {
        Send_Txbuff(&huart1, "Manual Mode\r\n");
    }
    else
    {
        target_x_angle = 0.0;
        target_y_angle = 0.0;
        AUTO = true;
        RUN = true;
        STG_STATE = INIT_X;

        Send_Txbuff(&huart1, "Auto Mode\r\n");
    }
}

void Key_Input_Check(void)
{
    if (!(GPIOB->IDR & 0x00002000))
    { // Check input Init-SW(PB13)
        init_sw++;
        if (init_sw == KEY_INPUT)
        {
            // Move FCR100 to initial(home) position
            Send_Txbuff(&huart1, "init_sw input\r\n");

            target_x_angle = 0.0;
            target_y_angle = 0.0;
            INIT_KEY = true;
            STG_STATE = INIT_X;
        }
        if (init_sw >= MAX_KEY_INPUT)
            init_sw = 0;
    }
    else
    {
        if (init_sw > 0)
            init_sw = 0;
    }

    if (!(GPIOB->IDR & 0x00004000))
    { // Check input Stop-SW(PB14)
        stop_sw++;
        if (stop_sw == KEY_INPUT)
        {
            // Send stop command to FCR100
            Send_Txbuff(&huart1, "stop_sw input\r\n");

            RUN = false;

            // Turn On CHECK-LED
            CHECK_LED_ON;
        }
        if (stop_sw >= MAX_KEY_INPUT)
            stop_sw = 0;
    }
    else
    {
        if (stop_sw > 0)
            stop_sw = 0;
    }

    if (!(GPIOB->IDR & 0x00008000))
    { // Check input Run-SW(PB15)
        run_sw++;
        if (run_sw == KEY_INPUT)
        {
            // Send command to find and move to center position
            Send_Txbuff(&huart1, "run_sw input\r\n");

            RUN = true;

            // Turn Off CHECK-LED
            CHECK_LED_OF;
        }
        if (run_sw >= MAX_KEY_INPUT)
            run_sw = 0;
    }
    else
    {
        if (run_sw > 0)
            run_sw = 0;
    }
}

void ADC_Process(void)
{
    if (ADC_Flag)
    {
        adc_cnt++;

        for (uint8_t i = 0; i < SAMPLE_ADC; i++)
        {
            cal_sum_adc[0] += adc_value[i * 4];
            cal_sum_adc[1] += adc_value[(i * 4) + 1];
            cal_sum_adc[2] += adc_value[(i * 4) + 2];
            cal_sum_adc[3] += adc_value[(i * 4) + 3];
        }

        for (uint8_t i = 0; i < NUM_ADC; i++)
        {
            cal_adc[i] = cal_sum_adc[i] / SAMPLE_ADC;
        }

        for (uint8_t i = 0; i < NUM_ADC; i++)
        {
            cal_sum_adc[i] = 0;
        }

        for (uint8_t i = 0; i < NUM_ADC; i++)
        {
            sum_adc[i] += cal_adc[i];
        }

        if (adc_cnt >= 300)
        {
            for (uint8_t i = 0; i < NUM_ADC; i++)
            {
                avg_adc[i] = sum_adc[i] / 300;
                sum_avg_adc += avg_adc[i];
            }

            for (uint8_t i = 0; i < NUM_ADC; i++)
            {
                sum_adc[i] = 0;
            }

            qpd_total = (int32_t)sum_avg_adc;
            sum_avg_adc = 0;

            if (qpd_total > 0)
            {
                // X = ((A + D) - (B + C)) / (A + B + C + D), 단위 0.00%(예, 300 -> 3.99%)
                qpd_x = (avg_adc[0] + avg_adc[3]) - (avg_adc[1] + avg_adc[2]);
                qpd_x *= 10000;
                qpd_x = qpd_x / qpd_total;

                // Y = ((A + B) - (C + D)) / (A + B + C + D)
                qpd_y = (avg_adc[0] + avg_adc[1]) - (avg_adc[2] + avg_adc[3]);
                qpd_y *= 10000;
                qpd_y = qpd_y / qpd_total;

                // 100%(10,000) 이상인 경우, 축 치우침이 너무 큼으로 광소스를 다시 찾아야???
                // 우선, X, Y값을 100%로 설정(판별은 QPD_PROCESS에서)
                if (qpd_x >= 10000)
                    qpd_x = 10000;
                else if (qpd_x <= -10000)
                    qpd_x = -10000;

                if (qpd_y >= 10000)
                    qpd_y = 10000;
                else if (qpd_y <= -10000)
                    qpd_y = -10000;
            }
            adc_cnt = 0;
        }

        ADC_Flag = false;
    }
}

void Check_Overrun_Uart(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
    {
        Send_Txbuff(&huart1, "Errupt USART1(ST-LINK) OVER-source_search\r\n");

        volatile uint32_t temp;
        temp = USART1->DR; // Read Rx-Register
        (void)temp;        // Prevent Compiler error

        HAL_UART_Receive_IT(&huart1, &rx_stdata, 1);
    }

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
    {
        Send_Txbuff(&huart1, "Errupt USART2(PC) OVER-source_search\r\n");

        volatile uint32_t temp;
        temp = USART2->DR; // Read Rx-Register
        (void)temp;        // Prevent Compiler error

        HAL_UART_Receive_IT(&huart2, &rx_pcdata, 1);
    }

    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE))
    {
        Send_Txbuff(&huart1, "Errupt UART4(FCR-Y) OVER-source_search\r\n");

        volatile uint32_t temp;
        temp = UART4->DR; // Read Rx-Register
        (void)temp;       // Prevent Compiler error

        HAL_UART_Receive_IT(&huart4, &rx_ydata, 1);
    }

    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_ORE))
    {
        Send_Txbuff(&huart1, "Errupt UART5(FCR-X) OVER-source_search\r\n");

        volatile uint32_t temp;
        temp = UART5->DR; // Read Rx-Register
        (void)temp;       // Prevent Compiler error

        HAL_UART_Receive_IT(&huart5, &rx_xdata, 1);
    }
}

void Process_QPD(void)
{
    // 임시 리턴 <- 코드 구성 후 삭제
    return;

    // QPD 값이 최소값보다 작은 경우, 계속 찾기
    if (qpd_total < MIN_QPD_TOTAL)
    {
        SEARCH_CPLT = false;
        QPD_STATE = QPD_ERR;
        return;
    }

    // QPD 중심점 이동 모드
    SEARCH_CPLT = true;
    STG_STATE = MOVE_DONE;

    if (abs(qpd_x) > MIN_QPD_X)
    {
        // X축 미세 이동(qpd_x는 x100한 %값으로 현재 angle에 .2f% 만큼 치우침 표시)
        // qpd_x 가 123인 경우, x가 중심점으로 부터 1.23%(+ 우축방향으로) 치우침을 말함.
        // 중심점으로 이동시키기위해 x_angle을 1.23%만큼 (- 좌측방향으로) 이동시켜 줘야 함.
        // 예) x_angle이 3.75(도)라면, x_angle에서 3.75(도)의 1.23% 에 해당하는 값을 뺀 후,
        // 목표각(target_x_angle)으로 설정.(만약 값이 (-)음수이면 -- -> + 로 교정됨)

        corr_x_angle = (x_angle * qpd_x) / 100.0f;
        corr_x_angle /= 4.0f;
        target_x_angle = x_angle - corr_x_angle;
        STG_STATE = MOVE_X;
        QPD_STATE = QPD_LOW;
    }
    else if (abs(qpd_y) > MIN_QPD_Y)
    {
        // Y축 미세 이동
        corr_y_angle = (x_angle * qpd_x) / 100.0f;
        corr_y_angle /= 4.0f;
        target_y_angle = y_angle - corr_y_angle;
        STG_STATE = MOVE_Y;
        QPD_STATE = QPD_LOW;
    }
    else
    {
        // 중심점 이동 완료
        STG_STATE = MOVE_DONE;
        QPD_STATE = QPD_HIGH;
    }
}

void Process_Stage(void)
{
    if (stg_delay != 0)
        return;

    // 실제테스트 시,
    // &huart1 -> &huart4
    // rx_stbuff -> rx_ybuff

    if (!INIT_CPLT)
    {
        FCR_STATE = STOP_MOVE;
        Stage_Init(&huart4, rx_ybuff);
    }
    else
    {
        // FCR100 상태 설정
        if (!SEARCH_CPLT)
            FCR_STATE = SEARCH_MOVE;
        else if (!CENTER_CPLT)
            FCR_STATE = CENTER_MOVE;
        else
            FCR_STATE = STOP_MOVE;

        Stage_Moving(&huart4, rx_ybuff);
    }
}

void Stage_Init(UART_HandleTypeDef *huart, const uint8_t *rxbuff)
{
    if (!AUTO && !INIT_KEY)
        return;

    switch (STG_STATE)
    {
    case INIT_X:
        STG_STATE = WAIT_X_OR;
        Send_Txbuff(huart, "1OR\r\n");
        break;

    case WAIT_X_OR:
        if (Stage_Response_Check(rxbuff, "1OR"))
        {
            STG_STATE = CHECK_X_TP;
            Send_Txbuff(huart, "1TP\r\n");
        }
        else
            STG_STATE = INIT_X;
        break;

    case CHECK_X_TP:
        if (Stage_Position_Check(rxbuff, "1TP"))
            STG_STATE = INIT_Y;
        else
            STG_STATE = MOVE_X;
        break;

    case MOVE_X:
        STG_STATE = WAIT_X_PA;
        snprintf(stage_buff, MAX_BUFF, "1PA%.6f\r\n", target_x_angle);
        Send_Txbuff(huart, stage_buff);
        break;

    case WAIT_X_PA:
        if (Stage_Response_Check(rxbuff, "1PA"))
        {
            STG_STATE = CHECK_X_TP;
            Send_Txbuff(huart, "1TP\r\n");
        }
        else
            STG_STATE = MOVE_X;
        break;

    case INIT_Y:
        STG_STATE = WAIT_Y_OR;
        Send_Txbuff(huart, "2OR\r\n");
        break;

    case WAIT_Y_OR:
        if (Stage_Response_Check(rxbuff, "2OR"))
        {
            STG_STATE = CHECK_Y_TP;
            Send_Txbuff(huart, "2TP\r\n");
        }
        else
            STG_STATE = INIT_Y;
        break;

    case CHECK_Y_TP:
        if (Stage_Position_Check(rxbuff, "2TP"))
            STG_STATE = MOVE_DONE;
        else
            STG_STATE = MOVE_Y;
        break;

    case MOVE_Y:
        STG_STATE = WAIT_Y_PA;
        snprintf(stage_buff, MAX_BUFF, "2PA%.6f\r\n", target_y_angle);
        Send_Txbuff(huart, stage_buff);
        break;

    case WAIT_Y_PA:
        if (Stage_Response_Check(rxbuff, "2PA"))
        {
            STG_STATE = CHECK_Y_TP;
            Send_Txbuff(huart, "2TP\r\n");
        }
        else
            STG_STATE = MOVE_Y;
        break;

    case MOVE_DONE:
        INIT_CPLT = true;
        STG_STATE = CALC_ANGLE;
        break;

    case IDLE:
    case CALC_ANGLE:
        break;

    default:
        break;
    }
}

void Stage_Moving(UART_HandleTypeDef *huart, const uint8_t *rxbuff)
{
    if (!RUN)
        return;

    switch (STG_STATE)
    {
    case CALC_ANGLE:
        Stage_Calculate_Moving_Angle();
        break;

    case MOVE_X:
        STG_STATE = WAIT_X_PA;
        snprintf(stage_buff, MAX_BUFF, "1PA%.6f\r\n", target_x_angle);
        Send_Txbuff(huart, stage_buff);
        break;

    case WAIT_X_PA:
        if (Stage_Response_Check(rxbuff, "1PA"))
        {
            STG_STATE = CHECK_X_TP;
            Send_Txbuff(huart, "1TP\r\n");
        }
        else
            STG_STATE = MOVE_X;
        break;

    case CHECK_X_TP:
        if (Stage_Position_Check(rxbuff, "1TP"))
        {
            if (!SEARCH_CPLT)
                STG_STATE = CALC_ANGLE;
            else
                STG_STATE = MOVE_DONE;
        }
        else
            STG_STATE = MOVE_X;
        break;

    case MOVE_Y:
        STG_STATE = WAIT_Y_PA;
        snprintf(stage_buff, MAX_BUFF, "2PA%.6f\r\n", target_y_angle);
        Send_Txbuff(huart, stage_buff);
        break;

    case WAIT_Y_PA:
        if (Stage_Response_Check(rxbuff, "2PA"))
        {
            STG_STATE = CHECK_Y_TP;
            Send_Txbuff(huart, "2TP\r\n");
        }
        else
            STG_STATE = MOVE_Y;
        break;

    case CHECK_Y_TP:
        if (Stage_Position_Check(rxbuff, "2TP"))
        {
            if (!SEARCH_CPLT)
                STG_STATE = CALC_ANGLE;
            else
                STG_STATE = MOVE_DONE;
        }
        else
            STG_STATE = MOVE_Y;
        break;

    case IDLE:
    case INIT_X:
    case INIT_Y:
    case WAIT_X_OR:
    case WAIT_Y_OR:
    case MOVE_DONE:
        break;

    default:
        break;
    }
}

bool Stage_Response_Check(const uint8_t *rxbuff, const char *compare)
{
    if (strncmp((char *)rxbuff, compare, 3) == 0)
    {
        return true;
    }
    else
    {
        snprintf(stage_buff, MAX_BUFF, "<RX> %s != <CMP> %s\r\n", rxbuff, compare);
        Send_Txbuff(&huart1, stage_buff);
        return false;
    }
}

bool Stage_Position_Check(const uint8_t *rxbuff, const char *compare)
{
    if (strncmp((char *)rxbuff, compare, 3) == 0)
    {
        bool is_negative = false;
        int i = 3;

        // Check Sign
        if (rxbuff[i] == '-')
        {
            is_negative = true;
            i++;
        }

        double angle = 0.0;
        bool decimal_part = false;
        double decimal_place = 0.1;

        while (rxbuff[i] != 0x0D)
        {
            char ch = rxbuff[i];

            if (ch >= '0' && ch <= '9')
            {
                if (!decimal_part)
                {
                    angle = angle * 10 + (ch - '0');
                }
                else
                {
                    angle += (ch - '0') * decimal_place;
                    decimal_place *= 0.1;
                }
            }
            else if (ch == '.')
                decimal_part = true;
            else
                break;

            i++;
        }

        if (is_negative)
            angle = -angle;

        if (rxbuff[0] == '1')
        {
            x_angle = angle;

            dmin = target_x_angle - 0.00025f;
            dmax = target_x_angle + 0.00025f;

            if (!(dmin < x_angle && dmax > x_angle))
            {
                snprintf(stage_buff, MAX_BUFF, "x_angle: %+09.6f (target: %+09.6f)\r\n",
                         x_angle, target_x_angle);
                Send_Txbuff(&huart1, stage_buff);
                return false;
            }
            else
            {
                return true;
            }
        }
        else if (rxbuff[0] == '2')
        {
            y_angle = angle;

            dmin = target_y_angle - 0.00025f;
            dmax = target_y_angle + 0.00025f;

            if (!(dmin < y_angle && dmax > y_angle))
            {
                snprintf(stage_buff, MAX_BUFF, "y_angle: %+09.6f (target: %+09.6f)\r\n",
                         y_angle, target_y_angle);
                Send_Txbuff(&huart1, stage_buff);
                return false;
            }
            else
            {
                return true;
            }
        }
    }

    snprintf(stage_buff, MAX_BUFF, "<RX> %s !=<CMP> %s\r\n", rxbuff, compare);
    Send_Txbuff(&huart1, stage_buff);
    return false;
}

void Stage_Calculate_Moving_Angle(void)
{
    if (fabs(target_y_angle) >= MAX_ANGLE && fabs(y_angle) >= MAX_ANGLE)
    {
        target_x_angle = 0.0;
        target_y_angle = 0.0;

        stage_step = 1;
        stage_dir = 0;
        stage_cnt = 0;

        // 이동 단계 총 갯수 클리어, 추후 삭세
        stage_move_cnt = 0;
    }

    // 이동 단계 카운트, 추후 삭제(변수도 삭제 필요)
    stage_move_cnt++;

    switch (stage_dir % 4)
    {
    case 0:
        target_x_angle += STEP_DEGREE; // +x
        STG_STATE = MOVE_X;
        break;
    case 1:
        target_y_angle += STEP_DEGREE; // +y
        STG_STATE = MOVE_Y;
        break;
    case 2:
        target_x_angle -= STEP_DEGREE; // -x
        STG_STATE = MOVE_X;
        break;
    case 3:
        target_y_angle -= STEP_DEGREE; // -y
        STG_STATE = MOVE_Y;
        break;

    default:
        break;
    }

    stage_cnt++;
    if (stage_cnt >= stage_step)
    {
        stage_cnt = 0;
        stage_dir++;

        if (stage_dir % 2 == 0)
            stage_step++;
    }
}

void Send_Data_To_PC(void)
{
    switch (pc_cnt)
    {
    case 10:
        snprintf(pc_buff, MAX_BUFF, "QA%05d\r\n", avg_adc[0]);
        Send_Txbuff(&huart2, pc_buff);
        break;

    case 30:
        snprintf(pc_buff, MAX_BUFF, "QB%05d\r\n", avg_adc[1]);
        Send_Txbuff(&huart2, pc_buff);
        break;

    case 50:
        snprintf(pc_buff, MAX_BUFF, "QC%05d\r\n", avg_adc[2]);
        Send_Txbuff(&huart2, pc_buff);
        break;

    case 70:
        snprintf(pc_buff, MAX_BUFF, "QD%05d\r\n", avg_adc[3]);
        Send_Txbuff(&huart2, pc_buff);
        break;

    case 90:
        if (x_angle < 0)
            snprintf(pc_buff, MAX_BUFF, "FX-%09.6f\r\n", fabs(x_angle));
        else
            snprintf(pc_buff, MAX_BUFF, "FX+%09.6f\r\n", x_angle);
        Send_Txbuff(&huart2, pc_buff);
        break;

    case 110:
        if (y_angle < 0)
            snprintf(pc_buff, MAX_BUFF, "FY-%09.6f\r\n", fabs(y_angle));
        else
            snprintf(pc_buff, MAX_BUFF, "FY+%09.6f\r\n", y_angle);
        Send_Txbuff(&huart2, pc_buff);
        break;

    case 130:
        snprintf(pc_buff, MAX_BUFF, "S%d%d%d%d\r\n",
                 QPD_STATE, FCR_STATE, SUB_INFO_1, SUB_INFO_2);
        Send_Txbuff(&huart2, pc_buff);
        break;

    case 150:
        pc_cnt = 0;
        break;

    default:
        break;
    }
}

void Send_Data_To_User(void)
{
    switch (sys_cnt)
    {
    case 1000:
        snprintf(usr_buff, MAX_BUFF, "QPD A: %4d, B: %4d, C: %4d, D: %4d\r\nSTAGE X: %+09.6f, Y: %+09.6f\r\n",
                 avg_adc[0], avg_adc[1], avg_adc[2], avg_adc[3], x_angle, y_angle);
        // Send_Txbuff(&huart1, usr_buff);
        sys_cnt = 0;
        break;

    default:
        break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    { // ST-LINK
        rx_stbuff[stnum] = rx_stdata;
        stnum++;

        if (stnum >= MIN_BUFF)
            stnum = 0;

        if (stnum > 2 && rx_stbuff[stnum - 2] == 0x0D && rx_stbuff[stnum - 1] == 0x0A)
        {
            stnum = 0;
            // 테스트용
            // stg_delay = 10;
        }

        HAL_UART_Receive_IT(&huart1, &rx_stdata, 1);
    }

    if (huart->Instance == USART2)
    { // PC
        rx_pcbuff[pcnum] = rx_pcdata;
        pcnum++;

        if (pcnum >= MIN_BUFF)
            pcnum = 0;

        if (pcnum > 2 && rx_pcbuff[pcnum - 2] == 0x0D && rx_pcbuff[pcnum - 1] == 0x0A)
        {
            pcnum = 0;
        }

        HAL_UART_Receive_IT(&huart2, &rx_pcdata, 1);
    }

    if (huart->Instance == UART4)
    { // FCR100 Y-AXIS (UART4 DMA Available)
        rx_ybuff[ynum] = rx_ydata;
        ynum++;

        if (ynum >= MIN_BUFF)
            ynum = 0;

        if (ynum > 2 && rx_ybuff[ynum - 2] == 0x0D && rx_ybuff[ynum - 1] == 0x0A)
        {
            ynum = 0;
            stg_delay = 10;
        }

        HAL_UART_Receive_IT(&huart4, &rx_ydata, 1);
    }

    if (huart->Instance == UART5)
    { // FCR100 X-AXIS
        rx_xbuff[xnum] = rx_xdata;
        xnum++;

        if (xnum >= MIN_BUFF)
            xnum = 0;

        if (xnum > 2 && rx_xbuff[xnum - 2] == 0x0D && rx_xbuff[xnum - 1] == 0x0A)
        {
            xnum = 0;
        }

        HAL_UART_Receive_IT(&huart5, &rx_xdata, 1);
    }
}

void Send_Txbuff(UART_HandleTypeDef *huart, char *data)
{
    if (huart->Instance == USART1)
    {
        // 테스트용
        // stg_delay = STAGE_DELAY;
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data, strlen(data));
    }

    if (huart->Instance == USART2)
    {
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)data, strlen(data));
    }

    if (huart->Instance == UART4)
    {
        stg_delay = STAGE_DELAY;
        HAL_UART_Transmit_DMA(&huart4, (uint8_t *)data, strlen(data));
    }

    if (huart->Instance == UART5)
    {
        HAL_UART_Transmit(&huart5, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __NOP();
    }

    if (huart->Instance == USART2)
    {
        __NOP();
    }

    if (huart->Instance == UART4)
    {
        __NOP();
    }

    if (huart->Instance == UART5)
    {
        __NOP();
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

#ifdef USE_FULL_ASSERT
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
