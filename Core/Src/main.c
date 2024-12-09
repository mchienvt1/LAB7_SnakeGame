/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "software_timer.h"
#include "led_7seg.h"
#include "button.h"
#include "lcd.h"
#include "picture.h"
#include "ds3231.h"
#include "sensor.h"
#include "buzzer.h"
#include "touch.h"
#include "time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SNAKE_MAX_SIZE 50

#define X0 0
#define Y0 0
#define X1 4
#define Y1 4
#define X2 236
#define Y2 216
#define X3 240
#define Y3 219
#define RADIUS 5

struct Point
{
	uint16_t x, y, color;
};

enum Direction
{
	up,
	down,
	left,
	right
};

struct Snake
{
	struct Point node[SNAKE_MAX_SIZE];
	uint8_t len;
	enum Direction direction;
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t newNode = 1;
struct Snake snake;
struct Point food;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void system_init();

uint8_t isStartPressed();
uint8_t isUpButtonPressed();
uint8_t isDownButtonPressed();
uint8_t isLeftButtonPressed();
uint8_t isRightButtonPressed();

double calculateDistance(uint16_t, uint16_t, uint16_t, uint16_t);
void createFood();
void drawGameFrame();
void updateSnakePosition();
uint8_t checkCollision();
void handleFoodConsumption();
void initNode(uint8_t index, uint16_t x, uint16_t y, uint16_t color);
void initializeGame();
void display();
void displayStartScreen();
void handleInput();
uint8_t gameTick();
void play();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_SPI1_Init();
	MX_TIM2_Init();
	MX_FSMC_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_TIM13_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	system_init();
	//  touch_Adjust();
	lcd_Clear(BLACK);

	uint8_t isPlaying = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */


	while (1)
	{

		if (!isPlaying)
		{
			// draw start button
			lcd_Clear(WHITE);
//			play();
			displayStartScreen();
			isPlaying = 1;
		}
		else
		{
			// play game or not
			// read screen
			//&& isStartPress()
			touch_Scan();
			if (touch_IsTouched() && isStartPressed())
			{
				lcd_Clear(WHITE);
				play();
				isPlaying = 0;
			}
		}
		HAL_Delay(20);

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
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void system_init()
{
	timer_init();
	button_init();
	lcd_init();
	touch_init();
	setTimer2(10);
}

uint8_t count_led_debug = 0;

uint8_t isButtonClear()
{
	if (!touch_IsTouched())
		return 0;
	return touch_GetX() > 60 && touch_GetX() < 180 &&
				 touch_GetY() > 10 && touch_GetY() < 60;
}


uint8_t isStartPressed()
{
    // Kiểm tra nếu không có thao tác chạm, trả về 0
    if (!touch_IsTouched())
        return 0;

    // Lấy tọa độ cảm ứng
    uint16_t touchX = touch_GetX();
    uint16_t touchY = touch_GetY();

    // Kiểm tra tọa độ nằm trong phạm vi nút "Start"
    if (touchX > 40 && touchX < 200 && touchY > 100 && touchY < 220)
        return 1;

    return 0;
}

uint8_t isUpButtonPressed()
{
	return touch_GetX() >= 140 && touch_GetX() <= 180 &&
				 touch_GetY() >= 222 && touch_GetY() <= 262;
}

uint8_t isDownButtonPressed()
{
	return touch_GetX() >= 140 && touch_GetX() <= 180 &&
				 touch_GetY() >= 272 && touch_GetY() <= 312;
}

uint8_t isLeftButtonPressed()
{
	// 74 <= x <= 101
	// 270 <= y <= 297
	return touch_GetX() >= 90 && touch_GetX() <= 130 &&
				 touch_GetY() >= 272 && touch_GetY() <= 312;
}

uint8_t isRightButtonPressed()
{
	// 130 <= x <= 157
	// 270 <= y <= 297
	return touch_GetX() >= 190 && touch_GetX() <= 230 &&
				 touch_GetY() >= 272 && touch_GetY() <= 312;
}

double calculateDistance(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    // Tính toán hiệu giữa các tọa độ
    int16_t dx = x1 - x2;
    int16_t dy = y1 - y2;

    // Tính toán khoảng cách theo công thức Euclidean và trả về kết quả
    return sqrt(dx * dx + dy * dy);
}

void createFood()
{
    // Giới hạn vị trí ngẫu nhiên trong khoảng từ X1 + RADIUS đến X2 - RADIUS (để tránh bị ra ngoài khung)
    food.x = (rand() % (X2 - X1 - 2 * RADIUS + 1)) + (X1 + RADIUS);
    food.y = (rand() % (Y2 - Y1 - 2 * RADIUS + 1)) + (Y1 + RADIUS);
    // Gán màu cho con mồi
    food.color = RED;
}

void drawGameFrame()
{
    // Draw game boundary frame
    lcd_Fill(X0, Y0, X3, Y1, GRAY); // Top boundary
    lcd_Fill(X0, Y2, X3, Y3, GRAY); // Bottom boundary
    lcd_Fill(X0, Y0, X1, Y3, GRAY); // Left boundary
    lcd_Fill(X2, Y0, X3, Y3, GRAY); // Right boundary

    // Render  buttons
    lcd_Fill(140, 222, 180, 262, RED); // Up button
    lcd_Fill(140, 272, 180, 312, RED); // Down button
    lcd_Fill(90, 272, 130, 312, RED);  // Left button
    lcd_Fill(190, 272, 230, 312, RED); // Right button

    // Display the score label and initial value
    lcd_ShowStr(4, 222, "Score:", BLACK, WHITE, 16, 1); // "Score:" label
    lcd_ShowIntNum(50, 222, 0, 3, BLACK, WHITE, 16);    // Initial score value: 0
}

void initNode(uint8_t index, uint16_t x, uint16_t y, uint16_t color)
{
    snake.node[index].x = x;
    snake.node[index].y = y;
    snake.node[index].color = color;
}

void initializeGame()
{
    // Khởi tạo độ dài rắn và hướng di chuyển
    snake.len = 2;
    snake.direction = right;

    // Khởi tạo các node của rắn
    initNode(0, X1 + 3 * RADIUS + 2, Y1 + RADIUS + 1, BLACK); // Head node
    initNode(newNode, X1 + RADIUS + 1, Y1 + RADIUS + 1, WHITE); // Clear node

    // Khởi tạo con mồi
    createFood();
}

void display()
{
	// ve con ran
	for (uint8_t i = 0; i < snake.len; i++)
	{
		lcd_DrawCircle(snake.node[i].x, snake.node[i].y, snake.node[i].color, RADIUS, 1);
	}

	// ve con moi
	lcd_DrawCircle(food.x, food.y, food.color, RADIUS, 1);
}

void displayStartScreen() {
    lcd_Fill(40, 100, 200, 220, BLACK);
    lcd_ShowStr(90, 145, "START", WHITE, BLACK, 24, 1);
}

void handleInput()
{
	// nhan 4 nut dieu huong

	// doc man hinh
	touch_Scan();

	if (touch_IsTouched())
	{
		if (isUpButtonPressed() && snake.direction != down)
		{
			snake.direction = up;
		}
		else if (isDownButtonPressed() && snake.direction != up)
		{
			snake.direction = down;
		}
		else if (isLeftButtonPressed() && snake.direction != right)
		{
			snake.direction = left;
		}
		else if (isRightButtonPressed() && snake.direction != left)
		{
			snake.direction = right;
		}
	}
}

void updateSnakePosition()
{
    // Cập nhật vị trí các node của con rắn, di chuyển từ cuối lên đầu
    for (uint8_t i = snake.len - 1; i > 0; i--)
    {
        snake.node[i].x = snake.node[i - 1].x;	// Sao chép vị trí của node trước cho node hiện tại
        snake.node[i].y = snake.node[i - 1].y;
    }

    // Cập nhật vị trí đầu con rắn dựa trên hướng di chuyển
    int8_t deltaX = 0, deltaY = 0;

    switch (snake.direction)
    {
        case up:    deltaY = -(2 * RADIUS + 1); break;
        case down:  deltaY = (2 * RADIUS + 1); break;
        case left:  deltaX = -(2 * RADIUS + 1); break;
        case right: deltaX = (2 * RADIUS + 1); break;
        default:    break;
    }

    // Cập nhật vị trí đầu con rắn
    if(deltaX) snake.node[0].x += deltaX;
    if(deltaY) snake.node[0].y += deltaY;
}

uint8_t checkCollision() {
    // Kiểm tra va chạm với biên
    if (snake.node[0].x < X1 - 2 * RADIUS || snake.node[0].x > X2 + 2 * RADIUS ||
        snake.node[0].y < Y1 - 2 * RADIUS || snake.node[0].y > Y2 + 2 * RADIUS) {
        return 1; // Va chạm với biên
    }

    // Kiểm tra va chạm với chính mình (trừ nút cần xóa)
    for (uint8_t i = 1; i < snake.len - 1; i++) {
        if (snake.node[i].x == snake.node[0].x && snake.node[i].y == snake.node[0].y) {
            return 1; // Va chạm với chính mình
        }
    }

    return 0; // Không có va chạm
}

void handleFoodConsumption() {
    // Kiểm tra khoảng cách giữa đầu rắn và con mồi
    if (calculateDistance(snake.node[0].x, snake.node[0].y, food.x, food.y) < 2 * RADIUS) {
        // Tăng độ dài của rắn
        snake.len++;

        // Xóa con mồi cũ khỏi màn hình
        lcd_DrawCircle(food.x, food.y, WHITE, RADIUS, 1);
        snake.node[0].color = BLACK;

        // Cập nhật node cũ thành node thường
        snake.node[newNode].color = BLACK;

        // Tạo node mới cho rắn
        snake.node[snake.len - 1] = snake.node[newNode];
        snake.node[snake.len - 1].color = WHITE;

        // Cập nhật vị trí node mới dựa vào hướng di chuyển
        int16_t offset = 2 * RADIUS + 1; // Khoảng cách dịch chuyển
        switch (snake.direction) {
            case up:
                snake.node[snake.len - 1].y += offset;
                break;
            case down:
                snake.node[snake.len - 1].y -= offset;
                break;
            case left:
                snake.node[snake.len - 1].x += offset;
                break;
            case right:
                snake.node[snake.len - 1].x -= offset;
                break;
            default:
                break;
        }

        // Cập nhật chỉ số node mới
        newNode = snake.len - 1;

        // Hiển thị điểm số (số node trừ 2 để bỏ qua đầu và đuôi mặc định)
        lcd_ShowIntNum(50, 222, snake.len - 2, 3, BLUE, WHITE, 16);

        // Tạo con mồi mới
        createFood();
    }
}

uint8_t gameTick()
{
	updateSnakePosition();

	if(checkCollision()) return 1;

	handleFoodConsumption();


	return 0;
}

uint8_t counter = 50;

void play()
{
	// draw game frame
	drawGameFrame();

	//create snake with food
	initializeGame();
	while (1)
	{
		if (flag_timer2)
		{
			setTimer2(10);

			if (counter <= 0)
			{
				//draw snake with food
				display();
			}

			// dieu khien
			handleInput();

			if (counter <= 0)
			{
				counter = 50;
				//handle
				if (gameTick())
				{
					break;
				}
			}
			counter--;
		}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
