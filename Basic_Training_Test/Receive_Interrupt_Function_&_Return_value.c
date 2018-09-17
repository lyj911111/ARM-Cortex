/*
	Receive 값을 받을때, Return값을 활용할것.
	Ctrl + 마우스 하면, 함수의 리턴값을 확인 할 수 있다.
*/

int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t uart_buf[2];
	//uint32_t count = 0;
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
  MX_USART3_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE END WHILE */

////////////////////////////////////////////////////////////////////////
	  if( (HAL_UART_Receive_IT(&huart3, uart_buf, 2)) == HAL_OK )
	  {
		   HAL_UART_Transmit_IT(&huart3, uart_buf, 2);
		   //memset(uart_buf, 0, sizeof(uart_buf));
		   HAL_Delay(1000);
	  }
/////////////////////////////////////////////////////////////////////////
	  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}