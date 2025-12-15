/* USER CODE BEGIN Header */
/**
  * PROJETO PI: ESTACIONAMENTO INTELIGENTE (Versão FINAL - GOLD)
  * * Hardware:
  * - STM32 BluePill (F103C8T6)
  * - Expansor PCF8575 (Endereço 0x42) -> 4 Pares de LED/Sensor
  * - LCD 16x2 I2C (Endereço 0x27)
  * - Biometria AS608 (UART2 - PA2/PA3 - 9600 Baud)
  * - Servo Motor SG90 (PWM TIM3 CH1 - PA6)
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include "as608.h" // Certifique-se que as608.h e as608.c estão no projeto

// --- CONFIGURAÇÕES DO SISTEMA ---
#define PCF_ADDR        0x42         // Endereço do Expansor (A0 no VCC)
#define LCD_ADDR        (0x27 << 1)  // Endereço do LCD
#define N_VAGAS         4            // Quantidade de vagas

// --- AJUSTE DO SERVO MOTOR ---
// Valores ajustados para evitar pico de corrente excessivo
#define SERVO_FECHADO   500   // 0 Graus
#define SERVO_ABERTO    1500  // 90 Graus

// Variáveis Globais
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

uint8_t leds_state = 0xFF; // Estado inicial dos LEDs (Apagados/Lógica Inversa)

/* --- PROTÓTIPOS DE FUNÇÃO --- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

// Funções de Controle do Projeto
int PCF_Read_Safe(uint16_t *val);
void PCF_Write_Safe(uint16_t val);
void Atualizar_Logica_Vaga(int vaga, int ocupado);
void I2C_Reset(void);
void Mover_Cancela(uint16_t posicao);

// Drivers LCD I2C
void LCD_Init(void);
void LCD_SendString(char *str);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Command(uint8_t cmd);
void LCD_Data(uint8_t data);
void LCD_Nibble(uint8_t nib, uint8_t rs);

// --- FUNÇÃO MAIN (O CÉREBRO) ---
int main(void)
{
  // 1. Inicialização do HAL (Hardware Abstraction Layer)
  HAL_Init();
  SystemClock_Config();

  // 2. Configuração dos Periféricos
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  // --- CONFIGURAÇÃO MANUAL DO PINO PA6 (GARANTIA DE FUNCIONAMENTO) ---
  // Força o pino PA6 a funcionar como saída do Timer 3 (PWM)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // ------------------------------------------------------------------

  HAL_Delay(1000); // Aguarda energia estabilizar (capacitor carregar)

  // 3. Inicializa Expansor e LCD
  PCF_Write_Safe(0xFFFF); // Reseta PCF
  LCD_Init();

  LCD_SetCursor(0, 0);
  LCD_SendString("TESTE SISTEMA...");

  // 4. TESTE INICIAL DA CANCELA (Abre e Fecha ao ligar)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  Mover_Cancela(SERVO_ABERTO);
  HAL_Delay(1500);

  Mover_Cancela(SERVO_FECHADO);
  HAL_Delay(1500);

  // 5. Sistema Pronto
  LCD_Init();
  LCD_SetCursor(0, 0);
  LCD_SendString("ESTACIONAMENTO");
  LCD_SetCursor(1, 0);
  LCD_SendString("INICIADO OK!");
  HAL_Delay(1000);

  // Acende LEDs iniciais
  PCF_Write_Safe(0xFF00 | leds_state);

  // --- LOOP INFINITO ---
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Pisca LED da placa (Heartbeat)

    /* ===================================================
       PARTE A: CONTROLE DE ACESSO (BIOMETRIA + CANCELA)
       =================================================== */
    int fingerID = 999;

    // Verifica se há um dedo no sensor
    if (AS608_CheckFinger(&huart2, &fingerID) == 0) {

        // Filtro: Aceita apenas IDs válidos (1 a 200) para evitar ruído
        if (fingerID > 0 && fingerID <= 200) {

            // 1. Mostra Mensagem
            LCD_Init();
            LCD_SetCursor(0, 0); LCD_SendString("BEM VINDO!");

            char msg[17];
            sprintf(msg, "ID: %d", fingerID);
            LCD_SetCursor(1, 0); LCD_SendString(msg);

            // 2. Abre Cancela
            Mover_Cancela(SERVO_ABERTO);
            HAL_Delay(3000); // Mantém aberta por 3 segundos

            // 3. Fecha Cancela
            Mover_Cancela(SERVO_FECHADO);

            // 4. Restaura Tela
            LCD_Init();
            LCD_SetCursor(0, 0); LCD_SendString("ESTACIONAMENTO");
        }
    }

    /* ===================================================
       PARTE B: CONTROLE DE VAGAS (SENSORES + LEDS)
       =================================================== */
    uint16_t leitura = 0xFFFF;

    // Tenta ler o PCF. Se falhar, reinicia I2C e pula o loop
    if (PCF_Read_Safe(&leitura) == 0) {
        I2C_Reset();
        continue;
    }

    uint8_t sensores = (leitura >> 8); // Parte Alta (P10-P13)
    int vagas_livres = 0;

    // Varre as 4 vagas
    for (int i = 0; i < N_VAGAS; i++) {
        // Verifica bit: 1 = Livre (Luz apagada/Sem reflexo), 0 = Ocupado (Reflexo)
        int is_livre = (sensores >> i) & 0x01;

        if (is_livre) {
            Atualizar_Logica_Vaga(i, 0); // 0 = Livre (Azul)
            vagas_livres++;
        } else {
            Atualizar_Logica_Vaga(i, 1); // 1 = Ocupado (Vermelho)
        }
    }

    // Atualiza os LEDs físicos no PCF
    PCF_Write_Safe(0xFF00 | leds_state);

    // Atualiza contagem no LCD (apenas se não estiver processando biometria)
    char buf[17];
    sprintf(buf, "Vagas: %d/%d      ", vagas_livres, N_VAGAS);
    LCD_SetCursor(1, 0);
    LCD_SendString(buf);

    HAL_Delay(50); // Pequeno delay para estabilidade
  }
}

/* --- IMPLEMENTAÇÃO DAS FUNÇÕES --- */

// Move o servo via PWM
void Mover_Cancela(uint16_t posicao) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, posicao);
}

// Atualiza a variável de estado dos LEDs (Azul/Vermelho)
void Atualizar_Logica_Vaga(int vaga, int ocupado) {
    int p_azul = vaga * 2;       // Ex: Vaga 0 -> P0
    int p_verm = (vaga * 2) + 1; // Ex: Vaga 0 -> P1

    if (ocupado) {
        leds_state |= (1 << p_azul);  // Desliga Azul (HIGH)
        leds_state &= ~(1 << p_verm); // Liga Vermelho (LOW)
    } else {
        leds_state &= ~(1 << p_azul); // Liga Azul (LOW)
        leds_state |= (1 << p_verm);  // Desliga Vermelho (HIGH)
    }
}

// Leitura I2C com verificação de erro
int PCF_Read_Safe(uint16_t *val) {
    uint8_t data[2];
    if (HAL_I2C_Master_Receive(&hi2c1, PCF_ADDR, data, 2, 20) == HAL_OK) {
        *val = (data[1] << 8) | data[0];
        return 1; // Sucesso
    }
    return 0; // Falha
}

// Escrita I2C
void PCF_Write_Safe(uint16_t val) {
    uint8_t data[2] = {val & 0xFF, (val >> 8) & 0xFF};
    HAL_I2C_Master_Transmit(&hi2c1, PCF_ADDR, data, 2, 20);
}

// Recuperação de falha I2C
void I2C_Reset(void) {
    HAL_I2C_DeInit(&hi2c1);
    HAL_Delay(5);
    MX_I2C1_Init();
}

// --- DRIVER LCD 16x2 I2C ---
void LCD_Nibble(uint8_t nib, uint8_t rs) {
    uint8_t pkg[2];
    uint8_t d = (nib & 0xF0) | (rs ? 1:0) | 0x08; // Backlight ON (0x08)
    pkg[0] = d | 0x04; // Enable High
    pkg[1] = d & ~0x04; // Enable Low
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, pkg, 2, 10);
}
void LCD_Command(uint8_t cmd) { LCD_Nibble(cmd & 0xF0, 0); LCD_Nibble((cmd << 4) & 0xF0, 0); }
void LCD_Data(uint8_t data) { LCD_Nibble(data & 0xF0, 1); LCD_Nibble((data << 4) & 0xF0, 1); }
void LCD_Init(void) {
    HAL_Delay(50);
    LCD_Nibble(0x30, 0); HAL_Delay(5); LCD_Nibble(0x30, 0); HAL_Delay(1);
    LCD_Nibble(0x30, 0); HAL_Delay(1); LCD_Nibble(0x20, 0); HAL_Delay(1);
    LCD_Command(0x28); LCD_Command(0x0C); LCD_Command(0x06); LCD_Command(0x01); HAL_Delay(2);
}
void LCD_SendString(char *str) { while(*str) LCD_Data(*str++); }
void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x80 : 0xC0;
    LCD_Command(addr + col);
}

// --- CONFIGURAÇÕES DE HARDWARE (Geradas pelo CubeMX) ---

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1; RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}
static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1; hi2c1.Init.ClockSpeed = 100000; hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0; hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
}
static void MX_TIM3_Init(void) {
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim3.Instance = TIM3; htim3.Init.Prescaler = 71; htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999; htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim3);
  sConfigOC.OCMode = TIM_OCMODE_PWM1; sConfigOC.Pulse = 500; sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  // NOTA: A configuração do pino PA6 foi movida para dentro do main() para garantir o funcionamento.
}
static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600; //
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1; huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX; huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  HAL_UART_Init(&huart2);
}
static void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOD_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0}; __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_13; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
void Error_Handler(void) { __disable_irq(); while (1) {} }
