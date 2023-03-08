/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);



  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);
  /* USER CODE BEGIN I2C1_Init 2 */
  LL_I2C_DisableBitPOS(I2C1);// Set POS low
  I2C1->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
  /* USER CODE END I2C1_Init 2 */

}
/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**I2C2 GPIO Configuration
  PB10   ------> I2C2_SCL
  PB11   ------> I2C2_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

  /* I2C2 interrupt Init */
  NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(I2C2_EV_IRQn);
  NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(I2C2_ER_IRQn);

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C2);
  LL_I2C_DisableGeneralCall(I2C2);
  LL_I2C_EnableClockStretching(I2C2);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C2, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C2, 0);
  /* USER CODE BEGIN I2C2_Init 2 */
  LL_I2C_DisableBitPOS(I2C2);// Set POS low
  I2C2->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
  /* USER CODE END I2C2_Init 2 */

}

/* USER CODE BEGIN 1 */
void MY_I2C1_Init(void)
{
    // Enable the I2C1 clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure the I2C1 pins (PB6 SCL, PB7 SDA)
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->CRL |= GPIO_CRL_MODE6 | GPIO_CRL_MODE7;
    GPIOB->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_CNF7);
    GPIOB->CRL |= GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1;

    // Configure the I2C1 registers
    I2C1->CR1 &= ~I2C_CR1_PE; // Disable the I2C1 peripheral
    I2C1->CR1 |= I2C_CR1_ACK; // Acknowledge enable
    I2C1->CR2 = 36; // Set the peripheral clock frequency to 72 MHz (72 MHz / 2)
    I2C1->CCR = 180; // Set the clock control register to 400 kHz with 72 MHz clock frequency
    I2C1->TRISE = 37; // Set the maximum rise time (1000 ns / (1 / 72 MHz) + 1)

    // Configure the I2C2 interrupts
    NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
    NVIC_EnableIRQ(I2C1_ER_IRQn);
    I2C1->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN); // Enable event interrupt

    // Enable the I2C2 peripheral
    I2C1->CR1 |= I2C_CR1_PE;
};

void MY_I2C2_Init(void)
{
    // Enable the I2C2 clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    // Configure the I2C2 pins (PB10 SCL, PB11 SDA)
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->CRH |= GPIO_CRH_MODE10 | GPIO_CRH_CNF10_1 |
                  GPIO_CRH_MODE11 | GPIO_CRH_CNF11_1;
    GPIOB->CRH &= ~(GPIO_CRH_CNF10_0 | GPIO_CRH_CNF11_0);

    // Configure the I2C2 registers
    I2C2->CR1 &= ~I2C_CR1_PE; // Disable the I2C2 peripheral
    I2C2->CR1 |= I2C_CR1_ACK; // Acknowledge enable
    I2C2->CR2 = 36; // Set the peripheral clock frequency to 72 MHz (72 MHz / 2)
    I2C2->CCR = 180; // Set the clock control register to 400 kHz with 72 MHz clock frequency
    I2C2->TRISE = 37; // Set the maximum rise time (1000 ns / (1 / 72 MHz) + 1)

    // Configure the I2C2 interrupts
    NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    NVIC_EnableIRQ(I2C2_EV_IRQn);
    NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
    NVIC_EnableIRQ(I2C2_ER_IRQn);
    I2C2->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN); // Enable event interrupt

    // Enable the I2C2 peripheral
    I2C2->CR1 |= I2C_CR1_PE;
}

/* USER CODE END 1 */
