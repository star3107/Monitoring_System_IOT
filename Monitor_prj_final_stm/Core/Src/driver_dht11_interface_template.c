/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_dht11_interface_template.c
 * @brief     driver dht11 interface template source file
 * @version   2.0.0
 * @author    Shifeng Li
 * @date      2021-03-12
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/03/12  <td>2.0      <td>Shifeng Li  <td>format the code
 * <tr><td>2020/11/19  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "main.h"
#include "core_cm4.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static GPIO_InitTypeDef dht_sig = {0};
extern UART_HandleTypeDef huart2;
// Enable DWT counter (do this once in your initialization)
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

void delay_us_80mhz(uint32_t us) {
    uint32_t start_tick = DWT->CYCCNT;
    uint32_t delay_ticks = us * 80;  // 80 cycles per microsecond

    while ((DWT->CYCCNT - start_tick) < delay_ticks) {
        // Busy wait - precise timing
    }
}

// Optimized millisecond delay for 80MHz
// At 80MHz: 1ms = 80,000 clock cycles
void delay_ms_80mhz(uint32_t ms) {
    uint32_t start_tick = DWT->CYCCNT;
    uint32_t delay_ticks = ms * 80000;  // 80,000 cycles per millisecond

    while ((DWT->CYCCNT - start_tick) < delay_ticks) {
        // Busy wait - precise timing
    }
}

/**
 * @brief  interface bus init
 * @return status code
 *         - 0 success
 *         - 1 bus init failed
 * @note   none
 */
uint8_t dht11_interface_init(void)
{
	//Configure GPIO PC10 as signal input
	__HAL_RCC_GPIOC_CLK_ENABLE();
	dht_sig.Pull = GPIO_PULLUP;
	dht_sig.Speed = GPIO_SPEED_FREQ_LOW;
	dht_sig.Pin = GPIO_PIN_10;
	dht_sig.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOC, &dht_sig);
	DWT_Init();
    return 0;
}

/**
 * @brief  interface bus deinit
 * @return status code
 *         - 0 success
 *         - 1 bus deinit failed
 * @note   none
 */
uint8_t dht11_interface_deinit(void)
{
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10);
    return 0;
}

/**
 * @brief      interface bus read
 * @param[out] *value pointer to a value buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t dht11_interface_read(uint8_t *value)
{
	MODIFY_REG(GPIOC->MODER,GPIO_MODER_MODE10,0);
	*value = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
    return 0;
}

/**
 * @brief     interface bus write
 * @param[in] value written value
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t dht11_interface_write(uint8_t value)
{
	MODIFY_REG(GPIOC->MODER,GPIO_MODER_MODE10,GPIO_MODER_MODE10_1);
	MODIFY_REG(GPIOC->OTYPER, GPIO_OTYPER_OT10, 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, value);
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void dht11_interface_delay_ms(uint32_t ms)
{
    delay_ms_80mhz(ms);
}

/**
 * @brief     interface delay us
 * @param[in] us time
 * @note      none
 */
void dht11_interface_delay_us(uint32_t us)
{
    delay_us_80mhz(us);
}

/**
 * @brief interface enable the interrupt
 * @note  none
 */
void dht11_interface_enable_irq(void)
{
    //__enable_irq();
}

/**
 * @brief interface disable the interrupt
 * @note  none
 */
void dht11_interface_disable_irq(void)
{
    //__disable_irq();
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void dht11_interface_debug_print(const char *const fmt, ...)
{
	char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);
	//HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
