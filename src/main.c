/*
 * This file is part of the stm32-template project.
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/cm3/nvic.h>

volatile int button_pressed;

/*static void button_interrupt_setup(void)
{
    nvic_enable_irq(NVIC_EXTI0_IRQ);

}*/

static void gpio_setup(void)
{
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
	gpio_clear(GPIOB, GPIO6);
}


static void reset_clock(void)
{
    clock_scale_t cl_config = {
        .hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
        .ppre1 = RCC_CFGR_PPRE1_HCLK_NODIV,
        .ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,
        .voltage_scale = RANGE2,    //range1
        .flash_config = FLASH_ACR_LATENCY_1WS,
        .apb1_frequency = 4194000,
        .apb2_frequency = 4194000,
        .msi_range = RCC_ICSCR_MSIRANGE_4MHZ,
    };
    rcc_clock_setup_msi(&cl_config);
    rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	//rcc_peripheral_enable_clock(RCC_APB1ENR,RCC_APB1ENR_PWREN);
}

int main(void)
{
	int i;
    button_pressed = 1;

    reset_clock();
	gpio_setup();
	//button_interrupt_setup();

	setup_rtc();
	setup_rtc_wakeup();

	/* Blink the LED (PC8) on the board. */
	while (1) {
		/* Using API function gpio_toggle(): */
        PWR_CR |= PWR_CR_LPSDSR;
        pwr_set_stop_mode();
        __asm volatile("wfi");
        gpio_set(GPIOB, GPIO6);	/* LED on/off */
		for (i = 0; i < 100000; i++)	/* Wait a bit. */
			__asm__("nop");
        gpio_clear(GPIOB,GPIO6);
	}

	return 0;
}
