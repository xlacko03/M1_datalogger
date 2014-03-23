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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

volatile int button_pressed;

static void button_interrupt_setup(void)
{
    nvic_enable_irq(NVIC_EXTI0_IRQ);
    exti_select_source(GPIO0,GPIOA);
    exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI0);
}

static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6 | GPIO7);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO0);
	gpio_set(GPIOB, GPIO6);
}

void exti0_isr(void)
{
    exti_reset_request(EXTI0);
    if(button_pressed)
    {
        button_pressed=0;
    }
    else
        button_pressed=1;
}

int main(void)
{
	int i;
    button_pressed = 1;

	gpio_setup();
	button_interrupt_setup();

	/* Blink the LED (PC8) on the board. */
	while (1) {
		/* Using API function gpio_toggle(): */
        if(button_pressed)
            __asm__("nop");
        if(!button_pressed)
            gpio_toggle(GPIOB, GPIO6 | GPIO7);	/* LED on/off */
		for (i = 0; i < 100000; i++)	/* Wait a bit. */
			__asm__("nop");
	}

	return 0;
}
