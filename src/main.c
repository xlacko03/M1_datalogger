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
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO0);
	gpio_clear(GPIOB, GPIO6);
}

static int setup_rtc(void)
{
    rcc_periph_clock_enable(RCC_PWR);
    pwr_disable_backup_domain_write_protect();

    RCC_CSR |= RCC_CSR_RTCRST;
    RCC_CSR &= ~RCC_CSR_RTCRST;

    rcc_osc_on(LSE);
    rcc_wait_for_osc_ready(LSE);

    rcc_rtc_select_clock(RCC_CSR_RTCSEL_LSE);

    RCC_CSR |= RCC_CSR_RTCEN;

    rtc_unlock();

    RTC_ISR |= RTC_ISR_INIT;
    while((RTC_ISR & RTC_ISR_INITF)==0);

    uint32_t sync = 255;
    uint32_t async = 127;
    rtc_set_prescaler(sync, async);

    //rtc_lock();

    RCC_CSR |= RCC_CSR_RTCEN;

    //pwr_disable_backup_domain_write_protect();
    //rtc_wait_for_synchro();

    return 0;

}

static int setup_rtc_wakeup(void)
{
    rtc_unlock();

    RTC_CR &= ~RTC_CR_WUTE;

    while((RTC_ISR & RTC_ISR_WUTWF) == 0);

    RTC_WUTR = 0;

    //RTC_CR &= ~(RTC_CR_WUCLKSEL_MASK << RTC_CR_WUCLKSEL_SHIFT);
    //RTC_CR |= (RTC_CR_WUCLKSEL_MASK << RTC_CR_WUCLKSEL_SHIFT);

    RTC_CR &= ~(1<<1);
    RTC_CR &= ~(1<<2);
    RTC_CR |= (1<<2);
    RTC_WUTR |= (1<<3)|(1<<1);


    RTC_CR |= RTC_CR_WUTE;

    RTC_CR |= RTC_CR_WUTIE;

    //rtc_lock();

    nvic_enable_irq(NVIC_RTC_WKUP_IRQ);
    //exti_select_source(GPIO0,GPIOA);
    exti_set_trigger(EXTI20, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI20);
    return 0;
}

void rtc_wkup_isr(void)
{
    RTC_ISR &= ~(RTC_ISR_WUTF);
    exti_reset_request(EXTI20);
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
