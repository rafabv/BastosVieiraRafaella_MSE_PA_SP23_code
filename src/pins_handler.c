#include "pins_handler.h"


void pins_init(){
	if (!device_is_ready(bootpin.port)) {
		return;
	}
	gpio_pin_configure_dt(&bootpin, GPIO_OUTPUT_ACTIVE);


	if (!device_is_ready(resetpin.port)) {
		return;
	}
	gpio_pin_configure_dt(&resetpin, GPIO_OUTPUT_INACTIVE);
	
	reset_stm();
}

void boot_activation(bool onoff){
	if(onoff){
		gpio_pin_configure_dt(&bootpin, GPIO_OUTPUT_ACTIVE);
	}
	else{
		gpio_pin_configure_dt(&bootpin, GPIO_OUTPUT_INACTIVE);
	}
	
}

void reset_stm(){
	gpio_pin_configure_dt(&resetpin, GPIO_OUTPUT_INACTIVE);
	k_msleep(100);
	gpio_pin_configure_dt(&resetpin, GPIO_OUTPUT_ACTIVE);

	k_msleep(100);
	gpio_pin_configure_dt(&resetpin, GPIO_OUTPUT_INACTIVE);
}