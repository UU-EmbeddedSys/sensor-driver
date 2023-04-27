#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include "i2c_registers.h"
#include "sample.h"

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

#define MY_STACK_SIZE 5000
#define MY_PRIORITY   5

#define SENSOR_NODE_ADDR 0x40

#define REFRESH_TIME 1000

LOG_MODULE_REGISTER(main);

K_THREAD_STACK_DEFINE(i2c_stack, MY_STACK_SIZE);

struct k_thread i2c_thread;

struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;

static int sensor_node_read_reg(const struct device *i2c_dev, uint8_t *read_buf, uint8_t num_bytes,
			       uint8_t start_address)
{
	int err = 0;
	struct i2c_msg msg[2];
	// Write on the I2C bus the internal address of the sensor we want to read to
	msg[0].buf = &start_address;
	msg[0].len = 1;
	msg[0].flags = I2C_MSG_WRITE;
	// Read the data from the bus
	msg[1].buf = (uint8_t *)read_buf;
	msg[1].len = num_bytes;
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;
	err = i2c_transfer(i2c_dev, msg, 2, SENSOR_NODE_ADDR);
	return err;
}

//for bme variables
double read_double(const struct device *i2c_dev, uint8_t register_address){
	uint8_t rx_buf[8];
	sensor_node_read_reg(i2c_dev, rx_buf, 8, register_address);

	double temp = 0;
	uint64_t* temp_ptr = (uint64_t*)&temp;
	for(int i = 0; i < 8; i++)
		*temp_ptr = (*temp_ptr) | (uint64_t)rx_buf[i] << (i*8);

	return temp;
}

//for bme variables
float read_float(const struct device *i2c_dev, uint8_t register_address){
	uint8_t rx_buf[4];
	sensor_node_read_reg(i2c_dev, rx_buf, 4, register_address);

	float temp = 0;
	uint32_t* temp_ptr = (uint32_t*)&temp;
	for(int i = 0; i < 4; i++)
		*temp_ptr = (*temp_ptr) | (uint32_t)rx_buf[i] << (i*8);

	return temp;
}

uint8_t read_byte(const struct device *i2c_dev, uint8_t register_address){
	uint8_t rx_buf;
	sensor_node_read_reg(i2c_dev, &rx_buf, 1, register_address);

	
	return rx_buf;
}

uint8_t write_configuration(const struct device *i2c_dev, uint8_t configuration_value, uint8_t register_address){
	read_byte(i2c_dev, CLEAR_I2C);
	return i2c_reg_write_byte(i2c_dev, SENSOR_NODE_ADDR, register_address, configuration_value);
}

void i2c_communication_test(void *p1, void *p2, void *p3)
{
	double received_bme680 = -1;
	float received_distance = -1;
	uint8_t received_byte = -1;
	uint8_t sent_byte = 0xFF;
	
	while(true){
		
		if (false){
			received_bme680 = read_double(i2c_dev, BME680_READ_TEMP);
			printf("Temperature: %f\n", received_bme680);

			received_bme680 = read_double(i2c_dev, BME680_READ_PRESSURE);
			printf("Pressure: %f\n", received_bme680);

			received_bme680 = read_double(i2c_dev, BME680_READ_HUMIDITY);
			printf("Humidity: %f\n", received_bme680);

			received_distance = read_float(i2c_dev, ULTRASONIC_READ);
			printf("Distance: %f\n", received_distance);


			received_distance = read_float(i2c_dev, ADXL345_READ_X);
			printf("X: %f\n", received_distance);

			received_distance = read_float(i2c_dev, ADXL345_READ_Y);
			printf("Y: %f\n", received_distance);

			received_distance = read_float(i2c_dev, ADXL345_READ_Z);
			printf("Z: %f\n", received_distance);
		}

		if(true){
			sent_byte = read_byte(i2c_dev, BME680_CONFIG_TEMP);
			printf("BEFORE_CONFIG Temperature config: %d\n", sent_byte);
			write_configuration(i2c_dev, 0x03, BME680_CONFIG_TEMP);
			sent_byte = read_byte(i2c_dev, BME680_CONFIG_TEMP);
			printf("AFTER CONFIG Temperature config: %d\n", sent_byte);
		}

		
		
		

		k_sleep(K_MSEC(2000));	}
}


void main(void)
{
	int err;
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C: Device is not ready.\n");
		return;
	}

	err = i2c_configure(i2c_dev, i2c_cfg);
	if (err != 0) {
		LOG_ERR("i2c_configure\n");
	}

	k_tid_t i2c_thread_tid = k_thread_create(
		&i2c_thread, i2c_stack, K_THREAD_STACK_SIZEOF(i2c_stack), i2c_communication_test, NULL,
		NULL, NULL, MY_PRIORITY, K_INHERIT_PERMS, K_FOREVER);

	k_thread_start(i2c_thread_tid);
}