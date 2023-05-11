#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include "sample.h"
#include <zephyr/drivers/sensor.h>

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

#define MY_STACK_SIZE 5000
#define MY_PRIORITY   5

#define SENSOR_NODE_ADDR 0x40

#define REFRESH_TIME 1000

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

K_THREAD_STACK_DEFINE(i2c_stack, MY_STACK_SIZE);

struct k_thread i2c_thread;

struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;

const struct device *const sensor = DEVICE_DT_GET_ANY(sensor_node);


void init_sensor(const struct device* sensor)
{
	struct sensor_value oversampling;
	// setting the oversampling to x2
	oversampling.val1 = 0b010;
	oversampling.val2 = 0;
	sensor_attr_set(sensor, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_OVERSAMPLING, &oversampling);
}

void i2c_communication_test(void *p1, void *p2, void *p3)
{

	struct sensor_value distance, temperature, humidity, pressure;
	struct sensor_value accel[3];

	while (true) {

		// sensor_sample_fetch(sensor);
		// sensor_channel_get(sensor, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
		// sensor_channel_get(sensor, SENSOR_CHAN_PRESS, &pressure);
		// sensor_channel_get(sensor, SENSOR_CHAN_HUMIDITY, &humidity);
		// sensor_channel_get(sensor, SENSOR_CHAN_DISTANCE, &distance);
		// sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, accel);


		// LOG_INF("Temperature: %d.%03d °C", temperature.val1, temperature.val2);
		// LOG_INF("Humidity: %d.%03d %%", humidity.val1, humidity.val2);
		// LOG_INF("Pressure: %d.%03d Pa", pressure.val1, pressure.val2);
		// LOG_INF("Distance: %d.%03d cm", distance.val1, distance.val2);
		// char axis = 'X';
		// for (size_t i = 0; i < 3; i++)
		// {
		// 	LOG_INF("%c acceleration: %d.%03d [m/s^2]", axis++, accel[i].val1, accel[i].val2);
		// }
		init_sensor(sensor);
		
		k_sleep(K_MSEC(2000));
	}
}

void main(void)
{
	LOG_WRN("*** SENSOR DRIVER ***");
	int err;
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C: Device is not ready.\n");
		return;
	}

	err = i2c_configure(i2c_dev, i2c_cfg);
	if (err != 0) {
		LOG_ERR("i2c_configure\n");
	}

	if (sensor == NULL) {
		LOG_ERR("error, no dev found\n");
		return;
	}
	// if (!device_is_ready(sensor)) {
	// 	LOG_ERR("dev not ready\n");
	// 	return NULL;
	// }

	LOG_INF("Found device \"%s\", getting sensor data\n", sensor->name);

	init_sensor(sensor);

	k_tid_t i2c_thread_tid = k_thread_create(
		&i2c_thread, i2c_stack, K_THREAD_STACK_SIZEOF(i2c_stack), i2c_communication_test,
		NULL, NULL, NULL, MY_PRIORITY, K_INHERIT_PERMS, K_FOREVER);

	k_thread_start(i2c_thread_tid);
}