#ifndef SENSOR_NODE_H

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

typedef enum int_source_t
{
	NONE,
	HUMIDITY,
	TEMPERATURE,
	PRESSURE,
	DISTANCE,
	ACCELERATOR
} int_source_t;

#define SENSOR_NODE_ISR_SRC 0x05 // TODO add in sensor node

#define TEST_READ_DOUBLE 0x10
#define TEST_READ_SCALE 0x20

//ADXL345
//BME680
//read: 3x4 bytes (float) xyz
#define ADXL345_READ_X 0xA0 //until 0x53
#define ADXL345_READ_Y 0xB0 //until 0x63
#define ADXL345_READ_Z 0xC0 //until 0x73

#define ADXL345_SAMPL_FREQ 0xD0

//BME680
//read: 3x8 bytes (double) temperature, pressure, humidity
//configuration: 3x1byte (uint8_t) sampling rate for temperature, pressure, humidity
#define BME680_READ_TEMP 0x50 //until 0x57
#define BME680_READ_PRESSURE 0x60 //until 0x67
#define BME680_READ_HUMIDITY 0x70 //until 0x77

#define BME680_CONFIG_TEMP 0x5A
#define BME680_CONFIG_PRESSURE 0x6A
#define BME680_CONFIG_HUMIDITY 0x7A

//ULTRASONIC
//read: 1x4bytes (float) distance in cm
//configuration 1x1byte (uint8_t) polling rate (not implemented)
#define ULTRASONIC_READ 0x80 //until 0x83
#define ULTRASONIC_POLLING 0x8A 

#define CLEAR_I2C 0xFF

#define SENSOR_NODE_ADDR 0x40

typedef int (*read_register)(const struct device *i2c_dev, uint8_t *read_buf, uint8_t num_bytes,
			     uint8_t start_address);

typedef int (*write_register)(const struct device *i2c_dev, uint8_t *write_buf, uint8_t num_bytes,
			     uint8_t start_address);

typedef int (*i2c_bus_check)(const struct device *dev);

struct sensor_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec int_gpio; // set the trigger gpio
	read_register read_reg;
	write_register write_reg;
	i2c_bus_check bus_check;
};

struct sensor_data {
	float distance;
	float x_acceleration;
	float y_acceleration;
	float z_acceleration;
	double temperature;
	double humidity;
	double pressure;
	struct gpio_callback gpio_cb; // store the callback of the interrupt
	const struct device* dev;
	K_KERNEL_STACK_MEMBER(thread_stack, 2048);
	struct k_sem gpio_sem;
	struct k_thread thread;
};

#endif