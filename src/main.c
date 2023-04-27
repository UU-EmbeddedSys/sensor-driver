#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

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

static int sensor_node_write_reg(const struct device *i2c_dev, uint8_t *write_buf, uint8_t num_bytes,
				uint8_t start_address){
	int err = 0;
	struct i2c_msg msg[2];
	// Write on the I2C bus the internal address of the sensor we want to read to
	msg[0].buf = &start_address;
	msg[0].len = 1;
	msg[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	// Read the data from the bus
	msg[1].buf = (uint8_t *)write_buf;
	msg[1].len = num_bytes;
	msg[1].flags = I2C_MSG_RESTART |I2C_MSG_WRITE | I2C_MSG_STOP;
	err = i2c_transfer(i2c_dev, msg, 2, SENSOR_NODE_ADDR);
	return err;
}

double parse_double(uint8_t* buffer){
	LOG_HEXDUMP_INF(buffer, 8, "buffer");
	double temp = 0;
	uint64_t* temp_ptr = (uint64_t*)&temp;
	for(int i = 0; i < 8; i++){
		//printf("Ax%02x ", buffer[i]);
		*temp_ptr = (*temp_ptr) | (uint64_t)buffer[i] << (i*8);
		

		for (int i = 0; i < 8; i++) {
			printf("%02x", (uint8_t)(((uint8_t*)&temp)[i]));
		}
		printf("\n");
	}
	printf("\n");
	return temp;
}


void i2c_communication_test_read(void *p1, void *p2, void *p3)
{
	double reconstructed_value = 0;
	double true_value = 0;

	uint8_t tx_buf;
	uint8_t rx_buf[10];


	uint8_t amount_to_read = 8;
	while(true){

        tx_buf = 0x10;
        int ret = i2c_write_read(i2c_dev, SENSOR_NODE_ADDR, &tx_buf, 1, rx_buf, amount_to_read);
        if (ret) {
            printf("Failed to read from BME680\n");
            return;
        }
		printf("--\n");
		reconstructed_value = parse_double(rx_buf);
		true_value = 0.5;
		printf("\nTemperature: %lf %lf %d\n", reconstructed_value, (double)true_value, sizeof(double));
		printf("RECONSTRUCTED HEX:\n");
		for (int i = 0; i < 8; i++) {
			printf("0x%02x ", (uint8_t)(((uint8_t*)&reconstructed_value)[i]));
		}
		printf("\nTRUE HEX:\n");
		for (int i = 0; i < 8; i++) {
			printf("0x%02x ", (uint8_t)(((uint8_t*)&true_value)[i]));
		}
		printf("\n");
		k_sleep(K_MSEC(4000));
	}
}

void i2c_communication_test_write(void *p1, void *p2, void *p3)
{
	uint8_t data_sent = 0x69;
	while(true){
		LOG_INF("SENDING DATA\n");
		sensor_node_write_reg(i2c_dev, &data_sent, 1, 0x99);
		data_sent = 0x68;
		sensor_node_write_reg(i2c_dev, &data_sent, 1, 0x95);		
		k_sleep(K_MSEC(2000));
	}
}

//slave only supports single byte write
void i2c_communication_test_read_write(void *p1, void *p2, void *p3)
{
	uint64_t reconstructed_value = 0;
	double true_value = 0;

	uint8_t tx_buf;
	uint8_t rx_buf[10];

	uint8_t data_sent = 0x69;

	uint8_t amount_to_read = 8;
	while(true){

        tx_buf = 0x20;
        int ret = i2c_write_read(i2c_dev, SENSOR_NODE_ADDR, &tx_buf, 1, rx_buf, amount_to_read);
        if (ret) {
            printf("Failed to read from BME680\n");
            return;
        }
		printf("READ VALUE HEX:0x");
		for (int i = 0; i < 8; i++) {
			printf("%02x ", (uint8_t)(((uint8_t*)&rx_buf)[i]));
		}
		printf("\n");
		
		k_sleep(K_MSEC(4000));

		sensor_node_write_reg(i2c_dev, &data_sent, 1, 0x99);
		sensor_node_write_reg(i2c_dev, &data_sent, 1, 0x95);

		k_sleep(K_MSEC(4000));

		tx_buf = 0x20;
        ret = i2c_write_read(i2c_dev, SENSOR_NODE_ADDR, &tx_buf, 1, rx_buf, amount_to_read);
        if (ret) {
            printf("Failed to read from BME680\n");
            return;
        }
		printf("READ VALUE HEX:0x");
		for (int i = 0; i < 8; i++) {
			printf("%02x ", (uint8_t)(((uint8_t*)&rx_buf)[i]));
		}
		printf("\n");
		
		sensor_node_write_reg(i2c_dev, &data_sent, 1, 0x99);
		k_sleep(K_MSEC(4000));

	}
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
		&i2c_thread, i2c_stack, K_THREAD_STACK_SIZEOF(i2c_stack), i2c_communication_test_read, NULL,
		NULL, NULL, MY_PRIORITY, K_INHERIT_PERMS, K_FOREVER);

	k_thread_start(i2c_thread_tid);
}