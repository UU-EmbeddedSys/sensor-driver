#ifndef SENSOR_NODE_H

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

#define SENSOR_NODE_ADDR 0x40

typedef int (*read_register)(const struct device *i2c_dev, uint8_t *read_buf, uint8_t num_bytes,
			     uint8_t start_address);

typedef int (*write_register)(const struct device *i2c_dev, uint8_t *write_buf, uint8_t num_bytes,
			     uint8_t start_address);

struct sensor_config {
	struct i2c_dt_spec i2c;
	read_register read_reg;
	write_register write_reg;
};

struct sensor_data {
	float distance;
};

#endif