#define DT_DRV_COMPAT sensor_node

#include "sensor_node.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_node, LOG_LEVEL_INF);

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

static int sensor_node_write_reg(const struct device *i2c_dev, uint8_t *write_buf,
				 uint8_t num_bytes, uint8_t start_address)
{
	int err = 0;
	struct i2c_msg msg[2];
	// Write on the I2C bus the internal address of the sensor we want to read to
	msg[0].buf = &start_address;
	msg[0].len = 1;
	msg[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	// Read the data from the bus
	msg[1].buf = (uint8_t *)write_buf;
	msg[1].len = num_bytes;
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_WRITE | I2C_MSG_STOP;
	err = i2c_transfer(i2c_dev, msg, 2, SENSOR_NODE_ADDR);
	return err;
}

static void sensor_node_init(void)
{
	LOG_INF("SENSOR INITIALIZED");
}

static int sensor_node_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	return 0;
}

static int sensore_node_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	return 0;
}

static const struct sensor_driver_api sensor_node_api_funcs = {
	.sample_fetch = sensor_node_sample_fetch,
	.channel_get = sensore_node_channel_get,
};

#define SENSOR_CONFIG_I2C(inst)                                                                    \
	{                                                                                          \
		.i2c =  I2C_DT_SPEC_INST_GET(inst), \
		 .read_reg = sensor_node_read_reg, \
		.write_reg = sensor_node_write_reg  \
	};

#define SENSOR_NODE_DEFINE(inst)                                                                   \
	static struct sensor_data sensor_data_##inst;                                              \
	static const struct sensor_config sensor_config_##inst = SENSOR_CONFIG_I2C(inst) \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, sensor_node_init, NULL, &sensor_data_##inst, \
			     &sensor_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
			     &sensor_node_api_funcs); \

DT_INST_FOREACH_STATUS_OKAY(SENSOR_NODE_DEFINE)