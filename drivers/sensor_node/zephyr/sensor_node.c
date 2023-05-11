#define DT_DRV_COMPAT sensor_node

#include "sensor_node.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_node, LOG_LEVEL_INF);

static int sensor_node_bus_check(const struct device *dev)
{
	const struct sensor_config *config = dev->config;
	return device_is_ready(config->i2c.bus) ? 0 : -ENODEV;
}

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

// for bme variables
double read_double(const struct device *i2c_dev, uint8_t register_address)
{
	uint8_t rx_buf[8];
	sensor_node_read_reg(i2c_dev, rx_buf, sizeof(rx_buf), register_address);

	double temp = 0;
	uint64_t *temp_ptr = (uint64_t *)&temp;
	for (int i = 0; i < sizeof(rx_buf); i++) {
		*temp_ptr = (*temp_ptr) | (uint64_t)rx_buf[i] << (i * 8);
	}

	return temp;
}

// for bme variables
float read_float(const struct device *i2c_dev, uint8_t register_address)
{
	uint8_t rx_buf[4];
	sensor_node_read_reg(i2c_dev, rx_buf, sizeof(rx_buf), register_address);

	float temp = 0;
	uint32_t *temp_ptr = (uint32_t *)&temp;
	for (int i = 0; i < sizeof(rx_buf); i++) {
		*temp_ptr = (*temp_ptr) | (uint32_t)rx_buf[i] << (i * 8);
	}

	return temp;
}

uint8_t read_byte(const struct device *i2c_dev, uint8_t register_address)
{
	uint8_t rx_buf;
	sensor_node_read_reg(i2c_dev, &rx_buf, 1, register_address);
	return rx_buf;
}

uint8_t write_configuration(const struct device *i2c_dev, uint8_t configuration_value,
			    uint8_t register_address)
{
	read_byte(i2c_dev, CLEAR_I2C);
	return i2c_reg_write_byte(i2c_dev, SENSOR_NODE_ADDR, register_address, configuration_value);
}

static void print_sensor_data(const struct sensor_data *data)
{
	LOG_INF("Distance: %.2f", data->distance);
	LOG_INF("X Acceleration: %.2f", data->x_acceleration);
	LOG_INF("Y Acceleration: %.2f", data->y_acceleration);
	LOG_INF("Z Acceleration: %.2f", data->z_acceleration);
	LOG_INF("Temperature: %.2f", data->temperature);
	LOG_INF("Humidity: %.2f", data->humidity);
	LOG_INF("Pressure: %.2f", data->pressure);
}

static int sensor_node_init(const struct device *dev)
{
	int err;

	err = sensor_node_bus_check(dev);
	if (err < 0) {
		LOG_ERR("Bus not ready for '%s'", dev->name);
		return err;
	}
	LOG_INF("SENSOR INITIALIZED");
	return 0;
}

static int sensor_node_attr_set(const struct device *dev, enum sensor_channel chan,
				enum sensor_attribute attr, const struct sensor_value *val)
{
	int err = 0;
	struct sensor_data *data = dev->data;
	const struct sensor_config *config = dev->config;
	const struct device *i2c_dev = config->i2c.bus;
	LOG_WRN("ATTR SET");
	switch (attr) {
	case SENSOR_ATTR_OVERSAMPLING:
		// BME
		switch (chan) {
		case SENSOR_CHAN_AMBIENT_TEMP:
			err = write_configuration(i2c_dev, (uint8_t)val->val1, BME680_CONFIG_TEMP);
			break;
		case SENSOR_CHAN_HUMIDITY:
			err = write_configuration(i2c_dev, (uint8_t)val->val1, BME680_CONFIG_HUMIDITY);
			break;
		case SENSOR_CHAN_PRESS:
			err = write_configuration(i2c_dev, (uint8_t)val->val1, BME680_CONFIG_PRESSURE);
			break;
		default:
			break;
		}
		break;
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		// ADXL
		err = write_configuration(i2c_dev, (uint8_t)val->val1, ADXL345_SAMPL_FREQ); // FIXME implement on the sensor too
		break;
	default:
		break;
	}
	return err;
}

static int sensor_node_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct sensor_data *data = dev->data;
	const struct sensor_config *config = dev->config;
	const struct device *i2c_dev = config->i2c.bus;
	LOG_WRN("FETCHING");
	data->temperature = read_double(i2c_dev, BME680_READ_TEMP);
	data->humidity = read_double(i2c_dev, BME680_READ_HUMIDITY);
	data->pressure = read_double(i2c_dev, BME680_READ_PRESSURE);
	data->distance = read_float(i2c_dev, ULTRASONIC_READ);
	data->x_acceleration = read_float(i2c_dev, ADXL345_READ_X);
	data->y_acceleration = read_float(i2c_dev, ADXL345_READ_Y);
	data->z_acceleration = read_float(i2c_dev, ADXL345_READ_Z);
	print_sensor_data(data);
	return 0;
}

// static inline void split_floating_point(double raw, struct sensor_value *val)
// {
// 	double integer, fractional;
// 	fractional = modf(raw, &integer);
// 	val->val1 = (int32_t)integer;
// 	val->val2 = (int32_t)(fractional * 1000);
// }

static int sensore_node_channel_get(const struct device *dev, enum sensor_channel chan,
				    struct sensor_value *val)
{
	struct sensor_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		/* code */
		sensor_value_from_double(val, data->temperature);
		break;
	case SENSOR_CHAN_HUMIDITY:
		sensor_value_from_double(val, data->temperature);
		break;
	case SENSOR_CHAN_PRESS:
		sensor_value_from_double(val, data->pressure);
		break;
	case SENSOR_CHAN_DISTANCE:
		sensor_value_from_double(val, data->distance);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		sensor_value_from_double(val++, data->x_acceleration);
		sensor_value_from_double(val++, data->y_acceleration);
		sensor_value_from_double(val, data->z_acceleration);
		break;
	default:
		break;
	}
	return 0;
}

static const struct sensor_driver_api sensor_node_api_funcs = {
	.sample_fetch = sensor_node_sample_fetch,
	.channel_get = sensore_node_channel_get,
	.attr_set = sensor_node_attr_set,
};

#define SENSOR_CONFIG_I2C(inst)                                                                    \
	{                                                                                          \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.read_reg = sensor_node_read_reg,                                                  \
		.write_reg = sensor_node_write_reg,                                                \
		.bus_check = sensor_node_bus_check,                                                \
	};

#define SENSOR_NODE_DEFINE(inst)                                                                   \
	static struct sensor_data sensor_data_##inst;                                              \
	static const struct sensor_config sensor_config_##inst = SENSOR_CONFIG_I2C(inst)           \
		SENSOR_DEVICE_DT_INST_DEFINE(inst, sensor_node_init, NULL, &sensor_data_##inst,    \
					     &sensor_config_##inst, POST_KERNEL,                   \
					     CONFIG_SENSOR_INIT_PRIORITY, &sensor_node_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(SENSOR_NODE_DEFINE)