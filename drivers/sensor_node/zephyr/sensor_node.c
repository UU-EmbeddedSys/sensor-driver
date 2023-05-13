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
	msg[0].flags = I2C_MSG_WRITE;
	// Read the data from the bus
	msg[1].buf = (uint8_t *)write_buf;
	msg[1].len = num_bytes;
	msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
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
	// read_byte(i2c_dev, CLEAR_I2C);
	// return i2c_reg_write_byte(i2c_dev, SENSOR_NODE_ADDR, register_address, configuration_value);
	return sensor_node_write_reg(i2c_dev, &configuration_value, sizeof(configuration_value), register_address);
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


static void setup_int(const struct device *dev,
		      bool enable)
{
	LOG_INF("setup_int %d", enable);
	const struct sensor_config *cfg = dev->config;
	gpio_flags_t flags = enable
		? GPIO_INT_EDGE_TO_ACTIVE
		: GPIO_INT_DISABLE;

	gpio_pin_interrupt_configure_dt(&cfg->int_gpio, flags);
}

static void process_int(const struct device *dev)
{
	LOG_INF("process_int");
	const struct sensor_config *config = dev->config;
	struct sensor_data *data = dev->data;
	uint8_t int_source;
	LOG_INF("ISR Processing");
	if (sensor_node_read_reg(config->i2c.bus, &int_source, sizeof(int_source),
				 SENSOR_NODE_ISR_SRC)) {
		LOG_ERR("Could not read interrupt source");
		int_source = 0U;
	}
	// int_source = read_byte(config->i2c.bus, SENSOR_NODE_ISR_SRC);
	LOG_WRN("src int: %d", int_source);
	switch (int_source) {
	case ACCELERATOR:
		/* code */
		break;
	case HUMIDITY:
		/* code */
		break;
	case TEMPERATURE:
		/* code */
		LOG_INF("SOURCE: TEMPERATURE");
		break;
	case PRESSURE:
		/* code */
		break;
	case DISTANCE:
		/* code */
		break;
	default:
		break;
	}

	// Clear the interrupt register
	uint8_t clear_int = 0;
	sensor_node_write_reg(config->i2c.bus, &clear_int, sizeof(clear_int), SENSOR_NODE_ISR_SRC);

	// Re-enable the interrupt
	setup_int(dev, true);

}

/**
 * @brief Callback associated at the interrupt
 * 
 * @param port 
 * @param cb 
 * @param pins 
 */
void sensor_node_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	LOG_WRN("ISR!");
	struct sensor_data *data = CONTAINER_OF(cb, struct sensor_data, gpio_cb);
	// Disable the interrupt
	setup_int(data->dev, false);
	// Signal the thread that in interrupt has been received
	k_sem_give(&data->gpio_sem);
}

static void sensor_node_thread(struct sensor_data *drv_data)
{
	LOG_INF("SN THR CREATED");
	while (true) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		process_int(drv_data->dev);
	}
}

/**
 * @brief Just for understanding. The trigger works with deferred interrupt handling.
 * A callback is registered on the interrupt pin.
 * A semaphore is initialized at 0.
 * A thread to handle the interrupt is created waiting for the semaphore being release.
 * The interrupt at first is disabled.
 * When from the main, a trigger is set, the interrupt is re-enabled (for now).
 * 
 * Once the interrupt is triggered, sensor_node_isr is called, which disable the int and 
 * release the semaphore for the thread in order to process it.
 * Once processed, the interrupt is re-enabled on that pin.
 * @param dev 
 * @return int 
 */
static int sensor_node_trigger_init(const struct device *dev)
{
	const struct sensor_config *config = dev->config;
	struct sensor_data *data = dev->data;

	data->dev = dev;

	if (config->int_gpio.pin == 20) {
		LOG_ERR("No interrupt pin configured, using default one");
		// return 0;
	}
	LOG_INF("TRIGGER PIN: %d", config->int_gpio.pin);
	if (!device_is_ready(config->int_gpio.port)) {
		LOG_ERR("%s: device %s is not ready", dev->name, config->int_gpio.port->name);
		return -ENODEV;
	}
	// Configure the interrupt pin
	gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	// Init the callback on the pin
	gpio_init_callback(&data->gpio_cb, sensor_node_isr, BIT(config->int_gpio.pin));

	if (gpio_add_callback(config->int_gpio.port, &data->gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback");
		return -EIO;
	}
	// Create semaphore and thread
	k_sem_init(&data->gpio_sem, 0, 1);

	k_thread_create(&data->thread, data->thread_stack,
			2048,
			(k_thread_entry_t)sensor_node_thread, data,
			NULL, NULL, K_PRIO_COOP(10),
			0, K_NO_WAIT);
	// Disable interrupt
	setup_int(dev, false);
	return 0;
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
	err = sensor_node_trigger_init(dev);
	return err;
}

static int sensor_node_attr_set(const struct device *dev, enum sensor_channel chan,
				enum sensor_attribute attr, const struct sensor_value *val)
{
	int err = 0;
	// struct sensor_data *data = dev->data;
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
			err = write_configuration(i2c_dev, (uint8_t)val->val1,
						  BME680_CONFIG_HUMIDITY);
			break;
		case SENSOR_CHAN_PRESS:
			err = write_configuration(i2c_dev, (uint8_t)val->val1,
						  BME680_CONFIG_PRESSURE);
			break;
		default:
			break;
		}
		break;
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		// ADXL
		err = write_configuration(i2c_dev, (uint8_t)val->val1,
					  ADXL345_SAMPL_FREQ); // FIXME implement on the sensor too
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
	// print_sensor_data(data);
	return 0;
}

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

static int sensor_node_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
				   sensor_trigger_handler_t handler)
{
	setup_int(dev, true);
	// TODO
	return 0;
}

static const struct sensor_driver_api sensor_node_api_funcs = {
	.sample_fetch = sensor_node_sample_fetch,
	.channel_get = sensore_node_channel_get,
	.attr_set = sensor_node_attr_set,
	.attr_get = NULL,
	.trigger_set = sensor_node_trigger_set,
};

#define SENSOR_CONFIG_I2C(inst)                                                                    \
	{                                                                                          \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, gpios, {20}),                           \
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