#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

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

// SHELL_UART_DEFINE(driver_transport, CONFIG_SHELL_BACKEND_SERIAL_TX_RING_BUFFER_SIZE, CONFIG_SHELL_BACKEND_SERIAL_RX_RING_BUFFER_SIZE);
// SHELL_DEFINE(driver_shell, "sensor-driver:~$ ", &driver_transport, 10, 0, SHELL_FLAG_OLF_CRLF);
// extern const struct shell* shell_uart;

/**
 * @brief Set sensor configurations
 * 
 * @param sensor 
 */
void init_sensor(const struct device* sensor)
{
	struct sensor_value oversampling;
	// setting the oversampling to x2
	oversampling.val1 = 0b010;
	oversampling.val2 = 0;
	sensor_attr_set(sensor, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_OVERSAMPLING, &oversampling);
	// setting the oversampling to x1
	oversampling.val1 = 0b001;
	oversampling.val2 = 0;
	sensor_attr_set(sensor, SENSOR_CHAN_HUMIDITY, SENSOR_ATTR_OVERSAMPLING, &oversampling);
	// setting the oversampling to x16
	oversampling.val1 = 0b101;
	oversampling.val2 = 0;
	sensor_attr_set(sensor, SENSOR_CHAN_PRESS, SENSOR_ATTR_OVERSAMPLING, &oversampling);
}

void i2c_communication_test(void *p1, void *p2, void *p3)
{

	struct sensor_value distance, temperature, humidity, pressure;
	struct sensor_value accel[3];

	while (true) {

		sensor_sample_fetch(sensor);
		sensor_channel_get(sensor, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
		sensor_channel_get(sensor, SENSOR_CHAN_PRESS, &pressure);
		sensor_channel_get(sensor, SENSOR_CHAN_HUMIDITY, &humidity);
		sensor_channel_get(sensor, SENSOR_CHAN_DISTANCE, &distance);
		sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, accel);
		init_sensor(sensor);


		LOG_INF("Temperature: %d.%03d °C", temperature.val1, temperature.val2);
		LOG_INF("Humidity: %d.%03d %%", humidity.val1, humidity.val2);
		LOG_INF("Pressure: %d.%03d Pa", pressure.val1, pressure.val2);
		LOG_INF("Distance: %d.%03d cm", distance.val1, distance.val2);
		char axis = 'X';
		for (size_t i = 0; i < 3; i++)
		{
			LOG_INF("%c acceleration: %d.%03d [m/s^2] - %d g", axis++, accel[i].val1, accel[i].val2, sensor_ms2_to_g(&accel[i]));
		}
		
		k_sleep(K_MSEC(2000));
	}
}

static int cmd_calibrate_distance_sensor(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "*** Position the distance sensor ***");
	shell_print(sh, "Press enter when is in the correct postion.");
	struct sensor_value distance;
	char data;
	int cnt;
	while(data != '\n' && data != 0x0D)
	{
		sh->iface->api->read(sh->iface, &data, 1, &cnt);
		k_sleep(K_MSEC(500));
		// shell_print(sh, "%c 0x%02X", data, data);
	}
	// FIXME set the calibration value to the sensor node
	sensor_sample_fetch_chan(sensor, SENSOR_CHAN_DISTANCE);
	sensor_channel_get(sensor, SENSOR_CHAN_DISTANCE, &distance);
	shell_print(sh, "sensor calibrated! Threshold distance: %f", sensor_value_to_double(&distance));
	return 0;
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

	LOG_INF("Found device \"%s\", getting sensor data\n", sensor->name);

	init_sensor(sensor);

	shell_execute_cmd(shell_backend_uart_get_ptr(), "log disable");
	shell_execute_cmd(shell_backend_uart_get_ptr(), "driver calibrate");

	k_tid_t i2c_thread_tid = k_thread_create(
		&i2c_thread, i2c_stack, K_THREAD_STACK_SIZEOF(i2c_stack), i2c_communication_test,
		NULL, NULL, NULL, MY_PRIORITY, K_INHERIT_PERMS, K_FOREVER);

	k_thread_start(i2c_thread_tid);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sensor_driver_cmd,
	SHELL_CMD(calibrate, NULL, "Calibrate the distance sensor.", cmd_calibrate_distance_sensor),
	SHELL_SUBCMD_SET_END	
);
/* Creating root (level 0) command "demo" */
SHELL_CMD_REGISTER(driver, &sensor_driver_cmd, "Sensor Driver commands.", NULL);