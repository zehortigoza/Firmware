#include <board_config.h>
#include <drivers/device/i2c.h>

#define SLAVE_ADDR 0x51

#define VERSION_REG 0x00

#define UART_INVERTED_REG 0x01
#define UART_INVERTED_RC_UART_VALUE 0x01
#define UART_INVERTED_TELEMETRY_UART_VALUE 0x02

#define TELEMETRY_CONNECTOR_SEL_REG 0x02
#define TELEMETRY_CONNECTOR_SEL_I2C_VALUE 0x01

extern "C" { __EXPORT int aerofc_fpga_config_main(int argc, char *argv[]); }

class aerofc_fpga_config : public device::I2C
{
public:
	aerofc_fpga_config(uint8_t bus);
	virtual ~aerofc_fpga_config();

	virtual int init();
	int set_telemetry_connector_selector(bool enable_i2c);
	int set_uart_inverted(bool invert_rc, bool invert_telemetry);
	int print_state();

private:
	virtual int probe();
};

aerofc_fpga_config::aerofc_fpga_config(uint8_t bus) :
	I2C("aerofc_config", "/dev/aerofc_fpga_config", bus, SLAVE_ADDR, 400000)
{
}

aerofc_fpga_config::~aerofc_fpga_config()
{
}

int aerofc_fpga_config::init()
{
	return I2C::init();
}

int aerofc_fpga_config::probe()
{
	uint8_t buffer = VERSION_REG;
	return transfer(&buffer, 1, &buffer, 1);
}

int aerofc_fpga_config::set_telemetry_connector_selector(bool enable_i2c)
{
	uint8_t buffer[2] = { TELEMETRY_CONNECTOR_SEL_REG, 0 };
	buffer[1] = enable_i2c ? TELEMETRY_CONNECTOR_SEL_I2C_VALUE : 0;
	return transfer(buffer, 2, nullptr, 0);
}

int aerofc_fpga_config::set_uart_inverted(bool invert_rc, bool invert_telemetry)
{
	uint8_t buffer[2] = { UART_INVERTED_REG, 0 };

	buffer[1] |= invert_rc ? UART_INVERTED_RC_UART_VALUE : 0;
	buffer[1] |= invert_telemetry ? UART_INVERTED_TELEMETRY_UART_VALUE : 0;

	if (invert_rc) {
		printf("RC UART is inverted\n");
	}
	if (invert_telemetry) {
		printf("Telemetry UART is inverted\n");
	}

	return transfer(buffer, 2, nullptr, 0);
}

int aerofc_fpga_config::print_state()
{
	uint8_t buffer = UART_INVERTED_REG;
	int ret = transfer(&buffer, 1, &buffer, 1);

	if (ret) {
		warnx("Error getting register values");
		return -1;
	}

	printf("RC UART inverted? %s\n", (buffer & UART_INVERTED_RC_UART_VALUE) ? "yes" : "no");
	printf("Telemetry UART inverted? %s\n", (buffer & UART_INVERTED_TELEMETRY_UART_VALUE) ? "yes" : "no");

	buffer = TELEMETRY_CONNECTOR_SEL_REG;
	ret = transfer(&buffer, 1, &buffer, 1);

	printf("Telemetry connector is running in %s mode\n", buffer ? "I2C" : "UART");

	return 0;
}

static void _telemetry_connector_configure(aerofc_fpga_config *config, bool enable_i2c)
{
	config->set_telemetry_connector_selector(enable_i2c);

	if (!enable_i2c) {
		return;
	}

	stm32_unconfiggpio(GPIO_UART8_RX);
	stm32_unconfiggpio(GPIO_UART8_TX);

	device::I2C::enable_runtime_bus(PX4_I2C_BUS_EXPANSION1);
}

int aerofc_fpga_config_main(int argc, char *argv[])
{
	param_t enable_i2c_param = param_find("AEROFC_I2C_TELEM");
	param_t uart_invert_param = param_find("AEROFC_UART_INV");
	aerofc_fpga_config *config = new aerofc_fpga_config(PX4_I2C_BUS_EXPANSION);
	int32_t value;

	if (!config) {
		warnx("Unable to instantiate aerofc_config");
		return -1;
	}

	if (config->init()) {
		warnx("Unable to initialize aerofc_config");
		goto end;
	}

	param_get(enable_i2c_param, &value);
	_telemetry_connector_configure(config, value);

	param_get(uart_invert_param, &value);
	config->set_uart_inverted(value & 0x01, value & 0x02);

	config->print_state();

	delete config;

	return 0;

end:
	delete config;
	return -1;
}
