/* Device UUID (Must be changed to a unique UUID)*/
static const char DEVICE_UUID[] = "d18e3d17-2d4f-4063-a82a-643598e2d82e";

static const char FW_VERSION[] = "3.0-5-a7870b";
static const unsigned char MECH_STAT_OPEN[] = {0xB8, 0x0B, 0x00, 0x00, 0x00, 0x00, 0b10100};
static const unsigned char MECH_STAT_CLOSE[] = {0xB8, 0x0B, 0x00, 0x00, 0x0E, 0x01, 0b1010010};

static const int PIN_BTN = 27; //Manual open/close button
static const int PIN_ACC_LED = 25; //Access lamp out // pull enabled

/* Call back functions */
static const int PIN_OUT = 32;
static const int PIN_INV = 21;

void on_init(){
	gpio_set_direction(PIN_OUT, GPIO_MODE_OUTPUT);
	gpio_set_direction(PIN_INV, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_OUT, 0);
	gpio_set_level(PIN_INV, 1);
}

void on_open_cb(){
	gpio_set_level(PIN_OUT, 1);
	gpio_set_level(PIN_INV, 0);
}

void on_close_cb(){
	gpio_set_level(PIN_OUT, 0);
	gpio_set_level(PIN_INV, 1);
}
