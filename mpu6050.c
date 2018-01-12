/*----------------------------------------------------------------------------*/
/* FILE NAME:     MPU6050                                                     */
/* Team Member: Ian Chen ,Omnon ,YoYo ,NekoSaiKou                             */
/* Group Number:4                                                             */
/* DESCRIPTION:   Application example for reading data from an I2C gyro       */
/*                sensor and displaying it on the debug console,              */
/*                and the debug console.                                      */
/*----------------------------------------------------------------------------*/
#include "mpu6050.h"
#include "starter.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "../io/Lcd/lcd.h"
#include "../io/gpio/src/gpio_hal.h"

/*****************************************************************************/
// FUNCTION PROTOTYPES
/*****************************************************************************/
void LED_Breath(DWCREG_PTR gpio, int times);
int Read_Gyro(DWCREG_PTR i2c);
void gyro_init(DWCREG_PTR console, DWCREG_PTR i2c, DWCREG_PTR gpio);
void delay(unsigned int tics);
/*****************************************************************************/
// GLOBAL VARIABLES
/*****************************************************************************/
char strng[64] = { 0 };
char datax[64] = { 0 };
char datay[64] = { 0 };
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
/*****************************************************************************/
/* MAIN */
/*****************************************************************************/
int main(void) {
	float angle_x_acc, angle_y_acc;
	float angle_x = 0;
	float angle_y = 0;
	int set_gyro_angles = 0;

	int stop = 0;
	DWCREG_PTR i2c = I2C_CTRL_ADDR;
	DWCREG_PTR mux = CREG_CTRL_ADDR;
	DWCREG_PTR console = CONSOLE_CTRL_ADDR;
	DWCREG_PTR gpio = GPIO_CTRL_ADDR;
	// initialize Pin multiplexer
	mux_init(mux);
	set_pmod_mux(mux,
	PM1_UR_GPIO_C | PM1_LR_GPIO_A |
	PM2_GPIO_AC | PM2_GPIO_AC |
	PM3_I2C_GPIO_D |
	PM4_GPIO_AC |
	PM5_LR_GPIO_A | PM5_UR_GPIO_C |
	PM6_UR_SPI_M0);
	// Pmod1[4:1] are connected to DW GPIO Port C[11:8]
	// Pmod1[10:7] are connected to DW GPIO Port A[11:8]

	// Pmod2[4:1] 	are connected to DW GPIO Port C[15:12]
	// Pmod2[10:7] are connected to DW GPIO Port A[15:12]

	// Pmod3[4:3] are connected to DW I2C signals,
	// Pmod3[2:1] are connected to DW GPIO Port D[1:0],

	// Pmod4[4:1] are connected to DW GPIO Port C[23:20]
	// Pmod4[10:7] are connected to DW GPIO Port A[23:20]

	// Pmod5[4:1] are connected to DW GPIO Port C[27:24]
	// Pmod5[10:7] are connected to DW GPIO Port A[27:24]

	// Pmod6[4:1] are connected to DW SPI Master signals using CS0_N

	//set gpio pins into output mode
	gpio[SWPORTC_DDR] = 0xff0ff00; //PortC - output 1111 1111 0000 1111 1111 0000 0000
	gpio[SWPORTA_DDR] = 0xff0ff00; //PortC - output 1111 1111 0000 1111 1111 0000 0000
	//initialize dw i2c controller
	i2c_init(i2c, I2C_HIGH_SPEED, I2C_SLAVE_ADDRESS);
	//initialize spi controller and LCD
	spi_init(SPIM_CTRL_ADDR, SPI_BAUDRATE(250000), 0x00);
	Lcd_Init(SPIM_CTRL_ADDR);
	Lcd_DisplayOnOff(true);  //display on
	Lcd_DisplayMode(true);   //wrap 16 chars
	Lcd_ClearScreen();       //clear screen
	Lcd_CursorOff();

	//turn off all led to indicate initializing process
	gpio_set_leds(gpio, 0x01ff);

	// initialize mpu6050
	uart_print(console, "Initializing.....");
	Lcd_DisplayString(0, 0, "Initializing.....");
	gyro_init(console, i2c, gpio);
	uart_print(console, "\nDone!!!\r\n");
	Lcd_DisplayString(0, 0, "Done!!");
	Lcd_DisplayString(1, 0, "Hello My Friend");
	LED_Breath(gpio, 3);

	// main loop
	//--------------------------------------------------------------------------

	while (!stop) {
		//start the timer
		write_auxreg(0, TIMER);

		Read_Gyro(i2c);
		gyro_x = gyro_x - gyro_x_cal;
		gyro_y = gyro_y - gyro_y_cal;
		gyro_z = gyro_z - gyro_z_cal;

		//0.00002181025 = 1 / (700Hz * 65.5)
		angle_x += gyro_x * 0.00007633587786;
		angle_y += gyro_y * 0.00007633587786;

		//0.0000003806606 = 0.00002181025 * (3.142(PI) / 180degr) The C sin function is in radian
		int xfix = angle_y * sin(gyro_z * 0.000000133231241);
		int yfix = angle_x * sin(gyro_z * 0.000000133231241);
		angle_x += xfix; //If the IMU has yawed transfer the y angle to the x angel
		angle_y -= yfix; //If the IMU has yawed transfer the y angle to the x angel

		//ACC
		acc_total_vector = sqrt(
				(acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
		//57.2957 = 1 / (3.142 / 180) The C arcsin function is in radian
		angle_y_acc = asin((float) acc_y / acc_total_vector) * 57.2957; //Calculate the y angle
		angle_x_acc = asin((float) acc_x / acc_total_vector) * -57.2957; //Calculate the x angle

		//Place the MPU-6050 spirit level and note the values in the following two lines for calibration
		angle_y_acc -= -0.9;             //Accelerometer calibration value for y
		angle_x_acc -= 2.0;              //Accelerometer calibration value for x

		if (set_gyro_angles) {                   //If the IMU is already started
			angle_y = angle_y * 0.94 + angle_y_acc * 0.06; //Correct the drift of the gyro y angle with the accelerometer y angle
			angle_x = angle_x * 0.94 + angle_x_acc * 0.06; //Correct the drift of the gyro x angle with the accelerometer x angle
		} else {                                                //At first start
			angle_y = -1 * angle_y_acc; //Set the gyro y angle equal to the accelerometer y angle
			angle_x = angle_x_acc; //Set the gyro roll x equal to the accelerometer x angle
			set_gyro_angles = 1;                      //Set the IMU started flag
		}

		;;
		float result_y = -1 * angle_y;
		float result_x = 1 * angle_x;

		sprintf(strng, "X:%10lf  Y:%10lf \r", result_x, result_y);
		uart_print(console, (strng));
		uart_print(console, "\r");

		float LCD_X = (int) (result_x * 10) / 10.0;
		float LCD_Y = (int) (result_y * 10) / 10.0;
		sprintf(datax, "X:%10lf", LCD_X);
		sprintf(datay, "Y:%10lf", LCD_Y);
		Lcd_DisplayString(0, 0, datax);
		Lcd_DisplayString(1, 0, datay);

		int x_leds = (result_x) / 10;
		int y_leds = (result_y) / 10;
		if (x_leds >= 0) {
			gpio[SWPORTA_DR] = ((int) pow(2, abs(x_leds)) - 1) << 20;
		} else {
			gpio[SWPORTA_DR] = ((int) pow(2, abs(x_leds)) - 1) << 8;
		}
		if (y_leds >= 0) {
			gpio[SWPORTC_DR] = ((int) pow(2, abs(y_leds)) - 1) << 8;
		} else {
			gpio[SWPORTC_DR] = ((int) pow(2, abs(y_leds)) - 1) << 20;
		}
		while (read_auxreg(TIMER) < 175000) {
		}  //200 loops per second  175000CLK per loop
	}
	return 0;
}
/*****************************************************************************/
/* Breathing light */
/*****************************************************************************/
void LED_Breath(DWCREG_PTR gpio, int length) {
	length *= 100;
	int breath_length = 0;
	int delaytime = 34000;
	int add = -500;
	int times = 0;
	while (1) {
		write_auxreg(0, TIMER);
		gpio[SWPORTC_DR] = 0xff0ff00;
		gpio[SWPORTA_DR] = 0xff0ff00;
		delay(delaytime);
		gpio[SWPORTC_DR] = 0x0000000;  //off
		gpio[SWPORTA_DR] = 0x0000000;  //off
		while (read_auxreg(TIMER) <= 35000) {
		}
		times++;
		if (times >= 10)  //0.01sec //frequency
				{
			breath_length++;
			write_auxreg(0, TIMER);
			times = 0;
			delaytime += add;
			if (delaytime <= 1000) {
				delaytime = 1000;
				add *= -1;
			}
			if (delaytime >= 34000) {
				delaytime = 34000;
				add *= -1;
			}
		}
		if (breath_length > length)
			break;
	}
}
/*****************************************************************************/
/* Read I2C Gyro Sensor Mpu6050 */
/*****************************************************************************/
int Read_Gyro(DWCREG_PTR i2c) {
	char buf[14];
	uint8_t ra = 0x3B;     //set data register address
	i2c[I2C_DATA_CMD] = ra; //put destination register onto bus
	//read from sensor
	if (i2c_read(i2c, buf, 14, TIMEOUT) == 0) {
		acc_x = ((buf[0] << 8) | (buf[1]));
		acc_y = ((buf[2] << 8) | (buf[3]));
		acc_z = ((buf[4] << 8) | (buf[5]));
		gyro_x = ((buf[8] << 8) | (buf[9]));
		gyro_y = ((buf[10] << 8) | (buf[11]));
		gyro_z = ((buf[12] << 8) | (buf[13]));

		acc_x = (acc_x <= 32767) ? -1 * acc_x : -1 * acc_x + 65536;
		acc_y = (acc_y <= 32767) ? -1 * acc_y : -1 * acc_y + 65536;
		acc_z = (acc_z <= 32767) ? -1 * acc_z : -1 * acc_z + 65536;
		gyro_x = (gyro_x <= 32767) ? -1 * gyro_x : -1 * gyro_x + 65536;
		gyro_y = (gyro_y <= 32767) ? -1 * gyro_y : -1 * gyro_y + 65536;
		gyro_z = (gyro_z <= 32767) ? -1 * gyro_z : -1 * gyro_z + 65536;
	}
	return 0;
}
/*****************************************************************************/
/* Initialize I2C Gyro Sensor Mpu6050 */
/*****************************************************************************/
void gyro_init(DWCREG_PTR console, DWCREG_PTR i2c, DWCREG_PTR gpio) {
	//wake her up;
	uint8_t ra = 0x6B;
	uint8_t cmd = 0x00;
	char buf[2];
	buf[0] = ra;
	buf[1] = cmd;
	i2c_write(i2c, buf, 2);

	//initialize acc
	ra = 0x1c;
	cmd = 0x10;
	buf[0] = ra;
	buf[1] = cmd;
	i2c_write(i2c, buf, 2);

	//initialize gyro
	ra = 0x1B;
	cmd = 0x08;
	buf[0] = ra;
	buf[1] = cmd;
	i2c_write(i2c, buf, 2);

	int process = 0;
	//get gyro average offset
	for (int cal_int = 1; cal_int <= 4000; cal_int++) { //Run this code 4000 times
		if (Read_Gyro(i2c) == 0) {
			gyro_x_cal += gyro_x; //Add the gyro x-axis offset to the gyro_x_cal variable
			gyro_y_cal += gyro_y; //Add the gyro y-axis offset to the gyro_y_cal variable
			gyro_z_cal += gyro_z; //Add the gyro z-axis offset to the gyro_z_cal variable
		}
		process = cal_int / 500;
		process = pow(2, process) - 1;
		gpio[SWPORTA_DR] = process << 8 | process << 20;
		gpio[SWPORTC_DR] = process << 8 | process << 20;

		sprintf(strng, "\rProcess %4d / 4000\r", cal_int);
		uart_print(console, (strng));
		//uart_print(console, "\r");
		sprintf(strng, "%4d/4000\r\n", cal_int);
		Lcd_DisplayString(1, 0, strng);
		delay(50000);
	}
	gyro_x_cal /= 4000; //Divide the gyro_x_cal variable by 2000 to get the average offset
	gyro_y_cal /= 4000; //Divide the gyro_y_cal variable by 2000 to get the average offset
	gyro_z_cal /= 4000; //Divide the gyro_z_cal variable by 2000 to get the average offset
}
/*****************************************************************************/
/* Time Delay */
/*****************************************************************************/
void delay(unsigned int tics) {
	unsigned int cycles;
	write_auxreg(0, TIMER);
	do {
		cycles = read_auxreg(TIMER);
	} while (cycles < tics);
}
