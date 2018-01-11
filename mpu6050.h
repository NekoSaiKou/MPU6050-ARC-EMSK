#define I2C_CTRL_ADDR   	(DWCREG_PTR) (DWC_I2C_0        	| PERIPHERAL_BASE)
#define CREG_CTRL_ADDR 		(DWCREG_PTR) (REG_FILE_0       	| PERIPHERAL_BASE)
#define CONSOLE_CTRL_ADDR	(DWCREG_PTR) (DWC_UART_CONSOLE 	| PERIPHERAL_BASE)
#define GPIO_CTRL_ADDR		(DWCREG_PTR) (DWC_GPIO_0       	| PERIPHERAL_BASE)
#define SPIM_CTRL_ADDR		(DWCREG_PTR) (DWC_SPI_0     	| PERIPHERAL_BASE)

#define I2C_SLAVE_ADDRESS  0x68			// I2C address of MPU6050 can be selected via jumpers: 0x68 0x69,default is 0x68
#define I2C_DATA_CMD          4  		// I2C Rx/Tx Data Buffer and Command Register
#define TIMEOUT            0x10000
#define TIMER 0x100 					// TCOUNT0 = 0x021, TCOUNT1 = 0x100 35000000CLK per second
