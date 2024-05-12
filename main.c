#include <stdio.h>
#include<string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>


#define SPI_PORT GPIOA
#define SPI_GPIO_RCC RCC_GPIOA
#define SPI_RCC RCC_SPI1
#define SPI_NSS GPIO4
#define SPI_SCK GPIO5
#define SPI_MISO GPIO6
#define SPI_MOSI GPIO7

#define FLASH_CS_LOW gpio_clear(SPI_PORT, SPI_NSS)
#define FLASH_CS_HIGH gpio_set(SPI_PORT, SPI_NSS)

#define LCD_PORT GPIOB
#define LCD_RCC RCC_GPIOB

#define LCD_RS GPIO10
#define LCD_EN GPIO11
#define LCD_D4 GPIO12
#define LCD_D5 GPIO13
#define LCD_D6 GPIO14
#define LCD_D7 GPIO15

#define LCD_CMD 0
#define LCD_DATA 1

#define SENSOR_PORT GPIOC
#define SENSOR_RCC RCC_GPIOC
#define SENSOR_PIN GPIO0

#define DHT11_PIN GPIOA, GPIO0
#define DHT11_OK                0
#define DHT11_ERROR_CHECKSUM   -1
#define DHT11_ERROR_TIMEOUT    -2


SemaphoreHandle_t xMutex;

/* DHT11 Sensor used for Temperature and HUmidity sensor*/
int dht11_read_byte(void);
int dht11_read( uint8_t *temperature, uint8_t *humidity);

/* NOR FLASH MEMORY - W25Q64 is used which have SPI interface*/
void spi_setup(void);
void flash_write_int(int value, uint32_t addr);
void flash_read_int(int *value, uint32_t addr);


/**   **/
void lcd_setup(void);
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_send_string(const char *str);
void lcd_send_integer(int value);


int dht11_read_byte(void) {
    int i;
    int data = 0;
    for (i = 0; i < 8; i++) {
        while (!gpio_get(GPIOA,GPIO0));
        vTaskDelay(pdMS_TO_TICKS(30));
        if (gpio_get(GPIOA,GPIO0))
            data |= (1 << (7 - i));
        while (gpio_get(GPIOA,GPIO0));
    }
    return data;
}


int dht11_read( uint8_t *temperature, uint8_t *humidity) {
    uint8_t data[5] = {0, 0, 0, 0, 0};

    // Start signal
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
    gpio_clear(GPIOA,GPIO0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set(GPIOA,GPIO0);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);

    // Wait for DHT11 response
    if (!gpio_get(GPIOA,GPIO0)) {
        while (!gpio_get(GPIOA,GPIO0));  // DHT11 response low
        while (gpio_get(GPIOA,GPIO0));   // DHT11 response high

        // Read data
        for (int i = 0; i < 5; i++) {
            data[i] = dht11_read_byte();
        }

        // Checksum
        if (data[0] + data[1] + data[2] + data[3] == data[4]) {
            *humidity = data[0];
            *temperature = data[2];
            return DHT11_OK;
        } else {
            return DHT11_ERROR_CHECKSUM;
        }
    } else {
        return DHT11_ERROR_TIMEOUT;
    }
}


void spi_setup(void) {
    /* Enable SPI1 clock and GPIO clock for SPI pins */
    rcc_periph_clock_enable(SPI_RCC);
    rcc_periph_clock_enable(SPI_GPIO_RCC);

    /* Configure GPIOs: NSS, SCK, MISO, MOSI */
    gpio_set_mode(SPI_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, SPI_NSS | SPI_SCK | SPI_MOSI);
    gpio_set_mode(SPI_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, SPI_MISO);


    /* Set up SPI in Master mode with:
     * Clock baud rate: 1/64 of peripheral clock frequency
     * Clock polarity: Idle Low
     * Clock phase: Data valid on 1st clock transition
     * Data frame format: 8-bit
     * MSB transmitted first
     */
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

    /* Enable SPI1 */
    spi_enable(SPI1);
}

void lcd_command(uint8_t cmd) {
    gpio_clear(LCD_PORT, LCD_RS); // Command mode
    spi_xfer(SPI1, cmd);
    gpio_set(LCD_PORT, LCD_RS);
}

void lcd_data(uint8_t data) {
    gpio_set(LCD_PORT, LCD_RS); // Data mode
    spi_xfer(SPI1, data);
}

void lcd_setup(void) {
    /* Enable GPIO clock for LCD pins */
    rcc_periph_clock_enable(LCD_RCC);

    /* Configure GPIOs: RS, EN, D4, D5, D6, D7 */
    gpio_set_mode(LCD_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LCD_RS | LCD_EN | LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7);

    /* Initialization sequence */
    gpio_clear(LCD_PORT, LCD_RS);
    gpio_clear(LCD_PORT, LCD_EN);
    gpio_clear(LCD_PORT, LCD_D4);
    gpio_clear(LCD_PORT, LCD_D5);
    gpio_clear(LCD_PORT, LCD_D6);
    gpio_clear(LCD_PORT, LCD_D7);
    for (int i = 0; i < 3; i++) {
        gpio_set(LCD_PORT, LCD_EN);
        gpio_clear(LCD_PORT, LCD_EN);
    }
    lcd_command(0x28); // 4-bit mode, 2 lines, 5x8 font
    lcd_command(0x0C); // Display on, cursor off, blink off
    lcd_command(0x06); // Increment cursor (shift cursor to right)
    lcd_command(0x01); // Clear display
    lcd_command(0x80); // Move cursor to beginning of first line
}

void lcd_send_string(const char *str) {
    while (*str) {
        lcd_data(*str++);
    }
}

void lcd_send_integer(int value) {
    char buffer[10];
    sprintf(buffer, "%d", value);
    lcd_send_string(buffer);
}

void flash_write_int(int value, uint32_t addr) {
    while (SPI_SR(SPI1) & SPI_SR_BSY);
    FLASH_CS_LOW;
    spi_xfer(SPI1, 0x06); // Write Enable
    FLASH_CS_HIGH;
    while (SPI_SR(SPI1) & SPI_SR_BSY);
    FLASH_CS_LOW;
    spi_xfer(SPI1, 0x02); // Page Program
    spi_xfer(SPI1, (addr >> 16) & 0xFF); // Send address MSB first
    spi_xfer(SPI1, (addr >> 8) & 0xFF);
    spi_xfer(SPI1, addr & 0xFF);
    spi_xfer(SPI1, (value >> 24) & 0xFF);
    spi_xfer(SPI1, (value >> 16) & 0xFF);
    spi_xfer(SPI1, (value >> 8) & 0xFF);
    spi_xfer(SPI1, value & 0xFF);
    FLASH_CS_HIGH;
}

void flash_read_int(int *value, uint32_t addr) {
    FLASH_CS_LOW;
    spi_xfer(SPI1, 0x03); // Read Data
    spi_xfer(SPI1, (addr >> 16) & 0xFF); // Send address MSB first
    spi_xfer(SPI1, (addr >> 8) & 0xFF);
    spi_xfer(SPI1, addr & 0xFF);
    *value = 0;
    *value |= (spi_xfer(SPI1,0x00) << 24);
    *value |= (spi_xfer(SPI1,0x00) << 16);
    *value |= (spi_xfer(SPI1,0x00) << 8);
    *value |= spi_xfer(SPI1,0x00);
    FLASH_CS_HIGH;
}


static void vSensorDataWriteTask(void *args __attribute((unused))) 
{
    	uint8_t temp, hum;

	for(;;)
	{
		// Read data from sensor
	    if (dht11_read(&temp, &hum) == DHT11_OK) 
	    {
        	// Write data to NOR Flash Memory
     		   xSemaphoreTake(xMutex,portMAX_DELAY);

		   flash_write_int((int)temp, 0x000000);
     		   flash_write_int((int)hum, 0x000004);
	           
		   xSemaphoreGive(xMutex);

    	   	  vTaskDelay(pdMS_TO_TICKS(2000));
	    }

	}

}

static void vSensorDataReadTask(void *args __attribute((unused)))
{
	char ch;
    	uint8_t temp, hum;
	
	

	for(;;)
        {
	   xSemaphoreTake(xMutex,portMAX_DELAY);
	// Read data from NOR Flash Memory
	    flash_read_int((int*)&temp, 0x000000);
	    flash_read_int((int*)&hum, 0x000004);
	  xSemaphoreGive(xMutex);
    	// Display data on LCD
	
  //read data from usart and check if user want temperature or humidity	  
	  ch = usart_recv(USART1);
	  if(ch == 'T')
	  {
            lcd_command(0x01); // Clear display
	    lcd_command(0x80); // Move cursor to beginning of first line
	    lcd_send_string("Temp: ");
	    lcd_send_integer(temp);
	    lcd_send_string(" C");
	  }
 	 else if(ch == 'H')
	 {
           lcd_command(0x01); // Clear display
 	    lcd_command(0x80); // Move cursor to beginning of second line
 	    lcd_send_string("Humidity: ");
	    lcd_send_integer(hum);
	    lcd_send_string(" %");
	 }
	vTaskDelay(pdMS_TO_TICKS(100));
	}
}

static void
uart_setup(void) {

        rcc_periph_clock_enable(RCC_GPIOA);
        rcc_periph_clock_enable(RCC_USART1);

        // UART TX on PA9 (GPIO_USART1_TX)
        gpio_set_mode(GPIOA,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART1_TX);

        usart_set_baudrate(USART1,38400);
        usart_set_databits(USART1,8);
        usart_set_stopbits(USART1,USART_STOPBITS_1);
        usart_set_mode(USART1,USART_MODE_TX);
        usart_set_parity(USART1,USART_PARITY_NONE);
        usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);
        usart_enable(USART1);

}





int main(void) {

    char str[100] = "Enter T for Tempearture or H for humidity\n";
    size_t i=0;
    // Create the mutex
    xMutex = xSemaphoreCreateMutex();
  
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);  // Setup system clock
    //rcc_clock_setup_in_hse_8mhz_out_72mhz(); // Setup system clock

    spi_setup(); // Setup SPI
    lcd_setup(); // Setup LCD
    uart_setup(); // Setup UART

   for(i=0;i<strlen(str);i++)
   {	   
    usart_send(USART1,*(str+i));
   }
    // Create task to read data from dht11 sensor and write it to flash memory
    xTaskCreate(vSensorDataWriteTask, "SensorDataWriteTask", 500, NULL, tskIDLE_PRIORITY + 1, NULL);

    //create task t read data from flash memory and send it back to lcd
    xTaskCreate(vSensorDataReadTask, "SensorDataReadTask", 500, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Start the scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (1);

    return 0;
}

