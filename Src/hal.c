/**
 * \copyright Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "hal.h"
#include "gpio.h"
#include "spi.h"


//****************************************************************************
//
// Internal variables
//
//****************************************************************************

// Flag to indicate if a /DRDY interrupt has occurred
static volatile bool flag_nDRDY_INTERRUPT = false;



//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************
void InitGPIO(void);
void InitSPI(void);
void GPIO_DRDY_IRQHandler(void);



//****************************************************************************
//
// External Functions (prototypes declared in hal.h)
//
//****************************************************************************


//*****************************************************************************
//
//! Initializes MCU peripherals for interfacing with the ADC.
//!
//! \fn void InitADC(void)
//!
//! \return None.
//
//*****************************************************************************
void InitADC(void)
{
    // IMPORTANT: Make sure device is powered before setting GPIOs pins to HIGH state.

    // Initialize GPIOs pins used by ADS131M0x
    InitGPIO();

    // Initialize SPI peripheral used by ADS131M0x
    InitSPI();

    // Run ADC startup function
    adcStartup();
}




//****************************************************************************
//
// Timing functions
//
//****************************************************************************



//*****************************************************************************
//
//! Provides a timing delay with 'ms' resolution.
//!
//! \fn void delay_ms(const uint32_t delay_time_ms)
//!
//! \param delay_time_ms is the number of milliseconds to delay.
//!
//! \return None.
//
//*****************************************************************************
void delay_ms(const uint32_t delay_time_ms)
{
    /* --- INSERT YOUR CODE HERE --- */
    HAL_Delay(delay_time_ms);
}



//*****************************************************************************
//
//! Provides a timing delay with 'us' resolution.
//!
//! \fn void delay_us(const uint32_t delay_time_us)
//!
//! \param delay_time_us is the number of microseconds to delay.
//!
//! \return None.
//
//*****************************************************************************
void delay_us(const uint32_t delay_time_us)
{
	if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) { // Check if DWT counter is not already enabled
        // Enable TRC (Trace Control) for the DWT unit
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        // Enable the cycle counter
        DWT->CYCCNT = 0; // Clear the cycle counter
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable cycle counter
    }
	
	uint32_t microseconds = delay_time_us; // VOLATILE IM FALL DER F'LLE
	uint32_t clk_cycle_start = DWT -> CYCCNT;
 
  /* Go to number of cycles for system */
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
 
  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);	
}



//****************************************************************************
//
// GPIO initialization
//
//****************************************************************************



//*****************************************************************************
//
//! Configures the MCU's GPIO pins that interface with the ADC.
//!
//! \fn void InitGPIO(void)
//!
//! \return None.
//
//*****************************************************************************
void InitGPIO(void)
{
	MX_GPIO_Init();
	
    /* --- INSERT YOUR CODE HERE --- */
    // NOTE: Not all hardware implementations may control each of these pins...

    /* Enable the clock to the GPIO Port K and wait for it to be ready */
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
//    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)))
//    {
//    }

//    /* Configure the GPIO for 'nSYNC_nRESET' as output and set high */
//    MAP_GPIOPinTypeGPIOOutput(nSYNC_nRESET_PORT, nSYNC_nRESET_PIN);
//    MAP_GPIOPinWrite(nSYNC_nRESET_PORT, nSYNC_nRESET_PIN, nSYNC_nRESET_PIN);

//    /* Configure the GPIO for 'nCS' as output and set high */
//    MAP_GPIOPinTypeGPIOOutput(nCS_PORT, nCS_PIN);
//    MAP_GPIOPinWrite(nCS_PORT, nCS_PIN, nCS_PIN);

//    /* Enable the clock to the GPIO Port M and wait for it to be ready */
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
//    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)))
//    {
//    }

//    /* Configure the GPIO for 'nDRDY' as input with falling edge interrupt */
//    GPIOIntRegister(nDRDY_PORT, GPIO_DRDY_IRQHandler);
//    MAP_GPIOPinTypeGPIOInput(nDRDY_PORT, nDRDY_PIN);
//    MAP_GPIOIntTypeSet(nDRDY_PORT, nDRDY_PIN, GPIO_FALLING_EDGE);
//    MAP_GPIOIntEnable(nDRDY_PORT, nDRDY_PIN);
//    MAP_IntEnable(nDRDY_INT);
		
}




//*****************************************************************************
//
// Interrupt handler for nDRDY GPIO
//
//*****************************************************************************

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == N_DRDY_Pin) {
		GPIO_DRDY_IRQHandler();
	}
}

//*****************************************************************************
//
//! Interrupt handler for /DRDY falling edge interrupt.
//!
//! \fn void GPIO_DRDY_IRQHandler(void)
//!
//! \return None.
//
//*****************************************************************************
void GPIO_DRDY_IRQHandler(void)
{
        /* Interrupt action: Set a flag */
        flag_nDRDY_INTERRUPT = true;
}
//****************************************************************************
//
// GPIO helper functions
//
//****************************************************************************



//*****************************************************************************
//
//! Reads that current state of the /CS GPIO pin.
//!
//! \fn bool getCS(void)
//!
//! \return boolean ('true' if /CS is high, 'false' if /CS is low).
//
//*****************************************************************************

bool getCS(void)
{
    /* --- INSERT YOUR CODE HERE --- */
	return false; //false, da keine CS-Leitung angschlossen
    //return (bool) GPIOPinRead(nCS_PORT, nCS_PIN);
}


//*****************************************************************************
//
//! Reads that current state of the nSYNC/nRESET GPIO pin.
//!
//! \fn bool getSYNC_RESET(void)
//!
//! \return boolean ('true' if nSYNC/nRESET is high, 'false' if nSYNC/nRESET is low).
//
//*****************************************************************************
bool getSYNC_RESET(void)
{
    /* --- INSERT YOUR CODE HERE --- */
		return (bool) HAL_GPIO_ReadPin(GPIOA, 3); //SYNC/Reset an GPIO PA3
    //return (bool) GPIOPinRead(nSYNC_nRESET_PORT, nSYNC_nRESET_PIN);
}



//*****************************************************************************
//
//! Controls the state of the /CS GPIO pin.
//!
//! \fn void setCS(const bool state)
//!
//! \param state boolean indicating which state to set the /CS pin (0=low, 1=high)
//!
//! NOTE: The 'HIGH' and 'LOW' macros defined in hal.h can be passed to this
//! function for the 'state' parameter value.
//!
//! \return None.
//
//*****************************************************************************
void setCS(const bool state)  //do nothing as the is no CS-Selection between ADC und yC
{
//    /* --- INSERT YOUR CODE HERE --- */

//    // td(CSSC) delay
//    //if(state) { SysCtlDelay(2); }

//    uint8_t value = (uint8_t) (state ? nCS_PIN : 0);
//	
//    //MAP_GPIOPinWrite(nCS_PORT, nCS_PIN, value);

//    // td(SCCS) delay
//    if(!state) { SysCtlDelay(2); }
}



//*****************************************************************************
//
//! Controls the state of the nSYNC/nRESET GPIO pin.
//!
//! \fn void setSYNC_RESET(const bool state)
//!
//! \param state boolean indicating which state to set the nSYNC/nRESET pin (0=low, 1=high)
//!
//! NOTE: The 'HIGH' and 'LOW' macros defined in hal.h can be passed to this
//! function for the 'state' parameter value.
//!
//! \return None.
//
//*****************************************************************************
void setSYNC_RESET(const bool state)
{
    /* --- INSERT YOUR CODE HERE --- */
    //uint8_t value = (uint8_t) (state ? nSYNC_nRESET_PIN : 0);
    //MAP_GPIOPinWrite(nSYNC_nRESET_PORT, nSYNC_nRESET_PIN, value);
		HAL_GPIO_WritePin(GPIOA, 3, state);
}

//*****************************************************************************
//
//! Toggles the "nSYNC/nRESET" pin to trigger a synchronization
//! (LOW, delay 2 us, then HIGH).
//!
//! \fn void toggleSYNC(void)
//!
//! \return None.
//
//*****************************************************************************
void toggleSYNC(void)
{
    /* --- INSERT YOUR CODE HERE --- */
		HAL_GPIO_WritePin(GPIOA, 3, LOW);
    //MAP_GPIOPinWrite(nSYNC_nRESET_PORT, nSYNC_nRESET_PIN, 0);
    // nSYNC pulse width must be between 1 and 2,048 CLKIN periods
    delay_us(2);
	
		HAL_GPIO_WritePin(GPIOA, 3, HIGH);
    //MAP_GPIOPinWrite(nSYNC_nRESET_PORT, nSYNC_nRESET_PIN, nSYNC_nRESET_PIN);
}



//*****************************************************************************
//
//! Toggles the "nSYNC/nRESET" pin to trigger a reset
//! (LOW, delay 2 ms, then HIGH).
//!
//! \fn void toggleRESET(void)
//!
//! \return None.
//
//*****************************************************************************
void toggleRESET(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    //MAP_GPIOPinWrite(nSYNC_nRESET_PORT, nSYNC_nRESET_PIN, 0);
		HAL_GPIO_WritePin(GPIOA, 3, LOW);

    // Minimum /RESET pulse width (tSRLRST) equals 2,048 CLKIN periods (1 ms @ 2.048 MHz)
    delay_ms(2);

		HAL_GPIO_WritePin(GPIOA, 3, HIGH);
    //MAP_GPIOPinWrite(nSYNC_nRESET_PORT, nSYNC_nRESET_PIN, nSYNC_nRESET_PIN);

	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
    // tREGACQ delay before communicating with the device again
    delay_us(5);

    // NOTE: The ADS131M0x's next response word should be (0xFF20 | CHANCNT).
    // A different response may be an indication that the device did not reset.

	
	
    // Update register array
    restoreRegisterDefaults();

    // Write to MODE register to enforce mode settings
    writeSingleRegister(MODE_ADDRESS, MODE_DEFAULT);
}



//*****************************************************************************
//
//! Waits for the nDRDY interrupt or until the specified timeout occurs.
//!
//! \fn bool waitForDRDYinterrupt(const uint32_t timeout_ms)
//!
//! \param timeout_ms number of milliseconds to wait before timeout event.
//!
//! \return Returns 'true' if nDRDY interrupt occurred before the timeout.
//
//*****************************************************************************
bool waitForDRDYinterrupt(const uint32_t timeout_ms) // JLXYLKJLJK�KGMDSMM MM MDFGMDMGMFDSMGMSDFMGMDGFMDSMGMGMJFKDSLJFA
{
    /* --- INSERT YOUR CODE HERE ---
     * Poll the nDRDY GPIO pin until it goes low. To avoid potential infinite
     * loops, you may also want to implement a timer interrupt to occur after
     * the specified timeout period, in case the nDRDY pin is not active.
     * Return a boolean to indicate if nDRDY went low or if a timeout occurred.
     */

    // Convert ms to a # of loop iterations, OR even better use a timer here...
    uint32_t timeout = timeout_ms * 6000;   // convert to # of loop iterations
   	// Kann man mit HAL_getTick ersetzen
	
    // Reset interrupt flag
    flag_nDRDY_INTERRUPT = false;

    // Enable interrupts
    //IntMasterEnable();
	
    // Wait for nDRDY interrupt or timeout - each iteration is about 20 ticks
    do {
        timeout--;
    } while (!flag_nDRDY_INTERRUPT && (timeout > 0));
		
    // Reset interrupt flag
    flag_nDRDY_INTERRUPT = false;

    // Timeout counter greater than zero indicates that an interrupt occurred
    return (timeout > 0);
}




//****************************************************************************
//
// SPI Communication
//
//****************************************************************************

#define SSI_BASE_ADDR       (SSI3_BASE)



//*****************************************************************************
//
//! Configures the MCU's SPI peripheral, for interfacing with the ADC.
//!
//! \fn void InitSPI(void)
//!
//! \return None.
//
//*****************************************************************************
void InitSPI(void) 
{
    /* --- INSERT YOUR CODE HERE ---
     * NOTE: The ADS131M0x operates in SPI mode 1 (CPOL = 0, CPHA = 1).
     */
		MX_SPI1_Init();
	
//    //
//    // Enable the clock to SSI-3 module and configure the SSI Master
//    //
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
//    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3)))
//    {
//    }

//    //
//    // Enable clocks to GPIO Port Q and configure pins as SSI
//    //
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
//    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ)))
//    {
//    }

//    MAP_GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
//    //MAP_GPIOPinConfigure(GPIO_PA3_SSI0FSS); // Using GPIO for nCS instead of the FSS pin.
//    MAP_GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
//    MAP_GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);
//    MAP_GPIOPinTypeSSI(GPIO_PORTQ_BASE, (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3));

//    // Configure: SPI MODE 1, 5 MHz SCLK, 8-bits per frame
//    MAP_SSIConfigSetExpClk(SSI_BASE_ADDR, getSysClockHz(), SSI_FRF_MOTO_MODE_1,   \
//                           SSI_MODE_MASTER, (getSysClockHz()/24), 8);
//    //MAP_SSIEnable(SSI_BASE_ADDR);

//    //
//    // Enable the SSI2 module.
//    //
//    SSIEnable(SSI_BASE_ADDR);
//    SSIAdvModeSet(SSI_BASE_ADDR, SSI_ADV_MODE_READ_WRITE);
//    SSIAdvFrameHoldDisable(SSI_BASE_ADDR);

//    //
//    // Read any residual data from the SSI port.  This makes sure the receive
//    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
//    // because the SPI SSI mode is full-duplex, which allows you to send and
//    // receive at the same time.  The SSIDataGetNonBlocking function returns
//    // "true" when data was returned, and "false" when no data was returned.
//    // The "non-blocking" function checks if there is any data in the receive
//    // FIFO and does not "hang" if there isn't.
//    //
//    uint32_t junk;
//    while(MAP_SSIDataGetNonBlocking(SSI_BASE_ADDR, &junk));
}



//*****************************************************************************
//
//! Sends SPI byte array on MOSI pin and captures MISO data to a byte array.
//!
//! \fn void spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength)
//!
//! \param const uint8_t dataTx[] byte array of SPI data to send on MOSI.
//!
//! \param uint8_t dataRx[] byte array of SPI data captured on MISO.
//!
//! \param uint8_t byteLength number of bytes to send & receive.
//!
//! NOTE: Make sure 'dataTx[]' and 'dataRx[]' contain at least as many bytes of data,
//! as indicated by 'byteLength'.
//!
//! \return None.
//
//*****************************************************************************
void spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength) /// FJHDLKJFSHGFDSKJHGLKFDSHGDFHGDSKJFHFDSJG
{
    /*  --- INSERT YOUR CODE HERE ---
     *
     *  This function should send and receive multiple bytes over the SPI.
     *
     *  A typical SPI send/receive sequence may look like the following:
     *  1) Make sure SPI receive buffer is empty
     *  2) Set the /CS pin low (if controlled by GPIO)
     *  3) Send command bytes to SPI transmit buffer
     *  4) Wait for SPI receive interrupt
     *  5) Retrieve data from SPI receive buffer
     *  6) Set the /CS pin high (if controlled by GPIO)
     */

    // Require that dataTx and dataRx are not NULL pointers
    assert(dataTx && dataRx);

    // Set the nCS pin LOW
    setCS(LOW);

    // Send all dataTx[] bytes on MOSI, and capture all MISO bytes in dataRx[]
    int i;
    for (i = 0; i < byteLength; i++)
    {
        dataRx[i] = spiSendReceiveByte(dataTx[i]);
    }

    // Set the nCS pin HIGH
    setCS(HIGH);
}



//*****************************************************************************
//
//! Sends SPI byte on MOSI pin and captures MISO return byte value.
//!
//! \fn uint8_t spiSendReceiveByte(const uint8_t dataTx)
//!
//! \param const uint8_t dataTx data byte to send on MOSI pin.
//!
//! NOTE: This function is called by spiSendReceiveArrays(). If it is called
//! directly, then the /CS pin must also be directly controlled.
//!
//! \return Captured MISO response byte.
//
//*****************************************************************************
// moglichkeiten zur besseren Fehlererkennung
bool HAL_DataGetNonBlocking (const uint32_t SP1_Base_Adr, uint32_t * junkk){
	
  HAL_StatusTypeDef status;
	bool sts;
	uint8_t puffer;
	
	//SPI_FLAG_RXNE gibt an, ob neue SPI-Daten verf�gbar sind
	if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE)) //zeigt an, ob neue Daten zum lesen vorhanden sind
        {
            // Empfange Daten, wenn verf�gbar
            status = HAL_SPI_Receive(&hspi1, &puffer, 1, HAL_MAX_DELAY);
						*junkk = (uint32_t) puffer;
				
            if (status == HAL_OK)
            {
                sts = 1;
            }
            else
            {
                sts = 0;
            }
        }
				return sts;
}

uint8_t spiSendReceiveByte(uint8_t dataTx) 
{
    /*  --- INSERT YOUR CODE HERE ---
     *  This function should send and receive single bytes over the SPI.
     *  NOTE: This function does not control the /CS pin to allow for
     *  more programming flexibility.
     */

    // Remove any residual or old data from the receive FIFO
    uint32_t junk;
    while (HAL_DataGetNonBlocking(SPI1_BASE, &junk));

    // SSI TX & RX
    uint8_t dataRx;
	
		HAL_SPI_Transmit(&hspi1, &dataTx, 1, HAL_MAX_DELAY);
    //MAP_SSIDataPut(SSI_BASE_ADDR, (uint32_t) dataTx);
		HAL_SPI_Receive(&hspi1, &dataRx, 1, HAL_MAX_DELAY);
    //(SSI_BASE_ADDR, (uint32_t *) &dataRx);

    return dataRx;
}


