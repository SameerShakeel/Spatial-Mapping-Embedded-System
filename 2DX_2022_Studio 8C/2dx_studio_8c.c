#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define MAXRETRIES              5           // number of receive attempts before giving up

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
// Initialized Port G for ToF sensor
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

// give clock to Port J and initalize as input GPIO
void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;                    // activate clock for Port J
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};    // allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;                                            // make PJ1 in 
  GPIO_PORTJ_DEN_R |= 0x02;                                             // enable digital I/O on PJ1
    
    GPIO_PORTJ_PCTL_R &= ~0x000000F0;                                     //  configure PJ1 as GPIO 
    GPIO_PORTJ_AMSEL_R &= ~0x02;                                            //  disable analog functionality on PJ1        
    GPIO_PORTJ_PUR_R |= 0x02;                                                    //    enable weak pull up resistor
}

// Initialized Port N1 for LED 1
void PortN1_Init(void){				//FROM MS1
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12; //activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};//allow time for clock to stabilize
	GPIO_PORTN_DIR_R=0b00000010; //Make PN1 output, to turn on LED's
	GPIO_PORTN_DEN_R=0b00000010; //Enable PN1
	
	return;
}

//	Initialized Port L for Waveforms
void PortL_Init(void){				//From MS1
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	// allow time for clock to stabilize
	GPIO_PORTL_DIR_R &= 0x04;        								// make PN0 out (PN0 built-in LED1)
	GPIO_PORTL_DIR_R |= 0x04; 
  GPIO_PORTL_AFSEL_R &= ~0x07;     								// disable alt funct on PN0
  GPIO_PORTL_DEN_R |= 0x07;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  GPIO_PORTL_AMSEL_R &= ~0x07;     								// disable analog functionality on PN0		
	return;
}

// Initialized Port M for stepper motor
void PortM_Init(void){				//From MS1
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0xFF;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTM_AFSEL_R &= ~0xFF;     								// disable alt funct on PN0
  GPIO_PORTM_DEN_R |= 0xFF;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  //GPIO_PORTM_PCTL_R = (GPIO_PORTM_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTM_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0		
	return;
}

void spin(int direction){
	// Created for loop to spin motor 5.625 degrees each turn, for a total of 5.625 * 64 = 360 degrees
	//Input of 1 is for counter clockwise,
	//Input of 0 is for clockwise
    for(int i=0; i<8; i++){
			if(direction == 1) { 
				GPIO_PORTM_DATA_R = 0b00001100;
        SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0b00000110;
        SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0b00000011;
        SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0b00001001;
        SysTick_Wait10ms(1);
      }
      else if(direction == 0) {
        GPIO_PORTM_DATA_R = 0b00001001;
        SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0b00000011;
        SysTick_Wait10ms(1);
				GPIO_PORTM_DATA_R = 0b00000110;
        SysTick_Wait10ms(1);
				GPIO_PORTM_DATA_R = 0b00001100;
        SysTick_Wait10ms(1);
      }
      else {
        GPIO_PORTH_DATA_R = 0b00000000;
        SysTick_Wait10ms(1);
			}        
    }
        GPIO_PORTM_DATA_R = 0b00000000;
}
//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t sensorState=0;
  uint16_t wordData;
  uint16_t Distance; 
  uint8_t dataReady;
	
	PortJ_Init(); //Initializing Port J for push buttons
	PortL_Init(); //Initializing Port L for AD2 data collection
	PortM_Init(); //Initializing Port M for stepper motor 
	PortN1_Init(); //Initializing Port N1 for PN1 LED D1
	PLL_Init();	//Initializing PLL
	SysTick_Init(); //Initializing SysTick
	onboardLEDs_Init(); //Initializing Onboard LEDs
	I2C_Init(); //Initializing I2C
	UART_Init(); //Initializing UART

	//Booting the Time of Flight (ToF) Sensor
	while(sensorState==0) {
		// Status code lines transmit UART data to microcontroller
		//UART communicates between Python and microcontroller
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
	}
	
	//Flash all the LEDs
	FlashAllLEDs();
	
	//Clear Interrupt to gather next interrupt
	status = VL53L1X_ClearInterrupt(dev);
	//Initialze sensor with default settings
	status = VL53L1X_SensorInit(dev);
	//Check status of the sensor
	Status_Check("SensorInit", status);

//	This for loop is used for outputting the waveforms for the AD2
//	If you would like to acquire the output to your AD2, then uncomment this for loop and reflash the program
//	for(int i = 0; i<24000000; i++){
//    GPIO_PORTL_DATA_R ^= 0b00000100;
//		SysTick_Wait(500);
//   }
	
		//Infinite while loop for when the program starts
		while(1){
			//Enable the correct ranging
			status = VL53L1X_StartRanging(dev);   														
			Again:
			
				//If statement that constantly checks if the push button PJ1 is ever pressed
				//If yes, it executes the program
				if((GPIO_PORTJ_DATA_R&0b00000010) == 0){
					
					//For loop that runs 64 times in order to get 64 measurements
					for(int i = 0; i < 64; i++) {																		
						//Setting the degrees based on the current iteration
						int degrees = 5.625*i;
						
						//While loop that waits until the time of flight's sensor data is ready
						while (dataReady == 0){																				
							status = VL53L1X_CheckForDataReady(dev, &dataReady);
							FlashLED3(1);
							VL53L1_WaitMs(dev, 5);
						}
						dataReady = 0;
	  
						//API Function call to get the distane measurement via I2C protocol
						status = VL53L1X_GetDistance(dev, &Distance);								
						//Flash LED D1 (PN1)
						FlashLED1(1);
						//Call ClearInterrupt
						status = VL53L1X_ClearInterrupt(dev);
						//Save the distance measurement as a double
						double displacement = Distance;
						//Print the reading in meters to the UART print buffer
						sprintf(printf_buffer,"%f\r\n", displacement/1000);
						//Print the stored measurement to the PC via UART
						UART_printf(printf_buffer);
					
						//Spin clockwise by 8 steps (5.625 degrees)
						spin(0);
						//Time delay
						SysTick_Wait10ms(1);
						
						//If statement that checks if PJ1 is ever pressed while it is collecting data
						//If yes, it returns back the same amount of steps it has currently travelled
						if((GPIO_PORTJ_DATA_R&0b00000010) == 0) {
                    for(int j = 0; j < i; j++) {
                        //Call spin funcion using counterclockwise direction input
												spin(1);
                    }
                    goto Again;
                }
					}
					
					//For loop that spins back counterclockwise to the original position to untangle the wires
					for(int i = 0; i<64;i++){
					spin(1);
					}
					VL53L1X_StopRanging(dev);
				}
		}
}

