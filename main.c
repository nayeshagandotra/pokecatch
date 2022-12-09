/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PWM Square Wave code example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"
#include "cycfg.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


/*******************************************************************************
* Macros
*******************************************************************************/

/* Encoder count timer clock value in Hz */
#define retrieve_count_timer_CLOCK_HZ (10000)

/* Encoder clock timer period value */
#define retrieve_count_timer_PERIOD (4999)

/* reset just fired timer clock value in Hz */
#define reset_just_fired_timer_CLOCK_HZ (10000) //10kHz clock

/* reset just fired timer period value */
#define reset_just_fired_timer_PERIOD (50000)

/* PWM Frequency = 50Hz */
#define PWM_FREQUENCY (50u)
/* PWM Duty-cycle = 5% */
#define PWM_DUTY_CYCLE (0.0f)

#define VPLUS_CHANNEL_0             (P10_0)
#define VPLUS_CHANNEL_1				(P10_1)

#define VPLUS_CHANNEL_2				(P5_5)
#define VPLUS_CHANNEL_3				(P5_6)

#define SERVO_CHANNEL_0				(P10_4)
#define SERVO_CHANNEL_1				(P10_3)

/* Channel 1 VREF input pin */
#define VREF_CHANNEL_1_SERVO_0      (P10_2)
#define VREF_CHANNEL_1_SERVO_1		(P5_2)

/* Pins for PDL */
/* P9_0 & P9_1 */

/* Number of scans every time ADC read is initiated */
#define NUM_SCAN                    (1)

#define TIMER_INTERRUPT_PRIORITY_ENCODER		   (7u)
#define GPIO_INTERRUPT_PRIORITY_BUTTON_trigger     (6u)
#define GPIO_INTERRUPT_PRIORITY_LASER	   		   (4u)
#define TIMER_INTERRUPT_PRIORITY		   		   (5u)

enum ADC_CHANNELS_1
{
  CHANNEL_0 = 0,
  CHANNEL_1,
  NUM_CHANNELS_1
} adc_channel_1;

/* Multichannel initialization function */
void adc_multi_channel_init_1(void);

/* Function to read input voltage from multiple channels */
void adc_multi_channel_process_1(void);


/* ADC Event Handler */
static void adc_event_handler_1(void* arg, cyhal_adc_event_t event);

/* Function proto for button */
static void gpio_interrupt_handler_button_trigger(void *handler_arg, cyhal_gpio_event_t event);

/* Function proto for Encoder */
void retrieve_count_timer_init(void);
static void isr_retrieve_count_timer(void *callback_arg, cyhal_timer_event_t event);
void handle_error(void);
void start_game(void);
void stop_game(void);

static void gpio_interrupt_handler_laser(void *handler_arg, cyhal_gpio_event_t event);


void timer_init(void);

static void isr_timer(void *callback_arg, cyhal_timer_event_t event);
static void isr_reset_just_fired_timer(void *callback_arg, cyhal_timer_event_t event);
void reset_just_fired_timer_init(void);

static float map(float x, float in_min, float in_max, float out_min, float out_max) ;
static void shoot(void);
float adc_averager(float pot_array[10]);


/* ADC Object */
cyhal_adc_t adc_obj_1;
cyhal_adc_t adc_obj_2;

/* ADC Channel 0 Object */
cyhal_adc_channel_t adc_chan_0_obj_1;
cyhal_adc_channel_t adc_chan_0_obj_2;


/* Default ADC configuration */
const cyhal_adc_config_t adc_config = {
        .continuous_scanning=false, // Continuous Scanning is disabled
        .average_count=1,           // Average count disabled
        .vref=CYHAL_ADC_REF_VDDA,   // VREF for Single ended channel set to VDDA
        .vneg=CYHAL_ADC_VNEG_VSSA,  // VNEG for Single ended channel set to VSSA
        .resolution = 12u,          // 12-bit resolution
        .ext_vref = NC,             // No connection
        .bypass_pin = NC };       // No connection

/* Asynchronous read complete flag, used in Event Handler */
static bool async_read_complete = false;

/* ADC Channel 1 Object */
cyhal_adc_channel_t adc_chan_1_obj_1;

/* Variable to store results from multiple channels during asynchronous read*/
int32_t result_arr_1[NUM_CHANNELS_1 * NUM_SCAN] = {0};


uint8_t uart_read_value;

bool launch = false;

volatile bool gpio_intr_flag = false;

cyhal_timer_t timer;
cyhal_pwm_t pwm_trigger;

#define ECHO_TIMER_CLOCK_HZ          (12000000)
#define LED_BLINK_TIMER_PERIOD            (9999)

bool timer_interrupt_flag = false;

cy_rslt_t result;

/* PWM objects */
cyhal_pwm_t servo_trigger;
cyhal_pwm_t servo_0;
cyhal_pwm_t servo_1;

/* Initialize pot values */
float pot0_val;
float pot1_val;

static bool human_bool = false;
static bool robot_bool = false;
static bool proj_hit = false;
static bool first_iter = false;
static bool	has_shot = false;


static int human_score = 0;
static int robot_score = 0;

/* Encoder global variables */
uint16_t encoder_A_count = 32768;
uint16_t encoder_A_count_prev = 32768;
uint16_t count_change = 0;
uint16_t capture = 0;
uint16_t track_width = 2430;
uint16_t start_of_track = 32765;
uint16_t distance_traveled = -1;
int  count_fired = 0;

/* Interrupt/Button Flags */
volatile bool game_on_flag = false; //start game button, game is initially off

volatile bool retrieve_count_flag = false; //flag for encoder count interrupt

static bool pwm_cw = true;
volatile bool counterclockwise = false;
static bool continuous = true;

int16_t velocity_A = 0;

/* Timer object used for retrieving encoder counts */
cyhal_timer_t retrieve_count_timer;

/* Timer object used for resetting just fired */
cyhal_timer_t reset_just_fired_timer;

/*initializing ping and pong arrays */
int16_t ping[100];
int16_t pong[100];

/*initializing a buffer which contains both ping and pong arrays */
int16_t* buffer[2] = {ping, pong};

/*index indicating which array is being written into. 0 = ping */
int index_buf = 0;

/*index of element inside ping/pong array */
int index_pp = 0;

/*booleans indicating whether ping or pong is being written into */
static bool writing_ping = true;

volatile bool reset_just_fired_flag = false;


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for the CPU. It configures the PWM and puts the CPU
* in Sleep mode to save power.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
	cyhal_pwm_t servo_0;
	cyhal_pwm_t servo_1;

    result = cybsp_init();
   /* Initialize the device and board peripherals */       /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init( CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE );

    CY_ASSERT( result == CY_RSLT_SUCCESS );

    printf( "Program started...\r\n" );

    __enable_irq();


    /* Initialize the PWMs */
    result = cyhal_pwm_init(&servo_0, SERVO_CHANNEL_0, NULL); // Servo 0
    result = cyhal_pwm_init(&servo_1, SERVO_CHANNEL_1, NULL); // Servo 1
    result = cyhal_pwm_init(&servo_trigger, P12_6, NULL); //Servo trigger

    /************************************************************************
    * Motor Initializing
    ************************************************************************/
    /*clockwise*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(TCPWM1, motor_a_cw_NUM, &motor_a_cw_config))
	{
		/* Handle possible errors */
		handle_error();
	}

	/* Enable the initialized PWM */
	Cy_TCPWM_PWM_Enable(TCPWM1, motor_a_cw_NUM);

	/*ccw*/
	if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(TCPWM1, motor_a_ccw_NUM, &motor_a_ccw_config))
	{
		/* Handle possible errors */
		handle_error();
	}

	/* Enable the initialized PWM */
	Cy_TCPWM_PWM_Enable(TCPWM1, motor_a_ccw_NUM);

	/**********************************************************************************
	 * Decoder/Encoder Setup and Start
	 *********************************************************************************/
	/*  Initialize Pins for Motor A Encoder */
	result = Cy_GPIO_Pin_Init(phi_A_1_PORT, phi_A_1_NUM, &phi_A_1_config); //P9[0]
	if (result != CY_RSLT_SUCCESS) {
			CY_ASSERT(0);
			printf("Initializing P9[0] failed\r\n"); //for testing purposes only
	}
	result = Cy_GPIO_Pin_Init(phi_A_2_PORT, phi_A_2_NUM, &phi_A_2_config); //P9[1]
	if (result != CY_RSLT_SUCCESS) {
				CY_ASSERT(0);
				printf("Initializing P9[1] failed\r\n"); //for testing purposes only
	}
	/* Initialize PWM for Quadrature Decoder A */
	result = Cy_TCPWM_QuadDec_Init(Dec_A_HW, Dec_A_NUM, &Dec_A_config); //16 bit Counter 2
	if (result != CY_RSLT_SUCCESS) {
				CY_ASSERT(0);
	}

	/* Enable PWM for decoder A */
	Cy_TCPWM_QuadDec_SetCounter(Dec_A_HW, Dec_A_NUM, 0);
	Cy_TCPWM_QuadDec_Enable(Dec_A_HW, Dec_A_NUM); //Decoder A
	Cy_TCPWM_QuadDec_SetCounter(Dec_A_HW, Dec_A_NUM, 32768);


    /* Set the PWM output frequency and duty cycle */
    result = cyhal_pwm_set_duty_cycle(&servo_0, PWM_DUTY_CYCLE, PWM_FREQUENCY);
    result = cyhal_pwm_set_duty_cycle(&servo_1, PWM_DUTY_CYCLE, PWM_FREQUENCY);
    result = cyhal_pwm_set_duty_cycle(&servo_trigger, 5, PWM_FREQUENCY);

    adc_multi_channel_init_1();
    result = cyhal_adc_configure(&adc_obj_1, &adc_config);

    if(result != CY_RSLT_SUCCESS)
        {
            printf("ADC configuration update failed. Error: %ld\n", (long unsigned int)result);
            CY_ASSERT(0);
        }

    /* Initializing GPIO button start game, robot play, human play */
    //result = cyhal_gpio_init(**,CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    //result = cyhal_gpio_init(**,CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    //result = cyhal_gpio_init(**,CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);

	/* Initializing GPIO button trigger */
    // Need to update on board
	result = cyhal_gpio_init(P7_2,CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);

	/* Initializing GPIO laser output*/
	result = cyhal_gpio_init(P9_5, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_BTN_OFF);

	/* Initializing GPIO laser input*/
	result = cyhal_gpio_init(P9_4, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, CYBSP_BTN_OFF);

	/* Configures timer*/
    const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0,                  // Timer compare value, not used
        .period        = 23050,               // Defines the timer period
        .direction     = CYHAL_TIMER_DIR_UP, // Timer counts up
        .is_compare    = false,              // Don't use compare mode
        .is_continuous = false,               // Run the timer indefinitely
        .value         = 0                   // Initial value of counter
    };

    /* Initialize timer */
    result = cyhal_timer_init(&timer, NC, NULL);

    /* Apply timer configuration such as period, count direction, run mode, etc. */
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_timer_configure(&timer, &timer_cfg);
    }

    /* Apply frequency of 10kHz */
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_timer_set_frequency(&timer, 10000);
    }

    /* Configure timer interrupt */
    cyhal_timer_register_callback(&timer,isr_timer, NULL);
    cyhal_timer_enable_event(&timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, TIMER_INTERRUPT_PRIORITY, true);

    /* Configure GPIO interrupt (button - trigger)*/
    cyhal_gpio_register_callback(P7_2, gpio_interrupt_handler_button_trigger, NULL);
    cyhal_gpio_enable_event(P7_2, CYHAL_GPIO_IRQ_RISE, GPIO_INTERRUPT_PRIORITY_BUTTON_trigger, true);

    /* Configure GPIO interrupt (Laser) */
    cyhal_gpio_register_callback(P9_4, gpio_interrupt_handler_laser, NULL);
    cyhal_gpio_enable_event(P9_4, CYHAL_GPIO_IRQ_RISE, GPIO_INTERRUPT_PRIORITY_LASER, true);



    printf("All initiations are complete.\r\n");

    float pot0_val_old = 0;
    float pot1_val_old = 0;

    first_iter = true;

    /***************************************************************************************
     * Turn on PWM's for motor and decoder
    ***************************************************************************************/
   /*Turn on PWM to Start decoder A */
    Cy_TCPWM_TriggerReloadOrIndex_Single(Dec_A_HW, Dec_A_NUM);
	retrieve_count_timer_init();
	int counter = 0;
	float pot_values_0[10];
	float pot_values_1[10];

	float pot0_val_avg;
	float pot1_val_avg;

    /* Loop infinitely */
    for (;;)
    {
    		if (human_bool == true)
    		{
				//printf("Laser input: %d\r\n", cyhal_gpio_read(P9_4));
				//cyhal_system_delay_ms(20);

				adc_multi_channel_process_1();

				pot_values_0[counter] = pot0_val;
				pot_values_1[counter] = pot1_val;

				if(counter == 9)
				{
					pot0_val_avg = adc_averager(pot_values_0);
					pot1_val_avg = adc_averager(pot_values_1);

				    /* Round values to nearest hundredth*/
					pot0_val_avg = round(pot0_val_avg*10)/10;
					pot1_val_avg = round(pot1_val_avg*10)/10;

					//printf("pot0_val_avg: %4f \t pot0_val: %4f\r\n", pot0_val_avg,pot0_val);

					if (pot0_val_old != pot0_val_avg)
					{
						result = cyhal_pwm_set_duty_cycle(&servo_0, pot0_val_avg, PWM_FREQUENCY);
						result = cyhal_pwm_start(&servo_0);
					}
					if (pot1_val_old != pot1_val_avg)
					{
						result = cyhal_pwm_set_duty_cycle(&servo_1, pot1_val_avg, PWM_FREQUENCY);
						result = cyhal_pwm_start(&servo_1);
					}
					pot0_val_old = pot0_val_avg;
					pot1_val_old = pot1_val_avg;
					counter = 0;
				}
				counter++;
				cyhal_system_delay_ms(10);

    		}
    	if (cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_read_value, 1) == CY_RSLT_SUCCESS) {
    	    	  switch(uart_read_value) {
    	    	  	  case 'S':
    	    	  		  if
    	    	  		  (game_on_flag) {
    	    	  			  //if game is already on
    	    	  			  printf("OGame is already on \r\n");
    	    	  		  }
    	    	          else  { //start game
    					  //add functionality of which dir to turn in based on current position
    					  printf("OGame is triggered on DC Motors start\r\n"); //for testing purposes only
    						/* Initialize timer interrupt */
    					  	retrieve_count_timer_init(); //add functionality in this to change motor direction
    					  	reset_just_fired_timer_init();
    	    	        	start_game();
    	    	          }
    	    	          game_on_flag = true;
    	    	          cyhal_system_delay_ms(1);
    	    	          break;
    	    	  	  case 'X':
    	    	  		  if (game_on_flag == 0) {
    	    	  			  //if game is already off
    	    	  	          printf("FGame is already off\r\n");
    	    	  		  }
    	    	  	      else  { //turn off game
    	    	  	          printf("FGame is triggered off\r\n"); //for testing purposes only
    	    	  	          stop_game();
    	    	  	      }
    	    	  	      game_on_flag = false;
    	    	  	      break;
    	    	  	case 'I':
    						printf("IPSOC\r\n");
    						break;
    	    	  	case 'D':
    	    	  		if (game_on_flag) {
    	    	  			if (writing_ping) {
    							/*send pong*/
    							printf("b%d,%d,%d,%d,%d,%d,%d,%d,%d,%de\n", pong[0], pong[1], pong[2], pong[3], pong[4], pong[5], pong[6], pong[7], pong[8], pong[9], pong[10]);
    							index_buf = 1;

    						} else {
    							/*send ping*/
    							index_buf = 0;
    							printf("b%d,%d,%d,%d,%d,%d,%d,%d,%d,%de\n", ping[0], ping[1], ping[2], ping[3], ping[4], ping[5], ping[6], ping[7], ping[8], ping[9], ping[10]);

    						}
    						index_pp = 0;
    						writing_ping = !writing_ping;
    	    	  		}
    	    	  			break;
    	    	  	case 'H':
    	    	  		if (game_on_flag) {
    	    	  			printf("Human mode selected\n");
    	    	  			human_bool = true;
    	    	  		}
    	    	  		else
    	    	  		{
    	    	  			printf("Please press the start button\n");
    	    	  		}
    	    	  		break;
    	    	  	case 'R':
    	    	  		if(game_on_flag){
    	    	  			printf("Robot mode selected\n");
    	    	  			robot_bool = true;
    	    	  		}
    	    	  		else
    	    	  		{
    	    	  			printf("Please press the start button\n");
    	    	  		}
    	    	  		break;

    	    	  	  default:
    	    	  		  break;
    	    	  }
    	    	}

    	if (retrieve_count_flag) {
    	 //if interrupt to retrieve encoder count has been triggered
		retrieve_count_flag = false; //clear flag
		//printf("Capture value: %d\r\n", capture);
    	}


    }
}

void adc_multi_channel_init_1(void)
{
    /* Variable to capture return value of functions */
    cy_rslt_t result;

    /* Initialize ADC. The ADC block which can connect to pin 10[0] is selected */
    result = cyhal_adc_init(&adc_obj_1, VPLUS_CHANNEL_0, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("ADC initialization failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    /* ADC channel configuration */
    const cyhal_adc_channel_config_t channel_config = {
            .enable_averaging = true,  // Disable averaging for channel
            .min_acquisition_ns = 50000, // Minimum acquisition time set to 1us
            .enabled = true };          // Sample this channel when ADC performs a scan

    /* Initialize a channel 0 and configure it to scan P10_0 in single ended mode. */
    result  = cyhal_adc_channel_init_diff(&adc_chan_0_obj_1, &adc_obj_1, VPLUS_CHANNEL_0,
    		 P10_5, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("ADC first channel initialization failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    /*
     * For multichannel configuration use to same channel configuration structure
     * "channel_config" to configure the second channel.
     * For second channel to be set to differential mode, two inputs from Pins
     * 10.1 and 10.2 are configured to be inputs.
     *
     */

    /* Initialize channel 1 in differential mode with VPLUS and VMINUS input pins */
    result = cyhal_adc_channel_init_diff(&adc_chan_1_obj_1, &adc_obj_1, VPLUS_CHANNEL_1,
    		P10_2, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("ADC second channel initialization failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    /* Register a callback to handle asynchronous read completion */
     cyhal_adc_register_callback(&adc_obj_1, &adc_event_handler_1, result_arr_1);

     /* Subscribe to the async read complete event to process the results */
     cyhal_adc_enable_event(&adc_obj_1, CYHAL_ADC_ASYNC_READ_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);

     printf("ADC is configured in multichannel configuration.\r\n\n");
     printf("Channel 0 is configured in single ended mode, connected to pin \r\n");
     printf("P10_0. Provide input voltage at P10_0\r\n");
     printf("Channel 1 is configured in differential mode, connected to pin \r\n");
     printf("P10_1 and P10_2. Provide input voltage at P10_1 and reference \r\n");
     printf("voltage at P10_2\r\n\n");
}

void adc_multi_channel_process_1(void)
{
	/* Initialize array */
	//static float result_array[1];

    /* Variable to store ADC conversion result from channel 0 */
    float adc_result_0 = 0;

    /* Variable to store ADC conversion result from channel 1 */
    float adc_result_1 = 0;

    /* Initiate an asynchronous read operation. The event handler will be called
     * when it is complete. */
    result = cyhal_adc_read_async_uv(&adc_obj_1, NUM_SCAN, result_arr_1);

    if(result != CY_RSLT_SUCCESS)
    {
        printf("ADC async read failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    /* Read data from result list, input voltage in the result list is in microvolts. */

    adc_result_0 = result_arr_1[CHANNEL_0]/1000;
    adc_result_1 = result_arr_1[CHANNEL_1]/1000;

    /* Clear async read complete flag */
    async_read_complete = false;

    /* Calculates duty cycle value based on experimental parameters */
    pot1_val = ((adc_result_1/330) - 2) * -1;
    pot0_val = (float) ((adc_result_0/330.0) - 2.0) * -1;

    /* Override and map the pot values */
    pot1_val = map(pot1_val,2,12,3.3,4.4);
    pot0_val = map(pot0_val,2,12,5,8);


    //printf("pot0_val: %4f \t pot1_val: %4f \t pot0_val_before: %4f\r\n",pot0_val,pot1_val,  (float) adc_result_0/330);
    /*printf("pot0_val: %4f \t pot1_val: %4f "
    		"\t pot0_val_round: %4f \t pot1_val_round: %4f\r\n",
    		pot0_val,pot1_val, round(pot0_val*10)/10, round(pot1_val*10)/10);*/
}

static void adc_event_handler_1(void *arg, cyhal_adc_event_t event)
{
    if(0u != (event & CYHAL_ADC_ASYNC_READ_COMPLETE))
    {
        /* Set async read complete flag to true */
        async_read_complete = true;
    }
}

static void gpio_interrupt_handler_button_trigger(void *handler_arg, cyhal_gpio_irq_event_t event)
{
	if(human_bool == true)
	{
		gpio_intr_flag = true;
		printf("A45\n");
		printf("F\n");
		printf("Pot0 value at button interrupt: %4f\r\n", pot0_val);
		printf("Pot1 value at button interrupt: %4f\r\n", pot1_val);

		result = cyhal_pwm_disable_output(&servo_0, CYHAL_PWM_OUTPUT_LINE_OUT);
		result = cyhal_pwm_disable_output(&servo_1, CYHAL_PWM_OUTPUT_LINE_OUT);

		result = cyhal_pwm_start(&servo_trigger);
		result = cyhal_timer_start(&timer);
		has_shot = true;
		cyhal_system_delay_ms(200);
	}


}

static void gpio_interrupt_handler_laser(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    gpio_intr_flag = true;
	printf("L1\r\n");
	proj_hit = true;
	if(human_bool == true)
	{
		human_score += 1;
	}
	else
	{
		robot_score += 1;
	}
}


void retrieve_count_timer_init(void) {
	cy_rslt_t result;

	const cyhal_timer_cfg_t retrieve_count_timer_cfg = {
		.compare_value = 0,                  //Timer compare value, not used
		.period = retrieve_count_timer_PERIOD,   // Defines the timer period
		.direction = CYHAL_TIMER_DIR_UP,    // Timer counts up
		.is_compare = false,               //  Don't use compare mode
		.is_continuous = true,             //  Run timer indefinitely
		.value = 0                         //  Initial value of counter
	};

	/* Initialize the timer object. Does not use input pin ('pin' is NC) and
	* does not use a pre-configured clock source ('clk' is NULL).*/
	result = cyhal_timer_init(&retrieve_count_timer, NC, NULL);

	/* timer init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS) {
	        CY_ASSERT(0);
	}

	/* Configure timer period and operation mode such as count direction,
	duration */
	cyhal_timer_configure(&retrieve_count_timer, &retrieve_count_timer_cfg);

	/* Set the frequency of timer's clock source */
	cyhal_timer_set_frequency(&retrieve_count_timer, retrieve_count_timer_CLOCK_HZ);

	/* Assign the ISR to execute on timer interrupt */
	cyhal_timer_register_callback(&retrieve_count_timer, isr_retrieve_count_timer, NULL);

	 /* Set the event on which timer interrupt occurs and enable it */
	cyhal_timer_enable_event(&retrieve_count_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
					TIMER_INTERRUPT_PRIORITY_ENCODER, true); //HIGH PRIORITY SINCE DATA DEPENDS ON TIME OF EXECUTION

}

static void isr_retrieve_count_timer(void *callback_arg, cyhal_timer_event_t event) {
    (void) callback_arg;
    (void) event;
    encoder_A_count_prev = encoder_A_count; //set last count as the previous count
    encoder_A_count = Cy_TCPWM_QuadDec_GetCounter(Dec_A_HW, Dec_A_NUM);
    count_change = encoder_A_count - encoder_A_count_prev;
    distance_traveled = start_of_track-encoder_A_count;


	/*
	 * write value into appropriate buffer and incrementing
	 * */
    if (writing_ping) /*write value into ping*/ {
		buffer[index_buf][index_pp] = encoder_A_count;
	} else /* write value into pong*/{
		buffer[index_buf][index_pp] = encoder_A_count;
	}
    index_pp++;
    capture++;

    //change the motor direction based on what the encoder count is
    if (encoder_A_count <= 32768 && encoder_A_count > 32700 && capture >=50) {
    	if (pwm_cw) {
			/*stop the cw pwm signal and start the ccw pwm signal*/
			 Cy_TCPWM_TriggerStopOrKill(TCPWM1, motor_a_cw_MASK);
			 Cy_TCPWM_TriggerReloadOrIndex(TCPWM1, motor_a_ccw_MASK);
			pwm_cw = false;
			printf( "turning counter clockwise \r\n"); /*debugging only*/

		} else {
			/*stop ccw signal start cw signal */
			Cy_TCPWM_TriggerStopOrKill(TCPWM1, motor_a_ccw_MASK);
			Cy_TCPWM_TriggerReloadOrIndex(TCPWM1, motor_a_cw_MASK);
			pwm_cw = true;
			printf( "turning clockwise \r\n"); /*debugging only*/
		}
    	capture = 0;
    	if (!has_shot && robot_bool) {
    		//shoot the ball
    		pot0_val = 0;
			pot1_val = 0;
			result = cyhal_pwm_set_duty_cycle(&servo_0,pot0_val, PWM_FREQUENCY);
			result = cyhal_pwm_set_duty_cycle(&servo_1, pot1_val, PWM_FREQUENCY);
			result = cyhal_pwm_start(&servo_0);
			result = cyhal_pwm_start(&servo_1);
			shoot();
    		cyhal_timer_start(&reset_just_fired_timer);
    	}

    } else if (encoder_A_count >= 31790 && encoder_A_count <= 31850 &&capture >=50){
    	if (pwm_cw) {
			/*stop the cw pwm signal and start the ccw pwm signal*/
			 Cy_TCPWM_TriggerStopOrKill(TCPWM1, motor_a_cw_MASK);
			 Cy_TCPWM_TriggerReloadOrIndex(TCPWM1, motor_a_ccw_MASK);
			pwm_cw = false;
			printf( "turning counter clockwise \r\n"); /*debugging only*/

		} else {
			/*stop ccw signal start cw signal */
			Cy_TCPWM_TriggerStopOrKill(TCPWM1, motor_a_ccw_MASK);
			Cy_TCPWM_TriggerReloadOrIndex(TCPWM1, motor_a_cw_MASK);
			pwm_cw = true;
			printf( "turning clockwise \r\n"); /*debugging only*/
		}
    	capture = 0;
    	if (!has_shot && robot_bool) {
			//shoot the ball
    		pot0_val = 0;
			pot1_val = 0;
			result = cyhal_pwm_set_duty_cycle(&servo_0,pot0_val , PWM_FREQUENCY);
			result = cyhal_pwm_set_duty_cycle(&servo_1,pot1_val, PWM_FREQUENCY);
			result = cyhal_pwm_start(&servo_0);
			result = cyhal_pwm_start(&servo_1);
			shoot();
    		cyhal_timer_start(&reset_just_fired_timer);
		}
    // Pot1_val = 3.525333
    // Pot0_val = 5.559091 R
    // Pot0_val = 6.134545 R

    }


    /*  Set the interrupt flag and process it from the main while(1) loop */
    retrieve_count_flag = true;

}


/*******************************************************************************
* Function Name: start_game
********************************************************************************
* Summary:
* This function turns on the DC motor for the tracking system and starts the timer
* interrupts.
*
* Parameters:
*  none
*
*******************************************************************************/
void start_game(void) {
	/* Turn dc motor on */
	 Cy_TCPWM_TriggerReloadOrIndex(TCPWM1, motor_a_cw_MASK);

	/* Start the timer with the configured settings */
	cyhal_timer_start(&retrieve_count_timer);
}
/*******************************************************************************
* Function Name: stop_game
********************************************************************************
* Summary:
* This function turns off the DC motor for the tracking system and stops the timer
* interrupts.
*
* Parameters:
*  none
*
*******************************************************************************/
void stop_game(void) {
	/* Turn dc motor off */
	printf("stop the pwm!\r\n");

	 Cy_TCPWM_TriggerStopOrKill(TCPWM1, motor_a_ccw_MASK);
	 Cy_TCPWM_TriggerStopOrKill(TCPWM1, motor_a_cw_MASK);

	/* Stop timers for interrupts */
	cyhal_timer_stop(&retrieve_count_timer);
}

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void) {
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: isr_timer
********************************************************************************
* Summary:
* This is the interrupt handler function for the timer interrupt.
*
* Parameters:
*    callback_arg    Arguments passed to the interrupt callback
*    event            Timer/counter interrupt triggers
*
*******************************************************************************/
static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

   /* Set the interrupt flag and process it from the main while(1) loop */
   timer_interrupt_flag = true;

   printf("Timer interrupted\r\n");

   result = cyhal_pwm_stop(&servo_trigger);
}

static float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void shoot(void)
{
	printf("A45\r\n");
	printf("Pot0 value at button interrupt: %4f\r\n", pot0_val);
	printf("Pot1 value at button interrupt: %4f\r\n", pot1_val);

	result = cyhal_pwm_disable_output(&servo_0, CYHAL_PWM_OUTPUT_LINE_OUT);
	result = cyhal_pwm_disable_output(&servo_1, CYHAL_PWM_OUTPUT_LINE_OUT);

	result = cyhal_pwm_start(&servo_trigger);
	result = cyhal_timer_start(&timer);
	has_shot = true;

}

float adc_averager(float pot_array[10])
{
	float sum = 0;
	int size = 10;
	for(int i = 0; i < 10; ++i)
	{
		sum = sum + pot_array[i];
	}
	//printf("Pot val avg: %4f\n", (float) sum/size);
	return (float) sum/size;
}
/*******************************************************************************
* Function Name: reset_just_fired_timer_init
********************************************************************************/

void reset_just_fired_timer_init(void) {
	cy_rslt_t result;

	const cyhal_timer_cfg_t reset_just_fired_timer_cfg = {
		.compare_value = 0,                  //Timer compare value, not used
		.period = reset_just_fired_timer_PERIOD,   // Defines the timer period
		.direction = CYHAL_TIMER_DIR_UP,    // Timer counts up
		.is_compare = false,               //  Don't use compare mode
		.is_continuous = true,             //  Run timer indefinitely
		.value = 0                         //  Initial value of counter
	};

	/* Initialize the timer object. Does not use input pin ('pin' is NC) and
	* does not use a pre-configured clock source ('clk' is NULL).*/
	result = cyhal_timer_init(&reset_just_fired_timer, NC, NULL);

	/* timer init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS) {
	        CY_ASSERT(0);
	}

	/* Configure timer period and operation mode such as count direction,
	duration */
	cyhal_timer_configure(&reset_just_fired_timer, &reset_just_fired_timer_cfg);

	/* Set the frequency of timer's clock source */
	cyhal_timer_set_frequency(&reset_just_fired_timer, reset_just_fired_timer_CLOCK_HZ);

	/* Assign the ISR to execute on timer interrupt */
	cyhal_timer_register_callback(&reset_just_fired_timer, isr_reset_just_fired_timer, NULL);

	 /* Set the event on which timer interrupt occurs and enable it */
	cyhal_timer_enable_event(&reset_just_fired_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
	                              6, true);

}
/*******************************************************************************
* Function Name: isr_reset_just_fired_timer()
********************************************************************************
* Summary:
* This is the interrupt handler function for the retrieve count timer interrupt.
*
* Parameters:
*    callback_arg    Arguments passed to the interrupt callback
*    event            Timer/counter interrupt triggers
*
*******************************************************************************/
static void isr_reset_just_fired_timer(void *callback_arg, cyhal_timer_event_t event) {
    (void) callback_arg;
    (void) event;

    if (has_shot) {
    	has_shot = false;
    	printf("R\n");
    }

    /*  Set the interrupt flag and process it from the main while(1) loop */
    reset_just_fired_flag = true;

}
/* [] END OF FILE */
