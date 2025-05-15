#include <util/delay.h>
#include "lib/ssd1306.h"
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <math.h> // for sqrt function
#include <stdlib.h>  // For abs() function
#include "MPU6050_res_define.h"
#include "i2cmaster.h"
#include <stdbool.h>
#include <avr/wdt.h>



#define F_CPU 16000000UL // Define the CPU frequency
//#define SAMPLE_SIZE 20    // Rolling average sample size for BPM
// Global variables for sensor data
float Acc_x, Acc_y, Acc_z, Temperature, Gyro_x, Gyro_y, Gyro_z;



// Function prototypes
void init(void);
uint16_t read_adc(uint8_t channel);
uint16_t filter_adc(uint8_t channel);
void init_display(void);
void transfer_bpm_adc(uint16_t adc_bpm);
void transfer_temp_adc(uint16_t adc_temp);


void uart_init(void);
void uart_send(uint8_t data);
void uart_send_string(const char* str);
void transfer_step(uint16_t step_count) ;
void test(uint16_t second_counter);
void SSD1306_ClearLine(uint8_t line);
void turn_on_heart_rate_sensor(void);
void turn_of_heart_rate_sensor(void);
void set_threshold(void);
void check_Button_State(uint8_t BUtton_num);
void reset_bpm_array(void);	

void set_threshold(void);

//mpu6050 initialixation
void MPU6050_Init(void);
void get_data_from_mpu6050(void);


// Variables for BPM smoothing and timing
volatile uint32_t compare_match_count = 0; // For Timer0 counter
volatile uint8_t minute_flag = 0;          // Set when 1 minute has passed for Timer1

  
uint8_t beat_count=0;           // Index to store new BPM sample
///bool prev_above_threshold=false;
uint16_t current_index = 0;  //current index

bool timer0_enable_flag=false; //when this variable is true counter begins


//#define SAMPLE_SIZE 10
#define BUFFER_SIZE 200  // Smaller buffer size 2ms
#define TOTAL_SAMPLES 2500
#define DURATION_SECONDS TOTAL_SAMPLES/BUFFER_SIZE


//timer1
volatile uint16_t second_counter = 0;
bool counting_steps=false;
// Variables for step counting
//static int step_count = 0;
//static int16_t last_accel_magnitude = 0;
//static int16_t last_gyro_magnitude = 0;
// MPU6050 address
#define MPU6050_ADDRESS 0x68

// Function prototypes
void update_bpm(uint16_t* data,  int length);
uint16_t calculate_running_mean(uint16_t* data, uint16_t start, uint16_t length);
uint32_t calculate_running_variance(uint16_t* data, uint16_t start, uint16_t length, uint16_t mean);


// Define constants for step detection
#define STEP_THRESHOLD 1.2  // Acceleration threshold for detecting steps (in g)
#define STEP_MIN_INTERVAL 200  // Minimum interval between steps in ms

// Variables for step detection
uint16_t step_count = 0;
uint16_t last_step_time = 0;  // Timestamp of the last detected step


// Function prototypes
void count_steps_calc(float Xa, float Ya, float Za);
void count_steps(void);
uint32_t millis(void);


// Timer1 ISR (fires every second)
ISR(TIMER1_COMPA_vect) {
    // Every second, set a flag for temperature reading
    
    second_counter++;
	
    // When 60 seconds (1 minute) is reached
    if (second_counter >= 60) {
        second_counter = 0;
        minute_flag = 1;  // Flag to trigger temperature and BPM transmission
	//	PORTB ^= (1 << PB2); // for test that the timer is wokin
	
		
    }
}

// Timer0 ISR 
ISR(TIMER0_COMPA_vect) {
    // Update every 1 ms
    compare_match_count++;
	
}


// Main function


//char buffer[150];
float Xa, Ya, Za, t;
float Xg = 0, Yg = 0, Zg = 0;
float acc_x, acc_y, acc_z;
float temperature;
float gyro_x, gyro_y, gyro_z;

 // Variables for sensor data
    //int16_t accel_x, accel_y, accel_z;
    //int16_t gyro_x, gyro_y, gyro_z;
	
	
int main(void) {
   
	//TWI_Reset();
    I2C_Init();
	//wdt_disable(); // Disable Watchdog Timer
    MPU6050_Init();
    
    init();                 // Initialize peripherals
    uart_init();            // Initialize UART
    _delay_ms(100);
    
	//turn on the ssd1306
//	turn_on_ssd();
    SSD1306_Init(SSD1306_ADDR); // Initialize the OLED display
    init_display();         // Prepare the display
    _delay_ms(100);
    
	//confirmation initialzation done
	SSD1306_SetPosition(0, 2); // Line 2 for display
    SSD1306_DrawString("initialization done");
    SSD1306_UpdateScreen(SSD1306_ADDR);
	_delay_ms(1500);	
	
	//turn_on_heart_rate_sensor();
	timer0_enable_flag=true;
	
    sei();  // Enable global interrupts
	//uart_send_string("come to start of the first loop");

    while (1) {
        // Debugging message before MPU6050 read
        //SSD1306_SetPosition(0, 6);
        //SSD1306_DrawString("Reading MPU...");
        //SSD1306_UpdateScreen(SSD1306_ADDR);
		check_Button_State(1); //check the the reset button //  must put external interuupt for handle this on future
		check_Button_State(2);
		_delay_ms(10);
		
		/* get_data_from_mpu6050();
       
		get_data_from_mpu6050();
		char buffer[100];
		sprintf(buffer, "Acc: %.3f %.3f %.3f Temp: %.3f Gyro: %.3f %.3f %.3f\n", 
		        acc_x, acc_y, acc_z, temperature,
		        gyro_x, gyro_y, gyro_z);
		
		// Here you would send 'buffer' via UART or display it on an LCD/OLED.
		uart_send_string(buffer);
		//uart_send_string("come to end of the first loop");
		_delay_ms(50); */
    }

    return 0;
}

void init(void) {
    // Set PB1 as output for LED
    DDRB |= (1 << PB1 |1<< PB2);

    // Initialize ADC
    ADMUX = (1 << REFS0); // Set reference voltage to AVcc
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC with prescaler 128

    // Initialize Timer0 for 10ms interrupts (for BPM smoothing)
    TCCR0A = (1 << WGM01); // Set Timer0 to CTC (Clear Timer on Compare Match) mode
    TCCR0B = (1 << CS01) | (1 << CS00); // Set prescaler to 64
    OCR0A = 249; // Compare match value for 1 ms interrupt (16 MHz / 64 / 1000) - 1
    TIMSK0 = (1 << OCIE0A); // Enable Timer0 compare match interrupt

    // Initialize Timer1 for 1-second interrupts (for temperature measurement)
    TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); // CTC mode, prescaler 1024
    OCR1A = 15624; // 16 MHz / 1024 / 15624 = 1 Hz (1-second)
    TIMSK1 = (1 << OCIE1A); // Enable Timer1 compare match interrupt
	
	DDRB &= ~(1<<PB0); //iniliazed the button 1
	DDRD &= ~(1<<PD7); //initialized the button 2
	
}

 


void check_Button_State(uint8_t BUtton_num){
	if(BUtton_num==1){
		if (PINB & (1 << PINB0)){
			//timer0_enable_flag=false;
			reset_bpm_array();	
			
		}
		
	}
	
	if(BUtton_num==2){
		if (PIND & (1 << PIND7)){
			//calculate the steps
			// printing the step count on oled
			SSD1306_SetPosition(35, 2);
			SSD1306_ClearLine(2);
			SSD1306_DrawString("calculating steps ");
			SSD1306_UpdateScreen(SSD1306_ADDR);
			count_steps();
			
			
		}
	
	
	}
	
}



void count_steps(void){

	// counting the steps
	while(1){
		get_data_from_mpu6050();
		count_steps_calc( Xa,Ya,Za);
		if ((PIND & (1 << PIND7)) || (PINB & (1 << PINB0))) {
			// printing the step count on oled
			
			
			
			break;
		}
		char buffer[20];
			SSD1306_ClearLine(2);
			SSD1306_SetPosition(50, 3);
			sprintf(buffer, " %d steps ",step_count );
			SSD1306_DrawString(buffer);
			SSD1306_UpdateScreen(SSD1306_ADDR);
	}
	
	

}

// Main function for step counting
void count_steps_calc(float Xa, float Ya, float Za) {
    // Calculate magnitude of acceleration vector
    float magnitude = sqrt(Xa * Xa + Ya * Ya + Za * Za);

    // Calculate time elapsed since last step
    uint16_t current_time = millis();
    uint16_t time_diff = current_time - last_step_time;

    // Check if acceleration magnitude is above threshold and time since last step is sufficient
    if (magnitude > STEP_THRESHOLD && time_diff > STEP_MIN_INTERVAL) {
        step_count++;  // Increment step count
        last_step_time = current_time;  // Update the last step time

        // Debugging: Send step count to UART
        char buffer[50];
        sprintf(buffer, "Step detected! Step count: %d\n", step_count);
        uart_send_string(buffer);
    }
}

// Function to get the current time in milliseconds (using Timer0)
uint32_t millis(void) {
    // Assuming Timer0 is configured to generate an interrupt every 1 ms
    return compare_match_count;  // Return the millisecond counter
}



void reset_bpm_array(void) {

	char buffer[20];
    uint16_t adc_buffer[BUFFER_SIZE];  
    uint16_t mean, variance;
    bool above_threshold = false;
    uint16_t beat_count = 0;
    uint16_t i, j;
    uint16_t threshold = 0;  // Initial threshold

    turn_of_heart_rate_sensor();

    SSD1306_ClearLine(2);
    SSD1306_SetPosition(0, 2);
    SSD1306_DrawString("getting data.. ");
    SSD1306_UpdateScreen(SSD1306_ADDR);
    _delay_ms(1000);

    turn_on_heart_rate_sensor();
    _delay_ms(5000);
    //uart_send_string("vCAME HERE");

    // Process data in chunks
    for (i = 0; i < TOTAL_SAMPLES; i += BUFFER_SIZE) {
        // Collect a chunk of ADC data into the buffer
        for (j = 0; j < BUFFER_SIZE; j++) {
            adc_buffer[j] = read_adc(3);
            _delay_ms(2); 
            
            // Debugging: Print ADC values
          // sprintf(buffer, "ADC[%d]: %d\n", j, adc_buffer[j]);
          // uart_send_string(buffer);uint
        }

        // Process the buffe
        mean = calculate_running_mean(adc_buffer, 0, BUFFER_SIZE);
        variance = calculate_running_variance(adc_buffer, 0, BUFFER_SIZE, mean);
        threshold = mean + (variance >> 2); 

        // Debugging: Print calculated values
        //sprintf(buffer, "Mean: %d, Variance: %d, Threshold: %d\n", mean, variance, threshold);
       // uart_send_string(buffer);

        // Detect beats in the buffer
        for (j = 0; j < BUFFER_SIZE; j++) {
            if (!above_threshold && adc_buffer[j] > threshold) {
                above_threshold = true;
                //uart_send_string("Rising edge detected\n");
            }
            if (above_threshold && adc_buffer[j] < threshold) {
                above_threshold = false;
                beat_count++;
                //uart_send_string("Falling edge detected\n");
            }
        }
    }
    
    // Calculate BPM
    //uint16_t bpm = (beat_count * 60) / DURATION_SECONDS;

    // Debugging: Check beat count before calculating BPM
    //sprintf(buffer, "Total beats: %d\n", beat_count);
    //uart_send_string(buffer);

    // Display the BPM on OLED
    SSD1306_ClearLine(2);
    SSD1306_SetPosition(40, 2);
    sprintf(buffer, " BPM =%d ",beat_count*12); // since we calculate the pulse through 5 s
    SSD1306_DrawString(buffer);
    SSD1306_UpdateScreen(SSD1306_ADDR);
	
	
	//send it via bluetooth
	sprintf(buffer, " %d ", beat_count*12);
	uart_send_string(buffer); // send the value in 


	//display the temperature reading also
	
	uint16_t adc_temp = filter_adc(2);
	uint16_t temp_whole = (adc_temp * 488) / 1000;  // Whole part of the temperature
	uint16_t temp_decimal = ((adc_temp * 488) % 1000) / 100;  // Extract the first decimal place

// Display the temperature with a decimal point
	SSD1306_ClearLine(3);
	SSD1306_SetPosition(10, 3);
	sprintf(buffer, "Skin temp = %d.%d C", temp_whole, temp_decimal);  // Display whole and decimal parts
	SSD1306_DrawString(buffer);
	SSD1306_UpdateScreen(SSD1306_ADDR);
	
	uart_send_string(",");
	sprintf(buffer, "%d.%d", temp_whole, temp_decimal); 
	uart_send_string(buffer); // send the value in 
	uart_send_string("\n");
	
	

    // Transmit the value via UART
    //uart_send_string(buffer);

    // Reset beat count for the next cycle
    beat_count = 0;

    turn_of_heart_rate_sensor();
}

uint16_t calculate_running_mean(uint16_t* data, uint16_t start, uint16_t length) {
    uint32_t sum = 0;
    for (uint16_t i = start; i < start + length; i++) {
        sum += data[i];
    }
    return (uint16_t)(sum / length);
}

uint32_t calculate_running_variance(uint16_t* data, uint16_t start, uint16_t length, uint16_t mean) {
    uint32_t sum = 0;  // Use uint32_t for summing the squared differences
    for (uint16_t i = start; i < start + length; i++) {
        int32_t diff = (int32_t)data[i] - (int32_t)mean;  // Use int32_t for difference
        sum += (uint32_t)(diff * diff);  // Calculate squared difference and add to sum
    }
    return (uint32_t)(sum / length);  // Return the average of squared differences
}



void turn_of_heart_rate_sensor(void){
	PORTB &= ~(1 << PB1);

}

void turn_on_heart_rate_sensor(void){

	PORTB |= (1 << PB1); //turn on  the sensor 
}

void turn_on_ssd(void){
	

}

// Filter ADC function to smooth the ADC readings
uint16_t filter_adc(uint8_t channel) {
    uint32_t sum = 0;
    const uint8_t samples = 5; // Number of samples for averaging
    for (uint8_t i = 0; i < samples; i++) {
        sum += read_adc(channel);
        _delay_us(50);
    }
    return (uint16_t)(sum / samples); // Return the average value
}


// Read ADC function
uint16_t read_adc(uint8_t channel) {
    // Select ADC channel with safety mask
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07); // Clear previous channel and set new one
    _delay_us(100); // Allow channel to stabilize
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
    return ADC; // Return ADC value
}



// UART initialization function
void uart_init(void) {
   // uint16_t baud_prescaler = 51; // For 19200 baud rate and F_CPU = 16 MHz
	// Equation: baud_prescaler = (F_CPU / (16 * baud rate)) - 1
	//           = (16000000 / (16 * 19200)) - 1 = 51
	
	
	uint16_t baud_prescaler = 103; // For 9600 baud rate and F_CPU = 16 MHz
// Equation: baud_prescaler = (F_CPU / (16 * baud rate)) - 1
//           = (16000000 / (16 * 9600)) - 1 = 103

	UBRR0H = (baud_prescaler >> 8);
	UBRR0L = baud_prescaler;

    // Enable transmitter and receiver
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);

    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// UART send function for single character
void uart_send(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait for empty transmit buffer
    UDR0 = data; // Send data
}

// UART send function for string
void uart_send_string(const char* str) {
    while (*str) {
        uart_send(*str++); // Send each character in string
    }
}

// Transfer BPM data via UART
void transfer_bpm_adc(uint16_t adc_bpm) {
	SSD1306_ClearLine(2);
    char buffer[20];
    sprintf(buffer, " %d\n", adc_bpm);
    uart_send_string(buffer);
}

// Transfer temperature data via UART
void transfer_temp_adc(uint16_t adc_temp) {
    char buffer[20];
    sprintf(buffer, " %d \n", adc_temp*488/1000);
    uart_send_string(buffer);
}

void transfer_step(uint16_t step_count) {
    char buffer[20];
    sprintf(buffer, "Temp: %d\n", step_count);
    uart_send_string(buffer);
}

// OLED display initialization
void init_display(void) {
    SSD1306_ClearScreen(); // Clear the OLED screen
    SSD1306_SetPosition(40, 0); // Set position for text
    SSD1306_DrawString("Medi-care"); // Draw string on OLED
    SSD1306_SetPosition(45, 1); // Set position for text
	
	//will update on the current heart beat 
    SSD1306_DrawString("Normal"); // Draw string on OLED
	SSD1306_SetPosition(30, 7); // Set position for text
    SSD1306_DrawString("version 1.0"); // Draw string on OLED
	
	
	//SSD1306_SetPosition(50, 5); // Line 4 for step count
	//SSD1306_DrawString("hi yaluwane");
	SSD1306_UpdateScreen(SSD1306_ADDR);
	_delay_ms(1); 
	
}




//mpu6050 
void MPU6050_Init(void) {
    _delay_ms(150);
    I2C_Start_Wait(0xD0);
    I2C_Write(SMPLRT_DIV);
    I2C_Write(0x07);
    I2C_Stop();

    I2C_Start_Wait(0xD0);
    I2C_Write(PWR_MGMT_1);
    I2C_Write(0x01);
    I2C_Stop();

    I2C_Start_Wait(0xD0);
    I2C_Write(CONFIG);
    I2C_Write(0x00);
    I2C_Stop();

    I2C_Start_Wait(0xD0);
    I2C_Write(GYRO_CONFIG);
    I2C_Write(0x18);
    I2C_Stop();

    I2C_Start_Wait(0xD0);
    I2C_Write(INT_ENABLE);
    I2C_Write(0x01);
    I2C_Stop();
}

void MPU_Start_Loc(void) {
    I2C_Start_Wait(0xD0);     // Start communication with MPU6050
    I2C_Write(ACCEL_XOUT_H);  // Write the starting register address for accelerometer data
    I2C_Repeated_Start(0xD1); // Start repeated start to read data
}

void Read_RawValue(void) {
    MPU_Start_Loc();

    // Read accelerometer, temperature, and gyroscope values
    Acc_x = (((int16_t)I2C_Read_Ack() << 8) | (int16_t)I2C_Read_Ack());
    Acc_y = (((int16_t)I2C_Read_Ack() << 8) | (int16_t)I2C_Read_Ack());
    Acc_z = (((int16_t)I2C_Read_Ack() << 8) | (int16_t)I2C_Read_Ack());
    Temperature = (((int16_t)I2C_Read_Ack() << 8) | (int16_t)I2C_Read_Ack());
    Gyro_x = (((int16_t)I2C_Read_Ack() << 8) | (int16_t)I2C_Read_Ack());
    Gyro_y = (((int16_t)I2C_Read_Ack() << 8) | (int16_t)I2C_Read_Ack());
    Gyro_z = (((int16_t)I2C_Read_Ack() << 8) | (int16_t)I2C_Read_Nack());

    I2C_Stop();  // Stop I2C communication

    // Debugging output: Print raw values to UART to check if they are correct
    //sprintf(buffer, "Raw Acc: %d %d %d, Temp: %d, Gyro: %d %d %d\n", 
    //        Acc_x, Acc_y, Acc_z, Temperature, Gyro_x, Gyro_y, Gyro_z);
    //uart_send_string(buffer);
    //_delay_ms(100);  // Delay for readability
}

void get_data_from_mpu6050(void) {
    Read_RawValue();
	char buffer[150];

    // Convert raw data to real-world values
    Xa = Acc_x / 16384.0;
    Ya = Acc_y / 16384.0;
    Za = Acc_z / 16384.0;

    Xg = Gyro_x / 16.4;
    Yg = Gyro_y / 16.4;
    Zg = Gyro_z / 16.4;

    t = (Temperature / 340.00) + 36.53;

    // Convert float values to scaled integers for UART output
    int16_t Xa_int = (int16_t)(Xa * 1000);  // Scale to 3 decimal places
    int16_t Ya_int = (int16_t)(Ya * 1000);
    int16_t Za_int = (int16_t)(Za * 1000);
    int16_t temp_int = (int16_t)(t * 100);  // Scale to 2 decimal places

    // Send the converted data as integers to UART
    sprintf(buffer, 
            "Acc: %d.%03d %d.%03d %d.%03d Temp: %d.%02d Gyro: %d %d %d\n", 
            Xa_int / 1000, abs(Xa_int % 1000), 
            Ya_int / 1000, abs(Ya_int % 1000), 
            Za_int / 1000, abs(Za_int % 1000),
            temp_int / 100, abs(temp_int % 100),
            (int)Xg, (int)Yg, (int)Zg);

    uart_send_string(buffer);
    //_delay_ms(300);  // Add delay for readability
}
void SSD1306_ClearLine(uint8_t line) {
    // Each character is 6 pixels wide, and the screen width is 128 pixels
    // 128 pixels / 6 pixels per character = 21 characters max in a line (rounded down)
    SSD1306_SetPosition(0, line); // Set cursor to the beginning of the specified line
    for (uint8_t i = 0; i < 21; i++) {
        SSD1306_DrawChar(' '); // Draw spaces to clear the line
    }
    SSD1306_UpdateScreen(SSD1306_ADDR); // Update the screen to show changes
}