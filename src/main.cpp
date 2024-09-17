// Introduction to DSP and Filters
#include "mbed.h"  
#include "arm_math.h"  // Includes the FFT library for frequency analysis
// #include "stm32f429i_discovery_lcd.h"  // Includes the LCD display library for setting up the screen
#include <drivers/LCD_DISCO_F429ZI.h>  // Includes the LCD driver for the STM32 F429ZI board

// Define Registers & Configurations for the gyroscope
#define CTRL_REG1 0x20  // Control register 1 (for configuring the gyroscope)
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1  // Configuration bits for CTRL_REG1

#define CTRL_REG4 0x23  // Control register 4 (for configuring DPS - degrees per second)
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0  // Configuration bits for CTRL_REG4

#define CTRL_REG3 0x22  // Control register 3
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000  // Configuration bits for CTRL_REG3

#define OUT_X_L 0x28  // Register for the output of the x-axis low byte

// Define flags for event handling
#define SPI_FLAG 1  // Flag indicating that SPI communication is complete
#define DATA_READY_FLAG 2  // Flag indicating that new data is ready
#define FILTER_COEFFICIENT 0.1f  // Coefficient for low-pass filtering (adjustable)

#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)  // Scaling factor for converting gyroscope data to radians per second

// Define the frequency range and thresholds for tremor detection (in Hz) and debounce
#define TREMOR_FREQ_LOW 3  // Low-frequency threshold for tremor detection
#define TREMOR_FREQ_HIGH 6  // High-frequency threshold for tremor detection
#define THRESHOLD_LOW 0.2f  // Lower threshold to detect a tremor
#define THRESHOLD_HIGH 10.0f;  // Upper threshold to filter out noise

// Create an EventFlags object to manage event-based signaling between the main loop and interrupt callbacks
EventFlags flags;

#define WINDOW_SIZE 256  // FFT window size (number of samples for processing)
float32_t fft_input_x[WINDOW_SIZE * 2];  // FFT input array for the x-axis (complex data)
float32_t fft_input_y[WINDOW_SIZE * 2];  // FFT input array for the y-axis (complex data)
float32_t fft_input_z[WINDOW_SIZE * 2];  // FFT input array for the z-axis (complex data)
float32_t fft_output_x[WINDOW_SIZE];  // FFT output array for the x-axis (real data)
float32_t fft_output_y[WINDOW_SIZE];  // FFT output array for the y-axis (real data)
float32_t fft_output_z[WINDOW_SIZE];  // FFT output array for the z-axis (real data)
arm_rfft_fast_instance_f32 s;  // FFT instance for handling FFT computations
LCD_DISCO_F429ZI lcd;  // LCD object for managing the display

// SPI callback function triggered when the SPI transaction completes
void spi_cb(int event) {
    flags.set(SPI_FLAG);  // Set the flag indicating SPI communication is complete
}

// Data ready callback function triggered when new data is available from the sensor
void data_cb() {
    flags.set(DATA_READY_FLAG);  // Set the flag indicating new data is ready
}

// Initialize the FFT instance
void init_fft() {
    arm_rfft_fast_init_f32(&s, WINDOW_SIZE);  // Initialize the FFT for the defined window size
}

// Initialize the LCD display settings
void init_lcd() {
   lcd.SetBackColor(LCD_COLOR_WHITE);  // Set the background color of the display to white
   lcd.SetTextColor(LCD_COLOR_BLACK);  // Set the text color to black
   lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"Detection Active", CENTER_MODE);  // Display the message "Detection Active" on the screen
}

/* 
// Uncomment this block if you want to analyze tremor for only the x-axis
void analyze_tremor(float32_t* fft_output, float& tremor_strength, bool& tremor_detected) {
    tremor_strength = 0.0f;  // Initialize tremor strength to zero
    tremor_detected = false;  // Set tremor detection status to false
    for (int i = TREMOR_FREQ_LOW; i <= TREMOR_FREQ_HIGH; i++) {
        float strength = sqrtf(fft_output[i] * fft_output[i]);  // Compute the magnitude for the given frequency
        if (strength > THRESHOLD) {  // Check if the strength exceeds the threshold
            tremor_detected = true;  // Mark tremor as detected
            tremor_strength = max(tremor_strength, strength);  // Update the tremor strength
        }
    }
}
*/

// Analyze tremor by averaging the strength from all three axes (x, y, z)
void analyze_tremor(float32_t* fft_output_x, float32_t* fft_output_y, float32_t* fft_output_z, float& tremor_strength, bool& tremor_detected) {
    tremor_strength = 0.0f;  // Initialize tremor strength to zero
    tremor_detected = false;  // Set tremor detection status to false
    for (int i = TREMOR_FREQ_LOW; i <= TREMOR_FREQ_HIGH; i++) {
        float strength_x = sqrtf(fft_output_x[i] * fft_output_x[i]);  // Compute the magnitude for the x-axis
        float strength_y = sqrtf(fft_output_y[i] * fft_output_y[i]);  // Compute the magnitude for the y-axis
        float strength_z = sqrtf(fft_output_z[i] * fft_output_z[i]);  // Compute the magnitude for the z-axis
        float combined_strength = (strength_x + strength_y + strength_z) / 3;  // Average the strength from all three axes
        if (combined_strength > THRESHOLD_LOW) {  // Check if the combined strength exceeds the threshold
            tremor_detected = true;  // Mark tremor as detected
            tremor_strength = max(tremor_strength, combined_strength);  // Update the maximum tremor strength
        }
    }
}

// Update the LCD display based on tremor detection status
void update_lcd_display(bool tremor_detected, float tremor_strength) {
    if (tremor_detected) {
        lcd.Clear(LCD_COLOR_WHITE);  // Clear the LCD screen
        lcd.SetBackColor(LCD_COLOR_RED);  // Set the background color to red if a tremor is detected
        lcd.SetTextColor(LCD_COLOR_WHITE);  // Set the text color to white
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Tremor Detected!", CENTER_MODE);  // Display the message "Tremor Detected!"
        char buf[32];
        snprintf(buf, 32, "Strength: %.2f Hz", tremor_strength);  // Format the tremor strength value for display
        lcd.DisplayStringAt(0, LINE(6), (uint8_t *)buf, CENTER_MODE);  // Display the tremor strength on the screen
    } else {
        lcd.Clear(LCD_COLOR_WHITE);  // Clear the LCD screen
        lcd.SetBackColor(LCD_COLOR_GREEN);  // Set the background color to green if no tremor is detected
        lcd.SetTextColor(LCD_COLOR_BLACK);  // Set the text color to black
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"No Tremor", CENTER_MODE);  // Display the message "No Tremor"
    }
}

// Main function
int main() {
    init_lcd();  // Initialize the LCD display
    init_fft();  // Initialize the FFT

    // SPI initialization for communication with the gyroscope
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);  // SPI pins configuration
    uint8_t write_buf[32], read_buf[32];  // Buffers for SPI communication

    // Interrupt initialization for handling data readiness
    InterruptIn int2(PA_2, PullDown);  // Define the interrupt pin for data ready signal
    int2.rise(&data_cb);  // Set the callback function for rising edge (data ready)

    // SPI format and frequency settings
    spi.format(8, 3);  // Set SPI data format (8 bits, mode 3)
    spi.frequency(1'000'000);  // Set SPI clock frequency to 1 MHz

    // Configure gyroscope registers via SPI
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);  // Transfer data to configure CTRL_REG1
    flags.wait_all(SPI_FLAG);  // Wait for SPI transfer to complete

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);  // Transfer data to configure CTRL_REG4
    flags.wait_all(SPI_FLAG);  // Wait for SPI transfer to complete

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);  // Transfer data to configure CTRL_REG3
    flags.wait_all(SPI_FLAG);  // Wait for SPI transfer to complete

    write_buf[1] = 0xFF;  // Reset buffer

    // Check if the data ready flag is set
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {
        flags.set(DATA_READY_FLAG);  // Set data ready flag if interrupt indicates new data is available
    }

    // Initialize variables for filtering and processing data
    float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f;  // Filtered gyroscope data
    int cycle = 0;  // Cycle counter for FFT window
    int cnt = 1;  // Count for processing loops

    while (1) {
        int16_t raw_gx, raw_gy, raw_gz;  // Raw gyroscope data
        float gx, gy, gz;  // Scaled gyroscope data

        // Wait for the data ready flag
        flags.wait_all(DATA_READY_FLAG);

        // Read gyroscope data from the output registers
        write_buf[0] = OUT_X_L | 0x80 | 0x40;
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);  // Transfer data to read the x, y, z outputs
        flags.wait_all(SPI_FLAG);  // Wait for SPI transfer to complete

        // Process raw data
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);  // Combine high and low bytes for x-axis
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);  // Combine high and low bytes for y-axis
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);  // Combine high and low bytes for z-axis

        // Convert raw data to actual values in radians per second
        gx = ((float)raw_gx) * SCALING_FACTOR;
        gy = ((float)raw_gy) * SCALING_FACTOR;
        gz = ((float)raw_gz) * SCALING_FACTOR;

        // Apply low-pass filter to smooth the data
        filtered_gx = FILTER_COEFFICIENT * gx + (1 - FILTER_COEFFICIENT) * filtered_gx;
        filtered_gy = FILTER_COEFFICIENT * gy + (1 - FILTER_COEFFICIENT) * filtered_gy;
        filtered_gz = FILTER_COEFFICIENT * gz + (1 - FILTER_COEFFICIENT) * filtered_gz;

        // Store filtered data for FFT input
        fft_input_x[2 * cycle] = (float)filtered_gx;  // Store filtered x-axis data
        fft_input_y[2 * cycle] = (float)filtered_gy;  // Store filtered y-axis data
        fft_input_z[2 * cycle] = (float)filtered_gz;  // Store filtered z-axis data

        // Once a full cycle of data is collected, perform FFT and analyze tremors
        if (cycle == 255) {
            printf("%d-Finish Reading\n", cnt);
            arm_rfft_fast_f32(&s, fft_input_x, fft_output_x, 0);  // Perform FFT on x-axis data
            arm_rfft_fast_f32(&s, fft_input_y, fft_output_y, 0);  // Perform FFT on y-axis data
            arm_rfft_fast_f32(&s, fft_input_z, fft_output_z, 0);  // Perform FFT on z-axis data

            // Analyze tremor strength and detection
            float tremor_strength = 0.0f;
            bool tremor_detected = false;
            analyze_tremor(fft_output_x, fft_output_y, fft_output_z, tremor_strength, tremor_detected);  // Analyze tremor from all axes
            if (tremor_detected) printf("%4.5f\n", tremor_strength);  // Print detected tremor strength
            printf("%d-Finish Analyzing\n", cnt);
            cnt++;  // Increment processing count

            // Update the LCD with tremor detection results
            update_lcd_display(tremor_detected, tremor_strength);  

            ThisThread::sleep_for(1s);  // Wait for 1 second before processing the next cycle
            cycle = 0;  // Reset cycle counter
            continue;
        }
        cycle++;  // Increment cycle counter
    }
}
