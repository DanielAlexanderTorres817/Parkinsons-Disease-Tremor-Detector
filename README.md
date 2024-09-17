# Parkinson's Disease Tremor Detection System Using DSP and Gyroscope Data

## Project Overview

This project implements a tremor detection system using a digital signal processing (DSP) approach. It leverages data from a gyroscope sensor to detect tremors in the frequency range of 3-6 Hz, which is the range for classical Parkinsonian tremors. The system processes raw sensor data, performs a Fast Fourier Transform (FFT) for frequency analysis, and displays tremor detection results on an LCD screen. The project is built using the **STM32 F429ZI** microcontroller with the **mbed** framework and the **ARM CMSIS DSP** library for FFT computations.

## Features

- **Real-time Tremor Detection**: Detects tremor frequencies in the range of 3-6 Hz using gyroscope data.
- **Frequency Analysis**: Uses FFT to convert gyroscope time-domain data into the frequency domain for tremor detection.
- **LCD Display**: Visual feedback on the screen shows if a tremor is detected, including the tremor's strength.
- **Low-pass Filtering**: Applies low-pass filtering to raw gyroscope data to reduce noise.
- **Event-driven**: Utilizes interrupts and flags for efficient handling of SPI communication and data processing.

## Hardware Requirements

- **STM32 F429ZI Discovery Board** (with a built-in accelerometer/gyroscope sensor)
- **LCD Display** (LCD_DISCO_F429ZI)
- **SPI Communication** (configured for interaction with the gyroscope)

## Software Requirements

- **mbed OS**: An open-source embedded operating system designed for IoT applications.
- **ARM CMSIS DSP Library**: Provides the Fast Fourier Transform (FFT) functions used for frequency analysis.
- **LCD_DISCO_F429ZI Driver**: For controlling the LCD screen.

## Project Structure

- **main.cpp**: The main program file that handles SPI communication with the gyroscope, performs FFT on sensor data, and displays results on the LCD.
- **arm_math.h**: DSP library used for FFT operations.
- **mbed.h**: Provides the core mbed API for hardware abstraction.
- **drivers/LCD_DISCO_F429ZI.h**: LCD driver for displaying tremor detection results.

## Key Functions

- **init_fft()**: Initializes the FFT instance used to process the gyroscope data.
- **init_lcd()**: Configures the LCD screen for displaying results.
- **spi_cb()**: Callback function that is triggered upon completion of SPI communication.
- **data_cb()**: Callback function triggered when new gyroscope data is ready.
- **analyze_tremor()**: Performs tremor detection by analyzing FFT output across all three axes (x, y, z).
- **update_lcd_display()**: Updates the LCD screen based on the results of tremor detection.

## How It Works

1. **Data Acquisition**: The system reads gyroscope data via SPI communication and stores it in buffers for the x, y, and z axes.
2. **Low-pass Filtering**: The raw gyroscope data is filtered to smooth out noise using a simple low-pass filter.
3. **FFT Processing**: The filtered data is then processed using FFT to convert it from the time domain to the frequency domain.
4. **Tremor Detection**: The tremor detection algorithm checks if the magnitude of the frequencies in the 3-6 Hz range exceeds a predefined threshold. If so, a tremor is detected.
5. **LCD Display**: The result of the tremor detection (either "Tremor Detected" or "No Tremor") and the strength of the tremor (in Hz) are displayed on the LCD screen.

## How to Build and Run

1. **Clone the repository** and open it in your mbed development environment.
2. **Connect your STM32 F429ZI** board to your computer.
3. **Compile the project** using mbed tools and flash the binary to the STM32 board.
4. **View tremor detection results** on the connected LCD display.

## Customization

- **Thresholds and Frequency Range**: You can adjust the tremor detection thresholds and frequency range by modifying the `TREMOR_FREQ_LOW`, `TREMOR_FREQ_HIGH`, `THRESHOLD_LOW`, and `THRESHOLD_HIGH` values in the code.
- **Filter Coefficient**: Modify the `FILTER_COEFFICIENT` to control the strength of the low-pass filter applied to the gyroscope data.

## Video Demonstration


https://github.com/user-attachments/assets/29c03684-1e09-4695-9fd7-00588c201299

