#include <mbed.h>
#include <vector>

// Define the registers and configuration constants for the gyroscope
enum GyroRegisters {
    CTRL_REG1 = 0x20,
    CTRL_REG4 = 0x23,
    OUT_X_L = 0x28
};

const uint8_t CTRL_REG1_CONFIG = 0b01101111;  // Output data rate set to 800Hz, bandwidth to 100Hz
const uint8_t CTRL_REG4_CONFIG = 0b00010000;  // Full scale set to 2000 degrees per second
const float SCALING_FACTOR = 17.5f * M_PI / 180.0f / 1000.0f;  // Converts milli-degrees per second to radians

// Define the Kalman filter structure for smoothing gyroscope readings
struct KalmanFilter {
    float processNoise;  // Process noise variance
    float measurementNoise;  // Measurement noise variance
    float estimate;  // Estimated value
    float estimateError;  // Estimate error variance
    float kalmanGain;  // Kalman gain

    KalmanFilter(float q, float r)
        : processNoise(q), measurementNoise(r), estimate(0.0f), estimateError(1.0f), kalmanGain(0.0f) {}

    void update(float measurement) {
        estimateError += processNoise;
        kalmanGain = estimateError / (estimateError + measurementNoise);
        estimate += kalmanGain * (measurement - estimate);
        estimateError *= (1 - kalmanGain);
    }

    float getValue() {
        return estimate;
    }
};

// Helper function to detect peaks in gyroscope data
int findPeaks(const std::vector<float>& values, float threshold, int minInterval) {
    int peakCount = 0;
    int lastPeakIndex = -minInterval;

    for (int i = 1; i < values.size() - 1; i++) {
        if (values[i] > values[i - 1] && values[i] > values[i + 1] && values[i] > threshold) {
            if (i - lastPeakIndex >= minInterval) {
                peakCount++;
                lastPeakIndex = i;
            }
        }
    }
    return peakCount;
}

// Function to configure the gyroscope
void configureGyroscope(SPI& spi, DigitalOut& cs) {
    uint8_t writeBuffer[2], readBuffer[2];
    cs = 0;
    writeBuffer[0] = CTRL_REG1;
    writeBuffer[1] = CTRL_REG1_CONFIG;
    if (!spi.write(reinterpret_cast<char*>(writeBuffer), 2, reinterpret_cast<char*>(readBuffer), 2)) {
        // Handle SPI communication failure
    }
    cs = 1;

    cs = 0;
    writeBuffer[0] = CTRL_REG4;
    writeBuffer[1] = CTRL_REG4_CONFIG;
    if (!spi.write(reinterpret_cast<char*>(writeBuffer), 2, reinterpret_cast<char*>(readBuffer), 2)) {
        // Handle SPI communication failure
    }
    cs = 1;
}

// Main program
int main() {
    SPI spi(PF_9, PF_8, PF_7);  // Initialize SPI interface
    spi.format(8, 3);  // 8-bit data, mode 3
    spi.frequency(1000000);  // Set SPI bus speed to 1 MHz

    DigitalOut cs(PC_1, 1);  // Gyroscope chip select signal
    DigitalOut led1(LED1, 0);
    DigitalOut led2(LED2, 0);

    KalmanFilter kalmanFilterX(0.1, 25.0);  // Kalman filter for X-axis
    configureGyroscope(spi, cs);  // Configure the gyroscope

    std::vector<float> gyroReadings;  // Store gyroscope readings
    gyroReadings.reserve(800);  // Pre-allocate memory to avoid reallocations

    while (true) {
        // Read gyroscope data
        cs = 0;
        uint8_t writeBuffer[1] = {OUT_X_L | 0x80 | 0x40};
        uint8_t readBuffer[7];
        if (!spi.write(reinterpret_cast<char*>(writeBuffer), 1, reinterpret_cast<char*>(readBuffer), 7)) {
            // Handle SPI communication failure
        }
        cs = 1;

        int16_t raw_gx = static_cast<int16_t>((readBuffer[2] << 8) | readBuffer[1]);
        float gx = raw_gx * SCALING_FACTOR;
        kalmanFilterX.update(gx);
        gyroReadings.push_back(kalmanFilterX.getValue());

        // Analyze data to detect peaks
        if (gyroReadings.size() >= 800) {
            int peaks = findPeaks(gyroReadings, 0.05, 10);
            float frequency = peaks / 2.0f;

            printf("Frequency: %.4f Hz\n", frequency);
            led1 = (frequency >= 3.0f && frequency <= 6.0f) ? 1 : 0;
            led2 = (frequency > 6.0f) ? 1 : 0;

            gyroReadings.clear();
        }

        ThisThread::sleep_for(1ms);
    }
}
