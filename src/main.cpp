#include <mbed.h>
#include <vector>

// Define registers and settings for the gyroscope
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01101111  // Output data rate = 800Hz, Bandwidth = 100 Hz
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b00010000  // 2000 dps full scale
#define OUT_X_L 0x28
#define SPI_FLAG 1
#define SCALING_FACTOR (17.5f * M_PI / 180.0f / 1000.0f)  // Convert from mdps to radians

// Kalman filter structure for smoothing gyro readings
struct KalmanFilter {
    float q;  // Process noise variance
    float r;  // Measurement noise variance
    float x;  // Estimated value
    float p;  // Estimation error variance
    float k;  // Kalman gain

    KalmanFilter(float process_noise, float measurement_noise)
        : q(process_noise), r(measurement_noise), x(0.0f), p(1.0f), k(0.0f) {}

    void update(float measurement) {
        // Prediction update
        p += q;
        // Measurement update
        k = p / (p + r);
        x += k * (measurement - x);
        p *= (1 - k);
    }

    float getValue() {
        return x;
    }
};

EventFlags flags;

void spi_cb(int event) {
    if (event & SPI_EVENT_COMPLETE) {
        flags.set(SPI_FLAG);
    }
}

// Helper function to detect peaks in gyroscope data
int findPeaks(const std::vector<float>& values, float threshold, int minInterval) {
    int peakCount = 0;
    int lastPeakIndex = -minInterval;

    for (int i = 1; i < values.size() - 1; i++) {
        if (values[i] > values[i-1] && values[i] > values[i+1] && values[i] > threshold) {
            if (i - lastPeakIndex >= minInterval) {
                peakCount++;
                lastPeakIndex = i;
            }
        }
    }
    return peakCount;
}

int main() {
    SPI spi(PF_9, PF_8, PF_7); // MOSI, MISO, SCLK
    spi.format(8, 3);          // 8-bit, mode 3
    spi.frequency(1000000);    // Set SPI bus speed to 1MHz

    DigitalOut cs(PC_1, 1);    // Chip select for the gyroscope
    DigitalOut led1(LED1, 0);
    DigitalOut led2(LED2, 0);

    uint8_t write_buf[32], read_buf[32];
    KalmanFilter kf_gx(0.1, 25.0);  // Kalman filter for gyro x-axis

    // Configure the gyroscope
    cs = 0;
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.write(reinterpret_cast<char*>(write_buf), 2, reinterpret_cast<char*>(read_buf), 2);
    cs = 1;

    cs = 0;
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.write(reinterpret_cast<char*>(write_buf), 2, reinterpret_cast<char*>(read_buf), 2);
    cs = 1;

    std::vector<float> gyroReadings;
    gyroReadings.reserve(800);  // Reserve space to avoid reallocation

    while (true) {
        // Read gyro data
        cs = 0;
        write_buf[0] = OUT_X_L | 0x80 | 0x40; // Read multiple registers
        spi.write(reinterpret_cast<char*>(write_buf), 1, reinterpret_cast<char*>(read_buf), 7);
        cs = 1;

        int16_t raw_gx = static_cast<int16_t>((read_buf[2] << 8) | read_buf[1]);
        float gx = raw_gx * SCALING_FACTOR;
        kf_gx.update(gx);
        gyroReadings.push_back(kf_gx.getValue());

        // Analyze the data for frequency
        if (gyroReadings.size() >= 800) {
            int peaks = findPeaks(gyroReadings, 0.05, 10);
            float frequency = peaks / 2.0f;  // Half cycles to frequency

            printf("Frequency: %.4f Hz\n", frequency);
            led1 = (frequency >= 3.0f && frequency <= 6.0f) ? 1 : 0;
            led2 = (frequency > 6.0f) ? 1 : 0;

            gyroReadings.clear();
        }

        ThisThread::sleep_for(1ms);
    }
}
