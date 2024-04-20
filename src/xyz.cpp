#include <mbed.h>
#include <vector>

#define SPI_FLAG 1
EventFlags flags;

// 定义陀螺仪的寄存器和配置常量
enum GyroRegisters {
    CTRL_REG1 = 0x20,
    CTRL_REG4 = 0x23,
    OUT_X_L = 0x28,
    OUT_X_H = 0x29,
    OUT_Y_L = 0x2A,
    OUT_Y_H = 0x2B,
    OUT_Z_L = 0x2C,
    OUT_Z_H = 0x2D
};

void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

const uint8_t CTRL_REG1_CONFIG = 0b01101111;  // 输出数据率为800Hz，带宽为100Hz
const uint8_t CTRL_REG4_CONFIG = 0b00010000;  // 全量程2000dps
const float SCALING_FACTOR = 17.5f * M_PI / 180.0f / 1000.0f;  // 将mdps转换为弧度

// 定义卡尔曼滤波器结构，用于平滑陀螺仪读数
struct KalmanFilter {
    float q;  // 过程噪声方差
    float r;  // 测量噪声方差
    float x;  // 估计值
    float p;  // 估计误差方差
    float k;  // 卡尔曼增益

    KalmanFilter(float process_noise, float measurement_noise)
        : q(process_noise), r(measurement_noise), x(0.0f), p(1.0f), k(0.0f) {}

    void update(float measurement) {
        p += q;
        k = p / (p + r);
        x += k * (measurement - x);
        p *= (1 - k);
    }

    float getValue() {
        return x;
    }
};

// 辅助函数，用于检测陀螺仪数据中的峰值
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

// 配置陀螺仪的函数
void configureGyroscope(SPI& spi, DigitalOut& cs) {
    cs = 0;
    spi.write(CTRL_REG1);
    spi.write(CTRL_REG1_CONFIG);
    cs = 1;

    cs = 0;
    spi.write(CTRL_REG4);
    spi.write(CTRL_REG4_CONFIG);
    cs = 1;
}

// 读取陀螺仪数据的函数
void readGyroData(SPI& spi, DigitalOut& cs, float& gx, float& gy, float& gz) {
    cs = 0;
    uint8_t cmd = OUT_X_L | 0x80;  // 开始从OUT_X_L读取，启用自动递增
    spi.write(cmd);
    char out[6];
    spi.write(NULL, 0, out, 6);
    cs = 1;

    int16_t rawX = (out[1] << 8) | out[0];
    int16_t rawY = (out[3] << 8) | out[2];
    int16_t rawZ = (out[5] << 8) | out[4];

    gx = rawX * SCALING_FACTOR;
    gy = rawY * SCALING_FACTOR;
    gz = rawZ * SCALING_FACTOR;
}

// 主程序
int main() {
    // Initialize the SPI object with specific pins.
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

    // Buffers for sending and receiving data over SPI.
    uint8_t write_buf[32], read_buf[32];

    // Configure SPI format and frequency.
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Configure CTRL_REG1 register.
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Configure CTRL_REG4 register.
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    DigitalOut cs(PC_1, 1);    // 片选信号
    DigitalOut led1(LED1);
    DigitalOut led2(LED2);

    KalmanFilter kf_x(0.1, 25.0), kf_y(0.1, 25.0), kf_z(0.1, 25.0);
    configureGyroscope(spi, cs);

    std::vector<float> readingsX, readingsY, readingsZ;
    readingsX.reserve(800); readingsY.reserve(800); readingsZ.reserve(800);

    while (true) {
        uint16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;

        // Prepare to read the gyroscope values starting from OUT_X_L
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        // Perform the SPI transfer to read 6 bytes of data (for x, y, and z axes)
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Convert the received data into 16-bit integers for each axis
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t) read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t) read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t) read_buf[5]);

        // Print the raw values for debugging 
   

        // Convert raw data to actual values using a scaling factor
        gx = ((float) raw_gx) * SCALING_FACTOR;
        gy = ((float) raw_gy) * SCALING_FACTOR;
        gz = ((float) raw_gz) * SCALING_FACTOR;

        //printf("Actual -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t\n", gx, gy, gz);

       // readGyroData(spi, cs, gx, gy, gz);

        kf_x.update(gx);
        kf_y.update(gy);
        kf_z.update(gz);

        readingsX.push_back(kf_x.getValue());
        readingsY.push_back(kf_y.getValue());
        readingsZ.push_back(kf_z.getValue());

        if (readingsX.size() >= 800 && readingsY.size() >= 800 && readingsZ.size() >= 800) {
            int peaksX = findPeaks(readingsX, 1, 10);
            int peaksY = findPeaks(readingsY, 1, 10);
            int peaksZ = findPeaks(readingsZ, 1, 10);

            float frequencyX = peaksX / 2.0;
            float frequencyY = peaksY / 2.0;
            float frequencyZ = peaksZ / 2.0;

            printf("Frequency X: %.4f Hz, Y: %.4f Hz, Z: %.4f Hz\n", frequencyX, frequencyY, frequencyZ);
            led1 = (frequencyX >= 3.0f && frequencyX <= 6.0f) ? 1 : 0;
            led2 = (frequencyY > 6.0f) ? 1 : 0;

            readingsX.clear();
            readingsY.clear();
            readingsZ.clear();
        }

        ThisThread::sleep_for(1ms);
    }
}
