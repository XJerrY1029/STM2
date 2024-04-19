#include <mbed.h>
#include <vector>



// 定义陀螺仪的寄存器和配置常量
enum GyroRegisters {
    CTRL_REG1 = 0x20,
    CTRL_REG4 = 0x23,
    OUT_X_L = 0x28
};

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
    uint8_t write_buf[2], read_buf[2];
    cs = 0;
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    if (!spi.write(reinterpret_cast<char*>(write_buf), 2, reinterpret_cast<char*>(read_buf), 2)) {
        // 处理SPI通信失败
    }
    cs = 1;

    cs = 0;
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    if (!spi.write(reinterpret_cast<char*>(write_buf), 2, reinterpret_cast<char*>(read_buf), 2)) {
        // 处理SPI通信失败
    }
    cs = 1;
}

// 主程序
int main() {
    SPI spi(PF_9, PF_8, PF_7); // 初始化SPI接口
    spi.format(8, 3);          // 8位数据，模式3
    spi.frequency(1000000);    // 设置SPI总线速度为1MHz

    DigitalOut cs(PC_1, 1);    // 陀螺仪的片选信号
    DigitalOut led1(LED1, 0);
    DigitalOut led2(LED2, 0);

    KalmanFilter kf_gx(0.1, 25.0); // X轴的卡尔曼滤波器
    configureGyroscope(spi, cs);   // 配置陀螺仪

    std::vector<float> gyroReadings; // 存储陀螺仪读数
    gyroReadings.reserve(800);       // 预分配空间以避免重新分配

    while (true) {
        // 读取陀螺仪数据
        cs = 0;
        uint8_t write_buf[1] = {OUT_X_L | 0x80 | 0x40}, read_buf[7];
        if (!spi.write(reinterpret_cast<char*>(write_buf), 1, reinterpret_cast<char*>(read_buf), 7)) {
            // 处理SPI通信失败
        }
        cs = 1;

        int16_t raw_gx = static_cast<int16_t>((read_buf[2] << 8) | read_buf[1]);
        float gx = raw_gx * SCALING_FACTOR;
        kf_gx.update(gx);
        gyroReadings.push_back(kf_gx.getValue());

        // 分析数据以检测频率
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
