#ifndef SIMPLE_KALMAN_FILTER_H
#define SIMPLE_KALMAN_FILTER_H

class SimpleKalmanFilter {
private:
    float Q; // Noise process
    float R; // Noise measurement
    float X; // Giá trị ước lượng hiện tại
    float P; // Sai số ước lượng
    float K; // Kalman Gain

public:
    SimpleKalmanFilter(float process_noise, float measurement_noise, float estimated_error) {
        Q = process_noise;
        R = measurement_noise;
        P = estimated_error;
        X = 0;
    }

    float update(float measurement) {
        // Dự đoán
        P = P + Q;

        // Tính Kalman Gain
        K = P / (P + R);

        // Cập nhật giá trị ước lượng
        X = X + K * (measurement - X);

        // Cập nhật sai số ước lượng
        P = (1 - K) * P;

        return X;
    }

    void setEstimate(float value) {
        X = value;
    }
};

#endif
