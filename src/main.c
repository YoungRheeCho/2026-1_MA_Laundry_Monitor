#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(vibration, LOG_LEVEL_INF);

/* 튜닝 파라미터 */
#define SAMPLE_COUNT   20       /* 샘플 몇 개 모아서 판단 */
#define SAMPLE_MS      50       /* 샘플링 간격 (50ms = 초당 20회) */
#define THRESHOLD      0.3f     /* RMS 임계값 — 실제 값 보고 조정 */

static const struct device *dev;

static float read_magnitude(void)
{
    struct sensor_value x, y, z;

    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &x);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &y);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &z);

    float fx = sensor_value_to_double(&x);
    float fy = sensor_value_to_double(&y);
    float fz = sensor_value_to_double(&z);

    return sqrtf(fx*fx + fy*fy + fz*fz);
}

static float calc_rms(float *samples, int count)
{
    float mean = 0.0f;
    float sum  = 0.0f;

    /* 평균 제거 후 RMS → 중력 성분 제거, 진동 성분만 추출 */
    for (int i = 0; i < count; i++) {
        mean += samples[i];
    }
    mean /= count;

    for (int i = 0; i < count; i++) {
        float diff = samples[i] - mean;
        sum += diff * diff;
    }

    return sqrtf(sum / count);
}

int main(void)
{
    dev = DEVICE_DT_GET_ANY(adi_adxl345);

    if (!device_is_ready(dev)) {
        LOG_ERR("ADXL345 준비 안됨! 연결 확인하세요.");
        return -1;
    }

    LOG_INF("ADXL345 초기화 완료. 진동 감지 시작...");

    float    samples[SAMPLE_COUNT];
    bool     is_running = false;
    uint32_t run_start  = 0;

    while (1) {
        /* 샘플 수집 */
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            samples[i] = read_magnitude();
            k_sleep(K_MSEC(SAMPLE_MS));
        }

        float rms = calc_rms(samples, SAMPLE_COUNT);

        /* RMS 값 항상 출력 — 처음엔 이 값으로 THRESHOLD 튜닝 */
        LOG_INF("RMS: %d.%03d m/s²",
                (int)rms,
                (int)((rms - (int)rms) * 1000));

        bool current = (rms > THRESHOLD);

        /* 상태 변화 시에만 출력 */
        if (current != is_running) {
            is_running = current;

            if (is_running) {
                run_start = k_uptime_get_32();
                LOG_INF(">>> 세탁기 동작 시작!");
            } else {
                uint32_t elapsed = (k_uptime_get_32() - run_start) / 1000;
                LOG_INF(">>> 세탁기 정지. 동작 시간: %d분 %d초",
                        elapsed / 60, elapsed % 60);
            }
        }
    }

    return 0;
}