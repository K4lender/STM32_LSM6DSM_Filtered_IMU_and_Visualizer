/*
 * Title: imu_sim.c
 *
 * Date: 20/02/2026
 *
 * Author: Muhammed Selman Çetin
 *
 * GitHub: K4lender
 */


#include "imu_sim.h"
#include "lsm6dsm_defs.h"
#include "lsm6dsm_driver.h"
#include "filter.h"
#include "ring_buffer.h"
#include "timer.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>


static uint32_t s_time_ms = 0;          // Simulation time (ms)
static float s_dt_ms = 0.0f;            // Sample period (ms) - set by IMU_Sim_Init()
static int s_rand_init = 0;             // Random number seed flag


static void Init_Random(void) {
    if (!s_rand_init) {
        srand((unsigned int)time(NULL));
        s_rand_init = 1;
    }
}


static float Gaussian_Random(void) {
    float u1, u2, r, mul;
    
    do {
        u1 = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
        u2 = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
        r = u1 * u1 + u2 * u2;
    } while (r >= 1.0f || r == 0.0f);
    
    mul = sqrtf(-2.0f * logf(r) / r);
    return u1 * mul;
}


static int16_t Generate_Noise(int std_dev) {
    float noise = Gaussian_Random() * (float)std_dev;
    
    if (noise > 3.0f * std_dev) noise = 3.0f * std_dev;
    if (noise < -3.0f * std_dev) noise = -3.0f * std_dev;
    
    return (int16_t)noise;
}


static int16_t Saturate(int32_t value) {
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}


void IMU_Sim_Init(float sample_rate_hz) {
    Init_Random();
    
    s_time_ms = 0;
    
    if (sample_rate_hz > 0.0f) {
        s_dt_ms = 1000.0f / sample_rate_hz;
    } else {
        s_dt_ms = 9.615f;  // Default 104 Hz
    }
}


void IMU_Sim_GetAccel(int16_t* ax, int16_t* ay, int16_t* az) {
    float t = (float)s_time_ms / 1000.0f;  // seconds
    float omega = 2.0f * (float)M_PI * SIM_DEFAULT_FREQ;
    
    int32_t x_val, y_val, z_val;
    
    // X-axis: Sinusoidal motion + noise
    x_val = (int32_t)(SIM_DEFAULT_AMP * sinf(omega * t));
    x_val += Generate_Noise(SIM_ACCEL_NOISE);
    
    // Y-axis: Sinusoidal with different phase + noise
    y_val = (int32_t)(SIM_DEFAULT_AMP * sinf(omega * t + M_PI / 4.0f));
    y_val += Generate_Noise(SIM_ACCEL_NOISE);
    
    // Z-axis: Gravity + small oscillation + noise
    z_val = SIM_GRAVITY_OFFSET;
    z_val += (int32_t)(SIM_DEFAULT_AMP / 4 * sinf(omega * t * 0.5f));
    z_val += Generate_Noise(SIM_ACCEL_NOISE);
    
    *ax = Saturate(x_val);
    *ay = Saturate(y_val);
    *az = Saturate(z_val);
}


void IMU_Sim_GetGyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    float t = (float)s_time_ms / 1000.0f;
    float omega = 2.0f * (float)M_PI * SIM_DEFAULT_FREQ;
    float omega_emi = 2.0f * (float)M_PI * SIM_EMI_FREQ;
    
    int32_t x_val, y_val, z_val;
    int32_t emi;  // 50 Hz power line interference (added to all axes)
    int16_t half_amp = SIM_DEFAULT_AMP / 2;
    
    // 50 Hz EMI component (power line interference simulation)
    emi = (int32_t)(SIM_EMI_AMP * sinf(omega_emi * t));
    
    // Gyroscope: Slower motion simulation + 50 Hz EMI
    x_val = (int32_t)(half_amp * sinf(omega * t * 0.7f));
    x_val += emi;
    x_val += Generate_Noise(SIM_GYRO_NOISE);
    
    y_val = (int32_t)(half_amp * sinf(omega * t * 0.7f + M_PI / 3.0f));
    y_val += emi;
    y_val += Generate_Noise(SIM_GYRO_NOISE);
    
    z_val = (int32_t)(half_amp / 2 * sinf(omega * t * 0.5f));
    z_val += emi;
    z_val += Generate_Noise(SIM_GYRO_NOISE);
    
    *gx = Saturate(x_val);
    *gy = Saturate(y_val);
    *gz = Saturate(z_val);
}


int16_t IMU_Sim_GetTemperature(void) {
    int16_t temp = Generate_Noise(5);
    return temp;
}


void IMU_Sim_Tick(void) {
    s_time_ms += (uint32_t)s_dt_ms;
}


void IMU_Sim_Reset(void) {
    s_time_ms = 0;
}


uint32_t IMU_Sim_GetTime(void) {
    return s_time_ms;
}


// Forward declaration: Sim_ProcessCallback is defined below alongside filter variables.
static LSM6DSM_Status_t Sim_ProcessCallback(const LSM6DSM_Data_t *scaled,
                                         LSM6DSM_FilteredData_t   *out,
                                         void                 *ctx);


void IMU_Sim_UpdateDriver(void) {
    LSM6DSM_RawData_t raw;

    IMU_Sim_GetAccel(&raw.accel_x, &raw.accel_y, &raw.accel_z);
    IMU_Sim_GetGyro (&raw.gyro_x,  &raw.gyro_y,  &raw.gyro_z);
    raw.temperature  = IMU_Sim_GetTemperature();
    raw.timestamp_ms = IMU_Sim_GetTime();
    IMU_Sim_Tick();
    LSM6DSM_InjectRawData(&raw);
}


void IMU_Sim_RegisterProcessFn(void) {
    LSM6DSM_RegisterProcessFn(Sim_ProcessCallback, NULL);
}


#define PRINT_INTERVAL_MS       200     // Print to screen every 200ms

#define LPF_CUTOFF_FREQ         20.0f
#define NOTCH_CENTER_FREQ       50.0f
#define NOTCH_WIDTH             10.0f
#define CF_ALPHA                0.98f

static RingBuffer_t s_buffer;

// Accelerometer filters
static LowPassFilter_t s_lpf_ax, s_lpf_ay, s_lpf_az;

// Gyroscope filters
static NotchFilter_t s_notch_gx, s_notch_gy, s_notch_gz;
static LowPassFilter_t s_lpf_gx, s_lpf_gy, s_lpf_gz;

// Complementary filter
static ComplementaryFilter_t s_cf_roll, s_cf_pitch;

// Computed Roll/Pitch
static float s_roll_deg = 0.0f;
static float s_pitch_deg = 0.0f;

// Performance statistics
static uint64_t s_total_filter_us = 0;
static uint32_t s_filter_count = 0;
static uint64_t s_max_filter_us = 0;


static void Init_Filters(float sample_rate_hz) {
    float sample_time_s = 1.0f / sample_rate_hz;
    
    LPF_Init(&s_lpf_ax, LPF_CUTOFF_FREQ, sample_time_s);
    LPF_Init(&s_lpf_ay, LPF_CUTOFF_FREQ, sample_time_s);
    LPF_Init(&s_lpf_az, LPF_CUTOFF_FREQ, sample_time_s);

    Notch_Init(&s_notch_gx, NOTCH_CENTER_FREQ, NOTCH_WIDTH, sample_time_s);
    Notch_Init(&s_notch_gy, NOTCH_CENTER_FREQ, NOTCH_WIDTH, sample_time_s);
    Notch_Init(&s_notch_gz, NOTCH_CENTER_FREQ, NOTCH_WIDTH, sample_time_s);
    LPF_Init(&s_lpf_gx, LPF_CUTOFF_FREQ, sample_time_s);
    LPF_Init(&s_lpf_gy, LPF_CUTOFF_FREQ, sample_time_s);
    LPF_Init(&s_lpf_gz, LPF_CUTOFF_FREQ, sample_time_s);

    CF_Init(&s_cf_roll,  CF_ALPHA, sample_time_s);
    CF_Init(&s_cf_pitch, CF_ALPHA, sample_time_s);
}


static void Reset_Stats(void) {
    s_total_filter_us = 0;
    s_filter_count = 0;
    s_max_filter_us = 0;
}


// Simulation filter pipeline - registered via LSM6DSM_RegisterProcessFn.
// Accelerometer: 2nd order Bessel LPF
// Gyroscope:  Notch (50 Hz) -> LPF
static LSM6DSM_Status_t Sim_ProcessCallback(const LSM6DSM_Data_t *scaled,
                                         LSM6DSM_FilteredData_t   *out,
                                         void                 *ctx) {
    float notch_gx, notch_gy, notch_gz;
    (void)ctx;

    // Accelerometer LowPass
    out->accel_x = LPF_Update(&s_lpf_ax, scaled->accel_x);
    out->accel_y = LPF_Update(&s_lpf_ay, scaled->accel_y);
    out->accel_z = LPF_Update(&s_lpf_az, scaled->accel_z);

    // Gyroscope Notch + LowPass
    notch_gx = Notch_Update(&s_notch_gx, scaled->gyro_x);
    notch_gy = Notch_Update(&s_notch_gy, scaled->gyro_y);
    notch_gz = Notch_Update(&s_notch_gz, scaled->gyro_z);
    out->gyro_x = LPF_Update(&s_lpf_gx, notch_gx);
    out->gyro_y = LPF_Update(&s_lpf_gy, notch_gy);
    out->gyro_z = LPF_Update(&s_lpf_gz, notch_gz);

    out->temperature_c = scaled->temp_c;
    out->timestamp_ms  = scaled->timestamp;
    return IMU_OK;
}


static int Get_IMU_Data(LSM6DSM_Data_t* raw_out, LSM6DSM_FilteredData_t* filt_out) {
    LSM6DSM_RawData_t raw_imu;
    LSM6DSM_Data_t scaled;
    LSM6DSM_FilteredData_t filtered;
    float accel_roll, accel_pitch;
    uint64_t t_start, t_end, elapsed;

    // 1. Push sim data into the driver's register table
    IMU_Sim_UpdateDriver();

    // 2. Read raw data and write to buffer (producer)
    if (LSM6DSM_ReadRaw(&raw_imu) != IMU_OK) {
        return -1;
    }
    RB_Write(&s_buffer, raw_imu);

    // 3. Scaled data (raw output for display)
    if (LSM6DSM_ReadScaled(&scaled) != IMU_OK) {
        return -1;
    }
    if (raw_out != NULL) {
        *raw_out = scaled;
    }

    // 4. Filtered data: via registered process callback (consumer)
    t_start = Timer_GetTick_us();
    if (LSM6DSM_ReadProcessed(&filtered) != IMU_OK) {
        return -1;
    }
    t_end = Timer_GetTick_us();

    // 5. Complementary Filter: Angle estimation
    accel_roll  = atan2f(filtered.accel_y, filtered.accel_z) * (180.0f / (float)M_PI);
    accel_pitch = atan2f(-filtered.accel_x, sqrtf(filtered.accel_y * filtered.accel_y + filtered.accel_z * filtered.accel_z)) * (180.0f / (float)M_PI);
    s_roll_deg  = CF_Update(&s_cf_roll,  filtered.gyro_x, accel_roll);
    s_pitch_deg = CF_Update(&s_cf_pitch, filtered.gyro_y, accel_pitch);

    elapsed = t_end - t_start;
    s_total_filter_us += elapsed;
    s_filter_count++;
    if (elapsed > s_max_filter_us) {
        s_max_filter_us = elapsed;
    }

    if (filt_out != NULL) {
        *filt_out = filtered;
    }

    return 0;
}


static void Print_Raw_And_Filtered(uint32_t t_ms, LSM6DSM_Data_t* imu_raw, LSM6DSM_FilteredData_t* imu_filter) {
    printf("t=%5lu ms\n", (unsigned long)t_ms);
    printf("  Ham    Accel: %+7.3f %+7.3f %+7.3f g  |  Gyro: %+8.2f %+8.2f %+8.2f dps\n",
           imu_raw->accel_x, imu_raw->accel_y, imu_raw->accel_z,
           imu_raw->gyro_x, imu_raw->gyro_y, imu_raw->gyro_z);
    printf("  Filt   Accel: %+7.3f %+7.3f %+7.3f g  |  Gyro: %+8.2f %+8.2f %+8.2f dps\n",
           imu_filter->accel_x, imu_filter->accel_y, imu_filter->accel_z,
           imu_filter->gyro_x, imu_filter->gyro_y, imu_filter->gyro_z);
    printf("  Aci    Roll: %+7.2f deg  |  Pitch: %+7.2f deg  (Complementary)\n",
           s_roll_deg, s_pitch_deg);
}


static void Print_Performance(void) {
    float avg_us = 0.0f;

    if (s_filter_count > 0) {
        avg_us = (float)s_total_filter_us / (float)s_filter_count;
    }

    printf("\n  [Performans] Filtreleme suresi:\n");
    printf("    Ortalama: %.1f us | Maksimum: %llu us | Toplam ornek: %lu\n",
           avg_us, (unsigned long long)s_max_filter_us,
           (unsigned long)s_filter_count);
    printf("    Buffer overflow: %lu | Buffer doluluk: %u/%d\n",
           (unsigned long)s_buffer.overflow_count,
           RB_GetCount(&s_buffer), BUFFER_SIZE);
}

//////////////////////////////////////////////////////////////////////////////  SCENARIOS  //////////////////////////////////////////////////////////////////////////

// Scenario 1: Normal operation
void IMU_Sim_Scenario_Normal(uint32_t duration_ms) {
    LSM6DSM_Data_t imu_raw;
    LSM6DSM_FilteredData_t imu_filter;
    uint32_t start_time, last_sample = 0, last_print = 0;
    uint32_t sample_count = 0;
    float sample_rate_hz = LSM6DSM_GetSampleRate();
    uint32_t sample_period_ms = (uint32_t)(1000.0f / sample_rate_hz);
    
    printf("\n");
    printf("--------------------------------------------------------------------------------\n");
    printf("SENARYO 1: Normal Calisma\n");
    printf("Aciklama: Standart kosullarda %.1f Hz ornekleme, LPF + Notch filtreleme\n", sample_rate_hz);
    
    IMU_Sim_Init(sample_rate_hz);
    IMU_Sim_RegisterProcessFn();
    RB_Init(&s_buffer);
    Init_Filters(sample_rate_hz);
    Reset_Stats();
    
    start_time = Timer_GetTick_ms();
    
    while (duration_ms == 0 || (Timer_GetTick_ms() - start_time) < duration_ms) {
        
        if (Timer_CheckPeriod(&last_sample, sample_period_ms)) {
            Get_IMU_Data(&imu_raw, &imu_filter);
            sample_count++;
            
            // Periodically print to screen and drain buffer
            if (Timer_CheckPeriod(&last_print, PRINT_INTERVAL_MS)) {
                LSM6DSM_RawData_t tmp;
                while (!RB_IsEmpty(&s_buffer)) { 
                    RB_Read(&s_buffer, &tmp); 
                }
                Print_Raw_And_Filtered(Timer_GetTick_ms() - start_time, &imu_raw, &imu_filter);
            }
        }
    }
    
    printf("\n  [Sonuc] Toplam ornek: %lu\n", (unsigned long)sample_count); // Cast to avoid hardware-dependent integer width.
    Print_Performance();
}


// Scenario 2: Data Loss (Buffer Overflow) test
void IMU_Sim_Scenario_Data_Loss(uint32_t duration_ms) {
    LSM6DSM_Data_t imu_raw;
    LSM6DSM_FilteredData_t imu_filter;
    uint32_t start_time, last_sample = 0, last_print = 0;
    uint32_t write_count = 0, read_count = 0;
    uint32_t overflow_before;
    float sample_rate_hz = LSM6DSM_GetSampleRate();
    uint32_t sample_period_ms = (uint32_t)(1000.0f / sample_rate_hz);
    
    printf("\n");
    printf("--------------------------------------------------------------------------------\n");
    printf("SENARYO 2: Veri Kaybi (Buffer Overflow) Testi\n");
    printf("Aciklama: Buffer 128 eleman. Yazma %.1f Hz, okuma sadece 500ms'de bir.\n", sample_rate_hz);
 

    IMU_Sim_Init(sample_rate_hz);
    IMU_Sim_RegisterProcessFn();
    RB_Init(&s_buffer);
    Init_Filters(sample_rate_hz);
    Reset_Stats();
    overflow_before = s_buffer.overflow_count;
    
    start_time = Timer_GetTick_ms();
    
    while (duration_ms == 0 || (Timer_GetTick_ms() - start_time) < duration_ms) {
        
        // Write data at sample rate
        if (Timer_CheckPeriod(&last_sample, sample_period_ms)) {
            Get_IMU_Data(&imu_raw, &imu_filter);
            write_count++;
        }
        
        // Read data every 500ms (consumer - reads raw data, conversion can be done here)
        if (Timer_CheckPeriod(&last_print, 500)) {
            LSM6DSM_RawData_t data;
            int drain = 0;
            
            // Read up to 5 items from buffer
            while (drain < 5 && !RB_IsEmpty(&s_buffer)) {
                RB_Read(&s_buffer, &data);
                read_count++;
                drain++;
            }
            
            printf("  t=%5lu ms | Yazilan: %lu | Okunan: %lu | Buffer: %u/%d | Overflow: %lu\n",
                   (unsigned long)(Timer_GetTick_ms() - start_time),
                   (unsigned long)write_count,
                   (unsigned long)read_count,
                   RB_GetCount(&s_buffer), BUFFER_SIZE,
                   (unsigned long)s_buffer.overflow_count);
        }
    }
    
    printf("\n  [Sonuc]\n");
    printf("    Toplam yazilan: %lu\n", (unsigned long)write_count);
    printf("    Toplam okunan: %lu\n", (unsigned long)read_count);
    printf("    Kaybedilen veri: %lu (overflow)\n", (unsigned long)(s_buffer.overflow_count - overflow_before));
    // Division by zero check
    if (write_count > 0) { 
        printf("    Kayip orani: %.1f%%\n", (float)(s_buffer.overflow_count - overflow_before) / (float)write_count * 100.0f);
    }
    Print_Performance();
}


// Scenario 3: Corrupted Data + 50 Hz EMI filtering test
void IMU_Sim_Scenario_Corrupted_Data(uint32_t duration_ms) {
    LSM6DSM_Data_t imu_raw;
    LSM6DSM_FilteredData_t imu_filter;
    uint32_t start_time, last_sample = 0, last_print = 0;
    uint32_t sample_count = 0;
    int spike_injected = 0;
    float sample_rate_hz = LSM6DSM_GetSampleRate();
    uint32_t sample_period_ms = (uint32_t)(1000.0f / sample_rate_hz);
    
    printf("\n");
    printf("--------------------------------------------------------------------------------\n");
    printf("SENARYO 3: Spike + 50 Hz EMI Filtreleme Testi\n");
    printf("Aciklama:\n");
    printf("  - Gyro sinyalinde 50 Hz EMI paraziti mevcut (Notch filtre testi)\n");
    printf("  - Her 50 ornekte accel_x'e +10g spike enjekte edilir (LPF testi)\n");
    printf("  - Ham vs Filtreli karsilastirmasindan Notch + LPF etkisi gorulur.\n");
    
    IMU_Sim_Init(sample_rate_hz);
    IMU_Sim_RegisterProcessFn();
    RB_Init(&s_buffer);
    Init_Filters(sample_rate_hz);
    Reset_Stats();
    
    start_time = Timer_GetTick_ms();
    
    while (duration_ms == 0 || (Timer_GetTick_ms() - start_time) < duration_ms) {
        
        if (Timer_CheckPeriod(&last_sample, sample_period_ms)) {
            Get_IMU_Data(&imu_raw, &imu_filter);
            sample_count++;
            
            // Inject spike every 50 samples
            if (sample_count % 50 == 0) {
                // Apply +10g corrupted data to accel_x: without corrupting filter state,
                // only show single-step LPF response (on a state copy)
                LowPassFilter_t lpf_tmp = s_lpf_ax;
                float spike_filtered = LPF_Update(&lpf_tmp, 10.0f);
                spike_injected++;
                
                printf("  >>> SPIKE ENJEKTE: t=%lu ms | Ham ax=+10.000 g | Filt ax=%+.3f g\n",
                       (unsigned long)(Timer_GetTick_ms() - start_time),
                       spike_filtered);
            }
            
            if (Timer_CheckPeriod(&last_print, PRINT_INTERVAL_MS)) {
                LSM6DSM_RawData_t tmp;
                while (!RB_IsEmpty(&s_buffer)) { RB_Read(&s_buffer, &tmp); }
                Print_Raw_And_Filtered(Timer_GetTick_ms() - start_time, &imu_raw, &imu_filter);
                printf("  [50Hz] Ham Gyro X: %+8.2f dps -> Filt: %+8.2f dps (Notch bastirma)\n",
                       imu_raw.gyro_x, imu_filter.gyro_x);
            }
        }
    }
    
    printf("\n  [Sonuc]\n");
    printf("    Toplam ornek: %lu | Spike sayisi: %d\n", (unsigned long)sample_count, spike_injected);
    printf("    NOT: Ham gyro verisinde 50 Hz EMI bileseni mevcut.\n");
    printf("         Filtreli gyro ciktisinda Notch filtre bu bileseni bastirir.\n");
    Print_Performance();
}


// Scenario 4: Delayed data test
void IMU_Sim_Scenario_Delayed_Data(uint32_t duration_ms) {
    LSM6DSM_Data_t imu_raw;
    LSM6DSM_FilteredData_t imu_filter;
    uint32_t start_time, last_sample = 0, last_print = 0;
    uint32_t sample_count = 0;
    uint32_t skipped_count = 0;
    float sample_rate_hz = LSM6DSM_GetSampleRate();
    uint32_t sample_period_ms = (uint32_t)(1000.0f / sample_rate_hz);
    uint32_t skip_duration_ms = sample_period_ms * 5;
    
    printf("\n");
    printf("--------------------------------------------------------------------------------\n");
    printf("SENARYO 4: Gecikmeli Veri Testi\n");
    printf("Aciklama: Her 30 ornekte, 5 ornek atlanir (sensor gecikme simulasyonu).\n");
    printf("          %lu ms'lik veri boslugu olusur. Filtrenin tepkisi incelenir.\n", (unsigned long)skip_duration_ms);
    
    IMU_Sim_Init(sample_rate_hz);
    IMU_Sim_RegisterProcessFn();
    RB_Init(&s_buffer);
    Init_Filters(sample_rate_hz);
    Reset_Stats();
    
    start_time = Timer_GetTick_ms();
    
    while (duration_ms == 0 || (Timer_GetTick_ms() - start_time) < duration_ms) {
        
        if (Timer_CheckPeriod(&last_sample, sample_period_ms)) {
            sample_count++;
            
            // Skip 5 samples every 30
            if ((sample_count % 30) >= 25) {
                skipped_count++;
                IMU_Sim_UpdateDriver();  // Zamani ilerlet ama okuma yapma
                if ((sample_count % 30) == 25) {
                    printf("  >>> GECIKME: t=%lu ms | 5 ornek atlanacak (%lu ms bosluk)\n", 
                           (unsigned long)(Timer_GetTick_ms() - start_time),
                           (unsigned long)skip_duration_ms);
                }
                continue;
            }
            
            // Normal sampling
            Get_IMU_Data(&imu_raw, &imu_filter);
            
            if (Timer_CheckPeriod(&last_print, PRINT_INTERVAL_MS)) {
                LSM6DSM_RawData_t tmp;
                while (!RB_IsEmpty(&s_buffer)) { 
                    RB_Read(&s_buffer, &tmp); 
                }
                Print_Raw_And_Filtered(Timer_GetTick_ms() - start_time, &imu_raw, &imu_filter);
            }
        }
    }
    
    printf("\n  [Sonuc]\n");
    printf("    Toplam ornek: %lu | Atlanan: %lu | Gercek ornekleme: %lu\n", 
                (unsigned long)sample_count, (unsigned long)skipped_count, (unsigned long)(sample_count - skipped_count));
    printf("    Veri kayip orani: %.1f%%\n", (float)skipped_count / (float)sample_count * 100.0f);
    Print_Performance();
}

// Scenario 0: Run all scenarios sequentially
void IMU_Sim_Scenario_All(uint32_t duration_ms) {
    IMU_Sim_Scenario_Normal(duration_ms);
    IMU_Sim_Scenario_Data_Loss(duration_ms);
    IMU_Sim_Scenario_Corrupted_Data(duration_ms);
    IMU_Sim_Scenario_Delayed_Data(duration_ms);
}