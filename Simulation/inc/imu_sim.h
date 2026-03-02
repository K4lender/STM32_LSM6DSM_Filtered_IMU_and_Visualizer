/*
 * Title: imu_sim.h
 *
 * Date: 20/02/2026
 *
 * Author: Muhammed Selman Çetin
 *
 * GitHub: K4lender
 */

 

#ifndef IMU_SIM_H
#define IMU_SIM_H

#include <stdint.h>

// Noise levels (standard deviation in LSB)
#define SIM_ACCEL_NOISE     50
#define SIM_GYRO_NOISE      30

// Gravity offset (Z-axis, sensor lying flat)
#define SIM_GRAVITY_OFFSET  16384   // ~1g on 2g scale

// Sinusoidal wave parameters
#define SIM_DEFAULT_FREQ    1.0f    // 1 Hz
#define SIM_DEFAULT_AMP     4000    // Amplitude (LSB)

// 50 Hz noise - for Notch filter test
#define SIM_EMI_FREQ        50.0f   // 50 Hz power line frequency
#define SIM_EMI_AMP         800     // EMI amplitude (LSB)

// Initialize simulation with given sample rate
void IMU_Sim_Init(float sample_rate_hz);

// Get accelerometer data from simulation
void IMU_Sim_GetAccel(int16_t* ax, int16_t* ay, int16_t* az);

// Get gyroscope data from simulation
void IMU_Sim_GetGyro(int16_t* gx, int16_t* gy, int16_t* gz);

// Get temperature data from simulation
int16_t IMU_Sim_GetTemperature(void);

// Advance simulation time
void IMU_Sim_Tick(void);

// Reset simulation
void IMU_Sim_Reset(void);

// Get simulation time (ms)
uint32_t IMU_Sim_GetTime(void);

// Inject simulation data into the LSM6DSM driver.
// (IMU_Sim_GetAccel/Gyro/Temperature + Tick + LSM6DSM_InjectRawData)
void IMU_Sim_UpdateDriver(void);

// Register the simulation's LPF+Notch filter chain as LSM6DSM driver process callback.
// Must be called before each scenario starts.
void IMU_Sim_RegisterProcessFn(void);

////////////////////////////////////  SCENARIOS  ///////////////////////////////////////////////////////////

// Scenario 1: Normal data flow
void IMU_Sim_Scenario_Normal(uint32_t duration_ms);

// Scenario 2: Data loss (Buffer overflow) test
void IMU_Sim_Scenario_Data_Loss(uint32_t duration_ms);

// Scenario 3: Corrupted data (spike) test
void IMU_Sim_Scenario_Corrupted_Data(uint32_t duration_ms);

// Scenario 4: Delayed data test
void IMU_Sim_Scenario_Delayed_Data(uint32_t duration_ms);

// Scenario 0: Run all scenarios sequentially
void IMU_Sim_Scenario_All(uint32_t duration_ms);

#endif /* IMU_SIM_H */
