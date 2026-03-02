#include "timer.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#include <unistd.h>
#endif


#ifdef _WIN32
static LARGE_INTEGER s_frequency;       // Performance counter frequency
static LARGE_INTEGER s_start_time;      // Start time
#else
static struct timespec s_start_time;    // POSIX start time
#endif

static int s_initialized = 0;


void Timer_Init(void) {
    if (s_initialized) {
        return;
    }
    
#ifdef _WIN32
    QueryPerformanceFrequency(&s_frequency);
    QueryPerformanceCounter(&s_start_time);
#else
    clock_gettime(CLOCK_MONOTONIC, &s_start_time);
#endif
    
    s_initialized = 1;
}

uint32_t Timer_GetTick_ms(void) {
    if (!s_initialized) {
        Timer_Init();
    }
    
#ifdef _WIN32
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    
    uint64_t elapsed = now.QuadPart - s_start_time.QuadPart;
    return (uint32_t)((elapsed * 1000) / s_frequency.QuadPart);
#else
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    
    int64_t sec_diff = now.tv_sec - s_start_time.tv_sec;
    int64_t ns_diff = now.tv_nsec - s_start_time.tv_nsec;
    
    return (uint32_t)((sec_diff * 1000) + (ns_diff / 1000000));
#endif
}

uint64_t Timer_GetTick_us(void) {
    if (!s_initialized) {
        Timer_Init();
    }
    
#ifdef _WIN32
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    
    uint64_t elapsed = now.QuadPart - s_start_time.QuadPart;
    return (elapsed * 1000000) / s_frequency.QuadPart;
#else
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    
    int64_t sec_diff = now.tv_sec - s_start_time.tv_sec;
    int64_t ns_diff = now.tv_nsec - s_start_time.tv_nsec;
    
    return (uint64_t)((sec_diff * 1000000) + (ns_diff / 1000));
#endif
}

void Timer_Delay_ms(uint32_t ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

uint32_t Timer_Elapsed(uint32_t start, uint32_t end) {
    // Handle overflow condition
    if (end >= start) {
        return end - start;
    } else {
        // 32-bit wrap-around
        return (0xFFFFFFFF - start) + end + 1;
    }
}

int Timer_CheckPeriod(uint32_t* last_time, uint32_t period_ms) {
    uint32_t now = Timer_GetTick_ms();
    
    if (Timer_Elapsed(*last_time, now) >= period_ms) {
        *last_time = now;
        return 1;
    }
    
    return 0;
}
