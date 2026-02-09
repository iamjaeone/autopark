/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/

#include "pd_control.h"
#include "bluetooth.h"
#include "asclin0.h"
#include "ultrasonic.h"
#include "util.h"
#include <stdlib.h>

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#ifdef NDEBUG
#define DEBUG_PRINTF(...) ((void)0)
#else
#define DEBUG_PRINTF(...) myPrintf(__VA_ARGS__)
#endif

#define FILTER_SIZE 6
#define ABNORMAL_DIFF 3000
#define MV_MAX 200
#define MV_MIN -200

/*********************************************************************************************************************/
/*-------------------------------------------------Static Variables--------------------------------------------------*/
/*********************************************************************************************************************/

// // PD 게인
static float g_Kp = 0.0;
static float g_Kd = 0.2;

// PD 계산용 변수
// static float g_error = 0;
static int g_error = 0;
static int g_last_error = 0;
static int g_derivative = 0;
// static float g_targetDistance = 0;


// 이동 평균 필터용 변수
static uint32 g_readings[FILTER_SIZE] = {0};
static int g_read_index = 0;
static int g_cur_readings_num = 0;
static uint32 g_total = 0;
static uint32 g_filtered_distance = 0;


static uint32 g_targetDistance = 0;
static uint32 g_previous_filtered_distance = 0;
static uint32 g_current_filtered_distance = 0;



// --- [시작] 여기에 새 EMA 필터 변수 추가 ---
// EMA(Low-Pass Filter) 설정
// static float g_ema_alpha = 0.1; // 필터 강도 (0.1 ~ 0.5 추천)
// static float g_previous_filtered_distance = 0.0;
// static float g_current_filtered_distance = 0.0;
// static boolean g_filter_initialized = FALSE;
// --- [끝] 여기까지 추가 ---


/*********************************************************************************************************************/
/*-------------------------------------------Private Function Prototypes---------------------------------------------*/
/*********************************************************************************************************************/

// static float getFilteredDistance(int distance);
static uint32 getFilteredDistance(int distance);

/*********************************************************************************************************************/
/*------------------------------------------Private Function Implementations-----------------------------------------*/
/*********************************************************************************************************************/

// // --- [시작] 새 EMA 필터 함수로 교체 ---
// static float getFilteredDistance(int distance)
// {
//     float current_value = (float)distance;

//     // 필터가 초기화되지 않았다면, 현재 값으로 즉시 설정
//     if (g_filter_initialized == FALSE)
//     {
//         g_current_filtered_distance = current_value;
//         g_filter_initialized = TRUE;
//     }
//     else
//     {
//         // E-MA (Low-Pass Filter) 공식
//         // filtered = alpha * new_value + (1 - alpha) * old_fil tered_value
//         g_current_filtered_distance = (g_ema_alpha * current_value) + ((1.0f - g_ema_alpha) * g_current_filtered_distance);
//     }
//     return g_current_filtered_distance;
// }
// // --- [끝] 여기까지 교체 ---


static uint32 getFilteredDistance(int distance)
{
    DEBUG_PRINTF("[GetFilteredDistance] distance: %d\n", distance);
    g_total -= g_readings[g_read_index];

    g_readings[g_read_index] = distance;
    g_total += distance;
    g_read_index = (g_read_index + 1) % FILTER_SIZE;

    if (g_cur_readings_num < FILTER_SIZE)
    {
        g_cur_readings_num++;
    }
    DEBUG_PRINTF("[GetFilteredDistance] total: %d\n", g_total);
    DEBUG_PRINTF("[GetFilteredDistance] cur_readings_num: %d\n", g_cur_readings_num);
    DEBUG_PRINTF("[GetFilteredDistance] return total / cur_reading_num: %d\n", g_total / g_cur_readings_num);
    return g_total / g_cur_readings_num;
}

/*********************************************************************************************************************/
/*-------------------------------------------Public Function Implementations-----------------------------------------*/
/*********************************************************************************************************************/

void pd_printState(void)
{
    bluetoothPrintf("Cur Gain: %f\t%f\n", g_Kp, g_Kd);
    myPrintf("Cur Gain: %f\t%f\n", g_Kp, g_Kd);
}

void pd_setGain(int n, float i)
{
    if (n == 0)
    {
        g_Kp = i;
        bluetoothPrintf("cur Kp: %f\n", g_Kp);
    }
    if (n == 1)
    {
        g_Kd = i;
        bluetoothPrintf("cur Kd: %f\n", g_Kd);
    }
}

// void pd_init(LevelDir dir)
// {
//     UltraDir ultDir = (dir == LEVEL_LEFT) ? ULT_LEFT : ULT_RIGHT;

//     g_integral = 0;
//     g_last_error = 0;

//     g_total = 0;
//     g_read_index = 0;
//     g_cur_readings_num = 0;

//     for (int i = 0; i < FILTER_SIZE; i++)
//     {
//         g_readings[i] = 0;
//     }

//     // 유효한 거리 값으로 필터 초기화
//     int ultDis = getDistanceByUltra(ultDir);
//     g_previous_filtered_distance = getFilteredDistance(ultDis);
// }

void updateTargetDistance(uint32 distance)
{
    g_targetDistance = distance;
}

void pd_init(LevelDir dir)
{
    UltraDir ultDir = (dir == LEVEL_LEFT) ? ULT_LEFT : ULT_RIGHT;

    g_last_error = 0;

    // EMA 필터 초기화 플래그 설정
    // g_filter_initialized = FALSE; 

    // 유효한 거리 값으로 필터 초기화
    int ultDis = getDistanceByUltra(ultDir);
    if (ultDis < 0) ultDis = getDistanceByUltra(ultDir); // 재시도

    // 필터를 첫 번째 유효한 값으로 설정 (getFilteredDistance 내부 로직이 처리)
    g_previous_filtered_distance = getFilteredDistance(ultDis);
    // 이 호출로 g_filter_initialized = TRUE가 되고,
    // g_current_filtered_distance가 ultDis로 설정되며,
    // g_previous_filtered_distance도 동일한 값으로 설정됩니다.
    g_targetDistance = g_previous_filtered_distance;
}


// --- [시작] 새 pd_calculateSteeringMv 함수로 교체 ---
int pd_calculateSteeringMv(int ultDis, LevelDir dir)
{
    DEBUG_PRINTF("[getMv] ultDis: %d\n", ultDis);
    
    // 1. 새 거리 값으로 필터 업데이트
    g_current_filtered_distance = getFilteredDistance(ultDis);

    DEBUG_PRINTF("[getMv] curfilteredDistance: %f\n", g_current_filtered_distance);

    // 2. 에러 계산 (float으로 수행, 로직은 동일)
    g_error = g_targetDistance - g_current_filtered_distance;

    // 3. 비정상 값 체크
    if (g_error >= (float)ABNORMAL_DIFF || g_error <= (float)-ABNORMAL_DIFF)
    {
        pd_init(dir);
        return 0;
    }

    g_derivative = g_error - g_last_error;

    float p_term = g_Kp * g_error;
    float d_term = g_Kd * g_derivative;

    float unconstrained_output = p_term + d_term;

    // // 6. Anti-Windup
    // if ((unconstrained_output > (float)MV_MAX && g_error > 0) ||
    //     (unconstrained_output < (float)MV_MIN && g_error < 0))
    // {
    //     // Windup occurs
    // }
    // else
    // {
    //     // Ki=0이므로 이 부분은 현재 중요하지 않음
    //     g_integral += g_error; 
    // }

    // 7. Saturation (출력 제한)
    int output;
    if (unconstrained_output > (float)MV_MAX)
    {
        output = MV_MAX;
    }
    else if (unconstrained_output < (float)MV_MIN)
    {
        output = MV_MIN;
    }
    else
    {
        output = (int)unconstrained_output;
    }

    // 8. 상태 업데이트 (다음 루프 준비)
    g_last_error = g_error;
    g_previous_filtered_distance = g_current_filtered_distance;

    bluetoothPrintf("%d,%d,%d\n", g_error, g_derivative, output);
    return output;
}




// int pd_calculateSteeringMv(int ultDis, LevelDir dir)
// {
//     DEBUG_PRINTF("[getMv] ultDis: %d\n", ultDis);
//     g_current_filtered_distance = getFilteredDistance(ultDis);

//     DEBUG_PRINTF("[getMv] curfilteredDistance: %d\n", g_current_filtered_distance);
//     g_error = (float)g_previous_filtered_distance - (float)g_current_filtered_distance;

//     if (g_error >= ABNORMAL_DIFF || g_error <= -ABNORMAL_DIFF)
//     {
//         pd_init(dir);
//         return 0;
//     }

//     g_derivative = g_error - g_last_error;
//     if (g_cur_readings_num <= 2)
//         g_derivative = 0;

//     // PID
//     float p_term = g_Kp * g_error;
//     float i_term = g_Ki * g_integral;
//     float d_term = g_Kd * g_derivative;

//     float unconstrained_output = p_term + i_term + d_term;

//     // Anti-Windup
//     if ((unconstrained_output > (float)MV_MAX && g_error > 0) ||
//         (unconstrained_output < (float)MV_MIN && g_error < 0))
//     {
//         // Windup occurs
//     }
//     else
//     {
//         g_integral += g_error;
//     }

//     // Saturation
//     int output;
//     if (unconstrained_output > (float)MV_MAX)
//     {
//         output = MV_MAX;
//     }
//     else if (unconstrained_output < (float)MV_MIN)
//     {
//         output = MV_MIN;
//     }
//     else
//     {
//         output = (int)unconstrained_output;
//     }

//     g_last_error = g_error;
//     g_previous_filtered_distance = g_current_filtered_distance;

//     bluetoothPrintf("%f,%f,%f,%d\n", g_error, g_integral, g_derivative, output);
//     return output;
// }