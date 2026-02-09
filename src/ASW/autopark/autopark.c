/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/

#include "autopark.h"
#include "pd_control.h"
 
#include "asclin0.h"
#include "bluetooth.h"
#include "ultrasonic.h"
#include "motor.h"
#include "util.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#define MOTOR_STOP_DELAY 500

#define DEBUG_PRINTF(...) myPrintf(__VA_ARGS__)


/*********************************************************************************************************************/
/*-------------------------------------------------External variables------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

static int g_parkingDistance = 200000;
static int g_parkingSpeedForward = 300;
static int g_parkingSpeedBackward = 300;
static volatile int g_parkingFoundTick = 30;
static int g_goForwardDelay = 0;
static int g_rotateDelay = 480;
static int g_stopDistance = 1000;

static char buf[64];

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

static void findSpace(void);
static void rotate(void);
static void goBackWard(void);

static void tuneParkingDistance(void);
static void tuneParkingSpeed(void);
static void tuneParkingFoundTick(void);
static void tuneRotate(void);
static void tuneStopDistance(void);

/*********************************************************************************************************************/
/*--------------------------------------Core Parking Functions (Combined)--------------------------------------------*/
/*********************************************************************************************************************/

static void findSpace(void)
{
    int curTick = 0;
    int stabilized = 0;

    // 1. PID 및 필터 초기화
    pd_init(LEVEL_LEFT); 
    DEBUG_PRINTF("[findSpace] PID Initialized. Start wall following.\n");

    while (1) // 주차 공간을 찾을 때까지 무한 루프
    {
        // 2. 유효한 좌측 거리 측정
        int ultDis = getDistanceByUltra(ULT_LEFT);
        if(ultDis < 0)
        {
            ultDis = getDistanceByUltra(ULT_LEFT);
        }

        // 3. 주차 공간 탐지 (튜닝된 변수 사용)
        if (ultDis >= g_parkingDistance)
        {
            curTick++;
            DEBUG_PRINTF("[findSpace] Tick #%d (Dist: %d)\n", curTick, ultDis);
            if (curTick >= g_parkingFoundTick)
            {
                DEBUG_PRINTF("[findSpace] Parking Spot Found!\n");
                break; // 공간 찾음! 루프 탈출
            }
        }
        else
        {
            // 공간이 아니면 틱 초기화
            if(curTick > 0)
            {
                DEBUG_PRINTF("[findSpace] Spot lost, resetting tick.\n");
            }
            curTick = 0;
        }

        // 4. PID 조향 값 계산
        int mv = pd_calculateSteeringMv(ultDis, LEVEL_LEFT);

        if(stabilized >= 5)
        {
            mv = 0;
        }
        else
        {
            if(mv < 100 && mv > -100)
            {
                stabilized++;
            }
            else
            {
                stabilized = 0;
            }
        }

        // 5. 모터 제어 (튜닝된 변수 사용)
        motorMovChAPwm(g_parkingSpeedForward + mv, 1); 
        motorMovChBPwm(g_parkingSpeedForward - mv, 1);
    }
    
    // 6. 공간을 찾았으므로 정지
    motorStop();
    DEBUG_PRINTF("[findSpace] Motor Stopped.\n");
    delayMs(50);
}

static void rotate(void)
{
    motorMoveForward(g_parkingSpeedForward);
    delayMs(g_goForwardDelay);
    motorStop();
    delayMs(MOTOR_STOP_DELAY);
    motorMovChAPwm(0, 1);
    motorMovChBPwm(1000, 0);
    delayMs(g_rotateDelay);
    motorStop();
}

static void goBackWard(void)
{
    motorMoveReverse(g_parkingSpeedBackward);

    // int rearDis = getDistanceByUltra(ULT_REAR);
    // while (rearDis > g_stopDistance)
    // {
    //     rearDis = getDistanceByUltra(ULT_REAR);
    //     delayMs(50);
    // }
    delayMs(g_stopDistance);
    motorStop();
}

/*********************************************************************************************************************/
/*----------------------------------------Tuning Functions (from autopark.c)-----------------------------------------*/
/*********************************************************************************************************************/

static void tuneParkingDistance(void)
{
    while (1)
    {
        bluetoothPrintf("주차 공간 입력 [c] - 왼쪽 초음파 거리, [y] - 확인 (현재거리: %d)\n", g_parkingDistance);
        bluetoothScanf("%s", buf);
        if (buf[0] == 'c') 
        {
            int leftDis = getDistanceByUltra(ULT_LEFT);
            bluetoothPrintf("현재 초음파 거리: %d\n", leftDis);
        }
        else if (buf[0] == 'y')
        {
            bluetoothPrintf("주차 공간 설정 완료: %d\n", g_parkingDistance);
            break;
        }
        else
        {
            g_parkingDistance = atoi(buf);
            bluetoothPrintf("parkingDistance 변경 완료: %d\n", g_parkingDistance);
        }
    }
}

static void tuneParkingSpeed(void)
{
    while (1)
    {
        bluetoothPrintf("[주차 공간 찾기] 직진 후진 속도 조절\n");
        bluetoothPrintf("?[y] - 확인\t(현재 직진: %d, 현재 후진: %d)\n", g_parkingSpeedForward, g_parkingSpeedBackward);
        
        bluetoothScanf("%s", buf);

        if (buf[0] == 'y')
        {
            bluetoothPrintf("속도 설정 완료\n");
            bluetoothPrintf("직진: %d\t후진: %d\n", g_parkingSpeedForward, g_parkingSpeedBackward);
            break;
        }
        else
        {
            char* first = strtok((char*)buf, " ");
            char* second = strtok(NULL, " ");
            if (first) g_parkingSpeedForward = atoi(first);
            if (second) g_parkingSpeedBackward = atoi(second);
            bluetoothPrintf("속도 변경: %d %d\n", g_parkingSpeedForward, g_parkingSpeedBackward);
        }

        motorMoveForward(g_parkingSpeedForward);
        delayMs(2000);
        motorStop();
        delayMs(MOTOR_STOP_DELAY);
        motorMoveReverse(g_parkingSpeedBackward);
        delayMs(2000);
        motorStop();
        delayMs(MOTOR_STOP_DELAY);
    }
}

static void tuneParkingFoundTick(void)
{
    while (1)
    {
        bluetoothPrintf("?[주차 공간 찾기] Tick 값 설정 [y] - 확인 (현재 Tick: %d) [t] - 테스트 [i] - PID Gain 설정\n", g_parkingFoundTick);
        bluetoothScanf("%s", buf);
        
        if (buf[0] == 'y')
        {
            bluetoothPrintf("Tick 값 설정 완료: %d\n", g_parkingFoundTick);
            break;
        }
        else if(buf[0] == 't')
        {
            bluetoothPrintf("PID로 공간 탐색 테스트 시작...\n");
            findSpace();
            bluetoothPrintf("테스트 완료.\n");
        }
        else if(buf[0] == 'i')
        {
            pd_printState();
            bluetoothPrintf("PID Gain 설정 (Kp Kd) 형식으로 입력:\n");
            bluetoothScanf("%s", buf);
            char* first = strtok((char*)buf, " ");
            char* second = strtok(NULL, " ");
            if (first) pd_setGain(0, atof(first));
            if (second) pd_setGain(1, atof(second));
            pd_printState();
            bluetoothPrintf("PID Gain 설정 완료.\n");
        }
        else
        {
            g_parkingFoundTick = atoi(buf);
        }
    }
}

static void tuneRotate(void)
{
    while (1)
    {
        bluetoothPrintf("[주차] 직진 & 회전 딜레이 조절 (현재 직진 딜레이: %d, 회전 딜레이: %d)\n", g_goForwardDelay, g_rotateDelay);
        bluetoothPrintf("?[y] - 확인 [r] - 주차 공간 찾기 (PID로)\n");
        bluetoothScanf("%s", buf);

        if (buf[0] == 'y')
        {
            bluetoothPrintf("딜레이 설정 완료 직진: %d\t회전: %d\n", g_goForwardDelay, g_rotateDelay);
            break;
        }
        else if (buf[0] == 'r')
        {
            bluetoothPrintf("PID로 공간 탐색 시작...\n");
            findSpace();
            bluetoothPrintf("공간 탐색 완료.\n");
        }
        else
        {
            char* first = strtok((char*)buf, " ");
            char* second = strtok(NULL, " ");
            if (first) g_goForwardDelay = atoi(first);
            if (second) g_rotateDelay = atoi(second);
            bluetoothPrintf("딜레이 변경: %d %d\n", g_goForwardDelay, g_rotateDelay);
        }
        rotate();
    }
}

static void tuneStopDistance(void)
{
    while (1)
    {
        bluetoothPrintf("[주차] 후진 거리 조절 (현재 후진 거리: %d)\n", g_stopDistance);
        bluetoothPrintf("?[c] - 뒤쪽 거리 출력\t[y] - 확인 \n");
        bluetoothScanf("%s", buf);
        
        if (buf[0] == 'y')
        {
            bluetoothPrintf("후진 거리 설정 완료: %d\n", g_stopDistance);
            break;
        }
        else if (buf[0] == 'c')
        {
            int rearDis = getDistanceByUltra(ULT_REAR);
            bluetoothPrintf("현재 초음파 거리: %d\n", rearDis);
        }
        else
        {
            g_stopDistance = atoi(buf);
            goBackWard();
        }
    }
}

/*********************************************************************************************************************/
/*--------------------------------------Public Functions (Entry Points)----------------------------------------------*/
/*********************************************************************************************************************/

void autoparkTune(void)
{
    boolean isTuned = FALSE;

    while (1)
    {
        bluetoothPrintf("\n");
        bluetoothPrintf("\n");
        bluetoothPrintf("===========현재 값 (PID 적용됨)===========\n");
        bluetoothPrintf("1. [주차 공간 찾기] 주차공간: %d\n", g_parkingDistance);
        bluetoothPrintf("2. [주차 공간 찾기] 전진(PID)속도: %d\t", g_parkingSpeedForward);
        bluetoothPrintf("후진속도: %d\n", g_parkingSpeedBackward);
        bluetoothPrintf("3. [주차 공간 찾기] Tick: %d\n", g_parkingFoundTick);
        bluetoothPrintf("4. [주차 90도 들어가기] 전진 딜레이: %d\t", g_goForwardDelay);
        bluetoothPrintf("4. 회전 딜레이: %d\n", g_rotateDelay);
        bluetoothPrintf("5. [주차] 후진 정지 거리: %d\n", g_stopDistance);
        bluetoothPrintf("?[r] - 시험 주행\t[c]- 확인\t[#]- 재설정\n");
        
        bluetoothScanf("%s", buf);
        
        switch (buf[0])
        {
        case 'r':
            autoparkExecute();
            break;
        case 'c':
            isTuned = TRUE;
            break;
        case '1':
            tuneParkingDistance();
            break;
        case '2':
            tuneParkingSpeed();
            break;
        case '3':
            tuneParkingFoundTick();
            break;
        case '4':
            tuneRotate();
            break;
        case '5':
            tuneStopDistance();
            break;
        default:
            bluetoothPrintf("UNKNOWN COMMAND\n");
            break;
        }

        if (isTuned)
            break;
    }
}

void autoparkExecute(void)
{
    bluetoothPrintf("[autopark] 1. Starting PID Space Finding...\n");
    findSpace(); // PID로 벽을 따라가며 공간 탐색
    
    bluetoothPrintf("[autopark] 2. Executing Rotation...\n");
    rotate();     // 90도 회전
    
    delayMs(MOTOR_STOP_DELAY); // 회전 후 잠시 대기
    
    bluetoothPrintf("[autopark] 3. Executing Backward Maneuver...\n");
    goBackWard(); // 후진 주차
    
    bluetoothPrintf("[autopark] Parking Complete.\n");
}