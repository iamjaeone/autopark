#include "systeminit.h"

#include "asclin0.h"
#include "asclin1.h"
#include "bluetooth.h"
#include "motor.h"
#include "uart.h"
#include "ultrasonic.h"


void systemInit(){
    bluetoothInit();
    motorInit();
    asclin0InitUart();
    uartInit();
    ultrasonicInit();
}
