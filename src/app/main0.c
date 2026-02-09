#include "main0.h"
#include "bluetooth.h"
#include "autopark.h"
#include "systeminit.h"
#include "uart.h"

void main0(void)
{
    systemInit();
    myPrintf("System Initialized.\n");
    bluetoothPrintf("System Initialized.\n");
    while (1)
    {
        bluetoothPrintf("Waiting for command...\n");
        char command = bluetoothRecvByteBlocked();
        bluetoothPrintf("Command: %c\n", command);
        switch(command)
        {
            case 'p':
            {
                autoparkExecute();
                break;
            }
            case 't':
            {
                autoparkTune();
                break;
            }
            default:
            {
                break;
            }
        }
    }
}
