/* Kernel includes. */
/*#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"*/
/*
* Register check tasks, and the tasks used to write over and check the contents
* of the FPU registers, as described at the top of this file.  The nature of
* these files necessitates that they are written in an assembly file.
*/
extern void vRegTest1Task( void *pvParameters );
extern void vRegTest2Task( void *pvParameters );
extern void vRegTestClearFlopRegistersToParameterValue( unsigned long ulValue );
extern unsigned long ulRegTestCheckFlopRegistersContainParameterValue( unsigned long ulValue );

/* The following two variables are used to communicate the status of the
register check tasks to the check software timer.  If the variables keep
incrementing, then the register check tasks have not discovered any errors.  If
a variable stops incrementing, then an error has been found. */
volatile unsigned long ulRegTest1LoopCounter = 0UL, ulRegTest2LoopCounter = 0UL;