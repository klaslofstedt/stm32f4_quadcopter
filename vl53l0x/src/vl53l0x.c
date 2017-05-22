#include "vl53l0x.h"
#include "uart.h"

#define VERSION_REQUIRED_MAJOR  1
#define VERSION_REQUIRED_MINOR  0
#define VERSION_REQUIRED_BUILD  1

#define STR_HELPER( x ) #x
#define STR( x )        STR_HELPER(x)

VL53L0X_Error Status = VL53L0X_ERROR_NONE;
VL53L0X_Dev_t MyDevice;
VL53L0X_Dev_t *pMyDevice  = &MyDevice;
VL53L0X_Version_t Version;
VL53L0X_Version_t *pVersion   = &Version;
VL53L0X_DeviceInfo_t DeviceInfo;


void VL53L0X_init(void) 
{
    uart_printf("Init VL53L0X\n\r");
    //int32_t   status_int;
    //int32_t   init_done = 0;
    
    uint32_t  refSpadCount;
    uint8_t   isApertureSpads;
    uint8_t   VhvSettings;
    uint8_t   PhaseCal;
    
    // Initialize Comms
    pMyDevice->I2cDevAddr      =  VL53L0X_I2C_ADDR;  // 7 bit addr
    pMyDevice->comms_type      =  1;
    pMyDevice->comms_speed_khz =  400;
    
    Status = VL53L0X_DataInit(&MyDevice);         // Data initialization
    Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
    if( Status == VL53L0X_ERROR_NONE )  {
        if((DeviceInfo.ProductRevisionMinor != 1)){
            Status = VL53L0X_ERROR_NOT_SUPPORTED;
        }
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        Status = VL53L0X_StaticInit( pMyDevice ); // Device Initialization
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {       
        Status = VL53L0X_PerformRefSpadManagement( pMyDevice, &refSpadCount, &isApertureSpads ); // Device Initialization
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        Status = VL53L0X_PerformRefCalibration( pMyDevice, &VhvSettings, &PhaseCal );           // Device Initialization
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        Status = VL53L0X_SetDeviceMode( pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING );        // Setup in single ranging mode
    }
    
    // Enable/Disable Sigma and Signal check
    if( Status == VL53L0X_ERROR_NONE ) {
        Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1 );
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1 );
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        Status = VL53L0X_SetLimitCheckValue( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)( 1.5 * 0.023 * 65536 ) );
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        uart_printf("Init VL53L0X succeded\n\r");
    } 
    else{
        uart_printf("Init VL53L0X failed\n\r");
    }
}


VL53L0X_Error VL53L0X_getSingleRangingMeasurement(VL53L0X_RangingMeasurementData_t *RangingMeasurementData)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    FixPoint1616_t LimitCheckCurrent;
    
    
    /*
    *  Step  4 : Test ranging mode
    */
    
    if( Status == VL53L0X_ERROR_NONE ) {
        
        //uart_printf( F( "sVL53L0X: PerformSingleRangingMeasurement" ) );
        
        Status = VL53L0X_PerformSingleRangingMeasurement( pMyDevice, RangingMeasurementData );
        
        
        VL53L0X_printRangeStatus( RangingMeasurementData );
        
        VL53L0X_GetLimitCheckCurrent( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent );
        
        /*uart_printf( F( "RANGE IGNORE THRESHOLD: " ) );
        uart_printf( (float)LimitCheckCurrent / 65536.0 );
        
        uart_printf( F( "Measured distance: " ) );
        uart_printf( RangingMeasurementData->RangeMilliMeter );*/
        
    }
    
    return Status;
}




void VL53L0X_printRangeStatus( VL53L0X_RangingMeasurementData_t* pRangingMeasurementData )
{
    char buf[ VL53L0X_MAX_STRING_LENGTH ];
    uint8_t RangeStatus;
    
    /*
    * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
    */
    
    RangeStatus = pRangingMeasurementData->RangeStatus;
    
    VL53L0X_GetRangeStatusString( RangeStatus, buf );
    
    /*uart_printf( F("Range Status: " ) );
    uart_printf( RangeStatus );
    uart_printf( F( " : " ) );
    uart_printf( buf );*/
}

VL53L0X_Error rangingTest(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData) 
{ 
    //debug = false;
    VL53L0X_getSingleRangingMeasurement(pRangingMeasurementData);
    
    return VL53L0X_ERROR_NONE;
};
