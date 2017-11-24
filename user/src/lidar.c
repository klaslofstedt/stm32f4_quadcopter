#include "lidar.h"
#include "i2c_1.h"
#include "uart.h"
#include "freertos_time.h"

#define LIDAR_LITE_ADDRESS              0x62

#define LIDAR_LITE_ACQ_COMMAND          0x00
#define LIDAR_LITE_STATUS               0x01
#define LIDAR_LITE_SIG_COUNT_VAL        0x02
#define LIDAR_LITE_ACQ_CONFIG_REG       0x04
#define LIDAR_LITE_THRESHOLD_BYPASS     0x1C

#define LIDAR_LITE_DATA                 0x8F

// Implemented based on: http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf

/*void lidar_init(lidar_data_t *in)
{
// kommentera bort ner till -----
uint8_t status;
Sensors_I2C1_ReadRegister(LIDAR_LITE_ADDRESS, LIDAR_LITE_STATUS, 1, &status);
//uint8_t status = i2cRead(LIDAR_LITE_ADDRESS, LIDAR_LITE_STATUS);
if (!(status & (1 << 6)) && (status & (1 << 5))) { // Read process error and health flag
uart_printf("LIDAR-Lite v3 found\n\r");
delay_ms(100);
    } else {

while (1){
uart_printf("Could not find LIDAR-Lite v3: %d\n\r", status);
delay_ms(500);
        }
    }
// -------
Sensors_I2C1_WriteReg(LIDAR_LITE_ADDRESS, LIDAR_LITE_ACQ_COMMAND, 0x00); //  Reset FPGA, all registers return to default values
delay_ms(22); // Wait 22 ms after reset according to datasheet

// Default configuration
Sensors_I2C1_WriteReg(LIDAR_LITE_ADDRESS, LIDAR_LITE_SIG_COUNT_VAL, 0x80); // Maximum acquisition count
Sensors_I2C1_WriteReg(LIDAR_LITE_ADDRESS, LIDAR_LITE_ACQ_CONFIG_REG, 0x08); // Disable measurement quick termination
Sensors_I2C1_WriteReg(LIDAR_LITE_ADDRESS, LIDAR_LITE_THRESHOLD_BYPASS, 0x00); // Use default valid measurement detection algorithm
}

uint8_t triggerLidarLite(void) {
uint8_t busyFlag = 0;

if (!busyFlag) {
static uint8_t biasCounter = 0;
if (biasCounter == 0){
Sensors_I2C1_WriteReg(LIDAR_LITE_ADDRESS, LIDAR_LITE_ACQ_COMMAND, 0x04); // Take acquisition & correlation processing with receiver bias correction
        }
        else{
Sensors_I2C1_WriteReg(LIDAR_LITE_ADDRESS, LIDAR_LITE_ACQ_COMMAND, 0x03); // Take acquisition & correlation processing without receiver bias correction
        }
if (++biasCounter >= 100){ // Receive bias correction every 100th measurement according to the datasheet
biasCounter = 0;
        }
    }
unsigned char reg_read;
Sensors_I2C1_ReadRegister(LIDAR_LITE_ADDRESS, LIDAR_LITE_STATUS, 1, &reg_read);
uart_printf("reg_read: %d ", reg_read);
busyFlag = reg_read & 0x01; // Read status register to check busy flag
return !busyFlag; // Return true if new measurement is ready
}

// Returns the distance in mm. Range is 0-40000 mm or -1 if the value is invalid.
void lidar_read(lidar_data_t *in) 
{
if(triggerLidarLite() == 1){
uart_printf("LIDAR V3 trigger successfull ");
    }
    else{
uart_printf("LIDAR V3 trigger NOT successfull ");
    }
unsigned char i2cBuffer[2]; // Buffer for I2C data
Sensors_I2C1_ReadRegister(LIDAR_LITE_ADDRESS, LIDAR_LITE_DATA, 2, i2cBuffer); // Read distance measurement

int32_t distance = (int16_t)((i2cBuffer[0] << 8) | i2cBuffer[1]); // Return the distance in cm
uart_printf("LIDAR V3: %d\n\r", distance);
in->range_cm = (float)distance;
}*/




#define LIDAR_ADDR_DEFAULT 0x62

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

static void write(char myAddress, char myValue, char lidarliteAddress);
//static void read2(char myAddress, int numOfBytes, unsigned short* arrayToSave, uint8_t monitorBusyFlag, char lidarliteAddress);
static void read2(char myAddress, int numOfBytes, unsigned char* arrayToSave, uint8_t monitorBusyFlag, char lidarliteAddress);


void lidar_init(lidar_data_t *in)
{
    uint8_t lidarliteAddress = LIDAR_ADDR_DEFAULT;
    uint8_t configuration = 0;
    
    write(0x00, 0x00, lidarliteAddress);
    delay_ms(30);
    
    switch (configuration)
    {
    case 0: // Default mode, balanced performance
        write(0x02,0x80,lidarliteAddress); // Default
        write(0x04,0x08,lidarliteAddress); // Default
        write(0x1c,0x00,lidarliteAddress); // Default
        uart_printf("Lidar initialized?");
        break;
        
    case 1: // Short range, high speed
        write(0x02,0x1d,lidarliteAddress);
        write(0x04,0x08,lidarliteAddress); // Default
        write(0x1c,0x00,lidarliteAddress); // Default
        break;
        
    case 2: // Default range, higher speed short range
        write(0x02,0x80,lidarliteAddress); // Default
        write(0x04,0x00,lidarliteAddress);
        write(0x1c,0x00,lidarliteAddress); // Default
        break;
        
    case 3: // Maximum range
        write(0x02,0xff,lidarliteAddress);
        write(0x04,0x08,lidarliteAddress); // Default
        write(0x1c,0x00,lidarliteAddress); // Default
        break;
        
    case 4: // High sensitivity detection, high erroneous measurements
        write(0x02,0x80,lidarliteAddress); // Default
        write(0x04,0x08,lidarliteAddress); // Default
        write(0x1c,0x80,lidarliteAddress);
        break;
        
    case 5: // Low sensitivity detection, low erroneous measurements
        write(0x02,0x80,lidarliteAddress); // Default
        write(0x04,0x08,lidarliteAddress); // Default
        write(0x1c,0xb0,lidarliteAddress);
        break;
    }
}

void lidar_read(lidar_data_t *in)
{
    //in->range_cm = -1;
    
    uint8_t biasCorrection = 1;
    uint8_t lidarliteAddress = LIDAR_ADDR_DEFAULT;
    
    if(biasCorrection)
    {
        // Take acquisition & correlation processing with receiver bias correction
        write(0x00,0x04,lidarliteAddress);
    }
    else
    {
        // Take acquisition & correlation processing without receiver bias correction
        write(0x00,0x03,lidarliteAddress);
    }
    // Array to store high and low bytes of distance
    unsigned char distanceArray[2];
    //unsigned char distanceArray2 = 0;
    // Read two bytes from register 0x8f (autoincrement for reading 0x0f and 0x10)
    read2(0x8f, 2, distanceArray, 1, lidarliteAddress);
    // Shift high byte and add to low byte
    int distance = (distanceArray[0] << 8) + distanceArray[1];
    in->range_cm = (float)distance;
    uart_printf("lidar :%d\n\r", distance);
    //return(distance);
}

void lidar_reset()
{
    uint8_t lidarliteAddress = LIDAR_ADDR_DEFAULT;
    write(0x00,0x00,lidarliteAddress);
    delay_ms(100);
}


static void read2(char myAddress, int numOfBytes, unsigned char* arrayToSave, uint8_t monitorBusyFlag, char lidarliteAddress)
{
    int busyFlag = 0; // busyFlag monitors when the device is done with a measurement
    if(monitorBusyFlag)
    {
        busyFlag = 1; // Begin read immediately if not monitoring busy flag
    }
    int busyCounter = 0; // busyCounter counts number of times busy flag is checked, for timeout
    
    while(busyFlag != 0) // Loop until device is not busy
    {
        // Read status register to check busy flag
        //write(0x01); // Set the status register to be read
        uint8_t busy_byte;
        Sensors_I2C1_ReadRegister(lidarliteAddress, 0x01, 0x01, &busy_byte);
        
        busyFlag = bitRead(busy_byte, 0); // Assign the LSB of the status register to busyFlag
        uart_printf("busyFlag%d\n\r", busyFlag);
        busyCounter++; // Increment busyCounter for timeout
        
        // Handle timeout condition, exit while loop and goto bailout
        if(busyCounter > 9999)
        {
            goto bailout; //------------------------------------------------------?
            uart_printf("Lidar bailout\n\r");
        }
    }
    uart_printf("Lidar finished busyflag!\n\r");
    
    // Device is not busy, begin read
    if(busyFlag == 0)
    {
        //uint8_t busy_byte;
        Sensors_I2C1_ReadRegister((unsigned char)lidarliteAddress, (unsigned char)myAddress, (unsigned short)numOfBytes, arrayToSave);
        uart_printf("arraytToSave: %d", arrayToSave[0]);
        uart_printf("%d\n\r", arrayToSave[1]);
    }
    
    // bailout reports error over serial
    if(busyCounter > 9999)
    {
    bailout:
        busyCounter = 0;
        uart_printf("> read failed");
    }
}


static void write(char myAddress, char myValue, char lidarliteAddress)
{
    Sensors_I2C1_WriteReg(lidarliteAddress, myAddress, myValue); 
    delay_ms(1); // 1 ms delay for robustness with successive reads and writes
}