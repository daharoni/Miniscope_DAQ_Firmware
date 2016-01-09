/*
 ## Cypress FX3 Camera Kit source file (sensor.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2012,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file implements the I2C based driver for the MT9M114 image sensor used
   in the FX3 HD 720p camera kit.

   Please refer to the Aptina MT9M114 sensor datasheet for the details of the
   I2C commands used to configure the sensor.
 */

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3uart.h>
#include <cyu3i2c.h>
#include <cyu3spi.h> //Added by Daniel 4_9_2015
#include <cyu3types.h>
#include <cyu3gpio.h>
#include <cyu3utils.h>
#include "sensor.h"


/* This function inserts a delay between successful I2C transfers to prevent
   false errors due to the slave being busy.
 */
static void
SensorI2CAccessDelay (
        CyU3PReturnStatus_t status)
{
    /* Add a 10us delay if the I2C operation that preceded this call was successful. */
    if (status == CY_U3P_SUCCESS)
        CyU3PBusyWait (200); //in us
}

/* Write to an I2C slave with two bytes of data. */
CyU3PReturnStatus_t
SensorWrite2B (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t highData,
        uint8_t lowData)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t  preamble;
    uint8_t buf[2];

    /* Validate the I2C slave address. */
    if ((slaveAddr != SENSOR_ADDR_WR) && (slaveAddr != I2C_MEMORY_ADDR_WR))
    {
        CyU3PDebugPrint (4, "I2C Slave address is not valid!\n");
        return 1;
    }

    /* Set the parameters for the I2C API access and then call the write API. */
    preamble.buffer[0] = slaveAddr;
    preamble.buffer[1] = highAddr;
    preamble.buffer[2] = lowAddr;
    preamble.length    = 3;             /*  Three byte preamble. */
    preamble.ctrlMask  = 0x0000;        /*  No additional start and stop bits. */

    buf[0] = highData;
    buf[1] = lowData;

    apiRetStatus = CyU3PI2cTransmitBytes (&preamble, buf, 2, 0);
    SensorI2CAccessDelay (apiRetStatus);

    return apiRetStatus;
}

CyU3PReturnStatus_t
SensorWrite (
        uint8_t slaveAddr,
        uint8_t Addr,
        uint8_t count,
        uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

    /* Validate the I2C slave address. */
    if ((slaveAddr != SENSOR_ADDR_WR) && (slaveAddr != I2C_MEMORY_ADDR_WR))
    {
        CyU3PDebugPrint (4, "I2C Slave address is not valid!\n");
     //   return 1;
    }

    if (count > 64)
    {
        CyU3PDebugPrint (4, "ERROR: SensorWrite count > 64\n");
        return 1;
    }

    /* Set up the I2C control parameters and invoke the write API. */
    preamble.buffer[0] = slaveAddr;
    preamble.buffer[1] = Addr;
    preamble.length    = 2;
    preamble.ctrlMask  = 0x0000;

    apiRetStatus = CyU3PI2cTransmitBytes (&preamble, buf, count, 0);
    SensorI2CAccessDelay (apiRetStatus);

    return apiRetStatus;
}

CyU3PReturnStatus_t
SensorRead2B (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

    if ((slaveAddr != SENSOR_ADDR_RD) && (slaveAddr != I2C_MEMORY_ADDR_RD))
    {
        CyU3PDebugPrint (4, "I2C Slave address is not valid!\n");
    //    return 1;
    }

    preamble.buffer[0] = slaveAddr & I2C_SLAVEADDR_MASK;        /*  Mask out the transfer type bit. */
    preamble.buffer[1] = highAddr;
    preamble.buffer[2] = lowAddr;
    preamble.buffer[3] = slaveAddr;
    preamble.length    = 4;
    preamble.ctrlMask  = 0x0004;                                /*  Send start bit after third byte of preamble. */

    apiRetStatus = CyU3PI2cReceiveBytes (&preamble, buf, 2, 0);
    SensorI2CAccessDelay (apiRetStatus);

    return apiRetStatus;
}

CyU3PReturnStatus_t
SensorRead (
        uint8_t slaveAddr,
        uint8_t Addr,
        uint8_t count,
        uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

    /* Validate the parameters. */
    if ((slaveAddr != SENSOR_ADDR_RD) && (slaveAddr != I2C_MEMORY_ADDR_RD))
    {
        CyU3PDebugPrint (4, "I2C Slave address is not valid!\n");
    //    return 1;
    }
    if ( count > 64 )
    {
        CyU3PDebugPrint (4, "ERROR: SensorWrite count > 64\n");
        return 1;
    }

    preamble.buffer[0] = slaveAddr & I2C_SLAVEADDR_MASK;        /*  Mask out the transfer type bit. */
    preamble.buffer[1] = Addr;
    preamble.buffer[2] = slaveAddr;
    preamble.length    = 3;
    preamble.ctrlMask  = 0x0002;                                /*  Send start bit after third byte of preamble. */

    apiRetStatus = CyU3PI2cReceiveBytes (&preamble, buf, count, 0);
    SensorI2CAccessDelay (apiRetStatus);

    return apiRetStatus;
}



void
SensorInit (
        void)
{
	uint8_t buf[2];

	//Added by Daniel 8_3_2015
	//buf[0] = 0b10000000;
	//SensorWrite (DESER_ADDR_WR, 0x21, 1, buf); // I2C Pass Through All (Deser)



	//SensorRead (SENSOR_ADDR_RD, 0x00, 2, buf);
	//buf[0] = 0;
	//SensorRead (DESER_ADDR_RD, 0x00, 1, buf);
	//CyU3PDebugPrint (4, "Deserializer Value = %d\n", buf[0]);

	 //buf[0] = 0b11001101;
	 //SensorWrite (SER_ADDR_WR, 0x03, 1, buf); // I2C Pass Through All (Ser)
	//------------------------

/*	CyU3PDebugPrint (4, "Hi!\n");
    if (SensorI2cBusTest () == CY_U3P_SUCCESS)         Verify that the sensor is connected.
    {
        CyU3PDebugPrint (4, "Correct Sensor ID!\n");
      //  return;
    }
    else {
    	 CyU3PDebugPrint (4, "Incorrect Sensor ID!\n");
    }*/

#ifdef IMAGE_SENSOR_LVDS
    //Reset
    buf[0] = 0;
    buf[1] = 1;
    SensorWrite (SENSOR_ADDR_WR, 0x0C, 2, buf);
    buf[0] = 0;
    buf[1] = 0;
    SensorWrite (SENSOR_ADDR_WR, 0x0C, 2, buf);

    SensorRead (SENSOR_ADDR_RD, 0x07, 2, buf);
    CyU3PDebugPrint (4, "Control Reg = %d %d\n", buf[0],buf[1]);

    //Enable LVDS driver bit4 = 0
    buf[0] = 0;
    buf[1] = 0;
    SensorWrite (SENSOR_ADDR_WR, 0xB3, 2, buf);

    //De-assert LVDS power down bit1 = 0
    buf[0] = 0;
    buf[1] = 0;
    SensorWrite (SENSOR_ADDR_WR, 0xB1, 2, buf);

    //Issue a soft reset bit0 = 1 followed bu bit0 = 0
    buf[0] = 0;
    buf[1] = 1;
    SensorWrite (SENSOR_ADDR_WR, 0x0C, 2, buf);
    buf[0] = 0;
    buf[1] = 0;
    SensorWrite (SENSOR_ADDR_WR, 0x0C, 2, buf);

    //Force sync pattern for deserializer to lock (added 8/6/2014)
    buf[0] = 0;
    buf[1] = 1;
    SensorWrite (SENSOR_ADDR_WR, 0xB5, 2, buf);
    CyU3PBusyWait(1000);
    buf[0] = 0;
    buf[1] = 0;
    SensorWrite (SENSOR_ADDR_WR, 0xB5, 2, buf);
    CyU3PBusyWait(1000);

    buf[0] = 0;
    buf[1] = 0; //bit0 is for Exposure | bit1 is for Gain
    SensorWrite (SENSOR_ADDR_WR, 0xAF, 2, buf); //Disables auto gain and exposure

//    buf[0] = 0x03; // max int time at 30Hz
//    buf[1] = 0xC0;
    buf[0] = 0x00;//0x05;
    buf[1] = 0xFF;//0x14;
    SensorWrite (SENSOR_ADDR_WR, 0x0B, 2, buf); //Sets total shutter width

    buf[0] = 0x02;//0x3A;
    buf[1] = 0x21;//0x34;
    SensorWrite (SENSOR_ADDR_WR, 0x06, 2, buf); //Sets vertical blanking to extend past shutter width

    buf[0] = 0;//0b00111000;
    buf[1] = 0x00;
    SensorWrite (SENSOR_ADDR_WR, 0x7F, 2, buf); //Test pattern off if buf[0] = 0

    buf[0] = 0x00;
    buf[1] = 0b00110100; //bit 5 enables noise correction
    SensorWrite (SENSOR_ADDR_WR, 0x70, 2, buf); //Turns off noise reduction

    //REPEAT!!!---------------
    CyU3PBusyWait(1000);
    //Reset
        buf[0] = 0;
        buf[1] = 1;
        SensorWrite (SENSOR_ADDR_WR, 0x0C, 2, buf);
        buf[0] = 0;
        buf[1] = 0;
        SensorWrite (SENSOR_ADDR_WR, 0x0C, 2, buf);

        SensorRead (SENSOR_ADDR_RD, 0x07, 2, buf);
        CyU3PDebugPrint (4, "Control Reg = %d %d\n", buf[0],buf[1]);

        //Enable LVDS driver bit4 = 0
        buf[0] = 0;
        buf[1] = 0;
        SensorWrite (SENSOR_ADDR_WR, 0xB3, 2, buf);

        //De-assert LVDS power down bit1 = 0
        buf[0] = 0;
        buf[1] = 0;
        SensorWrite (SENSOR_ADDR_WR, 0xB1, 2, buf);

        //Issue a soft reset bit0 = 1 followed bu bit0 = 0
        buf[0] = 0;
        buf[1] = 1;
        SensorWrite (SENSOR_ADDR_WR, 0x0C, 2, buf);
        buf[0] = 0;
        buf[1] = 0;
        SensorWrite (SENSOR_ADDR_WR, 0x0C, 2, buf);

        //Force sync pattern for deserializer to lock (added 8/6/2014)
        buf[0] = 0;
        buf[1] = 1;
        SensorWrite (SENSOR_ADDR_WR, 0xB5, 2, buf);
        CyU3PBusyWait(1000);
        buf[0] = 0;
        buf[1] = 0;
        SensorWrite (SENSOR_ADDR_WR, 0xB5, 2, buf);
        CyU3PBusyWait(1000);

        buf[0] = 0;
        buf[1] = 0; //bit0 is for Exposure | bit1 is for Gain
        SensorWrite (SENSOR_ADDR_WR, 0xAF, 2, buf); //Disables auto gain and exposure

    //    buf[0] = 0x03; // max int time at 30Hz
    //    buf[1] = 0xC0;
        buf[0] = 0x00;
        buf[1] = 0xFF;
        SensorWrite (SENSOR_ADDR_WR, 0x0B, 2, buf); //Sets total shutter width

        buf[0] = 0x02;
        buf[1] = 0x21;
        SensorWrite (SENSOR_ADDR_WR, 0x06, 2, buf); //Sets vertical blanking to extend past shutter width

        buf[0] = 0;//0b00111000;
        buf[1] = 0x00;
        SensorWrite (SENSOR_ADDR_WR, 0x7F, 2, buf); //Test pattern off if buf[0] = 0

        buf[0] = 0x00;
        buf[1] = 0b00110100; //bit 5 enables noise correction
        SensorWrite (SENSOR_ADDR_WR, 0x70, 2, buf); //Turns off noise reduction
        //----------------------------------------------------------------
	#else
    	buf[0] = SENSOR_ADDR_WR; //sets allowable i2c addresses to send through serializer
    	SensorWrite (DESER_ADDR_WR, 0x08, 1, buf);
    	SensorWrite (DESER_ADDR_WR, 0x10, 1, buf);

    	buf[0] = DAC_ADDR_WR; //sets allowable i2c addresses to send through serializer
    	SensorWrite (DESER_ADDR_WR, 0x09, 1, buf);
    	SensorWrite (DESER_ADDR_WR, 0x11, 1, buf);

        buf[0] = 0;
        buf[1] = 0; //bit0 is for Exposure | bit1 is for Gain
        SensorWrite (SENSOR_ADDR_WR, 0xAF, 2, buf); //Disables auto gain and exposure

            //    buf[0] = 0x03; // max int time at 30Hz
            //    buf[1] = 0xC0;
        buf[0] = 0x00;
        buf[1] = 0xFF;
        SensorWrite (SENSOR_ADDR_WR, 0x0B, 2, buf); //Sets total shutter width

        buf[0] = 0x02;
        buf[1] = 0x21;
        SensorWrite (SENSOR_ADDR_WR, 0x06, 2, buf); //Sets vertical blanking to extend past shutter width




//                buf[0] = 0;//0b00111000;
//                buf[1] = 0x00;
//                SensorWrite (SENSOR_ADDR_WR, 0x7F, 2, buf); //Test pattern off if buf[0] = 0
//
//                buf[0] = 0x00;
//                buf[1] = 0b00110100; //bit 5 enables noise correction
//                SensorWrite (SENSOR_ADDR_WR, 0x70, 2, buf); //Turns off noise reduction
	#endif

}

/*
 * Verify that the sensor can be accessed over the I2C bus from FX3.
 */
uint8_t
SensorI2cBusTest (
        void)
{
    /* The sensor ID register can be read here to verify sensor connectivity. */
    uint8_t buf[2];
    buf[0] = buf[1] = 1;
    /* Reading sensor ID */
    if (SensorRead (SENSOR_ADDR_RD, 0x00, 2, buf) == CY_U3P_SUCCESS)
    {
    	CyU3PDebugPrint (4, "ID Code = %d %d\n", buf[0],buf[1]);
    	//if ((buf[0] == 0x13) && (buf[1] == 0x24)) //for MT9V034
        if ((buf[0] == 0x13) && (buf[1] == 0x13)) // if ((buf[0] == 0x24) && (buf[1] == 0x81))
        {
            return CY_U3P_SUCCESS;
        }
    }
    CyU3PDebugPrint (4, "ID Code = %d %d\n", buf[0],buf[1]);
    return 1;
}



/*
   Get the current brightness setting from the MT9M114 sensor.
 */
uint8_t
SensorGetBrightness (
        void)
{
    uint8_t buf[2];
    SensorRead (SENSOR_ADDR_RD, 0x0B, 2, buf);
    uint16_t temp = (buf[0]<<8) | buf[1];
    uint8_t output = (uint8_t)((float)(temp)/960*255);
    //uint8_t output = (uint8_t)((float)(temp)/13000*255);
    return (uint8_t)output;
}

/*
   Update the brightness setting for the MT9M114 sensor.
 */
void
SensorSetBrightness (
        uint8_t brightness)
{
	uint16_t temp = brightness/255.0*960;
	//uint16_t temp = brightness/255.0*13000;
	uint8_t buf[2];
	    buf[0] = (uint8_t)((temp>>8)&0x007F);
	    buf[1] = (uint8_t)(temp&0x00FF);
	    SensorWrite (SENSOR_ADDR_WR, 0x0B, 2, buf); //Sets total shutter width


//	    buf[0]=(brightness>>4)|(0x30);
//	    buf[1]=(brightness<<4);
//	    CyFxDACSpiWrite(buf);
}
uint8_t
SensorGetGain (
        void)
{
    uint8_t buf[2];
    SensorRead (SENSOR_ADDR_RD, 0xBA, 2, buf);
    //uint16_t temp = (buf[0]<<8) | buf[1];
    return (uint8_t)buf[1]&0b01111111;
}

/*
   Update the brightness setting for the MT9M114 sensor.
 */
void
SensorSetGain (
        uint8_t gain)
{
	if (gain > 64)
		gain=64;

	uint8_t buf[2];
	    buf[0] = 0;
	    buf[1] = gain;

	    SensorWrite (SENSOR_ADDR_WR, 0x35, 2, buf); //Sets Gain
}
