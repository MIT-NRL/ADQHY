/**
 * AreaDetector driver for QHY cameras.
 * 
 * This driver was originally developed with an QHY294M Pro
 * using QHYs SDK version 24.1.9.12
 * 
 * This driver was written by Sean Fayfar based off QHY drivers written by Greg Guyotte
 * Dec 2024
 * 
 * 
 */

#include "QHYDriver.h"
#include "ADDriver.h"
#include "qhyccd.h"
#include "NDArray.h"
#include "NDAttribute.h"
#include "asynDriver.h"
#include "epicsExport.h"
#include "epicsStdio.h"
#include "epicsTypes.h"

#include <cstdint>
#include <stdio.h>
#include <string.h>
#include <string>

#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <iocsh.h>

static const char *driverName = "QHYDriver";
static const char *driverVersion = "0.0.1";

static void QHYDriverCaptureTaskC(void *drvPvt) {
    QHYDriver *driver = (QHYDriver *)drvPvt;
    driver->captureTask();
}

static void QHYDriverPollingTaskC(void *drvPvt) {
    QHYDriver *driver = (QHYDriver *)drvPvt;
    driver->pollingTask();
}

QHYDriver::QHYDriver(const char *portName, int maxBuffers, size_t maxMemory,
                     int priority, int stackSize)
    : ADDriver(portName, 1, 0, maxBuffers, maxMemory, 0,
               0,    /* No interfaces beyond those set in ADDriver.cpp */
               0, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize) {

    // createParam(ADOffsetString, asynParamFloat64, &ADOffset);

    //Add the params to the paramLib 
    //createParam adds the parameters to all param lists automatically (using maxAddr).
    createParam(QHYReadoutModeParamString,  asynParamInt32,    &QHYReadoutModeParam);
    createParam(QHYReadModeParamString,     asynParamInt32,    &QHYReadModeParam);
    createParam(QHYOffsetParamString,       asynParamInt32,    &QHYOffsetParam);
    createParam(QHYBitDepthParamString,     asynParamInt32,    &QHYBitDepthParam);
    createParam(QHYPercentCompleteParamString, asynParamFloat64,  &QHYPercentCompleteParam);
    createParam(QHYTEPowerParamString,      asynParamFloat64,  &QHYTEPowerParam);
    createParam(QHYLastParamString,         asynParamInt32,    &QHYLastParam);
    createParam(QHYUSBTrafficParamString,         asynParamInt32,    &QHYUSBTrafficParam);

    printf("\n\n\n\n\n");

    int status = asynSuccess;

    this->startEvent = new epicsEvent();
    this->stopEvent = new epicsEvent();

    this->cameraID = NULL;
    this->connect(this->pasynUserSelf);

    // Set default values
    status |= setIntegerParam(NDColorMode, NDColorModeMono);
    status |= setIntegerParam(NDDataType, NDUInt16);

    // Create the thread that performs the image capturing
    status = (epicsThreadCreate("QHYDriverCaptureTask", epicsThreadPriorityHigh,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)QHYDriverCaptureTaskC,
                                this) == NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s epicsThreadCreate failure for QHYDriverCaptureTask.\n",
                  driverName, __func__);
        return;
    }

    // Create the thread that periodically reads the temperature, etc.
    status = (epicsThreadCreate(
                  "QHYDriverPollingTask", epicsThreadPriorityMedium,
                  epicsThreadGetStackSize(epicsThreadStackMedium),
                  (EPICSTHREADFUNC)QHYDriverPollingTaskC, this) == NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s epicsThreadCreate failure for QHYDriverPollingTask.\n",
                  driverName, __func__);
        return;
    }

    printf("\n\n\n\n\n");
    return;
}

QHYDriver::~QHYDriver() { disconnect(this->pasynUserSelf); }

asynStatus QHYDriver::connect(asynUser *pasynUser) {
    disconnectCamera();
    return connectCamera();
}

asynStatus QHYDriver::disconnect(asynUser *pasynUser) {
    return this->disconnectCamera();
}

asynStatus QHYDriver::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    int status = asynSuccess;
    int reverseX, reverseY;

    int acquiring;
    getIntegerParam(ADAcquire, &acquiring);

    if (function == ADAcquire) {
        if (value == 1 && !acquiring) {
            startEvent->signal();
        }

        if (value == 0 && acquiring) {
            stopEvent->signal();
        }
    }

    // if ((function == ADBinX) || (function == ADBinY)) {
    //     // Keep BinX and BinY in sync, and ensure that they are valid values
    //     for (int i = 0; i < 16; i++) {
    //         if (cameraInfo.SupportedBins[i] == 0)
    //             break;
    //         if (cameraInfo.SupportedBins[i] == value) {
    //             status |= setIntegerParam(ADBinX, value);
    //             status |= setIntegerParam(ADBinY, value);
    //             status |= callParamCallbacks();
    //             return (asynStatus)status;
    //         }
    //     }

    //     return asynError;
    // }

    if (function == NDDataType) {
        if ((value != NDUInt8) && (value != NDUInt16)) {
            return asynError;
        }
    }

    status |= ADDriver::writeInt32(pasynUser, value);
    return (asynStatus)status;
}

asynStatus QHYDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
    int function = pasynUser->reason;
    int status = asynSuccess;

    if (function == ADAcquireTime) {
        double exposureTime = value * 1000 * 1000;
        status |=
            SetQHYCCDParam(cameraID, CONTROL_EXPOSURE, exposureTime);
    } else if (function == ADGain) {
        status |=
            SetQHYCCDParam(cameraID, CONTROL_GAIN, (long)value);
    } else if (function == ADOffset) {
        status |=
            SetQHYCCDParam(cameraID, CONTROL_OFFSET, (long)value);
    } else if (function == ADTemperature) {
        status |= ControlQHYCCDTemp(cameraID, value);
    }

    status |= ADDriver::writeFloat64(pasynUser, value);
    return (asynStatus)status;
}

asynStatus QHYDriver::connectCamera() {
    bool camFound = false;
    int status;

    unsigned char sVersion[80];
    SDKVersion(sVersion);
    const char* versionStr = reinterpret_cast<const char*>(sVersion);

    // init SDK
    unsigned int retVal = InitQHYCCDResource();
    if (QHYCCD_SUCCESS == retVal) {
        printf("SDK resources initialized.\n");
    }
    else {
        printf("Cannot initialize SDK resources, error: %d\n", retVal);
        return asynError;
    }

    // scan cameras
    int camCount = ScanQHYCCD();
    if (camCount > 0) {
        printf("Number of QHYCCD cameras found: %d \n", camCount);
    }
    else {
        printf("No QHYCCD camera found, please check USB or power.\n");
        return asynError;
    }

    for (int i = 0; i < camCount; i++) {
        retVal = GetQHYCCDId(i, camId);
        if (QHYCCD_SUCCESS == retVal) {
            printf("Application connected to the following camera from the list: Index: %d,  cameraID = %s\n", (i + 1), camId);
            camFound = true;
            break;
        }
    }

    if (!camFound) {
        printf("The detected camera is not QHYCCD or other error.\n");
        // release sdk resources
        retVal = ReleaseQHYCCDResource();
        if (QHYCCD_SUCCESS == retVal) {
            printf("SDK resources released.\n");
        }
        else {
            printf("Cannot release SDK resources, error %d.\n", retVal);
        }
        return asynError;
    }

    // open camera
    cameraID = OpenQHYCCD(camId);
    if (cameraID != NULL) {
        printf("Open QHYCCD success.\n");
    }
    else {
        printf("Open QHYCCD failure.\n");
        return asynError;
    }

    unsigned char FWInfo[128];
    FirmwareVersion(cameraID,FWInfo);

    // check camera support single frame
    retVal = IsQHYCCDControlAvailable(cameraID, CAM_SINGLEFRAMEMODE);
    if (QHYCCD_ERROR == retVal) {
        printf("The detected camera is not support single frame.\n");
        // release sdk resources
        retVal = ReleaseQHYCCDResource();
        if (QHYCCD_SUCCESS == retVal) {
            printf("SDK resources released.\n");
        }
        else {
            printf("Cannot release SDK resources, error %d.\n", retVal);
        }
        return asynError;
    }

    // set single frame mode
    int mode = 0;
    retVal = SetQHYCCDStreamMode(cameraID, mode);
    if (QHYCCD_SUCCESS == retVal) {
        printf("SetQHYCCDStreamMode set to: %d, success.\n", mode);
    }
    else {
        printf("SetQHYCCDStreamMode: %d failure, error: %d\n", mode, retVal);
        return asynError;
    }

    // initialize camera
    retVal = InitQHYCCD(cameraID);
    if (QHYCCD_SUCCESS == retVal) {
        printf("InitQHYCCD success.\n");
    }
    else {
        printf("InitQHYCCD faililure, error: %d\n", retVal);
        return asynError;
    }

    // get overscan area
    retVal = GetQHYCCDOverScanArea(cameraID, &overscanStartX, &overscanStartY, &overscanSizeX, &overscanSizeY);
    if (QHYCCD_SUCCESS == retVal) {
        printf("GetQHYCCDOverScanArea:\n");
        printf("Overscan Area startX x startY : %d x %d\n", overscanStartX, overscanStartY);
        printf("Overscan Area sizeX  x sizeY  : %d x %d\n", overscanSizeX, overscanSizeY);
    }
    else {
        printf("GetQHYCCDOverScanArea failure, error: %d\n", retVal);
        return asynError;
    }

    // get chip info
    retVal = GetQHYCCDChipInfo(cameraID, &chipWidthMM, &chipHeightMM, &maxImageSizeX, &maxImageSizeY, &pixelWidthUM, &pixelHeightUM, (uint32_t*)&bpp);
    if (QHYCCD_SUCCESS == retVal) {
        printf("GetQHYCCDChipInfo:\n");
        printf("Chip  size width x height     : %.3f x %.3f [mm]\n", chipWidthMM, chipHeightMM);
        printf("Pixel size width x height     : %.3f x %.3f [um]\n", pixelWidthUM, pixelHeightUM);
        printf("Image size width x height     : %d x %d\n", maxImageSizeX, maxImageSizeY);
    }
    else {
        printf("GetQHYCCDChipInfo failure, error: %d\n", retVal);
        return asynError;
    }

    // check color camera
    retVal = IsQHYCCDControlAvailable(cameraID, CAM_COLOR);
    if (retVal == BAYER_GB || retVal == BAYER_GR || retVal == BAYER_BG || retVal == BAYER_RG) {
        printf("This is a color camera.\n");
        printf("even this is a color camera, in Single Frame mode THE SDK ONLY SUPPORT RAW OUTPUT.So please do not set SetQHYCCDDebayerOnOff() to true;");
        //SetQHYCCDDebayerOnOff(cameraID, true);
        //SetQHYCCDParam(cameraID, CONTROL_WBR, 20);
        //SetQHYCCDParam(cameraID, CONTROL_WBG, 20);
        //SetQHYCCDParam(cameraID, CONTROL_WBB, 20);
    }
    else {
        printf("This is a mono camera.\n");
    }

    // check bin mode

    retVal = IsQHYCCDControlAvailable(cameraID, CAM_BIN1X1MODE);
    if (retVal == QHYCCD_SUCCESS)
        printf("1X1 binning mode available\n");
    else
        printf("1X1 binning mode not supported\n");
    retVal = IsQHYCCDControlAvailable(cameraID, CAM_BIN2X2MODE);
    if (retVal == QHYCCD_SUCCESS)
        printf("2X2 binning mode available\n");
    else
        printf("2X2 binning mode not supported\n");
    retVal = IsQHYCCDControlAvailable(cameraID, CAM_BIN3X3MODE);
    if (retVal == QHYCCD_SUCCESS)
        printf("3x3 binning mode available\n");
    else
        printf("3X3 binning mode not supported\n");
    retVal = IsQHYCCDControlAvailable(cameraID, CAM_BIN4X4MODE);
    if (retVal == QHYCCD_SUCCESS)
        printf("4X4 binning mode available\n");
    else
        printf("4X4 binning mode not supported\n");


    // check param min/max/step value for parameters we are interested to control
    double min,max,step;
    std::vector<CONTROL_ID> params = {CONTROL_GAIN, CONTROL_OFFSET, CONTROL_EXPOSURE, CONTROL_TRANSFERBIT};
    for (auto item : params) {
        retVal = IsQHYCCDControlAvailable(cameraID, item);
        if (retVal == QHYCCD_SUCCESS) {
            retVal = GetQHYCCDParamMinMaxStep(cameraID, item, &min, &max, &step);
            if (retVal == QHYCCD_SUCCESS)
                printf("min = %1f, max = %1f, step = %1f\n",min,max,step);
            else
                printf("get param min/max/step fail\n");
        }
    }

    // check traffic
    retVal = IsQHYCCDControlAvailable(cameraID, CONTROL_USBTRAFFIC);
    if (QHYCCD_SUCCESS == retVal) {
        retVal = SetQHYCCDParam(cameraID, CONTROL_USBTRAFFIC, USB_TRAFFIC);
        if (QHYCCD_SUCCESS == retVal) {
            printf("SetQHYCCDParam CONTROL_USBTRAFFIC set to: %d, success.\n", USB_TRAFFIC);
        }
        else {
            printf("SetQHYCCDParam CONTROL_USBTRAFFIC failure, error: %d\n", retVal);
            getchar();
            return asynError;
        }
    }

    // check temperature control
    retVal = IsQHYCCDControlAvailable(cameraID, CONTROL_COOLER);
    if (QHYCCD_SUCCESS == retVal) {
            printf("The camera has Auto Cooler mode available.\n");

            epicsFloat64 targetTemp;
            status |= getDoubleParam(ADTemperature, &targetTemp);
            retVal = ControlQHYCCDTemp(cameraID, targetTemp);
        }
    else {
        printf("Auto Cooler not available, error: %d\n", retVal);
    }
    // // check the current temp value
    // retVal = GetQHYCCDParam(cameraID, CONTROL_CURTEMP);
    // if (retVal != QHYCCD_ERROR){
    //     printf("GetQHYCCDParam CONTROL_CURTEMP at : %d, success.\n", retVal);
    // }
    // else {
    //     printf("GetQHYCCDParam CONTROL_CURTEMP failure, error: %d\n", retVal);
    // }
    // // check the current PWM value
    // retVal = GetQHYCCDParam(cameraID, CONTROL_CURPWM);
    // if (retVal != QHYCCD_ERROR){
    //     printf("GetQHYCCDParam CONTROL_CURPWM at : %d, success.\n", retVal);
    // }
    // else {
    //     printf("GetQHYCCDParam CONTROL_CURPWM failure, error: %d\n", retVal);
    // }

    // check humidity for sensor
    retVal = IsQHYCCDControlAvailable(cameraID, CAM_HUMIDITY);
    if (QHYCCD_SUCCESS == retVal) {
        double hd;
        retVal = GetQHYCCDHumidity(cameraID, &hd);
        if (QHYCCD_SUCCESS == retVal) {
            printf("The humidity of the camera is %f.\n", hd);
        }
    }
    else {
        printf("Humidity sensor not available.\n");
    }
/*
    // check gain
    retVal = IsQHYCCDControlAvailable(cameraID, CONTROL_GAIN);
    if (QHYCCD_SUCCESS == retVal) {
        retVal = SetQHYCCDParam(cameraID, CONTROL_GAIN, CHIP_GAIN);
        if (retVal == QHYCCD_SUCCESS) {
            printf("SetQHYCCDParam CONTROL_GAIN set to: %d, success\n", CHIP_GAIN);
        }
        else {
            printf("SetQHYCCDParam CONTROL_GAIN failure, error: %d\n", retVal);
            getchar();
            return 1;
        }
    }

    // check offset
    retVal = IsQHYCCDControlAvailable(cameraID, CONTROL_OFFSET);
    if (QHYCCD_SUCCESS == retVal) {
        retVal = SetQHYCCDParam(cameraID, CONTROL_OFFSET, CHIP_OFFSET);
        if (QHYCCD_SUCCESS == retVal)  {
            printf("SetQHYCCDParam CONTROL_GAIN set to: %d, success.\n", CHIP_OFFSET);
        }
        else {
            printf("SetQHYCCDParam CONTROL_GAIN failed.\n");
            getchar();
            return 1;
        }
    }
    */

    uint32_t numReadModes=0;

    GetQHYCCDNumberOfReadModes(cameraID, &numReadModes);
    printf("number of read modes: %d\n", numReadModes);

    char modeName[80];
    for (int i=0; i<numReadModes; i++) {
        GetQHYCCDReadModeName(cameraID, i, modeName);
        printf("Name %d: %s\n", i, modeName);
    }

    /*
    //Read the frame sizes 
    getIntegerParam(ADMinX, &roiStartX);
    getIntegerParam(ADMinY, &roiStartY);
    getIntegerParam(ADSizeX, &roiSizeX);
    getIntegerParam(ADSizeY, &roiSizeY);
    getIntegerParam(ADBinX, &camBinX);
    getIntegerParam(ADBinY, &camBinY);
    getIntegerParam(QHYBitDepthParam, &bpp);

    // set binning mode
    retVal = SetQHYCCDBinMode(cameraID, camBinX, camBinY);
    if (QHYCCD_SUCCESS == retVal) {
        printf("SetQHYCCDBinMode set to: binX: %d, binY: %d, success.\n", camBinX, camBinY);
    }
    else {
        printf("SetQHYCCDBinMode failure, error: %d\n", retVal);
        return 1;
    }
*/
    // set bit resolution
    retVal = IsQHYCCDControlAvailable(cameraID, CONTROL_TRANSFERBIT);
    if (QHYCCD_SUCCESS == retVal) {
        retVal = SetQHYCCDBitsMode(cameraID, bpp);
        if (QHYCCD_SUCCESS == retVal) {
            printf("SetQHYCCDParam CONTROL_TRANSFERBIT set to: %d, success.\n", bpp);
        }
        else {
            printf("SetQHYCCDParam CONTROL_TRANSFERBIT failure, error: %d\n", retVal);
            getchar();
            return asynError;
        }
    }
/*
    retVal = SetQHYCCDResolution(cameraID, roiStartX, roiStartY,
            roiSizeX/camBinX, roiSizeY/camBinY);
    if (retVal != QHYCCD_SUCCESS)
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "SetQHYCCDResolution error\n");
    
    // get effective area
    retVal = GetQHYCCDEffectiveArea(cameraID, &effectiveStartX, &effectiveStartY, &effectiveSizeX, &effectiveSizeY);
    if (QHYCCD_SUCCESS == retVal) {
        printf("GetQHYCCDEffectiveArea:\n");
        printf("Init Effective Area startX x startY: %d x %d\n", effectiveStartX, effectiveStartY);
        printf("Init Effective Area sizeX  x sizeY : %d x %d\n", effectiveSizeX, effectiveSizeY);
    }
    else {
        printf("GetQHYCCDEffectiveArea failure, error: %d\n", retVal);
    }
*/
        // Set some initial values for various parameters

    status |= setStringParam(ADManufacturer, "QHY");
    status |= setStringParam(ADModel, camId);
    status |= setStringParam(ADSerialNumber, "N/A");
    status |= setStringParam(ADFirmwareVersion, "N/A");
    status |= setStringParam(NDDriverVersion, driverVersion);
    status |= setStringParam(ADSDKVersion, versionStr);

    status |= setIntegerParam(ADSizeX, maxImageSizeX);
    status |= setIntegerParam(ADSizeY, maxImageSizeY);
    status |= setIntegerParam(ADMaxSizeX, maxImageSizeX);
    status |= setIntegerParam(ADMaxSizeY, maxImageSizeY);
    status |= setIntegerParam(NDArraySizeX, maxImageSizeX);
    status |= setIntegerParam(NDArraySizeY, maxImageSizeY);

    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: unable to set camera parameters on camera %d\n",
                  driverName, __func__, cameraID);
        return asynError;
    }


    return asynSuccess;
}

asynStatus QHYDriver::disconnectCamera() {
    if (cameraID == NULL)
        return asynDisconnected;

    int status = QHYCCD_SUCCESS;
    status |= CancelQHYCCDExposing(cameraID);
    status |= CloseQHYCCD(cameraID);

    cameraID = NULL;
    return (status == QHYCCD_SUCCESS) ? asynSuccess : asynError;
}


void QHYDriver::SDKVersion(unsigned char (&sVersion)[80])
{
    unsigned int  YMDS[4];
    // unsigned char sVersion[80];

    memset ((char *)sVersion,0x00,sizeof(sVersion));
    GetQHYCCDSDKVersion(&YMDS[0],&YMDS[1],&YMDS[2],&YMDS[3]);

    if ((YMDS[1] < 10)&&(YMDS[2] < 10))
    {
        sprintf((char *)sVersion,"V20%d0%d0%d_%d",YMDS[0],YMDS[1],YMDS[2],YMDS[3]	);
    }
    else if ((YMDS[1] < 10)&&(YMDS[2] > 10))
    {
        sprintf((char *)sVersion,"V20%d0%d%d_%d",YMDS[0],YMDS[1],YMDS[2],YMDS[3]	);
    }
    else if ((YMDS[1] > 10)&&(YMDS[2] < 10))
    {
        sprintf((char *)sVersion,"V20%d%d0%d_%d",YMDS[0],YMDS[1],YMDS[2],YMDS[3]	);
    }
    else
    {
        sprintf((char *)sVersion,"V20%d%d%d_%d",YMDS[0],YMDS[1],YMDS[2],YMDS[3]	);
    }

    fprintf(stderr,"QHYCCD SDK Version: %s\n", sVersion);
}

void QHYDriver::FirmwareVersion(qhyccd_handle *h, unsigned char (&FWInfo)[128])
{
    unsigned char fwv[32];
    unsigned int ret;
    memset (FWInfo,0x00,sizeof(FWInfo));
    ret = GetQHYCCDFWVersion(h,fwv);
    if(ret == QHYCCD_SUCCESS)
    {
        if((fwv[0] >> 4) <= 9)
        {

            sprintf((char *)FWInfo,"20%d_%d_%d",((fwv[0] >> 4) + 0x10),
                    (fwv[0]&~0xf0),fwv[1]);

        }
        else
        {

            sprintf((char *)FWInfo,"20%d_%d_%d",(fwv[0] >> 4),
                    (fwv[0]&~0xf0),fwv[1]);

        }
    }
    else
    {
        sprintf((char *)FWInfo,"Firmware version:Not Found!");
    }
    fprintf(stderr,"QHYCCD Firmware Version: %s\n", FWInfo);

}


void QHYDriver::captureTask() {
    unsigned int retVal;
    int status = asynSuccess;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int acquire = 0;
    int arrayCallbacks;
    epicsTimeStamp startTime, endTime;
    double acquirePeriod;

    u_int8_t exposureStatus;
    ROIFormat_t roiFormat;

    this->lock();
    while (true) {
        if (cameraID == NULL) {
            epicsThreadSleep(1);
            continue;
        }

        // If not currently acquiring, wait for semaphore signal
        if (!acquire) {
            this->unlock();
            bool signal = this->startEvent->wait(1);
            this->lock();

            if (!signal)
                continue;
            acquire = 1;
            setIntegerParam(ADNumImagesCounter, 0);
        }

        epicsTimeGetCurrent(&startTime);

        // Send parameters to camera
        // status = asynSuccess;
        // status |= setROIFormat(&roiFormat);

        // int reverseX, reverseY;
        // status |= getIntegerParam(ADReverseX, &reverseX);
        // status |= getIntegerParam(ADReverseY, &reverseY);
        // status |= setReverse(reverseX, reverseY);

        if (status != 0) {
            acquire = 0;
            setIntegerParam(ADAcquire, 0);
            setIntegerParam(ADStatus, ADStatusError);
            callParamCallbacks();
            continue;
        }

        // Wait until camera is ready to start with exposure
        this->unlock();
        while (GetQHYCCDCameraStatus(cameraID, &exposureStatus) != QHYCCD_SUCCESS &&
               GetQHYCCDExposureRemaining(cameraID) > 0) {
            epicsThreadSleep(SHORT_WAIT);
        }
        this->lock();
        
        retVal = ExpQHYCCDSingleFrame(cameraID);
        if (retVal != QHYCCD_SUCCESS) {
            // FAILED
            setIntegerParam(ADStatus, ADStatusError);
            callParamCallbacks();
            printf("ExpQHYCCDSingleFrame failure, error: %d\n", retVal);
            continue;
        }

        setIntegerParam(ADStatus, ADStatusAcquire);
        callParamCallbacks();

        // Wait until image has been acquired
        while (GetQHYCCDExposureRemaining(cameraID) > 0 &&
               !GetQHYCCDCameraStatus(cameraID, &exposureStatus)) {
            this->unlock();
            bool s = this->stopEvent->wait(SHORT_WAIT);
            this->lock();
            if (s) {
                // Abort exposure
                CancelQHYCCDExposing(cameraID);

                acquire = 0;
                setIntegerParam(ADAcquire, 0);
                getIntegerParam(ADImageMode, &imageMode);
                if (imageMode == ADImageContinuous) {
                    setIntegerParam(ADStatus, ADStatusIdle);
                } else {
                    setIntegerParam(ADStatus, ADStatusAborted);
                }
                callParamCallbacks();
                continue;
            }
        }

        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
        getDoubleParam(ADAcquirePeriod, &acquirePeriod);

        if (exposureStatus == QHYCCD_SUCCESS) {
            // Update counters
            numImagesCounter++;
            imageCounter++;
            setIntegerParam(NDArrayCounter, imageCounter);
            setIntegerParam(ADNumImagesCounter, numImagesCounter);

            // Allocate pImage and read data from camera
            NDArray *pImage;

            // if (roiFormat.imgType == ASI_IMG_RGB24) {
            //     size_t dims[3] = {(size_t)roiFormat.imgWidth,
            //                       (size_t)roiFormat.imgHeight, 3};
            //     pImage = this->pNDArrayPool->alloc(3, dims, roiFormat.dataType,
            //                                        0, NULL);
            // } else {
                size_t dims[2] = {(size_t)4212,
                                  (size_t)2850};
                pImage = this->pNDArrayPool->alloc(2, dims, NDUInt16,
                                                   0, NULL);
            // }

            pImage->uniqueId = imageCounter;
            pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
            updateTimeStamp(&pImage->epicsTS);

            GetQHYCCDSingleFrame(cameraID, (uint32_t*)4212,
                            (uint32_t*)2850, (uint32_t*)16, &channels, (unsigned char *)pImage->pData);

            setIntegerParam(NDArraySize, pImage->dataSize);

            this->getAttributes(pImage->pAttributeList);
            
            if (arrayCallbacks) {
                doCallbacksGenericPointer(pImage, NDArrayData, 0);
            }
            pImage->release();
        } else {
            // ERROR
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: Exposure failed with status %d\n", driverName,
                      __func__, exposureStatus);

            setIntegerParam(ADStatus, ADStatusError);
        }

        callParamCallbacks();

        // Check if we are done with acquisition
        getIntegerParam(ADAcquire, &acquire);
        if ((acquire == 0) || (imageMode == ADImageSingle) ||
            ((imageMode == ADImageMultiple) &&
             (numImagesCounter >= numImages))) {
            acquire = 0;
            setIntegerParam(ADAcquire, 0);
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();
        }

        if (acquire) {
            epicsTimeGetCurrent(&endTime);
            double elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);
            double delay = acquirePeriod - elapsedTime;

            if (delay > 0) {
                setIntegerParam(ADStatus, ADStatusWaiting);
                callParamCallbacks();
                this->unlock();
                bool s = this->stopEvent->wait(delay);
                this->lock();
                if (s) {
                    acquire = 0;
                    if (imageMode == ADImageContinuous) {
                        setIntegerParam(ADStatus, ADStatusIdle);
                    } else {
                        setIntegerParam(ADStatus, ADStatusAborted);
                    }
                    setIntegerParam(ADAcquire, 0);
                    callParamCallbacks();
                }
            }
        }
    }
}


void QHYDriver::pollingTask() {
    epicsFloat64 timeout = 1;

    unsigned int ccd_power;
    double ccd_temp;
    /* PAR_ERROR cam_err = CE_NO_ERROR;
       MY_LOGICAL te_status = FALSE;
       double ccd_temp_set = 0.0;
       double ccd_temp = 0.0;
       double te_power = 0.0;
       */
    double expTimeLeft;
    const char* functionName = "QHY::pollingTask";
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s Started Polling Thread.\n", functionName);

    while (true) {
        epicsThreadSleep(timeout);
        if (cameraID == NULL) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s NULL pointer.\n", functionName);
            continue;
        }

        lock();

        ccd_temp = GetQHYCCDParam(cameraID, CONTROL_CURTEMP);
        if (ccd_temp != QHYCCD_ERROR) {
            setDoubleParam(ADTemperatureActual, ccd_temp);
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Get camera temperature failure.\n");
            // unsigned int status = QHYDriver::InitCamera();
        }

        /* poll and update camera cooling power*/
        ccd_power = GetQHYCCDParam(cameraID, CONTROL_CURPWM);
        if (ccd_power != QHYCCD_ERROR) {
            setDoubleParam(QHYTEPowerParam, (ccd_power*100)/255);
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Get camera cooling power failure.\n");
        }

        expTimeLeft = GetQHYCCDExposureRemaining(cameraID);
        setDoubleParam(QHYPercentCompleteParam, 100-expTimeLeft);

        callParamCallbacks();
        unlock();
    }
}

/** Code for iocsh registration */
extern "C" int QHYDriverConfig(const char *portName, int maxBuffers,
                               size_t maxMemory, int priority, int stackSize) {
    new QHYDriver(portName, maxBuffers, maxMemory, priority, stackSize);
    return (asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg QHYDriverConfigArg0 = {"Port name", iocshArgString};
static const iocshArg QHYDriverConfigArg1 = {"maxBuffers", iocshArgInt};
static const iocshArg QHYDriverConfigArg2 = {"maxMemory", iocshArgInt};
static const iocshArg QHYDriverConfigArg3 = {"priority", iocshArgInt};
static const iocshArg QHYDriverConfigArg4 = {"stackSize", iocshArgInt};
static const iocshArg *const QHYDriverConfigArgs[] = {
    &QHYDriverConfigArg0, &QHYDriverConfigArg1, &QHYDriverConfigArg2,
    &QHYDriverConfigArg3, &QHYDriverConfigArg4,
};
static const iocshFuncDef configURLDriver = {"QHYDriverConfig", 5,
                                             QHYDriverConfigArgs};
static void configURLDriverCallFunc(const iocshArgBuf *args) {
    QHYDriverConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival,
                    args[4].ival);
}

static void QHYDriverRegister(void) {
    iocshRegister(&configURLDriver, configURLDriverCallFunc);
}

extern "C" {
epicsExportRegistrar(QHYDriverRegister);
}