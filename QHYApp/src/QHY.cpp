/**
 * Area detector driver for QHY cameras.
 * 
 * This driver was developed using a QHY600M Pro camera and version
 * 20.06.26 of the QHY SDK library on RHEL7.
 *
 * Greg Guyotte
 * March 2021
 *
 */

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsExport.h>
#include <epicsString.h>
#include <iocsh.h>
#include <drvSup.h>
#include <registryFunction.h>
#include <iostream>
#include <vector>

#include <libusb.h>

#include "qhyccd.h"

using std::vector;

//#include "lpardrv.h"
//#include "csbigcam.h"
//#include "csbigimg.h"

#include "QHY.h"
#include <cstring>

static void QHYReadoutTaskC(void *drvPvt);
static void QHYPollingTaskC(void *drvPvt);

/**
 * Constructor
 */
QHY::QHY(const char *portName, int maxBuffers, size_t maxMemory) : 
    ADDriver(portName, 1, NUM_DRIVER_PARAMS, 
            maxBuffers, maxMemory, 
            asynInt32Mask | asynInt32ArrayMask | asynDrvUserMask,
            asynInt32Mask | asynFloat64Mask, 
            ASYN_CANBLOCK | ASYN_MULTIDEVICE,
            1, 0, 0) 
{
    unsigned int retVal;
    int status = 0;
    const char *functionName = "QHY::QHY";
    m_Acquiring = 0;
    m_aborted = false;
    USB_TRAFFIC = 10;
    CHIP_GAIN = 10;
    CHIP_OFFSET = 140;
    EXPOSURE_TIME = 20000;
    camBinX = 1;
    camBinY = 1;
    pImgData = 0;

    //Create the epicsEvents for signaling the readout thread.
    m_startEvent = epicsEventMustCreate(epicsEventEmpty);
    if (!m_startEvent) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s epicsEventCreate failure for start event.\n", functionName);
        return;
    }
    m_stopEvent = epicsEventMustCreate(epicsEventEmpty);
    if (!m_stopEvent) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s epicsEventCreate failure for stop event.\n", functionName);
        return;
    }

    //Add the params to the paramLib 
    //createParam adds the parameters to all param lists automatically (using maxAddr).
    createParam(QHYFirstParamString,        asynParamInt32,    &QHYFirstParam);
    createParam(QHYReadoutModeParamString,  asynParamInt32,    &QHYReadoutModeParam);
    createParam(QHYReadModeParamString,     asynParamInt32,    &QHYReadModeParam);
    createParam(QHYOffsetParamString,       asynParamInt32,    &QHYOffsetParam);
    createParam(QHYBitDepthParamString,     asynParamInt32,    &QHYBitDepthParam);
    createParam(QHYPercentCompleteParamString, asynParamFloat64,  &QHYPercentCompleteParam);
    createParam(QHYTEPowerParamString,      asynParamFloat64,  &QHYTEPowerParam);
    createParam(QHYLastParamString,         asynParamInt32,    &QHYLastParam);
    createParam(QHYUSBTrafficParamString,         asynParamInt32,    &QHYUSBTrafficParam);

    //Connect to camera here and get library handle
    printf("%s Connecting to camera...\n", functionName);

    SDKVersion();


    /* GSG Code for single frame exposure 
    // single frame
    printf("ExpQHYCCDSingleFrame(pCam) - start...\n");
    retVal = ExpQHYCCDSingleFrame(pCam);
    printf("ExpQHYCCDSingleFrame(pCam) - end...\n");
    if (QHYCCD_ERROR != retVal) {
    printf("ExpQHYCCDSingleFrame success.\n");
    if (QHYCCD_READ_DIRECTLY != retVal) {
    sleep(1);
    }
    }
    else {
    printf("ExpQHYCCDSingleFrame failure, error: %d\n", retVal);
    return 1;
    }

    // get requested memory length
    uint32_t length = GetQHYCCDMemLength(pCam);

    if (length > 0) {
    pImgData = new unsigned char[length];
    memset(pImgData, 0, length);
    printf("Allocated memory for frame: %d [uchar].\n", length);
    }
    else {
    printf("Cannot allocate memory for frame.\n");
    return 1;
    }

    // get single frame
    retVal = GetQHYCCDSingleFrame(pCam, &roiSizeX, &roiSizeY, &bpp, &channels, pImgData);
    if (QHYCCD_SUCCESS == retVal) {
    printf("GetQHYCCDSingleFrame: %d x %d, bpp: %d, channels: %d, success.\n", roiSizeX, roiSizeY, bpp, channels);
    //process image here
    }
    else {
    printf("GetQHYCCDSingleFrame failure, error: %d\n", retVal);
    }

    delete [] pImgData;
    */

    retVal = InitCamera();
    if (retVal != 0)
        return;

    bool paramStatus = true;
    //Initialise any paramLib parameters that need passing up to device support
    paramStatus = ((setStringParam(ADManufacturer, "QHY") == asynSuccess) && paramStatus);
    paramStatus = ((setStringParam(ADModel, camId) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(ADMaxSizeX, maxImageSizeX) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(ADMaxSizeY, maxImageSizeY) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(ADSizeX, maxImageSizeX) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(ADSizeY, maxImageSizeY) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(NDArraySizeX, maxImageSizeX) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(NDArraySizeY, maxImageSizeY) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(NDArraySize, maxImageSizeX*maxImageSizeY*sizeof(epicsUInt16)) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(ADBinX, 1) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(ADBinY, 1) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(ADMinX, 0) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(ADMinY, 0) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(ADNumExposures, 1) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(ADNumImages, 1) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(ADImageMode, ADImageSingle) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(ADTriggerMode, ADTriggerInternal) == asynSuccess) && paramStatus); 
    paramStatus = ((setDoubleParam(ADAcquireTime, 1.0) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(NDDataType, NDUInt16) == asynSuccess) && paramStatus);
    paramStatus = ((setDoubleParam(ADTemperatureActual, 0.0) == asynSuccess) && paramStatus);

    paramStatus = ((setIntegerParam(QHYReadModeParam, 0) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(QHYOffsetParam, 0) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(QHYBitDepthParam, 0) == asynSuccess) && paramStatus);

    paramStatus = ((setIntegerParam(QHYReadoutModeParam, 0) == asynSuccess) && paramStatus);
    paramStatus = ((setDoubleParam(QHYPercentCompleteParam, 0.0) == asynSuccess) && paramStatus);
    paramStatus = ((setDoubleParam(QHYTEPowerParam, 0.0) == asynSuccess) && paramStatus);

    if (!paramStatus) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s Unable To Set Driver Parameters In Constructor.\n", functionName);
        return;
    }

    //Create the thread that reads the data 
    status = (epicsThreadCreate("QHYReadoutTask",
                epicsThreadPriorityHigh,
                epicsThreadGetStackSize(epicsThreadStackMedium),
                (EPICSTHREADFUNC)QHYReadoutTaskC,
                this) == NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s epicsThreadCreate failure for QHYReadoutTask.\n", functionName);
        return;
    }

    //Create the thread that periodically reads the temperature and readout progress
    status = (epicsThreadCreate("QHYPollingTask",
                epicsThreadPriorityMedium,
                epicsThreadGetStackSize(epicsThreadStackMedium),
                (EPICSTHREADFUNC)QHYPollingTaskC,
                this) == NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s epicsThreadCreate failure for QHYPollingTask.\n", functionName);
        return;
    }

    printf("%s Created OK.\n", functionName);
}

/**
 * Destructor. Should never get here.
 */
QHY::~QHY()
{
    printf("ERROR: QHY::~QHY Called.\n");

    unsigned int retVal = CancelQHYCCDExposingAndReadout(pCam);
    if (QHYCCD_SUCCESS == retVal) {
        printf("CancelQHYCCDExposingAndReadout success.\n");
    }
    else {
        printf("CancelQHYCCDExposingAndReadout failure, error: %d\n", retVal);
    }

    // close camera handle
    retVal = CloseQHYCCD(pCam);
    if (QHYCCD_SUCCESS == retVal) {
        printf("Close QHYCCD success.\n");
    }
    else {
        printf("Close QHYCCD failure, error: %d\n", retVal);
    }

    // release sdk resources
    retVal = ReleaseQHYCCDResource();
    if (QHYCCD_SUCCESS == retVal) {
        printf("SDK resources released.\n");
    }
    else {
        printf("Cannot release SDK resources, error %d.\n", retVal);
    }

    delete pImgData;
}

unsigned int QHY::InitCamera(void)
{
    // init SDK
    unsigned int retVal = InitQHYCCDResource();
    if (QHYCCD_SUCCESS == retVal) {
        printf("SDK resources initialized.\n");
    }
    else {
        printf("Cannot initialize SDK resources, error: %d\n", retVal);
        return 1;
    }

    // scan cameras
    int camCount = ScanQHYCCD();
    if (camCount > 0) {
        printf("Number of QHYCCD cameras found: %d \n", camCount);
    }
    else {
        printf("No QHYCCD camera found, please check USB or power.\n");
        return 1;
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
        return 1;
    }

    // open camera
    pCam = OpenQHYCCD(camId);
    if (pCam != NULL) {
        printf("Open QHYCCD success.\n");
    }
    else {
        printf("Open QHYCCD failure.\n");
        return 1;
    }

    FirmwareVersion(pCam);

    // check camera support single frame
    retVal = IsQHYCCDControlAvailable(pCam, CAM_SINGLEFRAMEMODE);
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
        return 1;
    }

    // set single frame mode
    int mode = 0;
    retVal = SetQHYCCDStreamMode(pCam, mode);
    if (QHYCCD_SUCCESS == retVal) {
        printf("SetQHYCCDStreamMode set to: %d, success.\n", mode);
    }
    else {
        printf("SetQHYCCDStreamMode: %d failure, error: %d\n", mode, retVal);
        return 1;
    }

    // initialize camera
    retVal = InitQHYCCD(pCam);
    if (QHYCCD_SUCCESS == retVal) {
        printf("InitQHYCCD success.\n");
    }
    else {
        printf("InitQHYCCD faililure, error: %d\n", retVal);
        return 1;
    }

    // get overscan area
    retVal = GetQHYCCDOverScanArea(pCam, &overscanStartX, &overscanStartY, &overscanSizeX, &overscanSizeY);
    if (QHYCCD_SUCCESS == retVal) {
        printf("GetQHYCCDOverScanArea:\n");
        printf("Overscan Area startX x startY : %d x %d\n", overscanStartX, overscanStartY);
        printf("Overscan Area sizeX  x sizeY  : %d x %d\n", overscanSizeX, overscanSizeY);
    }
    else {
        printf("GetQHYCCDOverScanArea failure, error: %d\n", retVal);
        return 1;
    }

    // get chip info
    retVal = GetQHYCCDChipInfo(pCam, &chipWidthMM, &chipHeightMM, &maxImageSizeX, &maxImageSizeY, &pixelWidthUM, &pixelHeightUM, (uint32_t*)&bpp);
    if (QHYCCD_SUCCESS == retVal) {
        printf("GetQHYCCDChipInfo:\n");
        printf("Chip  size width x height     : %.3f x %.3f [mm]\n", chipWidthMM, chipHeightMM);
        printf("Pixel size width x height     : %.3f x %.3f [um]\n", pixelWidthUM, pixelHeightUM);
        printf("Image size width x height     : %d x %d\n", maxImageSizeX, maxImageSizeY);
    }
    else {
        printf("GetQHYCCDChipInfo failure, error: %d\n", retVal);
        return 1;
    }

    // check color camera
    retVal = IsQHYCCDControlAvailable(pCam, CAM_COLOR);
    if (retVal == BAYER_GB || retVal == BAYER_GR || retVal == BAYER_BG || retVal == BAYER_RG) {
        printf("This is a color camera.\n");
        printf("even this is a color camera, in Single Frame mode THE SDK ONLY SUPPORT RAW OUTPUT.So please do not set SetQHYCCDDebayerOnOff() to true;");
        //SetQHYCCDDebayerOnOff(pCam, true);
        //SetQHYCCDParam(pCam, CONTROL_WBR, 20);
        //SetQHYCCDParam(pCam, CONTROL_WBG, 20);
        //SetQHYCCDParam(pCam, CONTROL_WBB, 20);
    }
    else {
        printf("This is a mono camera.\n");
    }

    // check bin mode

    retVal = IsQHYCCDControlAvailable(pCam, CAM_BIN1X1MODE);
    if (retVal == QHYCCD_SUCCESS)
        printf("1X1 binning mode available\n");
    else
        printf("1X1 binning mode not supported\n");
    retVal = IsQHYCCDControlAvailable(pCam, CAM_BIN2X2MODE);
    if (retVal == QHYCCD_SUCCESS)
        printf("2X2 binning mode available\n");
    else
        printf("2X2 binning mode not supported\n");
    retVal = IsQHYCCDControlAvailable(pCam, CAM_BIN3X3MODE);
    if (retVal == QHYCCD_SUCCESS)
        printf("3x3 binning mode available\n");
    else
        printf("3X3 binning mode not supported\n");
    retVal = IsQHYCCDControlAvailable(pCam, CAM_BIN4X4MODE);
    if (retVal == QHYCCD_SUCCESS)
        printf("4X4 binning mode available\n");
    else
        printf("4X4 binning mode not supported\n");


    // check param min/max/step value for parameters we are interested to control
    double min,max,step;
    vector<CONTROL_ID> params = {CONTROL_GAIN, CONTROL_OFFSET, CONTROL_EXPOSURE, CONTROL_TRANSFERBIT};
    for (auto item : params) {
        retVal = IsQHYCCDControlAvailable(pCam, item);
        if (retVal == QHYCCD_SUCCESS) {
            retVal = GetQHYCCDParamMinMaxStep(pCam, item, &min, &max, &step);
            if (retVal == QHYCCD_SUCCESS)
                printf("min = %1f, max = %1f, step = %1f\n",min,max,step);
            else
                printf("get param min/max/step fail\n");
        }
    }

    // check traffic
    retVal = IsQHYCCDControlAvailable(pCam, CONTROL_USBTRAFFIC);
    if (QHYCCD_SUCCESS == retVal) {
        retVal = SetQHYCCDParam(pCam, CONTROL_USBTRAFFIC, USB_TRAFFIC);
        if (QHYCCD_SUCCESS == retVal) {
            printf("SetQHYCCDParam CONTROL_USBTRAFFIC set to: %d, success.\n", USB_TRAFFIC);
        }
        else {
            printf("SetQHYCCDParam CONTROL_USBTRAFFIC failure, error: %d\n", retVal);
            getchar();
            return 1;
        }
    }

    // check temperature control
    retVal = IsQHYCCDControlAvailable(pCam, CONTROL_COOLER);
    if (QHYCCD_SUCCESS == retVal) {
            printf("The camera has Auto Cooler mode available.\n");
        }
    else {
        printf("Auto Cooler not available, error: %d\n", retVal);
    }
    // check the current temp value
    retVal = GetQHYCCDParam(pCam, CONTROL_CURTEMP);
    if (retVal != QHYCCD_ERROR){
        printf("GetQHYCCDParam CONTROL_CURTEMP at : %d, success.\n", retVal);
    }
    else {
        printf("GetQHYCCDParam CONTROL_CURTEMP failure, error: %d\n", retVal);
    }
    // check the current PWM value
    retVal = GetQHYCCDParam(pCam, CONTROL_CURPWM);
    if (retVal != QHYCCD_ERROR){
        printf("GetQHYCCDParam CONTROL_CURPWM at : %d, success.\n", retVal);
    }
    else {
        printf("GetQHYCCDParam CONTROL_CURPWM failure, error: %d\n", retVal);
    }

    // check humidity for sensor
    retVal = IsQHYCCDControlAvailable(pCam, CAM_HUMIDITY);
    if (QHYCCD_SUCCESS == retVal) {
        double hd;
        retVal = GetQHYCCDHumidity(pCam, &hd);
        if (QHYCCD_SUCCESS == retVal) {
            printf("The humidity of the camera is %f.\n", hd);
        }
    }
    else {
        printf("Humidity sensor not available.");
    }
/*
    // check gain
    retVal = IsQHYCCDControlAvailable(pCam, CONTROL_GAIN);
    if (QHYCCD_SUCCESS == retVal) {
        retVal = SetQHYCCDParam(pCam, CONTROL_GAIN, CHIP_GAIN);
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
    retVal = IsQHYCCDControlAvailable(pCam, CONTROL_OFFSET);
    if (QHYCCD_SUCCESS == retVal) {
        retVal = SetQHYCCDParam(pCam, CONTROL_OFFSET, CHIP_OFFSET);
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
/*
    // set exposure time
    retVal = SetQHYCCDParam(pCam, CONTROL_EXPOSURE, EXPOSURE_TIME);
    printf("SetQHYCCDParam CONTROL_EXPOSURE set to: %d, success.\n", EXPOSURE_TIME);
    if (QHYCCD_SUCCESS == retVal)
    {}
    else {
        printf("SetQHYCCDParam CONTROL_EXPOSURE failure, error: %d\n", retVal);
        getchar();
        return 1;
    }
*/
    uint32_t numReadModes=0;

    GetQHYCCDNumberOfReadModes(pCam, &numReadModes);
    printf("number of read modes: %d\n", numReadModes);

    char modeName[80];
    for (int i=0; i<numReadModes; i++) {
        GetQHYCCDReadModeName(pCam, i, modeName);
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
    retVal = SetQHYCCDBinMode(pCam, camBinX, camBinY);
    if (QHYCCD_SUCCESS == retVal) {
        printf("SetQHYCCDBinMode set to: binX: %d, binY: %d, success.\n", camBinX, camBinY);
    }
    else {
        printf("SetQHYCCDBinMode failure, error: %d\n", retVal);
        return 1;
    }
*/
    // set bit resolution
    retVal = IsQHYCCDControlAvailable(pCam, CONTROL_TRANSFERBIT);
    if (QHYCCD_SUCCESS == retVal) {
        retVal = SetQHYCCDBitsMode(pCam, bpp);
        if (QHYCCD_SUCCESS == retVal) {
            printf("SetQHYCCDParam CONTROL_TRANSFERBIT set to: %d, success.\n", bpp);
        }
        else {
            printf("SetQHYCCDParam CONTROL_TRANSFERBIT failure, error: %d\n", retVal);
            getchar();
            return 1;
        }
    }
/*
    retVal = SetQHYCCDResolution(pCam, roiStartX, roiStartY,
            roiSizeX/camBinX, roiSizeY/camBinY);
    if (retVal != QHYCCD_SUCCESS)
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "SetQHYCCDResolution error\n");
    
    // get effective area
    retVal = GetQHYCCDEffectiveArea(pCam, &effectiveStartX, &effectiveStartY, &effectiveSizeX, &effectiveSizeY);
    if (QHYCCD_SUCCESS == retVal) {
        printf("GetQHYCCDEffectiveArea:\n");
        printf("Init Effective Area startX x startY: %d x %d\n", effectiveStartX, effectiveStartY);
        printf("Init Effective Area sizeX  x sizeY : %d x %d\n", effectiveSizeX, effectiveSizeY);
    }
    else {
        printf("GetQHYCCDEffectiveArea failure, error: %d\n", retVal);
    }
*/
    return 0;
}

void QHY::SDKVersion()
{
    unsigned int  YMDS[4];
    unsigned char sVersion[80];

    memset ((char *)sVersion,0x00,sizeof(sVersion));
    GetQHYCCDSDKVersion(&YMDS[0],&YMDS[1],&YMDS[2],&YMDS[3]);

    if ((YMDS[1] < 10)&&(YMDS[2] < 10))
    {
        sprintf((char *)sVersion,"V20%d0%d0%d_%d\n",YMDS[0],YMDS[1],YMDS[2],YMDS[3]	);
    }
    else if ((YMDS[1] < 10)&&(YMDS[2] > 10))
    {
        sprintf((char *)sVersion,"V20%d0%d%d_%d\n",YMDS[0],YMDS[1],YMDS[2],YMDS[3]	);
    }
    else if ((YMDS[1] > 10)&&(YMDS[2] < 10))
    {
        sprintf((char *)sVersion,"V20%d%d0%d_%d\n",YMDS[0],YMDS[1],YMDS[2],YMDS[3]	);
    }
    else
    {
        sprintf((char *)sVersion,"V20%d%d%d_%d\n",YMDS[0],YMDS[1],YMDS[2],YMDS[3]	);
    }

    fprintf(stderr,"QHYCCD SDK Version: %s\n", sVersion);
}

void QHY::FirmwareVersion(qhyccd_handle *h)
{
    unsigned char fwv[32],FWInfo[256];
    unsigned int ret;
    memset (FWInfo,0x00,sizeof(FWInfo));
    ret = GetQHYCCDFWVersion(h,fwv);
    if(ret == QHYCCD_SUCCESS)
    {
        if((fwv[0] >> 4) <= 9)
        {

            sprintf((char *)FWInfo,"Firmware version:20%d_%d_%d\n",((fwv[0] >> 4) + 0x10),
                    (fwv[0]&~0xf0),fwv[1]);

        }
        else
        {

            sprintf((char *)FWInfo,"Firmware version:20%d_%d_%d\n",(fwv[0] >> 4),
                    (fwv[0]&~0xf0),fwv[1]);

        }
    }
    else
    {
        sprintf((char *)FWInfo,"Firmware version:Not Found!\n");
    }
    fprintf(stderr,"%s\n", FWInfo);

}

/**
 *
 */
void QHY::report(FILE *fp, int details)
{

    fprintf(fp, "QHY Detector Port: %s\n", this->portName);

    if (details > 0) {
        int ival = 0;

        getIntegerParam(ADSizeX, &ival);
        fprintf(fp, "  SizeX: %d\n", ival);
        getIntegerParam(ADSizeY, &ival);
        fprintf(fp, "  SizeY: %d\n", ival);
        getIntegerParam(ADBinX, &ival);
        fprintf(fp, "  BinX: %d\n", ival);
        getIntegerParam(ADBinY, &ival);
        fprintf(fp, "  BinY: %d\n", ival);
        getIntegerParam(ADMaxSizeX, &ival);
        fprintf(fp, "  Max SizeX: %d\n", ival);
        getIntegerParam(ADMaxSizeY, &ival);
        fprintf(fp, "  Max SizeY: %d\n", ival);
        getIntegerParam(NDDataType, &ival);
        fprintf(fp, "  NDArray Data Type: %d\n", ival);
        getIntegerParam(NDArraySize, &ival);
        fprintf(fp, "  NDArray Size: %d\n", ival);

    }
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}


/**
 * writeInt32. Write asyn integer values.
 */
asynStatus QHY::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
    int addr = 0;
    int adStatus = 0;
    int minX = 0;
    int minY = 0;
    int sizeX = 0;
    int sizeY = 0;
    int binning = 0;
    unsigned int cam_err = QHYCCD_SUCCESS;
    const char *functionName = "QHY::writeInt32";

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s Entry.\n", functionName);

    //Read the frame sizes 
    getIntegerParam(ADMinX, &minX);
    getIntegerParam(ADMinY, &minY);
    getIntegerParam(ADSizeX, &sizeX);
    getIntegerParam(ADSizeY, &sizeY);
    getIntegerParam(ADBinX, &binning);

    getIntegerParam(ADStatus, &adStatus);

    if (function == ADAcquire) {
        if ((value==1) && ((adStatus == ADStatusIdle) || (adStatus == ADStatusError) || (adStatus == ADStatusAborted))) {
            m_Acquiring = 1;
            m_aborted = false;
            setIntegerParam(ADStatus, ADStatusAcquire);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s Start Event.\n", functionName);
            epicsEventSignal(this->m_startEvent);
        }
        if ((value==0) && ((adStatus != ADStatusIdle) && (adStatus != ADStatusError) && (adStatus != ADStatusAborted))) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s Abort Exposure.\n", functionName);
            abortExposure();
        }
    } else if (function == QHYReadoutModeParam) {
        if ((value!=1) && (value!=2) && (value!=3) && (value!=4))
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Bin mode not supported\n");
        else {
            cam_err = SetQHYCCDBinMode(pCam, value, value);

            if (cam_err == QHYCCD_SUCCESS) {
                setIntegerParam(ADBinX, value);
                setIntegerParam(ADBinY, value);
            }
            /*
                setIntegerParam(ADSizeX, sizeX/value);
                setIntegerParam(ADSizeY, sizeY/value);
                cam_err = SetQHYCCDResolution(pCam, 0, 0, sizeX/value, sizeY/value);
                if (cam_err != QHYCCD_SUCCESS)
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "SetQHYCCDResolution error\n");
                // get effective area
                cam_err = GetQHYCCDEffectiveArea(pCam, &effectiveStartX, &effectiveStartY, &effectiveSizeX, &effectiveSizeY);
                if (QHYCCD_SUCCESS == cam_err) {
                    printf("GetQHYCCDEffectiveArea:\n");
                    printf("Effective Area startX x startY: %d x %d\n", effectiveStartX, effectiveStartY);
                    printf("Effective Area sizeX  x sizeY : %d x %d\n", effectiveSizeX, effectiveSizeY);
                }
                else {
                    printf("GetQHYCCDEffectiveArea failure, error: %d\n", cam_err);
                }
            }*/
        }
        printf("ReadoutMode set to %d\n", value);
    } else if (function == QHYReadModeParam) {
        if ((value!=0) && (value!=1) && (value!=2) && (value!=3) &&(value!=4))
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Read mode not supported\n");
        else {
            cam_err = SetQHYCCDReadMode(pCam, value);
            if (cam_err != QHYCCD_SUCCESS)
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "SetQHYCCDReadMode error\n");
        }
        printf("Read mode set to %d\n", value);
    } else if (function == QHYBitDepthParam) {
        if ((value != 8) && (value != 16) && (value != 32))
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Bit depth not supported\n");
        else {
            cam_err = SetQHYCCDParam(pCam, CONTROL_TRANSFERBIT, value);
            if (cam_err != QHYCCD_SUCCESS)
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "SetQHYCCDParam (bit depth error)\n");
        }
        if (value == 8)
            setIntegerParam(NDDataType, NDUInt8); 
        if (value == 16)
            setIntegerParam(NDDataType, NDUInt16); 
        if (value == 32)
            setIntegerParam(NDDataType, NDUInt32); 
        printf("Bit depth set to %d\n", value);
    } else if (function == QHYOffsetParam) {
        if ((value < 0) || (value > 255))
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Offset range exceeded (0-255)\n");
        else {
            cam_err = SetQHYCCDParam(pCam, CONTROL_OFFSET, value);
            if (cam_err != QHYCCD_SUCCESS)
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "SetQHYCCDParam (offset error)\n");
        }
        printf("Offset set to %d\n", value);
    } else if (function == QHYUSBTrafficParam) {
        printf("Setting USB_TRAFFIC to %d\n", value);
        int retVal = SetQHYCCDParam(pCam, CONTROL_USBTRAFFIC, value);
        if (retVal == QHYCCD_SUCCESS) printf("Successfully set USB_TRAFFIC\n");
	else printf("Error setting USB_TRAFFIC\n");
    } 
/*else if (function == ADMinX) {
        if (value > (((int)effectiveSizeX/binning) - 1)) {
            value = ((int)effectiveSizeX/binning) - 1;
        }
        if ((value + sizeX) > ((int)effectiveSizeX/binning)) {
            sizeX = (int)effectiveSizeX/binning - value;
            setIntegerParam(ADSizeX, sizeX);
        }
        printf("MinX set to %d\n", value);
    } else if (function == ADMinY) {

        printf("MinY value try to %d, effY = %d, binning=%d\n", value,effectiveSizeY,binning);
        if (value > (((inoeffectiveSizeY/binning) - 1)) {
            value = ((int)effectiveSizeY/binning) - 1;
        }
        if ((value + sizeY) > ((int)effectiveSizeY/binning)) {
            sizeY = (int)effectiveSizeY/binning - value;
            setIntegerParam(ADSizeY, sizeY);
        }
        printf("MinY set to %d\n", value);
    } else if (function == ADSizeX) {
        if ((minX + value) > ((int)effectiveSizeX/binning)) {
            value = (int)effectiveSizeX/binning - minX;
        }
        printf("SizeX set to %d\n", value);
    } else if (function == ADSizeY) {
        printf("MinY value try to %d, effY = %d, binning=%d\n", value,effectiveSizeY,binning);
        if ((minY + value) > ((int)effectiveSizeY/binning)) {
            value = (int)effectiveSizeY/binning - minY;
        }
        printf("SizeY set to %d\n", value);
    } */

    /*
       } else if (function == ADSBIGReadoutModeParam) {

//GSG replace with call to SetQHYCCDStreamMode()
p_Cam->SetReadoutMode(value);
if (value == 0) {
binning = 1;
} else if (value == 1) {
binning = 2;
} else if (value == 2) {
binning = 3;
}
//If we change the binning, reset the frame sizes. This forces
//the frame sizes to be set after the binning mode.
//The SubFrame sizes have the be set after we know the binning anyway.
p_Cam->SetSubFrame(0, 0, m_CamWidth/binning, m_CamHeight/binning);
setIntegerParam(ADMinX, 0);
setIntegerParam(ADMinY, 0);
setIntegerParam(ADSizeX, m_CamWidth/binning);
setIntegerParam(ADSizeY, m_CamHeight/binning);
setIntegerParam(ADBinX, binning);
setIntegerParam(ADBinY, binning);
} else if (function == ADMinX) {
if (value > ((m_CamWidth/binning) - 1)) {
value = (m_CamWidth/binning) - 1;
}
if ((value + sizeX) > (m_CamWidth/binning)) {
sizeX = m_CamWidth/binning - value;
setIntegerParam(ADSizeX, sizeX);
}
} else if (function == ADMinY) {
if (value > ((m_CamHeight/binning) - 1)) {
value = (m_CamHeight/binning) - 1;
}
if ((value + sizeY) > (m_CamHeight/binning)) {
sizeY = m_CamHeight/binning - value;
setIntegerParam(ADSizeY, sizeY);
}
} else if (function == ADSizeX) {
if ((minX + value) > (m_CamWidth/binning)) {
value = m_CamWidth/binning - minX;
}
} else if (function == ADSizeY) {
if ((minY + value) > (m_CamHeight/binning)) {
value = m_CamHeight/binning - minY;
}
}
*/


if ((status != asynSuccess) || (cam_err != QHYCCD_SUCCESS)) {
    callParamCallbacks(addr);
    return asynError;
}

status = (asynStatus) setIntegerParam(addr, function, value);
if (status!=asynSuccess) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s Error Setting Parameter. Asyn addr: %d, asynUser->reason: %d, value: %d\n", 
            functionName, addr, function, value);
    return(status);
}

//Do callbacks so higher layers see any changes 
callParamCallbacks(addr);

return status;
}


/**
 * writeFloat64. Write asyn float values.
 */
asynStatus QHY::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    unsigned int retVal;
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
    int addr = 0;
    int cam_err;
    const char *functionName = "QHY::writeFloat64";

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s Entry.\n", functionName);

    if (function == ADAcquireTime) {
        if (value > 0) {
            retVal = SetQHYCCDParam(pCam, CONTROL_EXPOSURE, value*1000*1000);
            if (retVal != QHYCCD_SUCCESS)
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Set camera exposure time failure.\n");
        }
        printf("Exposure time set to %f\n", value);
    } else if (function == ADTemperature) {
        if ((cam_err = ControlQHYCCDTemp(pCam, value)) != QHYCCD_SUCCESS) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Set camera temperature failure.\n");
            status = asynError;
        }
    } else if (function == ADGain) {
        if (value < 0)
            value = 0;
        if (value > 200)
            value = 200;
        cam_err = SetQHYCCDParam(pCam, CONTROL_GAIN, value);
        if (cam_err != QHYCCD_SUCCESS)
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "SetQHYCCDParam (gain error)\n");
        printf("Gain set to %f\n", value);
    } 

    if (status != asynSuccess) {
        callParamCallbacks(addr);
        return asynError;
    }

    status = (asynStatus) setDoubleParam(addr, function, value);
    if (status!=asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s Error Setting Parameter. Asyn addr: %d, asynUser->reason: %d, value: %f\n", 
                functionName, addr, function, value);
        return(status);
    }

    //Do callbacks so higher layers see any changes 
    callParamCallbacks(addr);

    return status;
}

/**
 * Abort the current aqusition.
 */
void QHY::abortExposure(void) 
{
    CancelQHYCCDExposingAndReadout(pCam);
    m_aborted = true;
}


/**
 * Readout thread function
 */
void QHY::readoutTask(void)
{
    epicsEventWaitStatus eventStatus;
    epicsFloat64 timeout = 0.001;
    bool error = false;
    size_t dims[2];
    int nDims = 2;
    /*
    epicsInt32 sizeX = 0;
    epicsInt32 sizeY = 0;
    epicsInt32 minX = 0;
    epicsInt32 minY = 0;
    epicsInt32 binX = 0;
    epicsInt32 binY = 0;
    */
    NDDataType_t dataType;
    epicsInt32 iDataType = 0;
    epicsUInt32 dataSize = 0;
    epicsTimeStamp nowTime;
    NDArray *pArray = NULL;
    epicsInt32 numImagesCounter = 0;
    epicsInt32 imageCounter = 0;
    unsigned int retVal;
    double expTime;

    const char* functionName = "QHY::readoutTask";
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s Started Readout Thread.\n", functionName);

    while (1) {

        //Wait for a stop event, with a short timeout, to catch any that were done after last one.
        eventStatus = epicsEventWaitWithTimeout(m_stopEvent, timeout);          
        if (eventStatus == epicsEventWaitOK) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s Got Stop Event Before Start Event.\n", functionName);
        }

        lock();
        if (!error) {
            setStringParam(ADStatusMessage, "Idle");
        }
        callParamCallbacks();
        unlock();

        eventStatus = epicsEventWait(m_startEvent);          
        if (eventStatus == epicsEventWaitOK) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s Got Start Event.\n", functionName);
            error = false;
            setStringParam(ADStatusMessage, " ");
            lock();
            setIntegerParam(ADNumImagesCounter, 0);
            setIntegerParam(ADNumExposuresCounter, 0);

            //Sanity checks
            printf("pCam = %d\n", (unsigned int *)pCam);
            if (pCam == NULL) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s NULL pointer.\n", functionName);
                break;
            }

            //Read the frame sizes 
            getIntegerParam(ADMinX, &roiStartX);
            getIntegerParam(ADMinY, &roiStartY);
            getIntegerParam(ADSizeX, &roiSizeX);
            getIntegerParam(ADSizeY, &roiSizeY);
            getIntegerParam(ADBinX, &camBinX);
            getIntegerParam(ADBinY, &camBinY);
            getIntegerParam(QHYBitDepthParam, &bpp);
            
            // set binning mode
            retVal = SetQHYCCDBinMode(pCam, camBinX, camBinY);
            if (QHYCCD_SUCCESS == retVal) {
                printf("SetQHYCCDBinMode set to: binX: %d, binY: %d, success.\n", camBinX, camBinY);
            }
            else {
                printf("SetQHYCCDBinMode failure, error: %d\n", retVal);
            }

            retVal = SetQHYCCDResolution(pCam, roiStartX, roiStartY,
                    roiSizeX/camBinX, roiSizeY/camBinY);
            if (retVal != QHYCCD_SUCCESS)
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "SetQHYCCDResolution error\n");
            // get effective area
            retVal = GetQHYCCDEffectiveArea(pCam, &effectiveStartX, &effectiveStartY, &effectiveSizeX, &effectiveSizeY);
            if (QHYCCD_SUCCESS == retVal) {
                printf("GetQHYCCDEffectiveArea:\n");
                printf("Effective Area startX x startY: %d x %d\n", effectiveStartX, effectiveStartY);
                printf("Effective Area sizeX  x sizeY : %d x %d\n", effectiveSizeX, effectiveSizeY);
            }
            else {
                printf("GetQHYCCDEffectiveArea failure, error: %d\n", retVal);
            }

            if (!error) {
                //Do exposure
                callParamCallbacks();
                unlock();
                retVal = ExpQHYCCDSingleFrame(pCam);
                lock();
                printf("after exposure started\n");
                if (retVal == QHYCCD_ERROR) {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                            "%s. ExpQHYCCDSingleFrame returned an error. \n", 
                            functionName);
                    error = true;
                    setStringParam(ADStatusMessage, "ExpQHYCCDSingleFrame error.");
                }
                
                /* While waiting for exposure to complete, put this thread to sleep.
                 * We call unlock() so the other thread can continue to process, updating
                 * temperature and exposure progress, etc.
                 */
                setStringParam(ADStatusMessage, "Waiting for exposure");
                getDoubleParam(ADAcquireTime, &expTime); 
                unlock();
                epicsThreadSleep(expTime);
                lock();

                // get requested memory length - will always be maximum size
                // according to QHY support.
                uint32_t length = GetQHYCCDMemLength(pCam);
                if (length > 0) {
                    pImgData = new unsigned char[length];
                    memset(pImgData, 0, length);
                    printf("Allocated memory for frame: %d [uchar].\n", length);
                } else {
                    error = true;
                    setStringParam(ADStatusMessage, "GetQHYCCDMemLength error.");
                }

                setStringParam(ADStatusMessage, "Image data copy");
                callParamCallbacks();
                if (!m_aborted) { 
                    // get single frame
                    retVal = GetQHYCCDSingleFrame(pCam, (uint32_t*)&roiSizeX,
                            (uint32_t*)&roiSizeY, (uint32_t*)&bpp, &channels, pImgData);
                    if (QHYCCD_SUCCESS == retVal) {
                        printf("GetQHYCCDSingleFrame: %d x %d, bpp: %d, channels: %d, success.\n", roiSizeX, roiSizeY, bpp, channels);
                        //process image here
                    } else {
                        printf("GetQHYCCDSingleFrame failure, error: %d\n", retVal);
                    }

                    //unsigned short *pData = p_Img->GetImagePointer();

                    //Update counters
                    getIntegerParam(NDArrayCounter, &imageCounter);
                    imageCounter++;
                    setIntegerParam(NDArrayCounter, imageCounter);
                    getIntegerParam(ADNumImagesCounter, &numImagesCounter);
                    numImagesCounter++;
                    setIntegerParam(ADNumImagesCounter, numImagesCounter);

                    //NDArray callbacks
                    int arrayCallbacks = 0;
                    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
                    getIntegerParam(NDDataType, &iDataType);
                    dataType = static_cast<NDDataType_t>(iDataType);
                    if (dataType == NDUInt8) {
                        dataSize = roiSizeX*roiSizeY*sizeof(epicsUInt8);
                    } else if (dataType == NDUInt16) {
                        dataSize = roiSizeX*roiSizeY*sizeof(epicsUInt16);
                    } else if (dataType == NDUInt32) {
                        dataSize = roiSizeX*roiSizeY*sizeof(epicsUInt32);
                    } else {
                        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                                "%s. ERROR: We can't handle this data type. dataType: %d\n", 
                                functionName, dataType);
                        error = true;
                        dataSize = 0;
                    }
                    setIntegerParam(NDArraySize, dataSize);
                    setIntegerParam(NDArraySizeX, roiSizeX);
                    setIntegerParam(NDArraySizeY, roiSizeY);

                    if (!error) {
                        printf("arrayCallbacks %d\n",arrayCallbacks);
                        if (arrayCallbacks) {
                            //Allocate an NDArray
                            dims[0] = roiSizeX;
                            dims[1] = roiSizeY;
                            if ((pArray = this->pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL)) == NULL) {
                                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                                        "%s. ERROR: pArray is NULL.\n", 
                                        functionName);
                            } else {
                                epicsTimeGetCurrent(&nowTime);
                                pArray->uniqueId = imageCounter;
                                pArray->timeStamp = nowTime.secPastEpoch + nowTime.nsec / 1.e9;
                                updateTimeStamp(&pArray->epicsTS);
                                //Get any attributes that have been defined for this driver
                                this->getAttributes(pArray->pAttributeList);
                                //We copy data because the SBIG class library holds onto the original buffer until the next acqusition
                                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                                        "%s: Copying data. dataSize: %d\n", functionName, dataSize);
                                memcpy(pArray->pData, pImgData, dataSize);

                                /* GSG: avoid memory leak by cleaning up allocated block each time */
                                delete [] pImgData;

                                unlock();
                                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s: Calling NDArray callback\n", functionName);
                                doCallbacksGenericPointer(pArray, NDArrayData, 0);
                                lock();
                                pArray->release();
                            }
                        }

                        setIntegerParam(ADStatus, ADStatusIdle);

                    } else {
                        setIntegerParam(ADStatus, ADStatusError);
                    }

                } else { //end if (!m_aborted)
                    setIntegerParam(ADStatus, ADStatusAborted);
                    m_aborted = false;
                }
            }

            callParamCallbacks();
            //Complete Acquire callback
            setIntegerParam(ADAcquire, 0);
            callParamCallbacks();
            unlock();

            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                    "%s Completed acqusition.\n", functionName);

        } //end of start event

    } //end of while(1)

    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s: ERROR: Exiting QHYReadoutTask main loop.\n", functionName);

}

/**
 * polling thread function
 */
void QHY::pollingTask(void)
{
    epicsFloat64 timeout = 1.0;
    unsigned int ccd_temp, ccd_power;
    /* PAR_ERROR cam_err = CE_NO_ERROR;
       MY_LOGICAL te_status = FALSE;
       double ccd_temp_set = 0.0;
       double ccd_temp = 0.0;
       double te_power = 0.0;
       */
    double expTimeLeft;
    const char* functionName = "QHY::pollingTask";
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s Started Polling Thread.\n", functionName);

    while (1) {

        epicsThreadSleep(timeout);

        //Sanity checks
        if (pCam == NULL) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s NULL pointer.\n", functionName);
            break;
        }

        lock();

        /* poll and update camera temperature */
        ccd_temp = GetQHYCCDParam(pCam, CONTROL_CURTEMP);
        if (ccd_temp != QHYCCD_ERROR) {
            setDoubleParam(ADTemperatureActual, ccd_temp);
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Get camera temperature failure.\n");
        }

        /* poll and update camera cooling power*/
        ccd_power = GetQHYCCDParam(pCam, CONTROL_CURPWM);
        if (ccd_power != QHYCCD_ERROR) {
            setDoubleParam(QHYTEPowerParam, (ccd_power*100)/255);
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Get camera cooling power failure.\n");
        }

        expTimeLeft = GetQHYCCDExposureRemaining(pCam);
        setDoubleParam(QHYPercentCompleteParam, 100-expTimeLeft);
        /*
           GRAB_STATE camState = GS_IDLE;
           double camPercentComplete = 0.0;
           int adStatus = 0;
           getIntegerParam(ADStatus, &adStatus);
           if ((adStatus == ADStatusAcquire || adStatus == ADStatusReadout)) {
           p_Cam->GetGrabState(camState, camPercentComplete);
           asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
           "%s Cam State: %d. Percent Complete: %f\n", 
           functionName, camState, (camPercentComplete*100.0));
           if ((camState == GS_DIGITIZING_DARK) || (camState == GS_DIGITIZING_LIGHT)) {
           setIntegerParam(ADStatus, ADStatusReadout);
           }
           setDoubleParam(ADSBIGPercentCompleteParam, (camPercentComplete*100.0));
           } else {
           if ((cam_err = p_Cam->QueryTemperatureStatus(te_status, ccd_temp, ccd_temp_set, te_power)) != CE_NO_ERROR) {
           asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
           "%s. CSBIGCam::QueryTemperatureStatus returned an error. %s\n", 
           functionName, p_Cam->GetErrorString(cam_err).c_str());
           } else {
           asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
           "%s Temperature Status: %d, %f, %f, %f\n", 
           functionName, te_status, ccd_temp, ccd_temp_set, te_power);
           setDoubleParam(ADTemperatureActual, ccd_temp);
           setDoubleParam(ADTemperature, ccd_temp_set);
           setIntegerParam(ADSBIGTEStatusParam, te_status);
           setDoubleParam(ADSBIGTEPowerParam, te_power*100.0);
           }
           }
           */
        callParamCallbacks();

        unlock();

    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s: ERROR: Exiting QHYPollingTask main loop.\n", functionName);

}


//Global C utility functions to tie in with EPICS
static void QHYReadoutTaskC(void *drvPvt)
{
    QHY *pPvt = (QHY *)drvPvt;

    pPvt->readoutTask();
}

static void QHYPollingTaskC(void *drvPvt)
{
    QHY *pPvt = (QHY *)drvPvt;

    pPvt->pollingTask();
}

/*************************************************************************************/
/** The following functions have C linkage, and can be called directly or from iocsh */

extern "C" {

    /**
     * Config function for IOC shell. It instantiates an instance of the driver.
     * @param portName The Asyn port name to use
     * @param maxBuffers Used by asynPortDriver (set to -1 for unlimited)
     * @param maxMemory Used by asynPortDriver (set to -1 for unlimited)
     */
    asynStatus QHYConfig(const char *portName, int maxBuffers, size_t maxMemory)
    {
        asynStatus status = asynSuccess;

        /*Instantiate class.*/
        try {
            new QHY(portName, maxBuffers, maxMemory);
        } catch (...) {
            printf("Unknown exception caught when trying to construct QHY.\n");
            status = asynError;
        }

        return(status);
    }


    /* Code for iocsh registration */

    /* QHYConfig */
    static const iocshArg QHYConfigArg0 = {"Port name", iocshArgString};
    static const iocshArg QHYConfigArg1 = {"Max Buffers", iocshArgInt};
    static const iocshArg QHYConfigArg2 = {"Max Memory", iocshArgInt};
    static const iocshArg * const QHYConfigArgs[] =  {&QHYConfigArg0,
        &QHYConfigArg1,
        &QHYConfigArg2};

    static const iocshFuncDef configQHY = {"QHYConfig", 3, QHYConfigArgs};
    static void configQHYCallFunc(const iocshArgBuf *args)
    {
        QHYConfig(args[0].sval, args[1].ival, args[2].ival);
    }

    static void QHYRegister(void)
    {
        iocshRegister(&configQHY, configQHYCallFunc);
    }

    epicsExportRegistrar(QHYRegister);

} // extern "C"
