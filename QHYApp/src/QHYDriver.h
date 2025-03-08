#ifndef QHYDRIVER_H
#define QHYDRIVER_H

#include "ADDriver.h"
#include "qhyccd.h"
#include "NDArray.h"
#include "NDAttribute.h"
#include "asynDriver.h"
#include <epicsEvent.h>

#define SHORT_WAIT (0.00025)

// #define ADOffsetString "OFFSET"
#define QHYFirstParamString              "QHY_FIRST"
#define QHYReadoutModeParamString       "QHY_READOUT_MODE"
#define QHYReadModeParamString          "QHY_READ_MODE"
#define QHYBitDepthParamString          "QHY_BIT_DEPTH"
#define QHYOffsetParamString            "QHY_OFFSET"
#define QHYPercentCompleteParamString    "QHY_PERCENT_COMPLETE"
#define QHYTEPowerParamString            "QHY_TEMP_POWER"
#define QHYLastParamString               "QHY_LAST"
#define QHYUSBTrafficParamString               "QHY_USB_TRAFFIC"

typedef struct ROIFormat {
    NDColorMode_t colorMode;
    NDDataType_t dataType;

    int imgWidth, imgHeight;
    int imgBin;
    int startX, startY;
} ROIFormat_t;

typedef struct _QHY_CAMERA_INFO {
    char Name[64];
    qhyccd_handle *cameraID;

    int SupportedBins[16]; // 1 means bin1 which is supported by every camera, 2 means bin 2 etc.. 0 is the end of supported binning method
    uint32_t numReadModes;

    double chipWidthMM;
    double chipHeightMM;
    double pixelWidthUM;
    double pixelHeightUM;

    unsigned int overscanStartX;
    unsigned int overscanStartY;
    unsigned int overscanSizeX;
    unsigned int overscanSizeY;

    unsigned int effectiveStartX;
    unsigned int effectiveStartY;
    unsigned int effectiveSizeX;
    unsigned int effectiveSizeY;

    unsigned int maxImageSizeX;
    unsigned int maxImageSizeY;

    unsigned int maxGain;
    unsigned int maxOffset;

    bool IsCoolerCam;
    bool IsHumiditySensor;

    bool IsColorCam;
    BAYER_ID BayerPattern;
} QHY_CAMERA_INFO;

class QHYDriver : public ADDriver {
public:
    /**
     * \param[in] portName The name of the asyn port driver to be created.
     * \param[in] maxBuffers The maximum number of NDArray buffers that the
     *    NDArrayPool for this driver is allowed to allocate. Set this to -1 to
     *    allow an unlimited number of buffers.
     * \param[in] maxMemory The maximum amount of memory that the NDArrayPool
     * for this driver is allowed to allocate. Set this to -1 to allow an
     * unlimited amount of memory. \param[in] priority The thread priority for
     * the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
     * \param[in] stackSize The stack size for
     *    the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
     */
    QHYDriver(const char *portName, int maxBuffers, size_t maxMemory,
              int priority, int stackSize);
    ~QHYDriver();

    virtual asynStatus connect(asynUser *pasynUser);
    virtual asynStatus disconnect(asynUser *pasynUser);

    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    void SDKVersion(unsigned char (&sVersion)[80]);
    void FirmwareVersion(qhyccd_handle *h, unsigned char (&FWInfo)[128]);

    void captureTask();
    void pollingTask();

private:
    char camId[32];
    qhyccd_handle *cameraID;
    QHY_CAMERA_INFO cameraInfo;

    epicsEvent *startEvent;
    epicsEvent *stopEvent;

    int USB_TRAFFIC;
    int CHIP_GAIN;
    int CHIP_OFFSET;
    int EXPOSURE_TIME;
    int camBinX;
    int camBinY;

    epicsInt32 roiStartX;
    epicsInt32 roiStartY;
    epicsInt32 roiSizeX;
    epicsInt32 roiSizeY;

    unsigned int maxImageSizeX;
    unsigned int maxImageSizeY;
    epicsInt32 bpp;
    unsigned int channels;

    asynStatus setROIFormat(ROIFormat_t *out);
    asynStatus connectCamera();
    asynStatus disconnectCamera();
    asynStatus setReverse();
    asynStatus setReadMode(int readMode);

protected:
    // int QHYOffset;
    int QHYReadoutModeParam;
    int QHYReadModeParam;
    int QHYBitDepthParam;
    int QHYOffsetParam;
    int QHYPercentCompleteParam;
    int QHYTEPowerParam;
    int QHYLastParam;
    int QHYUSBTrafficParam;
};

#endif