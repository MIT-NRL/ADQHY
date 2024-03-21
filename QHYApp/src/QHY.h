/**
 * Area detector driver for QHYCCD cameras.
 * 
 * This driver was developed using a QHY600M Pro and version
 * 20.06.26 of their SDK on RHEL7.
 *
 * Greg Guyotte
 * March 2021
 *
 */

#ifndef QHY_H
#define QHY_H

#include <stddef.h>

#include <epicsTime.h>
#include <epicsTypes.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <cantProceed.h>

#include <asynOctetSyncIO.h>

#include "ADDriver.h"

#define QHYFirstParamString              "QHY_FIRST"
#define QHYReadoutModeParamString       "QHY_READOUT_MODE"
#define QHYReadModeParamString          "QHY_READ_MODE"
#define QHYBitDepthParamString          "QHY_BIT_DEPTH"
#define QHYOffsetParamString            "QHY_OFFSET"
#define QHYPercentCompleteParamString    "QHY_PERCENT_COMPLETE"
#define QHYTEPowerParamString            "QHY_TEMP_POWER"
#define QHYLastParamString               "QHY_LAST"
#define QHYUSBTrafficParamString               "QHY_USB_TRAFFIC"

class QHY : public ADDriver {

 public:
  QHY(const char *portName, int maxBuffers, size_t maxMemory);
  virtual ~QHY();

  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

  virtual void report(FILE *fp, int details);

  void readoutTask(void);
  void pollingTask(void);
  void SDKVersion(void);
  unsigned int InitCamera(void);
  void FirmwareVersion(qhyccd_handle *h);

 private:
  
  //Private functions go here
  void abortExposure(void);

  //Private static data members
  int USB_TRAFFIC;
  int CHIP_GAIN;
  int CHIP_OFFSET;
  int EXPOSURE_TIME;
  int camBinX;
  int camBinY;

  double chipWidthMM;
  double chipHeightMM;
  double pixelWidthUM;
  double pixelHeightUM;

  epicsInt32 roiStartX;
  epicsInt32 roiStartY;
  epicsInt32 roiSizeX;
  epicsInt32 roiSizeY;

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
  epicsInt32 bpp;
  unsigned int channels;

  //Private dynamic data members
  epicsUInt32 m_Acquiring;
  char camId[32];
  bool camFound = false;
  qhyccd_handle *pCam;
  unsigned char *pImgData = 0;
  //int m_CamWidth;
  //int m_CamHeight;
  bool m_aborted;
  
  epicsEventId m_startEvent;
  epicsEventId m_stopEvent;

  //Parameter library indices
  int QHYFirstParam;
  #define QHY_FIRST_PARAM QHYFirstParam
  int QHYReadoutModeParam;
  int QHYReadModeParam;
  int QHYBitDepthParam;
  int QHYOffsetParam;
  int QHYPercentCompleteParam;
  int QHYTEPowerParam;
  int QHYLastParam;
  int QHYUSBTrafficParam;
  #define QHY_LAST_PARAM QHYLastParam
  
};

#define NUM_DRIVER_PARAMS (&QHY_LAST_PARAM - &QHY_FIRST_PARAM + 1)

#endif //QHY_H
