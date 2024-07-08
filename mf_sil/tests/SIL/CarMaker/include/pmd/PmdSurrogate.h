#pragma once

#include <Vehicle/Sensor_FSpace.h>
#include "geoml/Polygon2D.h"
#include <geoml/LSM_Math.h>
#include "PmdConfigReader.h"
//#include "pmd_config.h" is included in PmdConfigReader.h file

class PmdSurrogate
{
    pmd::PmdConfig mPmdConfig;
    LSM_GEOML::Polygon2D<4U> mRightCameraRoiMask;
    LSM_GEOML::Polygon2D<4U> mLeftCameraRoiMask;
    LSM_GEOML::Polygon2D<4U> mFrontCameraRoiMask;
    LSM_GEOML::Polygon2D<4U> mRearCameraRoiMask;
    LSM_GEOML::Polygon2D<4U> mRightCameraRoiImageMask;
    LSM_GEOML::Polygon2D<4U> mRearCameraRoiImageMask;
    LSM_GEOML::Polygon2D<4U> mLeftCameraRoiImageMask;
    LSM_GEOML::Polygon2D<4U> mFrontCameraRoiImageMask;
    void createRightCameraRoiMask(LSM_GEOML::Polygon2D<4U>& rightRoiMask);
    void createLeftCameraRoiMask(LSM_GEOML::Polygon2D<4U>& leftRoiMask);
    void createFrontCameraRoiMask(LSM_GEOML::Polygon2D<4U>& frontRoiMask);
    void createRearCameraRoiMask(LSM_GEOML::Polygon2D<4U>& rearRoiMask);
public:
    void createCameraRoiImageMask(const LSM_GEOML::Pose& cameraPose, const char sensorName[]);
    bool isSegmentPointDetected(const cml::Vec2Df& detectedPointPosition, const char sensorName[]);
    bool getCameraAvailability(const char sensorName[]);
    bool init(const bool readFromFile = true);
    void registerCarMakerDVAs();
};
