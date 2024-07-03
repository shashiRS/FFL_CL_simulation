#include "PmdSurrogate.h"
#include "DataDict.h"
#include <geoml/CoordinateTransformer2D.h>
#include "MfSilTypes.h"

static constexpr auto PMD_PEPDEMO_CONFIG_FILE = "../../../conf/package/pmd_ls/pepdemo_config.json";
static constexpr auto PMD_CONFIG_FILE = "../../../conf/package/pmd_ls/pmd_config.json";

bool PmdSurrogate::init(const bool readFromFile) {
    PmdConfigReader pmdConfigReader;
    if (readFromFile) {
        auto isPmdConfigRead = pmdConfigReader.loadPmdAlgorithmConfig(mPmdConfig, PMD_CONFIG_FILE, PMD_PEPDEMO_CONFIG_FILE);
        if (!isPmdConfigRead) {
            pmdConfigReader.setDefaultValuesPMDConfiguration(mPmdConfig);
        }
    }
    else {
        pmdConfigReader.setDefaultValuesPMDConfiguration(mPmdConfig);
    }

    createRightCameraRoiMask(mRightCameraRoiMask);
    createLeftCameraRoiMask(mLeftCameraRoiMask);
    createFrontCameraRoiMask(mFrontCameraRoiMask);
    createRearCameraRoiMask(mRearCameraRoiMask);

    return true;
}

void createMask(LSM_GEOML::Polygon2D<4U>& polygonMask, const pmd::LineSegmentConfig& lineSegmentParam, const cml::Vec2Df& refPos_m) {
    polygonMask.clear();
    polygonMask.append(refPos_m);
    polygonMask.append(cml::Vec2Df(refPos_m[0] - lineSegmentParam.roiThresholdingHeight_m, refPos_m[1]));
    polygonMask.append(cml::Vec2Df(refPos_m[0] - lineSegmentParam.roiThresholdingHeight_m,
        refPos_m[1] - lineSegmentParam.roiThresholdingWidth_m));
    polygonMask.append(cml::Vec2Df(refPos_m[0], refPos_m[1] - lineSegmentParam.roiThresholdingWidth_m));
}

void PmdSurrogate::createRightCameraRoiMask(LSM_GEOML::Polygon2D<4U>& rightRoiMask)
{
    //CarMaker Coordinates system is not the same like is for PMD Algorithm
    //   PMD         CarMaker
    //    --->x            ^x
    //   |                 |
    //   v y          y<---
    cml::Vec2Df rightCamTopDownImageRef{ static_cast<float32_t>(mPmdConfig.rightPmdCamera.topDownViewHeight_m / 2), 0.0f };
    cml::Vec2Df roiThresholdingPos_m{ static_cast<float32_t>(mPmdConfig.rightPmdCamera.lineSegmentConfig.roiThresholdingY_m),
        static_cast<float32_t>(mPmdConfig.rightPmdCamera.lineSegmentConfig.roiThresholdingX_m) };
    cml::Vec2Df rightCamRoiRef = rightCamTopDownImageRef - roiThresholdingPos_m;
    createMask(rightRoiMask, mPmdConfig.rightPmdCamera.lineSegmentConfig, rightCamRoiRef);
}

void PmdSurrogate::createLeftCameraRoiMask(LSM_GEOML::Polygon2D<4U>& leftRoiMask)
{
    //CarMaker Coordinates system is not the same like is for PMD Algorithm
    //   PMD         CarMaker
    //    --->x            ^x
    //   |                 |
    //   v y          y<---
    cml::Vec2Df leftCamTopDownImageRef{ static_cast<float32_t>(mPmdConfig.leftPmdCamera.topDownViewHeight_m / 2), static_cast<float32_t>(mPmdConfig.leftPmdCamera.topDownViewWidth_m) };
    cml::Vec2Df roiThresholdingPos_m{ static_cast<float32_t>(mPmdConfig.leftPmdCamera.lineSegmentConfig.roiThresholdingY_m),
        static_cast<float32_t>(mPmdConfig.leftPmdCamera.lineSegmentConfig.roiThresholdingX_m) };
    cml::Vec2Df leftCamRoiRef = leftCamTopDownImageRef - roiThresholdingPos_m;
    createMask(leftRoiMask, mPmdConfig.leftPmdCamera.lineSegmentConfig, leftCamRoiRef);
}

void PmdSurrogate::createFrontCameraRoiMask(LSM_GEOML::Polygon2D<4U>& frontRoiMask)
{
    //CarMaker Coordinates system is not the same like is for PMD Algorithm
    //   PMD         CarMaker
    //    --->x            ^x
    //   |                 |
    //   v y          y<---
    cml::Vec2Df frontCamTopDownImageRef{ cml::Vec2Df(static_cast<float32_t>(mPmdConfig.frontPmdCamera.topDownViewHeight_m),
        static_cast<float32_t>(mPmdConfig.frontPmdCamera.topDownViewWidth_m / 2)) };
    cml::Vec2Df roiThresholdingPos_m{ cml::Vec2Df(static_cast<float32_t>(mPmdConfig.frontPmdCamera.lineSegmentConfig.roiThresholdingY_m),
        static_cast<float32_t>(mPmdConfig.frontPmdCamera.lineSegmentConfig.roiThresholdingX_m)) };
    cml::Vec2Df frontCamRoiRef = frontCamTopDownImageRef - roiThresholdingPos_m;
    createMask(frontRoiMask, mPmdConfig.frontPmdCamera.lineSegmentConfig, frontCamRoiRef);
}

void PmdSurrogate::createRearCameraRoiMask(LSM_GEOML::Polygon2D<4U>& rearRoiMask)
{
    //CarMaker Coordinates system is not the same like is for PMD Algorithm
    //   PMD         CarMaker
    //    --->x            ^x
    //   |                 |
    //   v y          y<---
    cml::Vec2Df rearCamTopDownImageRef{ 0.0f, static_cast<float32_t>(mPmdConfig.rearPmdCamera.topDownViewWidth_m / 2) };
    cml::Vec2Df roiThresholdingPos_m{ static_cast<float32_t>(mPmdConfig.rearPmdCamera.lineSegmentConfig.roiThresholdingY_m),
        static_cast<float32_t>(mPmdConfig.rearPmdCamera.lineSegmentConfig.roiThresholdingX_m) };
    cml::Vec2Df rearCamRoiRef = rearCamTopDownImageRef - roiThresholdingPos_m;
    createMask(rearRoiMask, mPmdConfig.rearPmdCamera.lineSegmentConfig, rearCamRoiRef);
}

void transformPolygonVerticesToCameraCoordinateSystem(const LSM_GEOML::Polygon2D<4U>& roiMask, const LSM_GEOML::Pose& cameraPose, LSM_GEOML::Polygon2D<4U>& imageMask) {
    imageMask.clear();
    for (const auto& x : roiMask)
    {
        imageMask.append(LSM_GEOML::CoordinateTransformer2D::transform(cameraPose, x));
    }
}

void PmdSurrogate::createCameraRoiImageMask(const LSM_GEOML::Pose& cameraPose, const char sensorName[]) {

    if (0 == strncmp(sensorName, "CAM_R", 5) && mPmdConfig.rightPmdCamera.cameraAvailability) {
        transformPolygonVerticesToCameraCoordinateSystem(mRightCameraRoiMask, cameraPose, mRightCameraRoiImageMask);
    }
    else if (0 == strncmp(sensorName, "CAM_L", 5) && mPmdConfig.leftPmdCamera.cameraAvailability) {
        transformPolygonVerticesToCameraCoordinateSystem(mLeftCameraRoiMask, cameraPose, mLeftCameraRoiImageMask);
    }
    else if (0 == strncmp(sensorName, "CAM_F", 5) && mPmdConfig.frontPmdCamera.cameraAvailability) {
        transformPolygonVerticesToCameraCoordinateSystem(mFrontCameraRoiMask, cameraPose, mFrontCameraRoiImageMask);
    }
    else if (0 == strncmp(sensorName, "CAM_B", 5) && mPmdConfig.rearPmdCamera.cameraAvailability) {
        transformPolygonVerticesToCameraCoordinateSystem(mRearCameraRoiMask, cameraPose, mRearCameraRoiImageMask);
    }
    else {
        //do nothing
    }
}

bool PmdSurrogate::isSegmentPointDetected(const cml::Vec2Df& detectedPointPosition, const char sensorName[]) {
    bool isDetected{ false };

    if (0 == strncmp(sensorName, "CAM_R", 5) && mPmdConfig.rightPmdCamera.cameraAvailability) {
        isDetected = mRightCameraRoiImageMask.isPointWithin(detectedPointPosition);
        return isDetected;
    }
    else if (0 == strncmp(sensorName, "CAM_L", 5) && mPmdConfig.leftPmdCamera.cameraAvailability) {
        isDetected = mLeftCameraRoiImageMask.isPointWithin(detectedPointPosition);
        return isDetected;
    }
    else if (0 == strncmp(sensorName, "CAM_F", 5) && mPmdConfig.frontPmdCamera.cameraAvailability) {
        isDetected = mFrontCameraRoiImageMask.isPointWithin(detectedPointPosition);
        return isDetected;
    }
    else if (0 == strncmp(sensorName, "CAM_B", 5) && mPmdConfig.rearPmdCamera.cameraAvailability) {
        isDetected = mRearCameraRoiImageMask.isPointWithin(detectedPointPosition);
        return isDetected;
    }
    else { return isDetected; }
}

bool PmdSurrogate::getCameraAvailability(const char sensorName[]) {
    bool isAvailable = ((0 == strncmp(sensorName, "CAM_R", 5) && mPmdConfig.rightPmdCamera.cameraAvailability) ||
        (0 == strncmp(sensorName, "CAM_L", 5) && mPmdConfig.leftPmdCamera.cameraAvailability) ||
        (0 == strncmp(sensorName, "CAM_F", 5) && mPmdConfig.frontPmdCamera.cameraAvailability) ||
        (0 == strncmp(sensorName, "CAM_B", 5) && mPmdConfig.rearPmdCamera.cameraAvailability));
    return isAvailable;
}

void PmdSurrogate::registerCarMakerDVAs() {


    tDDefault *frontCameraRoiImageMask_Qu = DDefaultCreate("AP.pmdSurrogateModel.FrontCameraRoiImageMask");
    DDefUChar(NULL, "AP.cemSurrogateConfig.FrontCameraRoiImageMask.cameraAvailability_nu", "", (uint8_t*)&mPmdConfig.frontPmdCamera.cameraAvailability, DVA_None);
    mFrontCameraRoiImageMask.setSize(mFrontCameraRoiImageMask.getMaxSize());
    for (unsigned int i = 0; i < mFrontCameraRoiImageMask.getSize(); i++) {
        DDefPrefix(frontCameraRoiImageMask_Qu, "AP.pmdSurrogateModel.FrontCameraRoiImageMask.polygonPoint_%d.x", i);
        DDefFloat(frontCameraRoiImageMask_Qu, "", "m", &mFrontCameraRoiImageMask[i].x(), DVA_None);
        DDefPrefix(frontCameraRoiImageMask_Qu, "AP.pmdSurrogateModel.FrontCameraRoiImageMask.polygonPoint_%d.y", i);
        DDefFloat(frontCameraRoiImageMask_Qu, "", "m", &mFrontCameraRoiImageMask[i].y(), DVA_None);
    }
    mFrontCameraRoiImageMask.setSize(0U);

    tDDefault *leftCameraRoiImageMask_Qu = DDefaultCreate("AP.pmdSurrogateModel.LeftCameraRoiImageMask");
    DDefUChar(NULL, "AP.cemSurrogateConfig.LeftCameraRoiImageMask.cameraAvailability_nu", "", (uint8_t*)&mPmdConfig.leftPmdCamera.cameraAvailability, DVA_None);
    mLeftCameraRoiImageMask.setSize(mLeftCameraRoiImageMask.getMaxSize());
    for (unsigned int i = 0; i < mLeftCameraRoiImageMask.getSize(); i++) {
        DDefPrefix(leftCameraRoiImageMask_Qu, "AP.pmdSurrogateModel.LeftCameraRoiImageMask.polygonPoint_%d.x", i);
        DDefFloat(leftCameraRoiImageMask_Qu, "", "m", &mLeftCameraRoiImageMask[i].x(), DVA_None);
        DDefPrefix(leftCameraRoiImageMask_Qu, "AP.pmdSurrogateModel.LeftCameraRoiImageMask.polygonPoint_%d.y", i);
        DDefFloat(leftCameraRoiImageMask_Qu, "", "m", &mLeftCameraRoiImageMask[i].y(), DVA_None);
    }
    mLeftCameraRoiImageMask.setSize(0U);

    tDDefault *rearCameraRoiImageMask_Qu = DDefaultCreate("AP.pmdSurrogateModel.RearCameraRoiImageMask");
    DDefUChar(NULL, "AP.cemSurrogateConfig.RearCameraRoiImageMask.cameraAvailability_nu", "", (uint8_t*)&mPmdConfig.rearPmdCamera.cameraAvailability, DVA_None);
    mRearCameraRoiImageMask.setSize(mRearCameraRoiImageMask.getMaxSize());
    for (unsigned int i = 0; i < mRearCameraRoiImageMask.getSize(); i++) {
        DDefPrefix(rearCameraRoiImageMask_Qu, "AP.pmdSurrogateModel.RearCameraRoiImageMask.polygonPoint_%d.x", i);
        DDefFloat(rearCameraRoiImageMask_Qu, "", "m", &mRearCameraRoiImageMask[i].x(), DVA_None);
        DDefPrefix(rearCameraRoiImageMask_Qu, "AP.pmdSurrogateModel.RearCameraRoiImageMask.polygonPoint_%d.y", i);
        DDefFloat(rearCameraRoiImageMask_Qu, "", "m", &mRearCameraRoiImageMask[i].y(), DVA_None);
    }
    mRearCameraRoiImageMask.setSize(0U);

    tDDefault *rightCameraRoiImageMask_Qu = DDefaultCreate("AP.pmdSurrogateModel.RightCameraRoiImageMask");
    DDefUChar(NULL, "AP.cemSurrogateConfig.RightCameraRoiImageMask.cameraAvailability_nu", "", (uint8_t*)&mPmdConfig.rightPmdCamera.cameraAvailability, DVA_None);
    mRightCameraRoiImageMask.setSize(mRightCameraRoiImageMask.getMaxSize());
    for (unsigned int i = 0; i < mRightCameraRoiImageMask.getSize(); i++) {
        DDefPrefix(rightCameraRoiImageMask_Qu, "AP.pmdSurrogateModel.RightCameraRoiImageMask.polygonPoint_%d.x", i);
        DDefFloat(rightCameraRoiImageMask_Qu, "", "m", &mRightCameraRoiImageMask[i].x(), DVA_None);
        DDefPrefix(rightCameraRoiImageMask_Qu, "AP.pmdSurrogateModel.RightCameraRoiImageMask.polygonPoint_%d.y", i);
        DDefFloat(rightCameraRoiImageMask_Qu, "", "m", &mRightCameraRoiImageMask[i].y(), DVA_None);
    }
    mRightCameraRoiImageMask.setSize(0U);

}