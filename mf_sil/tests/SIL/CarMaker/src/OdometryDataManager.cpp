#include "OdometryDataManager.h"
#include "DataDict.h"
#include "MfSilTypes.h"

/// Maximal absolute value the SI's input \ref lsm_vedodo::OdoEstimation::xPosition_m can have.
/// Hard-coded here until provided by RTE.
/// @attention The hard-coded value shall correspond to VEDODO's behavior else overflow handling in
/// \ref OdometryDataManager will fail.
constexpr static float32_t MAX_POS_X_VALUE_M{ 1000.0F };

/// Maximal absolute value the SI's input \ref lsm_vedodo::OdoEstimation::yPosition_m can have.
/// Hard-coded here until provided by RTE.
/// @attention The hard-coded value shall correspond to VEDODO's behavior else overflow handling in
/// \ref OdometryDataManager will fail.
constexpr static float32_t MAX_POS_Y_VALUE_M{ 1000.0F };

void OdometryDataManager::registerDVAVariables()
{
    DDefFloat(NULL, "AP.absoluteOdoPose.x_m", "m", &mOdoCorrection.absoluteOdoPose.Pos().x(), DVA_None);
    DDefFloat(NULL, "AP.absoluteOdoPose.y_m", "m", &mOdoCorrection.absoluteOdoPose.Pos().y(), DVA_None);
    DDefFloat(NULL, "AP.absoluteOdoPose.yaw_rad", "rad", &mOdoCorrection.absoluteOdoPose.Yaw_rad(), DVA_None);
}

void OdometryDataManager::reset()
{
    mOdoCorrection = {};
}

void OdometryDataManager::update(const lsm_vedodo::OdoEstimation & odoPort)
{
    // TODO: move code from updateOdometryData(), the odoEstimationPortCM and odoGpsPort from mdl_APCtrl.cpp to here
    updateAbsoluteOdometryPose(odoPort);
}

void OdometryDataManager::updateAbsoluteOdometryPose(const lsm_vedodo::OdoEstimation & odoPort)
{
    constexpr float32_t MAX_POS_X_VALUE_HALF_M{ 0.5F * MAX_POS_X_VALUE_M };
    constexpr float32_t MAX_POS_Y_VALUE_HALF_M{ 0.5F * MAX_POS_Y_VALUE_M };
    if (fabsf(odoPort.xPosition_m - mOdoCorrection.lastOdoPos.x()) > MAX_POS_X_VALUE_HALF_M)
    {
        //reset in x direction detected
        if (mOdoCorrection.lastOdoPos.x() > MAX_POS_X_VALUE_HALF_M)
        {
            // Overflow
            mOdoCorrection.deltaPos.x() -= MAX_POS_X_VALUE_M;  //odometry was reduced by 1000m in x-direction
        }
        else if (mOdoCorrection.lastOdoPos.x() < -MAX_POS_X_VALUE_HALF_M)
        {
            // Underflow
            mOdoCorrection.deltaPos.x() += MAX_POS_X_VALUE_M;  //odometry was increased by 1000m in x-direction
        }
        else
        {
        }
    }
    if (fabsf(odoPort.yPosition_m - mOdoCorrection.lastOdoPos.y()) > MAX_POS_Y_VALUE_HALF_M)
    {
        //reset in y direction detected
        if (mOdoCorrection.lastOdoPos.y() > MAX_POS_Y_VALUE_HALF_M)
        {
            // Overflow
            mOdoCorrection.deltaPos.y() -= MAX_POS_Y_VALUE_M;  //odometry was reduced by 1000m in y-direction
        }
        else if (mOdoCorrection.lastOdoPos.y() < -MAX_POS_Y_VALUE_HALF_M)
        {
            // Underflow
            mOdoCorrection.deltaPos.y() += MAX_POS_Y_VALUE_M;  //odometry was increased by 1000m in y-direction
        }
        else
        {
        }
    }
    mOdoCorrection.lastOdoPos = { odoPort.xPosition_m, odoPort.yPosition_m };
    mOdoCorrection.absoluteOdoPose = { odoPort.xPosition_m - mOdoCorrection.deltaPos.x(),  odoPort.yPosition_m - mOdoCorrection.deltaPos.y(), odoPort.yawAngle_rad };

}
