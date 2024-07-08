#include "TestRunWrapper.h"
#include "math.h"
#include "string.h"
#include "Traffic.h"
#include "geoml/GeomlTypes.h"
#include "CemSurrogate.h"


bool trfObjIsOdo(tTrafficObj_Cfg const* _trfObj)
{
    if (strcmp(_trfObj->Name, "Odo") == 0) {
        return true;
    }

    return false;
}

bool trfObjIsDyn(tTrafficObj const * _trfObj)
{
    if (((_trfObj->Cfg.MotionKind == tMotionKind::MotionKind_2Wheel) || (_trfObj->Cfg.MotionKind == tMotionKind::MotionKind_4Wheel) ||
        (_trfObj->Cfg.MotionKind == tMotionKind::MotionKind_Pedestrian) || (_trfObj->Cfg.MotionKind == tMotionKind::MotionKind_Ball) ||
         ((fabs(float32_t(_trfObj->v_0[0U])) >= LSM_GEOML::MIN_FLT_DIVISOR) || (fabs(float32_t(_trfObj->v_0[1U])) >= LSM_GEOML::MIN_FLT_DIVISOR)))) {
        return true;
    }

    return false;
}

bool trfObjIsStatic(tTrafficObj const * _trfObj)
{
    if ((_trfObj->Cfg.h > VCEM::AP_G_MIN_HEIGHT_OBSTACLE_M) && (_trfObj->Cfg.Name[0U] == 'T') &&
        ((fabs(static_cast<float32_t>(_trfObj->v_0[0U])) <= LSM_GEOML::MIN_FLT_DIVISOR) ||
        (fabs(static_cast<float32_t>(_trfObj->v_0[1U])) <= LSM_GEOML::MIN_FLT_DIVISOR))) {
        return true;
    }

    return false;
}

bool trfObjIsParkBox(tTrafficObj_Cfg const * _trfObj)
{
    if ((_trfObj->h < VCEM::AP_G_MIN_HEIGHT_OBSTACLE_M) && (_trfObj->Name[0U] == 'P')) {
        return true;
    }

    return false;
}

bool trfObjIsODS(tTrafficObj_Cfg const * _trfObj)
{
    if (strncmp(_trfObj->Name, "ODS", 3U) == 0U) {
        return true;
    }

    return false;
}

bool trfObjIsRoundStatic(tTrafficObj_Cfg const * _trfObj)
{
    if ((strstr(_trfObj->Name, "Obs")) && (strstr(_trfObj->Info, "circle"))) {
        return true;
    }

    return false;
}

bool trfObjIsRectStatic(tTrafficObj_Cfg const * _trfObj)
{
    if ((strstr(_trfObj->Name, "Obs") && strstr(_trfObj->Info, "rect")) ||
        (strstr(_trfObj->Name, "Lim") && (strstr(_trfObj->Info, "wall")) || (strstr(_trfObj->Info, "guard")))) {
        return true;
    }

    return false;
}

bool trfObjIsCurbstone(tTrafficObj_Cfg const * _trfObj)
{
    if ((strstr(_trfObj->Name, "Lim")) && (strstr(_trfObj->Info, "curb_"))) {
        return true;
    }

    return false;
}

bool trfObjIsWhlStp(tTrafficObj_Cfg const * _trfObj)
{
    if ((strstr(_trfObj->Name, "Whs")) && (strstr(_trfObj->Info, "Wheelstopper"))) {
        return true;
    }

    return false;
}

bool trfObjIsRoadLaneMark(tTrafficObj_Cfg const * _trfObj)
{
    if ((strstr(_trfObj->Name, "Lim")) && (strstr(_trfObj->Info, "lim_Rd_P - line"))) {
        return true;
    }

    return false;
}

bool trfObjIsParkLineMark(tTrafficObj_Cfg const * _trfObj)
{
    if ((strstr(_trfObj->Name, "Lim")) && (strstr(_trfObj->Info, "- line")) &&
        (!strstr(_trfObj->Name, "e")) && (!strstr(_trfObj->Name, "r"))) { // exclude external and roadside lines for TestRuns with double lines (US_Scenarios)
        return true;
    }

    return false;
}

uint8_t calculateObjectContourPoints(
    const tTrafficObj* trafficObj,
    const TrafficContour2D &trafficContour2D_t,
    const float32_t offsetX_m,
    const float32_t offsetY_m,
    objectType objectCarMakerType_nu,
    cml::Vec2Df_POD worldPt_m[],
    const bool isSplited,
    bool& needToBeSplited,
    const uint8_t maxPointToSplit
)
{
    uint8_t numberOfPoints = 0U;
    float32_t cYaw = static_cast<float32_t>(cos(trafficObj->r_zyx[2]));
    float32_t sYaw = static_cast<float32_t>(sin(trafficObj->r_zyx[2]));
    //round the result of sin and cos (precision 6)
    cYaw = round(cYaw * 1e6f) / 1e6f;
    sYaw = round(sYaw * 1e6f) / 1e6f;
    double originCircleX_m;
    double originCircleY_m;
    std::vector<cml::Vec2Df> localPoint{ maxPointToSplit };

    //if the object are CircleObstacle or Rect4Points(calculate the points for object using the length and the width of object) don't use this calculation (localPoint
    if ((objectCarMakerType_nu != objectType::Rect4Points)
        && (objectCarMakerType_nu != objectType::CircleObstacle)) {
        if (trafficContour2D_t.isMirroringConstruction_nu == false) {
            if (trafficContour2D_t.nrRows != 0) {
                numberOfPoints = trafficContour2D_t.nrRows - 1;
            }
            else {
                numberOfPoints = 0u;
            }
            //if nrRows is 0 (not found 2d Contour in testRuns) -> calculate the points for object using the length and the width of object
            //special case -> objects with more than 16 points
            if ((trafficContour2D_t.nrRows != 0) && (numberOfPoints <= maxPointToSplit)) {
                //calculate the number of points (without the last point - is equal with the first point (0,0))
                for (uint8_t i = 0; i < numberOfPoints; i++)
                {
                    localPoint[i].x() = trafficContour2D_t.points[0][i];
                    localPoint[i].y() = trafficContour2D_t.points[1][i];
                }
            }
            else {
                numberOfPoints = 4u;
                objectCarMakerType_nu = objectType::Rect4Points;
            }
        }
        else {
            /* all carmaker points + their symmetric, without the first and the last points symmetric*/
            numberOfPoints = trafficContour2D_t.nrRows * 2 - 2;
            //if nrRows is 0 (not found 2d Contour in testRuns) -> calculate the points for object using the length and the width of object
            //*special case -> objects with more than 16 points*/
            if (trafficContour2D_t.nrRows != 0) {

                if (numberOfPoints <= maxPointToSplit)
                {
                    /* store first and last points without their symmetric */
                    localPoint[0].x() = 0;
                    localPoint[0].y() = 0;
                    localPoint[numberOfPoints / 2].x() = trafficContour2D_t.points[0][numberOfPoints / 2];
                    localPoint[numberOfPoints / 2].y() = trafficContour2D_t.points[1][numberOfPoints / 2];
                    /* store the rest of carmaker points + their symmetric*/
                    for (uint8_t i = 1; i < numberOfPoints / 2; i++)
                    {
                        localPoint[i].x() = trafficContour2D_t.points[0][i];
                        localPoint[i].y() = trafficContour2D_t.points[1][i];

                        localPoint[numberOfPoints - i].x() = trafficContour2D_t.points[0][i];
                        localPoint[numberOfPoints - i].y() = (-1) * trafficContour2D_t.points[1][i];
                    }

                }
                else if (!isSplited) {
                    //first - use only first maxPointToSplit (e.g. ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU = 16 points)
                    numberOfPoints = maxPointToSplit;
                    /* store first and last points without their symmetric */
                    localPoint[0].x() = 0.0;
                    localPoint[0].y() = 0.0;
                    //the last point (16th) is the 15th points, but with 0.0 on y axes (to have a straight line)
                    localPoint[numberOfPoints / 2].x() = trafficContour2D_t.points[0][numberOfPoints / 2 - 1];
                    localPoint[numberOfPoints / 2].y() = 0.0f;
                    /* store the rest of carmaker points + their symmetric*/
                    for (uint8_t i = 1; i < numberOfPoints / 2; i++)
                    {
                        localPoint[i].x() = trafficContour2D_t.points[0][i];
                        localPoint[i].y() = trafficContour2D_t.points[1][i];

                        localPoint[numberOfPoints - i].x() = trafficContour2D_t.points[0][i];
                        localPoint[numberOfPoints - i].y() = (-1) * trafficContour2D_t.points[1][i];
                    }
                    needToBeSplited = true;
                }
                else {

                    //aditional 4 points are added (to have the last 2 points from first build + we used maxPointToSplit - 1 points from 2d Contour)
                    numberOfPoints = numberOfPoints - maxPointToSplit + 4u;
                    /* store first and last points without their symmetric */
                    localPoint[0].x() = trafficContour2D_t.points[0][maxPointToSplit / 2 - 1];
                    localPoint[0].y() = 0.0f;
                    localPoint[numberOfPoints / 2].x() = trafficContour2D_t.points[0][maxPointToSplit / 2 + numberOfPoints / 2 - 1];
                    localPoint[numberOfPoints / 2].y() = trafficContour2D_t.points[1][maxPointToSplit / 2 + numberOfPoints / 2 - 1];
                    /* store the rest of carmaker points + their symmetric*/
                    for (uint8_t i = 0; i < numberOfPoints / 2; i++)
                    {
                        localPoint[i + 1].x() = trafficContour2D_t.points[0][maxPointToSplit / 2 + i - 1];
                        localPoint[i + 1].y() = trafficContour2D_t.points[1][maxPointToSplit / 2 + i - 1];

                        localPoint[numberOfPoints - i - 1].x() = trafficContour2D_t.points[0][maxPointToSplit / 2 + i - 1];
                        localPoint[numberOfPoints - i - 1].y() = (-1) * trafficContour2D_t.points[1][maxPointToSplit / 2 + i - 1];
                    }
                }

            }
            else {
                numberOfPoints = 4u;
                objectCarMakerType_nu = objectType::Rect4Points;
            }
        }
    }

    switch (objectCarMakerType_nu)
    {
        //calculate the 4 points of Parking Box for Right Side
    case objectType::ParkingBoxRightSide:
        /*transform front right -- CarMakerObjectOriginX + length; CarMakerObjectOriginY + width*/
        worldPt_m[0].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * localPoint[2].x() - sYaw * localPoint[2].y());
        worldPt_m[0].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * localPoint[2].x() + cYaw * localPoint[2].y());
        /*transform front left -- CarMakerObjectOriginX ; CarMakerObjectOriginY + width*/
        worldPt_m[1].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * localPoint[1].x() - sYaw * localPoint[1].y());
        worldPt_m[1].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * localPoint[1].x() + cYaw * localPoint[1].y());
        /*transform rear left -- CarMakerObjectOriginX; CarMakerObjectOriginY*/
        worldPt_m[2].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * localPoint[0].x() - sYaw * localPoint[0].y());
        worldPt_m[2].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * localPoint[0].x() + cYaw * localPoint[0].y());
        /*transform rear right -- CarMakerObjectOriginX + length; CarMakerObjectOriginY*/
        worldPt_m[3].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * localPoint[3].x() - sYaw * localPoint[3].y());
        worldPt_m[3].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * localPoint[3].x() + cYaw * localPoint[3].y());
        numberOfPoints = 4U;
        break;
        //calculate the 4 points of Parking Box for Light Side
    case objectType::ParkingBoxLeftSide:
        /*transform rear right -- CarMakerObjectOriginX; CarMakerObjectOriginY*/
        worldPt_m[0].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * localPoint[0].x() - sYaw * localPoint[0].y());
        worldPt_m[0].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * localPoint[0].x() + cYaw * localPoint[0].y());
        /*transform front right -- CarMakerObjectOriginX + length; CarMakerObjectOriginY*/
        worldPt_m[1].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * localPoint[3].x() - sYaw * localPoint[3].y());
        worldPt_m[1].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * localPoint[3].x() + cYaw * localPoint[3].y());
        /*transform front left -- CarMakerObjectOriginX + length; CarMakerObjectOriginY + width*/
        worldPt_m[2].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * localPoint[2].x() - sYaw * localPoint[2].y());
        worldPt_m[2].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * localPoint[2].x() + cYaw * localPoint[2].y());
        /*transform rear left -- CarMakerObjectOriginX; CarMakerObjectOriginY + width*/
        worldPt_m[3].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * localPoint[1].x() - sYaw * localPoint[1].y());
        worldPt_m[3].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * localPoint[1].x() + cYaw * localPoint[1].y());
        numberOfPoints = 4U;
        break;
        //calculate the 2 points for Lane Marking Object
    case objectType::LaneMarking:
        /*positions of start point of the parking space marking*/
        worldPt_m[0].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * localPoint[0].x() - sYaw * localPoint[1].y() / 2);
        worldPt_m[0].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * localPoint[0].x() + cYaw * localPoint[1].y() / 2);
        /*positions of end point of the parking space marking*/
        worldPt_m[1].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * localPoint[2].x() - sYaw * localPoint[1].y() / 2);
        worldPt_m[1].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * localPoint[2].x() + cYaw * localPoint[1].y() / 2);
        numberOfPoints = 2u;
        break;

        //calculate the point for Obstacle with Circle form
    case objectType::CircleObstacle:
        numberOfPoints = 16u;
        originCircleX_m = (trafficObj->t_0[0] + offsetX_m + trafficObj->Cfg.l / 2);
        originCircleY_m = (trafficObj->t_0[1] + offsetY_m);
        for (double j_deg = 0; j_deg < 360.0f; j_deg = j_deg + (360.0f / maxPointToSplit))
        {
            double circlePoint_X_m = trafficObj->Cfg.l / 2 * cos(j_deg * LSM_GEOML::LSM_PI / 180.0f);
            double circlePoint_Y_m = trafficObj->Cfg.l / 2 * sin(j_deg * LSM_GEOML::LSM_PI / 180.0f);
            if (j_deg == 180.0f)
            {
                //this is added because sin(180°) != 0 (e-16 value)
                worldPt_m[int(j_deg / (360.0f / maxPointToSplit))].x_dir = float(originCircleX_m + cYaw * circlePoint_X_m - 0.0); // sYaw * 0; sin(180°) = 0 => circlePoint_Y_m = 0
                worldPt_m[int(j_deg / (360.0f / maxPointToSplit))].y_dir = float(originCircleY_m + sYaw * circlePoint_X_m - 0.0); // cYaw * 0; sin(180°) = 0 => circlePoint_Y_m = 0
            }
            else
            {
                worldPt_m[int(j_deg / (360.0f / maxPointToSplit))].x_dir = float(originCircleX_m + cYaw * circlePoint_X_m - sYaw * circlePoint_Y_m);
                worldPt_m[int(j_deg / (360.0f / maxPointToSplit))].y_dir = float(originCircleY_m + sYaw * circlePoint_X_m + cYaw * circlePoint_Y_m);
            }

        }
        break;
        //for other case than describe before
    case objectType::Other:
        for (unsigned int i = numberOfPoints; i > 0; i--)
        {
            worldPt_m[numberOfPoints - i].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * localPoint[i - 1].x() - sYaw * localPoint[i - 1].y());
            worldPt_m[numberOfPoints - i].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * localPoint[i - 1].x() + cYaw * localPoint[i - 1].y());
        }
        break;
        //calculate the points for object using the length and the width of object
    case objectType::Rect4Points:
        /*transform front right -- CarMakerObjectOriginX + length; CarMakerObjectOriginY + width*/
        worldPt_m[0].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * trafficObj->Cfg.l - sYaw * trafficObj->Cfg.w / 2.0);
        worldPt_m[0].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * trafficObj->Cfg.l + cYaw * trafficObj->Cfg.w / 2.0);
        /*transform front left -- CarMakerObjectOriginX ; CarMakerObjectOriginY + width*/
        worldPt_m[1].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * 0.0f - sYaw * trafficObj->Cfg.w / 2.0);
        worldPt_m[1].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * 0.0f + cYaw * trafficObj->Cfg.w / 2.0);
        /*transform rear left -- CarMakerObjectOriginX; CarMakerObjectOriginY*/
        worldPt_m[2].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * 0.0f - sYaw * (-1) * trafficObj->Cfg.w / 2.0);
        worldPt_m[2].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * 0.0f + cYaw * (-1) * trafficObj->Cfg.w / 2.0);
        /*transform rear right -- CarMakerObjectOriginX + length; CarMakerObjectOriginY*/
        worldPt_m[3].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * trafficObj->Cfg.l - sYaw * (-1) * trafficObj->Cfg.w / 2.0);
        worldPt_m[3].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * trafficObj->Cfg.l + cYaw * (-1) * trafficObj->Cfg.w / 2.0);
        numberOfPoints = 4U;
        break;

    default:
        for (unsigned int i = numberOfPoints; i > 0; i--)
        {
            worldPt_m[numberOfPoints - i].x_dir = float((trafficObj->t_0[0] + offsetX_m) + cYaw * localPoint[i - 1].x() - sYaw * localPoint[i - 1].y());
            worldPt_m[numberOfPoints - i].y_dir = float((trafficObj->t_0[1] + offsetY_m) + sYaw * localPoint[i - 1].x() + cYaw * localPoint[i - 1].y());
        }
        break;
    }

    return numberOfPoints;
}
