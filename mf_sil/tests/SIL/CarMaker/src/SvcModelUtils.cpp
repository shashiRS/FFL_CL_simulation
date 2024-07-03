#include "SvcModelUtils.h"

#include <unordered_map>

#include "CarMaker.h"
#include "Car/Car.h"

#include "svcModelWrapper.h"
#include "TestRunWrapper.h"
#include "CemSurrogate.h"
#include "MfSilTypes.h"

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

uint32_t constexpr NUM_COORDS{ 3U }; // XYZ coordinate system; used to store XYZ points in sequential order in arrays (instead of utilizing multi-layered containers like matrices or structs)

std::array<float, 3U> SvcModelUtils::vehicleOriginPosition{ 0.0f, 0.0f, 0.0f };
double SvcModelUtils::rearLeftWheelPosDefault[3U]{ 1.096, 0.785, 0.3244 };
double SvcModelUtils::rearRightWheelPosDefault[3U]{ 1.096, -0.785, 0.3244 };
std::vector<uint8_t> SvcModelUtils::trafficObjects2DContureMirroring;

static std::vector<VCEM::ODSlot> const* odSlots{ new std::vector<VCEM::ODSlot>() };


void SvcModelUtils::getVehicleOriginPosition()
{
    double* rearLeftWheelPos;
    double* rearRightWheelPos;

    rearLeftWheelPos = iGetFixedTableOpt2(SimCore.Vhcl.Inf, "Wheel.rl.pos", rearLeftWheelPosDefault, 3, 1);
    rearRightWheelPos = iGetFixedTableOpt2(SimCore.Vhcl.Inf, "Wheel.rr.pos", rearRightWheelPosDefault, 3, 1);

    vehicleOriginPosition[0U] = static_cast<float>(0.5F * (rearLeftWheelPos[0U] + rearRightWheelPos[0U]));
    vehicleOriginPosition[1U] = static_cast<float>(0.5F * (rearLeftWheelPos[1U] + rearRightWheelPos[1U]));
    vehicleOriginPosition[2U] = static_cast<float>(0.5F * (rearLeftWheelPos[2U] + rearRightWheelPos[2U]));
}

void SvcModelUtils::getTrafficObjects2DContureMirroring()
{
    trafficObjects2DContureMirroring.clear();
    std::string paramName{ "Traffic.N" };
    int32_t numberOfObject{ 0 };

    if (iEntryExists(SimCore.TestRun.Inf, paramName.c_str())) {
        numberOfObject = iGetInt(SimCore.TestRun.Inf, paramName.c_str());
    }

    for (int32_t i{ 0 }; i < numberOfObject; ++i) {
        paramName = "Traffic." + std::to_string(i) + ".Basics.Contour.Mirror";
        trafficObjects2DContureMirroring.push_back(0U);

        if (iEntryExists(SimCore.TestRun.Inf, paramName.c_str())) {
            trafficObjects2DContureMirroring[i] = static_cast<uint8_t>(iGetInt(SimCore.TestRun.Inf, paramName.c_str()));
        }
    }
}

bool SvcModelUtils::is2DContureMirroring(int32_t const trafficObjectId)
{
    bool isMirroring{ false };

    if (trafficObjectId >= trafficObjects2DContureMirroring.size()) {
        return isMirroring;
    }

    isMirroring = (trafficObjects2DContureMirroring[trafficObjectId] != 0U) ? true : false;
    return isMirroring;
}

#ifdef USE_ENV_PLOTTER
static void copyGdrPointList(gdr::GdrPointList const& input, MF_Plot::sensorPointList& output)
{
    output.setSize(input.numPoints);

    for (LSM_GEOML::size_type i{ 0U }; i < output.getSize(); ++i) {
        output[i].point = cml::Vec2Df(input.pointList[i].x_m, input.pointList[i].y_m);
        output[i].height = input.pointList[i].z_m;
        output[i].confidence = static_cast<uint8_t>(std::round(100.0F * input.pointList[i].confidence));
    }
}

static void copyParkingLineList(pmsd::ParkingLineList const& input, MF_Plot::markerList& output)
{
    output.setSize(input.numberOfLines);

    for (LSM_GEOML::size_type i{ 0U }; i < output.getSize(); ++i) {
        output[i].start = cml::Vec2Df(input.parkingLines[i].startPoint.x, input.parkingLines[i].startPoint.y);
        output[i].end = cml::Vec2Df(input.parkingLines[i].endPoint.x, input.parkingLines[i].endPoint.y);
        output[i].confidence = static_cast<uint8_t>(std::round(100.0F * input.parkingLines[i].confidence));
    }
}

static void copyWheelStopperList(pmsd::WheelStopperList const& input, MF_Plot::markerList& output)
{
    output.setSize(input.numberOfWheelStoppers);

    for (LSM_GEOML::size_type i{ 0U }; i < output.getSize(); ++i) {
        output[i].start = cml::Vec2Df(input.wheelStoppers[i].startPoint.x, input.wheelStoppers[i].startPoint.y);
        output[i].end = cml::Vec2Df(input.wheelStoppers[i].endPoint.x, input.wheelStoppers[i].endPoint.y);
        output[i].confidence = static_cast<uint8_t>(std::round(100.0F * input.wheelStoppers[i].confidence));
    }
}

void SvcModelUtils::copySVCData(svc_model_processing::SvcModelProcessingOutput const& input, SVCPlotterData& output)
{
    copyGdrPointList(input.data->gdrPointsListFront, output.gdrFront);
    copyGdrPointList(input.data->gdrPointsListRear, output.gdrRear);
    copyGdrPointList(input.data->gdrPointsListRight, output.gdrRight);
    copyGdrPointList(input.data->gdrPointsListLeft, output.gdrLeft);

    copyParkingLineList(input.data->parkingLineListRight, output.pmdRight);
    copyParkingLineList(input.data->parkingLineListLeft, output.pmdLeft);
    copyParkingLineList(input.data->parkingLineListFront, output.pmdFront);
    copyParkingLineList(input.data->parkingLineListRear, output.pmdRear);

    copyWheelStopperList(input.data->wheelStopperListRight, output.wheelStopperListRight);
    copyWheelStopperList(input.data->wheelStopperListLeft, output.wheelStopperListLeft);
    copyWheelStopperList(input.data->wheelStopperListFront, output.wheelStopperListFront);
    copyWheelStopperList(input.data->wheelStopperListRear, output.wheelStopperListRear);
}
#endif

// functor struct used in the std::sort function call
struct DistFunctor
{
    DistFunctor(cml::Vec2Df const& _origin, cml::Vec2Df const& _leftEdgeDir) : origin(_origin), leftEdgeDir(_leftEdgeDir)
    {}

    bool operator()(cml::Vec2Df const& lhs, cml::Vec2Df const& rhs) const
    {
        cml::Vec2Df const& lhsDir{ LSM_GEOML::LineSegment2D(origin, lhs).directionVec() };
        cml::Vec2Df const& rhsDir{ LSM_GEOML::LineSegment2D(origin, rhs).directionVec() };
        ;
        return LSM_GEOML::calcAngleBetweenVectors(leftEdgeDir, lhsDir) < LSM_GEOML::calcAngleBetweenVectors(leftEdgeDir, rhsDir);
    }

private:
    cml::Vec2Df origin;
    cml::Vec2Df leftEdgeDir;
};

void SvcModelUtils::assignODSlots(std::vector<VCEM::ODSlot> const& _odSlots)
{
    odSlots = &_odSlots;
}

void SvcModelUtils::fillSvcModelInputByTrafficObjects(svc_model_processing::SvcModelProcessingInput& input, uint64_t const timeStamp_us,
                                                      svc_model_processing::SvcModelProcessingSettings const& settings)
{
    input.signalUpdates = settings.ports;

    uint32_t size{ std::min(static_cast<uint32_t>(Traffic.nObjs), input.maxNumObjects) };  // make sure the nb of traffic objs is not bigger than the max allowed nb of obj for the SVC input
    uint32_t pedCrossCount{ 0U };
    uint32_t dynBboxCount{ 0U };
    uint32_t dynCubCount{ 0U };

    input.timestamp = timeStamp_us;
    input.numObjects = size;

    std::transform(Car.Fr1.t_0, Car.Fr1.t_0 + NUM_COORDS, input.egoPosition,
                   [](auto& value)
    {
        return static_cast<float>(value);
    });  // car position in XYZ

    std::transform(Car.Fr1.r_zyx, Car.Fr1.r_zyx + NUM_COORDS, input.egoRotationAngles,
                   [](auto& value)
    {
        return static_cast<float>(value);
    });  // roll, pitch, yaw angles around ZYX axes

    std::copy(vehicleOriginPosition.begin(), vehicleOriginPosition.end(), input.egoOrigin);  // default: 0, 0, 0

    tTrafficObj* trafficObj;
    for (size_t i{ 0U }; i < size; ++i) {
        trafficObj = Traffic_GetByTrfId(static_cast<int32_t>(i));

        char trafficObjName[TRF_NAMESIZE];
        strcpy_s(trafficObjName, TRF_NAMESIZE, trafficObj->Cfg.Name);  // copy the trf obj name in the temp var

        if (strstr(trafficObjName, "Lim") && !strstr(trafficObj->Cfg.Info, "- line")) {  // when the trf obj isn't a line marking of any kind (road or park)
            char const obstacleNamePrefix[4U]{ "Obs" };
            uint32_t const obstacleNamePrefixLength{ 3U };  // len of 'Lim' and 'Obs'
            for (size_t ii{ 0U }; ii < obstacleNamePrefixLength; ++ii) {
                trafficObjName[ii] = obstacleNamePrefix[ii];  // replace 'Lim' with 'Obs'
            }
        }

        int32_t isDynBBox{ -1 }; // -1 = NOT DYN, 0 = CUB, 1 = BBOX
        if (strstr(trafficObjName, "Dyn")) {
            isDynBBox = 1;
            uint32_t constexpr dynObjPrefixLen{ 6U };
            char dynObjPrefix[dynObjPrefixLen]{ "" };

            if (strstr(trafficObj->Cfg.Info, "CAR")) {
                isDynBBox = 0;
                strcpy_s(dynObjPrefix, dynObjPrefixLen, "car--");
            }
            else if (strstr(trafficObj->Cfg.Info, "VAN")) {
                isDynBBox = 0;
                strcpy_s(dynObjPrefix, dynObjPrefixLen, "van--");
            }
            else if (strstr(trafficObj->Cfg.Info, "TRUCK")) {
                isDynBBox = 0;
                strcpy_s(dynObjPrefix, dynObjPrefixLen, "truck");
            }
            else if (strstr(trafficObj->Cfg.Info, "PEDESTRIAN")) {
                strcpy_s(dynObjPrefix, dynObjPrefixLen, "pedes");
            }
            else if (strstr(trafficObj->Cfg.Info, "TWOWHEELER")) {
                strcpy_s(dynObjPrefix, dynObjPrefixLen, "2whlr");
            }
            else if (strstr(trafficObj->Cfg.Info, "SHOPPING_CART")) {
                strcpy_s(dynObjPrefix, dynObjPrefixLen, "shopC");
            }
            else if (strstr(trafficObj->Cfg.Info, "ANIMAL")) {
                strcpy_s(dynObjPrefix, dynObjPrefixLen, "animl");
            }
            else if (strstr(trafficObj->Cfg.Info, "BBOX_UNKNOWN")) {
                strcpy_s(dynObjPrefix, dynObjPrefixLen, "bbUnk");
            }
            else if (strstr(trafficObj->Cfg.Info, "CUB_UNKNOWN")) {
                isDynBBox = 0;
                strcpy_s(dynObjPrefix, dynObjPrefixLen, "cbUnk");
            }

            for (size_t ii{ 0U }; ii < (dynObjPrefixLen - 1U); ++ii) {
                trafficObjName[ii] = dynObjPrefix[ii];  // replace 'Dyn' with the correct dynamic object type
            }
        }

        if (input.maxObjNameLength >= TRF_NAMESIZE) {  // if the trf obj name length is smaller than max, copy the whole name
            strcpy_s(input.objectsNames[i], input.maxObjNameLength, trafficObjName);
        }
        else {  // otherwise, only copy the max allowed length
            strncpy_s(input.objectsNames[i], input.maxObjNameLength,
                      trafficObjName, input.maxObjNameLength - 1U);
        }

        std::transform(trafficObj->t_0, trafficObj->t_0 + NUM_COORDS,
                       input.objectsPositions + i * NUM_COORDS,
                       [](auto& value)
        {
            return static_cast<float>(value);
        });  // XYZ coordinates (in global coord system) of the trf obj, stored in sequential order

        std::transform(trafficObj->r_zyx, trafficObj->r_zyx + NUM_COORDS,
                       input.objectsRotationAngles + i * NUM_COORDS,
                       [](auto& value)
        {
            return static_cast<float>(value);
        });  // roll, pitch, yaw angles around ZYX axes for the trf obj

        // save the trf obj length, width and height as the bounding box sizes
        input.objectsSizes[i * NUM_COORDS] = static_cast<float>(trafficObj->Cfg.l);
        input.objectsSizes[i * NUM_COORDS + 1U] = static_cast<float>(trafficObj->Cfg.w);
        input.objectsSizes[i * NUM_COORDS + 2U] = static_cast<float>(trafficObj->Cfg.h);
        input.objectsID[i] = static_cast<int32_t>(i);

        // any object that doesn't contain the confidence does not have a confidence in its output port or it has another processing method elsewhere
        // the main confidence value of the object is 'generic_conf', it's possible some objects have secondary confidences (such as SPP semantic points)
        // in the description, the parameter section is always like this: [<confidence_param> (, <optional further parameters>)]
        std::string trfObjInfo(trafficObj->Cfg.Info);
        size_t const infoParamStart{ trfObjInfo.find_last_of("[") };
        std::vector<std::string> confParamList;

        if (infoParamStart != std::string::npos) {
            trfObjInfo.erase(0U, infoParamStart + 1U); // erase all characters leading to the start of the param list, including the '['

            size_t const infoParamEnd{ trfObjInfo.find_first_of("]") };
            trfObjInfo.erase(infoParamEnd, std::string::npos); // erase until the end of the string
            std::remove_if(trfObjInfo.begin(), trfObjInfo.end(), isspace);  // clean the string of unnecessary spaces

            std::string splitItem;
            std::stringstream strStream(trfObjInfo);
            while (std::getline(strStream, splitItem, ',')) {
                confParamList.push_back(std::move(splitItem));
            }
        }

        uint32_t const numCubSubClass{ 3U };
        uint32_t const numBboxSubClass{ 4U };

        // before checking the Traffic Object attributes, assign the minimaneuver confidences; even if they haven't been set, they'll be -1.0, which will get checked further on
        input.objectsConf[i] = settings.genericConf[i]; // the objects main confidence (almost all objects have this confidence)
        input.objectsSemConf[i] = settings.semPointsConf[i]; // Semantic Points confidence (can be given optionally alongside objects that GDR points are generated for (static/dynamic objects)
        if (isDynBBox == 1) { // Dyn Bounding Box case
            for (uint32_t ii{ 0U }; ii < numBboxSubClass; ++ii) {
                input.bboxSubConf[dynBboxCount][ii] = settings.bboxSubConf[i][ii]; // dynamic objects cuboid subclass confidences 
            }
        }
        else if (isDynBBox == 0) { // Dyn Cuboid case
            for (uint32_t ii{ 0U }; ii < numCubSubClass; ++ii) {
                input.cubSubConf[dynCubCount][ii] = settings.cubSubConf[i][ii]; // dynamic objects bounding box subclass confidences 
            }
        }

        if (!confParamList.empty()) {
            // std::find returns an iterator to the element found
            // std::distance returns the hops taken to reach that element
            size_t const genericConfPos{ static_cast<size_t>(std::distance(confParamList.begin(), std::find(confParamList.begin(), confParamList.end(), "generic_conf"))) };
            size_t const semConfPos{ static_cast<size_t>(std::distance(confParamList.begin(), std::find(confParamList.begin(), confParamList.end(), "sem_points_conf"))) };
            size_t const dynConfPos{ static_cast<size_t>(std::distance(confParamList.begin(), std::find(confParamList.begin(), confParamList.end(), "subclass_conf"))) };

            if (input.objectsConf[i] < 0.0F) { // only continue if minimaneuver confidence has NOT been set
                if (genericConfPos < confParamList.size()) { // the  confidence has a valid position in the parameter list
                    input.objectsConf[i] = static_cast<float>(trafficObj->Cfg.attrib[genericConfPos]);
                }
                else {  // needs to be assigned here as well, in case the param list is not empty, but the current param is not present
                    input.objectsConf[i] = 1.0F;
                }
            }

            if (input.objectsSemConf[i] < 0.0F) {
                if (semConfPos < confParamList.size()) {
                    input.objectsSemConf[i] = static_cast<float>(trafficObj->Cfg.attrib[semConfPos]);
                }
                else {
                    input.objectsSemConf[i] = 1.0F;
                }
            }

            if (isDynBBox == 1) {

                if (dynConfPos < confParamList.size()) {
                    for (uint32_t ii{ 0U }; ii < numBboxSubClass; ++ii) {
                        if (input.bboxSubConf[dynBboxCount][ii] >= 0.0F) { // if already set previously with a valid confidence, skip
                            continue;
                        }

                        input.bboxSubConf[dynBboxCount][ii] = static_cast<float>(trafficObj->Cfg.attrib[dynConfPos + ii]);
                    }
                }
                else {
                    for (uint32_t ii{ 0U }; ii < numBboxSubClass; ++ii) {
                        input.bboxSubConf[dynBboxCount][ii] = 1.0F;
                    }
                }

                ++dynBboxCount;
            }
            else if (isDynBBox == 0) {

                if (dynConfPos < confParamList.size()) {
                    for (uint32_t ii{ 0U }; ii < numCubSubClass; ++ii) {
                        if (input.cubSubConf[dynCubCount][ii] >= 0.0F) { // if already set previously with a valid confidence, skip
                            continue;
                        }

                        input.cubSubConf[dynCubCount][ii] = static_cast<float>(trafficObj->Cfg.attrib[dynConfPos + ii]);
                    }
                }
                else {
                    for (uint32_t ii{ 0U }; ii < numCubSubClass; ++ii) {
                        input.cubSubConf[dynCubCount][ii] = 1.0F;
                    }
                }
                ++dynCubCount;
            }
        }
        else { // assume full confidence if parameters are omitted
            input.objectsConf[i] = 1.0F; 
            input.objectsSemConf[i] = 1.0F; // doesn't matter if it is set for objects that don't use it (it will just be ignored)

            for (uint32_t ii{ 0U }; ii < numBboxSubClass; ++ii) {
                input.bboxSubConf[dynBboxCount][ii] = 1.0F;
            }

            for (uint32_t ii{ 0U }; ii < numCubSubClass; ++ii) {
                input.cubSubConf[dynCubCount][ii] = 1.0F;
            }
        }

        // Odo will get filtered out further on in the code, so save the FreeSpace polylines confidence
        if (trfObjIsOdo(&(trafficObj->Cfg))) {
            input.fsConf = settings.sppPolyConf;

            if (!confParamList.empty()) {
                size_t const sppPolyConfPos{ static_cast<size_t>(std::distance(confParamList.begin(), std::find(confParamList.begin(), confParamList.end(), "spp_poly_conf"))) };

                if (input.fsConf < 0.0F) { // only continue if minimaneuver confidence has NOT been set
                    if (sppPolyConfPos < confParamList.size()) { // the confidence has a valid position in the parameter list
                        input.fsConf = static_cast<float>(trafficObj->Cfg.attrib[sppPolyConfPos]);
                    }
                    else { // needs to be assigned here as well, in case the param list is not empty, but the current param is not present
                        input.fsConf = 1.0F;
                    }
                }
            }
            else {
                input.fsConf = 1.0F; // assume full confidence if parameters are omitted
            }
        }

        if (strstr(trafficObjName, "PC")) { // pedestrian crossing
            for (uint32_t ii{ 0U }; ii < 4U; ++ii) {
                input.pedCrossX[pedCrossCount][ii] = trafficObj->Envelope.Areas_0[ii].Pos[0U];
                input.pedCrossY[pedCrossCount][ii] = trafficObj->Envelope.Areas_0[ii].Pos[1U];
            }
            ++pedCrossCount;
        }

        double localShifting[3U]{ 0.5 * trafficObj->Cfg.l, 0.0, 0.0 };
        if ((trafficObj->Cfg.Envelope.Mode == tEnvelopeMode::EnvelopeMode_2DContour) && !is2DContureMirroring(static_cast<int32_t>(i))) {
            localShifting[1U] = 0.5 * trafficObj->Cfg.w;
        }

        std::transform(localShifting, localShifting + NUM_COORDS,
                       input.objectsShifts + i * NUM_COORDS,
                       [](auto& value)
        {
            return static_cast<float>(value);
        });

        // ODO VEHICLE
        if (trfObjIsOdo(&(trafficObj->Cfg))) {
            input.semantic[i] = static_cast<uint8_t>(spp::SemanticInformation_t::BACKGROUND);
        }
        // DYNAMIC OBSTACLE
        else if (trfObjIsDyn(trafficObj)) {
            input.semantic[i] = static_cast<uint8_t>(spp::SemanticInformation_t::DYNAMIC_OBSTACLE);
        }
        // PARKING BOX
        else if (trfObjIsParkBox(&(trafficObj->Cfg))) {
            input.semantic[i] = static_cast<uint8_t>(spp::SemanticInformation_t::DRIVABLE_AREA);
        }
        // VEHICLE STATIC OBSTACLE
        else if (trfObjIsStatic(trafficObj)) {
            input.semantic[i] = static_cast<uint8_t>(spp::SemanticInformation_t::STATIC_OBSTACLE);
        }
        // OTHER STATIC OBSTACLES: ROUND
        else if (trfObjIsRoundStatic(&(trafficObj->Cfg))) {
            input.semantic[i] = static_cast<uint8_t>(spp::SemanticInformation_t::STATIC_OBSTACLE);
        }
        // OTHER STATIC OBSTACLES: RECTANGULAR
        else if (trfObjIsRectStatic(&(trafficObj->Cfg))) {
            input.semantic[i] = static_cast<uint8_t>(spp::SemanticInformation_t::STATIC_OBSTACLE);
        }
        // CURBSTONE
        else if (trfObjIsCurbstone(&(trafficObj->Cfg))) {
            input.semantic[i] = static_cast<uint8_t>(spp::SemanticInformation_t::CURB);
        }
        // WHEELSTOPPER
        else if (trfObjIsWhlStp(&(trafficObj->Cfg))) {
            input.semantic[i] = static_cast<uint8_t>(spp::SemanticInformation_t::DRIVABLE_AREA);
        }
        // ROAD LANE MARKING
        else if (trfObjIsRoadLaneMark(&(trafficObj->Cfg))) {
            input.semantic[i] = static_cast<uint8_t>(spp::SemanticInformation_t::DRIVABLE_AREA);
        }
        // PARKING LINE MARKER
        else if (trfObjIsParkLineMark(&(trafficObj->Cfg))) { // exclude external and roadside lines for TestRuns with double lines (US_Scenarios)
            input.semantic[i] = static_cast<uint8_t>(spp::SemanticInformation_t::PARKING_MARKER);
        }
    }

    uint32_t segmCount{ 0U };
    uint32_t trfObjSemCount{ 0U };

    for (int32_t i{ 0 }; i < FSpaceSensorCount; ++i) {

        size_t camID{ 0U };
        bool sideCam{ false };

        if (strncmp(FSpaceSensor[i].bs.Name, "CAM_F", 5U) == 0) {
            camID = 0U;
        }
        else if (strncmp(FSpaceSensor[i].bs.Name, "CAM_B", 5U) == 0) {
            camID = 1U;
        }
        else if (strncmp(FSpaceSensor[i].bs.Name, "CAM_L", 5U) == 0) {
            camID = 2U;
            sideCam = true;
        }
        else if (strncmp(FSpaceSensor[i].bs.Name, "CAM_R", 5U) == 0) {
            camID = 3U;
            sideCam = true;
        }
        else { // Ignore USS FSpaceSensors because they are disabled for the SVC surrogate model
            continue;
        }

        // Assert that the free-space sensor data is not older than 1 ms (e.g. one CarMaker cycle delayed)
        if (std::abs(SimCore.Time - FSpaceSensor[i].TimeStamp) < 0.0011) {
            // snippet taken from the CemSurrogate model to compute where the FOV limit arc would hit the ground
            float32_t const effectiveRange{ static_cast<float32_t>(std::sqrt(std::pow(FSpaceSensor[i].range, 2) - std::pow(FSpaceSensor[i].bs.Pos_B[2], 2))) };
            cml::Vec2Df const sensorPosition{ static_cast<float32_t>(FSpaceSensor[i].BS_Pos_0[0]), static_cast<float32_t>(FSpaceSensor[i].BS_Pos_0[1]) };
            LSM_GEOML::Pose sensorPose(sensorPosition, static_cast<float32_t>(FSpaceSensor[i].rot_zyx[2] + FSpaceSensor[i].bs.Fr->r_zyx[2]));
            SensorFov2D const sensorFov{ sensorPose, effectiveRange, static_cast<float32_t>(FSpaceSensor[i].alpha) };

            // references to edge lines points
            cml::Vec2Df const& leftP1{ sensorFov.getFovLeftEdge().p1() };
            cml::Vec2Df const& leftP2{ sensorFov.getFovLeftEdge().p2() };
            cml::Vec2Df const& rightP1{ sensorFov.getFovRightEdge().p1() };
            cml::Vec2Df const& rightP2{ sensorFov.getFovRightEdge().p2() };

            int32_t constexpr segmDivMult{ 2 }; // the number of horizontal segments gets multiplied by this arbitrary value for granularity
            int32_t const numSegToDivArc{ FSpaceSensor[i].nHorSegm * segmDivMult };

            float32_t hystVal{ 0.25F }; // this gets added to the left and right FOV edges to extend the lines and to the radius as well (default: 0.25)
            if (sideCam) {
                hystVal = 0.15F;
            }
            // these ratio variables are the divisions between the distances from the camera mounting point to the new points on the extended lines and the FOV line length
            // this is (FOV_len + x) / FOV_len ... which is 1 + (x / FOV_len)
            float32_t const leftFovDistRatio{ 1.0F + (hystVal / sensorFov.getFovLeftEdge().length()) };
            float32_t const rightFovDistRatio{ 1.0F + (hystVal / sensorFov.getFovRightEdge().length()) };
            cml::Vec2Df const extndLeftFovPoint{ cml::Vec2Df((1.0F - leftFovDistRatio) * leftP1.x() + leftFovDistRatio * leftP2.x(),
                                                             (1.0F - leftFovDistRatio) * leftP1.y() + leftFovDistRatio * leftP2.y()) };
            cml::Vec2Df const extndRightFovPoint{ cml::Vec2Df((1.0F - rightFovDistRatio) * rightP1.x() + rightFovDistRatio * rightP2.x(),
                                                              (1.0F - rightFovDistRatio) * rightP1.y() + rightFovDistRatio * rightP2.y()) };
            cml::Vec2Df const extLeftDir{ LSM_GEOML::LineSegment2D(leftP1, extndLeftFovPoint).directionVec() };
            cml::Vec2Df const extRightDir{ LSM_GEOML::LineSegment2D(rightP1, extndRightFovPoint).directionVec() };

            float32_t const fullAzimuth_rad{ LSM_GEOML::calcAngleBetweenVectors(extLeftDir, extRightDir) };
            // range has the same hysteresis added to it as the extended lines so that the computed points are beyond the CarMaker Freespace FOV
            float32_t const hysteresisRange{ (sensorFov.getFovRangeM() + hystVal) };
            input.fovMaxRadius[camID] = hysteresisRange;  // save this side's FOV limit

            float32_t const arcLength{ fullAzimuth_rad * hysteresisRange };  // circle arc length = angle_radians * circle_radius
            float32_t const arcDivUnit{ arcLength / numSegToDivArc };  // in how many parts the circle arc would be divided

            // sensor position point
            input.FSsegmX[segmCount] = leftP1.x();
            input.FSsegmY[segmCount] = leftP1.y();
            ++segmCount;

            std::vector<std::vector<cml::Vec2Df>> polyPointCoords;  // stores each consecutive cluster of points
            std::vector<std::pair<int32_t, int32_t>> polyIdInterv;  // stores each start and end ID of each polyline
            std::unordered_map<int32_t, std::vector<cml::Vec2Df>> fovLimitPoints;  // stores the points of the empty FreeSpace segments (keys are column IDs [0, num_horiz_segm))
            bool currPolyline{ false };  // flag to determine if a polyline is currently being processed

            // the '0' and 'numSegToDivArc' are the limits themselves (left and right edge segm ends); right edge is not included here but is generated below using the penultimate index
            for (int32_t ii{ 0 }; ii < FSpaceSensor[i].nHorSegm; ++ii) {

                bool detPoint{ false };
                // go through all the current vertical column segments; if any of them is filled, we save it as the polyline start
                for (int32_t iii{ 0 }; iii < FSpaceSensor[i].nVerSegm; ++iii) {
                    int32_t const vertSegmId{ ii + iii * FSpaceSensor[i].nHorSegm }; // travel downward on the column (just need to add the number of horizontal segments multiplied by the level)
                    tSegment const& sensorSegment{ FSpaceSensor[i].Segm[vertSegmId] };
                    tTrafficObj const* trafficObj{ Traffic_GetByObjId(sensorSegment.ObjId) };

                    if (std::abs(sensorSegment.P_0[0U]) > 1e-2 && std::abs(sensorSegment.P_0[1U]) > 1e-2) { // point is not empty

                        if (sensorSegment.ObjId >= 0 && trafficObj && !trfObjIsOdo(&(trafficObj->Cfg)) // exclude odometry box
                        && (trafficObj->Cfg.Name[0U] != 'P') && (!trfObjIsODS(&(trafficObj->Cfg))) // exclude parking boxes and OD Slots
                        // exclude road and parking line marking objects
                        && !(strlen(trafficObj->Cfg.Name) >= 3U && (strncmp("Lim", trafficObj->Cfg.Name, 3U) == 0) && strstr(trafficObj->Cfg.Info, "- line"))
                        && !(strstr(trafficObj->Cfg.Name, "PC")) && !(strstr(trafficObj->Cfg.Name, "SL")) 
                        && !(strstr(trafficObj->Cfg.Name, "WL"))) {

                            detPoint = true;

                            uint8_t fsSemantic;
                            if (trfObjIsDyn(trafficObj)) {
                                fsSemantic = static_cast<uint8_t>(spp::SemanticInformation_t::DYNAMIC_OBSTACLE);
                            }
                            else if (trfObjIsCurbstone(&(trafficObj->Cfg))) {
                                fsSemantic = static_cast<uint8_t>(spp::SemanticInformation_t::CURB);
                            }
                            else { // we exhausted all cases except any STATIC obstacle
                                fsSemantic = static_cast<uint8_t>(spp::SemanticInformation_t::STATIC_OBSTACLE);
                            }

                            bool skipChecks{ false };
                            uint32_t prevCount;

                            if (trfObjSemCount == 0U) {
                                skipChecks = true; // no other trf obj processed, append without checking
                            }
                            else {
                                int32_t lastPopSem{ -1 };
                                for (size_t iv{ 0U }; iv < 4U; ++iv) {
                                    if (input.numFSsemId[iv] != 0U) {
                                        lastPopSem = static_cast<int32_t>(input.numFSsemId[iv]);
                                    }
                                }

                                if (static_cast<int32_t>(trfObjSemCount) == lastPopSem) {  // we are processing a new camera which doesn't contain any FS semantic info yet, so skip checks
                                    skipChecks = true;
                                }
                                else { // otherwise, we either don't have any other cameras processed, OR the current camera already contains semantic points, so we need to check
                                    prevCount = trfObjSemCount - 1U;
                                }
                            }

                            if (skipChecks || (!skipChecks && input.FSsemantic[prevCount] != fsSemantic)) {
                                input.FSsemantic[trfObjSemCount] = fsSemantic;
                                input.FSsemanticId[trfObjSemCount] = ii * segmDivMult;  // populate with the ID of the start of the points in the current column (we know there cannot be more than 1 obj per segment)
                                ++trfObjSemCount;
                            }

                            if (!currPolyline) {
                                currPolyline = true;
                                polyPointCoords.push_back(std::vector<cml::Vec2Df>());
                                polyIdInterv.push_back(std::make_pair(ii, -1)); // save -1 as the polyline end ID because we don't know it yet
                            }

                            polyPointCoords.back().push_back(cml::Vec2Df(sensorSegment.P_0[0U], sensorSegment.P_0[1U]));
                            break;
                        }
                    }
                }

                // in case a polyline is active and we reached the end of the segments, take that as the end ID
                if ((currPolyline && (!detPoint)) || (currPolyline && (ii == (FSpaceSensor[i].nHorSegm - 1)))) {
                    currPolyline = false;  // the current polyline has stopped
                    polyIdInterv.back().second = (ii == (FSpaceSensor[i].nHorSegm - 1)) ? FSpaceSensor[i].nHorSegm : ii;  // the polyline end ID has been found

                    // sort the detected points LEFT to RIGHT (based on angles between the direction vectors and the reference: left edge)
                    cml::Vec2Df lastEmptySegm(extndLeftFovPoint);  // assume the left FOV edge as reference (in case of the first column being populated)
                    if (polyIdInterv.back().first != 0U) {
                        lastEmptySegm = fovLimitPoints[polyIdInterv.back().first - 1].back();
                    }

                    std::sort(std::begin(polyPointCoords.back()), std::end(polyPointCoords.back()), DistFunctor(leftP1, extLeftDir));

                    if (ii == (FSpaceSensor[i].nHorSegm - 1)) { // in case we have detected points in the last column of segments, skip the FOV limit computation
                        continue;
                    }
                }

                if (currPolyline) { // if we detected an object in the current column, skip processing it as a polyline is active
                    continue;
                }

                int32_t currColumnDivNum{ segmDivMult };
                if (ii == (FSpaceSensor[i].nHorSegm - 1)) {
                    ++currColumnDivNum; // for the last column, we need to compute the FOV right edge limit, as well, so 1 is added
                }

                // build the specified number of points for the current column, based on segm ID
                for (int32_t iii{ ii * segmDivMult }; iii < (ii * segmDivMult + currColumnDivNum); ++iii) {
                    // Theta is the angle between the leftmost point on the arc (left edge) and the next point to be generated, the 'rightmost' generated point so far (not right edge)
                    float32_t const currArcLength{ arcDivUnit * iii };
                    float32_t const theta{ currArcLength / hysteresisRange };

                    fovLimitPoints[ii].push_back(leftP1 + hysteresisRange * LSM_GEOML::getVecFromAngle(sensorFov.getSensorPose().Yaw_rad() + sensorFov.getFovAzimuthRad() - theta));
                }
            }

            // assume 0 (for no detected points cases), then change as necessary
            int32_t remainingEmptyFS{ 0 };

            for (uint32_t ii{ 0U }; ii < static_cast<uint32_t>(polyIdInterv.size()); ++ii) {
                int32_t currFreespStart{ 0 }; // assume 0 as the freespace start, in case of the first interval 
                if (ii > 0U) {
                    currFreespStart = polyIdInterv.at(ii - 1U).second;
                }

                // populate the input array with the current freespace points (no detected objects)
                for (int32_t iii{ currFreespStart }; iii < polyIdInterv.at(ii).first; ++iii) {
                    if (fovLimitPoints.count(iii)) {
                        for (size_t iv{ 0U }; iv < fovLimitPoints[iii].size(); ++iv) {

                            input.FSsegmX[segmCount] = fovLimitPoints[iii].at(iv).x();
                            input.FSsegmY[segmCount] = fovLimitPoints[iii].at(iv).y();
                            ++segmCount;
                        }
                    }
                }

                // next, populate the input array with the current polyline; we know 'ii' is the idx for both the <polyIdInterv> and <polyPointCoords> vectors
                for (size_t iii{ 0U }; iii < polyPointCoords.at(ii).size(); ++iii) {
                    input.FSsegmX[segmCount] = polyPointCoords.at(ii).at(iii).x(); // points are already sorted
                    input.FSsegmY[segmCount] = polyPointCoords.at(ii).at(iii).y();
                    ++segmCount;
                }
            }

            if (!polyIdInterv.empty()) {  // in case of valid polylines, save the last polyline end ID so that we include the freespace points that come AFTER the last detected point
                remainingEmptyFS = polyIdInterv.back().second;
            }

            for (int32_t ii{ remainingEmptyFS }; ii < FSpaceSensor[i].nHorSegm; ++ii) {
                if (fovLimitPoints.count(ii)) {
                    for (size_t iii{ 0U }; iii < fovLimitPoints[ii].size(); ++iii) {
                        input.FSsegmX[segmCount] = fovLimitPoints[ii].at(iii).x();
                        input.FSsegmY[segmCount] = fovLimitPoints[ii].at(iii).y();
                        ++segmCount;
                    }
                }
            }

            input.numFSsegm[camID] = segmCount;
            input.numFSsemId[camID] = trfObjSemCount;
        }
    }

    input.numODslots = static_cast<uint32_t>((*odSlots).size());

    for (size_t i{ 0U }; i < (*odSlots).size(); ++i) {
        input.exist_prob[i] = odSlots->at(i).existence_probability;
        input.angled_conf[i] = odSlots->at(i).parking_scenario_confidence.angled;
        input.paral_conf[i] = odSlots->at(i).parking_scenario_confidence.parallel;
        input.perp_conf[i] = odSlots->at(i).parking_scenario_confidence.perpendicular;

        for (size_t ii{ 0U }; ii < 4U; ++ii) {
            input.slot_corners_x[i][ii] = odSlots->at(i).slot_corners[ii].x();
            input.slot_corners_y[i][ii] = odSlots->at(i).slot_corners[ii].y();

            input.cameraFlags[i][ii] = odSlots->at(i).cameraId[ii];
        }
    }

    input.numPedCross = pedCrossCount;
    input.numDynCub = dynCubCount;
    input.numDynBbox = dynBboxCount;
}


void SvcModelUtils::logSvcModelProcessingInput(svc_model_processing::SvcModelProcessingInput& input)
{
    Log("Timestamp: %d \n", input.timestamp);
    Log("Ego position: x %f, y %f, z %f\n",
        input.egoPosition[0U],
        input.egoPosition[1U],
        input.egoPosition[2U]);

    Log("Ego angles: x %f, y %f, z %f\n",
        input.egoRotationAngles[0U],
        input.egoRotationAngles[1U],
        input.egoRotationAngles[2U]);

    for (uint32_t i{ 0U }; i < input.numObjects; ++i) {
        Log("objectsNames[%d]: %s \n", i, input.objectsNames[i]);
        Log("objectsID[%d]: %d \n", i, input.objectsID[i]);

        Log("objectsPositions[%d]: x % f, y %f, z %f\n", i,
            input.objectsPositions[i * NUM_COORDS + 0U],
            input.objectsPositions[i * NUM_COORDS + 1U],
            input.objectsPositions[i * NUM_COORDS + 2U]);

        Log("objectsRotationAngles[%d]: x % f, y %f, z %f\n", i,
            input.objectsRotationAngles[i * NUM_COORDS + 0U],
            input.objectsRotationAngles[i * NUM_COORDS + 1U],
            input.objectsRotationAngles[i * NUM_COORDS + 2U]);

        Log("objectsSizes[%d]: x % f, y %f, z %f\n", i,
            input.objectsSizes[i * NUM_COORDS + 0U],
            input.objectsSizes[i * NUM_COORDS + 1U],
            input.objectsSizes[i * NUM_COORDS + 2U]);
    }
}

void SvcModelUtils::logGdrPointList(gdr::GdrPointList& list, std::string listName)
{
    Log((listName + " timestamp: %d\n").c_str(), list.sSigHeader.uiTimeStamp);
    Log((listName + " cameraID: %d\n").c_str(), list.sensorSource);
    Log((listName + " numberOfPoints: %d\n").c_str(), list.numPoints);
}

void SvcModelUtils::logParkingLineList(pmsd::ParkingLineList& list, std::string listName)
{
    Log((listName + " timestamp: %d\n").c_str(), list.sSigHeader.uiTimeStamp);
    Log((listName + " numberOfLines: %d\n").c_str(), list.numberOfLines);
}

void SvcModelUtils::logWheelStopperList(pmsd::WheelStopperList & list, std::string listName)
{
    Log((listName + " timestamp: %d\n").c_str(), list.sSigHeader.uiTimeStamp);
    Log((listName + " numberOfWheelStoppers: %d\n").c_str(), list.numberOfWheelStoppers);
}

void SvcModelUtils::logSvcModelProcessingOutput(svc_model_processing::SvcModelProcessingOutput& output)
{
    logGdrPointList(output.data->gdrPointsListFront, "gdrFront");
    logGdrPointList(output.data->gdrPointsListRear, "gdrRear");
    logGdrPointList(output.data->gdrPointsListLeft, "gdrLeft");
    logGdrPointList(output.data->gdrPointsListRight, "gdrRight");

    logParkingLineList(output.data->parkingLineListFront, "pmdFront");
    logParkingLineList(output.data->parkingLineListRear, "pmdRear");
    logParkingLineList(output.data->parkingLineListLeft, "pmdLeft");
    logParkingLineList(output.data->parkingLineListRight, "pmdRight");

    logWheelStopperList(output.data->wheelStopperListFront, "whsFront");
    logWheelStopperList(output.data->wheelStopperListRear, "whsRear");
    logWheelStopperList(output.data->wheelStopperListLeft, "whsLeft");
    logWheelStopperList(output.data->wheelStopperListRight, "whsRight");
}


void SvcModelUtils::declQuantsSvcModelProcessingInput(std::string prefixStr, svc_model_processing::SvcModelProcessingInput& input)
{
    DDefULLong(NULL, (prefixStr + "timestamp").c_str(), "us", &input.timestamp, DVA_None);
    DDefUInt(NULL, (prefixStr + "numObjects").c_str(), "", &input.numObjects, DVA_None);

    DDefFloat(NULL, (prefixStr + "egoPosition.x").c_str(), "m", &input.egoPosition[0U], DVA_None);
    DDefFloat(NULL, (prefixStr + "egoPosition.y").c_str(), "m", &input.egoPosition[1U], DVA_None);
    DDefFloat(NULL, (prefixStr + "egoPosition.z").c_str(), "m", &input.egoPosition[2U], DVA_None);

    DDefFloat(NULL, (prefixStr + "egoRotationAngles.x").c_str(), "rad", &input.egoRotationAngles[0U], DVA_None);
    DDefFloat(NULL, (prefixStr + "egoRotationAngles.y").c_str(), "rad", &input.egoRotationAngles[1U], DVA_None);
    DDefFloat(NULL, (prefixStr + "egoRotationAngles.z").c_str(), "rad", &input.egoRotationAngles[2U], DVA_None);

    DDefFloat(NULL, (prefixStr + "egoOrigin.x").c_str(), "m", &input.egoOrigin[0U], DVA_None);
    DDefFloat(NULL, (prefixStr + "egoOrigin.y").c_str(), "m", &input.egoOrigin[1U], DVA_None);
    DDefFloat(NULL, (prefixStr + "egoOrigin.z").c_str(), "m", &input.egoOrigin[2U], DVA_None);

    tDDefault *prefix{ DDefaultCreate(prefixStr.c_str()) };

    for (uint32_t i{ 0U }; i < input.maxNumObjects; ++i) {
        DDefPrefix(prefix, (prefixStr + "objectsID[%d]").c_str(), i);
        DDefInt(prefix, "", "", &input.objectsID[i], DVA_None);

        DDefPrefix(prefix, (prefixStr + "objectsPositions[%d].").c_str(), i);
        DDefFloat(prefix, "x", "m", &input.objectsPositions[i * NUM_COORDS + 0U], DVA_None);
        DDefFloat(prefix, "y", "m", &input.objectsPositions[i * NUM_COORDS + 1U], DVA_None);
        DDefFloat(prefix, "z", "m", &input.objectsPositions[i * NUM_COORDS + 2U], DVA_None);

        DDefPrefix(prefix, (prefixStr + "objectsRotationAngles[%d].").c_str(), i);
        DDefFloat(prefix, "x", "rad", &input.objectsRotationAngles[i * NUM_COORDS + 0U], DVA_None);
        DDefFloat(prefix, "y", "rad", &input.objectsRotationAngles[i * NUM_COORDS + 1U], DVA_None);
        DDefFloat(prefix, "z", "rad", &input.objectsRotationAngles[i * NUM_COORDS + 2U], DVA_None);

        DDefPrefix(prefix, (prefixStr + "objectsSizes[%d].").c_str(), i);
        DDefFloat(prefix, "length", "m", &input.objectsSizes[i * NUM_COORDS + 0U], DVA_None);
        DDefFloat(prefix, "width", "m", &input.objectsSizes[i * NUM_COORDS + 1U], DVA_None);
        DDefFloat(prefix, "height", "m", &input.objectsSizes[i * NUM_COORDS + 2U], DVA_None);
    }

    DDefaultDelete(prefix);
}

void SvcModelUtils::declQuantsGdrPointListInput(std::string prefixStr, gdr::GdrPointList& gdrPointsList)
{
    DDefULLong(NULL, (prefixStr + "timestamp").c_str(), "us", &gdrPointsList.sSigHeader.uiTimeStamp, DVA_None);
    DDefUChar(NULL, (prefixStr + "signalStatus").c_str(), "", reinterpret_cast<uint8*>(&gdrPointsList.sSigHeader.eSigStatus), DVA_None);
    DDefUChar(NULL, (prefixStr + "sensorSource").c_str(), "", reinterpret_cast<uint8*>(&gdrPointsList.sensorSource), DVA_None);
    DDefUInt(NULL, (prefixStr + "numPoints").c_str(), "", &gdrPointsList.numPoints, DVA_None);

    size_t const maxNumberOfPoints{ 600U };  // realistically reduced from 1000 because of IPGMovie limitations
    std::string pointListPrefixStr{ prefixStr + "pointList" };
    tDDefault* prefix{ DDefaultCreate(pointListPrefixStr.c_str()) };

    for (size_t i{ 0U }; i < maxNumberOfPoints; ++i) {
        DDefPrefix(prefix, (pointListPrefixStr + "[%d].").c_str(), i);
        DDefUInt(prefix, "id", "", &gdrPointsList.pointList[i].id, DVA_None);
        DDefFloat(prefix, "x_m", "m", &gdrPointsList.pointList[i].x_m, DVA_None);
        DDefFloat(prefix, "y_m", "m", &gdrPointsList.pointList[i].y_m, DVA_None);
        DDefFloat(prefix, "z_m", "m", &gdrPointsList.pointList[i].z_m, DVA_None);
        DDefFloat(prefix, "xStdDev_mm", "mm", &gdrPointsList.pointList[i].xStdDev_mm, DVA_None);
        DDefFloat(prefix, "yStdDev_mm", "mm", &gdrPointsList.pointList[i].yStdDev_mm, DVA_None);
        DDefFloat(prefix, "zStdDev_mm", "mm", &gdrPointsList.pointList[i].zStdDev_mm, DVA_None);
        DDefFloat(prefix, "confidence", "", &gdrPointsList.pointList[i].confidence, DVA_None);

    }

    DDefaultDelete(prefix);
}

void SvcModelUtils::declQuantsParkingLineListInput(std::string prefixStr, pmsd::ParkingLineList& parkingLineList)
{
    DDefULLong(NULL, (prefixStr + "timestamp").c_str(), "us", &parkingLineList.sSigHeader.uiTimeStamp, DVA_None);
    DDefUChar(NULL, (prefixStr + "signalStatus").c_str(), "", reinterpret_cast<uint8*>(&parkingLineList.sSigHeader.eSigStatus), DVA_None);
    DDefUShort(NULL, (prefixStr + "numberOfLines").c_str(), "", &parkingLineList.numberOfLines, DVA_None);

    size_t const maxNumberOfLines{ 100U };
    std::string parkingLinesPrefixStr{ prefixStr + "parkingLines" };
    tDDefault* prefix{ DDefaultCreate(parkingLinesPrefixStr.c_str()) };

    for (size_t i{ 0U }; i < maxNumberOfLines; ++i) {
        DDefPrefix(prefix, (parkingLinesPrefixStr + "[%d].").c_str(), i);
        DDefFloat(prefix, "startPoint.x", "m", &parkingLineList.parkingLines[i].startPoint.x, DVA_None);
        DDefFloat(prefix, "startPoint.y", "m", &parkingLineList.parkingLines[i].startPoint.y, DVA_None);
        DDefFloat(prefix, "endPoint.x", "m", &parkingLineList.parkingLines[i].endPoint.x, DVA_None);
        DDefFloat(prefix, "endPoint.y", "m", &parkingLineList.parkingLines[i].endPoint.y, DVA_None);
        DDefFloat(prefix, "confidence", "", &parkingLineList.parkingLines[i].confidence, DVA_None);
    }

    DDefaultDelete(prefix);
}

void SvcModelUtils::declQuantsWheelStopperListInput(std::string prefixStr, pmsd::WheelStopperList & wheelStopperList)
{
    DDefULLong(NULL, (prefixStr + "timestamp").c_str(), "us", &wheelStopperList.sSigHeader.uiTimeStamp, DVA_None);
    DDefUChar(NULL, (prefixStr + "signalStatus").c_str(), "", reinterpret_cast<uint8*>(&wheelStopperList.sSigHeader.eSigStatus), DVA_None);
    DDefUChar(NULL, (prefixStr + "numberOfWheelStoppers").c_str(), "", &wheelStopperList.numberOfWheelStoppers, DVA_None);

    size_t const maxNumberOfWhlStop{ 32U };
    std::string wheelStoppersPrefixStr{ prefixStr + "wheelStoppers" };
    tDDefault* prefix{ DDefaultCreate(wheelStoppersPrefixStr.c_str()) };

    for (size_t i{ 0U }; i < maxNumberOfWhlStop; ++i) {
        DDefPrefix(prefix, (wheelStoppersPrefixStr + "[%d].").c_str(), i);
        DDefFloat(prefix, "startPoint.x", "m", &wheelStopperList.wheelStoppers[i].startPoint.x, DVA_None);
        DDefFloat(prefix, "startPoint.y", "m", &wheelStopperList.wheelStoppers[i].startPoint.y, DVA_None);
        DDefFloat(prefix, "endPoint.x", "m", &wheelStopperList.wheelStoppers[i].endPoint.x, DVA_None);
        DDefFloat(prefix, "endPoint.y", "m", &wheelStopperList.wheelStoppers[i].endPoint.y, DVA_None);
        DDefFloat(prefix, "confidence", "", &wheelStopperList.wheelStoppers[i].confidence, DVA_None);
    }

    DDefaultDelete(prefix);
}

void SvcModelUtils::declQuantsWheelLockerListInput(std::string prefixStr, pmsd::WheelLockerList& wheelLockerList)
{ 
    DDefULLong(NULL, (prefixStr + "timestamp").c_str(), "us", &wheelLockerList.sSigHeader.uiTimeStamp, DVA_None);
    DDefUChar(NULL, (prefixStr + "signalStatus").c_str(), "", reinterpret_cast<uint8*>(&wheelLockerList.sSigHeader.eSigStatus), DVA_None);
    DDefUChar(NULL, (prefixStr + "numberOfWheelLockers").c_str(), "", &wheelLockerList.numberOfWheelLockers, DVA_None);

    size_t const maxNumWhlLock{ 32U };
    std::string whlLockPrefixStr{ prefixStr + "wheelLockers" };
    tDDefault* prefix{ DDefaultCreate(whlLockPrefixStr.c_str()) };

    for (size_t i{ 0U }; i < maxNumWhlLock; ++i) {
        DDefPrefix(prefix, (whlLockPrefixStr + "[%d].").c_str(), i);

        DDefFloat(prefix, "confidence", "", &wheelLockerList.wheelLockers[i].confidence, DVA_None);

        DDefFloat(prefix, "NDS.start.x", "m", &wheelLockerList.wheelLockers[i].nonDrivableStart.startPoint.x, DVA_None);
        DDefFloat(prefix, "NDS.start.y", "m", &wheelLockerList.wheelLockers[i].nonDrivableStart.startPoint.y, DVA_None);

        DDefFloat(prefix, "NDS.end.x", "m", &wheelLockerList.wheelLockers[i].nonDrivableStart.endPoint.x, DVA_None);
        DDefFloat(prefix, "NDS.end.y", "m", &wheelLockerList.wheelLockers[i].nonDrivableStart.endPoint.y, DVA_None);

        DDefFloat(prefix, "NDE.start.x", "m", &wheelLockerList.wheelLockers[i].nonDrivableEnd.startPoint.x, DVA_None);
        DDefFloat(prefix, "NDE.start.y", "m", &wheelLockerList.wheelLockers[i].nonDrivableEnd.startPoint.y, DVA_None);

        DDefFloat(prefix, "NDE.end.x", "m", &wheelLockerList.wheelLockers[i].nonDrivableEnd.endPoint.x, DVA_None);
        DDefFloat(prefix, "NDE.end.y", "m", &wheelLockerList.wheelLockers[i].nonDrivableEnd.endPoint.y, DVA_None);

        DDefFloat(prefix, "DRIV.start.x", "m", &wheelLockerList.wheelLockers[i].drivable.startPoint.x, DVA_None);
        DDefFloat(prefix, "DRIV.start.y", "m", &wheelLockerList.wheelLockers[i].drivable.startPoint.y, DVA_None);

        DDefFloat(prefix, "DRIV.end.x", "m", &wheelLockerList.wheelLockers[i].drivable.endPoint.x, DVA_None);
        DDefFloat(prefix, "DRIV.end.y", "m", &wheelLockerList.wheelLockers[i].drivable.endPoint.y, DVA_None);
    }

    DDefaultDelete(prefix);
}

void SvcModelUtils::declQuantsParkingSlotInput(std::string prefixStr, pmsd::ParkingSlotList& parkingSlotList)
{
    DDefULLong(NULL, (prefixStr + "timestamp").c_str(), "us", &parkingSlotList.sSigHeader.uiTimeStamp, DVA_None);
    DDefUChar(NULL, (prefixStr + "signalStatus").c_str(), "", reinterpret_cast<uint8*>(&parkingSlotList.sSigHeader.eSigStatus), DVA_None);
    DDefUChar(NULL, (prefixStr + "numberOfSlots").c_str(), "", &parkingSlotList.numberOfSlots, DVA_None);

    size_t const maxNumSlots{ 16U };
    size_t const numberOfCorners{ 4U };
    std::string parkingSlotsPrefixStr{ prefixStr + "parkingSlots" };
    tDDefault* prefix{ DDefaultCreate(parkingSlotsPrefixStr.c_str()) };

    for (size_t i{ 0U }; i < maxNumSlots; ++i) {
        DDefPrefix(prefix, (parkingSlotsPrefixStr + "[%d].").c_str(), i);

        DDefUChar(prefix, "type", "",
            reinterpret_cast<uint8*>(&parkingSlotList.parkingSlots[i].type), DVA_None);

        DDefFloat(prefix, "existenceProbability", "",
            &parkingSlotList.parkingSlots[i].existenceProbability, DVA_None);

        DDefFloat(prefix, "parkingScenarioConfidence.angled", "",
            &parkingSlotList.parkingSlots[i].parkingScenarioConfidence.angled, DVA_None);

        DDefFloat(prefix, "parkingScenarioConfidence.parallel", "",
            &parkingSlotList.parkingSlots[i].parkingScenarioConfidence.parallel, DVA_None);

        DDefFloat(prefix, "parkingScenarioConfidence.perpendicular", "",
            &parkingSlotList.parkingSlots[i].parkingScenarioConfidence.perpendicular, DVA_None);

        DDefFloat(prefix, "corner_0.x", "m", &parkingSlotList.parkingSlots[i].corner_0.x, DVA_None);
        DDefFloat(prefix, "corner_0.y", "m", &parkingSlotList.parkingSlots[i].corner_0.y, DVA_None);

        DDefFloat(prefix, "corner_1.x", "m", &parkingSlotList.parkingSlots[i].corner_1.x, DVA_None);
        DDefFloat(prefix, "corner_1.y", "m", &parkingSlotList.parkingSlots[i].corner_1.y, DVA_None);

        DDefFloat(prefix, "corner_2.x", "m", &parkingSlotList.parkingSlots[i].corner_2.x, DVA_None);
        DDefFloat(prefix, "corner_2.y", "m", &parkingSlotList.parkingSlots[i].corner_2.y, DVA_None);

        DDefFloat(prefix, "corner_3.x", "m", &parkingSlotList.parkingSlots[i].corner_3.x, DVA_None);
        DDefFloat(prefix, "corner_3.y", "m", &parkingSlotList.parkingSlots[i].corner_3.y, DVA_None);

        for (size_t ii{ 0U }; ii < numberOfCorners; ++ii) {
            DDefPrefix(prefix, (parkingSlotsPrefixStr + "[%d].cornerOcclusionState[%d]").c_str(), i, ii);
            DDefFloat(prefix, "", "", &parkingSlotList.parkingSlots[i].cornerOcclusionState[ii], DVA_None);
        }
    }

    DDefaultDelete(prefix);
}

void SvcModelUtils::declQuantsStopLinesInput(std::string prefixStr, pmsd::StopLineList& stopLineList)
{ 
    DDefULLong(NULL, (prefixStr + "timestamp").c_str(), "us", &stopLineList.sSigHeader.uiTimeStamp, DVA_None);
    DDefUChar(NULL, (prefixStr + "signalStatus").c_str(), "", reinterpret_cast<uint8*>(&stopLineList.sSigHeader.eSigStatus), DVA_None);
    DDefUChar(NULL, (prefixStr + "numberOfLines").c_str(), "", &stopLineList.numberOfLines, DVA_None);

    size_t const maxNumStopLines{ 16U };
    std::string stopLinesPrefixStr{ prefixStr + "stopLines" };
    tDDefault* prefix{ DDefaultCreate(stopLinesPrefixStr.c_str()) };

    for (size_t i{ 0U }; i < maxNumStopLines; ++i) {
        DDefPrefix(prefix, (stopLinesPrefixStr + "[%d].").c_str(), i);

        DDefFloat(prefix, "confidence", "", &stopLineList.stopLines[i].confidence, DVA_None);
        DDefFloat(prefix, "start.x", "m", &stopLineList.stopLines[i].startPoint.x, DVA_None);
        DDefFloat(prefix, "start.y", "m", &stopLineList.stopLines[i].startPoint.y, DVA_None);
        DDefFloat(prefix, "end.x", "m", &stopLineList.stopLines[i].endPoint.x, DVA_None);
        DDefFloat(prefix, "end.y", "m", &stopLineList.stopLines[i].endPoint.y, DVA_None);
    }

    DDefaultDelete(prefix);
}

void SvcModelUtils::declQuantsPedestrCrossInput(std::string prefixStr, pmsd::PedestrianCrossingList & pedCrossList)
{ 
    DDefULLong(NULL, (prefixStr + "timestamp").c_str(), "us", &pedCrossList.sSigHeader.uiTimeStamp, DVA_None);
    DDefUChar(NULL, (prefixStr + "signalStatus").c_str(), "", reinterpret_cast<uint8*>(&pedCrossList.sSigHeader.eSigStatus), DVA_None);
    DDefUChar(NULL, (prefixStr + "numberOfCrossings").c_str(), "", &pedCrossList.numberOfCrossings, DVA_None);

    size_t const maxNumPedCross{ 16U };
    std::string pedCrossPrefixStr{ prefixStr + "pedCrossings" };
    tDDefault* prefix{ DDefaultCreate(pedCrossPrefixStr.c_str()) };

    for (size_t i{ 0U }; i < maxNumPedCross; ++i) {
        DDefPrefix(prefix, (pedCrossPrefixStr + "[%d].").c_str(), i);

        DDefFloat(prefix, "confidence", "", &pedCrossList.pedestrianCrossings[i].confidence, DVA_None);

        DDefFloat(prefix, "bound_0.x", "m", &pedCrossList.pedestrianCrossings[i].boundaryPoints[0U].x, DVA_None);
        DDefFloat(prefix, "bound_0.y", "m", &pedCrossList.pedestrianCrossings[i].boundaryPoints[0U].y, DVA_None);

        DDefFloat(prefix, "bound_1.x", "m", &pedCrossList.pedestrianCrossings[i].boundaryPoints[1U].x, DVA_None);
        DDefFloat(prefix, "bound_1.y", "m", &pedCrossList.pedestrianCrossings[i].boundaryPoints[1U].y, DVA_None);

        DDefFloat(prefix, "bound_2.x", "m", &pedCrossList.pedestrianCrossings[i].boundaryPoints[2U].x, DVA_None);
        DDefFloat(prefix, "bound_2.y", "m", &pedCrossList.pedestrianCrossings[i].boundaryPoints[2U].y, DVA_None);

        DDefFloat(prefix, "bound_3.x", "m", &pedCrossList.pedestrianCrossings[i].boundaryPoints[3U].x, DVA_None);
        DDefFloat(prefix, "bound_3.y", "m", &pedCrossList.pedestrianCrossings[i].boundaryPoints[3U].y, DVA_None);
    }

    DDefaultDelete(prefix);
}

void SvcModelUtils::declQuantsDynamicObjectInputList(std::string prefixStr, tpp::DynamicObjectList_t& dynamicObjectList)
{
    DDefULLong(NULL, (prefixStr + "timestamp").c_str(), "us", &dynamicObjectList.sSigHeader.uiTimeStamp, DVA_None);
    DDefUChar(NULL, (prefixStr + "signalStatus").c_str(), "", reinterpret_cast<uint8*>(&dynamicObjectList.sSigHeader.eSigStatus), DVA_None);
    DDefUShort(NULL, (prefixStr + "numberOfBBoxObjects").c_str(), "", &dynamicObjectList.numberOfBBoxObjects, DVA_None);
    DDefUShort(NULL, (prefixStr + "numberOfCuboidObjects").c_str(), "", &dynamicObjectList.numberOfCuboidObjects, DVA_None);

    size_t const maxNumberOfObjects{ 32U };
    std::string cuboidObjectsPrefixStr{ prefixStr + "cuboidObjects" };
    tDDefault* cubPrefix{ DDefaultCreate(cuboidObjectsPrefixStr.c_str()) };

    for (size_t i{ 0U }; i < maxNumberOfObjects; ++i) {
        DDefPrefix(cubPrefix, (cuboidObjectsPrefixStr + "[%d].").c_str(), i);
        DDefUChar(cubPrefix, "subClassId", "", reinterpret_cast<uint8*>(&dynamicObjectList.cuboidObjects[i].subClassId), DVA_None);

        DDefFloat(cubPrefix, "confidence", "", &dynamicObjectList.cuboidObjects[i].confidence, DVA_None);

        DDefFloat(cubPrefix, "centerPoint.x", "m", &dynamicObjectList.cuboidObjects[i].centerPoint.x, DVA_None);
        DDefFloat(cubPrefix, "centerPoint.y", "m", &dynamicObjectList.cuboidObjects[i].centerPoint.y, DVA_None);
        DDefFloat(cubPrefix, "centerPoint.z", "m", &dynamicObjectList.cuboidObjects[i].centerPoint.z, DVA_None);

        DDefFloat(cubPrefix, "objectSize.length", "m", &dynamicObjectList.cuboidObjects[i].objectSize.length, DVA_None);
        DDefFloat(cubPrefix, "objectSize.width", "m", &dynamicObjectList.cuboidObjects[i].objectSize.width, DVA_None);
        DDefFloat(cubPrefix, "objectSize.height", "m", &dynamicObjectList.cuboidObjects[i].objectSize.height, DVA_None);

        DDefFloat(cubPrefix, "objectYaw", "rad", &dynamicObjectList.cuboidObjects[i].objectYaw, DVA_None);
    }

    DDefaultDelete(cubPrefix);

    std::string bBoxObjectsPrefixStr{ prefixStr + "bBoxObjects" };
    tDDefault* bBoxPrefix{ DDefaultCreate(bBoxObjectsPrefixStr.c_str()) };

    for (size_t i{ 0U }; i < maxNumberOfObjects; ++i) {
        DDefPrefix(bBoxPrefix, (bBoxObjectsPrefixStr + "[%d].").c_str(), i);
        DDefUChar(bBoxPrefix, "subClassId", "", reinterpret_cast<uint8*>(&dynamicObjectList.boundingBoxObjects[i].subClassId), DVA_None);

        DDefFloat(bBoxPrefix, "confidence", "", &dynamicObjectList.boundingBoxObjects[i].confidence, DVA_None);

        DDefFloat(bBoxPrefix, "centerPoint.x", "m", &dynamicObjectList.boundingBoxObjects[i].centerPoint.x, DVA_None);
        DDefFloat(bBoxPrefix, "centerPoint.y", "m", &dynamicObjectList.boundingBoxObjects[i].centerPoint.y, DVA_None);
        DDefFloat(bBoxPrefix, "centerPoint.z", "m", &dynamicObjectList.boundingBoxObjects[i].centerPoint.z, DVA_None);

        DDefFloat(bBoxPrefix, "objectSize.width", "m", &dynamicObjectList.boundingBoxObjects[i].objectSize.width, DVA_None);
        DDefFloat(bBoxPrefix, "objectSize.height", "m", &dynamicObjectList.boundingBoxObjects[i].objectSize.height, DVA_None);

        DDefFloat(bBoxPrefix, "objectYaw", "rad", &dynamicObjectList.boundingBoxObjects[i].objectYaw, DVA_None);
    }

    DDefaultDelete(bBoxPrefix);

}

void SvcModelUtils::declQuantsSppPolylineInput(std::string prefixStr, spp::SppPolylineList_t& sppPolylineList)
{
    DDefULLong(NULL, (prefixStr + "timestamp").c_str(), "us", &sppPolylineList.sSigHeader.uiTimeStamp, DVA_None);
    DDefUChar(NULL, (prefixStr + "signalStatus").c_str(), "", reinterpret_cast<uint8*>(&sppPolylineList.sSigHeader.eSigStatus), DVA_None);
    DDefUShort(NULL, (prefixStr + "numberOfPolylines").c_str(), "", &sppPolylineList.numberOfPolylines, DVA_None);
    DDefUChar(NULL, (prefixStr + "numberOfPolygons").c_str(), "", &sppPolylineList.numberOfPolygons, DVA_None);

    size_t const maxNumberOfPolylines{ 40U };  // realistically reduced from 512 because of an error in IPGMovie (too many subscribed variables)
    std::string sppPolyLinesPrefixStr{ prefixStr + "polylines" };
    tDDefault* poly_prefix{ DDefaultCreate(sppPolyLinesPrefixStr.c_str()) };

    for (size_t i{ 0U }; i < maxNumberOfPolylines; ++i) {
        DDefPrefix(poly_prefix, (sppPolyLinesPrefixStr + "[%d].").c_str(), i);

        DDefUShort(poly_prefix, "vertexStartIndex", "", &sppPolylineList.polylines[i].vertexStartIndex, DVA_None);
        DDefUShort(poly_prefix, "numVertices", "", &sppPolylineList.polylines[i].numVertices, DVA_None);
    }
    DDefaultDelete(poly_prefix);

    size_t const maxNumberOfVrts{ 400U };  // realistically reduced from 3072 because of an error in IPGMovie (too many subscribed variables)
    std::string sppPolyVrtxPrefixStr{ prefixStr + "vertices" };
    tDDefault* vrtx_prefix{ DDefaultCreate(sppPolyVrtxPrefixStr.c_str()) };

    for (size_t i{ 0U }; i < maxNumberOfVrts; ++i) {
        DDefPrefix(vrtx_prefix, (sppPolyVrtxPrefixStr + "[%d].").c_str(), i);

        DDefFloat(vrtx_prefix, "x_m", "m", &sppPolylineList.vertices[i].x_m, DVA_None);
        DDefFloat(vrtx_prefix, "y_m", "m", &sppPolylineList.vertices[i].y_m, DVA_None);
    }
    DDefaultDelete(vrtx_prefix);
}

void SvcModelUtils::declQuantsVizDebugValues(std::string prefixStr, svc_model_processing::DebugVizSide& _debugVizSide)
{
    DDefUInt(NULL, (prefixStr + "numDynBboxObj").c_str(), "", &_debugVizSide.numDynBboxObj, DVA_None);

    size_t const maxDynObj{ 10U };
    std::string dynObjPrefix{ prefixStr + "dynBboxObj" };
    tDDefault* dyn_prefix{ DDefaultCreate(dynObjPrefix.c_str()) };

    for (size_t i{ 0U }; i < maxDynObj; ++i) {
        DDefPrefix(dyn_prefix, (dynObjPrefix + "Centers[%d].").c_str(), i);
        DDefFloat(dyn_prefix, "center.x", "m", &_debugVizSide.dynBboxObjCenters[i].x, DVA_None);
        DDefFloat(dyn_prefix, "center.y", "m", &_debugVizSide.dynBboxObjCenters[i].y, DVA_None);

        DDefPrefix(dyn_prefix, (dynObjPrefix + "LenWidth[%d].").c_str(), i);
        DDefFloat(dyn_prefix, "size.length", "m", &_debugVizSide.dynBboxObjLenWidth[i].x, DVA_None);
        DDefFloat(dyn_prefix, "size.width", "m", &_debugVizSide.dynBboxObjLenWidth[i].y, DVA_None);
    }

    DDefaultDelete(dyn_prefix);
}

void SvcModelUtils::declQuantsSettingsValues(std::string prefixStr, svc_model_processing::SvcModelProcessingSettings& _settings)
{
    std::string const confPrefix{ prefixStr + ".trfObjConf" };
    DDefFloat(NULL, (confPrefix + ".spp_poly").c_str(), "", &_settings.sppPolyConf, DVA_IO_In);

    tDDefault* svcConfPrefix{ DDefaultCreate((confPrefix + "_.").c_str()) };

    for (uint32_t it{ 0U }; it < _settings.maxConfTrfObjCount; ++it) {
        DDefPrefix(svcConfPrefix, (confPrefix + "_%d.").c_str(), it);
        DDefFloat(svcConfPrefix, "generic", "", &_settings.genericConf[it], DVA_IO_In);
        DDefFloat(svcConfPrefix, "sem_points", "", &_settings.semPointsConf[it], DVA_IO_In);
        DDefFloat(svcConfPrefix, "car", "", &_settings.cubSubConf[it][0U], DVA_IO_In);
        DDefFloat(svcConfPrefix, "van", "", &_settings.cubSubConf[it][1U], DVA_IO_In);
        DDefFloat(svcConfPrefix, "truck", "", &_settings.cubSubConf[it][2U], DVA_IO_In);
        DDefFloat(svcConfPrefix, "pedestr", "", &_settings.bboxSubConf[it][0U], DVA_IO_In);
        DDefFloat(svcConfPrefix, "2whl", "", &_settings.bboxSubConf[it][1U], DVA_IO_In);
        DDefFloat(svcConfPrefix, "shopcart", "", &_settings.bboxSubConf[it][2U], DVA_IO_In);
        DDefFloat(svcConfPrefix, "animal", "", &_settings.bboxSubConf[it][3U], DVA_IO_In);
    }

    DDefaultDelete(svcConfPrefix);

    DDefChar(NULL, (prefixStr + ".gdr.front.sigStatus").c_str(), "", &_settings.ports.gdr[0U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".gdr.rear.sigStatus").c_str(), "", &_settings.ports.gdr[1U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".gdr.left.sigStatus").c_str(), "", &_settings.ports.gdr[2U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".gdr.right.sigStatus").c_str(), "", &_settings.ports.gdr[3U].sigStatus, DVA_IO_In);

    DDefChar(NULL, (prefixStr + ".pmd.front.sigStatus").c_str(), "", &_settings.ports.pmd[0U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".pmd.rear.sigStatus").c_str(), "", &_settings.ports.pmd[1U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".pmd.left.sigStatus").c_str(), "", &_settings.ports.pmd[2U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".pmd.right.sigStatus").c_str(), "", &_settings.ports.pmd[3U].sigStatus, DVA_IO_In);

    DDefChar(NULL, (prefixStr + ".wh_stop.front.sigStatus").c_str(), "", &_settings.ports.wh_stop[0U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".wh_stop.rear.sigStatus").c_str(), "", &_settings.ports.wh_stop[1U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".wh_stop.left.sigStatus").c_str(), "", &_settings.ports.wh_stop[2U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".wh_stop.right.sigStatus").c_str(), "", &_settings.ports.wh_stop[3U].sigStatus, DVA_IO_In);

    DDefChar(NULL, (prefixStr + ".wh_lock.front.sigStatus").c_str(), "", &_settings.ports.wh_lock[0U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".wh_lock.rear.sigStatus").c_str(), "", &_settings.ports.wh_lock[1U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".wh_lock.left.sigStatus").c_str(), "", &_settings.ports.wh_lock[2U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".wh_lock.right.sigStatus").c_str(), "", &_settings.ports.wh_lock[3U].sigStatus, DVA_IO_In);

    DDefChar(NULL, (prefixStr + ".od_slots.front.sigStatus").c_str(), "", &_settings.ports.od_slots[0U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".od_slots.rear.sigStatus").c_str(), "", &_settings.ports.od_slots[1U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".od_slots.left.sigStatus").c_str(), "", &_settings.ports.od_slots[2U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".od_slots.right.sigStatus").c_str(), "", &_settings.ports.od_slots[3U].sigStatus, DVA_IO_In);

    DDefChar(NULL, (prefixStr + ".stop_lines.front.sigStatus").c_str(), "", &_settings.ports.stop_lines[0U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".stop_lines.rear.sigStatus").c_str(), "", &_settings.ports.stop_lines[1U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".stop_lines.left.sigStatus").c_str(), "", &_settings.ports.stop_lines[2U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".stop_lines.right.sigStatus").c_str(), "", &_settings.ports.stop_lines[3U].sigStatus, DVA_IO_In);

    DDefChar(NULL, (prefixStr + ".ped_cross.front.sigStatus").c_str(), "", &_settings.ports.ped_cross[0U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".ped_cross.rear.sigStatus").c_str(), "", &_settings.ports.ped_cross[1U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".ped_cross.left.sigStatus").c_str(), "", &_settings.ports.ped_cross[2U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".ped_cross.right.sigStatus").c_str(), "", &_settings.ports.ped_cross[3U].sigStatus, DVA_IO_In);

    DDefChar(NULL, (prefixStr + ".dyn.front.sigStatus").c_str(), "", &_settings.ports.dyn[0U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".dyn.rear.sigStatus").c_str(), "", &_settings.ports.dyn[1U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".dyn.left.sigStatus").c_str(), "", &_settings.ports.dyn[2U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".dyn.right.sigStatus").c_str(), "", &_settings.ports.dyn[3U].sigStatus, DVA_IO_In);

    DDefChar(NULL, (prefixStr + ".sem_points.front.sigStatus").c_str(), "", &_settings.ports.sem_points[0U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".sem_points.rear.sigStatus").c_str(), "", &_settings.ports.sem_points[1U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".sem_points.left.sigStatus").c_str(), "", &_settings.ports.sem_points[2U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".sem_points.right.sigStatus").c_str(), "", &_settings.ports.sem_points[3U].sigStatus, DVA_IO_In);

    DDefChar(NULL, (prefixStr + ".spp_poly.front.sigStatus").c_str(), "", &_settings.ports.spp_poly[0U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".spp_poly.rear.sigStatus").c_str(), "", &_settings.ports.spp_poly[1U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".spp_poly.left.sigStatus").c_str(), "", &_settings.ports.spp_poly[2U].sigStatus, DVA_IO_In);
    DDefChar(NULL, (prefixStr + ".spp_poly.right.sigStatus").c_str(), "", &_settings.ports.spp_poly[3U].sigStatus, DVA_IO_In);
}


void SvcModelUtils::declQuants(svc_model_processing::SvcModelProcessingInput& input, svc_model_processing::SvcModelProcessingOutput& output, svc_model_processing::SvcModelProcessingSettings& settings)
{
    declQuantsSvcModelProcessingInput("AP.svcModelProcessingInput.", input);

    declQuantsGdrPointListInput("AP.svcModelProcessingOutput.data.gdrPointsListFront.", output.data->gdrPointsListFront);
    declQuantsGdrPointListInput("AP.svcModelProcessingOutput.data.gdrPointsListRear.", output.data->gdrPointsListRear);
    declQuantsGdrPointListInput("AP.svcModelProcessingOutput.data.gdrPointsListLeft.", output.data->gdrPointsListLeft);
    declQuantsGdrPointListInput("AP.svcModelProcessingOutput.data.gdrPointsListRight.", output.data->gdrPointsListRight);

    declQuantsParkingLineListInput("AP.svcModelProcessingOutput.data.parkingLineListFront.", output.data->parkingLineListFront);
    declQuantsParkingLineListInput("AP.svcModelProcessingOutput.data.parkingLineListRear.", output.data->parkingLineListRear);
    declQuantsParkingLineListInput("AP.svcModelProcessingOutput.data.parkingLineListLeft.", output.data->parkingLineListLeft);
    declQuantsParkingLineListInput("AP.svcModelProcessingOutput.data.parkingLineListRight.", output.data->parkingLineListRight);

    declQuantsWheelStopperListInput("AP.svcModelProcessingOutput.data.wheelStopperListFront.", output.data->wheelStopperListFront);
    declQuantsWheelStopperListInput("AP.svcModelProcessingOutput.data.wheelStopperListRear.", output.data->wheelStopperListRear);
    declQuantsWheelStopperListInput("AP.svcModelProcessingOutput.data.wheelStopperListLeft.", output.data->wheelStopperListLeft);
    declQuantsWheelStopperListInput("AP.svcModelProcessingOutput.data.wheelStopperListRight.", output.data->wheelStopperListRight);

    declQuantsWheelLockerListInput("AP.svcModelProcessingOutput.data.wheelLockerListFront.", output.data->wheelLockerListFront);
    declQuantsWheelLockerListInput("AP.svcModelProcessingOutput.data.wheelLockerListRear.", output.data->wheelLockerListRear);
    declQuantsWheelLockerListInput("AP.svcModelProcessingOutput.data.wheelLockerListLeft.", output.data->wheelLockerListLeft);
    declQuantsWheelLockerListInput("AP.svcModelProcessingOutput.data.wheelLockerListRight.", output.data->wheelLockerListRight);

    declQuantsParkingSlotInput("AP.svcModelProcessingOutput.data.parkingSlotListFront.", output.data->parkingSlotListFront);
    declQuantsParkingSlotInput("AP.svcModelProcessingOutput.data.parkingSlotListRear.", output.data->parkingSlotListRear);
    declQuantsParkingSlotInput("AP.svcModelProcessingOutput.data.parkingSlotListLeft.", output.data->parkingSlotListLeft);
    declQuantsParkingSlotInput("AP.svcModelProcessingOutput.data.parkingSlotListRight.", output.data->parkingSlotListRight);

    declQuantsStopLinesInput("AP.svcModelProcessingOutput.data.stopLineListFront.", output.data->stopLineListFront);
    declQuantsStopLinesInput("AP.svcModelProcessingOutput.data.stopLineListRear.", output.data->stopLineListRear);
    declQuantsStopLinesInput("AP.svcModelProcessingOutput.data.stopLineListLeft.", output.data->stopLineListLeft);
    declQuantsStopLinesInput("AP.svcModelProcessingOutput.data.stopLineListRight.", output.data->stopLineListRight);

    declQuantsPedestrCrossInput("AP.svcModelProcessingOutput.data.pedestrCrossListFront.", output.data->pedestrCrossListFront);
    declQuantsPedestrCrossInput("AP.svcModelProcessingOutput.data.pedestrCrossListRear.", output.data->pedestrCrossListRear);
    declQuantsPedestrCrossInput("AP.svcModelProcessingOutput.data.pedestrCrossListLeft.", output.data->pedestrCrossListLeft);
    declQuantsPedestrCrossInput("AP.svcModelProcessingOutput.data.pedestrCrossListRight.", output.data->pedestrCrossListRight);

    declQuantsDynamicObjectInputList("AP.svcModelProcessingOutput.data.dynamicObjectsListFront.", output.data->dynamicObjectsListFront);
    declQuantsDynamicObjectInputList("AP.svcModelProcessingOutput.data.dynamicObjectsListRear.", output.data->dynamicObjectsListRear);
    declQuantsDynamicObjectInputList("AP.svcModelProcessingOutput.data.dynamicObjectsListLeft.", output.data->dynamicObjectsListLeft);
    declQuantsDynamicObjectInputList("AP.svcModelProcessingOutput.data.dynamicObjectsListRight.",output.data->dynamicObjectsListRight);

    declQuantsSppPolylineInput("AP.svcModelProcessingOutput.data.sppPolylineListFront.", output.data->sppPolylineListFront);
    declQuantsSppPolylineInput("AP.svcModelProcessingOutput.data.sppPolylineListRear.", output.data->sppPolylineListRear);
    declQuantsSppPolylineInput("AP.svcModelProcessingOutput.data.sppPolylineListLeft.", output.data->sppPolylineListLeft);
    declQuantsSppPolylineInput("AP.svcModelProcessingOutput.data.sppPolylineListRight.", output.data->sppPolylineListRight);

    declQuantsVizDebugValues("AP.svcModelProcessingOutput.debugViz.debugVizFront.", output.debugViz->debugVizFront);
    declQuantsVizDebugValues("AP.svcModelProcessingOutput.debugViz.debugVizRear.", output.debugViz->debugVizRear);
    declQuantsVizDebugValues("AP.svcModelProcessingOutput.debugViz.debugVizLeft.", output.debugViz->debugVizLeft);
    declQuantsVizDebugValues("AP.svcModelProcessingOutput.debugViz.debugVizRight.", output.debugViz->debugVizRight);

    declQuantsSettingsValues("AP.svcModelProcessingSettings", settings);
}
