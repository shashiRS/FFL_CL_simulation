#include "SiHighWrapper.h"
#include "CEM_disp.hpp"

#include "SiUtility.h" // getSiParameters()
#include <si_core/SiInterface.h> // new entry point of SI
#include <si/si_generated_types.h> // SI::MarkingsPortHack
#include <mf_memory_parking/MemParkParamReader.h>
#include <mf_mempark/mf_mempark_generated_types.h>
#include <relocalization/relocalization_generated_types.h>
#include <ap_commonvehsigprovider/odo_gps_port.h>

// mf_memory_parking currently not used in Entry
#include <relocalization/relocalizationModule_interface.h>//Temp solution only!

#include <geoml/CoordinateTransformer2D.h>

#include <functional>
#include "MfSilTypes.h"

static void logWarningOrError(const com::ComResult callResult, const char* message) {
    if (com::ComResult::COMRES_WARNING == callResult) {
        PLP_LOG_ADDMESSAGE(logger::LogLevel::LOGGER_WARNING, message);
    }
    else if (com::ComResult::COMRES_ERROR == callResult) {
        PLP_LOG_ADDMESSAGE(logger::LogLevel::LOGGER_ERROR, message);
    }
}

static aupdf::HeightConfidences mapToElementHeightConfidences(const si::StaticObjHeigthType objHeightType) {
    aupdf::HeightConfidences heightConfidence{};
    switch (objHeightType) {
    case si::StaticObjHeigthType::SO_HI_WHEEL_TRAVERSABLE:
        heightConfidence.wheelTraversable = 1.0F;
        break;
    case si::StaticObjHeigthType::SO_HI_BODY_TRAVERSABLE:
        heightConfidence.bodyTraversable = 1.0F;
        break;
    case si::StaticObjHeigthType::SO_HI_DOOR_OPENABLE:
    case si::StaticObjHeigthType::SO_HI_LOWER_BUMPER_HEIGHT:
    case si::StaticObjHeigthType::SO_HI_HIGH_OBSTACLE:
        heightConfidence.high = 1.0F;
        break;
    case si::StaticObjHeigthType::SO_HI_HANGING_OBJECT:
        heightConfidence.hanging = 1.0F;
        break;
    default: //si::StaticObjHeigthType::SO_HI_UNKNOWN
        heightConfidence.high = 1.0F;
    }
    return heightConfidence;
}

static aupdf::SemanticType mapToElementSemantic(const si::StaticObjHeigthType objHeightType
#ifdef VARIANT_PERFORMANCE
    , const si::StaticObjectClass classification
#endif
) {
#ifdef VARIANT_PERFORMANCE
    if (classification == si::StaticObjectClass::STAT_OBJ_WHEEL_STOPPER)
    {
        return aupdf::SemanticType::WHEELSTOPPER_TYPE;
    }
    else
#endif
    {
        switch (objHeightType) {
        case si::StaticObjHeigthType::SO_HI_WHEEL_TRAVERSABLE:
        case si::StaticObjHeigthType::SO_HI_BODY_TRAVERSABLE:
        case si::StaticObjHeigthType::SO_HI_DOOR_OPENABLE:
        case si::StaticObjHeigthType::SO_HI_LOWER_BUMPER_HEIGHT:
            return aupdf::SemanticType::CURB_TYPE;
        case si::StaticObjHeigthType::SO_HI_HIGH_OBSTACLE:
            return aupdf::SemanticType::STATIC_TYPE;
        default:
            return aupdf::SemanticType::UNKNOWN_TYPE;
        }
    }
}

static void calculateSgfHulls(const std::vector<VCEM::ConvexHull> &convexHulls, const uint64_t &timestamp_us,
    aupdf::SgfOutput &sgfOutput)
{
    if (convexHulls.size() > aupdf::Constants::SGF_MAX_OBJECTS) {
        PLP_LOG_ADDMESSAGE(logger::LogLevel::LOGGER_ERROR,
            "SiHighWrapper:calculateSgfHulls() - too many SGF elements");
    }

    sgfOutput.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
    sgfOutput.sSigHeader.uiTimeStamp = timestamp_us;
    sgfOutput.staticObjectsOutput.numberOfObjects = static_cast<uint8>(convexHulls.size());

    size_t numUsedVertices = 0;
    size_t vertexCount = 0;
    for (size_t iElem = 0; iElem < std::min(convexHulls.size(), static_cast<size_t>(aupdf::Constants::SGF_MAX_OBJECTS)); ++iElem) {
        const auto& hull = convexHulls[iElem];
        const bool isDynamicObject{ hull.objectType == VCEM::CemObjectType::DYNAMIC_OBJECT };
        const bool isStationaryOrStopped{ (hull.dynamicObjectProperty_nu.dynamicProperty == VCEM::DynamicProperty::STATIONARY) || (hull.dynamicObjectProperty_nu.dynamicProperty == VCEM::DynamicProperty::STOPPED) };
        if ((!isDynamicObject) || isStationaryOrStopped)
        {
            //Append to element
            aupdf::StaticObject &element = sgfOutput.staticObjectsOutput.objects[iElem];
            element.obstacleId = hull.u_id; //This is being used when mapping the static and dynamic objects together
            element.associatedDynamicObjectId = std::numeric_limits<uint32_t>::max();
            element.usedVertices = static_cast<uint8>(hull.polygon.size());
            element.vertexStartIndex = static_cast<uint16>(numUsedVertices);
            element.heightConfidences = mapToElementHeightConfidences(hull.objHeightClass_nu);
            element.existenceProbability = 1.0F;
            element.semanticType = mapToElementSemantic(hull.objHeightClass_nu
#ifdef VARIANT_PERFORMANCE
                , hull.classification
#endif
            );
            numUsedVertices += element.usedVertices;

            //Append to vertices
            for (const cml::Vec2Df &vertex : hull.polygon)
            {
                sgfOutput.staticObjectVerticesOutput.vertices[vertexCount] = { vertex.x(), vertex.y() };
                ++vertexCount;
            }
        }
    }

    if (numUsedVertices > aupdf::Constants::SGF_MAX_TOTAL_VERTICES) {
        PLP_LOG_ADDMESSAGE(logger::LogLevel::LOGGER_ERROR,
            "SiHighWrapper:calculateSgfHulls() - too many SGF vertices");
    }
    sgfOutput.staticObjectVerticesOutput.numberOfVertices = static_cast<uint16>(numUsedVertices);
}

static void calculatePclOutput(
    const std::vector<VCEM::PclDelimiter> &pclDelimiters, const uint64_t &timestamp_us, aupdf::PclOutput &pclOutput)
{
    if (pclDelimiters.size() > aupdf::Constants::PFS_MAX_PCL_DELIMITERS) {
        PLP_LOG_ADDMESSAGE(logger::LogLevel::LOGGER_ERROR,
            "SiHighWrapper:calculatePclOutput() - too many PCL delimiters");
    }
    pclOutput.numberOfDelimiters = static_cast<uint16>(pclDelimiters.size());
    pclOutput.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
    pclOutput.sSigHeader.uiTimeStamp = timestamp_us;

    for (size_t i = 0; i < pclDelimiters.size(); ++i) {
        pclOutput.delimiters[i].id = pclDelimiters[i].u_id;
        pclOutput.delimiters[i].startPointXPosition = pclDelimiters[i].startPoint.x();
        pclOutput.delimiters[i].startPointYPosition = pclDelimiters[i].startPoint.y();
        pclOutput.delimiters[i].endPointXPosition = pclDelimiters[i].endPoint.x();
        pclOutput.delimiters[i].endPointYPosition = pclDelimiters[i].endPoint.y();
        pclOutput.delimiters[i].confidence = 1.0F; // range from 0.0 to 1.0.
    }
}

static void calculateCnnParkingSpaces(
    const std::vector<VCEM::ODSlot> &odSlots, const uint64_t &timestamp_us, aupdf::ParkingSlotDetectionOutput &cnnParkingSpaces)
{
    cnnParkingSpaces.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
    cnnParkingSpaces.sSigHeader.uiTimeStamp = timestamp_us;
    cnnParkingSpaces.numberOfSlots = static_cast<uint8>(std::min(odSlots.size(), static_cast<size_t>(aupdf::Constants::PFS_PSD_MAX_PARKING_SLOTS)));
    for (uint8 i = 0; i < cnnParkingSpaces.numberOfSlots; ++i) {
        cnnParkingSpaces.parkingSlots[i].slotId = odSlots[i].slotId;
        cnnParkingSpaces.parkingSlots[i].existenceProbability = static_cast<float32_t>(odSlots[i].existence_probability) / 100.0F;
        cnnParkingSpaces.parkingSlots[i].parkingScenarioConfidence.angled = static_cast<float32_t>(odSlots[i].parking_scenario_confidence.angled) / 100.0F;
        cnnParkingSpaces.parkingSlots[i].parkingScenarioConfidence.parallel = static_cast<float32_t>(odSlots[i].parking_scenario_confidence.parallel) / 100.0F;
        cnnParkingSpaces.parkingSlots[i].parkingScenarioConfidence.perpendicular = static_cast<float32_t>(odSlots[i].parking_scenario_confidence.perpendicular) / 100.0F;
        for (int16_t j = 0; j < 4; j++) {
            cnnParkingSpaces.parkingSlots[i].slotCorners[j].x = odSlots[i].slot_corners[j].x();
            cnnParkingSpaces.parkingSlots[i].slotCorners[j].y = odSlots[i].slot_corners[j].y();
        }
    }
}

void mappingDynamicObject(
    const std::vector<VCEM::ConvexHull> &convexHulls,
    const uint64_t &timestamp_us,
    aupdf::DynamicEnvironment &dynEnvironment)
{
    dynEnvironment.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
    dynEnvironment.sSigHeader.uiTimeStamp = timestamp_us;
    dynEnvironment.numberOfObjects = 0U;
    for (const auto& convexHull : convexHulls)
    {
        if( VCEM::CemObjectType::DYNAMIC_OBJECT == convexHull.objectType )
        {
            if ((convexHull.dynamicObjectProperty_nu.state == VCEM::MeasurementObjState::MEASURED)
                || (convexHull.dynamicObjectProperty_nu.state == VCEM::MeasurementObjState::PREDICTED))
            {
                aupdf::DynamicObject &dynObject = dynEnvironment.objects[dynEnvironment.numberOfObjects];
                dynObject.id = convexHull.u_id; //This will be used in the linking of static and dynamic objects
                switch (convexHull.dynamicObjectProperty_nu.objectClass) {
                case VCEM::DynObjClass::CAR:
                    dynObject.objectClass = aupdf::DynamicObjectClassType::CAR;
                    break;
                case VCEM::DynObjClass::PEDESTRIAN:
                    dynObject.objectClass = aupdf::DynamicObjectClassType::PEDESTRIAN;
                    break;
                case VCEM::DynObjClass::MOTORCYCLE:
                case VCEM::DynObjClass::BICYCLE:
                    dynObject.objectClass = aupdf::DynamicObjectClassType::TWOWHEELER;
                    break;
                case VCEM::DynObjClass::CLASS_UNKNOWN:
                    dynObject.objectClass = aupdf::DynamicObjectClassType::UNKNOWN;
                    break;
                }
                dynObject.classProbability = 1.0F;
                switch (convexHull.dynamicObjectProperty_nu.dynamicProperty) {
                case VCEM::DynamicProperty::DYN_PROP_UNKNOWN:
                    dynObject.dynamicProperty = aupdf::DynamicProperty::UNKNOWN;
                    break;
                case VCEM::DynamicProperty::MOVING:
                    dynObject.dynamicProperty = aupdf::DynamicProperty::MOVING;
                    break;
                case VCEM::DynamicProperty::STATIONARY:
                case VCEM::DynamicProperty::STOPPED:
                    dynObject.dynamicProperty = aupdf::DynamicProperty::STATIONARY;
                    break;
                }
                dynObject.lifetime = convexHull.dynamicObjectProperty_nu.lifetime;
                switch (convexHull.dynamicObjectProperty_nu.state) {
                case VCEM::MeasurementObjState::MEASURED:
                    dynObject.state = aupdf::MaintenanceState::MEASURED;
                    break;
                case VCEM::MeasurementObjState::PREDICTED:
                    dynObject.state = aupdf::MaintenanceState::PREDICTED;
                    break;
                }
                dynObject.existenceCertainty = 1.0F;
                dynObject.orientation = convexHull.dynamicObjectProperty_nu.orientation;
                dynObject.orientationStandardDeviation = 0.0F;
                dynObject.velocity = { convexHull.dynamicObjectProperty_nu.velocity.x(), convexHull.dynamicObjectProperty_nu.velocity.y() };
                dynObject.velocityStandardDeviation = { 0.0F, 0.0F };
                dynObject.acceleration = { convexHull.dynamicObjectProperty_nu.acceleration.x(), convexHull.dynamicObjectProperty_nu.acceleration.y() };
                dynObject.accelerationStandardDeviation = { 0.0F, 0.0F };
                dynObject.yawRate = convexHull.dynamicObjectProperty_nu.yawRate;
                dynObject.yawRateStandardDeviation = 0.0F;
                for (uint8_t j = 0U; j < aupdf::Constants::TPF_NUMBER_OF_SHAPEPOINTS; ++j) {
                    dynObject.shapePoints.points[j].position = { convexHull.dynamicObjectProperty_nu.points[j].x(), convexHull.dynamicObjectProperty_nu.points[j].y() };
                    dynObject.shapePoints.points[j].varianceX = 0.0F;
                    dynObject.shapePoints.points[j].varianceY = 0.0F;
                    dynObject.shapePoints.points[j].covarianceXY = 0.0F;
                }
                dynObject.shapePoints.referencePoint = { convexHull.dynamicObjectProperty_nu.referencePoint.x(), convexHull.dynamicObjectProperty_nu.referencePoint.y() };

                ++dynEnvironment.numberOfObjects;
            }
        }
    }
}

void SiHighWrapper::initSiAlgorithm(const ap_common::Vehicle_Params& vehicleParams)
{
    mMetaMapStorage = {};

    const static si::SiConfig siConfiguration{ &vehicleParams, &SiUtility::getSiParameters(true) };
    const auto result = si::SiInterface::getInstance().init(siConfiguration);
    logWarningOrError(result, "SiHighWrapper:initSiAlgorithm() - SiInterface.init() failed");

    //load config
    const static relocalization::RelocalizationModuleConfig relocParams{ &SiUtility::getMemParkParameters(true) };
    const static mf_mempark::MF_MemoryParkingConfig memParkParams{ &SiUtility::getMemParkParameters(true) };

    const auto result2 = relocalization::RelocalizationModule_interface::getInstance().init(relocParams);
    const auto result3 = mf_mempark::mf_memory_parking_interface::getInstance().init(memParkParams);
}

void SiHighWrapper::resetSiAlgorithm() {
    mMetaMapStorage = {};

    const auto result  = si::SiInterface::getInstance().reset();
    const auto result2 = relocalization::RelocalizationModule_interface::getInstance().reset();
    const auto result3 = mf_mempark::mf_memory_parking_interface::getInstance().reset();
    logWarningOrError(result, "SiHighWrapper:resetSiAlgorithm() - SiInterface.reset() failed");
    // reset coordinate transformation
    mCoordinateTransformer.reset();
}

void SiHighWrapper::runSiAlgorithm(const uint64_t &timestamp_us,
    const ap_psm::SlotCtrlPort &slotControlPort,
    const lsm_vedodo::OdoEstimationOutputPort &odoEstimationPort,
    const std::vector<VCEM::ConvexHull> &convexHulls,
    const std::vector<VCEM::PclDelimiter> &pclDelimiters,
    const std::vector<VCEM::ODSlot> &odSlots,
    const mf_hmih::UserDefinedSlotPort &userDefinedSlotPort,
    const ap_tp::TargetPosesPort &targetPosesPort,
    const ap_commonvehsigprovider::OdoGpsPort gpsPort,
    const uint8 environmentModelActive,
    si::ApEnvModelPort &environmentModel,
    si::ApParkingBoxPort &parkingBoxes,
    si::CollEnvModelPort &collisionEnvironmentModel,
    si::PerceptionAvailabilityPort &perceptionAvailability,
    si::EgoMotionPort &egoMotion,
    mf_mempark::MemoryParkingStatusPort& memoryParkingStatusPort
#if defined(USE_ENV_PLOTTER)
    , si::PlotData &plotDataPort
#endif
    )
{
    static aupdf::SgfOutput sgfHulls{ eco::create_default<aupdf::SgfOutput>() };
    static aupdf::PclOutput pclOutput{ eco::create_default<aupdf::PclOutput>() };
    static aupdf::ParkingSlotDetectionOutput cnnBasedParkingSpacesInput{ eco::create_default<aupdf::ParkingSlotDetectionOutput>() };
    static aupdf::DynamicEnvironment dynamicEnvironmentInput{ eco::create_default<aupdf::DynamicEnvironment>() };

    if (environmentModelActive) {
        sgfHulls = *sgf_output_subscriber;
        pclOutput = *pcl_output_subscriber;
        cnnBasedParkingSpacesInput = *parking_slot_detection_output_subscriber;
        dynamicEnvironmentInput = *dynamic_environment_subscriber;
        mVehiclePoseEstAtCemTime = *ego_motion_at_cem_output_subscriber;
    }
    else {
        calculateSgfHulls(convexHulls, timestamp_us, sgfHulls);// set sgfHulls
        calculatePclOutput(pclDelimiters, timestamp_us, pclOutput); // set pclOutput
        calculateCnnParkingSpaces(odSlots, timestamp_us, cnnBasedParkingSpacesInput); // set cnnBasedParkingSpacesInput
        mappingDynamicObject(convexHulls, timestamp_us, dynamicEnvironmentInput); // mapping dynamic objects
        //Linking of static and dynamic objects
        linkStaticAndDynamicObjects(sgfHulls, dynamicEnvironmentInput);
        convertLsmVedodoDataToCemEgoData(mVehiclePoseEstAtCemTime, odoEstimationPort.odoEstimation);
    }

    //Hack MemPark Component
    static relocalization::RelocalizationResultPort relocalizationResultPort{};
    static relocalization::MapStatusPort mapStatusPort{};
    static relocalization::RecordingResultPort recordingResultPort{};
    static relocalization::DeletionResultPort deletionResultPort{};
    relocalizationResultPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
    relocalizationResultPort.sSigHeader.uiTimeStamp = timestamp_us;
    deletionResultPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
    deletionResultPort.sSigHeader.uiTimeStamp = timestamp_us;

    //Memory Parking Module
    mf_mempark::MF_MemoryParkingInput memParkInput{};
    memParkInput.odometryPort               = &odoEstimationPort;
    memParkInput.slotCtrlPort               = &slotControlPort;
    memParkInput.targetPosesPort            = &targetPosesPort;
    memParkInput.relocalizationResultPort   = &relocalizationResultPort;
    memParkInput.mapStatusPort              = &mapStatusPort;
    memParkInput.recordingResultPort        = &recordingResultPort;
    memParkInput.deletionResultPort         = &deletionResultPort;
    memParkInput.sgfData                    = &sgfHulls;
    memParkInput.mapMetaData                = &mMetaMapStorage;
    memParkInput.gpsData                    = &gpsPort;

    mf_mempark::MF_MemoryParkingOutput memParkOutput{};
    memParkOutput.memoryParkingStatusPort   = &memoryParkingStatusPort;
    static mf_mempark::RecordingRequestPort recordingRequest{};
    memParkOutput.recordingRequest          = &recordingRequest;
    static mf_mempark::RelocalizationRequestPort relocalizationRequestPort{};
    memParkOutput.relocalizationRequestPort = &relocalizationRequestPort;
    static mf_mempark::SystemDefinedPosePort systemDefinedPosePort{};
    memParkOutput.systemDefinedPosePort     = &systemDefinedPosePort;
    memParkOutput.mapMetaData               = &mMetaMapStorage;

    mf_mempark::mf_memory_parking_interface::getInstance().run(memParkInput, memParkOutput);

    //Re-localization module temp
    relocalization::RelocalizationModuleInput cemRelocInput{};
    cemRelocInput.lineData                  = &pclOutput;
    cemRelocInput.recordingRequest          = &recordingRequest;
    cemRelocInput.relocalizationRequestPort = &relocalizationRequestPort;
    cemRelocInput.sgfData                   = &sgfHulls;
    cemRelocInput.odometryPort = &odoEstimationPort;

    relocalization::RelocalizationModuleOutput cemRelocOutput{};
    cemRelocOutput.relocalizationResult     = &relocalizationResultPort;
    cemRelocOutput.mapStatusPort            = &mapStatusPort;
    cemRelocOutput.recordingResultPort      = &recordingResultPort;
    cemRelocOutput.deletionResultPort       = &deletionResultPort;

    relocalization::RelocalizationModule_interface::getInstance().run(cemRelocInput, cemRelocOutput);

    // set SiInput and SiOutput and run SI component
    si::SiInput siInput{};
    siInput.sgfData                 = &sgfHulls;
    siInput.pclData                 = &pclOutput;
    siInput.cnnBasedParkingSpaces   = &cnnBasedParkingSpacesInput;
    siInput.dynamicEnvironment      = &dynamicEnvironmentInput;
    siInput.vehiclePoseEstAtCemTime = &mVehiclePoseEstAtCemTime;
    siInput.slotCtrlPort            = &slotControlPort;
    siInput.odoEstimationOutputPort = &odoEstimationPort;
    siInput.systemDefinedPosePort   = &systemDefinedPosePort;
    siInput.userDefinedSlotPort     = &userDefinedSlotPort;

    si::SiOutput siOutput{};
    siOutput.environmentModel          = &environmentModel;
    siOutput.parkingBoxes              = &parkingBoxes;
    siOutput.collisionEnvironmentModel = &collisionEnvironmentModel;
    siOutput.perceptionAvailability    = &perceptionAvailability;
    siOutput.egoMotion                 = &egoMotion;
#if defined(USE_ENV_PLOTTER)
    siOutput.plotData                  = &plotDataPort;
#endif
    const auto result = si::SiInterface::getInstance().run(siInput, siOutput);
    logWarningOrError(result, "SiHighWrapper:runSiAlgorithm() - SiInterface.run() failed");

    // Update the inverse transformation (SI to world coordinates)
    mCoordinateTransformer.updateCoordinateTransformation(siOutput.environmentModel->resetOriginResult);

    // Temporary hack to copy timestamp (see long term fix: DFMFPT-1758)
    siOutput.collisionEnvironmentModel->sSigHeader.uiTimeStamp = timestamp_us;
    siOutput.environmentModel->sSigHeader.uiTimeStamp = timestamp_us;
    siOutput.parkingBoxes->sSigHeader.uiTimeStamp = timestamp_us;
}

void SiHighWrapper::convertLsmVedodoDataToCemEgoData(aupdf::EgoMotionAtCemOutput& egoMotionAtCemOutput, const lsm_vedodo::OdoEstimation& egoMotionData)
{
    egoMotionAtCemOutput.sSigHeader = egoMotionData.sSigHeader;
    egoMotionAtCemOutput.odoEstimationAtCemTime.xPosition_m = egoMotionData.xPosition_m;
    egoMotionAtCemOutput.odoEstimationAtCemTime.yPosition_m = egoMotionData.yPosition_m;
    egoMotionAtCemOutput.odoEstimationAtCemTime.yawAngle_rad = egoMotionData.yawAngle_rad;
    egoMotionAtCemOutput.odoEstimationAtCemTime.drivenDistance_m = egoMotionData.drivenDistance_m;
}

#if defined(USE_ENV_PLOTTER) //Visualization stuff
relocalization::DeveloperData SiHighWrapper::getRelocPlotData() {
    return relocalization::RelocalizationModule_interface::getInstance().getDeveloperData();
}
mf_mempark::DeveloperData SiHighWrapper::getDebugData() {
    return mf_mempark::mf_memory_parking_interface::getInstance().getDebugData();
}
#endif

void SiHighWrapper::linkStaticAndDynamicObjects(aupdf::SgfOutput& sgfHulls, aupdf::DynamicEnvironment& dynamicEnvironmentInput)
{
    for (LSM_GEOML::size_type i{ 0U }; i < dynamicEnvironmentInput.numberOfObjects; ++i)
    {
        for (LSM_GEOML::size_type k{ 0U }; k < sgfHulls.staticObjectsOutput.numberOfObjects; ++k)
        {
            const bool isStationaryOrStopped{ (dynamicEnvironmentInput.objects[i].dynamicProperty == VCEM::DynamicProperty::STATIONARY)
                                           || (dynamicEnvironmentInput.objects[i].dynamicProperty == VCEM::DynamicProperty::STOPPED) };
            if (isStationaryOrStopped && (dynamicEnvironmentInput.objects[i].id == sgfHulls.staticObjectsOutput.objects[k].obstacleId))
            {
                sgfHulls.staticObjectsOutput.objects[k].associatedDynamicObjectId = dynamicEnvironmentInput.objects[i].id;
            }
        }
    }
}

