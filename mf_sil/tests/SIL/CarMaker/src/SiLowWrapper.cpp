#include "SiLowWrapper.h"
#include "SiUtility.h"
#include "pod_class_converter.h"
#include <si/si_params.h>
#include <mf_mempark/system_defined_pose_port.h>
#include <geoml/CoordinateTransformer2D.h>

namespace si {

    SiLowWrapper& SiLowWrapper::getInstance() {
        static SiLowWrapper instance{};
        return instance;
    }

    void SiLowWrapper::init(const ap_common::Vehicle_Params &vehicleParams) {
        static si::SiConfig siConfig{};
        siConfig.siParams = &SiUtility::getSiParameters();
        siConfig.vehicleParams = &vehicleParams;
        SiInterface::getInstance().init(siConfig);
    }

    void SiLowWrapper::reset() {
        // reset input
        mSiInput = {};
        // reset output
        mSiOutput = {};
        mParkBoxData = eco::create_default<si::ApParkingBoxPort>();
        mPsiEnvModelPort = eco::create_default<si::ApEnvModelPort>();
        // reset coordinate transformation
        mCoordinateTransformer.reset();
    }

    std::pair<SiOutput, std::string> SiLowWrapper::run(
        const lsm_vedodo::OdoEstimationOutputPort &odoEstimationPort,
        const ap_psm::SlotCtrlPort &slotCtrlPort,
        const us_em::UsEnvModelPort& usEnvModelPort,
        const us_em::PerceptionAvailabilityPort& inputPercAvailPort,
        si::PerceptionAvailabilityPort& outputPercAvailPort,
        si::EgoMotionPort& egoMotionPort,
        si::CollEnvModelPort& collEnvModelPort
#if defined(USE_ENV_PLOTTER)
        , si::PlotData& plotDataPort
#endif
        )
    {
        mf_mempark::SystemDefinedPosePort systemDefinedPosePortDummy{ eco::create_default<mf_mempark::SystemDefinedPosePort>() }; //for now, just provide empty port
        systemDefinedPosePortDummy.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        systemDefinedPosePortDummy.sSigHeader.uiTimeStamp = usEnvModelPort.sSigHeader.uiTimeStamp;

        //To be removed once SI Entry Variant correctly takes us_em data structures
        static_assert(sizeof(si::CollEnvModelPort) == sizeof(us_em::UsEnvModelPort), "us_em::UsEnvModelPort and si::CollEnvModelPort have diverged. Temporary workaround mapping is not working anymore. Please utilize us_em data structures in SI Entry variant.");
        static_assert(sizeof(si::PerceptionAvailabilityPort) == sizeof(us_em::PerceptionAvailabilityPort), "us_em::PerceptionAvailabilityPort and si::PerceptionAvailabilityPort have diverged. Temporary workaround mapping is not working anymore. Please utilize us_em data structures in SI Entry variant.");
        //Also to be removed, either once other components also correctly use usEnvModelPort or a different solution is found
        memcpy(&collEnvModelPort, &usEnvModelPort, sizeof(si::CollEnvModelPort));

        mSiInput = { &odoEstimationPort,  &slotCtrlPort, reinterpret_cast<const si::CollEnvModelPort*>(&usEnvModelPort), reinterpret_cast<const si::PerceptionAvailabilityPort*>(&inputPercAvailPort), &systemDefinedPosePortDummy };
        mSiOutput = { &mParkBoxData, &mPsiEnvModelPort, &outputPercAvailPort };
#if defined(USE_ENV_PLOTTER)
        mSiOutput.plotData = &plotDataPort;
#endif

        const auto result = SiInterface::getInstance().run(mSiInput, mSiOutput);
        //Todo move this into the si run function
        fillEgoMotionOutput(odoEstimationPort, egoMotionPort);
        const std::string message = (com::ComResult::COMRES_OK == result) ? "" :
            "SiLowWrapper:run() - SiInterface.run() failed";

        // Update the inverse transformation (SI to world coordinates)
        mCoordinateTransformer.updateCoordinateTransformation(mSiOutput.environmentModel->resetOriginResult);

        return std::make_pair(mSiOutput, message);
    }

    void SiLowWrapper::fillEgoMotionOutput(const lsm_vedodo::OdoEstimationOutputPort& odoInput, si::EgoMotionPort& egoMotionPort)
    {
        const lsm_vedodo::OdoEstimation& odo = odoInput.odoEstimation;
        egoMotionPort.uiVersionNumber = si::EgoMotionPort_InterfaceVersion::EgoMotionPort_VERSION;
        egoMotionPort.sSigHeader = odo.sSigHeader;
        egoMotionPort.motionState_nu = static_cast<si::SIMotionState>(odo.motionStatus_nu);
        egoMotionPort.pitch_rad = odo.pitchAngle_rad;
        egoMotionPort.roll_rad = odo.rollRate_radps;
        egoMotionPort.vel_mps = odo.longiVelocity_mps;
        egoMotionPort.yawRate_radps = odo.yawRate_radps;
        egoMotionPort.accel_mps2 = odo.longiAcceleration_mps2;
        egoMotionPort.drivenDistance_m = odo.drivenDistance_m;
        egoMotionPort.frontWheelAngle_rad = odo.steerAngFrontAxle_rad;
        egoMotionPort.rearWheelAngle_rad = odo.steerAngRearAxle_rad;
    }

    void SiLowWrapper::convertToUsEnvModelPort(const uint64_t& timestamp_us, const std::vector<VCEM::ConvexHull>& convexHulls,
        us_em::UsEnvModelPort& usEnvModelPort)
    {
        // fill environment model for SI
        std::vector<VCEM::ConvexHull>static_convex_hulls;
        std::vector<VCEM::ConvexHull>dynamic_convex_hulls;
        for (size_t iObj = 0; iObj < convexHulls.size(); ++iObj) {
            if (convexHulls[iObj].objectType == VCEM::CemObjectType::DYNAMIC_OBJECT) {
                dynamic_convex_hulls.push_back(convexHulls[iObj]);
            }
            else {
                static_convex_hulls.push_back(convexHulls[iObj]);
            }
        }
        usEnvModelPort.sSigHeader.uiTimeStamp = timestamp_us;
        usEnvModelPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        usEnvModelPort.numberOfStaticObjects_u8 = static_cast<uint8>(std::min(static_convex_hulls.size(),
            static_cast<size_t>(ap_common::AP_COMMON_TYPES_Consts::COLL_G_MAX_NUM_STATIC_OBJ_NU)));
        usEnvModelPort.numberOfDynamicObjects_u8 = static_cast<uint8>(std::min(dynamic_convex_hulls.size(),
            static_cast<size_t>(ap_common::AP_COMMON_TYPES_Consts::COLL_G_MAX_NUM_DYN_OBJECTS_NU)));
        // Static objects
        for (uint8 iObj_static = 0U; iObj_static < usEnvModelPort.numberOfStaticObjects_u8; ++iObj_static) {
            auto &staticObject = usEnvModelPort.staticObjects[iObj_static];
            assert(static_convex_hulls[iObj_static].u_id <= static_cast<uint32_t>(std::numeric_limits<uint16>::max()));
            staticObject.refObjID_nu = static_cast<uint16_t>(static_convex_hulls[iObj_static].u_id);
            staticObject.readFromNVRAM_nu = false;
            staticObject.existenceProb_perc = 100u;
            staticObject.objHeightClass_nu = static_convex_hulls[iObj_static].objHeightClass_nu;
            staticObject.objHeightClassConfidence_perc = 100u;
            staticObject.objAgeInCycles_nu = static_convex_hulls[iObj_static].objAgeInCycles_nu;

            assert(static_convex_hulls[iObj_static].polygon.size() <= ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU);
            staticObject.objShape_m.actualSize = static_cast<lsm_geoml::size_type>(std::min(static_convex_hulls[iObj_static].polygon.size(),
                static_cast<size_t>(ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU)));
            for (lsm_geoml::size_type iVert{ 0U }; iVert < staticObject.objShape_m.actualSize; ++iVert) {
                staticObject.objShape_m.array[iVert] = convert(static_convex_hulls[iObj_static].polygon[iVert]);
            }
        }
        // Dynamic Objects
        for (uint8 iObj_dyn = 0U; iObj_dyn < usEnvModelPort.numberOfDynamicObjects_u8; ++iObj_dyn) {
            auto &dynamicObject = usEnvModelPort.dynamicObjects[iObj_dyn];
            const VCEM::ConvexHull &convexHull = dynamic_convex_hulls[iObj_dyn];
            dynamicObject.existenceProb_perc = 100u;
            dynamicObject.vel_mps = { convexHull.dynamicObjectProperty_nu.velocity.x(), convexHull.dynamicObjectProperty_nu.velocity.y() };
            dynamicObject.accel_mps2 = { convexHull.dynamicObjectProperty_nu.acceleration.x(), convexHull.dynamicObjectProperty_nu.acceleration.y() };
            dynamicObject.headingAngle_rad = LSM_GEOML::radMod(convexHull.dynamicObjectProperty_nu.orientation);
            switch (convexHull.dynamicObjectProperty_nu.state) {
            case VCEM::MeasurementObjState::MEASURED:
                dynamicObject.measurementState_nu = us_em::ObjMeasurementState::MEAS_STATE_MEASURED;
                break;
            case VCEM::MeasurementObjState::PREDICTED:
                dynamicObject.measurementState_nu = us_em::ObjMeasurementState::MEAS_STATE_PREDICTED;
                break;
            case VCEM::MeasurementObjState::DELETED:
                dynamicObject.measurementState_nu = us_em::ObjMeasurementState::MEAS_STATE_DELETED;
                break;
            case VCEM::MeasurementObjState::INVALID:
                dynamicObject.measurementState_nu = us_em::ObjMeasurementState::MAX_NUM_MEASUREMENT_STATES; // ?? not sure ??
                break;
            }
            assert(convexHull.u_id <= static_cast<uint32_t>(std::numeric_limits<uint16>::max()));
            dynamicObject.refObjID_nu = convexHull.u_id;
            assert(VCEM::DYN_OBJECT_NUM_SHAPE_POINTS <= ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_FOR_DYN_POLY_NU);
            dynamicObject.objShape_m.actualSize = static_cast<lsm_geoml::size_type>(std::min(VCEM::DYN_OBJECT_NUM_SHAPE_POINTS,
                static_cast<int32_t>(ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_FOR_DYN_POLY_NU)));
            for (lsm_geoml::size_type j = 0U; j < dynamicObject.objShape_m.actualSize; ++j) {
                dynamicObject.objShape_m.array[j] = convert(convexHull.dynamicObjectProperty_nu.points[j]);
            }
        }

        SiUtility::getInstance().sortStaticObjectsByDistance(usEnvModelPort);
        SiUtility::getInstance().setFirstObjOutDetZoneIdx(usEnvModelPort);
    }

    void SiLowWrapper::update(us_em::PerceptionAvailabilityPort& percAvailPort)
    {
        for (auto &usSensorStatus : percAvailPort.statusUSSensors_nu) {
            usSensorStatus = us_em::AvailabilityStatus::ITEM_AVAILABLE;
        }
        percAvailPort.statusEnvModel_nu = us_em::AvailabilityStatus::ITEM_AVAILABLE;
    }
}
