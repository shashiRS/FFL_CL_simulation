#include "MocoWrapper.h"
#include <cmath>
#include "MfSilTypes.h"

// HACK: Enable simultaneous inclusion of mf_vedodo_types/LSM_VEDODO_Outputs.h in C and C++
#undef LSM_VEDODO_OUTPUTS_HEADER_
// HACK: Enable simultaneous inclusion of mf_trjctl_types/AP_TRJCTL_Outputs.h in C and C++
#undef AP_TRJCTL_OUTPUTS_HEADER_
#undef AP_COMMONVEHSIGPROVIDER_HEADER_

// HACK: define missing C-type variant of com::ComSignalState_t
typedef enum {
    COMSIGSTATE_INIT = 0U,  //!< COMSIGSTATE_INIT   data or struct is not valid
    COMSIGSTATE_VALID = 1U,  //!< COMSIGSTATE_VALID  data is valid for other usage
    COMSIGSTATE_INVALID = 2U   //!< COMSIGSTATE_INVALID  data is invalid or out of range, usage with care
// COMSIGSTATE_ERROR, COMSIGSTATE_TIMEOUT: incompatible to ADAS AlgoSignalState_t, removed. use COMSIGSTATE_INVALID instead
} ComSignalState_t;

#include "TRATCO/tratco_vsp_ext.h"
#include "VECONA/vecona_vsp_ext.h"

#include "Vehicle.h" // TODO remove CarMaker dependency from MocoWrapper
#include "Car/Car.h"
#include "Car/Brake.h"
#include "Car/Powertrain.h"
#include "Car/Steering.h"

MocoWrapper& MocoWrapper::getInstance() {
    static MocoWrapper instance{};
    return instance;
}

void MocoWrapper::init(const AP_Common::Vehicle_Params& vehicleParams)
{
    // TRATCO + VECONA inputs  
    mLongDriverFeedback = {};

    mVehParam = {};
    mVehParam.stat.vehHeightMax = static_cast<float32>(Vehicle.Cfg.OuterSkin[1][2]);
    const float32_t mirrorWidth_m = (4U == vehicleParams.AP_V_MIRROR_SHAPE_SIZE_NU) ? std::abs(vehicleParams.AP_V_LEFT_MIRROR_SHAPE_Y_M[0] - vehicleParams.AP_V_LEFT_MIRROR_SHAPE_Y_M[1]) : 0.0f;
    mVehParam.stat.vehWidthMax = vehicleParams.AP_V_WIDTH_M + 2.0f * mirrorWidth_m;
    mVehParam.stat.vehOverhangFront = vehicleParams.AP_V_LENGTH_M - vehicleParams.AP_V_WHEELBASE_M - vehicleParams.AP_V_OVERHANG_REAR_M;
    mVehParam.stat.vehOverhangRear = vehicleParams.AP_V_OVERHANG_REAR_M;
    mVehParam.stat.bumperPosFront = vehicleParams.AP_V_LENGTH_M - vehicleParams.AP_V_OVERHANG_REAR_M;
    mVehParam.stat.bumperPosRear = mVehParam.stat.vehOverhangRear;
    mVehParam.stat.wheelBase = vehicleParams.AP_V_WHEELBASE_M;
    mVehParam.stat.tireRadius = 0.5F * (vehicleParams.AP_V_TYRE_CIRCUMFERENCE_FRONT_M + vehicleParams.AP_V_TYRE_CIRCUMFERENCE_REAR_M) / C_TWOPI;
    mVehParam.stat.tireCrnrStiffFront = 154627.0F; // For now, cornering stiffness taken from Continental_ContiPremiumContact5_215_55R17_94W_MF-Tyre_62.tir
    mVehParam.stat.tireCrnrStiffRear = 154627.0F; // For now, cornering stiffness taken from Continental_ContiPremiumContact5_215_55R17_94W_MF-Tyre_62.tir
    mVehParam.stat.vehMassMean = static_cast<uint16>(std::round(Vehicle.Cfg.MassTotal));
    mVehParam.stat.vehMassEmpty = static_cast<uint16>(std::round(Vehicle.Cfg.MassTotal));

    // TRATCO
    mControlData_TRATCO = {};
    mControlData_TRATCO.opMode = DF_OP_MODE_RESET;

    mTratcoCycleCounter = 0U;
    mTratcoLongAccelReq = {};
    mTratcoLatKinReq = {};

    // VECONA
    mControlData_VECONA = {};
    mControlData_VECONA.opMode = DF_OP_MODE_RESET;

    mVeconaCycleCounter = 0U;
    mVehDynFcu = {};
    mVeconaVehDyn = {};
    mBrakeFb = {};
    mPowertrainFb = {};
    mSteeringFrontFb = {};
    mSteeringRearFb = {};
    mLatVeconaStatus = {};
    mVeconaLatKinRequest = {};

    //Ego vehicle
    mEgoVehicle = AP_Common::EgoVehicle();
    mEgoVehicle.init(&vehicleParams, AP_Common::EgoVehicleShapeType::EGO_VEH_SHAPE_STANDARD);

    //Vehicle parameter
    mSteerRatio_nu = vehicleParams.AP_V_STEER_RATIO_NU;
}

static FCU_te_GearPos toGearPos(const AP_CommonVehSigProvider::Gear& gearLeverPosition)
{
    switch (gearLeverPosition)
    {
    case AP_CommonVehSigProvider::Gear::GEAR_N: return FCU_GEAR_POS_NEUTRAL;
    case AP_CommonVehSigProvider::Gear::GEAR_1: return FCU_GEAR_FORWARD;
    case AP_CommonVehSigProvider::Gear::GEAR_2: return FCU_GEAR_FORWARD;
    case AP_CommonVehSigProvider::Gear::GEAR_3: return FCU_GEAR_FORWARD;
    case AP_CommonVehSigProvider::Gear::GEAR_4: return FCU_GEAR_FORWARD;
    case AP_CommonVehSigProvider::Gear::GEAR_5: return FCU_GEAR_FORWARD;
    case AP_CommonVehSigProvider::Gear::GEAR_6: return FCU_GEAR_FORWARD;
    case AP_CommonVehSigProvider::Gear::GEAR_7: return FCU_GEAR_FORWARD;
    case AP_CommonVehSigProvider::Gear::GEAR_8: return FCU_GEAR_FORWARD;
    case AP_CommonVehSigProvider::Gear::GEAR_P: return FCU_GEAR_POS_PARKING;
    case AP_CommonVehSigProvider::Gear::GEAR_S: return FCU_GEAR_FORWARD;
    case AP_CommonVehSigProvider::Gear::GEAR_D: return FCU_GEAR_FORWARD;
    case AP_CommonVehSigProvider::Gear::GEAR_INTERMEDIATE_POS: return FCU_GEAR_POS_NEUTRAL;
    case AP_CommonVehSigProvider::Gear::GEAR_R: return FCU_GEAR_POS_REVERSE;
    case AP_CommonVehSigProvider::Gear::GEAR_NOT_DEFINED: return FCU_GEAR_POS_PARKING; // safe state
    case AP_CommonVehSigProvider::Gear::GEAR_ERROR: return FCU_GEAR_POS_PARKING; // safe state
    }
    return FCU_GEAR_POS_NEUTRAL;
}

static FCU_te_TransmissionGear toTransmissionGear(const AP_CommonVehSigProvider::Gear& gearCur_nu)
{
    switch (gearCur_nu)
    {
    case AP_CommonVehSigProvider::Gear::GEAR_N: return FCU_GEAR_NEUTRAL;
    case AP_CommonVehSigProvider::Gear::GEAR_1: return FCU_GEAR_FIRST;
    case AP_CommonVehSigProvider::Gear::GEAR_2: return FCU_GEAR_SECOND;
    case AP_CommonVehSigProvider::Gear::GEAR_3: return FCU_GEAR_THIRD;
    case AP_CommonVehSigProvider::Gear::GEAR_4: return FCU_GEAR_FOURTH;
    case AP_CommonVehSigProvider::Gear::GEAR_5: return FCU_GEAR_FIFTH;
    case AP_CommonVehSigProvider::Gear::GEAR_6: return FCU_GEAR_SIXTH;
    case AP_CommonVehSigProvider::Gear::GEAR_7: return FCU_GEAR_SEVENTH;
    case AP_CommonVehSigProvider::Gear::GEAR_8: return FCU_GEAR_EIGHTH;
    case AP_CommonVehSigProvider::Gear::GEAR_P: return FCU_GEAR_PARK;
    case AP_CommonVehSigProvider::Gear::GEAR_S: return FCU_GEAR_INVALID; // should never happen
    case AP_CommonVehSigProvider::Gear::GEAR_D: return FCU_GEAR_FIRST; // should never happen
    case AP_CommonVehSigProvider::Gear::GEAR_INTERMEDIATE_POS: return FCU_TRANSMISSION_POWER_FREE;
    case AP_CommonVehSigProvider::Gear::GEAR_R: return FCU_GEAR_REVERSE;
    case AP_CommonVehSigProvider::Gear::GEAR_NOT_DEFINED: return FCU_GEAR_INVALID;
    case AP_CommonVehSigProvider::Gear::GEAR_ERROR: return FCU_GEAR_INVALID;
    }
    return FCU_GEAR_INVALID;
}

static void convertOdoEstimationPort(OdoEstimationPort& odoEstimationPortC, const LSM_VEDODO::OdoEstimationPort& odoEstimationPortCPP) {
    //drivingDirection_nu
    switch (odoEstimationPortCPP.drivingDirection_nu) {
    case LSM_VEDODO::Direction::DIRECTION_FORWARD:
        odoEstimationPortC.drivingDirection_nu = DIRECTION_FORWARD;
        break;
    case LSM_VEDODO::Direction::DIRECTION_REVERSE:
        odoEstimationPortC.drivingDirection_nu = DIRECTION_REVERSE;
        break;
    case LSM_VEDODO::Direction::DIRECTION_UNDEFINED:
        odoEstimationPortC.drivingDirection_nu = DIRECTION_UNDEFINED;
        break;
    default:
        break;
    }

    //motionStatus_nu
    switch (odoEstimationPortCPP.motionStatus_nu) {
    case LSM_VEDODO::MotionState::ODO_STANDSTILL:
        odoEstimationPortC.motionStatus_nu = ODO_STANDSTILL;
        break;
    case LSM_VEDODO::MotionState::ODO_NO_STANDSTILL:
        odoEstimationPortC.motionStatus_nu = ODO_NO_STANDSTILL;
        break;
    default:
        break;
    }

    //signalState_nu
    switch (odoEstimationPortCPP.signalState_nu) {
    case com::ComSignalState_t::COMSIGSTATE_INIT:
        odoEstimationPortC.signalState_nu = COMSIGSTATE_INIT;
        break;
    case com::ComSignalState_t::COMSIGSTATE_INVALID:
        odoEstimationPortC.signalState_nu = COMSIGSTATE_INVALID;
        break;
    case com::ComSignalState_t::COMSIGSTATE_VALID:
        odoEstimationPortC.signalState_nu = COMSIGSTATE_VALID;
        break;
    default:
        break;
    }
}

static void convertLongManeuverRequestPort(MF_CONTROL_t_LongManeuverRequestPort& longManeuverRequestPortC, const AP_TRJCTL::MF_CONTROL_t_LongManeuverRequestPort longManeuverRequestPortCPP) {
    //signalState_nu
    switch (longManeuverRequestPortCPP.signalState_nu) {
    case com::ComSignalState_t::COMSIGSTATE_INIT:
        longManeuverRequestPortC.signalState_nu = COMSIGSTATE_INIT;
        break;
    case com::ComSignalState_t::COMSIGSTATE_INVALID:
        longManeuverRequestPortC.signalState_nu = COMSIGSTATE_INVALID;
        break;
    case com::ComSignalState_t::COMSIGSTATE_VALID:
        longManeuverRequestPortC.signalState_nu = COMSIGSTATE_VALID;
        break;
    default:
        break;
    }
}

void MocoWrapper::run(const MocoSimulatorInput& simulatorInput,
    const LSM_VEDODO::OdoEstimationPort& odoEstimationPort,
    const AP_CommonVehSigProvider::GearboxCtrlStatusPort& gearboxCtrlStatusPort,
    const AP_TRJCTL::MF_CONTROL_t_LongManeuverRequestPort& longManeuverRequestPort,
    DF_t_ControlData& controlData_TRATCO,
    DF_t_ControlData& controlData_VECONA,
    FCU_t_LongDriverFb& longDriverFeedback,
    FCU_t_VehParam& vehParam,
    VECONA_t_VehDyn& vehDynFcu,
    FCU_t_BrakeFb& brakeFb,
    FCU_t_PowertrainFb& powertrainFb,
    FCU_t_SteeringFrontFb& steeringFrontFb,
    FCU_t_SteeringRearFb& steeringRearFb,
    TRATCO_t_CpldTratcoStatus& tratcoStatusPort,
    TRATCO_t_LongAccelReq& longAccelReq,
    TRATCO_VSPProcMem& tratcoProcessMemory,
    VECONA_t_VehDyn& vehDynVecona,
    VECONA_t_LongVeconaStatus& longVeconaStatusPort,
    VECONA_t_BrakeReq& veconaBrakeRequest,
    VECONA_t_PowertrainReq& veconaPowertrainRequest,
    VECONA_VSPProcMem& veconaProcessMemory)
{
    if (simulatorInput.timestamp_ms % TRATCO_CYCLE_TIME_MS == 0U)
    {
        mTratcoCycleCounter++;
        const SignalHeader_t tratcoSignalHeader{ simulatorInput.timestamp_ms, mTratcoCycleCounter, mTratcoCycleCounter, AL_SIG_STATE_OK };

        mControlData_TRATCO.versionNumber; // TODO
        mControlData_TRATCO.sigHeader = tratcoSignalHeader;
        if (mControlData_TRATCO.sigHeader.uiCycleCounter > 2U)
        {
          mControlData_TRATCO.opMode = DF_OP_MODE_NORMAL;
        }

        mVehParam.versionNumber; // TODO
        mVehParam.sigHeader = tratcoSignalHeader;
        // dynamic vehicle params
        mVehParam.dyn.cntrOfGrav = { // center of mass of generalized vehicle body relative to middle of rear axle
            static_cast<float32>(Car.GenBdy1.t_0[0] - Car.Fr1.t_0[0]) - mVehParam.stat.vehOverhangRear,
            static_cast<float32>(Car.GenBdy1.t_0[1] - Car.Fr1.t_0[1]),
            static_cast<float32>(Car.GenBdy1.t_0[2] - Car.Fr1.t_0[2]) };
        mVehParam.dyn.tireCrnrStiffFront = mVehParam.stat.tireCrnrStiffFront;
        mVehParam.dyn.tireCrnrStiffRear = mVehParam.stat.tireCrnrStiffRear;
        mVehParam.dyn.tireRadius = mVehParam.stat.tireRadius;
        mVehParam.dyn.vehMass = mVehParam.stat.vehMassMean;
        mVehParam.dyn.trailerAttached = FALSE;

        // TRATCO require ports struct
        static TRATCO_FrameReqPorts tratcoRequirePorts{};
        tratcoRequirePorts.p_controlData = &mControlData_TRATCO;
        tratcoRequirePorts.p_odoEstimation = reinterpret_cast<const OdoEstimationPort*>(&odoEstimationPort);

        //Issue in transformation C++ to C in drivingDirection_nu, motionStatus_nu, signalState_nu
        OdoEstimationPort odoEstimationPortC = *tratcoRequirePorts.p_odoEstimation;
        convertOdoEstimationPort(odoEstimationPortC, odoEstimationPort);
        tratcoRequirePorts.p_odoEstimation = &odoEstimationPortC;

        tratcoRequirePorts.p_vehDynVecona = &mVeconaVehDyn;
        tratcoRequirePorts.p_longDrvFb = &mLongDriverFeedback;
        tratcoRequirePorts.p_longManeuverReq = reinterpret_cast<const MF_CONTROL_t_LongManeuverRequestPort*>(&longManeuverRequestPort);

        //Issue in transformation C++ to C in signalState_nu
        MF_CONTROL_t_LongManeuverRequestPort longManeuverRequestPortC = *tratcoRequirePorts.p_longManeuverReq;
        convertLongManeuverRequestPort(longManeuverRequestPortC, longManeuverRequestPort);
        tratcoRequirePorts.p_longManeuverReq = &longManeuverRequestPortC;

        tratcoRequirePorts.p_longVeconaStatus = &longVeconaStatusPort;
        tratcoRequirePorts.p_vehParam = &mVehParam;

        // TRATCO provide ports struct
        static TRATCO_FrameProPorts tratcoProvidePorts{};
        tratcoProvidePorts.p_cpldTratcoStatus = &tratcoStatusPort;
        tratcoProvidePorts.p_longAccelReq = &mTratcoLongAccelReq;

        TRATCO_v_exec_sim(&tratcoRequirePorts, &tratcoProvidePorts, &tratcoProcessMemory);

    }

    if (simulatorInput.timestamp_ms % VECONA_CYCLE_TIME_MS == 0U)
    {
        static double gasPedalPreviousCycle{};
        if (simulatorInput.timestamp_ms < VECONA_CYCLE_TIME_MS) 
        {
            // initialization in first cycle
            gasPedalPreviousCycle = DrivMan.Gas;
        }
        mVeconaCycleCounter++;
        const SignalHeader_t veconaSignalHeader{ simulatorInput.timestamp_ms, mVeconaCycleCounter, mVeconaCycleCounter, AL_SIG_STATE_OK };

        mControlData_VECONA.versionNumber; // TODO
        mControlData_VECONA.sigHeader = veconaSignalHeader;
        if (mControlData_VECONA.sigHeader.uiCycleCounter > 2U)
        {
          mControlData_VECONA.opMode = DF_OP_MODE_NORMAL;
        }

        mBrakeFb.versionNumber; // TODO
        mBrakeFb.sigHeader = veconaSignalHeader;
        mBrakeFb.axleTrqSumCur = -static_cast<float32>(Brake.Trq_tot[0] + Brake.Trq_tot[1] + Brake.Trq_tot[2] + Brake.Trq_tot[3]);
        mBrakeFb.executedFunMode = MOCO_FUN_MODE_MANEUVER;
        mBrakeFb.ssmStatus = MOCO_SSM_STATUS_INVALID; // SSM is handled by VECONA internally.
        mBrakeFb.available = TRUE;
        mBrakeFb.failure = FALSE;
        mBrakeFb.driverBraking = simulatorInput.overrideDecel;
        mBrakeFb.parkingBrkOpen = !simulatorInput.vehicleWasSecuredMoCo_nu;

        mVehDynFcu.versionNumber; // TODO
        mVehDynFcu.bankAgl = 0.0f; // TODO: Lateral TRATCO/VECONA
        mVehDynFcu.frontSteerAgl = simulatorInput.steerAngleFront_rad;  //steer angle at wheels
        mVehDynFcu.crv = mEgoVehicle.calcCrvFromWhlAngle(static_cast<float32>(simulatorInput.steerAngleFront_rad));   // TODO: Felix check whether VEDODO can provide curvature output at rear axle center (like EML is doing)
        mVehDynFcu.imageProcTime = 0; // TODO???
        mVehDynFcu.rearSteerAgl = 0.0f; // TODO: Rear axle steering
        mVehDynFcu.roadSlope = 0.0f; // TODO: Felix check whether VEDODO can provide slope output
        mVehDynFcu.sideSlpAgl = 0.0f; // TODO: Lateral TRATCO/VECONA
        mVehDynFcu.sigHeader = veconaSignalHeader;

        const double factorGearbox2Axle{ PowerTrain.DriveLineIF.CfgIF->iDiff_mean };
        mPowertrainFb.versionNumber; // TODO
        mPowertrainFb.sigHeader = veconaSignalHeader;
        mPowertrainFb.axleTrqSumCur = static_cast<float32>((std::abs(PowerTrain.GearBoxIF.i) * factorGearbox2Axle) * simulatorInput.engineTrq_Nm); //static_cast<float32>(PowerTrain.EngineIF.Trq * (PowerTrain.GearBoxIF.i * factorGearbox2Axle));
        mPowertrainFb.axleTrqSumDistrib.frontRear = 50U; // default
        mPowertrainFb.axleTrqSumDistrib.frontLeftRight = 50U; // default
        mPowertrainFb.axleTrqSumDistrib.rearLeftRight = 50U; // default
        mPowertrainFb.axleTrqSumMax = static_cast<float32>((std::abs(PowerTrain.GearBoxIF.i) * factorGearbox2Axle) * simulatorInput.engineMaxTrq_Nm);
        mPowertrainFb.axleTrqSumMin = 50.0f; //static_cast<float32>((std::abs(PowerTrain.GearBoxIF.i) * factorGearbox2Axle) * simulatorInput.engineMinTrq_Nm);
        mPowertrainFb.axleTrqSumFastMax = mPowertrainFb.axleTrqSumMax; // TODO fast torque response of engine
        mPowertrainFb.axleTrqSumFastMin = mPowertrainFb.axleTrqSumMin; // TODO fast torque response of engine
        mPowertrainFb.axleTrqDriverReq = static_cast<float32>((std::abs(PowerTrain.GearBoxIF.i) * factorGearbox2Axle) * simulatorInput.engineTrqDriverReq_Nm);
        mPowertrainFb.executedFunMode = MOCO_FUN_MODE_MANEUVER;
        mPowertrainFb.accelPdlPos = static_cast<uint8>(std::round(DrivMan.Gas * 100)); // unit: %
        mPowertrainFb.accelPdlGrd = static_cast<uint8>(std::round((DrivMan.Gas - gasPedalPreviousCycle) * 100 / VECONA_CYCLE_TIME_MS * 1000)); // unit: %/s
        gasPedalPreviousCycle = DrivMan.Gas;
        mPowertrainFb.gearReqDriver = toGearPos(gearboxCtrlStatusPort.gearLeverInformation.gearLeverPositionCur_nu);
        mPowertrainFb.gearShiftActive = FALSE; // The torque drop during gear shift in CarMaker lasts for ~200ms, which is acceptable for Moco and requires no feedback of the gear shift.
        mPowertrainFb.gearCur = toTransmissionGear(gearboxCtrlStatusPort.gearInformation.gearCur_nu);
        mPowertrainFb.sailingStatus = FCU_SAILING_NOT_AVAILABLE;
        mPowertrainFb.available = TRUE;
        mPowertrainFb.failure = FALSE;
        mPowertrainFb.arbitrationWinner = TRUE;

        mSteeringFrontFb.versionNumber; // TODO
        mSteeringFrontFb.sigHeader = veconaSignalHeader;
        mSteeringFrontFb.steerAglCur = simulatorInput.steerAngleFront_rad;  //steer angle at wheels
        mSteeringFrontFb.steerAglVeloCur = static_cast<float32>(Steering.IF.AngVel)/ mSteerRatio_nu;  //steer velocity at wheels
        mSteeringFrontFb.steerTrqCur = static_cast<float32>(Steering.IF.Trq);;
        mSteeringFrontFb.columnTrqCur = 0.0F; // torque imposed by driver nearly zero, since driver not allowed to steer during automated parking.
        mSteeringFrontFb.executedFunMode = MOCO_FUN_MODE_MANEUVER;
        mSteeringFrontFb.available = TRUE;
        mSteeringFrontFb.failure = FALSE;

        mSteeringRearFb.versionNumber; // TODO
        mSteeringRearFb.sigHeader = veconaSignalHeader;
        mSteeringRearFb.steerAglCur; // TODO: Rear axle steering
        mSteeringRearFb.steerAglVeloCur; // TODO: Rear axle steering
        mSteeringRearFb.executedFunMode = MOCO_FUN_MODE_INVALID; // rear-axle steering not available
        mSteeringRearFb.available = FALSE;
        mSteeringRearFb.failure = FALSE;

        // VECONA require ports struct
        static VECONA_PlatformReqPorts veconaRequirePorts{};
        veconaRequirePorts.p_controlData = &mControlData_VECONA;
        veconaRequirePorts.p_vehDynFcu = &mVehDynFcu;
        veconaRequirePorts.p_odoEstimation = reinterpret_cast<const OdoEstimationPort*>(&odoEstimationPort);
        
        //Issue in transformation C++ to C in drivingDirection_nu, motionStatus_nu, signalState_nu
        OdoEstimationPort odoEstimationPortC = *veconaRequirePorts.p_odoEstimation;
        convertOdoEstimationPort(odoEstimationPortC, odoEstimationPort);
        veconaRequirePorts.p_odoEstimation = &odoEstimationPortC;
        
        veconaRequirePorts.p_brakeFb = &mBrakeFb;
        veconaRequirePorts.p_powertrainFb = &mPowertrainFb;
        veconaRequirePorts.p_steeringFrontFb = &mSteeringFrontFb;
        veconaRequirePorts.p_steeringRearFb = &mSteeringRearFb;
        veconaRequirePorts.p_latKinReqTratco = &mTratcoLatKinReq; // lateral not used
        veconaRequirePorts.p_longAccelReq = &mTratcoLongAccelReq;
        veconaRequirePorts.p_vehParam = &mVehParam;

        // VECONA provide ports struct
        static VECONA_PlatformProPorts veconaProvidePorts{};
        veconaProvidePorts.p_vehDynVecona = &mVeconaVehDyn;
        veconaProvidePorts.p_longDrvFb = &mLongDriverFeedback;
        veconaProvidePorts.p_latVeconaStatus = &mLatVeconaStatus;  // lateral not used
        veconaProvidePorts.p_longVeconaStatus = &longVeconaStatusPort;
        veconaProvidePorts.p_brakeReq = &veconaBrakeRequest;
        veconaProvidePorts.p_powertrainReq = &veconaPowertrainRequest;
        veconaProvidePorts.p_latKinReqVecona = &mVeconaLatKinRequest;  // lateral not used

        VECONA_v_exec_sim(&veconaRequirePorts, &veconaProvidePorts, &veconaProcessMemory);
    }

    // Link data to extern sructs for CarMaker DVA visualization (TODO: Check which functionality/members can be removed in MoCoWrapper and covered by mdlApCtrl)
    //Inputs
    controlData_TRATCO = mControlData_TRATCO;
    controlData_VECONA = mControlData_VECONA;
    vehParam = mVehParam;
    vehDynFcu = mVehDynFcu;
    brakeFb = mBrakeFb;
    powertrainFb = mPowertrainFb;
    steeringFrontFb = mSteeringFrontFb;
    steeringRearFb = mSteeringRearFb;

    //TRATCO outputs
    longAccelReq = mTratcoLongAccelReq;

    //VECONA outputs
    longDriverFeedback = mLongDriverFeedback;
    vehDynVecona = mVeconaVehDyn;
}
