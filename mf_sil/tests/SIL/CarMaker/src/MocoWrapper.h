#pragma once

#include "mf_common_types/Vehicle_Params.h"
#include "mf_vedodo_types/LSM_VEDODO_Outputs.h"
#include "mf_trjctl_types/MoCo_Outputs.h"
#include "mf_trjctl_types/MoCo_FCU_Outputs.h"
#include "mf_trjctl_types/AP_TRJCTL_Outputs.h"
#include "mf_common/EgoVehicle.h"

struct MocoSimulatorInput {
    uint64 timestamp_ms{ 0U };
    bool overrideAccel{ false }; // Driver overriding by accelerator pedal
    bool overrideDecel{ false }; // Driver overriding by decelerator pedal
    double engineMaxTrq_Nm{ 0.0 }; // Engine maximal (full) torque at current rotation speed
    double engineMinTrq_Nm{ 0.0 }; // Engine minimal (drag) torque at current rotation speed
    double engineTrq_Nm{ 0.0 }; // Engine torque at current rotation speed
    double engineTrqDriverReq_Nm{ 0.0 }; // Engine torque requested by the driver
    float steerAngleFront_rad{ 0.0f }; // Front steer angle at wheels
    bool vehicleWasSecuredMoCo_nu{ false }; //Indicate that vehicle was secured (hold data)
    //bool vehicleStandstill;
};

struct TRATCO_VSPProcMem;
struct VECONA_VSPProcMem;

class MocoWrapper
{
public:
    static constexpr uint64 TRATCO_CYCLE_TIME_MS = 20U;
    static constexpr uint64 VECONA_CYCLE_TIME_MS = 10U;

    // @details return the instance of the MocoWrapper
    static MocoWrapper& getInstance();
    MocoWrapper(MocoWrapper const&) = delete; // prevent copying the singleton
    MocoWrapper& operator= (MocoWrapper const&) = delete;

    //!
    //! \brief      Initialize the motion control wrapper
    //! \param[in]  vehicleParams   Parameters of the ego vehicle
    //!
    void init(const AP_Common::Vehicle_Params& vehicleParams);

    //!
    //! \brief      Main function to run the motion control component
    void run(const MocoSimulatorInput& simulatorInput,
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
        VECONA_VSPProcMem& veconaProcessMemory);

private:
    MocoWrapper() = default;  // prevent creation of another instance outside of this class

    // TRATCO + VECONA inputs
    FCU_t_LongDriverFb mLongDriverFeedback{};
    FCU_t_VehParam mVehParam{};

    // TRATCO
    DF_t_ControlData mControlData_TRATCO{};
    uint16 mTratcoCycleCounter{ 0U };
    TRATCO_t_LongAccelReq mTratcoLongAccelReq{};
    TRATCO_t_LatKinReq mTratcoLatKinReq{}; // lateral not used

    // VECONA
    DF_t_ControlData mControlData_VECONA{};
    uint16 mVeconaCycleCounter{ 0U };
    VECONA_t_VehDyn mVehDynFcu{};
    VECONA_t_VehDyn mVeconaVehDyn{};
    FCU_t_BrakeFb mBrakeFb{};
    FCU_t_PowertrainFb mPowertrainFb{};
    FCU_t_SteeringFrontFb mSteeringFrontFb{};
    FCU_t_SteeringRearFb mSteeringRearFb{};
    VECONA_t_LatVeconaStatus mLatVeconaStatus{}; // lateral not used
    VECONA_t_LatKinReq mVeconaLatKinRequest{}; // lateral not used

    //Ego vehicle
    AP_Common::EgoVehicle mEgoVehicle{};

    //Vehicle parameter
    float mSteerRatio_nu{ 0.0f };
};

