#include "parkingscenemodel.h"

#include <MF_TRJPLA_TestDataHandler_wMetaData.h>

#include <QFileInfo>
#include <QDebug>
#include <QMetaProperty>
#include <QMessageBox>

ParkingSceneModel::ParkingSceneModel(QObject *parent)
    : QObject(parent)
    , mStartPoseModel(this)
    , mDebugVehicle(this)
    , mParkingBoxCollection(this)
{
    mSceneCollection.append(SceneEntry{SceneData{}});
    mCurrentSceneEntry = &mSceneCollection.front();
    mCurrentSceneEntry->sceneData.reset();

    mLoadedScenesListModel.setStringList({"0"});

    memset(&mVehicleParams, 0, sizeof(mVehicleParams));

    mVehicleParams.AP_V_WHEELBASE_M = 2.786f; // some default value
    //mVehicleParams.AP_V_MAX_STEER_ANG_RAD = 0.595f; // some default value
    mVehicleParams.AP_V_TRACK_FRONT_M = 1.586f; // some default value

    for(auto& modelPtr: mTargetPoseModels) {
        modelPtr.reset(new TargetPoseModel(this));
    }

    connect(&mParkingBoxCollection, &ParkingBoxCollectionModel::dataChanged, this, &ParkingSceneModel::onDataChanged);
    mStartPoseModel.setVisible(true);

    for(size_t i = 0; i < mStaticStructuresModels.size(); i++) {
        connect(&mStaticStructuresModels.at(i), &StaticObstacleModel::dataChanged, this, &ParkingSceneModel::onDataChanged);
    }
#ifndef ULTRASONIC_ONLY
    for(size_t i = 0; i < mParkingSpaceMarkingModels.size(); i++) {
        connect(&mParkingSpaceMarkingModels.at(i), &ParkingSpaceMarkingModel::markingChanged, this, &ParkingSceneModel::onDataChanged);
    }
#endif
}

void ParkingSceneModel::loadEM(QString filename)
{
    // check if path exists and is file
    QFileInfo fileInfo(filename);
    if(!fileInfo.exists() || !fileInfo.isFile()) {
        QMessageBox::critical(nullptr, "File does not exist", "Could not load EM: file \""
                              + filename + "\" does not exist.");
        return;
    }

    si::SI_OUTPUT_JsonTestDataHandler jsonTestDataHandler;
    ap_tp::MF_TRJPLA_TestDataHandler_wMetaData mftrjplaTestDataHandler;

    memset((void*)&jsonTestDataHandler.mApEnvModelPort, 0, sizeof(jsonTestDataHandler.mApEnvModelPort));
    memset((void*)&jsonTestDataHandler.mApParkingBoxPort, 0, sizeof(jsonTestDataHandler.mApParkingBoxPort));
    memset((void*)&jsonTestDataHandler.mEgoMotionPort, 0, sizeof(jsonTestDataHandler.mEgoMotionPort));
    memset((void*)&mftrjplaTestDataHandler.mTargetPosesPort, 0, sizeof(mftrjplaTestDataHandler.mTargetPosesPort));
    memset((void*)&mftrjplaTestDataHandler.mTrjplaMetaData, 0, sizeof(mftrjplaTestDataHandler.mTrjplaMetaData));

    auto changeGuard = startBigChange();

    mSceneCollection.clear();

    jsonTestDataHandler.init(filename.toStdString());
    mftrjplaTestDataHandler.init(filename.toStdString());
    if(jsonTestDataHandler.mJVal.isMember("Input")) {
        qDebug() << "loading scene from Input list of size" << jsonTestDataHandler.getInputSize();

        if (!jsonTestDataHandler.mJVal["Input"].isArray()) {
            jsonTestDataHandler.getOutputPorts(jsonTestDataHandler.mJVal["Input"]);
            addSceneEntryFromJson(&jsonTestDataHandler.mJVal["Input"], jsonTestDataHandler, mftrjplaTestDataHandler);
        } else {
            for (int i = 0; i < jsonTestDataHandler.getInputSize(); i++) {
                // perform step if we need to read the next scene
                // note that the first step is already done inside jsonTestDataHandler.init()
                if (i > 0) {
                    jsonTestDataHandler.step();
                    // target pose port is considered as expected output and not read by default
                    // therefore we need to read it in manually
                    // reset target pose data since we don't want to take over previous target pose data in case
                    // that this i'th scene does not contain any target pose data.
                    memset((void*)&mftrjplaTestDataHandler.mTargetPosesPort, 0, sizeof(mftrjplaTestDataHandler.mTargetPosesPort));
                    mftrjplaTestDataHandler.step();
                }
                jsonTestDataHandler.getOutputPorts(jsonTestDataHandler.mJVal["Input"][i]);

                addSceneEntryFromJson(&jsonTestDataHandler.mJVal["Input"][i], jsonTestDataHandler, mftrjplaTestDataHandler);
            }
        }
    } else {
        jsonTestDataHandler.getOutputPorts(jsonTestDataHandler.mJVal);
        addSceneEntryFromJson(&jsonTestDataHandler.mJVal, jsonTestDataHandler, mftrjplaTestDataHandler);
    }

    mCurrentSceneEntry = mSceneCollection.empty()? nullptr : &mSceneCollection.front();

    if(mCurrentSceneEntry) {
        reloadSceneData();
    }

    // fill model list for displaying
    QStringList stringList;
    for(int i = 0; i < mSceneCollection.size(); i++) {
        stringList.append(QString::number(i));
    }
    mLoadedScenesListModel.setStringList(stringList);

    // signal about changed EM
    mSelectedEMIndex = 0;
    emit selectedEMIndexChanged(mSelectedEMIndex);

    mLoadedEMFilename = filename;
    emit loadedEMPathChanged(mLoadedEMFilename);
}

void ParkingSceneModel::addSceneEntryFromJson(::Json::Value *rootVal, si::SI_OUTPUT_JsonTestDataHandler &jsonTestDataHandler, ap_tp::MF_TRJPLA_TestDataHandler_wMetaData &trjplaTestDataHandler)
{
    mSceneCollection.append(SceneEntry {
                SceneData{jsonTestDataHandler.mApEnvModelPort,
                                      trjplaTestDataHandler.mTargetPosesPort,
                                      jsonTestDataHandler.mEgoMotionPort,
                                      jsonTestDataHandler.mApParkingBoxPort,
                                      trjplaTestDataHandler.mTrjplaMetaData} });

    QList<TrajectoryEntry> paths;

    if(rootVal && rootVal->isMember("EgoPath")) {
        Trajectory path;

        auto& jsonPath = (*rootVal)["EgoPath"];
        for(unsigned i = 1; i < jsonPath.size(); i++) { // starting from 1 since after first pose CS is reseted
            auto const& jPose = jsonPath[i];
            TrajectoryPose trajPose;
            trajPose.trans.translate(jPose["pos_m"][0].asFloat(), jPose["pos_m"][1].asFloat());
            trajPose.trans.rotateRadians(jPose["yaw_rad"].asFloat());

            if(jPose.isMember("vel_mps")) {
                trajPose.velocity_mps = jPose["vel_mps"].asFloat();
            }

            path.append(trajPose);
        }

        qDebug() << "Loaded ego path of size" << path.size();
        paths.append(TrajectoryEntry{"EgoPath", path});
    }


    if(rootVal && rootVal->isMember("PlannedPath")) {
        Trajectory path;

        auto& jsonPath = (*rootVal)["PlannedPath"];
        for(unsigned i = 0; i < jsonPath.size(); i++) {
            auto const& jPose = jsonPath[i];
            TrajectoryPose trajPose;

            trajPose.trans.translate(jPose["pos_m"][0].asFloat(), jPose["pos_m"][1].asFloat());
            trajPose.trans.rotateRadians(jPose["yaw_rad"].asFloat());

            if(jPose.isMember("vel_mps")) {
                trajPose.velocity_mps = jPose["vel_mps"].asFloat();
            }

            path.append(trajPose);
        }

        qDebug() << "Loaded planned path of size" << path.size();
        //paths.append(TrajectoryEntry{ "PlannedPath", path });
    }

    if(paths.size() > 0) {
        mTrajectoriesModel.setTrajectories(paths);
    }

}

void ParkingSceneModel::reloadSceneData()
{
    auto change = startBigChange();

    setEnvModelPort(mCurrentSceneEntry->sceneData.emModel);
    setTargetPosesPort(mCurrentSceneEntry->sceneData.targetPoseIn);
    setEgoMotionPort(mCurrentSceneEntry->sceneData.egoMotion);
    setParkingBoxPort(mCurrentSceneEntry->sceneData.parkingBoxPort);
    setTrjplaMetaData(mCurrentSceneEntry->sceneData.metaData);
}

void ParkingSceneModel::saveEM(QString filename, bool forFakeEM)
{
    // update internal cache
    getEnvModelPort();
    getEgoMotionPort();
    getTargetPosesPort();
    getParkingBoxPort();
    getTrjplaMetaData();

    // copied from pep_demo/monitormenus.cpp
    std::string filenamestr = filename.toStdString();
    //
    // Open file
    //
    FILE * pFile = fopen(filenamestr.c_str(), "w");
    fprintf(pFile, "{\n");
    fprintf(pFile, "    \"Input\": [\n");

    unsigned int stepnumber{ 0U };
    const auto totalNumberOfSteps{ mSceneCollection.size() };
    for (const auto& sceneEntry : mSceneCollection) {
        fprintf(pFile, "        /* STEP %d/%d */\n", ++stepnumber, totalNumberOfSteps);
        fprintf(pFile, "        {\n");

        si::ApEnvModelPort EM = sceneEntry.sceneData.emModel;
        bool writeApEnvPort{ EM.numberOfStaticObjects_u8 != 0 || EM.egoVehiclePoseForAP.yaw_rad != 0.0F || EM.egoVehiclePoseForAP.x_dir != 0.0F || EM.egoVehiclePoseForAP.y_dir != 0.0F };
#ifndef ULTRASONIC_ONLY
        writeApEnvPort |= EM.numberOfParkMarkings_u8 != 0;
#endif
        if (writeApEnvPort) {
            fprintf(pFile, "            \"ApEnvModelPort\": {\n");
            fprintf(pFile, "                \"numberOfStaticObjects_u8\": %d,\n", EM.numberOfStaticObjects_u8);
            fprintf(pFile, "                \"numberOfDynamicObjects_u8\": %d,\n", EM.numberOfDynamicObjects_u8);
#ifndef ULTRASONIC_ONLY
            fprintf(pFile, "                \"numberOfParkMarkings_u8\": %d,\n", EM.numberOfParkMarkings_u8);
#endif
            fprintf(pFile, "                \"staticObjects\": [");
            for (uint32_t iP = 0; iP < EM.numberOfStaticObjects_u8; ++iP) {
                if (iP > 0) fprintf(pFile, ",");
                fprintf(pFile, "\n");
                si::StaticObjectSerializable const & p = EM.staticObjects[iP];
                fprintf(pFile, "                    {\n");
                fprintf(pFile, "                        \"refObjID_nu\": %d,\n", p.refObjID_nu);
#ifndef ULTRASONIC_ONLY
                fprintf(pFile, "                        \"refObjClass_nu\": %d, /* %s */\n", static_cast<uint32_t>(p.refObjClass_nu), ObjClass_to_string(p.refObjClass_nu));
#endif
                fprintf(pFile, "                        \"existenceProb_perc\": %d,\n", p.existenceProb_perc);
#ifndef ULTRASONIC_ONLY
                fprintf(pFile, "                        \"measurementPrinciple_nu\": %d,\n", static_cast<uint32_t>(p.measurementPrinciple_nu));
#endif
                fprintf(pFile, "                        \"objAgeInCycles_nu\": %d,\n", p.objAgeInCycles_nu);

                fprintf(pFile, "                        \"objMeasLastUpdateInCycles_nu\": %d,\n", p.objMeasLastUpdateInCycles_nu);

                fprintf(pFile, "                        \"objTrendLastUpdateInCycles_nu\": %d,\n", p.objTrendLastUpdateInCycles_nu);

                fprintf(pFile, "                        \"objTrend_nu\": %d,\n", static_cast<uint32_t>(p.objTrend_nu));

                fprintf(pFile, "                        \"readFromNVRAM_nu\": %s,\n", (p.readFromNVRAM_nu) ? "true" : "false");

                fprintf(pFile, "                        \"objShape_m\": [");
                for (uint32_t iV = 0; iV < p.objShape_m.actualSize; ++iV) {
                    if (iV > 0) fprintf(pFile, ",");
                    fprintf(pFile, "\n");
                    fprintf(pFile, "                                [%.5f, %.5f]", p.objShape_m.array[iV].x_dir, p.objShape_m.array[iV].y_dir);
                }
                fprintf(pFile, "\n");
                fprintf(pFile, "                        ],\n"); // Closes "objShape_m": [-
                fprintf(pFile, "                        \"objHeightClass_nu\": %d,  /* %s */\n", static_cast<uint32_t>(p.objHeightClass_nu), ObjHeight_to_string(p.objHeightClass_nu));
                fprintf(pFile, "                        \"objHeightClassConfidence_perc\": %d\n", p.objHeightClassConfidence_perc);

                fprintf(pFile, "                    }");
            }
            fprintf(pFile, "\n");
            fprintf(pFile, "                ],\n"); // Closes "staticObjects": [-


#ifndef ULTRASONIC_ONLY
            fprintf(pFile, "                \"parkingSpaceMarkings\": [");
            for (uint32_t iP = 0; iP < EM.numberOfParkMarkings_u8; ++iP) {
                if (iP > 0) fprintf(pFile, ",");
                fprintf(pFile, "\n");
                auto const & psm = EM.parkingSpaceMarkings[iP];
                fprintf(pFile, "                    {\n");

                fprintf(pFile, "                        \"pos_m\": [\n");
                fprintf(pFile, "                            [%.3f, %.3f],\n", static_cast<float32_t>(psm.pos_m.array[0].x_dir), static_cast<float32_t>(psm.pos_m.array[0].y_dir));
                fprintf(pFile, "                            [%.3f, %.3f]\n", static_cast<float32_t>(psm.pos_m.array[1].x_dir), static_cast<float32_t>(psm.pos_m.array[1].y_dir));
                fprintf(pFile, "                        ],\n");
                fprintf(pFile, "                        \"type_nu\": %d,    /* %s */\n", static_cast<uint8_t>(psm.type_nu), PLT_to_string(psm.type_nu));
                fprintf(pFile, "                        \"width_m\": %.3f,\n", psm.width_m);
                fprintf(pFile, "                        \"existenceProb_perc\": %d\n", psm.existenceProb_perc);
                fprintf(pFile, "                    }");
            }
            fprintf(pFile, "\n");
            fprintf(pFile, "                ],\n"); // Closes "parkingSpaceMarkings": [-

            si::RoadDescription roadDescr = EM.roadDescription;
            if (roadDescr.laneBoundaries->laneShape.actualSize != 0) {
                fprintf(pFile, "                \"roadDescription\": {\n");
                fprintf(pFile, "                    \"laneBoundaries\": [");
                for (uint32_t iP = 0; iP < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_LANE_BOUNDARIES_NU; ++iP) {
                    auto const & lb = roadDescr.laneBoundaries[iP];
                    if (lb.laneShape.actualSize != 0) {
                        if (iP > 0) fprintf(pFile, ",");
                        fprintf(pFile, "\n");
                        fprintf(pFile, "                       {\n");
                        fprintf(pFile, "                        \"laneShape\": [");
                        for (uint32_t iPls = 0; iPls < lb.laneShape.actualSize; ++iPls) {
                            if (iPls > 0) fprintf(pFile, ",");
                            fprintf(pFile, "\n");
                            auto const & lbs = lb.laneShape.array[iPls];
                            fprintf(pFile, "                            [%.3f, %.3f]", static_cast<float32_t>(lbs.x_dir), static_cast<float32_t>(lbs.y_dir));
                        }
                        fprintf(pFile, "\n");
                        fprintf(pFile, "                        ],\n"); // Closes "laneShape": [-
                        fprintf(pFile, "                        \"estimationState_nu\": %d,  /* %s */\n", static_cast<uint8_t>(lb.estimationState_nu), LBEst_to_string(lb.estimationState_nu));
                        fprintf(pFile, "                        \"type_nu\": %d  /* %s */\n", static_cast<uint8_t>(lb.type_nu), Lbt_to_string(lb.type_nu));
                        fprintf(pFile, "                       }");
                    }
                }
                fprintf(pFile, "\n");
                fprintf(pFile, "                    ],\n"); // Closes "laneBoundaries"
                fprintf(pFile, "                    \"lanes\": [");
                for (uint32_t iP = 0; iP < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_LANES_NU; ++iP) {
                    auto const &lane = roadDescr.lanes[iP];
                    if (lane.laneValid_nu) {
                        if (iP > 0) fprintf(pFile, ",");
                        fprintf(pFile, "\n");
                        fprintf(pFile, "                        {\n");
                        fprintf(pFile, "                            \"idxLeftLaneBoundary_nu\": %d,\n", lane.idxLeftLaneBoundary_nu);
                        fprintf(pFile, "                            \"idxRightLaneBoundary_nu\": %d,\n", lane.idxRightLaneBoundary_nu);
                        fprintf(pFile, "                            \"laneValid_nu\": %s,\n", lane.laneValid_nu ? "true" : "false");
                        fprintf(pFile, "                            \"orientationAngle_rad\": %.5f,\n", lane.orientationAngle_rad);
                        fprintf(pFile, "                            \"lanePositionType_nu\": %d,  /* %s */\n", static_cast<uint8_t>(lane.lanePositionType_nu), LPt_to_string(lane.lanePositionType_nu));
                        fprintf(pFile, "                            \"isOneWayLane_nu\": %s,\n", lane.isOneWayLane_nu ? "true" : "false");
                        fprintf(pFile, "                            \"isDeadEnd_nu\": %s\n", lane.isDeadEnd_nu ? "true" : "false");
                        fprintf(pFile, "                        }");
                    }
                }
                fprintf(pFile, "  \n");
                fprintf(pFile, "                    ]\n"); // Closes "lane"
                fprintf(pFile, "                },\n"); // Closes "roadDescription"
            }
#endif
            LSM_GEOML::Pose const & trafoToOdo = EM.transformationToOdometry;
            if (trafoToOdo.Yaw_rad() != 0.0F || trafoToOdo.Pos().x() != 0.0F || trafoToOdo.Pos().y() != 0) {
                fprintf(pFile, "                \"transformationToOdometry\": {\n");
                fprintf(pFile, "                    \"pos\": [%.5f, %.5f],\n", trafoToOdo.Pos().x(), trafoToOdo.Pos().y());
                fprintf(pFile, "                    \"yaw_rad\": %.5f\n", trafoToOdo.Yaw_rad());
                fprintf(pFile, "                },\n"); // Closes "transformationToOdometry"
            }
            LSM_GEOML::Pose const & trafo = EM.resetOriginResult.originTransformation;
            if (trafo.Yaw_rad() != 0.0F || trafo.Pos().x() != 0.0F || trafo.Pos().y() != 0 || EM.resetOriginResult.resetCounter_nu != 0) {
                fprintf(pFile, "                \"resetOriginResult\": {\n");
                fprintf(pFile, "                    \"originTransformation\": {\n");
                fprintf(pFile, "                        \"pos\": [%.5f, %.5f],\n", trafo.Pos().x(), trafo.Pos().y());
                fprintf(pFile, "                        \"yaw_rad\": %.5f\n", trafo.Yaw_rad());
                fprintf(pFile, "                    }\n"); // Closes "originTransformation"
                fprintf(pFile, "                },\n"); // Closes "resetOriginResult"
            }
            fprintf(pFile, "                \"egoVehiclePoseForAP\": {\n");
            LSM_GEOML::Pose const & pose = EM.egoVehiclePoseForAP;
            fprintf(pFile, "                    \"pos\": [%.5f, %.5f],\n", pose.Pos().x(), pose.Pos().y());
            fprintf(pFile, "                    \"yaw_rad\": %.5f\n", pose.Yaw_rad());
            fprintf(pFile, "                }\n"); // Closes "egoVehiclePoseForAP"

            fprintf(pFile, "            },\n"); // Closes "ApEnvModelPort": {-
        }
        // Write si::ApParkingBoxPort (do not use dfi data!)

        fprintf(pFile, "            \"ApParkingBoxPort\": {\n");
        fprintf(pFile, "                \"parkingBoxes\": [\n");
        for (uint32_t iP = 0; iP < sceneEntry.sceneData.parkingBoxPort.numValidParkingBoxes_nu; ++iP) {
            if (iP > 0) fprintf(pFile, ",");
            fprintf(pFile, "\n");
            si::ParkingBoxSerializable const & p = sceneEntry.sceneData.parkingBoxPort.parkingBoxes[iP];
            fprintf(pFile, "                    {\n");
            fprintf(pFile, "                        \"parkingBoxID_nu\": %d,\n", p.parkingBoxID_nu);

            fprintf(pFile, "                        \"slotCoordinates_m\": [");
            for (uint32_t iV = 0; iV < 4; ++iV) {
                if (iV > 0) fprintf(pFile, ",");
                fprintf(pFile, "\n");
                fprintf(pFile, "                            [%.3f, %.3f]", p.slotCoordinates_m.array[iV].x_dir, p.slotCoordinates_m.array[iV].y_dir);
            }
            fprintf(pFile, "\n");
            fprintf(pFile, "                        ],\n"); // Closes "pos_m": [-

            fprintf(pFile, "                        \"existenceProb_perc\": %d,\n", p.existenceProb_perc);

            fprintf(pFile, "                        \"parkingScenario_nu\": %d, /* %s */\n", static_cast<uint32_t>(p.parkingScenario_nu), ParkingScen_to_string(p.parkingScenario_nu));

            fprintf(pFile, "                        \"delimiters\": [");
            for (uint32_t i = 0; i < p.numValidDelimiters_nu; ++i) {
                const auto &delim = p.delimiters[i];
                if (i > 0) fprintf(pFile, ",");
                fprintf(pFile, "\n");
                fprintf(pFile, "                            {\n");
                fprintf(pFile, "                                \"indexInList_nu\": %d,\n", delim.indexInList_nu);
#ifndef ULTRASONIC_ONLY
                fprintf(pFile, "                                \"delimiterType_nu\": %u,   /* %s */\n", (uint8_t)delim.delimiterType_nu, DelimType_to_string(delim.delimiterType_nu));
#endif
                fprintf(pFile, "                                \"delimitingSide_nu\": %u,   /* %s */\n", (uint8_t)delim.delimitingSide_nu, DelimSide_to_string(delim.delimitingSide_nu));
                fprintf(pFile, "                                \"virtLineIdx_nu\": %u\n", delim.virtLineIdx_nu);
                fprintf(pFile, "                            }");
            }
            fprintf(pFile, "\n");
            fprintf(pFile, "                        ],\n"); // Closes "delimiters": [-

            fprintf(pFile, "                        \"numValidDelimiters_nu\": %d,\n", p.numValidDelimiters_nu);
            fprintf(pFile, "                        \"numVirtualLines_nu\": %d,\n", p.numVirtualLines_nu);
            fprintf(pFile, "                        \"virtualLine\": [");
            for (uint32_t i = 0; i < p.numVirtualLines_nu; ++i) {
                si::VirtualLineSerializable l = p.virtualLines[i];
                if (i > 0) fprintf(pFile, ",");
                fprintf(pFile, "\n");
                fprintf(pFile, "                            {\n");
                fprintf(pFile, "                                \"virtLineVertices_m\": [ ");
                fprintf(pFile, "[%.3f, %.3f], [%.3f, %.3f] ]\n", l.virtLineVertices_m.array[0].x_dir, l.virtLineVertices_m.array[0].y_dir, l.virtLineVertices_m.array[1].x_dir, l.virtLineVertices_m.array[1].y_dir);
                fprintf(pFile, "                            }");
            }
            fprintf(pFile, "\n");
            fprintf(pFile, "                        ]\n"); // Closes "virtualLine": [-
            fprintf(pFile, "                    }");
        }
        fprintf(pFile, "\n");
        fprintf(pFile, "                ],\n"); // Closes "parkingBoxes": [-

        fprintf(pFile, "                \"numValidParkingBoxes_nu\": %d\n", sceneEntry.sceneData.parkingBoxPort.numValidParkingBoxes_nu);

        fprintf(pFile, "            },\n"); // Closes "ParkingBoxPort": {-

        fprintf(pFile, "            \"TrjplaMetaData\": {\n");
        ap_tp::TrjplaMetaData metaData = sceneEntry.sceneData.metaData;
            if (metaData.drivingDirection != ap_common::DrivingDirection::DIRECTION_UNKNOWN) {
                fprintf(pFile, "                \"DrivingDirection\": %u,   /* %s */\n", (uint8_t)metaData.drivingDirection, DrivingDir_to_string(metaData.drivingDirection));
            }
        fprintf(pFile, "                \"SteeringDirection\": %u  /* %s */\n", (uint8_t)metaData.steeringDirection, SteeringDir_to_string(metaData.steeringDirection));
        fprintf(pFile, "            }\n"); // Closes "TrjplaMetaData": {-

        fprintf(pFile, "        }");
        if (&sceneEntry != &mSceneCollection.back()) {
            fprintf(pFile, ",");
        }
        fprintf(pFile, "\n");
    }

        fprintf(pFile, "    ]\n");


        if (!forFakeEM) {
            fprintf(pFile, ",\n");
            fprintf(pFile, "    \"ExpectedOutput\": [\n");
            stepnumber = 0;
            for (const auto& SceneEntry : mSceneCollection) {
                fprintf(pFile, "        /* STEP %d/%d */\n", ++stepnumber, totalNumberOfSteps);
                fprintf(pFile, "        {\n");
                // write TargetPosesPort
                fprintf(pFile, "            \"TargetPosesPort\": {\n");
                ap_tp::TargetPosesPort tpIn = SceneEntry.sceneData.targetPoseIn;
                fprintf(pFile, "                \"targetPoses\": [");

                // TODO: reorder targetPoses if not in right order ! wrt numValidPoses!
                for (uint32_t iP = 0; iP < tpIn.numValidPoses; ++iP) {
                    if (iP > 0) fprintf(pFile, ",");
                    fprintf(pFile, "\n");

                    auto const& tp = tpIn.targetPoses[iP];
                    fprintf(pFile, "                    {\n");
                    fprintf(pFile, "                        \"pose\": {\n");
                    fprintf(pFile, "                            \"pos\": {\n");
                    fprintf(pFile, "                                 \"x\": %.3f,\n", tp.pose.x_dir);
                    fprintf(pFile, "                                 \"y\": %.3f\n", tp.pose.y_dir);
                    fprintf(pFile, "                            },\n"); // Closes pos
                    fprintf(pFile, "                            \"yaw_rad\": %f\n", tp.pose.yaw_rad);
                    fprintf(pFile, "                        }, \n"); // Closes pose
                    fprintf(pFile, "                        \"pose_ID\": %d,\n", static_cast<uint8_t>(tp.pose_ID));
                    fprintf(pFile, "                        \"relatedParkingBoxID\": %d,\n", static_cast<uint8_t>(tp.relatedParkingBoxID));
                    fprintf(pFile, "                        \"type\": %d,   /* %s */\n", int(tp.type), TPType_to_string(tp.type));
                    fprintf(pFile, "                        \"poseFailReason\": %d,   /* %s */\n", int(tp.poseFailReason), TPPoseFailReason_to_string(tp.poseFailReason));
                    fprintf(pFile, "                        \"targetSide\": %d,   /* %s */\n", int(tp.targetSide), TPSide_to_string(tp.targetSide));
                    fprintf(pFile, "                        \"reachableStatus\": %d     /* %s */\n", int(tp.reachableStatus), ReachStatus_to_string(tp.reachableStatus));
                    fprintf(pFile, "                    }");
                }
                fprintf(pFile, "\n                ],\n"); // Closes "targetPoses": [-
                fprintf(pFile, "                \"numValidPoses\": %d,\n", static_cast<uint8_t>(tpIn.numValidPoses));
                fprintf(pFile, "                \"failReason\": %d,   /* %s */\n", int(tpIn.failReason), FailR_to_string(tpIn.failReason));
                fprintf(pFile, "                \"anyPathFound\": %s,\n", (tpIn.anyPathFound) ? "true" : "false");
                fprintf(pFile, "                \"selectedPoseData\": {\n");
                fprintf(pFile, "                   \"selectionStatus\": %d,\n", int(tpIn.selectedPoseData.selectionStatus));
                fprintf(pFile, "                   \"reachedStatus\": %d,   /* %s */\n", int(tpIn.selectedPoseData.reachedStatus), ReachedStatus_to_string(tpIn.selectedPoseData.reachedStatus));
                fprintf(pFile, "                   \"distanceToStart_m\": %.3f\n", tpIn.selectedPoseData.distanceToStart_m);
                fprintf(pFile, "                }\n"); // Closes selectedPoseData
                fprintf(pFile, "            }\n"); // Closes "TargetPosesPort": {-

                fprintf(pFile, "        }");
                if (&SceneEntry != &mSceneCollection.back()) {
                    fprintf(pFile, ",");
                }
                fprintf(pFile, "\n");
            }
        }
    fprintf(pFile, "    ]\n");

    //
    // Close file
    //
    fprintf(pFile, "}\n");
    fclose(pFile);

    mLoadedEMFilename = filename;
    emit loadedEMPathChanged(mLoadedEMFilename);
}

void ParkingSceneModel::saveReach(QString filename)
{

    // copied from pep_demo/monitormenus.cpp
    std::string filenamestr = filename.toStdString();
    //
    // Open file
    //
    FILE * pFile = fopen(filenamestr.c_str(), "w");

    const auto model = getTargetPoseReachableAreaModel();
    fprintf(pFile, "{\n");
    fprintf(pFile, "    \"reachability_area\": [");
    const auto poses = model->getTargetReachablePoses();
    for (uint32_t iP = 0U; iP < static_cast<uint32_t>(poses.size()); ++iP) {
        if (iP > 0) fprintf(pFile, ",");
        fprintf(pFile, "\n");
        const uint8_t reachable = poses[iP].reachable ? 0 : 1;
        fprintf(pFile, "        [[%.3f, %.3f, %.3f], %d, %d]", poses[iP].pose.x(), poses[iP].pose.y(), poses[iP].pose.z(), reachable, poses[iP].nrStrokes);
    }
    fprintf(pFile, "\n    ], \n");
    fprintf(pFile, "    \"x_range\": [%.3f, %.3f, %.3f],\n", model->getXstart(), model->getXend(), model->getXstep());
    fprintf(pFile, "    \"y_range\": [%.3f, %.3f, %.3f],\n", model->getYstart(), model->getYend(), model->getYstep());
    fprintf(pFile, "    \"yaw_range\": [%.3f, %.3f, %.3f]\n", model->getYawAngleDegStart(), model->getYawAngleDegEnd(), model->getYawAngleDegStep());
    fprintf(pFile, "}\n");
    fclose(pFile);
}
void ParkingSceneModel::setEnvModelPort(const si::ApEnvModelPort &em)
{
    auto change = startBigChange();

    mCurrentSceneEntry->sceneData.emModel = em;

    int numV = 0;
    for(size_t i = 0; i < mStaticStructuresModels.size(); i++) {
        if (i < mCurrentSceneEntry->sceneData.emModel.numberOfStaticObjects_u8) {
            mStaticStructuresModels.at(i).setStructureData(mCurrentSceneEntry->sceneData.emModel.staticObjects[i]);
            if (mCurrentSceneEntry->sceneData.emModel.staticObjects[i].existenceProb_perc > 0) {
                numV += mCurrentSceneEntry->sceneData.emModel.staticObjects[i].objShape_m.actualSize;
            }
        } else {
            mStaticStructuresModels.at(i).setVisible(false);
        }
    }

#ifndef ULTRASONIC_ONLY
    for(uint8 i = 0; i < mCurrentSceneEntry->sceneData.emModel.numberOfParkMarkings_u8; i++) {
        mParkingSpaceMarkingModels.at(i).setMarking(mCurrentSceneEntry->sceneData.emModel.parkingSpaceMarkings[i]);
    }
#endif
    mStartPoseModel.setPos(mCurrentSceneEntry->sceneData.emModel.egoVehiclePoseForAP.x_dir,
        mCurrentSceneEntry->sceneData.emModel.egoVehiclePoseForAP.y_dir,
        mCurrentSceneEntry->sceneData.emModel.egoVehiclePoseForAP.yaw_rad);
}

void ParkingSceneModel::applyHistoryData()
{
    auto change = startBigChange();

    auto& data = mCurrentSceneEntry->history.at(mCurrentSceneEntry->historyIndex);
    setEnvModelPort(data.emModel);
    setTargetPosesPort(data.targetPoseIn);
    setEgoMotionPort(data.egoMotion);
    setParkingBoxPort(data.parkingBoxPort);
}

const si::ApEnvModelPort* ParkingSceneModel::getEnvModelPort()
{
    memset(mCurrentSceneEntry->sceneData.emModel.staticObjects, 0, sizeof(mCurrentSceneEntry->sceneData.emModel.staticObjects));
    // mCurrentSceneEntry->sceneData.emModel.numValidStaticObj = (uint16_t)std::min<size_t>(mStaticObjectsModel.childCount(), ap_common::AP_G_MAX_NUM_STATIC_OBJ_NU);
    uint8_t numStaticObj{ 0 };
    for(int objectIndex = 0; objectIndex < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_STATIC_OBJ_NU; objectIndex++) {
        auto& s = mCurrentSceneEntry->sceneData.emModel.staticObjects[objectIndex];
        s = mStaticStructuresModels.at(objectIndex).getStructureData();
        if (mStaticStructuresModels.at(objectIndex).isVisible()) { numStaticObj++; }
        s.refObjID_nu = objectIndex;
    }
    mCurrentSceneEntry->sceneData.emModel.numberOfStaticObjects_u8 = numStaticObj;

#ifndef ULTRASONIC_ONLY
    uint8_t numParkMark{ 0 };
    for(size_t i = 0; i < mParkingSpaceMarkingModels.size(); i++) {
        mCurrentSceneEntry->sceneData.emModel.parkingSpaceMarkings[i] = mParkingSpaceMarkingModels.at(i).getMarking();
        if (mParkingSpaceMarkingModels.at(i).isVisible()) { numParkMark++; }
    }
    mCurrentSceneEntry->sceneData.emModel.numberOfParkMarkings_u8 = numParkMark;
#endif

    mCurrentSceneEntry->sceneData.emModel.egoVehiclePoseForAP.x_dir = mStartPoseModel.getPosX_m();
    mCurrentSceneEntry->sceneData.emModel.egoVehiclePoseForAP.y_dir = mStartPoseModel.getPosY_m();
    mCurrentSceneEntry->sceneData.emModel.egoVehiclePoseForAP.yaw_rad = mStartPoseModel.getPosYawAngle_Rad();

    return &mCurrentSceneEntry->sceneData.emModel;
}

void ParkingSceneModel::setEgoMotionPort(const si::EgoMotionPort &egoMotion)
{
    mCurrentSceneEntry->sceneData.egoMotion = egoMotion;
    mStartPoseModel.setSteeringAngle(egoMotion.frontWheelAngle_rad);
}

const si::EgoMotionPort *ParkingSceneModel::getEgoMotionPort()
{
    mCurrentSceneEntry->sceneData.egoMotion.frontWheelAngle_rad = mStartPoseModel.getSteeringAngle();
    return &mCurrentSceneEntry->sceneData.egoMotion;
}

void ParkingSceneModel::setTargetPosesPort(const ap_tp::TargetPosesPort &target)
{
    mCurrentSceneEntry->sceneData.targetPoseIn = target;

    for(size_t i = 0; i < mTargetPoseModels.size(); i++) {
        if (i < target.numValidPoses) {
            mTargetPoseModels.at(i)->setPoseData(target.targetPoses[i]);
            mTargetPoseModels.at(i)->setPoseValid(true);
            mTargetPoseModels.at(i)->setRightDeviation(-mTrajplaParams->taposdParams.AP_T_MIN_LATERAL_DEVIATION_M);
            mTargetPoseModels.at(i)->setLeftDeviation(mTrajplaParams->taposdParams.AP_T_MIN_LATERAL_DEVIATION_M);
        } else {
            mTargetPoseModels.at(i)->setPoseValid(false);
        }
            //memset(&mTargetPoseModels.at(i), 0, sizeof(mTargetPoseModels.at(i)));

    }
}

const ap_tp::TargetPosesPort* ParkingSceneModel::getTargetPosesPort()
{
    uint8_t numValidTargetPoses = 0;
    for(size_t i = 0; i < mTargetPoseModels.size(); i++) {
        mCurrentSceneEntry->sceneData.targetPoseIn.targetPoses[i] = mTargetPoseModels.at(i)->getPoseData();
        mCurrentSceneEntry->sceneData.targetPoseIn.targetPoses[i].pose_ID = static_cast<uint8_t>(i);
        if (mTargetPoseModels.at(i)->isPoseValid()) {
            numValidTargetPoses++;
        }
    }
    mCurrentSceneEntry->sceneData.targetPoseIn.numValidPoses = numValidTargetPoses;

    return &mCurrentSceneEntry->sceneData.targetPoseIn;
}

void ParkingSceneModel::setParkingBoxPort(const si::ApParkingBoxPort &port)
{
    auto change = startBigChange();

    mCurrentSceneEntry->sceneData.parkingBoxPort = port;
    mParkingBoxCollection.setParkingBoxes(port.numValidParkingBoxes_nu, port.parkingBoxes);
}

const si::ApParkingBoxPort *ParkingSceneModel::getParkingBoxPort()
{
    mCurrentSceneEntry->sceneData.parkingBoxPort.numValidParkingBoxes_nu = mParkingBoxCollection.childCount();
    for(int i = 0; i < mCurrentSceneEntry->sceneData.parkingBoxPort.numValidParkingBoxes_nu; i++) {
        auto model = mParkingBoxCollection.getChildModel(i);
        auto box = model->getBoxData();
        mCurrentSceneEntry->sceneData.parkingBoxPort.parkingBoxes[i] = box;
        // count virtual lines if one point is not equal origin
        uint8_t numVirtLines{ 0 };
        for (auto virtLin : box.virtualLines) {
            if (virtLin.virtLineVertices_m.actualSize != 0) {
                if (virtLin.virtLineVertices_m.array[0].x_dir != 0.0F || virtLin.virtLineVertices_m.array[0].x_dir != 0.0F
                    || virtLin.virtLineVertices_m.array[1].x_dir != 0.0F || virtLin.virtLineVertices_m.array[1].y_dir != 0.0F) {
                    numVirtLines++;
                }
            }
        }
        mCurrentSceneEntry->sceneData.parkingBoxPort.parkingBoxes[i].numVirtualLines_nu = numVirtLines;
        uint8_t numDelim{ 0 };
        // count delimiters if side is set
        for (auto delim : box.delimiters) {
            if (delim.delimitingSide_nu != si::RelativeLocationToParkingBox::UNDEFINED_EDGE) {
                numDelim++;
            }
        }
        mCurrentSceneEntry->sceneData.parkingBoxPort.parkingBoxes[i].numValidDelimiters_nu = numDelim;
    }

    return &mCurrentSceneEntry->sceneData.parkingBoxPort;
}

void ParkingSceneModel::setTrjplaMetaData(const ap_tp::TrjplaMetaData& metaData) {

    mCurrentSceneEntry->sceneData.metaData = metaData;
    mStartPoseModel.setDrivingDirection(metaData.drivingDirection);
    mStartPoseModel.setSteeringDirection(metaData.steeringDirection);
    setVehInflRadius_m(metaData.inflationDist_m);
    setReplanTrigger(metaData.replanTrigger);
}

const ap_tp::TrjplaMetaData *ParkingSceneModel::getTrjplaMetaData() {
    mCurrentSceneEntry->sceneData.metaData.drivingDirection = static_cast<ap_common::DrivingDirection>(mStartPoseModel.getDrivingDirection().asInt());
    mCurrentSceneEntry->sceneData.metaData.steeringDirection = static_cast<ap_common::SteeringDirection>(mStartPoseModel.getSteeringDirection().asInt());
    mCurrentSceneEntry->sceneData.metaData.inflationDist_m = getVehInflRadius_m();
    mCurrentSceneEntry->sceneData.metaData.replanTrigger = static_cast<ap_tp::ReplanTrigger>(getReplanTrigger().asInt());
    return  &mCurrentSceneEntry->sceneData.metaData;
}


void ParkingSceneModel::setSelectedObject(QObject *object)
{
    if(mSelectedObject != object) {
        auto prev = mSelectedObject;
        mSelectedObject = object;

        emit selectedObjectChanged(object, prev);
    }
}

void ParkingSceneModel::setVehicleParameters(const ap_common::Vehicle_Params& params)
{
    mVehicleParams = params;

    emit vehicleParamsChanged(mVehicleParams);
}

int ParkingSceneModel::getSelectedTargetPose() const
{
    return mSelectedPoseIndex;
}

void ParkingSceneModel::setSelectedTargetPose(int index)
{
    if (mSelectedPoseIndex != index) {
        auto prev = mSelectedPoseIndex;
        mSelectedPoseIndex = index;
        emit selectedTargetPoseChanged(prev, mSelectedPoseIndex);
    }

    if(index >= 0 && index < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_TARGET_POSES_NU) {
        mTargetPoseModels.at(mSelectedPoseIndex)->setPoseValid(true);
    }
}

TargetPoseModel *ParkingSceneModel::getSelectedTargetPoseModel()
{
    if (mSelectedPoseIndex >= 0 && mSelectedPoseIndex < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_TARGET_POSES_NU
            && mTargetPoseModels.at(mSelectedPoseIndex)->isPoseValid()
        ) {
        return mTargetPoseModels.at(mSelectedPoseIndex).get();
    } else {
        return nullptr;
    }
}

void ParkingSceneModel::createHistoryPoint()
{
    mCurrentSceneEntry->createHistoryPoint(SceneData {*getEnvModelPort(), *getTargetPosesPort(), *getEgoMotionPort(), *getParkingBoxPort(), *getTrjplaMetaData()});
}

void ParkingSceneModel::undo()
{
    if(mCurrentSceneEntry->history.size() > 1 && mCurrentSceneEntry->historyIndex > 0) {
        mCurrentSceneEntry->historyIndex--;
        applyHistoryData();
    }
}

void ParkingSceneModel::redo()
{
    if(mCurrentSceneEntry->history.size() > 1 && mCurrentSceneEntry->historyIndex < mCurrentSceneEntry->history.size()-1) {
        mCurrentSceneEntry->historyIndex++;
        applyHistoryData();
    }
}

bool ParkingSceneModel::isFollowingPath() const
{
    return mTrajectoriesModel.getSelectedTrajectory() && !mTrajectoriesModel.getSelectedTrajectory()->empty();
}

void ParkingSceneModel::setVehInflRadius_m(float radius)
{
    if (mVehInflRadiusOverride != radius) {
        mVehInflRadiusOverride = radius;
        emit vehInflRadiusChanged(radius);
    }
}

void ParkingSceneModel::resetTargetPoseReachableArea(void){
    mTargetPoseReachableAreaModel.setAreaReady2D(false);
    return mTargetPoseReachableAreaModel.reset();
}

void ParkingSceneModel::selectEMbyIndex(int index)
{
    index = std::max(0, std::min(index, mSceneCollection.size()-1));

    if(index != mSelectedEMIndex) {
        mSelectedEMIndex = index;
        mCurrentSceneEntry = &mSceneCollection[mSelectedEMIndex];
        reloadSceneData();
        emit selectedEMIndexChanged(mSelectedEMIndex);
    }
}

QAbstractListModel *ParkingSceneModel::getLoadedScenesModel()
{
    return &mLoadedScenesListModel;
}

void ParkingSceneModel::addNewSceneEntry()
{
    mSceneCollection.append(SceneEntry {mCurrentSceneEntry->sceneData });
    QStringList stringList {mLoadedScenesListModel.stringList()};
    stringList.append(QString::number(stringList.size()));
    mLoadedScenesListModel.setStringList(stringList);
    mSelectedEMIndex = mSceneCollection.size()-1;
    mCurrentSceneEntry = &mSceneCollection[mSelectedEMIndex];
    reloadSceneData();
    emit selectedEMIndexChanged(mSelectedEMIndex);
}

void ParkingSceneModel::removeSelectedSceneEntry()
{
    if(mSceneCollection.size() > 1) {
        mSceneCollection.removeAt(mSelectedEMIndex);
        QStringList stringList {mLoadedScenesListModel.stringList()};
        stringList.removeAt(mSelectedEMIndex);

        if(mSelectedEMIndex >= mSceneCollection.size()) {
            mSelectedEMIndex--;
        }
        else{
            for(int i=mSelectedEMIndex;i<stringList.size();i++){
                stringList.replace(i,QString::number(stringList.at(i).toUInt()-1));
            }
        }
        mCurrentSceneEntry = &mSceneCollection[mSelectedEMIndex];
        reloadSceneData();
        mLoadedScenesListModel.setStringList(stringList);
        emit selectedEMIndexChanged(mSelectedEMIndex);
    }
}

void ParkingSceneModel::onDataChanged()
{
    if(mDoingBigChange <= 0) {
        emit dataChanged();
    }
}

void ParkingSceneModel::setTargetPoseReachableAreaReady2D(bool areaReady){
    mTargetPoseReachableAreaModel.setAreaReady2D(areaReady);
}

void ParkingSceneModel::setTargetPoseReachableAreaReady3D(void){
    emit targetPoseReachabilityAreaReady3D();
}

void ParkingSceneModel::setReplanTrigger(const EnumProperty& prop) {
    mReplanTrigger = (ap_tp::ReplanTrigger)prop.asInt();
}

bool ParkingSceneModel::isReplanning(void){
    return mIsReplanning;
}

void ParkingSceneModel::setIsReplanning(bool isReplanning){
    mIsReplanning= isReplanning;
}

ParkingSceneModel::ChangeGuard ParkingSceneModel::startBigChange()
{
    return ChangeGuard(this);
}
