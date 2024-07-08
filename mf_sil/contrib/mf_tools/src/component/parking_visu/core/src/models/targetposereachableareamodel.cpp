#include "targetposereachableareamodel.h"
#include <json/json.h>
#include <fstream>
TargetPoseReachableAreaModel::TargetPoseReachableAreaModel(QObject *parent) : QObject(parent)
{
    mTargetReachablePoses.clear();
    mAreaReady2D=false;
}

void TargetPoseReachableAreaModel::reset(void){
    mTargetReachablePoses.clear();
    mAreaReady2D=false;
}

void TargetPoseReachableAreaModel::setPoseInTargetReachableArea(const ReachablePoseEntry &entry){
    mTargetReachablePoses.append(entry);
}

const QList<ReachablePoseEntry>& TargetPoseReachableAreaModel::getTargetReachablePoses(){
    return mTargetReachablePoses;
}
void TargetPoseReachableAreaModel::setAreaReady2D(bool ready){
    if(mAreaReady2D != ready){
        mAreaReady2D= ready;
    }
    emit areaReady2D(ready);
}

bool TargetPoseReachableAreaModel::isAreaReady2D(void){
   return mAreaReady2D;
}

void TargetPoseReachableAreaModel::setSearchArea(float_t xStart,float_t xEnd,float_t yStart,float_t yEnd, float_t yawAngleStart, float_t yawAngleEnd){
    mXstart= xStart;
    mXend= xEnd;
    mYstart= yStart;
    mYend= yEnd;
    mYawAngleDegStart= yawAngleStart;
    mYawAngleDegEnd= yawAngleEnd;
}

float TargetPoseReachableAreaModel::getXstart(void){
    return mXstart;
}

float TargetPoseReachableAreaModel::getXend(void){
    return mXend;
}

float TargetPoseReachableAreaModel::getXstep(void){
    return mXStep;
}

float TargetPoseReachableAreaModel::getYstart(void){
    return mYstart;
}
float TargetPoseReachableAreaModel::getYend(void){
    return mYend;
}

float TargetPoseReachableAreaModel::getYstep(void){
    return mYStep;
}

float TargetPoseReachableAreaModel::getYawAngleDegStart(void){
    return mYawAngleDegStart;
}

float TargetPoseReachableAreaModel::getYawAngleDegEnd(void){
    return mYawAngleDegEnd;
}

float TargetPoseReachableAreaModel::getYawAngleDegStep(void){
    return mYawAngleDegStep;
}

float TargetPoseReachableAreaModel::getYawAngleDegSelected(void){
    return mYawAngleDegSelected;
}

bool TargetPoseReachableAreaModel::isReplanning(void){
    return mIsReplanning;
}


void TargetPoseReachableAreaModel::setXstart(float xStart){
    mXstart= xStart;
}

void TargetPoseReachableAreaModel::setXend(float xEnd){
    mXend= xEnd;
}

void TargetPoseReachableAreaModel::setXstep(float xStep){
    mXStep= xStep;
}

void TargetPoseReachableAreaModel::setYstart(float yStart){
    mYstart= yStart;
}

void TargetPoseReachableAreaModel::setYend(float yEnd){
    mYend= yEnd;
}

void TargetPoseReachableAreaModel::setYstep(float yStep){
    mYStep= yStep;
}

void TargetPoseReachableAreaModel::setYawAngleDegStart(float yawAngleDegStart){
    mYawAngleDegStart=  yawAngleDegStart;
}

void TargetPoseReachableAreaModel::setYawAngleDegEnd(float yawAngleDegEnd){
    mYawAngleDegEnd= yawAngleDegEnd;
}

void TargetPoseReachableAreaModel::setYawAngleDegStep(float yawAngleDegStep){
    mYawAngleDegStep= yawAngleDegStep;
}

void TargetPoseReachableAreaModel::setYawAngleDegSelected(float yawAngleDegSelected){
    mYawAngleDegSelected= yawAngleDegSelected;
}

void TargetPoseReachableAreaModel::setIsReplanning(bool isReplanning){
    mIsReplanning= isReplanning;
}

void TargetPoseReachableAreaModel::loadREACH(QString filename){

    // Load test data from json file
    std::string errs;
    Json::Value root;
    //::Json::Reader reader;
    ::Json::CharReaderBuilder builder;
    // Json config file
    std::ifstream jsonFile(filename.toStdString());
    if (jsonFile.is_open()) {
        // Parse the json config file with the reader and load it into the json value variable
        bool isOk = Json::parseFromStream(builder, jsonFile, &root, &errs);
        if (!isOk) {
            throw std::runtime_error("Could not parse json file: " + filename.toStdString());
            return;
        }
    } else {
        throw std::runtime_error("Could not open json file for reading: " + filename.toStdString());
    }

    reset();
    if (root.isMember("reachability_area")) {
        for (const Json::Value& val : root["reachability_area"]) {
            float x, y, yaw;
            int nrStrokes;
            bool reachable;
            x = val[0][0].asFloat();
            y = val[0][1].asFloat();
            yaw = val[0][2].asFloat();
            if (val[1].asInt() == 0) reachable = true;
            else reachable = false;
            nrStrokes = val[2].asInt();
            QVector3D position{ x,y,yaw };
            ReachablePoseEntry entry{ position, reachable, nrStrokes };
            setPoseInTargetReachableArea(entry);
        }
        setAreaReady2D(true);
    }
    setSearchArea(root["x_range"][0].asFloat(),root["x_range"][1].asFloat(),root["y_range"][0].asFloat(),root["y_range"][1].asFloat(),
                  root["yaw_range"][0].asFloat(), root["yaw_range"][1].asFloat());
    setXstep(root["x_range"][2].asFloat());
    setYstep(root["y_range"][2].asFloat());
    setYawAngleDegStep(root["yaw_range"][2].asFloat());
}
