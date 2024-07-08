#ifndef VISUCLIENTINTERFACE_H
#define VISUCLIENTINTERFACE_H

#include <string>
#include <list>
#include <si/ap_env_model_port.h>
#include <si/ap_parking_box_port.h>
#include <si/ego_motion_port.h>
#include <ap_common/vehicle_params.h>
#include <geoml/Polygon2D.h>
#include <ParkingPath.h>
#include <GeometricPath.h>
#include <ManeuverSet.h>

class VisuClientInterface
{
public:
    /**
     * @brief get the client interface if configured
     * @return interface object or nullptr
     */
    static VisuClientInterface* get();

    virtual ~VisuClientInterface() = default;

    static void setInterface(VisuClientInterface* instance);

    /* virtual QPainter* beginPaint() = 0;
    virtual void endPaint() = 0; */

    virtual void clearDraw() = 0;

    virtual void drawPoint(float x, float y, const std::string& name) = 0;
    virtual void drawLine(float x1, float y1, float x2, float y2, const std::string& name) = 0;
    virtual void drawCircle(float x, float y, float r, const std::string& name) = 0;
    virtual void drawPolygon(const cml::Vec2Df* begin, const cml::Vec2Df* end, const std::string& name) = 0;

    virtual void drawDebugVehicle(float x, float y, float yaw) = 0;
    virtual void drawDebugVehicleRadius(float radius) = 0;

    virtual void drawPath(const ap_tp::ParkingPath& path, const std::string& name) = 0;
    virtual void drawPathSet(const std::list<ap_tp::ParkingPath>& pathSet, const std::string& name) = 0;

    virtual void drawGeomPath(const ap_tp::GeometricPath& geomPath, const std::string& name) = 0;
    virtual void drawManeuverSet(const ap_tp::ManeuverSet& meneuverSet, const std::string& name) = 0;

    virtual void updateEM(const si::ApEnvModelPort& em) = 0;
    virtual void updateParkBox(const si::ApParkingBoxPort& pbPort) = 0;
    virtual void updateEgoPort(const si::ApEnvModelPort& emData, const si::EgoMotionPort& egoPort) = 0;
    virtual void updateTargetPosesPort(const ap_tp::TargetPosesPort& targetPoses) = 0;

    virtual void updateVehicleInflation(float radius) = 0;
    virtual void updateVehicleParams(const ap_common::Vehicle_Params& params) = 0;
protected:
    VisuClientInterface();

private:
    static VisuClientInterface * smInstance;
};

inline void VISU_CLEAR_DRAW() {
    if(auto visu = VisuClientInterface::get()) {
        visu->clearDraw();
    }
}

inline void VISU_DRAW_POINT(float x, float y, const std::string& name) {
    if(auto visu = VisuClientInterface::get()) {
        visu->drawPoint(x, y, name);
    }
}

inline void VISU_DRAW_POINT(const cml::Vec2Df& pt, const std::string& name) {
    if (auto visu = VisuClientInterface::get()) {
        visu->drawPoint(pt.x(), pt.y(), name);
    }
}

inline void VISU_DRAW_LINE(float x1, float y1, float x2, float y2, const std::string& name) {
    if (auto visu = VisuClientInterface::get()) {
        visu->drawLine(x1, y1, x2, y2, name);
    }
}

inline void VISU_DRAW_LINE(const cml::Vec2Df& pt1, const cml::Vec2Df& pt2, const std::string& name) {
    if (auto visu = VisuClientInterface::get()) {
        visu->drawLine(pt1.x(), pt1.y(), pt2.x(), pt2.y(), name);
    }
}

inline void VISU_DRAW_CIRCLE(float x, float y, float r, const std::string& name) {
    if (auto visu = VisuClientInterface::get()) {
        visu->drawCircle(x, y, r, name);
    }
}

inline void VISU_DRAW_CIRCLE(const cml::Vec2Df& pt, float r, const std::string& name) {
    if (auto visu = VisuClientInterface::get()) {
        visu->drawCircle(pt.x(), pt.y(), r, name);
    }
}

template<uint8_t Size>
inline void VISU_DRAW_POLYGON(const LSM_GEOML::SizeHandledArray<cml::Vec2Df, Size>& poly, const std::string& name) {
    if (auto visu = VisuClientInterface::get()) {
        if (poly.getSize() > 0) {
            visu->drawPolygon(&*poly.begin(), (&*poly.begin()) + poly.getSize(), name);
        }
    }
}

inline void VISU_DRAW_DEBUG_VEHICLE(float x, float y, float yaw) {
    if (auto visu = VisuClientInterface::get()) {
        visu->drawDebugVehicle(x, y, yaw);
    }
}

inline void VISU_DRAW_DEBUG_VEHICLE_RADIUS(float r) {
    if (auto visu = VisuClientInterface::get()) {
        visu->drawDebugVehicleRadius(r);
    }
}

inline void VISU_DRAW_PATH(const ap_tp::ParkingPath& path, const std::string& name) {
    if (auto visu = VisuClientInterface::get()) {
        visu->drawPath(path, name);
    }
}

inline void VISU_DRAW_PATH_SET(const std::list<ap_tp::ParkingPath>& pathSet, const std::string& name) {
    if (auto visu = VisuClientInterface::get()) {
        visu->drawPathSet(pathSet, name);
    }
}

inline void VISU_DRAW_GEOM_PATH(const ap_tp::GeometricPath& geomPath, const std::string& name) {
    if (auto visu = VisuClientInterface::get()) {
        visu->drawGeomPath(geomPath, name);
    }
}

inline void VISU_DRAW_MANEUVER_SET(const ap_tp::ManeuverSet& maneuverSet, const std::string& name) {
    if (auto visu = VisuClientInterface::get()) {
        visu->drawManeuverSet(maneuverSet, name);
    }
}

inline void VISU_UPDATE_EM(const si::ApEnvModelPort& em) {
    if (auto visu = VisuClientInterface::get()) {
        visu->updateEM(em);
    }
}

inline void VISU_UPDATE_PARKBOX(const si::ApParkingBoxPort& pbPort) {
    if (auto visu = VisuClientInterface::get()) {
        visu->updateParkBox(pbPort);
    }
}

inline void VISU_UPDATE_EGO_PORT(const si::ApEnvModelPort& emData, const si::EgoMotionPort& egoPort) {
    if (auto visu = VisuClientInterface::get()) {
        visu->updateEgoPort(emData, egoPort);
    }
}

inline void VISU_UPDATE_TARGET_POSES(const ap_tp::TargetPosesPort& targetPoses) {
    if (auto visu = VisuClientInterface::get()) {
        visu->updateTargetPosesPort(targetPoses);
    }
}

inline void VISU_UPDATE_VEHICLE_INFLATION(float r) {
    if (auto visu = VisuClientInterface::get()) {
        visu->updateVehicleInflation(r);
    }
}

inline void VISU_UPDATE_VEHICLE_PARAMS(const ap_common::Vehicle_Params& params) {
    if (auto visu = VisuClientInterface::get()) {
        visu->updateVehicleParams(params);
    }
}

#endif // VISUCLIENTINTERFACE_H
