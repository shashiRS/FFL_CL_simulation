#pragma once

#include "geoml/Polygon2D.h"
#include "MfSilTypes.h"

// Descibes a sensor field-of-view in 2D by a circular sector.
class SensorFov2D {

    LSM_GEOML::Pose mSensorPose{};  // mounting pose of the sensor (i.e. x, y position and yaw angle)
    float32_t mFovRange_m{};        // range (=radius) of the field of view in meters
    float32_t mFovAzimuth_rad{};    // azimuthal opening angle of the field of view in radians
    LSM_GEOML::LineSegment2D mFovLeftEdge{ 0.0f, 0.0f, 0.0f, 0.0f };    // left edge of the sensor FOV circular sector
    LSM_GEOML::LineSegment2D mFovRightEdge{ 0.0f, 0.0f, 0.0f, 0.0f };   // right edge of the sensor FOV circular sector

    void updateFovEdges()
    {
        mFovLeftEdge.setP1(mSensorPose.Pos());
        mFovLeftEdge.setP2(mSensorPose.Pos() + mFovRange_m * LSM_GEOML::getVecFromAngle(mSensorPose.Yaw_rad() + mFovAzimuth_rad));
        mFovRightEdge.setP1(mSensorPose.Pos());
        mFovRightEdge.setP2(mSensorPose.Pos() + mFovRange_m * LSM_GEOML::getVecFromAngle(mSensorPose.Yaw_rad() - mFovAzimuth_rad));
    }

public:
    SensorFov2D() = default;

    //! \brief      Contructs and initializes a new sensor field-of-view.
    //! \param[in]  sensorPose      sensor pose (x, y, yaw angle)
    //! \param[in]  fovRange_m      range (=radius) of the field of view in meters
    //! \param[in]  fovAzimuth_rad  azimuthal opening angle of the field of view in radians
    SensorFov2D(const LSM_GEOML::Pose& sensorPose, const float32_t fovRange_m, const float32_t fovAzimuth_rad)
        : mSensorPose(sensorPose), mFovRange_m(fovRange_m), mFovAzimuth_rad(fovAzimuth_rad) 
    {
        updateFovEdges();
    }

    //! \brief      Sensor pose const getter
    LSM_GEOML::Pose const& getSensorPose() const
    {
        return mSensorPose;
    }

    //! \brief      FOV range in meters const getter
    float32_t const& getFovRangeM() const
    {
        return mFovRange_m;
    }

    //! \brief      FOV Azimuth angle in radians const getter
    float32_t const& getFovAzimuthRad() const
    {
        return mFovAzimuth_rad;
    }

    //! \brief      FOV left edge segment const getter
    LSM_GEOML::LineSegment2D const& getFovLeftEdge() const
    {
        return mFovLeftEdge;
    }

    //! \brief      FOV right edge segment const getter
    LSM_GEOML::LineSegment2D const& getFovRightEdge() const
    {
        return mFovRightEdge;
    }

    //! \brief      Sets the sensor pose (x, y, yaw angle) for this field of view
    //! \param[in]  sensorPose      sensor pose (x, y, yaw angle) to set
    void setSensorPose(const LSM_GEOML::Pose& sensorPose)
    {
        mSensorPose = sensorPose;
        updateFovEdges();
    }

    //! \brief      Checks whether a point (x, y) lies within this field of view
    //! \param[in]  point      point (x, y) to check
    //! \returns    true if the point lies within this field of view
    bool isPointWithin(const cml::Vec2Df& point) const
    {
        const cml::Vec2Df vecSensorToPoint{ point - mSensorPose.Pos() };
        const bool isWithin{ (vecSensorToPoint.norm() < mFovRange_m) && (LSM_GEOML::absAngularDifference(vecSensorToPoint.getAngle(), mSensorPose.Yaw_rad()) < mFovAzimuth_rad) };
        return isWithin;
    }

    //! \brief      Checks whether a line segment intersects the contour of this field of view
    //! \param[in]  lineSegment     line segment to check
    //! \returns    true if the line segment intersects the FOV contour
    //! \returns    vector containing all the intersection points with the sensor field-of-view contour
    std::pair<bool, std::vector<cml::Vec2Df>> intersectsFovContour(const LSM_GEOML::LineSegment2D& lineSegment) const
    {
        std::vector<cml::Vec2Df> intersectionPoints;
        // 1. Test intersection with field-of-view side edges
        for (const auto& fovEdge : { mFovLeftEdge,  mFovRightEdge }) {
            const auto fovEdgeIntersectionResult{ lineSegment.determineSegmentIntersection(fovEdge) };
            if (LSM_GEOML::TypeOfSegmentIntersection::ONE_POINT_INTERSECTION == std::get<0>(fovEdgeIntersectionResult)) {
                intersectionPoints.push_back(std::get<1>(fovEdgeIntersectionResult));
            }
        }

        // 2. Test intersection with field-of-view arc
        const cml::Vec2Df lineDirection{ lineSegment.directionVec() };
        // The intersections are calculated for the endless line and the complete circle.
        const auto fovCircleIntersections{ LSM_GEOML::calcLineCircleIntersections(lineSegment.p1(), lineDirection, mSensorPose.Pos(), mFovRange_m) };
        // Ignore a single intersection because it means that the line is a tangent to the field-of-view circle and does not reach into the FOV.
        if (2U == fovCircleIntersections.getSize()) {
            for (const cml::Vec2Df& intersectionPoint : fovCircleIntersections)
            {
                const float32_t relativePositionOnLineSegment = static_cast<cml::Vec2Df>(intersectionPoint - lineSegment.p1()).scalarProduct(lineDirection) / lineDirection.normSq();
                // Is the intersection point on the line segment?
                if (relativePositionOnLineSegment > 0.0F && relativePositionOnLineSegment < 1.0F)
                {
                    const cml::Vec2Df vecSensorToIntersection{ intersectionPoint - mSensorPose.Pos() };
                    // Is the intersection point on the field-of-view arc
                    if (LSM_GEOML::absAngularDifference(vecSensorToIntersection.getAngle(), mSensorPose.Yaw_rad()) < mFovAzimuth_rad) {
                        intersectionPoints.push_back(intersectionPoint);
                    }
                }
            }
        }
        return { !intersectionPoints.empty(), intersectionPoints };
    }

    //! \brief      Checks whether an object polygon overlaps with this field of view
    //! \param[in]  objectPolygon       the polygon describing the object to check
    //! \returns    true if the polygon overlaps with this field of view
    bool doOverlap(const std::vector<cml::Vec2Df>& objectPolygon) const
    {
        for (const cml::Vec2Df& objectPoint : objectPolygon) {
            if (isPointWithin(objectPoint)) {
                return true;
            }
        }
        for (size_t i{ 0U }; i < objectPolygon.size(); ++i) {
            const size_t iCyclicNext = (i + 1U == objectPolygon.size()) ? 0U : i + 1U;
            if (intersectsFovContour(LSM_GEOML::LineSegment2D(objectPolygon[i], objectPolygon[iCyclicNext])).first) {
                return true;
            }
        }
        return false;
    }
};