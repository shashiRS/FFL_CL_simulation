// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP14_TYPES_USED

#ifndef US_PROCESSING_US_PROCESSING_OBJECT_H_
#define US_PROCESSING_US_PROCESSING_OBJECT_H_

#include "Platform_Types.h"
#include "us_processing/us_processing_measurement_status.h"
#include "us_processing/us_processing_object_type.h"
#include "us_processing/us_processing_object_height.h"
#include "us_processing/us_processing_object_kinematic.h"


namespace us_processing
{

  struct UsProcessingObject
  {
    ///@unit{us}
    ///timestamp when object detection occured
    uint64 timestamp_us{};
    ///@unit{mps}
    ///X velocity of point relative to car
    float32 xRelativeVelocity_mps{};
    ///@unit{mps}
    ///Y velocity of point relative to car
    float32 yRelativeVelocity_mps{};
    ///@unit{m}
    ///Object Height estimation
    float32 heightEstimation_m{};
    ///@range{0.0,1.0}
    ///@unit{nu}
    ///confidence level of detection, normalized in the range  0.0f to 1.0f
    float32 confidenceLevel{};
    ///@range{0,65535}
    ///@unit{nu}
    ///Unique ID number of object this point originated from
    uint16 objectId{};
    ///Vertex Start Index gives reference to the location of the first vertex describes this object in VertexStartIndex
    uint16 vertexStartIndex{};
    ///Number of Verticies describes this object within Verticies List starts from VertexStartIndex
    uint16 numOfVertices{};
    ///@range{0,3}
    ///see definition of UsProcessingMeasurementStatus
    UsProcessingMeasurementStatus measurementStatus{};
    ///@range{0,5}
    ///see definition of UsProcessingObjectType
    UsProcessingObjectType objectType{};
    ///@range{0,2}
    ///see definition of UsProcessingObjectHeight
    UsProcessingObjectHeight heightClassificationLevel{};
    ///@range{0,2}
    ///see definition of UsProcessingObjectKinematic
    UsProcessingObjectKinematic kinematicClassificationGuess{};
  };

} // namespace us_processing

#endif // US_PROCESSING_US_PROCESSING_OBJECT_H_
