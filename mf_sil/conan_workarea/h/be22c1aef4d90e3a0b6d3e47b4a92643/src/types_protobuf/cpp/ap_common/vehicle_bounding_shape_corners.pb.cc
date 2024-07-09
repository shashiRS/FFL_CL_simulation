// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_common/vehicle_bounding_shape_corners.proto

#include "ap_common/vehicle_bounding_shape_corners.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
namespace pb {
namespace ap_common {
namespace vehicle_bounding_shape_corners {
}  // namespace vehicle_bounding_shape_corners
}  // namespace ap_common
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n.ap_common/vehicle_bounding_shape_corne"
  "rs.proto\022+pb.ap_common.vehicle_bounding_"
  "shape_corners*\206\002\n\033VehicleBoundingShapeCo"
  "rners\022\024\n\020FRONT_LEFT_INNER\020\000\022\024\n\020FRONT_LEF"
  "T_OUTER\020\001\022\r\n\tAXIS_LEFT\020\002\022\023\n\017REAR_LEFT_OU"
  "TER\020\003\022\023\n\017REAR_LEFT_INNER\020\004\022\024\n\020REAR_RIGHT"
  "_INNER\020\005\022\024\n\020REAR_RIGHT_OUTER\020\006\022\016\n\nAXIS_R"
  "IGHT\020\007\022\025\n\021FRONT_RIGHT_OUTER\020\010\022\025\n\021FRONT_R"
  "IGHT_INNER\020\t\022\030\n\024NUM_RELEVANT_CORNERS\020\n"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto_once;
static bool descriptor_table_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto = {
  &descriptor_table_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto_initialized, descriptor_table_protodef_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto, "ap_common/vehicle_bounding_shape_corners.proto", 358,
  &descriptor_table_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto_once, descriptor_table_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto_sccs, descriptor_table_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto::offsets,
  file_level_metadata_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto, 0, file_level_enum_descriptors_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto, file_level_service_descriptors_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto), true);
namespace pb {
namespace ap_common {
namespace vehicle_bounding_shape_corners {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* VehicleBoundingShapeCorners_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto);
  return file_level_enum_descriptors_ap_5fcommon_2fvehicle_5fbounding_5fshape_5fcorners_2eproto[0];
}
bool VehicleBoundingShapeCorners_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace vehicle_bounding_shape_corners
}  // namespace ap_common
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>