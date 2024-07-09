// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_common/vehicle_part.proto

#include "ap_common/vehicle_part.pb.h"

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
namespace vehicle_part {
}  // namespace vehicle_part
}  // namespace ap_common
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_ap_5fcommon_2fvehicle_5fpart_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_ap_5fcommon_2fvehicle_5fpart_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5fcommon_2fvehicle_5fpart_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5fcommon_2fvehicle_5fpart_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_ap_5fcommon_2fvehicle_5fpart_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\034ap_common/vehicle_part.proto\022\031pb.ap_co"
  "mmon.vehicle_part*\300\001\n\013VehiclePart\022\024\n\020WHE"
  "EL_FRONT_LEFT\020\000\022\023\n\017WHEEL_REAR_LEFT\020\001\022\024\n\020"
  "WHEEL_REAR_RIGHT\020\002\022\025\n\021WHEEL_FRONT_RIGHT\020"
  "\003\022\014\n\010CAR_BODY\020\004\022\017\n\013MIRROR_LEFT\020\005\022\020\n\014MIRR"
  "OR_RIGHT\020\006\022\021\n\rTRAILER_HITCH\020\007\022\025\n\021MAX_NUM"
  "_VEH_PARTS\020\010"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5fcommon_2fvehicle_5fpart_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5fcommon_2fvehicle_5fpart_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5fcommon_2fvehicle_5fpart_2eproto_once;
static bool descriptor_table_ap_5fcommon_2fvehicle_5fpart_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fcommon_2fvehicle_5fpart_2eproto = {
  &descriptor_table_ap_5fcommon_2fvehicle_5fpart_2eproto_initialized, descriptor_table_protodef_ap_5fcommon_2fvehicle_5fpart_2eproto, "ap_common/vehicle_part.proto", 252,
  &descriptor_table_ap_5fcommon_2fvehicle_5fpart_2eproto_once, descriptor_table_ap_5fcommon_2fvehicle_5fpart_2eproto_sccs, descriptor_table_ap_5fcommon_2fvehicle_5fpart_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_ap_5fcommon_2fvehicle_5fpart_2eproto::offsets,
  file_level_metadata_ap_5fcommon_2fvehicle_5fpart_2eproto, 0, file_level_enum_descriptors_ap_5fcommon_2fvehicle_5fpart_2eproto, file_level_service_descriptors_ap_5fcommon_2fvehicle_5fpart_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5fcommon_2fvehicle_5fpart_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5fcommon_2fvehicle_5fpart_2eproto), true);
namespace pb {
namespace ap_common {
namespace vehicle_part {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* VehiclePart_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_ap_5fcommon_2fvehicle_5fpart_2eproto);
  return file_level_enum_descriptors_ap_5fcommon_2fvehicle_5fpart_2eproto[0];
}
bool VehiclePart_IsValid(int value) {
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
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace vehicle_part
}  // namespace ap_common
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>