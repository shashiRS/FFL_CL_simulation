// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_psm_app/ppcparking_mode.proto

#include "ap_psm_app/ppcparking_mode.pb.h"

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
namespace ap_psm_app {
namespace ppcparking_mode {
}  // namespace ppcparking_mode
}  // namespace ap_psm_app
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n ap_psm_app/ppcparking_mode.proto\022\035pb.a"
  "p_psm_app.ppcparking_mode*\263\003\n\016PPCParking"
  "Mode\022\032\n\026PARKING_MODE_NOT_VALID\020\000\022!\n\035PARK"
  "_IN_FULL_MANEUVERING_AREA\020\001\022\'\n#PARK_IN_R"
  "ESTRICTED_MANEUVERING_AREA\020\r\022)\n%PARK_OUT"
  "_UNTIL_CRITICAL_POINT_REACHED\020\002\022\033\n\027PARK_"
  "OUT_TO_TARGET_POSE\020\016\022\025\n\021GARAGE_PARKING_I"
  "N\020\003\022\026\n\022GARAGE_PARKING_OUT\020\004\022\031\n\025TRAINED_P"
  "ARKING_TRAIN\020\005\022\030\n\024TRAINED_PARKING_EXEC\020\006"
  "\022\026\n\022REMOTE_MANEUVERING\020\007\022\030\n\024MEMORY_PARKI"
  "NG_TRAIN\020\010\022\027\n\023MEMORY_PARKING_EXEC\020\t\022\021\n\rU"
  "NDO_MANEUVER\020\n\022\031\n\025REVERSE_ASSIST_ACTIVE\020"
  "\013\022\024\n\020REMOTE_SELF_TEST\020\014"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto_once;
static bool descriptor_table_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto = {
  &descriptor_table_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto_initialized, descriptor_table_protodef_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto, "ap_psm_app/ppcparking_mode.proto", 503,
  &descriptor_table_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto_once, descriptor_table_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto_sccs, descriptor_table_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto::offsets,
  file_level_metadata_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto, 0, file_level_enum_descriptors_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto, file_level_service_descriptors_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto), true);
namespace pb {
namespace ap_psm_app {
namespace ppcparking_mode {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* PPCParkingMode_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto);
  return file_level_enum_descriptors_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto[0];
}
bool PPCParkingMode_IsValid(int value) {
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
    case 11:
    case 12:
    case 13:
    case 14:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace ppcparking_mode
}  // namespace ap_psm_app
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
