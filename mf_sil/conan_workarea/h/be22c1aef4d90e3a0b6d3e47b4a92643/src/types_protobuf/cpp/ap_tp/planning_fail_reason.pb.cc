// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_tp/planning_fail_reason.proto

#include "ap_tp/planning_fail_reason.pb.h"

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
namespace ap_tp {
namespace planning_fail_reason {
}  // namespace planning_fail_reason
}  // namespace ap_tp
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_ap_5ftp_2fplanning_5ffail_5freason_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_ap_5ftp_2fplanning_5ffail_5freason_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5ftp_2fplanning_5ffail_5freason_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5ftp_2fplanning_5ffail_5freason_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_ap_5ftp_2fplanning_5ffail_5freason_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n ap_tp/planning_fail_reason.proto\022\035pb.a"
  "p_tp.planning_fail_reason*\245\001\n\022PlanningFa"
  "ilReason\022\014\n\010PFR_NONE\020\000\022\030\n\024PFR_TARGET_POS"
  "E_LOST\020\001\022\030\n\024PFR_PARKING_BOX_LOST\020\002\022\027\n\023PF"
  "R_INPUT_CORRUPTED\020\003\022\023\n\017PFR_REPLAN_FAIL\020\004"
  "\022\037\n\033MAX_NUM_PLANNING_FAIL_TYPES\020\005"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5ftp_2fplanning_5ffail_5freason_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5ftp_2fplanning_5ffail_5freason_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5ftp_2fplanning_5ffail_5freason_2eproto_once;
static bool descriptor_table_ap_5ftp_2fplanning_5ffail_5freason_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftp_2fplanning_5ffail_5freason_2eproto = {
  &descriptor_table_ap_5ftp_2fplanning_5ffail_5freason_2eproto_initialized, descriptor_table_protodef_ap_5ftp_2fplanning_5ffail_5freason_2eproto, "ap_tp/planning_fail_reason.proto", 233,
  &descriptor_table_ap_5ftp_2fplanning_5ffail_5freason_2eproto_once, descriptor_table_ap_5ftp_2fplanning_5ffail_5freason_2eproto_sccs, descriptor_table_ap_5ftp_2fplanning_5ffail_5freason_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_ap_5ftp_2fplanning_5ffail_5freason_2eproto::offsets,
  file_level_metadata_ap_5ftp_2fplanning_5ffail_5freason_2eproto, 0, file_level_enum_descriptors_ap_5ftp_2fplanning_5ffail_5freason_2eproto, file_level_service_descriptors_ap_5ftp_2fplanning_5ffail_5freason_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5ftp_2fplanning_5ffail_5freason_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5ftp_2fplanning_5ffail_5freason_2eproto), true);
namespace pb {
namespace ap_tp {
namespace planning_fail_reason {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* PlanningFailReason_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_ap_5ftp_2fplanning_5ffail_5freason_2eproto);
  return file_level_enum_descriptors_ap_5ftp_2fplanning_5ffail_5freason_2eproto[0];
}
bool PlanningFailReason_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace planning_fail_reason
}  // namespace ap_tp
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
