// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_trjctl/moco_te_fun_mode.proto

#include "ap_trjctl/moco_te_fun_mode.pb.h"

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
namespace ap_trjctl {
namespace moco_te_fun_mode {
}  // namespace moco_te_fun_mode
}  // namespace ap_trjctl
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n ap_trjctl/moco_te_fun_mode.proto\022\035pb.a"
  "p_trjctl.moco_te_fun_mode*\273\002\n\017MOCO_te_Fu"
  "nMode\022\031\n\025MOCO_FUN_MODE_INVALID\020\000\022\030\n\024MOCO"
  "_FUN_MODE_CRUISE\020\001\022\034\n\030MOCO_FUN_MODE_CRUI"
  "SE_LAT\020\002\022\035\n\031MOCO_FUN_MODE_CRUISE_LONG\020\003\022"
  "\033\n\027MOCO_FUN_MODE_SPEED_LIM\020\004\022!\n\035MOCO_FUN"
  "_MODE_SAFETY_LONG_AEB\020\005\022!\n\035MOCO_FUN_MODE"
  "_SAFETY_LONG_VDS\020\006\022\034\n\030MOCO_FUN_MODE_SAFE"
  "TY_LAT\020\007\022\032\n\026MOCO_FUN_MODE_MANEUVER\020\010\022\031\n\025"
  "MOCO_FUN_MODE_OFFROAD\020\t"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto_once;
static bool descriptor_table_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto = {
  &descriptor_table_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto_initialized, descriptor_table_protodef_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto, "ap_trjctl/moco_te_fun_mode.proto", 383,
  &descriptor_table_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto_once, descriptor_table_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto_sccs, descriptor_table_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto::offsets,
  file_level_metadata_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto, 0, file_level_enum_descriptors_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto, file_level_service_descriptors_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto), true);
namespace pb {
namespace ap_trjctl {
namespace moco_te_fun_mode {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* MOCO_te_FunMode_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto);
  return file_level_enum_descriptors_ap_5ftrjctl_2fmoco_5fte_5ffun_5fmode_2eproto[0];
}
bool MOCO_te_FunMode_IsValid(int value) {
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
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace moco_te_fun_mode
}  // namespace ap_trjctl
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
