// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_tp/pose_type.proto

#include "ap_tp/pose_type.pb.h"

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
namespace pose_type {
}  // namespace pose_type
}  // namespace ap_tp
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_ap_5ftp_2fpose_5ftype_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_ap_5ftp_2fpose_5ftype_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5ftp_2fpose_5ftype_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5ftp_2fpose_5ftype_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_ap_5ftp_2fpose_5ftype_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\025ap_tp/pose_type.proto\022\022pb.ap_tp.pose_t"
  "ype*\372\003\n\010PoseType\022\026\n\022T_PARALLEL_PARKING\020\000"
  "\022\026\n\022T_PERP_PARKING_FWD\020\001\022\026\n\022T_PERP_PARKI"
  "NG_BWD\020\002\022\035\n\031T_ANGLED_PARKING_STANDARD\020\003\022"
  "\034\n\030T_ANGLED_PARKING_REVERSE\020\004\022\021\n\rT_REM_M"
  "AN_FWD\020\005\022\021\n\rT_REM_MAN_BWD\020\006\022\032\n\026T_PERP_PA"
  "RKING_OUT_FWD\020\007\022\032\n\026T_PERP_PARKING_OUT_BW"
  "D\020\010\022\025\n\021T_PAR_PARKING_OUT\020\t\022!\n\035T_ANGLED_P"
  "ARKING_STANDARD_OUT\020\n\022 \n\034T_ANGLED_PARKIN"
  "G_REVERSE_OUT\020\013\022\n\n\006T_UNDO\020\014\022\014\n\010T_GP_FWD\020"
  "\r\022\014\n\010T_GP_BWD\020\016\022\020\n\014T_GP_OUT_FWD\020\017\022\020\n\014T_G"
  "P_OUT_BWD\020\020\022\021\n\rT_GP_FWD_AXIS\020\021\022\021\n\rT_GP_B"
  "WD_AXIS\020\022\022\025\n\021T_GP_OUT_FWD_AXIS\020\023\022\025\n\021T_GP"
  "_OUT_BWD_AXIS\020\024\022\017\n\013T_UNDEFINED\020\025"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5ftp_2fpose_5ftype_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5ftp_2fpose_5ftype_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5ftp_2fpose_5ftype_2eproto_once;
static bool descriptor_table_ap_5ftp_2fpose_5ftype_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftp_2fpose_5ftype_2eproto = {
  &descriptor_table_ap_5ftp_2fpose_5ftype_2eproto_initialized, descriptor_table_protodef_ap_5ftp_2fpose_5ftype_2eproto, "ap_tp/pose_type.proto", 552,
  &descriptor_table_ap_5ftp_2fpose_5ftype_2eproto_once, descriptor_table_ap_5ftp_2fpose_5ftype_2eproto_sccs, descriptor_table_ap_5ftp_2fpose_5ftype_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_ap_5ftp_2fpose_5ftype_2eproto::offsets,
  file_level_metadata_ap_5ftp_2fpose_5ftype_2eproto, 0, file_level_enum_descriptors_ap_5ftp_2fpose_5ftype_2eproto, file_level_service_descriptors_ap_5ftp_2fpose_5ftype_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5ftp_2fpose_5ftype_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5ftp_2fpose_5ftype_2eproto), true);
namespace pb {
namespace ap_tp {
namespace pose_type {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* PoseType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_ap_5ftp_2fpose_5ftype_2eproto);
  return file_level_enum_descriptors_ap_5ftp_2fpose_5ftype_2eproto[0];
}
bool PoseType_IsValid(int value) {
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
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace pose_type
}  // namespace ap_tp
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
