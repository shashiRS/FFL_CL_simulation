// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_trjctl/lo_dmcctrl_request_source_type.proto

#include "ap_trjctl/lo_dmcctrl_request_source_type.pb.h"

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
namespace lo_dmcctrl_request_source_type {
}  // namespace lo_dmcctrl_request_source_type
}  // namespace ap_trjctl
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n.ap_trjctl/lo_dmcctrl_request_source_ty"
  "pe.proto\022+pb.ap_trjctl.lo_dmcctrl_reques"
  "t_source_type*\267\001\n\032LoDMCCtrlRequestSource"
  "Type\022\037\n\033LODMC_REQ_SRC_NO_REQEUESTER\020\000\022\025\n"
  "\021LODMC_REQ_SRC_AUP\020\001\022\026\n\022LODMC_REQ_SRC_LS"
  "CA\020\002\022\030\n\024LODMC_REQ_SRC_REMOTE\020\003\022\025\n\021LODMC_"
  "REQ_SRC_MSP\020\004\022\030\n\024LODMC_REQ_SRC_SAFBAR\020\005"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto_once;
static bool descriptor_table_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto = {
  &descriptor_table_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto_initialized, descriptor_table_protodef_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto, "ap_trjctl/lo_dmcctrl_request_source_type.proto", 279,
  &descriptor_table_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto_once, descriptor_table_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto_sccs, descriptor_table_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto::offsets,
  file_level_metadata_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto, 0, file_level_enum_descriptors_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto, file_level_service_descriptors_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto), true);
namespace pb {
namespace ap_trjctl {
namespace lo_dmcctrl_request_source_type {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* LoDMCCtrlRequestSourceType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto);
  return file_level_enum_descriptors_ap_5ftrjctl_2flo_5fdmcctrl_5frequest_5fsource_5ftype_2eproto[0];
}
bool LoDMCCtrlRequestSourceType_IsValid(int value) {
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
}  // namespace lo_dmcctrl_request_source_type
}  // namespace ap_trjctl
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
