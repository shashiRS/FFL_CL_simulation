// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: com/com_ssm_state_t.proto

#include "com/com_ssm_state_t.pb.h"

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
namespace com {
namespace com_ssm_state_t {
}  // namespace com_ssm_state_t
}  // namespace com
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_com_2fcom_5fssm_5fstate_5ft_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_com_2fcom_5fssm_5fstate_5ft_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_com_2fcom_5fssm_5fstate_5ft_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_com_2fcom_5fssm_5fstate_5ft_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_com_2fcom_5fssm_5fstate_5ft_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\031com/com_ssm_state_t.proto\022\026pb.com.com_"
  "ssm_state_t*\332\001\n\016ComSSM_State_t\022\024\n\020COM_SS"
  "M_SYS_INIT\020\000\022\023\n\017COM_SSM_SYS_RUN\020\001\022\033\n\027COM"
  "_SSM_SYS_RUN_LIMITED\020\002\022\030\n\024COM_SSM_SYS_FA"
  "ILSAFE\020\003\022\030\n\024COM_SSM_SYS_SHUTDOWN\020\004\022\022\n\016CO"
  "M_SSM_SD_OFF\020\000\022\026\n\022COM_SSM_SD_RESTART\020\001\022\034"
  "\n\030COM_SSM_SD_GET_PERS_DATA\020\002\032\002\020\001"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_com_2fcom_5fssm_5fstate_5ft_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_com_2fcom_5fssm_5fstate_5ft_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_com_2fcom_5fssm_5fstate_5ft_2eproto_once;
static bool descriptor_table_com_2fcom_5fssm_5fstate_5ft_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_com_2fcom_5fssm_5fstate_5ft_2eproto = {
  &descriptor_table_com_2fcom_5fssm_5fstate_5ft_2eproto_initialized, descriptor_table_protodef_com_2fcom_5fssm_5fstate_5ft_2eproto, "com/com_ssm_state_t.proto", 272,
  &descriptor_table_com_2fcom_5fssm_5fstate_5ft_2eproto_once, descriptor_table_com_2fcom_5fssm_5fstate_5ft_2eproto_sccs, descriptor_table_com_2fcom_5fssm_5fstate_5ft_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_com_2fcom_5fssm_5fstate_5ft_2eproto::offsets,
  file_level_metadata_com_2fcom_5fssm_5fstate_5ft_2eproto, 0, file_level_enum_descriptors_com_2fcom_5fssm_5fstate_5ft_2eproto, file_level_service_descriptors_com_2fcom_5fssm_5fstate_5ft_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_com_2fcom_5fssm_5fstate_5ft_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_com_2fcom_5fssm_5fstate_5ft_2eproto), true);
namespace pb {
namespace com {
namespace com_ssm_state_t {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ComSSM_State_t_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_com_2fcom_5fssm_5fstate_5ft_2eproto);
  return file_level_enum_descriptors_com_2fcom_5fssm_5fstate_5ft_2eproto[0];
}
bool ComSSM_State_t_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace com_ssm_state_t
}  // namespace com
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>