// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_drvwarnsm_core/dwfcore_state.proto

#include "mf_drvwarnsm_core/dwfcore_state.pb.h"

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
namespace mf_drvwarnsm_core {
namespace dwfcore_state {
}  // namespace dwfcore_state
}  // namespace mf_drvwarnsm_core
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n%mf_drvwarnsm_core/dwfcore_state.proto\022"
  "\"pb.mf_drvwarnsm_core.dwfcore_state*\216\001\n\014"
  "DWFCoreState\022\021\n\rDWF_CORE_INIT\020\000\022\020\n\014DWF_C"
  "ORE_OFF\020\001\022\025\n\021DWF_CORE_INACTIVE\020\002\022\023\n\017DWF_"
  "CORE_ACTIVE\020\003\022\024\n\020DWF_CORE_FAILURE\020\004\022\027\n\023D"
  "WF_CORE_NUM_STATES\020\005"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto_once;
static bool descriptor_table_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto = {
  &descriptor_table_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto_initialized, descriptor_table_protodef_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto, "mf_drvwarnsm_core/dwfcore_state.proto", 220,
  &descriptor_table_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto_once, descriptor_table_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto_sccs, descriptor_table_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto::offsets,
  file_level_metadata_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto, 0, file_level_enum_descriptors_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto, file_level_service_descriptors_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto), true);
namespace pb {
namespace mf_drvwarnsm_core {
namespace dwfcore_state {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* DWFCoreState_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto);
  return file_level_enum_descriptors_mf_5fdrvwarnsm_5fcore_2fdwfcore_5fstate_2eproto[0];
}
bool DWFCoreState_IsValid(int value) {
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
}  // namespace dwfcore_state
}  // namespace mf_drvwarnsm_core
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
