// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_lodmc/lo_dmcsystem_state.proto

#include "ap_lodmc/lo_dmcsystem_state.pb.h"

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
namespace ap_lodmc {
namespace lo_dmcsystem_state {
}  // namespace lo_dmcsystem_state
}  // namespace ap_lodmc
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n!ap_lodmc/lo_dmcsystem_state.proto\022\036pb."
  "ap_lodmc.lo_dmcsystem_state*\314\002\n\020LoDMCSys"
  "temState\022\027\n\023LODMC_NOT_AVAILABLE\020\000\022\030\n\024LOD"
  "MC_INITIALISATION\020\001\022\032\n\026LSCA_SUPPORT_AVAI"
  "LABLE\020\002\022\036\n\032LSCA_MSP_SUPPORT_AVAILABLE\020\003\022"
  "\"\n\036LSCA_MSP_APA_SUPPORT_AVAILABLE\020\004\022)\n%L"
  "SCA_MSP_APA_REMOTE_SUPPORT_AVAILABLE\020\005\0226"
  "\n2LSCA_MSP_APA_REMOTE_SUPPORT_AVAILABLE_"
  "REMOTE_READY\020\006\022\025\n\021LODMC_CTRL_ACTIVE\020\007\022\032\n"
  "\026LODMC_CANCEL_BY_DRIVER\020\010\022\017\n\013LODMC_ERROR"
  "\020\t"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto_once;
static bool descriptor_table_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto = {
  &descriptor_table_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto_initialized, descriptor_table_protodef_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto, "ap_lodmc/lo_dmcsystem_state.proto", 402,
  &descriptor_table_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto_once, descriptor_table_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto_sccs, descriptor_table_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto::offsets,
  file_level_metadata_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto, 0, file_level_enum_descriptors_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto, file_level_service_descriptors_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto), true);
namespace pb {
namespace ap_lodmc {
namespace lo_dmcsystem_state {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* LoDMCSystemState_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto);
  return file_level_enum_descriptors_ap_5flodmc_2flo_5fdmcsystem_5fstate_2eproto[0];
}
bool LoDMCSystemState_IsValid(int value) {
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
}  // namespace lo_dmcsystem_state
}  // namespace ap_lodmc
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>