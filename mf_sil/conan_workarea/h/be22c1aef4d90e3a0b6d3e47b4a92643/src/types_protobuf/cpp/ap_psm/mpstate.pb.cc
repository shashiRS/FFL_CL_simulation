// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_psm/mpstate.proto

#include "ap_psm/mpstate.pb.h"

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
namespace ap_psm {
namespace mpstate {
}  // namespace mpstate
}  // namespace ap_psm
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_ap_5fpsm_2fmpstate_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_ap_5fpsm_2fmpstate_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5fpsm_2fmpstate_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5fpsm_2fmpstate_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_ap_5fpsm_2fmpstate_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\024ap_psm/mpstate.proto\022\021pb.ap_psm.mpstat"
  "e*\272\001\n\007MPState\022\017\n\013MP_INACTIVE\020\000\022\021\n\rMP_TRA"
  "IN_SCAN\020\001\022\020\n\014MP_TRAIN_AVG\020\002\022\025\n\021MP_TRAIN_"
  "FINISHED\020\003\022\025\n\021MP_SILENT_MAP_LOC\020\004\022\022\n\016MP_"
  "RECALL_SCAN\020\005\022\021\n\rMP_RECALL_AVG\020\006\022\026\n\022MP_R"
  "ECALL_FINISHED\020\007\022\014\n\010MP_PAUSE\020\010"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5fpsm_2fmpstate_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5fpsm_2fmpstate_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5fpsm_2fmpstate_2eproto_once;
static bool descriptor_table_ap_5fpsm_2fmpstate_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fpsm_2fmpstate_2eproto = {
  &descriptor_table_ap_5fpsm_2fmpstate_2eproto_initialized, descriptor_table_protodef_ap_5fpsm_2fmpstate_2eproto, "ap_psm/mpstate.proto", 230,
  &descriptor_table_ap_5fpsm_2fmpstate_2eproto_once, descriptor_table_ap_5fpsm_2fmpstate_2eproto_sccs, descriptor_table_ap_5fpsm_2fmpstate_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_ap_5fpsm_2fmpstate_2eproto::offsets,
  file_level_metadata_ap_5fpsm_2fmpstate_2eproto, 0, file_level_enum_descriptors_ap_5fpsm_2fmpstate_2eproto, file_level_service_descriptors_ap_5fpsm_2fmpstate_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5fpsm_2fmpstate_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5fpsm_2fmpstate_2eproto), true);
namespace pb {
namespace ap_psm {
namespace mpstate {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* MPState_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_ap_5fpsm_2fmpstate_2eproto);
  return file_level_enum_descriptors_ap_5fpsm_2fmpstate_2eproto[0];
}
bool MPState_IsValid(int value) {
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
}  // namespace mpstate
}  // namespace ap_psm
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
