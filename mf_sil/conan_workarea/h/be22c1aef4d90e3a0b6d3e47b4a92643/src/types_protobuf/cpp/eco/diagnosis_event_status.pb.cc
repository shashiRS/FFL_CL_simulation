// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: eco/diagnosis_event_status.proto

#include "eco/diagnosis_event_status.pb.h"

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
namespace eco {
namespace diagnosis_event_status {
}  // namespace diagnosis_event_status
}  // namespace eco
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_eco_2fdiagnosis_5fevent_5fstatus_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_eco_2fdiagnosis_5fevent_5fstatus_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_eco_2fdiagnosis_5fevent_5fstatus_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_eco_2fdiagnosis_5fevent_5fstatus_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_eco_2fdiagnosis_5fevent_5fstatus_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n eco/diagnosis_event_status.proto\022\035pb.e"
  "co.diagnosis_event_status*\250\001\n\024DiagnosisE"
  "ventStatus\022!\n\035DIAGNOSIS_EVENT_STATUS_PAS"
  "SED\020\000\022!\n\035DIAGNOSIS_EVENT_STATUS_FAILED\020\001"
  "\022$\n DIAGNOSIS_EVENT_STATUS_PREPASSED\020\002\022$"
  "\n DIAGNOSIS_EVENT_STATUS_PREFAILED\020\003"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_eco_2fdiagnosis_5fevent_5fstatus_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_eco_2fdiagnosis_5fevent_5fstatus_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_eco_2fdiagnosis_5fevent_5fstatus_2eproto_once;
static bool descriptor_table_eco_2fdiagnosis_5fevent_5fstatus_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_eco_2fdiagnosis_5fevent_5fstatus_2eproto = {
  &descriptor_table_eco_2fdiagnosis_5fevent_5fstatus_2eproto_initialized, descriptor_table_protodef_eco_2fdiagnosis_5fevent_5fstatus_2eproto, "eco/diagnosis_event_status.proto", 236,
  &descriptor_table_eco_2fdiagnosis_5fevent_5fstatus_2eproto_once, descriptor_table_eco_2fdiagnosis_5fevent_5fstatus_2eproto_sccs, descriptor_table_eco_2fdiagnosis_5fevent_5fstatus_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_eco_2fdiagnosis_5fevent_5fstatus_2eproto::offsets,
  file_level_metadata_eco_2fdiagnosis_5fevent_5fstatus_2eproto, 0, file_level_enum_descriptors_eco_2fdiagnosis_5fevent_5fstatus_2eproto, file_level_service_descriptors_eco_2fdiagnosis_5fevent_5fstatus_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_eco_2fdiagnosis_5fevent_5fstatus_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_eco_2fdiagnosis_5fevent_5fstatus_2eproto), true);
namespace pb {
namespace eco {
namespace diagnosis_event_status {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* DiagnosisEventStatus_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_eco_2fdiagnosis_5fevent_5fstatus_2eproto);
  return file_level_enum_descriptors_eco_2fdiagnosis_5fevent_5fstatus_2eproto[0];
}
bool DiagnosisEventStatus_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace diagnosis_event_status
}  // namespace eco
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
