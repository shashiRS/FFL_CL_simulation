// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/availability_status.proto

#include "si/availability_status.pb.h"

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
namespace si {
namespace availability_status {
}  // namespace availability_status
}  // namespace si
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_si_2favailability_5fstatus_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_si_2favailability_5fstatus_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_si_2favailability_5fstatus_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_si_2favailability_5fstatus_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_si_2favailability_5fstatus_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\034si/availability_status.proto\022\031pb.si.av"
  "ailability_status*\227\001\n\022AvailabilityStatus"
  "\022\033\n\027ITEM_IN_DEGREADED_STATE\020\000\022\027\n\023ITEM_IN"
  "_ERROR_STATE\020\001\022\026\n\022ITEM_NOT_AVAILABLE\020\002\022\022"
  "\n\016ITEM_AVAILABLE\020\003\022\037\n\033MAX_NUM_AVAILABILI"
  "TY_STATES\020\004"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_si_2favailability_5fstatus_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_si_2favailability_5fstatus_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_si_2favailability_5fstatus_2eproto_once;
static bool descriptor_table_si_2favailability_5fstatus_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2favailability_5fstatus_2eproto = {
  &descriptor_table_si_2favailability_5fstatus_2eproto_initialized, descriptor_table_protodef_si_2favailability_5fstatus_2eproto, "si/availability_status.proto", 211,
  &descriptor_table_si_2favailability_5fstatus_2eproto_once, descriptor_table_si_2favailability_5fstatus_2eproto_sccs, descriptor_table_si_2favailability_5fstatus_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_si_2favailability_5fstatus_2eproto::offsets,
  file_level_metadata_si_2favailability_5fstatus_2eproto, 0, file_level_enum_descriptors_si_2favailability_5fstatus_2eproto, file_level_service_descriptors_si_2favailability_5fstatus_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_si_2favailability_5fstatus_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_si_2favailability_5fstatus_2eproto), true);
namespace pb {
namespace si {
namespace availability_status {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* AvailabilityStatus_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_si_2favailability_5fstatus_2eproto);
  return file_level_enum_descriptors_si_2favailability_5fstatus_2eproto[0];
}
bool AvailabilityStatus_IsValid(int value) {
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
}  // namespace availability_status
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
