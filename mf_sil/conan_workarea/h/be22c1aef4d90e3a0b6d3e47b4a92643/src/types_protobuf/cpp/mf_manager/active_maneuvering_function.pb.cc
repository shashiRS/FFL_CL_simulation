// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_manager/active_maneuvering_function.proto

#include "mf_manager/active_maneuvering_function.pb.h"

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
namespace mf_manager {
namespace active_maneuvering_function {
}  // namespace active_maneuvering_function
}  // namespace mf_manager
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n,mf_manager/active_maneuvering_function"
  ".proto\022)pb.mf_manager.active_maneuvering"
  "_function*\212\002\n\031ActiveManeuveringFunction\022"
  "\021\n\rFUNCTION_NONE\020\000\022\036\n\032FUNCTION_COLLISION"
  "_WARNING\020\001\022\021\n\rFUNCTION_LSCA\020\002\022\036\n\032FUNCTIO"
  "N_AUTOMATED_PARKING\020\003\022!\n\035FUNCTION_TRAILE"
  "R_HITCH_ASSIST\020\004\022#\n\037FUNCTION_TRAILER_REV"
  "ERSE_ASSIST\020\005\022\034\n\030FUNCTION_TRAINED_PARKIN"
  "G\020\006\022!\n\035MAX_NUM_MANEUVERING_FUNCTIONS\020\007"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto_once;
static bool descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto = {
  &descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto_initialized, descriptor_table_protodef_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto, "mf_manager/active_maneuvering_function.proto", 358,
  &descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto_once, descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto_sccs, descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto::offsets,
  file_level_metadata_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto, 0, file_level_enum_descriptors_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto, file_level_service_descriptors_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto), true);
namespace pb {
namespace mf_manager {
namespace active_maneuvering_function {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ActiveManeuveringFunction_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto);
  return file_level_enum_descriptors_mf_5fmanager_2factive_5fmaneuvering_5ffunction_2eproto[0];
}
bool ActiveManeuveringFunction_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace active_maneuvering_function
}  // namespace mf_manager
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
