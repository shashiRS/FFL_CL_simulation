// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_em/obj_measurement_state.proto

#include "us_em/obj_measurement_state.pb.h"

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
namespace us_em {
namespace obj_measurement_state {
}  // namespace obj_measurement_state
}  // namespace us_em
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_us_5fem_2fobj_5fmeasurement_5fstate_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_us_5fem_2fobj_5fmeasurement_5fstate_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_us_5fem_2fobj_5fmeasurement_5fstate_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_us_5fem_2fobj_5fmeasurement_5fstate_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_us_5fem_2fobj_5fmeasurement_5fstate_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n!us_em/obj_measurement_state.proto\022\036pb."
  "us_em.obj_measurement_state*\224\001\n\023ObjMeasu"
  "rementState\022\022\n\016MEAS_STATE_NEW\020\000\022\027\n\023MEAS_"
  "STATE_MEASURED\020\001\022\030\n\024MEAS_STATE_PREDICTED"
  "\020\002\022\026\n\022MEAS_STATE_DELETED\020\003\022\036\n\032MAX_NUM_ME"
  "ASUREMENT_STATES\020\004"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_us_5fem_2fobj_5fmeasurement_5fstate_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_us_5fem_2fobj_5fmeasurement_5fstate_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_us_5fem_2fobj_5fmeasurement_5fstate_2eproto_once;
static bool descriptor_table_us_5fem_2fobj_5fmeasurement_5fstate_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fem_2fobj_5fmeasurement_5fstate_2eproto = {
  &descriptor_table_us_5fem_2fobj_5fmeasurement_5fstate_2eproto_initialized, descriptor_table_protodef_us_5fem_2fobj_5fmeasurement_5fstate_2eproto, "us_em/obj_measurement_state.proto", 218,
  &descriptor_table_us_5fem_2fobj_5fmeasurement_5fstate_2eproto_once, descriptor_table_us_5fem_2fobj_5fmeasurement_5fstate_2eproto_sccs, descriptor_table_us_5fem_2fobj_5fmeasurement_5fstate_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_us_5fem_2fobj_5fmeasurement_5fstate_2eproto::offsets,
  file_level_metadata_us_5fem_2fobj_5fmeasurement_5fstate_2eproto, 0, file_level_enum_descriptors_us_5fem_2fobj_5fmeasurement_5fstate_2eproto, file_level_service_descriptors_us_5fem_2fobj_5fmeasurement_5fstate_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_us_5fem_2fobj_5fmeasurement_5fstate_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_us_5fem_2fobj_5fmeasurement_5fstate_2eproto), true);
namespace pb {
namespace us_em {
namespace obj_measurement_state {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ObjMeasurementState_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_us_5fem_2fobj_5fmeasurement_5fstate_2eproto);
  return file_level_enum_descriptors_us_5fem_2fobj_5fmeasurement_5fstate_2eproto[0];
}
bool ObjMeasurementState_IsValid(int value) {
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
}  // namespace obj_measurement_state
}  // namespace us_em
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
