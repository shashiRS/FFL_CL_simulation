// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/parking_scenario_types.proto

#include "si/parking_scenario_types.pb.h"

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
namespace parking_scenario_types {
}  // namespace parking_scenario_types
}  // namespace si
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_si_2fparking_5fscenario_5ftypes_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_si_2fparking_5fscenario_5ftypes_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_si_2fparking_5fscenario_5ftypes_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_si_2fparking_5fscenario_5ftypes_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_si_2fparking_5fscenario_5ftypes_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\037si/parking_scenario_types.proto\022\034pb.si"
  ".parking_scenario_types*\354\002\n\024ParkingScena"
  "rioTypes\022\024\n\020PARALLEL_PARKING\020\000\022\031\n\025PERPEN"
  "DICULAR_PARKING\020\001\022\'\n#ANGLED_PARKING_OPEN"
  "ING_TOWARDS_BACK\020\002\022(\n$ANGLED_PARKING_OPE"
  "NING_TOWARDS_FRONT\020\003\022\022\n\016GARAGE_PARKING\020\004"
  "\022\022\n\016DIRECT_PARKING\020\005\022\033\n\027EXTERNAL_TAPOS_P"
  "ARALLEL\020\006\022 \n\034EXTERNAL_TAPOS_PERPENDICULA"
  "R\020\007\022\037\n\033EXTERNAL_TAPOS_PARALLEL_OUT\020\010\022$\n "
  "EXTERNAL_TAPOS_PERPENDICULAR_OUT\020\t\022\"\n\036MA"
  "X_NUM_PARKING_SCENARIO_TYPES\020\n"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_si_2fparking_5fscenario_5ftypes_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_si_2fparking_5fscenario_5ftypes_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_si_2fparking_5fscenario_5ftypes_2eproto_once;
static bool descriptor_table_si_2fparking_5fscenario_5ftypes_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fparking_5fscenario_5ftypes_2eproto = {
  &descriptor_table_si_2fparking_5fscenario_5ftypes_2eproto_initialized, descriptor_table_protodef_si_2fparking_5fscenario_5ftypes_2eproto, "si/parking_scenario_types.proto", 430,
  &descriptor_table_si_2fparking_5fscenario_5ftypes_2eproto_once, descriptor_table_si_2fparking_5fscenario_5ftypes_2eproto_sccs, descriptor_table_si_2fparking_5fscenario_5ftypes_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_si_2fparking_5fscenario_5ftypes_2eproto::offsets,
  file_level_metadata_si_2fparking_5fscenario_5ftypes_2eproto, 0, file_level_enum_descriptors_si_2fparking_5fscenario_5ftypes_2eproto, file_level_service_descriptors_si_2fparking_5fscenario_5ftypes_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_si_2fparking_5fscenario_5ftypes_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_si_2fparking_5fscenario_5ftypes_2eproto), true);
namespace pb {
namespace si {
namespace parking_scenario_types {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ParkingScenarioTypes_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_si_2fparking_5fscenario_5ftypes_2eproto);
  return file_level_enum_descriptors_si_2fparking_5fscenario_5ftypes_2eproto[0];
}
bool ParkingScenarioTypes_IsValid(int value) {
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
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace parking_scenario_types
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>