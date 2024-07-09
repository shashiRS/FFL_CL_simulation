// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/accstatus.proto

#include "ap_vehstatesigprovider/accstatus.pb.h"

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
namespace ap_vehstatesigprovider {
namespace accstatus {
}  // namespace accstatus
}  // namespace ap_vehstatesigprovider
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_ap_5fvehstatesigprovider_2faccstatus_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_ap_5fvehstatesigprovider_2faccstatus_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5fvehstatesigprovider_2faccstatus_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5fvehstatesigprovider_2faccstatus_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_ap_5fvehstatesigprovider_2faccstatus_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n&ap_vehstatesigprovider/accstatus.proto"
  "\022#pb.ap_vehstatesigprovider.accstatus*\241\001"
  "\n\tACCStatus\022\013\n\007ACC_OFF\020\000\022\014\n\010ACC_INIT\020\001\022\017"
  "\n\013ACC_STANDBY\020\002\022\016\n\nACC_ACTIVE\020\003\022\020\n\014ACC_O"
  "VERRIDE\020\004\022\020\n\014ACC_TURN_OFF\020\005\022\030\n\024ACC_ERROR"
  "_REVERSIBLE\020\006\022\032\n\026ACC_ERROR_IRREVERSIBLE\020"
  "\007"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5fvehstatesigprovider_2faccstatus_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5fvehstatesigprovider_2faccstatus_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5fvehstatesigprovider_2faccstatus_2eproto_once;
static bool descriptor_table_ap_5fvehstatesigprovider_2faccstatus_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2faccstatus_2eproto = {
  &descriptor_table_ap_5fvehstatesigprovider_2faccstatus_2eproto_initialized, descriptor_table_protodef_ap_5fvehstatesigprovider_2faccstatus_2eproto, "ap_vehstatesigprovider/accstatus.proto", 241,
  &descriptor_table_ap_5fvehstatesigprovider_2faccstatus_2eproto_once, descriptor_table_ap_5fvehstatesigprovider_2faccstatus_2eproto_sccs, descriptor_table_ap_5fvehstatesigprovider_2faccstatus_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_ap_5fvehstatesigprovider_2faccstatus_2eproto::offsets,
  file_level_metadata_ap_5fvehstatesigprovider_2faccstatus_2eproto, 0, file_level_enum_descriptors_ap_5fvehstatesigprovider_2faccstatus_2eproto, file_level_service_descriptors_ap_5fvehstatesigprovider_2faccstatus_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5fvehstatesigprovider_2faccstatus_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5fvehstatesigprovider_2faccstatus_2eproto), true);
namespace pb {
namespace ap_vehstatesigprovider {
namespace accstatus {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ACCStatus_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_ap_5fvehstatesigprovider_2faccstatus_2eproto);
  return file_level_enum_descriptors_ap_5fvehstatesigprovider_2faccstatus_2eproto[0];
}
bool ACCStatus_IsValid(int value) {
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
}  // namespace accstatus
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
