// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_mempark/user_update_request_status.proto

#include "mf_mempark/user_update_request_status.pb.h"

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
namespace mf_mempark {
namespace user_update_request_status {
}  // namespace user_update_request_status
}  // namespace mf_mempark
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n+mf_mempark/user_update_request_status."
  "proto\022(pb.mf_mempark.user_update_request"
  "_status*\216\003\n\027UserUpdateRequestStatus\022\026\n\022U"
  "PDATE_STATUS_INIT\020\000\022\031\n\025UPDATE_STATUS_SUC"
  "CESS\020\001\022%\n!UPDATE_STATUS_SUCCESS_MAX_C_AN"
  "GLE\020\002\022&\n\"UPDATE_STATUS_SUCCESS_MAX_CC_AN"
  "GLE\020\003\022)\n%UPDATE_STATUS_SUCCESS_MAX_LONG_"
  "POS_UP\020\004\022+\n\'UPDATE_STATUS_SUCCESS_MAX_LO"
  "NG_POS_DOWN\020\005\022*\n&UPDATE_STATUS_SUCCESS_M"
  "AX_LAT_POS_LEFT\020\006\022+\n\'UPDATE_STATUS_SUCCE"
  "SS_MAX_LAT_POS_RIGHT\020\007\022&\n\"UPDATE_STATUS_"
  "SUCCESS_MAX_ATTEMPTS\020\010\022\030\n\024UPDATE_STATUS_"
  "FAILED\020\t"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto_once;
static bool descriptor_table_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto = {
  &descriptor_table_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto_initialized, descriptor_table_protodef_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto, "mf_mempark/user_update_request_status.proto", 488,
  &descriptor_table_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto_once, descriptor_table_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto_sccs, descriptor_table_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto::offsets,
  file_level_metadata_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto, 0, file_level_enum_descriptors_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto, file_level_service_descriptors_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto), true);
namespace pb {
namespace mf_mempark {
namespace user_update_request_status {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* UserUpdateRequestStatus_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto);
  return file_level_enum_descriptors_mf_5fmempark_2fuser_5fupdate_5frequest_5fstatus_2eproto[0];
}
bool UserUpdateRequestStatus_IsValid(int value) {
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
}  // namespace user_update_request_status
}  // namespace mf_mempark
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
