// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_drv/us_drv_request_meas_mode.proto

#include "us_drv/us_drv_request_meas_mode.pb.h"

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
namespace us_drv {
namespace us_drv_request_meas_mode {
}  // namespace us_drv_request_meas_mode
}  // namespace us_drv
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n%us_drv/us_drv_request_meas_mode.proto\022"
  "\"pb.us_drv.us_drv_request_meas_mode*p\n\024U"
  "sDrvRequestMeasMode\022 \n\034US_MEASMODE_NEAR_"
  "RANGE_FIELD\020\000\022\037\n\033US_MEASMODE_FAR_RANGE_F"
  "IELD\020\001\022\025\n\021US_MEASMODE_COUNT\020\002"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto_once;
static bool descriptor_table_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto = {
  &descriptor_table_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto_initialized, descriptor_table_protodef_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto, "us_drv/us_drv_request_meas_mode.proto", 189,
  &descriptor_table_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto_once, descriptor_table_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto_sccs, descriptor_table_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto::offsets,
  file_level_metadata_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto, 0, file_level_enum_descriptors_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto, file_level_service_descriptors_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto), true);
namespace pb {
namespace us_drv {
namespace us_drv_request_meas_mode {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* UsDrvRequestMeasMode_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto);
  return file_level_enum_descriptors_us_5fdrv_2fus_5fdrv_5frequest_5fmeas_5fmode_2eproto[0];
}
bool UsDrvRequestMeasMode_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace us_drv_request_meas_mode
}  // namespace us_drv
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
