// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_hmih/hu_frame_ready.proto

#include "mf_hmih/hu_frame_ready.pb.h"

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
namespace mf_hmih {
namespace hu_frame_ready {
}  // namespace hu_frame_ready
}  // namespace mf_hmih
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_mf_5fhmih_2fhu_5fframe_5fready_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_mf_5fhmih_2fhu_5fframe_5fready_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_mf_5fhmih_2fhu_5fframe_5fready_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_mf_5fhmih_2fhu_5fframe_5fready_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_mf_5fhmih_2fhu_5fframe_5fready_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\034mf_hmih/hu_frame_ready.proto\022\031pb.mf_hm"
  "ih.hu_frame_ready*\304\002\n\014HuFrameReady\022\013\n\007NO"
  "_USED\020\000\022\021\n\rIN_TRANSITION\020\001\022\017\n\013FRAME_ERRO"
  "R\020\002\022\023\n\017EARLY_RVC_READY\020\003\022\r\n\tAPA_READY\020\004\022"
  "\021\n\rPARK_IN_READY\020\005\022\022\n\016PARK_OUT_READY\020\006\022\030"
  "\n\024GARAGE_PARKING_READY\020\007\022\030\n\024REVERSE_ASSI"
  "ST_READY\020\010\022\016\n\nRCTA_READY\020\t\022\r\n\tBVM_READY\020"
  "\n\022\024\n\020TWO_D_VIEW_READY\020\013\022\026\n\022THREE_D_VIEW_"
  "READY\020\014\022\033\n\027FRONT_CORNER_VIEW_READY\020\r\022\032\n\026"
  "REAR_CORNER_VIEW_READY\020\016"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_mf_5fhmih_2fhu_5fframe_5fready_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_mf_5fhmih_2fhu_5fframe_5fready_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_mf_5fhmih_2fhu_5fframe_5fready_2eproto_once;
static bool descriptor_table_mf_5fhmih_2fhu_5fframe_5fready_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fhmih_2fhu_5fframe_5fready_2eproto = {
  &descriptor_table_mf_5fhmih_2fhu_5fframe_5fready_2eproto_initialized, descriptor_table_protodef_mf_5fhmih_2fhu_5fframe_5fready_2eproto, "mf_hmih/hu_frame_ready.proto", 384,
  &descriptor_table_mf_5fhmih_2fhu_5fframe_5fready_2eproto_once, descriptor_table_mf_5fhmih_2fhu_5fframe_5fready_2eproto_sccs, descriptor_table_mf_5fhmih_2fhu_5fframe_5fready_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_mf_5fhmih_2fhu_5fframe_5fready_2eproto::offsets,
  file_level_metadata_mf_5fhmih_2fhu_5fframe_5fready_2eproto, 0, file_level_enum_descriptors_mf_5fhmih_2fhu_5fframe_5fready_2eproto, file_level_service_descriptors_mf_5fhmih_2fhu_5fframe_5fready_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_mf_5fhmih_2fhu_5fframe_5fready_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_mf_5fhmih_2fhu_5fframe_5fready_2eproto), true);
namespace pb {
namespace mf_hmih {
namespace hu_frame_ready {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* HuFrameReady_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_mf_5fhmih_2fhu_5fframe_5fready_2eproto);
  return file_level_enum_descriptors_mf_5fhmih_2fhu_5fframe_5fready_2eproto[0];
}
bool HuFrameReady_IsValid(int value) {
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
    case 11:
    case 12:
    case 13:
    case 14:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace hu_frame_ready
}  // namespace mf_hmih
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
