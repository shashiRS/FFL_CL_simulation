// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/pb_side.proto

#include "si/pb_side.pb.h"

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
namespace pb_side {
}  // namespace pb_side
}  // namespace si
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_si_2fpb_5fside_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_si_2fpb_5fside_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_si_2fpb_5fside_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_si_2fpb_5fside_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_si_2fpb_5fside_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\020si/pb_side.proto\022\rpb.si.pb_side*\223\002\n\007PB"
  "_Side\022\020\n\014PB_SIDE_LEFT\020\000\022\021\n\rPB_SIDE_RIGHT"
  "\020\001\022\022\n\016PB_SIDE_CENTER\020\002\022-\n)PB_SIDE_RIGHT_"
  "ANGLED_OPENING_TOWARDS_BACK\020\003\022.\n*PB_SIDE"
  "_RIGHT_ANGLED_OPENING_TOWARDS_FRONT\020\004\022,\n"
  "(PB_SIDE_LEFT_ANGLED_OPENING_TOWARDS_BAC"
  "K\020\005\022-\n)PB_SIDE_LEFT_ANGLED_OPENING_TOWAR"
  "DS_FRONT\020\006\022\023\n\017MAX_NUM_PB_SIDE\020\007"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_si_2fpb_5fside_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_si_2fpb_5fside_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_si_2fpb_5fside_2eproto_once;
static bool descriptor_table_si_2fpb_5fside_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fpb_5fside_2eproto = {
  &descriptor_table_si_2fpb_5fside_2eproto_initialized, descriptor_table_protodef_si_2fpb_5fside_2eproto, "si/pb_side.proto", 311,
  &descriptor_table_si_2fpb_5fside_2eproto_once, descriptor_table_si_2fpb_5fside_2eproto_sccs, descriptor_table_si_2fpb_5fside_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_si_2fpb_5fside_2eproto::offsets,
  file_level_metadata_si_2fpb_5fside_2eproto, 0, file_level_enum_descriptors_si_2fpb_5fside_2eproto, file_level_service_descriptors_si_2fpb_5fside_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_si_2fpb_5fside_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_si_2fpb_5fside_2eproto), true);
namespace pb {
namespace si {
namespace pb_side {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* PB_Side_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_si_2fpb_5fside_2eproto);
  return file_level_enum_descriptors_si_2fpb_5fside_2eproto[0];
}
bool PB_Side_IsValid(int value) {
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
}  // namespace pb_side
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
