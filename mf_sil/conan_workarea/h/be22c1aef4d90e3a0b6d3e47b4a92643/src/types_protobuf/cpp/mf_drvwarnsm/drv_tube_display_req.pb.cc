// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_drvwarnsm/drv_tube_display_req.proto

#include "mf_drvwarnsm/drv_tube_display_req.pb.h"

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
namespace mf_drvwarnsm {
namespace drv_tube_display_req {
}  // namespace drv_tube_display_req
}  // namespace mf_drvwarnsm
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\'mf_drvwarnsm/drv_tube_display_req.prot"
  "o\022$pb.mf_drvwarnsm.drv_tube_display_req*"
  "e\n\021DrvTubeDisplayReq\022\031\n\025PDW_DRV_TUBE_REQ"
  "_NONE\020\000\022\032\n\026PDW_DRV_TUBE_REQ_FRONT\020\001\022\031\n\025P"
  "DW_DRV_TUBE_REQ_REAR\020\002"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto_once;
static bool descriptor_table_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto = {
  &descriptor_table_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto_initialized, descriptor_table_protodef_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto, "mf_drvwarnsm/drv_tube_display_req.proto", 182,
  &descriptor_table_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto_once, descriptor_table_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto_sccs, descriptor_table_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto::offsets,
  file_level_metadata_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto, 0, file_level_enum_descriptors_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto, file_level_service_descriptors_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto), true);
namespace pb {
namespace mf_drvwarnsm {
namespace drv_tube_display_req {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* DrvTubeDisplayReq_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto);
  return file_level_enum_descriptors_mf_5fdrvwarnsm_2fdrv_5ftube_5fdisplay_5freq_2eproto[0];
}
bool DrvTubeDisplayReq_IsValid(int value) {
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
}  // namespace drv_tube_display_req
}  // namespace mf_drvwarnsm
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
