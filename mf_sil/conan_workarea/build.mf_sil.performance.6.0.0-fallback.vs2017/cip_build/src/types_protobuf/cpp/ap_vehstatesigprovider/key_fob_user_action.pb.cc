// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/key_fob_user_action.proto

#include "ap_vehstatesigprovider/key_fob_user_action.pb.h"

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
namespace key_fob_user_action {
}  // namespace key_fob_user_action
}  // namespace ap_vehstatesigprovider
}  // namespace pb
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n0ap_vehstatesigprovider/key_fob_user_ac"
  "tion.proto\022-pb.ap_vehstatesigprovider.ke"
  "y_fob_user_action*\316\002\n\020KeyFobUserAction\022\031"
  "\n\025AP_KEY_NO_USER_ACTION\020\000\022\037\n\033AP_KEY_TAP_"
  "ON_START_PARKING\020\001\022\033\n\027AP_KEY_TAP_ON_INTE"
  "RRUPT\020\002\022\032\n\026AP_KEY_TAP_ON_CONTINUE\020\003\022\026\n\022A"
  "P_KEY_TAP_ON_UNDO\020\004\022\030\n\024AP_KEY_TAP_ON_CAN"
  "CEL\020\005\022\026\n\022AP_KEY_TAP_ON_REDO\020\006\022\"\n\036AP_KEY_"
  "FOB_MANEUVER_AUTHORIZED\020\007\022\035\n\031AP_KEY_FOB_"
  "MANEUVER_ABORT\020\010\022\036\n\032AP_KEY_FOB_MANEUVER_"
  "PAUSED\020\t\022\030\n\024AP_KEY_FOB_ACTIVATED\020\n"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto_once;
static bool descriptor_table_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto = {
  &descriptor_table_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto_initialized, descriptor_table_protodef_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto, "ap_vehstatesigprovider/key_fob_user_action.proto", 434,
  &descriptor_table_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto_once, descriptor_table_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto_sccs, descriptor_table_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto::offsets,
  file_level_metadata_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto, 0, file_level_enum_descriptors_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto, file_level_service_descriptors_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto), true);
namespace pb {
namespace ap_vehstatesigprovider {
namespace key_fob_user_action {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* KeyFobUserAction_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto);
  return file_level_enum_descriptors_ap_5fvehstatesigprovider_2fkey_5ffob_5fuser_5faction_2eproto[0];
}
bool KeyFobUserAction_IsValid(int value) {
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
}  // namespace key_fob_user_action
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
