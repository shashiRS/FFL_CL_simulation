// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_hmih/apuser_action_head_unit.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fapuser_5faction_5fhead_5funit_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fapuser_5faction_5fhead_5funit_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3011000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3011004 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fhmih_2fapuser_5faction_5fhead_5funit_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fhmih_2fapuser_5faction_5fhead_5funit_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fhmih_2fapuser_5faction_5fhead_5funit_2eproto;
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_hmih {
namespace apuser_action_head_unit {

enum APUserActionHeadUnit : int {
  AP_NO_USER_ACTION = 0,
  AP_TAP_ON_START_SELECTION = 1,
  AP_TAP_ON_START_PARKING = 2,
  AP_TAP_ON_INTERRUPT = 3,
  AP_TAP_ON_CONTINUE = 4,
  AP_TAP_ON_UNDO = 5,
  AP_TAP_ON_CANCEL = 6,
  AP_TAP_ON_REDO = 7,
  AP_TAP_ON_START_REMOTE_PARKING = 8,
  AP_TAP_ON_SWITCH_DIRECTION = 9,
  AP_TAP_ON_SWITCH_ORIENTATION = 10,
  AP_TAP_ON_PREVIOUS_SCREEN = 11,
  AP_TOGGLE_AP_ACTIVE = 12,
  AP_TAP_ON_FULLY_AUTOMATED_PARKING = 13,
  AP_TAP_ON_SEMI_AUTOMATED_PARKING = 14,
  AP_TAP_ON_START_KEY_PARKING = 15,
  AP_TAP_ON_GP = 16,
  AP_TAP_ON_RM = 17,
  AP_TAP_SWITCH_TO_REMOTE_APP = 18,
  AP_TAP_SWITCH_TO_REMOTE_KEY = 19,
  AP_TAP_ON_EXPLICIT_SCANNING = 20,
  AP_TAP_ON_REVERSE_ASSIST = 21,
  AP_TAP_ON_USER_SLOT_DEFINE = 22,
  AP_TAP_ON_MEMORY_PARKING = 23,
  AP_TAP_ON_USER_SLOT_REFINE = 24,
  AP_TAP_ON_USER_SLOT_SAVE = 25,
  AP_TAP_ON_USER_SLOT_CLOSE = 26,
  AP_TAP_ON_USER_SLOT_DELETE = 27,
  AP_TAP_ON_CROSS = 28
};
bool APUserActionHeadUnit_IsValid(int value);
constexpr APUserActionHeadUnit APUserActionHeadUnit_MIN = AP_NO_USER_ACTION;
constexpr APUserActionHeadUnit APUserActionHeadUnit_MAX = AP_TAP_ON_CROSS;
constexpr int APUserActionHeadUnit_ARRAYSIZE = APUserActionHeadUnit_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* APUserActionHeadUnit_descriptor();
template<typename T>
inline const std::string& APUserActionHeadUnit_Name(T enum_t_value) {
  static_assert(::std::is_same<T, APUserActionHeadUnit>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function APUserActionHeadUnit_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    APUserActionHeadUnit_descriptor(), enum_t_value);
}
inline bool APUserActionHeadUnit_Parse(
    const std::string& name, APUserActionHeadUnit* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<APUserActionHeadUnit>(
    APUserActionHeadUnit_descriptor(), name, value);
}
// ===================================================================


// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace apuser_action_head_unit
}  // namespace mf_hmih
}  // namespace pb

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::pb::mf_hmih::apuser_action_head_unit::APUserActionHeadUnit> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::pb::mf_hmih::apuser_action_head_unit::APUserActionHeadUnit>() {
  return ::pb::mf_hmih::apuser_action_head_unit::APUserActionHeadUnit_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fapuser_5faction_5fhead_5funit_2eproto
