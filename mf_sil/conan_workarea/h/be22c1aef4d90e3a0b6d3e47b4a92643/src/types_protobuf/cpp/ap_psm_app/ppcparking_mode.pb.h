// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_psm_app/ppcparking_mode.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto;
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_psm_app {
namespace ppcparking_mode {

enum PPCParkingMode : int {
  PARKING_MODE_NOT_VALID = 0,
  PARK_IN_FULL_MANEUVERING_AREA = 1,
  PARK_IN_RESTRICTED_MANEUVERING_AREA = 13,
  PARK_OUT_UNTIL_CRITICAL_POINT_REACHED = 2,
  PARK_OUT_TO_TARGET_POSE = 14,
  GARAGE_PARKING_IN = 3,
  GARAGE_PARKING_OUT = 4,
  TRAINED_PARKING_TRAIN = 5,
  TRAINED_PARKING_EXEC = 6,
  REMOTE_MANEUVERING = 7,
  MEMORY_PARKING_TRAIN = 8,
  MEMORY_PARKING_EXEC = 9,
  UNDO_MANEUVER = 10,
  REVERSE_ASSIST_ACTIVE = 11,
  REMOTE_SELF_TEST = 12
};
bool PPCParkingMode_IsValid(int value);
constexpr PPCParkingMode PPCParkingMode_MIN = PARKING_MODE_NOT_VALID;
constexpr PPCParkingMode PPCParkingMode_MAX = PARK_OUT_TO_TARGET_POSE;
constexpr int PPCParkingMode_ARRAYSIZE = PPCParkingMode_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* PPCParkingMode_descriptor();
template<typename T>
inline const std::string& PPCParkingMode_Name(T enum_t_value) {
  static_assert(::std::is_same<T, PPCParkingMode>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function PPCParkingMode_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    PPCParkingMode_descriptor(), enum_t_value);
}
inline bool PPCParkingMode_Parse(
    const std::string& name, PPCParkingMode* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<PPCParkingMode>(
    PPCParkingMode_descriptor(), name, value);
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

}  // namespace ppcparking_mode
}  // namespace ap_psm_app
}  // namespace pb

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::pb::ap_psm_app::ppcparking_mode::PPCParkingMode> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::pb::ap_psm_app::ppcparking_mode::PPCParkingMode>() {
  return ::pb::ap_psm_app::ppcparking_mode::PPCParkingMode_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_5fapp_2fppcparking_5fmode_2eproto
