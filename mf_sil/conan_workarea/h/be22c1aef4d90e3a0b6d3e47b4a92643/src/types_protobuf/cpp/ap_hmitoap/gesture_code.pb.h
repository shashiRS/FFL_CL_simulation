// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_hmitoap/gesture_code.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fhmitoap_2fgesture_5fcode_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fhmitoap_2fgesture_5fcode_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ap_5fhmitoap_2fgesture_5fcode_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fhmitoap_2fgesture_5fcode_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fhmitoap_2fgesture_5fcode_2eproto;
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_hmitoap {
namespace gesture_code {

enum GestureCode : int {
  NO_ACTION = 0,
  PRESS = 1,
  LONGPRESS = 2,
  RELEASE = 3,
  CLICK = 4,
  RIGHTCLICK = 5,
  DRAG = 6,
  ZOOM = 7,
  ROTATE = 8,
  RAW = 9,
  DOUBLECLICK = 10,
  TRIPLECLICK = 11,
  FLICK = 12,
  INVALID = 31
};
bool GestureCode_IsValid(int value);
constexpr GestureCode GestureCode_MIN = NO_ACTION;
constexpr GestureCode GestureCode_MAX = INVALID;
constexpr int GestureCode_ARRAYSIZE = GestureCode_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* GestureCode_descriptor();
template<typename T>
inline const std::string& GestureCode_Name(T enum_t_value) {
  static_assert(::std::is_same<T, GestureCode>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function GestureCode_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    GestureCode_descriptor(), enum_t_value);
}
inline bool GestureCode_Parse(
    const std::string& name, GestureCode* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<GestureCode>(
    GestureCode_descriptor(), name, value);
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

}  // namespace gesture_code
}  // namespace ap_hmitoap
}  // namespace pb

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::pb::ap_hmitoap::gesture_code::GestureCode> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::pb::ap_hmitoap::gesture_code::GestureCode>() {
  return ::pb::ap_hmitoap::gesture_code::GestureCode_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fhmitoap_2fgesture_5fcode_2eproto
