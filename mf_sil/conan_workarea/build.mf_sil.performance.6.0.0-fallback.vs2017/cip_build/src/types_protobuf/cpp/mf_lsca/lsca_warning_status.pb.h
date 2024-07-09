// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_lsca/lsca_warning_status.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5flsca_2flsca_5fwarning_5fstatus_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5flsca_2flsca_5fwarning_5fstatus_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_mf_5flsca_2flsca_5fwarning_5fstatus_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5flsca_2flsca_5fwarning_5fstatus_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5flsca_2flsca_5fwarning_5fstatus_2eproto;
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_lsca {
namespace lsca_warning_status {

enum LSCA_WARNING_STATUS : int {
  LSCA_WARNING_NONE = 0,
  LSCA_WARNING_ALL = 1,
  LSCA_WARNING_FRONT = 2,
  LSCA_WARNING_LEFT = 3,
  LSCA_WARNING_RIGHT = 4,
  LSCA_WARNING_REAR = 5,
  LSCA_WARNING_FRONT_LEFT = 6,
  LSCA_WARNING_FRONT_RIGHT = 7,
  LSCA_WARNING_REAR_LEFT = 8,
  LSCA_WARNING_REAR_RIGHT = 9
};
bool LSCA_WARNING_STATUS_IsValid(int value);
constexpr LSCA_WARNING_STATUS LSCA_WARNING_STATUS_MIN = LSCA_WARNING_NONE;
constexpr LSCA_WARNING_STATUS LSCA_WARNING_STATUS_MAX = LSCA_WARNING_REAR_RIGHT;
constexpr int LSCA_WARNING_STATUS_ARRAYSIZE = LSCA_WARNING_STATUS_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* LSCA_WARNING_STATUS_descriptor();
template<typename T>
inline const std::string& LSCA_WARNING_STATUS_Name(T enum_t_value) {
  static_assert(::std::is_same<T, LSCA_WARNING_STATUS>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function LSCA_WARNING_STATUS_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    LSCA_WARNING_STATUS_descriptor(), enum_t_value);
}
inline bool LSCA_WARNING_STATUS_Parse(
    const std::string& name, LSCA_WARNING_STATUS* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<LSCA_WARNING_STATUS>(
    LSCA_WARNING_STATUS_descriptor(), name, value);
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

}  // namespace lsca_warning_status
}  // namespace mf_lsca
}  // namespace pb

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS>() {
  return ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5flsca_2flsca_5fwarning_5fstatus_2eproto