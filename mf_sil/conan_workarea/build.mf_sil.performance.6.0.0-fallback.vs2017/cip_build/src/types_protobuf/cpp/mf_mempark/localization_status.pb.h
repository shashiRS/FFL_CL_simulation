// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_mempark/localization_status.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2flocalization_5fstatus_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2flocalization_5fstatus_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_mf_5fmempark_2flocalization_5fstatus_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fmempark_2flocalization_5fstatus_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmempark_2flocalization_5fstatus_2eproto;
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_mempark {
namespace localization_status {

enum LocalizationStatus : int {
  LOCALIZE_STATUS_INIT = 0,
  LOCALIZE_STATUS_UNAVAIL_OUTSIDE_ZONE = 1,
  LOCALIZE_STATUS_ACTIVE = 2,
  LOCALIZE_STATUS_SUCCESS_OFFER = 3,
  LOCALIZE_STATUS_SUCCESS_NO_OFFER = 4,
  LOCALIZE_STATUS_FAILED = 5
};
bool LocalizationStatus_IsValid(int value);
constexpr LocalizationStatus LocalizationStatus_MIN = LOCALIZE_STATUS_INIT;
constexpr LocalizationStatus LocalizationStatus_MAX = LOCALIZE_STATUS_FAILED;
constexpr int LocalizationStatus_ARRAYSIZE = LocalizationStatus_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* LocalizationStatus_descriptor();
template<typename T>
inline const std::string& LocalizationStatus_Name(T enum_t_value) {
  static_assert(::std::is_same<T, LocalizationStatus>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function LocalizationStatus_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    LocalizationStatus_descriptor(), enum_t_value);
}
inline bool LocalizationStatus_Parse(
    const std::string& name, LocalizationStatus* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<LocalizationStatus>(
    LocalizationStatus_descriptor(), name, value);
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

}  // namespace localization_status
}  // namespace mf_mempark
}  // namespace pb

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::pb::mf_mempark::localization_status::LocalizationStatus> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::pb::mf_mempark::localization_status::LocalizationStatus>() {
  return ::pb::mf_mempark::localization_status::LocalizationStatus_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2flocalization_5fstatus_2eproto
