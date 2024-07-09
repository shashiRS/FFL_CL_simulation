// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_hmitoap/parking_augmentation_type.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fhmitoap_2fparking_5faugmentation_5ftype_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fhmitoap_2fparking_5faugmentation_5ftype_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ap_5fhmitoap_2fparking_5faugmentation_5ftype_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fhmitoap_2fparking_5faugmentation_5ftype_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fhmitoap_2fparking_5faugmentation_5ftype_2eproto;
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_hmitoap {
namespace parking_augmentation_type {

enum ParkingAugmentationType : int {
  PARKING_AUGMENTATION_OFF = 0,
  PARKING_IN_ON = 1,
  PARKING_OUT_ON = 2
};
bool ParkingAugmentationType_IsValid(int value);
constexpr ParkingAugmentationType ParkingAugmentationType_MIN = PARKING_AUGMENTATION_OFF;
constexpr ParkingAugmentationType ParkingAugmentationType_MAX = PARKING_OUT_ON;
constexpr int ParkingAugmentationType_ARRAYSIZE = ParkingAugmentationType_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ParkingAugmentationType_descriptor();
template<typename T>
inline const std::string& ParkingAugmentationType_Name(T enum_t_value) {
  static_assert(::std::is_same<T, ParkingAugmentationType>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function ParkingAugmentationType_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    ParkingAugmentationType_descriptor(), enum_t_value);
}
inline bool ParkingAugmentationType_Parse(
    const std::string& name, ParkingAugmentationType* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<ParkingAugmentationType>(
    ParkingAugmentationType_descriptor(), name, value);
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

}  // namespace parking_augmentation_type
}  // namespace ap_hmitoap
}  // namespace pb

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::pb::ap_hmitoap::parking_augmentation_type::ParkingAugmentationType> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::pb::ap_hmitoap::parking_augmentation_type::ParkingAugmentationType>() {
  return ::pb::ap_hmitoap::parking_augmentation_type::ParkingAugmentationType_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fhmitoap_2fparking_5faugmentation_5ftype_2eproto