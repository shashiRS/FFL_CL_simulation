// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_common/vehicle_side.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fcommon_2fvehicle_5fside_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fcommon_2fvehicle_5fside_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ap_5fcommon_2fvehicle_5fside_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fcommon_2fvehicle_5fside_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fcommon_2fvehicle_5fside_2eproto;
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_common {
namespace vehicle_side {

enum VehicleSide : int {
  VEHICLE_FRONT = 0,
  VEHICLE_LEFT_SIDE = 1,
  VEHICLE_REAR = 2,
  VEHICLE_RIGHT_SIDE = 3,
  VEHICLE_NO_SIDE = 4
};
bool VehicleSide_IsValid(int value);
constexpr VehicleSide VehicleSide_MIN = VEHICLE_FRONT;
constexpr VehicleSide VehicleSide_MAX = VEHICLE_NO_SIDE;
constexpr int VehicleSide_ARRAYSIZE = VehicleSide_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* VehicleSide_descriptor();
template<typename T>
inline const std::string& VehicleSide_Name(T enum_t_value) {
  static_assert(::std::is_same<T, VehicleSide>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function VehicleSide_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    VehicleSide_descriptor(), enum_t_value);
}
inline bool VehicleSide_Parse(
    const std::string& name, VehicleSide* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<VehicleSide>(
    VehicleSide_descriptor(), name, value);
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

}  // namespace vehicle_side
}  // namespace ap_common
}  // namespace pb

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::pb::ap_common::vehicle_side::VehicleSide> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::pb::ap_common::vehicle_side::VehicleSide>() {
  return ::pb::ap_common::vehicle_side::VehicleSide_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fcommon_2fvehicle_5fside_2eproto