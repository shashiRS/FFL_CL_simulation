// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_manager/lo_ctrl_request_source.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fmanager_2flo_5fctrl_5frequest_5fsource_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fmanager_2flo_5fctrl_5frequest_5fsource_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_mf_5fmanager_2flo_5fctrl_5frequest_5fsource_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fmanager_2flo_5fctrl_5frequest_5fsource_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmanager_2flo_5fctrl_5frequest_5fsource_2eproto;
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_manager {
namespace lo_ctrl_request_source {

enum loCtrlRequestSource : int {
  LODMC_REQ_SRC_NO_REQEUESTER = 0,
  LODMC_REQ_SRC_AUP = 1,
  LODMC_REQ_SRC_AUP_WITH_REMOTE = 6,
  LODMC_REQ_SRC_LSCA = 2,
  LODMC_REQ_SRC_REMOTE_MANEUVERING = 3,
  LODMC_REQ_SRC_MSP = 4,
  LODMC_REQ_SRC_SAFBAR = 5
};
bool loCtrlRequestSource_IsValid(int value);
constexpr loCtrlRequestSource loCtrlRequestSource_MIN = LODMC_REQ_SRC_NO_REQEUESTER;
constexpr loCtrlRequestSource loCtrlRequestSource_MAX = LODMC_REQ_SRC_AUP_WITH_REMOTE;
constexpr int loCtrlRequestSource_ARRAYSIZE = loCtrlRequestSource_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* loCtrlRequestSource_descriptor();
template<typename T>
inline const std::string& loCtrlRequestSource_Name(T enum_t_value) {
  static_assert(::std::is_same<T, loCtrlRequestSource>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function loCtrlRequestSource_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    loCtrlRequestSource_descriptor(), enum_t_value);
}
inline bool loCtrlRequestSource_Parse(
    const std::string& name, loCtrlRequestSource* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<loCtrlRequestSource>(
    loCtrlRequestSource_descriptor(), name, value);
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

}  // namespace lo_ctrl_request_source
}  // namespace mf_manager
}  // namespace pb

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::pb::mf_manager::lo_ctrl_request_source::loCtrlRequestSource> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::pb::mf_manager::lo_ctrl_request_source::loCtrlRequestSource>() {
  return ::pb::mf_manager::lo_ctrl_request_source::loCtrlRequestSource_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fmanager_2flo_5fctrl_5frequest_5fsource_2eproto
