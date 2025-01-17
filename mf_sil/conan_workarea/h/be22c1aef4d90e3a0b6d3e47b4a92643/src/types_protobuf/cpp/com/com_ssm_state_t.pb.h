// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: com/com_ssm_state_t.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_com_2fcom_5fssm_5fstate_5ft_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_com_2fcom_5fssm_5fstate_5ft_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_com_2fcom_5fssm_5fstate_5ft_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_com_2fcom_5fssm_5fstate_5ft_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_com_2fcom_5fssm_5fstate_5ft_2eproto;
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace com {
namespace com_ssm_state_t {

enum ComSSM_State_t : int {
  COM_SSM_SYS_INIT = 0,
  COM_SSM_SYS_RUN = 1,
  COM_SSM_SYS_RUN_LIMITED = 2,
  COM_SSM_SYS_FAILSAFE = 3,
  COM_SSM_SYS_SHUTDOWN = 4,
  COM_SSM_SD_OFF = 0,
  COM_SSM_SD_RESTART = 1,
  COM_SSM_SD_GET_PERS_DATA = 2
};
bool ComSSM_State_t_IsValid(int value);
constexpr ComSSM_State_t ComSSM_State_t_MIN = COM_SSM_SYS_INIT;
constexpr ComSSM_State_t ComSSM_State_t_MAX = COM_SSM_SYS_SHUTDOWN;
constexpr int ComSSM_State_t_ARRAYSIZE = ComSSM_State_t_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ComSSM_State_t_descriptor();
template<typename T>
inline const std::string& ComSSM_State_t_Name(T enum_t_value) {
  static_assert(::std::is_same<T, ComSSM_State_t>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function ComSSM_State_t_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    ComSSM_State_t_descriptor(), enum_t_value);
}
inline bool ComSSM_State_t_Parse(
    const std::string& name, ComSSM_State_t* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<ComSSM_State_t>(
    ComSSM_State_t_descriptor(), name, value);
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

}  // namespace com_ssm_state_t
}  // namespace com
}  // namespace pb

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::pb::com::com_ssm_state_t::ComSSM_State_t> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::pb::com::com_ssm_state_t::ComSSM_State_t>() {
  return ::pb::com::com_ssm_state_t::ComSSM_State_t_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_com_2fcom_5fssm_5fstate_5ft_2eproto
