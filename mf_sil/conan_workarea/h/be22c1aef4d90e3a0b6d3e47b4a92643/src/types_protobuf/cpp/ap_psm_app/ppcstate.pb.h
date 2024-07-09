// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_psm_app/ppcstate.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_5fapp_2fppcstate_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_5fapp_2fppcstate_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ap_5fpsm_5fapp_2fppcstate_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fpsm_5fapp_2fppcstate_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fpsm_5fapp_2fppcstate_2eproto;
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_psm_app {
namespace ppcstate {

enum PPCState : int {
  PPC_INACTIVE = 0,
  PPC_BEHAVIOR_INACTIVE = 1,
  PPC_SCANNING_INACTIVE = 2,
  PPC_SCANNING_IN = 3,
  PPC_SCANNING_OUT = 4,
  PPC_SCANNING_RM = 5,
  PPC_PARKING_INACTIVE = 6,
  PPC_PREPARE_PARKING = 7,
  PPC_PERFORM_PARKING = 8,
  PPC_PARKING_PAUSE = 9,
  PPC_PARKING_FAILED = 10,
  PPC_OFFER_HANDOVER = 11,
  PPC_SUCCESS = 12,
  PPC_PARKING_CANCELED = 13,
  PPC_IRREVERSIBLE_ERROR = 14,
  PPC_REVERSIBLE_ERROR = 15,
  PPC_DEMO_OFF = 23,
  PPC_SCANNING_GP = 24
};
bool PPCState_IsValid(int value);
constexpr PPCState PPCState_MIN = PPC_INACTIVE;
constexpr PPCState PPCState_MAX = PPC_SCANNING_GP;
constexpr int PPCState_ARRAYSIZE = PPCState_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* PPCState_descriptor();
template<typename T>
inline const std::string& PPCState_Name(T enum_t_value) {
  static_assert(::std::is_same<T, PPCState>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function PPCState_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    PPCState_descriptor(), enum_t_value);
}
inline bool PPCState_Parse(
    const std::string& name, PPCState* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<PPCState>(
    PPCState_descriptor(), name, value);
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

}  // namespace ppcstate
}  // namespace ap_psm_app
}  // namespace pb

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::pb::ap_psm_app::ppcstate::PPCState> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::pb::ap_psm_app::ppcstate::PPCState>() {
  return ::pb::ap_psm_app::ppcstate::PPCState_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_5fapp_2fppcstate_2eproto