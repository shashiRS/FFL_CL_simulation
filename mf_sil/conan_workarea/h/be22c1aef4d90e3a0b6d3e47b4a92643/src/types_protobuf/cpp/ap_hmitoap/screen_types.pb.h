// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_hmitoap/screen_types.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fhmitoap_2fscreen_5ftypes_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fhmitoap_2fscreen_5ftypes_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ap_5fhmitoap_2fscreen_5ftypes_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fhmitoap_2fscreen_5ftypes_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fhmitoap_2fscreen_5ftypes_2eproto;
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_hmitoap {
namespace screen_types {

enum ScreenTypes : int {
  BLANK_SCREEN = 0,
  BOWL_VIEW_1 = 1,
  BOWL_VIEW_2 = 2,
  BOWL_VIEW_3 = 3,
  BOWL_VIEW_4 = 4,
  BOWL_VIEW_5 = 5,
  BOWL_VIEW_6 = 6,
  BOWL_VIEW_7 = 7,
  BOWL_VIEW_8 = 8,
  BOWL_VIEW_9 = 9,
  BOWL_VIEW_10 = 10,
  BOWL_VIEW_11 = 11,
  BOWL_VIEW_12 = 12,
  BOWL_VIEW_13 = 13,
  BOWL_VIEW_14 = 14,
  BOWL_VIEW_MODIFIED = 15,
  FRONT_VIEW_FULL = 16,
  FRONT_VIEW_SPLIT = 17,
  PANORAMIC_FRONT_VIEW = 18,
  BLACK_SCREEN = 19,
  FRONT_TOP_ZOOM = 20,
  LAST_FRONT_VIEW = 30,
  TOP_VIEW = 31,
  TOP_VIEW_SFM = 32,
  TOP_VIEW_PSD = 33,
  TOP_VIEW_PMD = 34,
  LAST_TOP_VIEW = 45,
  REAR_VIEW_FULL = 46,
  REAR_VIEW_SPLIT = 47,
  PANORAMIC_REAR_VIEW = 48,
  TRAILER_VIEW = 49,
  REAR_TOP_ZOOM = 50,
  LAST_REAR_VIEW = 60,
  KERB_VIEW_FRONT = 61,
  KERB_VIEW_REAR = 62,
  LAST_KERB_VIEW = 75,
  ARA_REAR_NORMAL = 76,
  ARA_REAR_IRREGULAR = 77,
  ARA_PANORAMIC_NORMAL = 78,
  ARA_PANORAMIC_IRREGULAR = 79,
  LEFT_VIEW_SPLIT = 80,
  LEFT_VIEW_FULL = 81,
  PANORAMIC_LEFT_VIEW = 82,
  RIGHT_VIEW_SPLIT = 83,
  RIGHT_VIEW_FULL = 84,
  PANORAMIC_RIGHT_VIEW = 85,
  LAST_ARA_VIEW = 90,
  IPA_FRONT_VIEW = 91,
  IPA_REAR_VIEW = 92,
  IPA_TOP_VIEW = 93,
  PARK_FRONT_VIEW = 94,
  PARK_REAR_VIEW = 95,
  PARK_TOP_VIEW = 96,
  PARK_SPACES_TOP_VIEW = 97,
  PARK_WARN_TOP_VIEW = 98,
  PARK_TRANSPARENT_PDW = 99,
  LAST_IPA_VIEW = 105,
  MORPH_FRONT = 106,
  MORPH_FRONT_FULL = 107,
  LAST_MORPH_FRONT = 120,
  MORPH_REAR = 121,
  MORPH_REAR_FULL = 122,
  MORPH_ARA = 123,
  LAST_MORPH_VIEW = 135,
  SMARTPHONE_VIEW = 151,
  BLIND_SPOT_LEFT = 152,
  BLIND_SPOT_RIGHT = 153,
  LAST_SPECIAL_VIEW = 165,
  TRANSPARENT_HOOD = 166,
  TRANSPARENT_ALPHA = 167,
  TRANSPARENT_GHOST = 168,
  TRANSPARENT_TRUNK = 169,
  TRANSPARENT_ALPHA_SPLIT = 170,
  TRANSPARENT_HOOD_SPLIT = 171,
  TRANSPARENT_TRUNK_SPLIT = 172,
  ADAPTIVE_FULL_BOWL = 173,
  STATIC_BOWL = 174,
  ADAPTIVE_SPLIT_BOWL = 175,
  STATIC_SPLIT_BOWL = 176,
  RAW_CAMERAS = 181,
  RAW_CAMERAS_SFM = 182,
  DEBUG_FRONT_CAMERA_EXTERN = 200,
  DEBUG_FRONT_CAMERA_VECTORS = 201,
  DEBUG_FRONT_CAMERA_EDGES = 202,
  DEBUG_FRONT_CAMERA_CLEANING = 203,
  DEBUG_REAR_CAMERA_EXTERN = 204,
  DEBUG_REAR_CAMERA_VECTORS = 205,
  DEBUG_REAR_CAMERA_EDGES = 206,
  DEBUG_REAR_CAMERA_CLEANING = 207,
  DEBUG_LEFT_CAMERA_EXTERN = 208,
  DEBUG_LEFT_CAMERA_VECTORS = 209,
  DEBUG_LEFT_CAMERA_EDGES = 210,
  DEBUG_LEFT_CAMERA_CLEANING = 211,
  DEBUG_RIGHT_CAMERA_EXTERN = 212,
  DEBUG_RIGHT_CAMERA_VECTORS = 213,
  DEBUG_RIGHT_CAMERA_EDGES = 214,
  DEBUG_RIGHT_CAMERA_CLEANING = 215,
  DEBUG_CALIBRATION_FLMC_QUAD_VIEW = 216,
  DEBUG_CALIBRATION_PGM_QUAD_VIEW = 217,
  DEBUG_TESTSCREEN_4_CAMS = 218,
  DEBUG_CALIBRATION_OLMC_QUAD_VIEW = 219,
  DEBUG_COLOUR_BARS = 220,
  DEBUG_RAW_CAMERAS = 221,
  DEBUG_RAW_CAMERAS_SFM = 222,
  DEBUG_CURB_MESH_DEMO = 223,
  DEBUG_HEIGHT_MAP = 224,
  DEBUG_PGM_QUAD_OVERLAY = 225,
  DEBUG_IMAGE_HARMONIZATION = 226,
  PARKING_IN_SLOT_SELECTION_VIEW = 227,
  PARKING_IN_SLOT_CONFIRMATION_VIEW = 228,
  PARKING_IN_MANEUVER_PHASE_FORWARD_VIEW = 229,
  PARKING_IN_MANEUVER_PHASE_BACKWARD_VIEW = 230,
  PARKING_IN_MANEUVER_APPROACH_PHASE_VIEW = 231,
  PARKING_IN_MANEUVER_APPROACH_WHEELSTOPPER_PHASE_FORWARD_VIEW = 232,
  PARKING_IN_MANEUVER_APPROACH_WHEELSTOPPER_PHASE_BACKWARD_VIEW = 233,
  PARKING_OUT_DIRECTION_SELECTION_VIEW = 234,
  PARKING_OUT_MANEUVER_FORWARD_VIEW = 235,
  PARKING_OUT_MANEUVER_BACKWARD_VIEW = 236,
  PARKING_OUT_MANEUVER_FINISH_PHASE_VIEW = 237,
  NO_STREAM_CHANGE = 255,
  SIZE = 256
};
bool ScreenTypes_IsValid(int value);
constexpr ScreenTypes ScreenTypes_MIN = BLANK_SCREEN;
constexpr ScreenTypes ScreenTypes_MAX = SIZE;
constexpr int ScreenTypes_ARRAYSIZE = ScreenTypes_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ScreenTypes_descriptor();
template<typename T>
inline const std::string& ScreenTypes_Name(T enum_t_value) {
  static_assert(::std::is_same<T, ScreenTypes>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function ScreenTypes_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    ScreenTypes_descriptor(), enum_t_value);
}
inline bool ScreenTypes_Parse(
    const std::string& name, ScreenTypes* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<ScreenTypes>(
    ScreenTypes_descriptor(), name, value);
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

}  // namespace screen_types
}  // namespace ap_hmitoap
}  // namespace pb

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::pb::ap_hmitoap::screen_types::ScreenTypes> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::pb::ap_hmitoap::screen_types::ScreenTypes>() {
  return ::pb::ap_hmitoap::screen_types::ScreenTypes_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fhmitoap_2fscreen_5ftypes_2eproto