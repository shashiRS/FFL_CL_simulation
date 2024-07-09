// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/ego_motion_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_si_2fego_5fmotion_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_si_2fego_5fmotion_5fport_2eproto

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
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "eco/signal_header.pb.h"
#include "si/simotion_state.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_si_2fego_5fmotion_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_si_2fego_5fmotion_5fport_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[2]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fego_5fmotion_5fport_2eproto;
namespace pb {
namespace si {
namespace ego_motion_port {
class EgoMotionPort;
class EgoMotionPortDefaultTypeInternal;
extern EgoMotionPortDefaultTypeInternal _EgoMotionPort_default_instance_;
class EgoMotionPort_array_port;
class EgoMotionPort_array_portDefaultTypeInternal;
extern EgoMotionPort_array_portDefaultTypeInternal _EgoMotionPort_array_port_default_instance_;
}  // namespace ego_motion_port
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::si::ego_motion_port::EgoMotionPort* Arena::CreateMaybeMessage<::pb::si::ego_motion_port::EgoMotionPort>(Arena*);
template<> ::pb::si::ego_motion_port::EgoMotionPort_array_port* Arena::CreateMaybeMessage<::pb::si::ego_motion_port::EgoMotionPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace si {
namespace ego_motion_port {

// ===================================================================

class EgoMotionPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.ego_motion_port.EgoMotionPort) */ {
 public:
  EgoMotionPort();
  virtual ~EgoMotionPort();

  EgoMotionPort(const EgoMotionPort& from);
  EgoMotionPort(EgoMotionPort&& from) noexcept
    : EgoMotionPort() {
    *this = ::std::move(from);
  }

  inline EgoMotionPort& operator=(const EgoMotionPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline EgoMotionPort& operator=(EgoMotionPort&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const EgoMotionPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const EgoMotionPort* internal_default_instance() {
    return reinterpret_cast<const EgoMotionPort*>(
               &_EgoMotionPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(EgoMotionPort& a, EgoMotionPort& b) {
    a.Swap(&b);
  }
  inline void Swap(EgoMotionPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline EgoMotionPort* New() const final {
    return CreateMaybeMessage<EgoMotionPort>(nullptr);
  }

  EgoMotionPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<EgoMotionPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const EgoMotionPort& from);
  void MergeFrom(const EgoMotionPort& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(EgoMotionPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.ego_motion_port.EgoMotionPort";
  }
  private:
  inline ::PROTOBUF_NAMESPACE_ID::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fego_5fmotion_5fport_2eproto);
    return ::descriptor_table_si_2fego_5fmotion_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kAccelMps2FieldNumber = 244,
    kMotionStateNuFieldNumber = 330,
    kYawRateRadpsFieldNumber = 349,
    kVelMpsFieldNumber = 474,
    kFrontWheelAngleRadFieldNumber = 944,
    kRearWheelAngleRadFieldNumber = 1331,
    kDrivenDistanceMFieldNumber = 1548,
    kRollRadFieldNumber = 2073,
    kUiVersionNumberFieldNumber = 2124,
    kPitchRadFieldNumber = 4033,
  };
  // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
  bool has_ssigheader() const;
  private:
  bool _internal_has_ssigheader() const;
  public:
  void clear_ssigheader();
  const ::pb::eco::signal_header::SignalHeader& ssigheader() const;
  ::pb::eco::signal_header::SignalHeader* release_ssigheader();
  ::pb::eco::signal_header::SignalHeader* mutable_ssigheader();
  void set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader);
  private:
  const ::pb::eco::signal_header::SignalHeader& _internal_ssigheader() const;
  ::pb::eco::signal_header::SignalHeader* _internal_mutable_ssigheader();
  public:

  // optional float accel_mps2 = 244;
  bool has_accel_mps2() const;
  private:
  bool _internal_has_accel_mps2() const;
  public:
  void clear_accel_mps2();
  float accel_mps2() const;
  void set_accel_mps2(float value);
  private:
  float _internal_accel_mps2() const;
  void _internal_set_accel_mps2(float value);
  public:

  // optional .pb.si.simotion_state.SIMotionState motionState_nu = 330;
  bool has_motionstate_nu() const;
  private:
  bool _internal_has_motionstate_nu() const;
  public:
  void clear_motionstate_nu();
  ::pb::si::simotion_state::SIMotionState motionstate_nu() const;
  void set_motionstate_nu(::pb::si::simotion_state::SIMotionState value);
  private:
  ::pb::si::simotion_state::SIMotionState _internal_motionstate_nu() const;
  void _internal_set_motionstate_nu(::pb::si::simotion_state::SIMotionState value);
  public:

  // optional float yawRate_radps = 349;
  bool has_yawrate_radps() const;
  private:
  bool _internal_has_yawrate_radps() const;
  public:
  void clear_yawrate_radps();
  float yawrate_radps() const;
  void set_yawrate_radps(float value);
  private:
  float _internal_yawrate_radps() const;
  void _internal_set_yawrate_radps(float value);
  public:

  // optional float vel_mps = 474;
  bool has_vel_mps() const;
  private:
  bool _internal_has_vel_mps() const;
  public:
  void clear_vel_mps();
  float vel_mps() const;
  void set_vel_mps(float value);
  private:
  float _internal_vel_mps() const;
  void _internal_set_vel_mps(float value);
  public:

  // optional float frontWheelAngle_rad = 944;
  bool has_frontwheelangle_rad() const;
  private:
  bool _internal_has_frontwheelangle_rad() const;
  public:
  void clear_frontwheelangle_rad();
  float frontwheelangle_rad() const;
  void set_frontwheelangle_rad(float value);
  private:
  float _internal_frontwheelangle_rad() const;
  void _internal_set_frontwheelangle_rad(float value);
  public:

  // optional float rearWheelAngle_rad = 1331;
  bool has_rearwheelangle_rad() const;
  private:
  bool _internal_has_rearwheelangle_rad() const;
  public:
  void clear_rearwheelangle_rad();
  float rearwheelangle_rad() const;
  void set_rearwheelangle_rad(float value);
  private:
  float _internal_rearwheelangle_rad() const;
  void _internal_set_rearwheelangle_rad(float value);
  public:

  // optional float drivenDistance_m = 1548;
  bool has_drivendistance_m() const;
  private:
  bool _internal_has_drivendistance_m() const;
  public:
  void clear_drivendistance_m();
  float drivendistance_m() const;
  void set_drivendistance_m(float value);
  private:
  float _internal_drivendistance_m() const;
  void _internal_set_drivendistance_m(float value);
  public:

  // optional float roll_rad = 2073;
  bool has_roll_rad() const;
  private:
  bool _internal_has_roll_rad() const;
  public:
  void clear_roll_rad();
  float roll_rad() const;
  void set_roll_rad(float value);
  private:
  float _internal_roll_rad() const;
  void _internal_set_roll_rad(float value);
  public:

  // optional uint32 uiVersionNumber = 2124;
  bool has_uiversionnumber() const;
  private:
  bool _internal_has_uiversionnumber() const;
  public:
  void clear_uiversionnumber();
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber() const;
  void set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_uiversionnumber() const;
  void _internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional float pitch_rad = 4033;
  bool has_pitch_rad() const;
  private:
  bool _internal_has_pitch_rad() const;
  public:
  void clear_pitch_rad();
  float pitch_rad() const;
  void set_pitch_rad(float value);
  private:
  float _internal_pitch_rad() const;
  void _internal_set_pitch_rad(float value);
  public:

  // @@protoc_insertion_point(class_scope:pb.si.ego_motion_port.EgoMotionPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  float accel_mps2_;
  int motionstate_nu_;
  float yawrate_radps_;
  float vel_mps_;
  float frontwheelangle_rad_;
  float rearwheelangle_rad_;
  float drivendistance_m_;
  float roll_rad_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  float pitch_rad_;
  friend struct ::TableStruct_si_2fego_5fmotion_5fport_2eproto;
};
// -------------------------------------------------------------------

class EgoMotionPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.ego_motion_port.EgoMotionPort_array_port) */ {
 public:
  EgoMotionPort_array_port();
  virtual ~EgoMotionPort_array_port();

  EgoMotionPort_array_port(const EgoMotionPort_array_port& from);
  EgoMotionPort_array_port(EgoMotionPort_array_port&& from) noexcept
    : EgoMotionPort_array_port() {
    *this = ::std::move(from);
  }

  inline EgoMotionPort_array_port& operator=(const EgoMotionPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline EgoMotionPort_array_port& operator=(EgoMotionPort_array_port&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const EgoMotionPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const EgoMotionPort_array_port* internal_default_instance() {
    return reinterpret_cast<const EgoMotionPort_array_port*>(
               &_EgoMotionPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(EgoMotionPort_array_port& a, EgoMotionPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(EgoMotionPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline EgoMotionPort_array_port* New() const final {
    return CreateMaybeMessage<EgoMotionPort_array_port>(nullptr);
  }

  EgoMotionPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<EgoMotionPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const EgoMotionPort_array_port& from);
  void MergeFrom(const EgoMotionPort_array_port& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(EgoMotionPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.ego_motion_port.EgoMotionPort_array_port";
  }
  private:
  inline ::PROTOBUF_NAMESPACE_ID::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fego_5fmotion_5fport_2eproto);
    return ::descriptor_table_si_2fego_5fmotion_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1996,
  };
  // repeated .pb.si.ego_motion_port.EgoMotionPort data = 1996;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::si::ego_motion_port::EgoMotionPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::ego_motion_port::EgoMotionPort >*
      mutable_data();
  private:
  const ::pb::si::ego_motion_port::EgoMotionPort& _internal_data(int index) const;
  ::pb::si::ego_motion_port::EgoMotionPort* _internal_add_data();
  public:
  const ::pb::si::ego_motion_port::EgoMotionPort& data(int index) const;
  ::pb::si::ego_motion_port::EgoMotionPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::ego_motion_port::EgoMotionPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.si.ego_motion_port.EgoMotionPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::ego_motion_port::EgoMotionPort > data_;
  friend struct ::TableStruct_si_2fego_5fmotion_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// EgoMotionPort

// optional uint32 uiVersionNumber = 2124;
inline bool EgoMotionPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000200u) != 0;
  return value;
}
inline bool EgoMotionPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void EgoMotionPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000200u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 EgoMotionPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 EgoMotionPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.si.ego_motion_port.EgoMotionPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void EgoMotionPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000200u;
  uiversionnumber_ = value;
}
inline void EgoMotionPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.si.ego_motion_port.EgoMotionPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool EgoMotionPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool EgoMotionPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& EgoMotionPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& EgoMotionPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.si.ego_motion_port.EgoMotionPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* EgoMotionPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.si.ego_motion_port.EgoMotionPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* EgoMotionPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* EgoMotionPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.si.ego_motion_port.EgoMotionPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void EgoMotionPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(ssigheader_);
  }
  if (ssigheader) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      ssigheader = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, ssigheader, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  ssigheader_ = ssigheader;
  // @@protoc_insertion_point(field_set_allocated:pb.si.ego_motion_port.EgoMotionPort.sSigHeader)
}

// optional .pb.si.simotion_state.SIMotionState motionState_nu = 330;
inline bool EgoMotionPort::_internal_has_motionstate_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool EgoMotionPort::has_motionstate_nu() const {
  return _internal_has_motionstate_nu();
}
inline void EgoMotionPort::clear_motionstate_nu() {
  motionstate_nu_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::si::simotion_state::SIMotionState EgoMotionPort::_internal_motionstate_nu() const {
  return static_cast< ::pb::si::simotion_state::SIMotionState >(motionstate_nu_);
}
inline ::pb::si::simotion_state::SIMotionState EgoMotionPort::motionstate_nu() const {
  // @@protoc_insertion_point(field_get:pb.si.ego_motion_port.EgoMotionPort.motionState_nu)
  return _internal_motionstate_nu();
}
inline void EgoMotionPort::_internal_set_motionstate_nu(::pb::si::simotion_state::SIMotionState value) {
  assert(::pb::si::simotion_state::SIMotionState_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  motionstate_nu_ = value;
}
inline void EgoMotionPort::set_motionstate_nu(::pb::si::simotion_state::SIMotionState value) {
  _internal_set_motionstate_nu(value);
  // @@protoc_insertion_point(field_set:pb.si.ego_motion_port.EgoMotionPort.motionState_nu)
}

// optional float pitch_rad = 4033;
inline bool EgoMotionPort::_internal_has_pitch_rad() const {
  bool value = (_has_bits_[0] & 0x00000400u) != 0;
  return value;
}
inline bool EgoMotionPort::has_pitch_rad() const {
  return _internal_has_pitch_rad();
}
inline void EgoMotionPort::clear_pitch_rad() {
  pitch_rad_ = 0;
  _has_bits_[0] &= ~0x00000400u;
}
inline float EgoMotionPort::_internal_pitch_rad() const {
  return pitch_rad_;
}
inline float EgoMotionPort::pitch_rad() const {
  // @@protoc_insertion_point(field_get:pb.si.ego_motion_port.EgoMotionPort.pitch_rad)
  return _internal_pitch_rad();
}
inline void EgoMotionPort::_internal_set_pitch_rad(float value) {
  _has_bits_[0] |= 0x00000400u;
  pitch_rad_ = value;
}
inline void EgoMotionPort::set_pitch_rad(float value) {
  _internal_set_pitch_rad(value);
  // @@protoc_insertion_point(field_set:pb.si.ego_motion_port.EgoMotionPort.pitch_rad)
}

// optional float roll_rad = 2073;
inline bool EgoMotionPort::_internal_has_roll_rad() const {
  bool value = (_has_bits_[0] & 0x00000100u) != 0;
  return value;
}
inline bool EgoMotionPort::has_roll_rad() const {
  return _internal_has_roll_rad();
}
inline void EgoMotionPort::clear_roll_rad() {
  roll_rad_ = 0;
  _has_bits_[0] &= ~0x00000100u;
}
inline float EgoMotionPort::_internal_roll_rad() const {
  return roll_rad_;
}
inline float EgoMotionPort::roll_rad() const {
  // @@protoc_insertion_point(field_get:pb.si.ego_motion_port.EgoMotionPort.roll_rad)
  return _internal_roll_rad();
}
inline void EgoMotionPort::_internal_set_roll_rad(float value) {
  _has_bits_[0] |= 0x00000100u;
  roll_rad_ = value;
}
inline void EgoMotionPort::set_roll_rad(float value) {
  _internal_set_roll_rad(value);
  // @@protoc_insertion_point(field_set:pb.si.ego_motion_port.EgoMotionPort.roll_rad)
}

// optional float vel_mps = 474;
inline bool EgoMotionPort::_internal_has_vel_mps() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool EgoMotionPort::has_vel_mps() const {
  return _internal_has_vel_mps();
}
inline void EgoMotionPort::clear_vel_mps() {
  vel_mps_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline float EgoMotionPort::_internal_vel_mps() const {
  return vel_mps_;
}
inline float EgoMotionPort::vel_mps() const {
  // @@protoc_insertion_point(field_get:pb.si.ego_motion_port.EgoMotionPort.vel_mps)
  return _internal_vel_mps();
}
inline void EgoMotionPort::_internal_set_vel_mps(float value) {
  _has_bits_[0] |= 0x00000010u;
  vel_mps_ = value;
}
inline void EgoMotionPort::set_vel_mps(float value) {
  _internal_set_vel_mps(value);
  // @@protoc_insertion_point(field_set:pb.si.ego_motion_port.EgoMotionPort.vel_mps)
}

// optional float yawRate_radps = 349;
inline bool EgoMotionPort::_internal_has_yawrate_radps() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool EgoMotionPort::has_yawrate_radps() const {
  return _internal_has_yawrate_radps();
}
inline void EgoMotionPort::clear_yawrate_radps() {
  yawrate_radps_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float EgoMotionPort::_internal_yawrate_radps() const {
  return yawrate_radps_;
}
inline float EgoMotionPort::yawrate_radps() const {
  // @@protoc_insertion_point(field_get:pb.si.ego_motion_port.EgoMotionPort.yawRate_radps)
  return _internal_yawrate_radps();
}
inline void EgoMotionPort::_internal_set_yawrate_radps(float value) {
  _has_bits_[0] |= 0x00000008u;
  yawrate_radps_ = value;
}
inline void EgoMotionPort::set_yawrate_radps(float value) {
  _internal_set_yawrate_radps(value);
  // @@protoc_insertion_point(field_set:pb.si.ego_motion_port.EgoMotionPort.yawRate_radps)
}

// optional float accel_mps2 = 244;
inline bool EgoMotionPort::_internal_has_accel_mps2() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool EgoMotionPort::has_accel_mps2() const {
  return _internal_has_accel_mps2();
}
inline void EgoMotionPort::clear_accel_mps2() {
  accel_mps2_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline float EgoMotionPort::_internal_accel_mps2() const {
  return accel_mps2_;
}
inline float EgoMotionPort::accel_mps2() const {
  // @@protoc_insertion_point(field_get:pb.si.ego_motion_port.EgoMotionPort.accel_mps2)
  return _internal_accel_mps2();
}
inline void EgoMotionPort::_internal_set_accel_mps2(float value) {
  _has_bits_[0] |= 0x00000002u;
  accel_mps2_ = value;
}
inline void EgoMotionPort::set_accel_mps2(float value) {
  _internal_set_accel_mps2(value);
  // @@protoc_insertion_point(field_set:pb.si.ego_motion_port.EgoMotionPort.accel_mps2)
}

// optional float drivenDistance_m = 1548;
inline bool EgoMotionPort::_internal_has_drivendistance_m() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool EgoMotionPort::has_drivendistance_m() const {
  return _internal_has_drivendistance_m();
}
inline void EgoMotionPort::clear_drivendistance_m() {
  drivendistance_m_ = 0;
  _has_bits_[0] &= ~0x00000080u;
}
inline float EgoMotionPort::_internal_drivendistance_m() const {
  return drivendistance_m_;
}
inline float EgoMotionPort::drivendistance_m() const {
  // @@protoc_insertion_point(field_get:pb.si.ego_motion_port.EgoMotionPort.drivenDistance_m)
  return _internal_drivendistance_m();
}
inline void EgoMotionPort::_internal_set_drivendistance_m(float value) {
  _has_bits_[0] |= 0x00000080u;
  drivendistance_m_ = value;
}
inline void EgoMotionPort::set_drivendistance_m(float value) {
  _internal_set_drivendistance_m(value);
  // @@protoc_insertion_point(field_set:pb.si.ego_motion_port.EgoMotionPort.drivenDistance_m)
}

// optional float frontWheelAngle_rad = 944;
inline bool EgoMotionPort::_internal_has_frontwheelangle_rad() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool EgoMotionPort::has_frontwheelangle_rad() const {
  return _internal_has_frontwheelangle_rad();
}
inline void EgoMotionPort::clear_frontwheelangle_rad() {
  frontwheelangle_rad_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline float EgoMotionPort::_internal_frontwheelangle_rad() const {
  return frontwheelangle_rad_;
}
inline float EgoMotionPort::frontwheelangle_rad() const {
  // @@protoc_insertion_point(field_get:pb.si.ego_motion_port.EgoMotionPort.frontWheelAngle_rad)
  return _internal_frontwheelangle_rad();
}
inline void EgoMotionPort::_internal_set_frontwheelangle_rad(float value) {
  _has_bits_[0] |= 0x00000020u;
  frontwheelangle_rad_ = value;
}
inline void EgoMotionPort::set_frontwheelangle_rad(float value) {
  _internal_set_frontwheelangle_rad(value);
  // @@protoc_insertion_point(field_set:pb.si.ego_motion_port.EgoMotionPort.frontWheelAngle_rad)
}

// optional float rearWheelAngle_rad = 1331;
inline bool EgoMotionPort::_internal_has_rearwheelangle_rad() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool EgoMotionPort::has_rearwheelangle_rad() const {
  return _internal_has_rearwheelangle_rad();
}
inline void EgoMotionPort::clear_rearwheelangle_rad() {
  rearwheelangle_rad_ = 0;
  _has_bits_[0] &= ~0x00000040u;
}
inline float EgoMotionPort::_internal_rearwheelangle_rad() const {
  return rearwheelangle_rad_;
}
inline float EgoMotionPort::rearwheelangle_rad() const {
  // @@protoc_insertion_point(field_get:pb.si.ego_motion_port.EgoMotionPort.rearWheelAngle_rad)
  return _internal_rearwheelangle_rad();
}
inline void EgoMotionPort::_internal_set_rearwheelangle_rad(float value) {
  _has_bits_[0] |= 0x00000040u;
  rearwheelangle_rad_ = value;
}
inline void EgoMotionPort::set_rearwheelangle_rad(float value) {
  _internal_set_rearwheelangle_rad(value);
  // @@protoc_insertion_point(field_set:pb.si.ego_motion_port.EgoMotionPort.rearWheelAngle_rad)
}

// -------------------------------------------------------------------

// EgoMotionPort_array_port

// repeated .pb.si.ego_motion_port.EgoMotionPort data = 1996;
inline int EgoMotionPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int EgoMotionPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void EgoMotionPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::si::ego_motion_port::EgoMotionPort* EgoMotionPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.si.ego_motion_port.EgoMotionPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::ego_motion_port::EgoMotionPort >*
EgoMotionPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.si.ego_motion_port.EgoMotionPort_array_port.data)
  return &data_;
}
inline const ::pb::si::ego_motion_port::EgoMotionPort& EgoMotionPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::si::ego_motion_port::EgoMotionPort& EgoMotionPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.si.ego_motion_port.EgoMotionPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::si::ego_motion_port::EgoMotionPort* EgoMotionPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::si::ego_motion_port::EgoMotionPort* EgoMotionPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.si.ego_motion_port.EgoMotionPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::ego_motion_port::EgoMotionPort >&
EgoMotionPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.si.ego_motion_port.EgoMotionPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace ego_motion_port
}  // namespace si
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_si_2fego_5fmotion_5fport_2eproto
