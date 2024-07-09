// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: lsm_vedodo/odo_pers_data_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_lsm_5fvedodo_2fodo_5fpers_5fdata_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_lsm_5fvedodo_2fodo_5fpers_5fdata_5fport_2eproto

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
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_lsm_5fvedodo_2fodo_5fpers_5fdata_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_lsm_5fvedodo_2fodo_5fpers_5fdata_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_lsm_5fvedodo_2fodo_5fpers_5fdata_5fport_2eproto;
namespace pb {
namespace lsm_vedodo {
namespace odo_pers_data_port {
class OdoPersDataPort;
class OdoPersDataPortDefaultTypeInternal;
extern OdoPersDataPortDefaultTypeInternal _OdoPersDataPort_default_instance_;
class OdoPersDataPort_array_port;
class OdoPersDataPort_array_portDefaultTypeInternal;
extern OdoPersDataPort_array_portDefaultTypeInternal _OdoPersDataPort_array_port_default_instance_;
}  // namespace odo_pers_data_port
}  // namespace lsm_vedodo
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort* Arena::CreateMaybeMessage<::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort>(Arena*);
template<> ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort_array_port* Arena::CreateMaybeMessage<::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace lsm_vedodo {
namespace odo_pers_data_port {

// ===================================================================

class OdoPersDataPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort) */ {
 public:
  OdoPersDataPort();
  virtual ~OdoPersDataPort();

  OdoPersDataPort(const OdoPersDataPort& from);
  OdoPersDataPort(OdoPersDataPort&& from) noexcept
    : OdoPersDataPort() {
    *this = ::std::move(from);
  }

  inline OdoPersDataPort& operator=(const OdoPersDataPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline OdoPersDataPort& operator=(OdoPersDataPort&& from) noexcept {
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
  static const OdoPersDataPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const OdoPersDataPort* internal_default_instance() {
    return reinterpret_cast<const OdoPersDataPort*>(
               &_OdoPersDataPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(OdoPersDataPort& a, OdoPersDataPort& b) {
    a.Swap(&b);
  }
  inline void Swap(OdoPersDataPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline OdoPersDataPort* New() const final {
    return CreateMaybeMessage<OdoPersDataPort>(nullptr);
  }

  OdoPersDataPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<OdoPersDataPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const OdoPersDataPort& from);
  void MergeFrom(const OdoPersDataPort& from);
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
  void InternalSwap(OdoPersDataPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_lsm_5fvedodo_2fodo_5fpers_5fdata_5fport_2eproto);
    return ::descriptor_table_lsm_5fvedodo_2fodo_5fpers_5fdata_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kSensorOffsetLateralAccelerationMpsFieldNumber = 3445,
    kSensorOffsetYawRateRadpsFieldNumber = 1628,
    kUiVersionNumberFieldNumber = 2124,
    kSensorOffsetSteeringWheelAngleRadFieldNumber = 2486,
    kSensorOffsetLongitudinalAccelerationMpsFieldNumber = 3025,
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

  // optional float SensorOffsetLateralAcceleration_mps = 3445;
  bool has_sensoroffsetlateralacceleration_mps() const;
  private:
  bool _internal_has_sensoroffsetlateralacceleration_mps() const;
  public:
  void clear_sensoroffsetlateralacceleration_mps();
  float sensoroffsetlateralacceleration_mps() const;
  void set_sensoroffsetlateralacceleration_mps(float value);
  private:
  float _internal_sensoroffsetlateralacceleration_mps() const;
  void _internal_set_sensoroffsetlateralacceleration_mps(float value);
  public:

  // optional float SensorOffsetYawRate_radps = 1628;
  bool has_sensoroffsetyawrate_radps() const;
  private:
  bool _internal_has_sensoroffsetyawrate_radps() const;
  public:
  void clear_sensoroffsetyawrate_radps();
  float sensoroffsetyawrate_radps() const;
  void set_sensoroffsetyawrate_radps(float value);
  private:
  float _internal_sensoroffsetyawrate_radps() const;
  void _internal_set_sensoroffsetyawrate_radps(float value);
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

  // optional float SensorOffsetSteeringWheelAngle_rad = 2486;
  bool has_sensoroffsetsteeringwheelangle_rad() const;
  private:
  bool _internal_has_sensoroffsetsteeringwheelangle_rad() const;
  public:
  void clear_sensoroffsetsteeringwheelangle_rad();
  float sensoroffsetsteeringwheelangle_rad() const;
  void set_sensoroffsetsteeringwheelangle_rad(float value);
  private:
  float _internal_sensoroffsetsteeringwheelangle_rad() const;
  void _internal_set_sensoroffsetsteeringwheelangle_rad(float value);
  public:

  // optional float SensorOffsetLongitudinalAcceleration_mps = 3025;
  bool has_sensoroffsetlongitudinalacceleration_mps() const;
  private:
  bool _internal_has_sensoroffsetlongitudinalacceleration_mps() const;
  public:
  void clear_sensoroffsetlongitudinalacceleration_mps();
  float sensoroffsetlongitudinalacceleration_mps() const;
  void set_sensoroffsetlongitudinalacceleration_mps(float value);
  private:
  float _internal_sensoroffsetlongitudinalacceleration_mps() const;
  void _internal_set_sensoroffsetlongitudinalacceleration_mps(float value);
  public:

  // @@protoc_insertion_point(class_scope:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  float sensoroffsetlateralacceleration_mps_;
  float sensoroffsetyawrate_radps_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  float sensoroffsetsteeringwheelangle_rad_;
  float sensoroffsetlongitudinalacceleration_mps_;
  friend struct ::TableStruct_lsm_5fvedodo_2fodo_5fpers_5fdata_5fport_2eproto;
};
// -------------------------------------------------------------------

class OdoPersDataPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort_array_port) */ {
 public:
  OdoPersDataPort_array_port();
  virtual ~OdoPersDataPort_array_port();

  OdoPersDataPort_array_port(const OdoPersDataPort_array_port& from);
  OdoPersDataPort_array_port(OdoPersDataPort_array_port&& from) noexcept
    : OdoPersDataPort_array_port() {
    *this = ::std::move(from);
  }

  inline OdoPersDataPort_array_port& operator=(const OdoPersDataPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline OdoPersDataPort_array_port& operator=(OdoPersDataPort_array_port&& from) noexcept {
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
  static const OdoPersDataPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const OdoPersDataPort_array_port* internal_default_instance() {
    return reinterpret_cast<const OdoPersDataPort_array_port*>(
               &_OdoPersDataPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(OdoPersDataPort_array_port& a, OdoPersDataPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(OdoPersDataPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline OdoPersDataPort_array_port* New() const final {
    return CreateMaybeMessage<OdoPersDataPort_array_port>(nullptr);
  }

  OdoPersDataPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<OdoPersDataPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const OdoPersDataPort_array_port& from);
  void MergeFrom(const OdoPersDataPort_array_port& from);
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
  void InternalSwap(OdoPersDataPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_lsm_5fvedodo_2fodo_5fpers_5fdata_5fport_2eproto);
    return ::descriptor_table_lsm_5fvedodo_2fodo_5fpers_5fdata_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2673,
  };
  // repeated .pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort data = 2673;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort >*
      mutable_data();
  private:
  const ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort& _internal_data(int index) const;
  ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort* _internal_add_data();
  public:
  const ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort& data(int index) const;
  ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort > data_;
  friend struct ::TableStruct_lsm_5fvedodo_2fodo_5fpers_5fdata_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// OdoPersDataPort

// optional uint32 uiVersionNumber = 2124;
inline bool OdoPersDataPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool OdoPersDataPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void OdoPersDataPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 OdoPersDataPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 OdoPersDataPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void OdoPersDataPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  uiversionnumber_ = value;
}
inline void OdoPersDataPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool OdoPersDataPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool OdoPersDataPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& OdoPersDataPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& OdoPersDataPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* OdoPersDataPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* OdoPersDataPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* OdoPersDataPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void OdoPersDataPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.sSigHeader)
}

// optional float SensorOffsetLateralAcceleration_mps = 3445;
inline bool OdoPersDataPort::_internal_has_sensoroffsetlateralacceleration_mps() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool OdoPersDataPort::has_sensoroffsetlateralacceleration_mps() const {
  return _internal_has_sensoroffsetlateralacceleration_mps();
}
inline void OdoPersDataPort::clear_sensoroffsetlateralacceleration_mps() {
  sensoroffsetlateralacceleration_mps_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline float OdoPersDataPort::_internal_sensoroffsetlateralacceleration_mps() const {
  return sensoroffsetlateralacceleration_mps_;
}
inline float OdoPersDataPort::sensoroffsetlateralacceleration_mps() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.SensorOffsetLateralAcceleration_mps)
  return _internal_sensoroffsetlateralacceleration_mps();
}
inline void OdoPersDataPort::_internal_set_sensoroffsetlateralacceleration_mps(float value) {
  _has_bits_[0] |= 0x00000002u;
  sensoroffsetlateralacceleration_mps_ = value;
}
inline void OdoPersDataPort::set_sensoroffsetlateralacceleration_mps(float value) {
  _internal_set_sensoroffsetlateralacceleration_mps(value);
  // @@protoc_insertion_point(field_set:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.SensorOffsetLateralAcceleration_mps)
}

// optional float SensorOffsetLongitudinalAcceleration_mps = 3025;
inline bool OdoPersDataPort::_internal_has_sensoroffsetlongitudinalacceleration_mps() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool OdoPersDataPort::has_sensoroffsetlongitudinalacceleration_mps() const {
  return _internal_has_sensoroffsetlongitudinalacceleration_mps();
}
inline void OdoPersDataPort::clear_sensoroffsetlongitudinalacceleration_mps() {
  sensoroffsetlongitudinalacceleration_mps_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline float OdoPersDataPort::_internal_sensoroffsetlongitudinalacceleration_mps() const {
  return sensoroffsetlongitudinalacceleration_mps_;
}
inline float OdoPersDataPort::sensoroffsetlongitudinalacceleration_mps() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.SensorOffsetLongitudinalAcceleration_mps)
  return _internal_sensoroffsetlongitudinalacceleration_mps();
}
inline void OdoPersDataPort::_internal_set_sensoroffsetlongitudinalacceleration_mps(float value) {
  _has_bits_[0] |= 0x00000020u;
  sensoroffsetlongitudinalacceleration_mps_ = value;
}
inline void OdoPersDataPort::set_sensoroffsetlongitudinalacceleration_mps(float value) {
  _internal_set_sensoroffsetlongitudinalacceleration_mps(value);
  // @@protoc_insertion_point(field_set:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.SensorOffsetLongitudinalAcceleration_mps)
}

// optional float SensorOffsetYawRate_radps = 1628;
inline bool OdoPersDataPort::_internal_has_sensoroffsetyawrate_radps() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool OdoPersDataPort::has_sensoroffsetyawrate_radps() const {
  return _internal_has_sensoroffsetyawrate_radps();
}
inline void OdoPersDataPort::clear_sensoroffsetyawrate_radps() {
  sensoroffsetyawrate_radps_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float OdoPersDataPort::_internal_sensoroffsetyawrate_radps() const {
  return sensoroffsetyawrate_radps_;
}
inline float OdoPersDataPort::sensoroffsetyawrate_radps() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.SensorOffsetYawRate_radps)
  return _internal_sensoroffsetyawrate_radps();
}
inline void OdoPersDataPort::_internal_set_sensoroffsetyawrate_radps(float value) {
  _has_bits_[0] |= 0x00000004u;
  sensoroffsetyawrate_radps_ = value;
}
inline void OdoPersDataPort::set_sensoroffsetyawrate_radps(float value) {
  _internal_set_sensoroffsetyawrate_radps(value);
  // @@protoc_insertion_point(field_set:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.SensorOffsetYawRate_radps)
}

// optional float SensorOffsetSteeringWheelAngle_rad = 2486;
inline bool OdoPersDataPort::_internal_has_sensoroffsetsteeringwheelangle_rad() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool OdoPersDataPort::has_sensoroffsetsteeringwheelangle_rad() const {
  return _internal_has_sensoroffsetsteeringwheelangle_rad();
}
inline void OdoPersDataPort::clear_sensoroffsetsteeringwheelangle_rad() {
  sensoroffsetsteeringwheelangle_rad_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline float OdoPersDataPort::_internal_sensoroffsetsteeringwheelangle_rad() const {
  return sensoroffsetsteeringwheelangle_rad_;
}
inline float OdoPersDataPort::sensoroffsetsteeringwheelangle_rad() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.SensorOffsetSteeringWheelAngle_rad)
  return _internal_sensoroffsetsteeringwheelangle_rad();
}
inline void OdoPersDataPort::_internal_set_sensoroffsetsteeringwheelangle_rad(float value) {
  _has_bits_[0] |= 0x00000010u;
  sensoroffsetsteeringwheelangle_rad_ = value;
}
inline void OdoPersDataPort::set_sensoroffsetsteeringwheelangle_rad(float value) {
  _internal_set_sensoroffsetsteeringwheelangle_rad(value);
  // @@protoc_insertion_point(field_set:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort.SensorOffsetSteeringWheelAngle_rad)
}

// -------------------------------------------------------------------

// OdoPersDataPort_array_port

// repeated .pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort data = 2673;
inline int OdoPersDataPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int OdoPersDataPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void OdoPersDataPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort* OdoPersDataPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort >*
OdoPersDataPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort_array_port.data)
  return &data_;
}
inline const ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort& OdoPersDataPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort& OdoPersDataPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort* OdoPersDataPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort* OdoPersDataPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::odo_pers_data_port::OdoPersDataPort >&
OdoPersDataPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.lsm_vedodo.odo_pers_data_port.OdoPersDataPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace odo_pers_data_port
}  // namespace lsm_vedodo
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_lsm_5fvedodo_2fodo_5fpers_5fdata_5fport_2eproto
