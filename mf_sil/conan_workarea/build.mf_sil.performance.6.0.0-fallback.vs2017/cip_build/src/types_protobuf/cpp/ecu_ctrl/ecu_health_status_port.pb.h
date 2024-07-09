// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ecu_ctrl/ecu_health_status_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ecu_5fctrl_2fecu_5fhealth_5fstatus_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ecu_5fctrl_2fecu_5fhealth_5fstatus_5fport_2eproto

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
#include "ecu_ctrl/ecu_state.pb.h"
#include "ecu_ctrl/voltage_state.pb.h"
#include "ecu_ctrl/comm_state.pb.h"
#include "ecu_ctrl/temp_state.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ecu_5fctrl_2fecu_5fhealth_5fstatus_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ecu_5fctrl_2fecu_5fhealth_5fstatus_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ecu_5fctrl_2fecu_5fhealth_5fstatus_5fport_2eproto;
namespace pb {
namespace ecu_ctrl {
namespace ecu_health_status_port {
class EcuHealthStatusPort;
class EcuHealthStatusPortDefaultTypeInternal;
extern EcuHealthStatusPortDefaultTypeInternal _EcuHealthStatusPort_default_instance_;
class EcuHealthStatusPort_array_port;
class EcuHealthStatusPort_array_portDefaultTypeInternal;
extern EcuHealthStatusPort_array_portDefaultTypeInternal _EcuHealthStatusPort_array_port_default_instance_;
}  // namespace ecu_health_status_port
}  // namespace ecu_ctrl
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort* Arena::CreateMaybeMessage<::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort>(Arena*);
template<> ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort_array_port* Arena::CreateMaybeMessage<::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ecu_ctrl {
namespace ecu_health_status_port {

// ===================================================================

class EcuHealthStatusPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort) */ {
 public:
  EcuHealthStatusPort();
  virtual ~EcuHealthStatusPort();

  EcuHealthStatusPort(const EcuHealthStatusPort& from);
  EcuHealthStatusPort(EcuHealthStatusPort&& from) noexcept
    : EcuHealthStatusPort() {
    *this = ::std::move(from);
  }

  inline EcuHealthStatusPort& operator=(const EcuHealthStatusPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline EcuHealthStatusPort& operator=(EcuHealthStatusPort&& from) noexcept {
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
  static const EcuHealthStatusPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const EcuHealthStatusPort* internal_default_instance() {
    return reinterpret_cast<const EcuHealthStatusPort*>(
               &_EcuHealthStatusPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(EcuHealthStatusPort& a, EcuHealthStatusPort& b) {
    a.Swap(&b);
  }
  inline void Swap(EcuHealthStatusPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline EcuHealthStatusPort* New() const final {
    return CreateMaybeMessage<EcuHealthStatusPort>(nullptr);
  }

  EcuHealthStatusPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<EcuHealthStatusPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const EcuHealthStatusPort& from);
  void MergeFrom(const EcuHealthStatusPort& from);
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
  void InternalSwap(EcuHealthStatusPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ecu_5fctrl_2fecu_5fhealth_5fstatus_5fport_2eproto);
    return ::descriptor_table_ecu_5fctrl_2fecu_5fhealth_5fstatus_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kGlobalStateEFieldNumber = 3596,
    kCommStateEFieldNumber = 1491,
    kTemperatureStateEFieldNumber = 2194,
    kEcuVoltageStateEFieldNumber = 2754,
    kEcuVoltageValueVFieldNumber = 3592,
    kTimestampUsU64FieldNumber = 3650,
  };
  // optional .pb.ecu_ctrl.ecu_state.EcuState globalState_e = 3596;
  bool has_globalstate_e() const;
  private:
  bool _internal_has_globalstate_e() const;
  public:
  void clear_globalstate_e();
  ::pb::ecu_ctrl::ecu_state::EcuState globalstate_e() const;
  void set_globalstate_e(::pb::ecu_ctrl::ecu_state::EcuState value);
  private:
  ::pb::ecu_ctrl::ecu_state::EcuState _internal_globalstate_e() const;
  void _internal_set_globalstate_e(::pb::ecu_ctrl::ecu_state::EcuState value);
  public:

  // optional .pb.ecu_ctrl.comm_state.CommState commState_e = 1491;
  bool has_commstate_e() const;
  private:
  bool _internal_has_commstate_e() const;
  public:
  void clear_commstate_e();
  ::pb::ecu_ctrl::comm_state::CommState commstate_e() const;
  void set_commstate_e(::pb::ecu_ctrl::comm_state::CommState value);
  private:
  ::pb::ecu_ctrl::comm_state::CommState _internal_commstate_e() const;
  void _internal_set_commstate_e(::pb::ecu_ctrl::comm_state::CommState value);
  public:

  // optional .pb.ecu_ctrl.temp_state.TempState temperatureState_e = 2194;
  bool has_temperaturestate_e() const;
  private:
  bool _internal_has_temperaturestate_e() const;
  public:
  void clear_temperaturestate_e();
  ::pb::ecu_ctrl::temp_state::TempState temperaturestate_e() const;
  void set_temperaturestate_e(::pb::ecu_ctrl::temp_state::TempState value);
  private:
  ::pb::ecu_ctrl::temp_state::TempState _internal_temperaturestate_e() const;
  void _internal_set_temperaturestate_e(::pb::ecu_ctrl::temp_state::TempState value);
  public:

  // optional .pb.ecu_ctrl.voltage_state.VoltageState ecuVoltageState_e = 2754;
  bool has_ecuvoltagestate_e() const;
  private:
  bool _internal_has_ecuvoltagestate_e() const;
  public:
  void clear_ecuvoltagestate_e();
  ::pb::ecu_ctrl::voltage_state::VoltageState ecuvoltagestate_e() const;
  void set_ecuvoltagestate_e(::pb::ecu_ctrl::voltage_state::VoltageState value);
  private:
  ::pb::ecu_ctrl::voltage_state::VoltageState _internal_ecuvoltagestate_e() const;
  void _internal_set_ecuvoltagestate_e(::pb::ecu_ctrl::voltage_state::VoltageState value);
  public:

  // optional float ecuVoltageValue_V = 3592;
  bool has_ecuvoltagevalue_v() const;
  private:
  bool _internal_has_ecuvoltagevalue_v() const;
  public:
  void clear_ecuvoltagevalue_v();
  float ecuvoltagevalue_v() const;
  void set_ecuvoltagevalue_v(float value);
  private:
  float _internal_ecuvoltagevalue_v() const;
  void _internal_set_ecuvoltagevalue_v(float value);
  public:

  // optional uint64 timestamp_us_u64 = 3650;
  bool has_timestamp_us_u64() const;
  private:
  bool _internal_has_timestamp_us_u64() const;
  public:
  void clear_timestamp_us_u64();
  ::PROTOBUF_NAMESPACE_ID::uint64 timestamp_us_u64() const;
  void set_timestamp_us_u64(::PROTOBUF_NAMESPACE_ID::uint64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint64 _internal_timestamp_us_u64() const;
  void _internal_set_timestamp_us_u64(::PROTOBUF_NAMESPACE_ID::uint64 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int globalstate_e_;
  int commstate_e_;
  int temperaturestate_e_;
  int ecuvoltagestate_e_;
  float ecuvoltagevalue_v_;
  ::PROTOBUF_NAMESPACE_ID::uint64 timestamp_us_u64_;
  friend struct ::TableStruct_ecu_5fctrl_2fecu_5fhealth_5fstatus_5fport_2eproto;
};
// -------------------------------------------------------------------

class EcuHealthStatusPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort_array_port) */ {
 public:
  EcuHealthStatusPort_array_port();
  virtual ~EcuHealthStatusPort_array_port();

  EcuHealthStatusPort_array_port(const EcuHealthStatusPort_array_port& from);
  EcuHealthStatusPort_array_port(EcuHealthStatusPort_array_port&& from) noexcept
    : EcuHealthStatusPort_array_port() {
    *this = ::std::move(from);
  }

  inline EcuHealthStatusPort_array_port& operator=(const EcuHealthStatusPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline EcuHealthStatusPort_array_port& operator=(EcuHealthStatusPort_array_port&& from) noexcept {
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
  static const EcuHealthStatusPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const EcuHealthStatusPort_array_port* internal_default_instance() {
    return reinterpret_cast<const EcuHealthStatusPort_array_port*>(
               &_EcuHealthStatusPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(EcuHealthStatusPort_array_port& a, EcuHealthStatusPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(EcuHealthStatusPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline EcuHealthStatusPort_array_port* New() const final {
    return CreateMaybeMessage<EcuHealthStatusPort_array_port>(nullptr);
  }

  EcuHealthStatusPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<EcuHealthStatusPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const EcuHealthStatusPort_array_port& from);
  void MergeFrom(const EcuHealthStatusPort_array_port& from);
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
  void InternalSwap(EcuHealthStatusPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ecu_5fctrl_2fecu_5fhealth_5fstatus_5fport_2eproto);
    return ::descriptor_table_ecu_5fctrl_2fecu_5fhealth_5fstatus_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1289,
  };
  // repeated .pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort data = 1289;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort >*
      mutable_data();
  private:
  const ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort& _internal_data(int index) const;
  ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort* _internal_add_data();
  public:
  const ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort& data(int index) const;
  ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort > data_;
  friend struct ::TableStruct_ecu_5fctrl_2fecu_5fhealth_5fstatus_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// EcuHealthStatusPort

// optional uint64 timestamp_us_u64 = 3650;
inline bool EcuHealthStatusPort::_internal_has_timestamp_us_u64() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool EcuHealthStatusPort::has_timestamp_us_u64() const {
  return _internal_has_timestamp_us_u64();
}
inline void EcuHealthStatusPort::clear_timestamp_us_u64() {
  timestamp_us_u64_ = PROTOBUF_ULONGLONG(0);
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 EcuHealthStatusPort::_internal_timestamp_us_u64() const {
  return timestamp_us_u64_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 EcuHealthStatusPort::timestamp_us_u64() const {
  // @@protoc_insertion_point(field_get:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort.timestamp_us_u64)
  return _internal_timestamp_us_u64();
}
inline void EcuHealthStatusPort::_internal_set_timestamp_us_u64(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _has_bits_[0] |= 0x00000020u;
  timestamp_us_u64_ = value;
}
inline void EcuHealthStatusPort::set_timestamp_us_u64(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _internal_set_timestamp_us_u64(value);
  // @@protoc_insertion_point(field_set:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort.timestamp_us_u64)
}

// optional float ecuVoltageValue_V = 3592;
inline bool EcuHealthStatusPort::_internal_has_ecuvoltagevalue_v() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool EcuHealthStatusPort::has_ecuvoltagevalue_v() const {
  return _internal_has_ecuvoltagevalue_v();
}
inline void EcuHealthStatusPort::clear_ecuvoltagevalue_v() {
  ecuvoltagevalue_v_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline float EcuHealthStatusPort::_internal_ecuvoltagevalue_v() const {
  return ecuvoltagevalue_v_;
}
inline float EcuHealthStatusPort::ecuvoltagevalue_v() const {
  // @@protoc_insertion_point(field_get:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort.ecuVoltageValue_V)
  return _internal_ecuvoltagevalue_v();
}
inline void EcuHealthStatusPort::_internal_set_ecuvoltagevalue_v(float value) {
  _has_bits_[0] |= 0x00000010u;
  ecuvoltagevalue_v_ = value;
}
inline void EcuHealthStatusPort::set_ecuvoltagevalue_v(float value) {
  _internal_set_ecuvoltagevalue_v(value);
  // @@protoc_insertion_point(field_set:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort.ecuVoltageValue_V)
}

// optional .pb.ecu_ctrl.ecu_state.EcuState globalState_e = 3596;
inline bool EcuHealthStatusPort::_internal_has_globalstate_e() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool EcuHealthStatusPort::has_globalstate_e() const {
  return _internal_has_globalstate_e();
}
inline void EcuHealthStatusPort::clear_globalstate_e() {
  globalstate_e_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::pb::ecu_ctrl::ecu_state::EcuState EcuHealthStatusPort::_internal_globalstate_e() const {
  return static_cast< ::pb::ecu_ctrl::ecu_state::EcuState >(globalstate_e_);
}
inline ::pb::ecu_ctrl::ecu_state::EcuState EcuHealthStatusPort::globalstate_e() const {
  // @@protoc_insertion_point(field_get:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort.globalState_e)
  return _internal_globalstate_e();
}
inline void EcuHealthStatusPort::_internal_set_globalstate_e(::pb::ecu_ctrl::ecu_state::EcuState value) {
  assert(::pb::ecu_ctrl::ecu_state::EcuState_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  globalstate_e_ = value;
}
inline void EcuHealthStatusPort::set_globalstate_e(::pb::ecu_ctrl::ecu_state::EcuState value) {
  _internal_set_globalstate_e(value);
  // @@protoc_insertion_point(field_set:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort.globalState_e)
}

// optional .pb.ecu_ctrl.voltage_state.VoltageState ecuVoltageState_e = 2754;
inline bool EcuHealthStatusPort::_internal_has_ecuvoltagestate_e() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool EcuHealthStatusPort::has_ecuvoltagestate_e() const {
  return _internal_has_ecuvoltagestate_e();
}
inline void EcuHealthStatusPort::clear_ecuvoltagestate_e() {
  ecuvoltagestate_e_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::pb::ecu_ctrl::voltage_state::VoltageState EcuHealthStatusPort::_internal_ecuvoltagestate_e() const {
  return static_cast< ::pb::ecu_ctrl::voltage_state::VoltageState >(ecuvoltagestate_e_);
}
inline ::pb::ecu_ctrl::voltage_state::VoltageState EcuHealthStatusPort::ecuvoltagestate_e() const {
  // @@protoc_insertion_point(field_get:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort.ecuVoltageState_e)
  return _internal_ecuvoltagestate_e();
}
inline void EcuHealthStatusPort::_internal_set_ecuvoltagestate_e(::pb::ecu_ctrl::voltage_state::VoltageState value) {
  assert(::pb::ecu_ctrl::voltage_state::VoltageState_IsValid(value));
  _has_bits_[0] |= 0x00000008u;
  ecuvoltagestate_e_ = value;
}
inline void EcuHealthStatusPort::set_ecuvoltagestate_e(::pb::ecu_ctrl::voltage_state::VoltageState value) {
  _internal_set_ecuvoltagestate_e(value);
  // @@protoc_insertion_point(field_set:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort.ecuVoltageState_e)
}

// optional .pb.ecu_ctrl.comm_state.CommState commState_e = 1491;
inline bool EcuHealthStatusPort::_internal_has_commstate_e() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool EcuHealthStatusPort::has_commstate_e() const {
  return _internal_has_commstate_e();
}
inline void EcuHealthStatusPort::clear_commstate_e() {
  commstate_e_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::pb::ecu_ctrl::comm_state::CommState EcuHealthStatusPort::_internal_commstate_e() const {
  return static_cast< ::pb::ecu_ctrl::comm_state::CommState >(commstate_e_);
}
inline ::pb::ecu_ctrl::comm_state::CommState EcuHealthStatusPort::commstate_e() const {
  // @@protoc_insertion_point(field_get:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort.commState_e)
  return _internal_commstate_e();
}
inline void EcuHealthStatusPort::_internal_set_commstate_e(::pb::ecu_ctrl::comm_state::CommState value) {
  assert(::pb::ecu_ctrl::comm_state::CommState_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  commstate_e_ = value;
}
inline void EcuHealthStatusPort::set_commstate_e(::pb::ecu_ctrl::comm_state::CommState value) {
  _internal_set_commstate_e(value);
  // @@protoc_insertion_point(field_set:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort.commState_e)
}

// optional .pb.ecu_ctrl.temp_state.TempState temperatureState_e = 2194;
inline bool EcuHealthStatusPort::_internal_has_temperaturestate_e() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool EcuHealthStatusPort::has_temperaturestate_e() const {
  return _internal_has_temperaturestate_e();
}
inline void EcuHealthStatusPort::clear_temperaturestate_e() {
  temperaturestate_e_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::ecu_ctrl::temp_state::TempState EcuHealthStatusPort::_internal_temperaturestate_e() const {
  return static_cast< ::pb::ecu_ctrl::temp_state::TempState >(temperaturestate_e_);
}
inline ::pb::ecu_ctrl::temp_state::TempState EcuHealthStatusPort::temperaturestate_e() const {
  // @@protoc_insertion_point(field_get:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort.temperatureState_e)
  return _internal_temperaturestate_e();
}
inline void EcuHealthStatusPort::_internal_set_temperaturestate_e(::pb::ecu_ctrl::temp_state::TempState value) {
  assert(::pb::ecu_ctrl::temp_state::TempState_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  temperaturestate_e_ = value;
}
inline void EcuHealthStatusPort::set_temperaturestate_e(::pb::ecu_ctrl::temp_state::TempState value) {
  _internal_set_temperaturestate_e(value);
  // @@protoc_insertion_point(field_set:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort.temperatureState_e)
}

// -------------------------------------------------------------------

// EcuHealthStatusPort_array_port

// repeated .pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort data = 1289;
inline int EcuHealthStatusPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int EcuHealthStatusPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void EcuHealthStatusPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort* EcuHealthStatusPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort >*
EcuHealthStatusPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort_array_port.data)
  return &data_;
}
inline const ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort& EcuHealthStatusPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort& EcuHealthStatusPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort* EcuHealthStatusPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort* EcuHealthStatusPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ecu_ctrl::ecu_health_status_port::EcuHealthStatusPort >&
EcuHealthStatusPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ecu_ctrl.ecu_health_status_port.EcuHealthStatusPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace ecu_health_status_port
}  // namespace ecu_ctrl
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ecu_5fctrl_2fecu_5fhealth_5fstatus_5fport_2eproto
