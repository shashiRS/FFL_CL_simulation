// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_commonvehsigprovider/steer_ctrl_status_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fcommonvehsigprovider_2fsteer_5fctrl_5fstatus_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fcommonvehsigprovider_2fsteer_5fctrl_5fstatus_5fport_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ap_5fcommonvehsigprovider_2fsteer_5fctrl_5fstatus_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fcommonvehsigprovider_2fsteer_5fctrl_5fstatus_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fcommonvehsigprovider_2fsteer_5fctrl_5fstatus_5fport_2eproto;
namespace pb {
namespace ap_commonvehsigprovider {
namespace steer_ctrl_status_port {
class SteerCtrlStatusPort;
class SteerCtrlStatusPortDefaultTypeInternal;
extern SteerCtrlStatusPortDefaultTypeInternal _SteerCtrlStatusPort_default_instance_;
class SteerCtrlStatusPort_array_port;
class SteerCtrlStatusPort_array_portDefaultTypeInternal;
extern SteerCtrlStatusPort_array_portDefaultTypeInternal _SteerCtrlStatusPort_array_port_default_instance_;
}  // namespace steer_ctrl_status_port
}  // namespace ap_commonvehsigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort* Arena::CreateMaybeMessage<::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort>(Arena*);
template<> ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort_array_port* Arena::CreateMaybeMessage<::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_commonvehsigprovider {
namespace steer_ctrl_status_port {

// ===================================================================

class SteerCtrlStatusPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort) */ {
 public:
  SteerCtrlStatusPort();
  virtual ~SteerCtrlStatusPort();

  SteerCtrlStatusPort(const SteerCtrlStatusPort& from);
  SteerCtrlStatusPort(SteerCtrlStatusPort&& from) noexcept
    : SteerCtrlStatusPort() {
    *this = ::std::move(from);
  }

  inline SteerCtrlStatusPort& operator=(const SteerCtrlStatusPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline SteerCtrlStatusPort& operator=(SteerCtrlStatusPort&& from) noexcept {
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
  static const SteerCtrlStatusPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SteerCtrlStatusPort* internal_default_instance() {
    return reinterpret_cast<const SteerCtrlStatusPort*>(
               &_SteerCtrlStatusPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(SteerCtrlStatusPort& a, SteerCtrlStatusPort& b) {
    a.Swap(&b);
  }
  inline void Swap(SteerCtrlStatusPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SteerCtrlStatusPort* New() const final {
    return CreateMaybeMessage<SteerCtrlStatusPort>(nullptr);
  }

  SteerCtrlStatusPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SteerCtrlStatusPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SteerCtrlStatusPort& from);
  void MergeFrom(const SteerCtrlStatusPort& from);
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
  void InternalSwap(SteerCtrlStatusPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fcommonvehsigprovider_2fsteer_5fctrl_5fstatus_5fport_2eproto);
    return ::descriptor_table_ap_5fcommonvehsigprovider_2fsteer_5fctrl_5fstatus_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kSteeringWheelAngleRadFieldNumber = 23,
    kSteeringWheelAngleOffsetRadFieldNumber = 35,
    kSteeringWheelAngleVelocityRadpsFieldNumber = 411,
    kEpsAppliedTieRodForceNmFieldNumber = 702,
    kUiVersionNumberFieldNumber = 2124,
    kSteeringWheelAngleQFNuFieldNumber = 2733,
    kSteeringWheelAngleVelocityQFNuFieldNumber = 3830,
    kCalculatedSteeringWheelAngleQFNuFieldNumber = 2672,
    kEpsAppliedTieRodForceQFNmFieldNumber = 3584,
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

  // optional float steeringWheelAngle_rad = 23;
  bool has_steeringwheelangle_rad() const;
  private:
  bool _internal_has_steeringwheelangle_rad() const;
  public:
  void clear_steeringwheelangle_rad();
  float steeringwheelangle_rad() const;
  void set_steeringwheelangle_rad(float value);
  private:
  float _internal_steeringwheelangle_rad() const;
  void _internal_set_steeringwheelangle_rad(float value);
  public:

  // optional float steeringWheelAngleOffset_rad = 35;
  bool has_steeringwheelangleoffset_rad() const;
  private:
  bool _internal_has_steeringwheelangleoffset_rad() const;
  public:
  void clear_steeringwheelangleoffset_rad();
  float steeringwheelangleoffset_rad() const;
  void set_steeringwheelangleoffset_rad(float value);
  private:
  float _internal_steeringwheelangleoffset_rad() const;
  void _internal_set_steeringwheelangleoffset_rad(float value);
  public:

  // optional float steeringWheelAngleVelocity_radps = 411;
  bool has_steeringwheelanglevelocity_radps() const;
  private:
  bool _internal_has_steeringwheelanglevelocity_radps() const;
  public:
  void clear_steeringwheelanglevelocity_radps();
  float steeringwheelanglevelocity_radps() const;
  void set_steeringwheelanglevelocity_radps(float value);
  private:
  float _internal_steeringwheelanglevelocity_radps() const;
  void _internal_set_steeringwheelanglevelocity_radps(float value);
  public:

  // optional float epsAppliedTieRodForce_Nm = 702;
  bool has_epsappliedtierodforce_nm() const;
  private:
  bool _internal_has_epsappliedtierodforce_nm() const;
  public:
  void clear_epsappliedtierodforce_nm();
  float epsappliedtierodforce_nm() const;
  void set_epsappliedtierodforce_nm(float value);
  private:
  float _internal_epsappliedtierodforce_nm() const;
  void _internal_set_epsappliedtierodforce_nm(float value);
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

  // optional bool steeringWheelAngle_QF_nu = 2733;
  bool has_steeringwheelangle_qf_nu() const;
  private:
  bool _internal_has_steeringwheelangle_qf_nu() const;
  public:
  void clear_steeringwheelangle_qf_nu();
  bool steeringwheelangle_qf_nu() const;
  void set_steeringwheelangle_qf_nu(bool value);
  private:
  bool _internal_steeringwheelangle_qf_nu() const;
  void _internal_set_steeringwheelangle_qf_nu(bool value);
  public:

  // optional bool steeringWheelAngleVelocity_QF_nu = 3830;
  bool has_steeringwheelanglevelocity_qf_nu() const;
  private:
  bool _internal_has_steeringwheelanglevelocity_qf_nu() const;
  public:
  void clear_steeringwheelanglevelocity_qf_nu();
  bool steeringwheelanglevelocity_qf_nu() const;
  void set_steeringwheelanglevelocity_qf_nu(bool value);
  private:
  bool _internal_steeringwheelanglevelocity_qf_nu() const;
  void _internal_set_steeringwheelanglevelocity_qf_nu(bool value);
  public:

  // optional bool calculatedSteeringWheelAngle_QF_nu = 2672;
  bool has_calculatedsteeringwheelangle_qf_nu() const;
  private:
  bool _internal_has_calculatedsteeringwheelangle_qf_nu() const;
  public:
  void clear_calculatedsteeringwheelangle_qf_nu();
  bool calculatedsteeringwheelangle_qf_nu() const;
  void set_calculatedsteeringwheelangle_qf_nu(bool value);
  private:
  bool _internal_calculatedsteeringwheelangle_qf_nu() const;
  void _internal_set_calculatedsteeringwheelangle_qf_nu(bool value);
  public:

  // optional bool epsAppliedTieRodForce_QF_Nm = 3584;
  bool has_epsappliedtierodforce_qf_nm() const;
  private:
  bool _internal_has_epsappliedtierodforce_qf_nm() const;
  public:
  void clear_epsappliedtierodforce_qf_nm();
  bool epsappliedtierodforce_qf_nm() const;
  void set_epsappliedtierodforce_qf_nm(bool value);
  private:
  bool _internal_epsappliedtierodforce_qf_nm() const;
  void _internal_set_epsappliedtierodforce_qf_nm(bool value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  float steeringwheelangle_rad_;
  float steeringwheelangleoffset_rad_;
  float steeringwheelanglevelocity_radps_;
  float epsappliedtierodforce_nm_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  bool steeringwheelangle_qf_nu_;
  bool steeringwheelanglevelocity_qf_nu_;
  bool calculatedsteeringwheelangle_qf_nu_;
  bool epsappliedtierodforce_qf_nm_;
  friend struct ::TableStruct_ap_5fcommonvehsigprovider_2fsteer_5fctrl_5fstatus_5fport_2eproto;
};
// -------------------------------------------------------------------

class SteerCtrlStatusPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort_array_port) */ {
 public:
  SteerCtrlStatusPort_array_port();
  virtual ~SteerCtrlStatusPort_array_port();

  SteerCtrlStatusPort_array_port(const SteerCtrlStatusPort_array_port& from);
  SteerCtrlStatusPort_array_port(SteerCtrlStatusPort_array_port&& from) noexcept
    : SteerCtrlStatusPort_array_port() {
    *this = ::std::move(from);
  }

  inline SteerCtrlStatusPort_array_port& operator=(const SteerCtrlStatusPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline SteerCtrlStatusPort_array_port& operator=(SteerCtrlStatusPort_array_port&& from) noexcept {
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
  static const SteerCtrlStatusPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SteerCtrlStatusPort_array_port* internal_default_instance() {
    return reinterpret_cast<const SteerCtrlStatusPort_array_port*>(
               &_SteerCtrlStatusPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(SteerCtrlStatusPort_array_port& a, SteerCtrlStatusPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(SteerCtrlStatusPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SteerCtrlStatusPort_array_port* New() const final {
    return CreateMaybeMessage<SteerCtrlStatusPort_array_port>(nullptr);
  }

  SteerCtrlStatusPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SteerCtrlStatusPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SteerCtrlStatusPort_array_port& from);
  void MergeFrom(const SteerCtrlStatusPort_array_port& from);
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
  void InternalSwap(SteerCtrlStatusPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fcommonvehsigprovider_2fsteer_5fctrl_5fstatus_5fport_2eproto);
    return ::descriptor_table_ap_5fcommonvehsigprovider_2fsteer_5fctrl_5fstatus_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2534,
  };
  // repeated .pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort data = 2534;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort >*
      mutable_data();
  private:
  const ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort& _internal_data(int index) const;
  ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort* _internal_add_data();
  public:
  const ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort& data(int index) const;
  ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort > data_;
  friend struct ::TableStruct_ap_5fcommonvehsigprovider_2fsteer_5fctrl_5fstatus_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SteerCtrlStatusPort

// optional uint32 uiVersionNumber = 2124;
inline bool SteerCtrlStatusPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool SteerCtrlStatusPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void SteerCtrlStatusPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 SteerCtrlStatusPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 SteerCtrlStatusPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void SteerCtrlStatusPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  uiversionnumber_ = value;
}
inline void SteerCtrlStatusPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool SteerCtrlStatusPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool SteerCtrlStatusPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& SteerCtrlStatusPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& SteerCtrlStatusPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* SteerCtrlStatusPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* SteerCtrlStatusPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* SteerCtrlStatusPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void SteerCtrlStatusPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.sSigHeader)
}

// optional float steeringWheelAngle_rad = 23;
inline bool SteerCtrlStatusPort::_internal_has_steeringwheelangle_rad() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool SteerCtrlStatusPort::has_steeringwheelangle_rad() const {
  return _internal_has_steeringwheelangle_rad();
}
inline void SteerCtrlStatusPort::clear_steeringwheelangle_rad() {
  steeringwheelangle_rad_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline float SteerCtrlStatusPort::_internal_steeringwheelangle_rad() const {
  return steeringwheelangle_rad_;
}
inline float SteerCtrlStatusPort::steeringwheelangle_rad() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.steeringWheelAngle_rad)
  return _internal_steeringwheelangle_rad();
}
inline void SteerCtrlStatusPort::_internal_set_steeringwheelangle_rad(float value) {
  _has_bits_[0] |= 0x00000002u;
  steeringwheelangle_rad_ = value;
}
inline void SteerCtrlStatusPort::set_steeringwheelangle_rad(float value) {
  _internal_set_steeringwheelangle_rad(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.steeringWheelAngle_rad)
}

// optional float steeringWheelAngleOffset_rad = 35;
inline bool SteerCtrlStatusPort::_internal_has_steeringwheelangleoffset_rad() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool SteerCtrlStatusPort::has_steeringwheelangleoffset_rad() const {
  return _internal_has_steeringwheelangleoffset_rad();
}
inline void SteerCtrlStatusPort::clear_steeringwheelangleoffset_rad() {
  steeringwheelangleoffset_rad_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float SteerCtrlStatusPort::_internal_steeringwheelangleoffset_rad() const {
  return steeringwheelangleoffset_rad_;
}
inline float SteerCtrlStatusPort::steeringwheelangleoffset_rad() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.steeringWheelAngleOffset_rad)
  return _internal_steeringwheelangleoffset_rad();
}
inline void SteerCtrlStatusPort::_internal_set_steeringwheelangleoffset_rad(float value) {
  _has_bits_[0] |= 0x00000004u;
  steeringwheelangleoffset_rad_ = value;
}
inline void SteerCtrlStatusPort::set_steeringwheelangleoffset_rad(float value) {
  _internal_set_steeringwheelangleoffset_rad(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.steeringWheelAngleOffset_rad)
}

// optional float steeringWheelAngleVelocity_radps = 411;
inline bool SteerCtrlStatusPort::_internal_has_steeringwheelanglevelocity_radps() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool SteerCtrlStatusPort::has_steeringwheelanglevelocity_radps() const {
  return _internal_has_steeringwheelanglevelocity_radps();
}
inline void SteerCtrlStatusPort::clear_steeringwheelanglevelocity_radps() {
  steeringwheelanglevelocity_radps_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float SteerCtrlStatusPort::_internal_steeringwheelanglevelocity_radps() const {
  return steeringwheelanglevelocity_radps_;
}
inline float SteerCtrlStatusPort::steeringwheelanglevelocity_radps() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.steeringWheelAngleVelocity_radps)
  return _internal_steeringwheelanglevelocity_radps();
}
inline void SteerCtrlStatusPort::_internal_set_steeringwheelanglevelocity_radps(float value) {
  _has_bits_[0] |= 0x00000008u;
  steeringwheelanglevelocity_radps_ = value;
}
inline void SteerCtrlStatusPort::set_steeringwheelanglevelocity_radps(float value) {
  _internal_set_steeringwheelanglevelocity_radps(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.steeringWheelAngleVelocity_radps)
}

// optional float epsAppliedTieRodForce_Nm = 702;
inline bool SteerCtrlStatusPort::_internal_has_epsappliedtierodforce_nm() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool SteerCtrlStatusPort::has_epsappliedtierodforce_nm() const {
  return _internal_has_epsappliedtierodforce_nm();
}
inline void SteerCtrlStatusPort::clear_epsappliedtierodforce_nm() {
  epsappliedtierodforce_nm_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline float SteerCtrlStatusPort::_internal_epsappliedtierodforce_nm() const {
  return epsappliedtierodforce_nm_;
}
inline float SteerCtrlStatusPort::epsappliedtierodforce_nm() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.epsAppliedTieRodForce_Nm)
  return _internal_epsappliedtierodforce_nm();
}
inline void SteerCtrlStatusPort::_internal_set_epsappliedtierodforce_nm(float value) {
  _has_bits_[0] |= 0x00000010u;
  epsappliedtierodforce_nm_ = value;
}
inline void SteerCtrlStatusPort::set_epsappliedtierodforce_nm(float value) {
  _internal_set_epsappliedtierodforce_nm(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.epsAppliedTieRodForce_Nm)
}

// optional bool steeringWheelAngle_QF_nu = 2733;
inline bool SteerCtrlStatusPort::_internal_has_steeringwheelangle_qf_nu() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool SteerCtrlStatusPort::has_steeringwheelangle_qf_nu() const {
  return _internal_has_steeringwheelangle_qf_nu();
}
inline void SteerCtrlStatusPort::clear_steeringwheelangle_qf_nu() {
  steeringwheelangle_qf_nu_ = false;
  _has_bits_[0] &= ~0x00000040u;
}
inline bool SteerCtrlStatusPort::_internal_steeringwheelangle_qf_nu() const {
  return steeringwheelangle_qf_nu_;
}
inline bool SteerCtrlStatusPort::steeringwheelangle_qf_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.steeringWheelAngle_QF_nu)
  return _internal_steeringwheelangle_qf_nu();
}
inline void SteerCtrlStatusPort::_internal_set_steeringwheelangle_qf_nu(bool value) {
  _has_bits_[0] |= 0x00000040u;
  steeringwheelangle_qf_nu_ = value;
}
inline void SteerCtrlStatusPort::set_steeringwheelangle_qf_nu(bool value) {
  _internal_set_steeringwheelangle_qf_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.steeringWheelAngle_QF_nu)
}

// optional bool steeringWheelAngleVelocity_QF_nu = 3830;
inline bool SteerCtrlStatusPort::_internal_has_steeringwheelanglevelocity_qf_nu() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool SteerCtrlStatusPort::has_steeringwheelanglevelocity_qf_nu() const {
  return _internal_has_steeringwheelanglevelocity_qf_nu();
}
inline void SteerCtrlStatusPort::clear_steeringwheelanglevelocity_qf_nu() {
  steeringwheelanglevelocity_qf_nu_ = false;
  _has_bits_[0] &= ~0x00000080u;
}
inline bool SteerCtrlStatusPort::_internal_steeringwheelanglevelocity_qf_nu() const {
  return steeringwheelanglevelocity_qf_nu_;
}
inline bool SteerCtrlStatusPort::steeringwheelanglevelocity_qf_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.steeringWheelAngleVelocity_QF_nu)
  return _internal_steeringwheelanglevelocity_qf_nu();
}
inline void SteerCtrlStatusPort::_internal_set_steeringwheelanglevelocity_qf_nu(bool value) {
  _has_bits_[0] |= 0x00000080u;
  steeringwheelanglevelocity_qf_nu_ = value;
}
inline void SteerCtrlStatusPort::set_steeringwheelanglevelocity_qf_nu(bool value) {
  _internal_set_steeringwheelanglevelocity_qf_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.steeringWheelAngleVelocity_QF_nu)
}

// optional bool calculatedSteeringWheelAngle_QF_nu = 2672;
inline bool SteerCtrlStatusPort::_internal_has_calculatedsteeringwheelangle_qf_nu() const {
  bool value = (_has_bits_[0] & 0x00000100u) != 0;
  return value;
}
inline bool SteerCtrlStatusPort::has_calculatedsteeringwheelangle_qf_nu() const {
  return _internal_has_calculatedsteeringwheelangle_qf_nu();
}
inline void SteerCtrlStatusPort::clear_calculatedsteeringwheelangle_qf_nu() {
  calculatedsteeringwheelangle_qf_nu_ = false;
  _has_bits_[0] &= ~0x00000100u;
}
inline bool SteerCtrlStatusPort::_internal_calculatedsteeringwheelangle_qf_nu() const {
  return calculatedsteeringwheelangle_qf_nu_;
}
inline bool SteerCtrlStatusPort::calculatedsteeringwheelangle_qf_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.calculatedSteeringWheelAngle_QF_nu)
  return _internal_calculatedsteeringwheelangle_qf_nu();
}
inline void SteerCtrlStatusPort::_internal_set_calculatedsteeringwheelangle_qf_nu(bool value) {
  _has_bits_[0] |= 0x00000100u;
  calculatedsteeringwheelangle_qf_nu_ = value;
}
inline void SteerCtrlStatusPort::set_calculatedsteeringwheelangle_qf_nu(bool value) {
  _internal_set_calculatedsteeringwheelangle_qf_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.calculatedSteeringWheelAngle_QF_nu)
}

// optional bool epsAppliedTieRodForce_QF_Nm = 3584;
inline bool SteerCtrlStatusPort::_internal_has_epsappliedtierodforce_qf_nm() const {
  bool value = (_has_bits_[0] & 0x00000200u) != 0;
  return value;
}
inline bool SteerCtrlStatusPort::has_epsappliedtierodforce_qf_nm() const {
  return _internal_has_epsappliedtierodforce_qf_nm();
}
inline void SteerCtrlStatusPort::clear_epsappliedtierodforce_qf_nm() {
  epsappliedtierodforce_qf_nm_ = false;
  _has_bits_[0] &= ~0x00000200u;
}
inline bool SteerCtrlStatusPort::_internal_epsappliedtierodforce_qf_nm() const {
  return epsappliedtierodforce_qf_nm_;
}
inline bool SteerCtrlStatusPort::epsappliedtierodforce_qf_nm() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.epsAppliedTieRodForce_QF_Nm)
  return _internal_epsappliedtierodforce_qf_nm();
}
inline void SteerCtrlStatusPort::_internal_set_epsappliedtierodforce_qf_nm(bool value) {
  _has_bits_[0] |= 0x00000200u;
  epsappliedtierodforce_qf_nm_ = value;
}
inline void SteerCtrlStatusPort::set_epsappliedtierodforce_qf_nm(bool value) {
  _internal_set_epsappliedtierodforce_qf_nm(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort.epsAppliedTieRodForce_QF_Nm)
}

// -------------------------------------------------------------------

// SteerCtrlStatusPort_array_port

// repeated .pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort data = 2534;
inline int SteerCtrlStatusPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int SteerCtrlStatusPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void SteerCtrlStatusPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort* SteerCtrlStatusPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort >*
SteerCtrlStatusPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort_array_port.data)
  return &data_;
}
inline const ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort& SteerCtrlStatusPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort& SteerCtrlStatusPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort* SteerCtrlStatusPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort* SteerCtrlStatusPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::steer_ctrl_status_port::SteerCtrlStatusPort >&
SteerCtrlStatusPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_commonvehsigprovider.steer_ctrl_status_port.SteerCtrlStatusPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace steer_ctrl_status_port
}  // namespace ap_commonvehsigprovider
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fcommonvehsigprovider_2fsteer_5fctrl_5fstatus_5fport_2eproto
