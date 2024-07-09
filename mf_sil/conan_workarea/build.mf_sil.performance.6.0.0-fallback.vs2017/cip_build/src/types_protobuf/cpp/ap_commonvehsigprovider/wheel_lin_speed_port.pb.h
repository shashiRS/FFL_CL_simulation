// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_commonvehsigprovider/wheel_lin_speed_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fcommonvehsigprovider_2fwheel_5flin_5fspeed_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fcommonvehsigprovider_2fwheel_5flin_5fspeed_5fport_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ap_5fcommonvehsigprovider_2fwheel_5flin_5fspeed_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fcommonvehsigprovider_2fwheel_5flin_5fspeed_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fcommonvehsigprovider_2fwheel_5flin_5fspeed_5fport_2eproto;
namespace pb {
namespace ap_commonvehsigprovider {
namespace wheel_lin_speed_port {
class WheelLinSpeedPort;
class WheelLinSpeedPortDefaultTypeInternal;
extern WheelLinSpeedPortDefaultTypeInternal _WheelLinSpeedPort_default_instance_;
class WheelLinSpeedPort_array_port;
class WheelLinSpeedPort_array_portDefaultTypeInternal;
extern WheelLinSpeedPort_array_portDefaultTypeInternal _WheelLinSpeedPort_array_port_default_instance_;
}  // namespace wheel_lin_speed_port
}  // namespace ap_commonvehsigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort* Arena::CreateMaybeMessage<::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort>(Arena*);
template<> ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort_array_port* Arena::CreateMaybeMessage<::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_commonvehsigprovider {
namespace wheel_lin_speed_port {

// ===================================================================

class WheelLinSpeedPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort) */ {
 public:
  WheelLinSpeedPort();
  virtual ~WheelLinSpeedPort();

  WheelLinSpeedPort(const WheelLinSpeedPort& from);
  WheelLinSpeedPort(WheelLinSpeedPort&& from) noexcept
    : WheelLinSpeedPort() {
    *this = ::std::move(from);
  }

  inline WheelLinSpeedPort& operator=(const WheelLinSpeedPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline WheelLinSpeedPort& operator=(WheelLinSpeedPort&& from) noexcept {
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
  static const WheelLinSpeedPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const WheelLinSpeedPort* internal_default_instance() {
    return reinterpret_cast<const WheelLinSpeedPort*>(
               &_WheelLinSpeedPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(WheelLinSpeedPort& a, WheelLinSpeedPort& b) {
    a.Swap(&b);
  }
  inline void Swap(WheelLinSpeedPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline WheelLinSpeedPort* New() const final {
    return CreateMaybeMessage<WheelLinSpeedPort>(nullptr);
  }

  WheelLinSpeedPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<WheelLinSpeedPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const WheelLinSpeedPort& from);
  void MergeFrom(const WheelLinSpeedPort& from);
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
  void InternalSwap(WheelLinSpeedPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fcommonvehsigprovider_2fwheel_5flin_5fspeed_5fport_2eproto);
    return ::descriptor_table_ap_5fcommonvehsigprovider_2fwheel_5flin_5fspeed_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kWheelLinSpeedFRMpsFieldNumber = 3594,
    kWheelLinSpeedRRMpsFieldNumber = 559,
    kWheelLinSpeedFLMpsFieldNumber = 1528,
    kUiVersionNumberFieldNumber = 2124,
    kWheelLinSpeedRLMpsFieldNumber = 2525,
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

  // optional float wheelLinSpeedFR_mps = 3594;
  bool has_wheellinspeedfr_mps() const;
  private:
  bool _internal_has_wheellinspeedfr_mps() const;
  public:
  void clear_wheellinspeedfr_mps();
  float wheellinspeedfr_mps() const;
  void set_wheellinspeedfr_mps(float value);
  private:
  float _internal_wheellinspeedfr_mps() const;
  void _internal_set_wheellinspeedfr_mps(float value);
  public:

  // optional float wheelLinSpeedRR_mps = 559;
  bool has_wheellinspeedrr_mps() const;
  private:
  bool _internal_has_wheellinspeedrr_mps() const;
  public:
  void clear_wheellinspeedrr_mps();
  float wheellinspeedrr_mps() const;
  void set_wheellinspeedrr_mps(float value);
  private:
  float _internal_wheellinspeedrr_mps() const;
  void _internal_set_wheellinspeedrr_mps(float value);
  public:

  // optional float wheelLinSpeedFL_mps = 1528;
  bool has_wheellinspeedfl_mps() const;
  private:
  bool _internal_has_wheellinspeedfl_mps() const;
  public:
  void clear_wheellinspeedfl_mps();
  float wheellinspeedfl_mps() const;
  void set_wheellinspeedfl_mps(float value);
  private:
  float _internal_wheellinspeedfl_mps() const;
  void _internal_set_wheellinspeedfl_mps(float value);
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

  // optional float wheelLinSpeedRL_mps = 2525;
  bool has_wheellinspeedrl_mps() const;
  private:
  bool _internal_has_wheellinspeedrl_mps() const;
  public:
  void clear_wheellinspeedrl_mps();
  float wheellinspeedrl_mps() const;
  void set_wheellinspeedrl_mps(float value);
  private:
  float _internal_wheellinspeedrl_mps() const;
  void _internal_set_wheellinspeedrl_mps(float value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  float wheellinspeedfr_mps_;
  float wheellinspeedrr_mps_;
  float wheellinspeedfl_mps_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  float wheellinspeedrl_mps_;
  friend struct ::TableStruct_ap_5fcommonvehsigprovider_2fwheel_5flin_5fspeed_5fport_2eproto;
};
// -------------------------------------------------------------------

class WheelLinSpeedPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort_array_port) */ {
 public:
  WheelLinSpeedPort_array_port();
  virtual ~WheelLinSpeedPort_array_port();

  WheelLinSpeedPort_array_port(const WheelLinSpeedPort_array_port& from);
  WheelLinSpeedPort_array_port(WheelLinSpeedPort_array_port&& from) noexcept
    : WheelLinSpeedPort_array_port() {
    *this = ::std::move(from);
  }

  inline WheelLinSpeedPort_array_port& operator=(const WheelLinSpeedPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline WheelLinSpeedPort_array_port& operator=(WheelLinSpeedPort_array_port&& from) noexcept {
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
  static const WheelLinSpeedPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const WheelLinSpeedPort_array_port* internal_default_instance() {
    return reinterpret_cast<const WheelLinSpeedPort_array_port*>(
               &_WheelLinSpeedPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(WheelLinSpeedPort_array_port& a, WheelLinSpeedPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(WheelLinSpeedPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline WheelLinSpeedPort_array_port* New() const final {
    return CreateMaybeMessage<WheelLinSpeedPort_array_port>(nullptr);
  }

  WheelLinSpeedPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<WheelLinSpeedPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const WheelLinSpeedPort_array_port& from);
  void MergeFrom(const WheelLinSpeedPort_array_port& from);
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
  void InternalSwap(WheelLinSpeedPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fcommonvehsigprovider_2fwheel_5flin_5fspeed_5fport_2eproto);
    return ::descriptor_table_ap_5fcommonvehsigprovider_2fwheel_5flin_5fspeed_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1542,
  };
  // repeated .pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort data = 1542;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort >*
      mutable_data();
  private:
  const ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort& _internal_data(int index) const;
  ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort* _internal_add_data();
  public:
  const ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort& data(int index) const;
  ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort > data_;
  friend struct ::TableStruct_ap_5fcommonvehsigprovider_2fwheel_5flin_5fspeed_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// WheelLinSpeedPort

// optional uint32 uiVersionNumber = 2124;
inline bool WheelLinSpeedPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool WheelLinSpeedPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void WheelLinSpeedPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 WheelLinSpeedPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 WheelLinSpeedPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void WheelLinSpeedPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  uiversionnumber_ = value;
}
inline void WheelLinSpeedPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool WheelLinSpeedPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool WheelLinSpeedPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& WheelLinSpeedPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& WheelLinSpeedPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* WheelLinSpeedPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* WheelLinSpeedPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* WheelLinSpeedPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void WheelLinSpeedPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.sSigHeader)
}

// optional float wheelLinSpeedFL_mps = 1528;
inline bool WheelLinSpeedPort::_internal_has_wheellinspeedfl_mps() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool WheelLinSpeedPort::has_wheellinspeedfl_mps() const {
  return _internal_has_wheellinspeedfl_mps();
}
inline void WheelLinSpeedPort::clear_wheellinspeedfl_mps() {
  wheellinspeedfl_mps_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float WheelLinSpeedPort::_internal_wheellinspeedfl_mps() const {
  return wheellinspeedfl_mps_;
}
inline float WheelLinSpeedPort::wheellinspeedfl_mps() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.wheelLinSpeedFL_mps)
  return _internal_wheellinspeedfl_mps();
}
inline void WheelLinSpeedPort::_internal_set_wheellinspeedfl_mps(float value) {
  _has_bits_[0] |= 0x00000008u;
  wheellinspeedfl_mps_ = value;
}
inline void WheelLinSpeedPort::set_wheellinspeedfl_mps(float value) {
  _internal_set_wheellinspeedfl_mps(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.wheelLinSpeedFL_mps)
}

// optional float wheelLinSpeedFR_mps = 3594;
inline bool WheelLinSpeedPort::_internal_has_wheellinspeedfr_mps() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool WheelLinSpeedPort::has_wheellinspeedfr_mps() const {
  return _internal_has_wheellinspeedfr_mps();
}
inline void WheelLinSpeedPort::clear_wheellinspeedfr_mps() {
  wheellinspeedfr_mps_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline float WheelLinSpeedPort::_internal_wheellinspeedfr_mps() const {
  return wheellinspeedfr_mps_;
}
inline float WheelLinSpeedPort::wheellinspeedfr_mps() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.wheelLinSpeedFR_mps)
  return _internal_wheellinspeedfr_mps();
}
inline void WheelLinSpeedPort::_internal_set_wheellinspeedfr_mps(float value) {
  _has_bits_[0] |= 0x00000002u;
  wheellinspeedfr_mps_ = value;
}
inline void WheelLinSpeedPort::set_wheellinspeedfr_mps(float value) {
  _internal_set_wheellinspeedfr_mps(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.wheelLinSpeedFR_mps)
}

// optional float wheelLinSpeedRL_mps = 2525;
inline bool WheelLinSpeedPort::_internal_has_wheellinspeedrl_mps() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool WheelLinSpeedPort::has_wheellinspeedrl_mps() const {
  return _internal_has_wheellinspeedrl_mps();
}
inline void WheelLinSpeedPort::clear_wheellinspeedrl_mps() {
  wheellinspeedrl_mps_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline float WheelLinSpeedPort::_internal_wheellinspeedrl_mps() const {
  return wheellinspeedrl_mps_;
}
inline float WheelLinSpeedPort::wheellinspeedrl_mps() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.wheelLinSpeedRL_mps)
  return _internal_wheellinspeedrl_mps();
}
inline void WheelLinSpeedPort::_internal_set_wheellinspeedrl_mps(float value) {
  _has_bits_[0] |= 0x00000020u;
  wheellinspeedrl_mps_ = value;
}
inline void WheelLinSpeedPort::set_wheellinspeedrl_mps(float value) {
  _internal_set_wheellinspeedrl_mps(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.wheelLinSpeedRL_mps)
}

// optional float wheelLinSpeedRR_mps = 559;
inline bool WheelLinSpeedPort::_internal_has_wheellinspeedrr_mps() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool WheelLinSpeedPort::has_wheellinspeedrr_mps() const {
  return _internal_has_wheellinspeedrr_mps();
}
inline void WheelLinSpeedPort::clear_wheellinspeedrr_mps() {
  wheellinspeedrr_mps_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float WheelLinSpeedPort::_internal_wheellinspeedrr_mps() const {
  return wheellinspeedrr_mps_;
}
inline float WheelLinSpeedPort::wheellinspeedrr_mps() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.wheelLinSpeedRR_mps)
  return _internal_wheellinspeedrr_mps();
}
inline void WheelLinSpeedPort::_internal_set_wheellinspeedrr_mps(float value) {
  _has_bits_[0] |= 0x00000004u;
  wheellinspeedrr_mps_ = value;
}
inline void WheelLinSpeedPort::set_wheellinspeedrr_mps(float value) {
  _internal_set_wheellinspeedrr_mps(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort.wheelLinSpeedRR_mps)
}

// -------------------------------------------------------------------

// WheelLinSpeedPort_array_port

// repeated .pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort data = 1542;
inline int WheelLinSpeedPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int WheelLinSpeedPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void WheelLinSpeedPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort* WheelLinSpeedPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort >*
WheelLinSpeedPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort_array_port.data)
  return &data_;
}
inline const ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort& WheelLinSpeedPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort& WheelLinSpeedPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort* WheelLinSpeedPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort* WheelLinSpeedPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::wheel_lin_speed_port::WheelLinSpeedPort >&
WheelLinSpeedPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_commonvehsigprovider.wheel_lin_speed_port.WheelLinSpeedPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace wheel_lin_speed_port
}  // namespace ap_commonvehsigprovider
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fcommonvehsigprovider_2fwheel_5flin_5fspeed_5fport_2eproto
