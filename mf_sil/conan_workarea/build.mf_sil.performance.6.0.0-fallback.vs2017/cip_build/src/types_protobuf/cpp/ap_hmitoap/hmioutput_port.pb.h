// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_hmitoap/hmioutput_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fhmitoap_2fhmioutput_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fhmitoap_2fhmioutput_5fport_2eproto

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
#include "ap_hmitoap/user_action_head_unit.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5fhmitoap_2fhmioutput_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fhmitoap_2fhmioutput_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fhmitoap_2fhmioutput_5fport_2eproto;
namespace pb {
namespace ap_hmitoap {
namespace hmioutput_port {
class HMIOutputPort;
class HMIOutputPortDefaultTypeInternal;
extern HMIOutputPortDefaultTypeInternal _HMIOutputPort_default_instance_;
class HMIOutputPort_array_port;
class HMIOutputPort_array_portDefaultTypeInternal;
extern HMIOutputPort_array_portDefaultTypeInternal _HMIOutputPort_array_port_default_instance_;
}  // namespace hmioutput_port
}  // namespace ap_hmitoap
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort* Arena::CreateMaybeMessage<::pb::ap_hmitoap::hmioutput_port::HMIOutputPort>(Arena*);
template<> ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort_array_port* Arena::CreateMaybeMessage<::pb::ap_hmitoap::hmioutput_port::HMIOutputPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_hmitoap {
namespace hmioutput_port {

// ===================================================================

class HMIOutputPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_hmitoap.hmioutput_port.HMIOutputPort) */ {
 public:
  HMIOutputPort();
  virtual ~HMIOutputPort();

  HMIOutputPort(const HMIOutputPort& from);
  HMIOutputPort(HMIOutputPort&& from) noexcept
    : HMIOutputPort() {
    *this = ::std::move(from);
  }

  inline HMIOutputPort& operator=(const HMIOutputPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline HMIOutputPort& operator=(HMIOutputPort&& from) noexcept {
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
  static const HMIOutputPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const HMIOutputPort* internal_default_instance() {
    return reinterpret_cast<const HMIOutputPort*>(
               &_HMIOutputPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(HMIOutputPort& a, HMIOutputPort& b) {
    a.Swap(&b);
  }
  inline void Swap(HMIOutputPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline HMIOutputPort* New() const final {
    return CreateMaybeMessage<HMIOutputPort>(nullptr);
  }

  HMIOutputPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<HMIOutputPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const HMIOutputPort& from);
  void MergeFrom(const HMIOutputPort& from);
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
  void InternalSwap(HMIOutputPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_hmitoap.hmioutput_port.HMIOutputPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fhmitoap_2fhmioutput_5fport_2eproto);
    return ::descriptor_table_ap_5fhmitoap_2fhmioutput_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kUserActionHeadUnitNuFieldNumber = 1844,
    kUiVersionNumberFieldNumber = 2124,
    kPdwAutoActivateNuFieldNumber = 2682,
    kLscaAutoActivateNuFieldNumber = 2695,
    kUserActionHUCounterNuFieldNumber = 3627,
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

  // optional .pb.ap_hmitoap.user_action_head_unit.UserActionHeadUnit userActionHeadUnit_nu = 1844;
  bool has_useractionheadunit_nu() const;
  private:
  bool _internal_has_useractionheadunit_nu() const;
  public:
  void clear_useractionheadunit_nu();
  ::pb::ap_hmitoap::user_action_head_unit::UserActionHeadUnit useractionheadunit_nu() const;
  void set_useractionheadunit_nu(::pb::ap_hmitoap::user_action_head_unit::UserActionHeadUnit value);
  private:
  ::pb::ap_hmitoap::user_action_head_unit::UserActionHeadUnit _internal_useractionheadunit_nu() const;
  void _internal_set_useractionheadunit_nu(::pb::ap_hmitoap::user_action_head_unit::UserActionHeadUnit value);
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

  // optional bool pdwAutoActivate_nu = 2682;
  bool has_pdwautoactivate_nu() const;
  private:
  bool _internal_has_pdwautoactivate_nu() const;
  public:
  void clear_pdwautoactivate_nu();
  bool pdwautoactivate_nu() const;
  void set_pdwautoactivate_nu(bool value);
  private:
  bool _internal_pdwautoactivate_nu() const;
  void _internal_set_pdwautoactivate_nu(bool value);
  public:

  // optional bool lscaAutoActivate_nu = 2695;
  bool has_lscaautoactivate_nu() const;
  private:
  bool _internal_has_lscaautoactivate_nu() const;
  public:
  void clear_lscaautoactivate_nu();
  bool lscaautoactivate_nu() const;
  void set_lscaautoactivate_nu(bool value);
  private:
  bool _internal_lscaautoactivate_nu() const;
  void _internal_set_lscaautoactivate_nu(bool value);
  public:

  // optional uint32 userActionHUCounter_nu = 3627;
  bool has_useractionhucounter_nu() const;
  private:
  bool _internal_has_useractionhucounter_nu() const;
  public:
  void clear_useractionhucounter_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 useractionhucounter_nu() const;
  void set_useractionhucounter_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_useractionhucounter_nu() const;
  void _internal_set_useractionhucounter_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_hmitoap.hmioutput_port.HMIOutputPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  int useractionheadunit_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  bool pdwautoactivate_nu_;
  bool lscaautoactivate_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 useractionhucounter_nu_;
  friend struct ::TableStruct_ap_5fhmitoap_2fhmioutput_5fport_2eproto;
};
// -------------------------------------------------------------------

class HMIOutputPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_hmitoap.hmioutput_port.HMIOutputPort_array_port) */ {
 public:
  HMIOutputPort_array_port();
  virtual ~HMIOutputPort_array_port();

  HMIOutputPort_array_port(const HMIOutputPort_array_port& from);
  HMIOutputPort_array_port(HMIOutputPort_array_port&& from) noexcept
    : HMIOutputPort_array_port() {
    *this = ::std::move(from);
  }

  inline HMIOutputPort_array_port& operator=(const HMIOutputPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline HMIOutputPort_array_port& operator=(HMIOutputPort_array_port&& from) noexcept {
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
  static const HMIOutputPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const HMIOutputPort_array_port* internal_default_instance() {
    return reinterpret_cast<const HMIOutputPort_array_port*>(
               &_HMIOutputPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(HMIOutputPort_array_port& a, HMIOutputPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(HMIOutputPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline HMIOutputPort_array_port* New() const final {
    return CreateMaybeMessage<HMIOutputPort_array_port>(nullptr);
  }

  HMIOutputPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<HMIOutputPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const HMIOutputPort_array_port& from);
  void MergeFrom(const HMIOutputPort_array_port& from);
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
  void InternalSwap(HMIOutputPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_hmitoap.hmioutput_port.HMIOutputPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fhmitoap_2fhmioutput_5fport_2eproto);
    return ::descriptor_table_ap_5fhmitoap_2fhmioutput_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1683,
  };
  // repeated .pb.ap_hmitoap.hmioutput_port.HMIOutputPort data = 1683;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort >*
      mutable_data();
  private:
  const ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort& _internal_data(int index) const;
  ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort* _internal_add_data();
  public:
  const ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort& data(int index) const;
  ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_hmitoap.hmioutput_port.HMIOutputPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort > data_;
  friend struct ::TableStruct_ap_5fhmitoap_2fhmioutput_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// HMIOutputPort

// optional uint32 uiVersionNumber = 2124;
inline bool HMIOutputPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool HMIOutputPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void HMIOutputPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 HMIOutputPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 HMIOutputPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void HMIOutputPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  uiversionnumber_ = value;
}
inline void HMIOutputPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool HMIOutputPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool HMIOutputPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& HMIOutputPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& HMIOutputPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* HMIOutputPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* HMIOutputPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* HMIOutputPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void HMIOutputPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.sSigHeader)
}

// optional .pb.ap_hmitoap.user_action_head_unit.UserActionHeadUnit userActionHeadUnit_nu = 1844;
inline bool HMIOutputPort::_internal_has_useractionheadunit_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool HMIOutputPort::has_useractionheadunit_nu() const {
  return _internal_has_useractionheadunit_nu();
}
inline void HMIOutputPort::clear_useractionheadunit_nu() {
  useractionheadunit_nu_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::pb::ap_hmitoap::user_action_head_unit::UserActionHeadUnit HMIOutputPort::_internal_useractionheadunit_nu() const {
  return static_cast< ::pb::ap_hmitoap::user_action_head_unit::UserActionHeadUnit >(useractionheadunit_nu_);
}
inline ::pb::ap_hmitoap::user_action_head_unit::UserActionHeadUnit HMIOutputPort::useractionheadunit_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.userActionHeadUnit_nu)
  return _internal_useractionheadunit_nu();
}
inline void HMIOutputPort::_internal_set_useractionheadunit_nu(::pb::ap_hmitoap::user_action_head_unit::UserActionHeadUnit value) {
  assert(::pb::ap_hmitoap::user_action_head_unit::UserActionHeadUnit_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  useractionheadunit_nu_ = value;
}
inline void HMIOutputPort::set_useractionheadunit_nu(::pb::ap_hmitoap::user_action_head_unit::UserActionHeadUnit value) {
  _internal_set_useractionheadunit_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.userActionHeadUnit_nu)
}

// optional uint32 userActionHUCounter_nu = 3627;
inline bool HMIOutputPort::_internal_has_useractionhucounter_nu() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool HMIOutputPort::has_useractionhucounter_nu() const {
  return _internal_has_useractionhucounter_nu();
}
inline void HMIOutputPort::clear_useractionhucounter_nu() {
  useractionhucounter_nu_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 HMIOutputPort::_internal_useractionhucounter_nu() const {
  return useractionhucounter_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 HMIOutputPort::useractionhucounter_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.userActionHUCounter_nu)
  return _internal_useractionhucounter_nu();
}
inline void HMIOutputPort::_internal_set_useractionhucounter_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  useractionhucounter_nu_ = value;
}
inline void HMIOutputPort::set_useractionhucounter_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_useractionhucounter_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.userActionHUCounter_nu)
}

// optional bool pdwAutoActivate_nu = 2682;
inline bool HMIOutputPort::_internal_has_pdwautoactivate_nu() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool HMIOutputPort::has_pdwautoactivate_nu() const {
  return _internal_has_pdwautoactivate_nu();
}
inline void HMIOutputPort::clear_pdwautoactivate_nu() {
  pdwautoactivate_nu_ = false;
  _has_bits_[0] &= ~0x00000008u;
}
inline bool HMIOutputPort::_internal_pdwautoactivate_nu() const {
  return pdwautoactivate_nu_;
}
inline bool HMIOutputPort::pdwautoactivate_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.pdwAutoActivate_nu)
  return _internal_pdwautoactivate_nu();
}
inline void HMIOutputPort::_internal_set_pdwautoactivate_nu(bool value) {
  _has_bits_[0] |= 0x00000008u;
  pdwautoactivate_nu_ = value;
}
inline void HMIOutputPort::set_pdwautoactivate_nu(bool value) {
  _internal_set_pdwautoactivate_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.pdwAutoActivate_nu)
}

// optional bool lscaAutoActivate_nu = 2695;
inline bool HMIOutputPort::_internal_has_lscaautoactivate_nu() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool HMIOutputPort::has_lscaautoactivate_nu() const {
  return _internal_has_lscaautoactivate_nu();
}
inline void HMIOutputPort::clear_lscaautoactivate_nu() {
  lscaautoactivate_nu_ = false;
  _has_bits_[0] &= ~0x00000010u;
}
inline bool HMIOutputPort::_internal_lscaautoactivate_nu() const {
  return lscaautoactivate_nu_;
}
inline bool HMIOutputPort::lscaautoactivate_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.lscaAutoActivate_nu)
  return _internal_lscaautoactivate_nu();
}
inline void HMIOutputPort::_internal_set_lscaautoactivate_nu(bool value) {
  _has_bits_[0] |= 0x00000010u;
  lscaautoactivate_nu_ = value;
}
inline void HMIOutputPort::set_lscaautoactivate_nu(bool value) {
  _internal_set_lscaautoactivate_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_hmitoap.hmioutput_port.HMIOutputPort.lscaAutoActivate_nu)
}

// -------------------------------------------------------------------

// HMIOutputPort_array_port

// repeated .pb.ap_hmitoap.hmioutput_port.HMIOutputPort data = 1683;
inline int HMIOutputPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int HMIOutputPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void HMIOutputPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort* HMIOutputPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_hmitoap.hmioutput_port.HMIOutputPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort >*
HMIOutputPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_hmitoap.hmioutput_port.HMIOutputPort_array_port.data)
  return &data_;
}
inline const ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort& HMIOutputPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort& HMIOutputPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_hmitoap.hmioutput_port.HMIOutputPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort* HMIOutputPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort* HMIOutputPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_hmitoap.hmioutput_port.HMIOutputPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_hmitoap::hmioutput_port::HMIOutputPort >&
HMIOutputPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_hmitoap.hmioutput_port.HMIOutputPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace hmioutput_port
}  // namespace ap_hmitoap
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fhmitoap_2fhmioutput_5fport_2eproto