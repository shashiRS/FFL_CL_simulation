// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vc/user_event_data_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_vc_2fuser_5fevent_5fdata_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_vc_2fuser_5fevent_5fdata_5fport_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_vc_2fuser_5fevent_5fdata_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_vc_2fuser_5fevent_5fdata_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_vc_2fuser_5fevent_5fdata_5fport_2eproto;
namespace pb {
namespace vc {
namespace user_event_data_port {
class UserEventDataPort;
class UserEventDataPortDefaultTypeInternal;
extern UserEventDataPortDefaultTypeInternal _UserEventDataPort_default_instance_;
class UserEventDataPort_array_port;
class UserEventDataPort_array_portDefaultTypeInternal;
extern UserEventDataPort_array_portDefaultTypeInternal _UserEventDataPort_array_port_default_instance_;
}  // namespace user_event_data_port
}  // namespace vc
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::vc::user_event_data_port::UserEventDataPort* Arena::CreateMaybeMessage<::pb::vc::user_event_data_port::UserEventDataPort>(Arena*);
template<> ::pb::vc::user_event_data_port::UserEventDataPort_array_port* Arena::CreateMaybeMessage<::pb::vc::user_event_data_port::UserEventDataPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace vc {
namespace user_event_data_port {

// ===================================================================

class UserEventDataPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.vc.user_event_data_port.UserEventDataPort) */ {
 public:
  UserEventDataPort();
  virtual ~UserEventDataPort();

  UserEventDataPort(const UserEventDataPort& from);
  UserEventDataPort(UserEventDataPort&& from) noexcept
    : UserEventDataPort() {
    *this = ::std::move(from);
  }

  inline UserEventDataPort& operator=(const UserEventDataPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline UserEventDataPort& operator=(UserEventDataPort&& from) noexcept {
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
  static const UserEventDataPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UserEventDataPort* internal_default_instance() {
    return reinterpret_cast<const UserEventDataPort*>(
               &_UserEventDataPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UserEventDataPort& a, UserEventDataPort& b) {
    a.Swap(&b);
  }
  inline void Swap(UserEventDataPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UserEventDataPort* New() const final {
    return CreateMaybeMessage<UserEventDataPort>(nullptr);
  }

  UserEventDataPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UserEventDataPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UserEventDataPort& from);
  void MergeFrom(const UserEventDataPort& from);
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
  void InternalSwap(UserEventDataPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.vc.user_event_data_port.UserEventDataPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_vc_2fuser_5fevent_5fdata_5fport_2eproto);
    return ::descriptor_table_vc_2fuser_5fevent_5fdata_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kDeltaZoomFieldNumber = 3970,
    kClickEventXPxFieldNumber = 237,
    kClickEventYPxFieldNumber = 1625,
    kUiVersionNumberFieldNumber = 2124,
    kDeltaAzimuthAngleU16FieldNumber = 2339,
    kGestureFingerNuU8FieldNumber = 2378,
    kDeltaPolarAngleU16FieldNumber = 3026,
    kGestureCounterFieldNumber = 3375,
    kIsSequenceFieldNumber = 3707,
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

  // optional uint32 deltaZoom = 3970;
  bool has_deltazoom() const;
  private:
  bool _internal_has_deltazoom() const;
  public:
  void clear_deltazoom();
  ::PROTOBUF_NAMESPACE_ID::uint32 deltazoom() const;
  void set_deltazoom(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_deltazoom() const;
  void _internal_set_deltazoom(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional float clickEventX_px = 237;
  bool has_clickeventx_px() const;
  private:
  bool _internal_has_clickeventx_px() const;
  public:
  void clear_clickeventx_px();
  float clickeventx_px() const;
  void set_clickeventx_px(float value);
  private:
  float _internal_clickeventx_px() const;
  void _internal_set_clickeventx_px(float value);
  public:

  // optional float clickEventY_px = 1625;
  bool has_clickeventy_px() const;
  private:
  bool _internal_has_clickeventy_px() const;
  public:
  void clear_clickeventy_px();
  float clickeventy_px() const;
  void set_clickeventy_px(float value);
  private:
  float _internal_clickeventy_px() const;
  void _internal_set_clickeventy_px(float value);
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

  // optional uint32 deltaAzimuthAngle_u16 = 2339;
  bool has_deltaazimuthangle_u16() const;
  private:
  bool _internal_has_deltaazimuthangle_u16() const;
  public:
  void clear_deltaazimuthangle_u16();
  ::PROTOBUF_NAMESPACE_ID::uint32 deltaazimuthangle_u16() const;
  void set_deltaazimuthangle_u16(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_deltaazimuthangle_u16() const;
  void _internal_set_deltaazimuthangle_u16(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 GestureFinger_nu_u8 = 2378;
  bool has_gesturefinger_nu_u8() const;
  private:
  bool _internal_has_gesturefinger_nu_u8() const;
  public:
  void clear_gesturefinger_nu_u8();
  ::PROTOBUF_NAMESPACE_ID::uint32 gesturefinger_nu_u8() const;
  void set_gesturefinger_nu_u8(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_gesturefinger_nu_u8() const;
  void _internal_set_gesturefinger_nu_u8(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 deltaPolarAngle_u16 = 3026;
  bool has_deltapolarangle_u16() const;
  private:
  bool _internal_has_deltapolarangle_u16() const;
  public:
  void clear_deltapolarangle_u16();
  ::PROTOBUF_NAMESPACE_ID::uint32 deltapolarangle_u16() const;
  void set_deltapolarangle_u16(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_deltapolarangle_u16() const;
  void _internal_set_deltapolarangle_u16(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 gestureCounter = 3375;
  bool has_gesturecounter() const;
  private:
  bool _internal_has_gesturecounter() const;
  public:
  void clear_gesturecounter();
  ::PROTOBUF_NAMESPACE_ID::uint32 gesturecounter() const;
  void set_gesturecounter(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_gesturecounter() const;
  void _internal_set_gesturecounter(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional bool isSequence = 3707;
  bool has_issequence() const;
  private:
  bool _internal_has_issequence() const;
  public:
  void clear_issequence();
  bool issequence() const;
  void set_issequence(bool value);
  private:
  bool _internal_issequence() const;
  void _internal_set_issequence(bool value);
  public:

  // @@protoc_insertion_point(class_scope:pb.vc.user_event_data_port.UserEventDataPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint32 deltazoom_;
  float clickeventx_px_;
  float clickeventy_px_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  ::PROTOBUF_NAMESPACE_ID::uint32 deltaazimuthangle_u16_;
  ::PROTOBUF_NAMESPACE_ID::uint32 gesturefinger_nu_u8_;
  ::PROTOBUF_NAMESPACE_ID::uint32 deltapolarangle_u16_;
  ::PROTOBUF_NAMESPACE_ID::uint32 gesturecounter_;
  bool issequence_;
  friend struct ::TableStruct_vc_2fuser_5fevent_5fdata_5fport_2eproto;
};
// -------------------------------------------------------------------

class UserEventDataPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.vc.user_event_data_port.UserEventDataPort_array_port) */ {
 public:
  UserEventDataPort_array_port();
  virtual ~UserEventDataPort_array_port();

  UserEventDataPort_array_port(const UserEventDataPort_array_port& from);
  UserEventDataPort_array_port(UserEventDataPort_array_port&& from) noexcept
    : UserEventDataPort_array_port() {
    *this = ::std::move(from);
  }

  inline UserEventDataPort_array_port& operator=(const UserEventDataPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UserEventDataPort_array_port& operator=(UserEventDataPort_array_port&& from) noexcept {
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
  static const UserEventDataPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UserEventDataPort_array_port* internal_default_instance() {
    return reinterpret_cast<const UserEventDataPort_array_port*>(
               &_UserEventDataPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UserEventDataPort_array_port& a, UserEventDataPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UserEventDataPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UserEventDataPort_array_port* New() const final {
    return CreateMaybeMessage<UserEventDataPort_array_port>(nullptr);
  }

  UserEventDataPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UserEventDataPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UserEventDataPort_array_port& from);
  void MergeFrom(const UserEventDataPort_array_port& from);
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
  void InternalSwap(UserEventDataPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.vc.user_event_data_port.UserEventDataPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_vc_2fuser_5fevent_5fdata_5fport_2eproto);
    return ::descriptor_table_vc_2fuser_5fevent_5fdata_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3171,
  };
  // repeated .pb.vc.user_event_data_port.UserEventDataPort data = 3171;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::vc::user_event_data_port::UserEventDataPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::vc::user_event_data_port::UserEventDataPort >*
      mutable_data();
  private:
  const ::pb::vc::user_event_data_port::UserEventDataPort& _internal_data(int index) const;
  ::pb::vc::user_event_data_port::UserEventDataPort* _internal_add_data();
  public:
  const ::pb::vc::user_event_data_port::UserEventDataPort& data(int index) const;
  ::pb::vc::user_event_data_port::UserEventDataPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::vc::user_event_data_port::UserEventDataPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.vc.user_event_data_port.UserEventDataPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::vc::user_event_data_port::UserEventDataPort > data_;
  friend struct ::TableStruct_vc_2fuser_5fevent_5fdata_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UserEventDataPort

// optional uint32 uiVersionNumber = 2124;
inline bool UserEventDataPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool UserEventDataPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void UserEventDataPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UserEventDataPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UserEventDataPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.vc.user_event_data_port.UserEventDataPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void UserEventDataPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  uiversionnumber_ = value;
}
inline void UserEventDataPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.vc.user_event_data_port.UserEventDataPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool UserEventDataPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool UserEventDataPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& UserEventDataPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& UserEventDataPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.vc.user_event_data_port.UserEventDataPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* UserEventDataPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.vc.user_event_data_port.UserEventDataPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* UserEventDataPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* UserEventDataPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.vc.user_event_data_port.UserEventDataPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void UserEventDataPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.vc.user_event_data_port.UserEventDataPort.sSigHeader)
}

// optional uint32 deltaPolarAngle_u16 = 3026;
inline bool UserEventDataPort::_internal_has_deltapolarangle_u16() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool UserEventDataPort::has_deltapolarangle_u16() const {
  return _internal_has_deltapolarangle_u16();
}
inline void UserEventDataPort::clear_deltapolarangle_u16() {
  deltapolarangle_u16_ = 0u;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UserEventDataPort::_internal_deltapolarangle_u16() const {
  return deltapolarangle_u16_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UserEventDataPort::deltapolarangle_u16() const {
  // @@protoc_insertion_point(field_get:pb.vc.user_event_data_port.UserEventDataPort.deltaPolarAngle_u16)
  return _internal_deltapolarangle_u16();
}
inline void UserEventDataPort::_internal_set_deltapolarangle_u16(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000080u;
  deltapolarangle_u16_ = value;
}
inline void UserEventDataPort::set_deltapolarangle_u16(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_deltapolarangle_u16(value);
  // @@protoc_insertion_point(field_set:pb.vc.user_event_data_port.UserEventDataPort.deltaPolarAngle_u16)
}

// optional uint32 deltaAzimuthAngle_u16 = 2339;
inline bool UserEventDataPort::_internal_has_deltaazimuthangle_u16() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool UserEventDataPort::has_deltaazimuthangle_u16() const {
  return _internal_has_deltaazimuthangle_u16();
}
inline void UserEventDataPort::clear_deltaazimuthangle_u16() {
  deltaazimuthangle_u16_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UserEventDataPort::_internal_deltaazimuthangle_u16() const {
  return deltaazimuthangle_u16_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UserEventDataPort::deltaazimuthangle_u16() const {
  // @@protoc_insertion_point(field_get:pb.vc.user_event_data_port.UserEventDataPort.deltaAzimuthAngle_u16)
  return _internal_deltaazimuthangle_u16();
}
inline void UserEventDataPort::_internal_set_deltaazimuthangle_u16(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  deltaazimuthangle_u16_ = value;
}
inline void UserEventDataPort::set_deltaazimuthangle_u16(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_deltaazimuthangle_u16(value);
  // @@protoc_insertion_point(field_set:pb.vc.user_event_data_port.UserEventDataPort.deltaAzimuthAngle_u16)
}

// optional uint32 GestureFinger_nu_u8 = 2378;
inline bool UserEventDataPort::_internal_has_gesturefinger_nu_u8() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool UserEventDataPort::has_gesturefinger_nu_u8() const {
  return _internal_has_gesturefinger_nu_u8();
}
inline void UserEventDataPort::clear_gesturefinger_nu_u8() {
  gesturefinger_nu_u8_ = 0u;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UserEventDataPort::_internal_gesturefinger_nu_u8() const {
  return gesturefinger_nu_u8_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UserEventDataPort::gesturefinger_nu_u8() const {
  // @@protoc_insertion_point(field_get:pb.vc.user_event_data_port.UserEventDataPort.GestureFinger_nu_u8)
  return _internal_gesturefinger_nu_u8();
}
inline void UserEventDataPort::_internal_set_gesturefinger_nu_u8(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000040u;
  gesturefinger_nu_u8_ = value;
}
inline void UserEventDataPort::set_gesturefinger_nu_u8(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_gesturefinger_nu_u8(value);
  // @@protoc_insertion_point(field_set:pb.vc.user_event_data_port.UserEventDataPort.GestureFinger_nu_u8)
}

// optional uint32 gestureCounter = 3375;
inline bool UserEventDataPort::_internal_has_gesturecounter() const {
  bool value = (_has_bits_[0] & 0x00000100u) != 0;
  return value;
}
inline bool UserEventDataPort::has_gesturecounter() const {
  return _internal_has_gesturecounter();
}
inline void UserEventDataPort::clear_gesturecounter() {
  gesturecounter_ = 0u;
  _has_bits_[0] &= ~0x00000100u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UserEventDataPort::_internal_gesturecounter() const {
  return gesturecounter_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UserEventDataPort::gesturecounter() const {
  // @@protoc_insertion_point(field_get:pb.vc.user_event_data_port.UserEventDataPort.gestureCounter)
  return _internal_gesturecounter();
}
inline void UserEventDataPort::_internal_set_gesturecounter(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000100u;
  gesturecounter_ = value;
}
inline void UserEventDataPort::set_gesturecounter(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_gesturecounter(value);
  // @@protoc_insertion_point(field_set:pb.vc.user_event_data_port.UserEventDataPort.gestureCounter)
}

// optional uint32 deltaZoom = 3970;
inline bool UserEventDataPort::_internal_has_deltazoom() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool UserEventDataPort::has_deltazoom() const {
  return _internal_has_deltazoom();
}
inline void UserEventDataPort::clear_deltazoom() {
  deltazoom_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UserEventDataPort::_internal_deltazoom() const {
  return deltazoom_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UserEventDataPort::deltazoom() const {
  // @@protoc_insertion_point(field_get:pb.vc.user_event_data_port.UserEventDataPort.deltaZoom)
  return _internal_deltazoom();
}
inline void UserEventDataPort::_internal_set_deltazoom(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  deltazoom_ = value;
}
inline void UserEventDataPort::set_deltazoom(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_deltazoom(value);
  // @@protoc_insertion_point(field_set:pb.vc.user_event_data_port.UserEventDataPort.deltaZoom)
}

// optional bool isSequence = 3707;
inline bool UserEventDataPort::_internal_has_issequence() const {
  bool value = (_has_bits_[0] & 0x00000200u) != 0;
  return value;
}
inline bool UserEventDataPort::has_issequence() const {
  return _internal_has_issequence();
}
inline void UserEventDataPort::clear_issequence() {
  issequence_ = false;
  _has_bits_[0] &= ~0x00000200u;
}
inline bool UserEventDataPort::_internal_issequence() const {
  return issequence_;
}
inline bool UserEventDataPort::issequence() const {
  // @@protoc_insertion_point(field_get:pb.vc.user_event_data_port.UserEventDataPort.isSequence)
  return _internal_issequence();
}
inline void UserEventDataPort::_internal_set_issequence(bool value) {
  _has_bits_[0] |= 0x00000200u;
  issequence_ = value;
}
inline void UserEventDataPort::set_issequence(bool value) {
  _internal_set_issequence(value);
  // @@protoc_insertion_point(field_set:pb.vc.user_event_data_port.UserEventDataPort.isSequence)
}

// optional float clickEventX_px = 237;
inline bool UserEventDataPort::_internal_has_clickeventx_px() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool UserEventDataPort::has_clickeventx_px() const {
  return _internal_has_clickeventx_px();
}
inline void UserEventDataPort::clear_clickeventx_px() {
  clickeventx_px_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float UserEventDataPort::_internal_clickeventx_px() const {
  return clickeventx_px_;
}
inline float UserEventDataPort::clickeventx_px() const {
  // @@protoc_insertion_point(field_get:pb.vc.user_event_data_port.UserEventDataPort.clickEventX_px)
  return _internal_clickeventx_px();
}
inline void UserEventDataPort::_internal_set_clickeventx_px(float value) {
  _has_bits_[0] |= 0x00000004u;
  clickeventx_px_ = value;
}
inline void UserEventDataPort::set_clickeventx_px(float value) {
  _internal_set_clickeventx_px(value);
  // @@protoc_insertion_point(field_set:pb.vc.user_event_data_port.UserEventDataPort.clickEventX_px)
}

// optional float clickEventY_px = 1625;
inline bool UserEventDataPort::_internal_has_clickeventy_px() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool UserEventDataPort::has_clickeventy_px() const {
  return _internal_has_clickeventy_px();
}
inline void UserEventDataPort::clear_clickeventy_px() {
  clickeventy_px_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float UserEventDataPort::_internal_clickeventy_px() const {
  return clickeventy_px_;
}
inline float UserEventDataPort::clickeventy_px() const {
  // @@protoc_insertion_point(field_get:pb.vc.user_event_data_port.UserEventDataPort.clickEventY_px)
  return _internal_clickeventy_px();
}
inline void UserEventDataPort::_internal_set_clickeventy_px(float value) {
  _has_bits_[0] |= 0x00000008u;
  clickeventy_px_ = value;
}
inline void UserEventDataPort::set_clickeventy_px(float value) {
  _internal_set_clickeventy_px(value);
  // @@protoc_insertion_point(field_set:pb.vc.user_event_data_port.UserEventDataPort.clickEventY_px)
}

// -------------------------------------------------------------------

// UserEventDataPort_array_port

// repeated .pb.vc.user_event_data_port.UserEventDataPort data = 3171;
inline int UserEventDataPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UserEventDataPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void UserEventDataPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::vc::user_event_data_port::UserEventDataPort* UserEventDataPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.vc.user_event_data_port.UserEventDataPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::vc::user_event_data_port::UserEventDataPort >*
UserEventDataPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.vc.user_event_data_port.UserEventDataPort_array_port.data)
  return &data_;
}
inline const ::pb::vc::user_event_data_port::UserEventDataPort& UserEventDataPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::vc::user_event_data_port::UserEventDataPort& UserEventDataPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.vc.user_event_data_port.UserEventDataPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::vc::user_event_data_port::UserEventDataPort* UserEventDataPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::vc::user_event_data_port::UserEventDataPort* UserEventDataPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.vc.user_event_data_port.UserEventDataPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::vc::user_event_data_port::UserEventDataPort >&
UserEventDataPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.vc.user_event_data_port.UserEventDataPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace user_event_data_port
}  // namespace vc
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_vc_2fuser_5fevent_5fdata_5fport_2eproto
