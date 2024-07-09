// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_trjctl/gearbox_ctrl_request_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5ftrjctl_2fgearbox_5fctrl_5frequest_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5ftrjctl_2fgearbox_5fctrl_5frequest_5fport_2eproto

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
#include "ap_commonvehsigprovider/gear.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5ftrjctl_2fgearbox_5fctrl_5frequest_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5ftrjctl_2fgearbox_5fctrl_5frequest_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftrjctl_2fgearbox_5fctrl_5frequest_5fport_2eproto;
namespace pb {
namespace ap_trjctl {
namespace gearbox_ctrl_request_port {
class GearboxCtrlRequestPort;
class GearboxCtrlRequestPortDefaultTypeInternal;
extern GearboxCtrlRequestPortDefaultTypeInternal _GearboxCtrlRequestPort_default_instance_;
class GearboxCtrlRequestPort_array_port;
class GearboxCtrlRequestPort_array_portDefaultTypeInternal;
extern GearboxCtrlRequestPort_array_portDefaultTypeInternal _GearboxCtrlRequestPort_array_port_default_instance_;
}  // namespace gearbox_ctrl_request_port
}  // namespace ap_trjctl
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort* Arena::CreateMaybeMessage<::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort>(Arena*);
template<> ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort_array_port* Arena::CreateMaybeMessage<::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_trjctl {
namespace gearbox_ctrl_request_port {

// ===================================================================

class GearboxCtrlRequestPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort) */ {
 public:
  GearboxCtrlRequestPort();
  virtual ~GearboxCtrlRequestPort();

  GearboxCtrlRequestPort(const GearboxCtrlRequestPort& from);
  GearboxCtrlRequestPort(GearboxCtrlRequestPort&& from) noexcept
    : GearboxCtrlRequestPort() {
    *this = ::std::move(from);
  }

  inline GearboxCtrlRequestPort& operator=(const GearboxCtrlRequestPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline GearboxCtrlRequestPort& operator=(GearboxCtrlRequestPort&& from) noexcept {
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
  static const GearboxCtrlRequestPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const GearboxCtrlRequestPort* internal_default_instance() {
    return reinterpret_cast<const GearboxCtrlRequestPort*>(
               &_GearboxCtrlRequestPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(GearboxCtrlRequestPort& a, GearboxCtrlRequestPort& b) {
    a.Swap(&b);
  }
  inline void Swap(GearboxCtrlRequestPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline GearboxCtrlRequestPort* New() const final {
    return CreateMaybeMessage<GearboxCtrlRequestPort>(nullptr);
  }

  GearboxCtrlRequestPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<GearboxCtrlRequestPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const GearboxCtrlRequestPort& from);
  void MergeFrom(const GearboxCtrlRequestPort& from);
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
  void InternalSwap(GearboxCtrlRequestPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftrjctl_2fgearbox_5fctrl_5frequest_5fport_2eproto);
    return ::descriptor_table_ap_5ftrjctl_2fgearbox_5fctrl_5frequest_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kGearboxCtrlRequestNuFieldNumber = 3762,
    kGearSwitchRequestNuFieldNumber = 2874,
    kGearReqNuFieldNumber = 480,
    kUiVersionNumberFieldNumber = 2124,
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

  // optional bool gearboxCtrlRequest_nu = 3762;
  bool has_gearboxctrlrequest_nu() const;
  private:
  bool _internal_has_gearboxctrlrequest_nu() const;
  public:
  void clear_gearboxctrlrequest_nu();
  bool gearboxctrlrequest_nu() const;
  void set_gearboxctrlrequest_nu(bool value);
  private:
  bool _internal_gearboxctrlrequest_nu() const;
  void _internal_set_gearboxctrlrequest_nu(bool value);
  public:

  // optional bool gearSwitchRequest_nu = 2874;
  bool has_gearswitchrequest_nu() const;
  private:
  bool _internal_has_gearswitchrequest_nu() const;
  public:
  void clear_gearswitchrequest_nu();
  bool gearswitchrequest_nu() const;
  void set_gearswitchrequest_nu(bool value);
  private:
  bool _internal_gearswitchrequest_nu() const;
  void _internal_set_gearswitchrequest_nu(bool value);
  public:

  // optional .pb.ap_commonvehsigprovider.gear.Gear gearReq_nu = 480;
  bool has_gearreq_nu() const;
  private:
  bool _internal_has_gearreq_nu() const;
  public:
  void clear_gearreq_nu();
  ::pb::ap_commonvehsigprovider::gear::Gear gearreq_nu() const;
  void set_gearreq_nu(::pb::ap_commonvehsigprovider::gear::Gear value);
  private:
  ::pb::ap_commonvehsigprovider::gear::Gear _internal_gearreq_nu() const;
  void _internal_set_gearreq_nu(::pb::ap_commonvehsigprovider::gear::Gear value);
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

  // @@protoc_insertion_point(class_scope:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  bool gearboxctrlrequest_nu_;
  bool gearswitchrequest_nu_;
  int gearreq_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  friend struct ::TableStruct_ap_5ftrjctl_2fgearbox_5fctrl_5frequest_5fport_2eproto;
};
// -------------------------------------------------------------------

class GearboxCtrlRequestPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort_array_port) */ {
 public:
  GearboxCtrlRequestPort_array_port();
  virtual ~GearboxCtrlRequestPort_array_port();

  GearboxCtrlRequestPort_array_port(const GearboxCtrlRequestPort_array_port& from);
  GearboxCtrlRequestPort_array_port(GearboxCtrlRequestPort_array_port&& from) noexcept
    : GearboxCtrlRequestPort_array_port() {
    *this = ::std::move(from);
  }

  inline GearboxCtrlRequestPort_array_port& operator=(const GearboxCtrlRequestPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline GearboxCtrlRequestPort_array_port& operator=(GearboxCtrlRequestPort_array_port&& from) noexcept {
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
  static const GearboxCtrlRequestPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const GearboxCtrlRequestPort_array_port* internal_default_instance() {
    return reinterpret_cast<const GearboxCtrlRequestPort_array_port*>(
               &_GearboxCtrlRequestPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(GearboxCtrlRequestPort_array_port& a, GearboxCtrlRequestPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(GearboxCtrlRequestPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline GearboxCtrlRequestPort_array_port* New() const final {
    return CreateMaybeMessage<GearboxCtrlRequestPort_array_port>(nullptr);
  }

  GearboxCtrlRequestPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<GearboxCtrlRequestPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const GearboxCtrlRequestPort_array_port& from);
  void MergeFrom(const GearboxCtrlRequestPort_array_port& from);
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
  void InternalSwap(GearboxCtrlRequestPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftrjctl_2fgearbox_5fctrl_5frequest_5fport_2eproto);
    return ::descriptor_table_ap_5ftrjctl_2fgearbox_5fctrl_5frequest_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2038,
  };
  // repeated .pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort data = 2038;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort >*
      mutable_data();
  private:
  const ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort& _internal_data(int index) const;
  ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort* _internal_add_data();
  public:
  const ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort& data(int index) const;
  ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort > data_;
  friend struct ::TableStruct_ap_5ftrjctl_2fgearbox_5fctrl_5frequest_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// GearboxCtrlRequestPort

// optional uint32 uiVersionNumber = 2124;
inline bool GearboxCtrlRequestPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool GearboxCtrlRequestPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void GearboxCtrlRequestPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 GearboxCtrlRequestPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 GearboxCtrlRequestPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void GearboxCtrlRequestPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  uiversionnumber_ = value;
}
inline void GearboxCtrlRequestPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool GearboxCtrlRequestPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool GearboxCtrlRequestPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& GearboxCtrlRequestPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& GearboxCtrlRequestPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* GearboxCtrlRequestPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* GearboxCtrlRequestPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* GearboxCtrlRequestPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void GearboxCtrlRequestPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort.sSigHeader)
}

// optional bool gearboxCtrlRequest_nu = 3762;
inline bool GearboxCtrlRequestPort::_internal_has_gearboxctrlrequest_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool GearboxCtrlRequestPort::has_gearboxctrlrequest_nu() const {
  return _internal_has_gearboxctrlrequest_nu();
}
inline void GearboxCtrlRequestPort::clear_gearboxctrlrequest_nu() {
  gearboxctrlrequest_nu_ = false;
  _has_bits_[0] &= ~0x00000002u;
}
inline bool GearboxCtrlRequestPort::_internal_gearboxctrlrequest_nu() const {
  return gearboxctrlrequest_nu_;
}
inline bool GearboxCtrlRequestPort::gearboxctrlrequest_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort.gearboxCtrlRequest_nu)
  return _internal_gearboxctrlrequest_nu();
}
inline void GearboxCtrlRequestPort::_internal_set_gearboxctrlrequest_nu(bool value) {
  _has_bits_[0] |= 0x00000002u;
  gearboxctrlrequest_nu_ = value;
}
inline void GearboxCtrlRequestPort::set_gearboxctrlrequest_nu(bool value) {
  _internal_set_gearboxctrlrequest_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort.gearboxCtrlRequest_nu)
}

// optional bool gearSwitchRequest_nu = 2874;
inline bool GearboxCtrlRequestPort::_internal_has_gearswitchrequest_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool GearboxCtrlRequestPort::has_gearswitchrequest_nu() const {
  return _internal_has_gearswitchrequest_nu();
}
inline void GearboxCtrlRequestPort::clear_gearswitchrequest_nu() {
  gearswitchrequest_nu_ = false;
  _has_bits_[0] &= ~0x00000004u;
}
inline bool GearboxCtrlRequestPort::_internal_gearswitchrequest_nu() const {
  return gearswitchrequest_nu_;
}
inline bool GearboxCtrlRequestPort::gearswitchrequest_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort.gearSwitchRequest_nu)
  return _internal_gearswitchrequest_nu();
}
inline void GearboxCtrlRequestPort::_internal_set_gearswitchrequest_nu(bool value) {
  _has_bits_[0] |= 0x00000004u;
  gearswitchrequest_nu_ = value;
}
inline void GearboxCtrlRequestPort::set_gearswitchrequest_nu(bool value) {
  _internal_set_gearswitchrequest_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort.gearSwitchRequest_nu)
}

// optional .pb.ap_commonvehsigprovider.gear.Gear gearReq_nu = 480;
inline bool GearboxCtrlRequestPort::_internal_has_gearreq_nu() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool GearboxCtrlRequestPort::has_gearreq_nu() const {
  return _internal_has_gearreq_nu();
}
inline void GearboxCtrlRequestPort::clear_gearreq_nu() {
  gearreq_nu_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::pb::ap_commonvehsigprovider::gear::Gear GearboxCtrlRequestPort::_internal_gearreq_nu() const {
  return static_cast< ::pb::ap_commonvehsigprovider::gear::Gear >(gearreq_nu_);
}
inline ::pb::ap_commonvehsigprovider::gear::Gear GearboxCtrlRequestPort::gearreq_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort.gearReq_nu)
  return _internal_gearreq_nu();
}
inline void GearboxCtrlRequestPort::_internal_set_gearreq_nu(::pb::ap_commonvehsigprovider::gear::Gear value) {
  assert(::pb::ap_commonvehsigprovider::gear::Gear_IsValid(value));
  _has_bits_[0] |= 0x00000008u;
  gearreq_nu_ = value;
}
inline void GearboxCtrlRequestPort::set_gearreq_nu(::pb::ap_commonvehsigprovider::gear::Gear value) {
  _internal_set_gearreq_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort.gearReq_nu)
}

// -------------------------------------------------------------------

// GearboxCtrlRequestPort_array_port

// repeated .pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort data = 2038;
inline int GearboxCtrlRequestPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int GearboxCtrlRequestPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void GearboxCtrlRequestPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort* GearboxCtrlRequestPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort >*
GearboxCtrlRequestPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort_array_port.data)
  return &data_;
}
inline const ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort& GearboxCtrlRequestPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort& GearboxCtrlRequestPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort* GearboxCtrlRequestPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort* GearboxCtrlRequestPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::gearbox_ctrl_request_port::GearboxCtrlRequestPort >&
GearboxCtrlRequestPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_trjctl.gearbox_ctrl_request_port.GearboxCtrlRequestPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace gearbox_ctrl_request_port
}  // namespace ap_trjctl
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5ftrjctl_2fgearbox_5fctrl_5frequest_5fport_2eproto
