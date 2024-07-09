// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_commonvehsigprovider/gearbox_ctrl_status_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fcommonvehsigprovider_2fgearbox_5fctrl_5fstatus_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fcommonvehsigprovider_2fgearbox_5fctrl_5fstatus_5fport_2eproto

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
#include "ap_commonvehsigprovider/gear_information.pb.h"
#include "ap_commonvehsigprovider/gear_lever_information.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5fcommonvehsigprovider_2fgearbox_5fctrl_5fstatus_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fcommonvehsigprovider_2fgearbox_5fctrl_5fstatus_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fcommonvehsigprovider_2fgearbox_5fctrl_5fstatus_5fport_2eproto;
namespace pb {
namespace ap_commonvehsigprovider {
namespace gearbox_ctrl_status_port {
class GearboxCtrlStatusPort;
class GearboxCtrlStatusPortDefaultTypeInternal;
extern GearboxCtrlStatusPortDefaultTypeInternal _GearboxCtrlStatusPort_default_instance_;
class GearboxCtrlStatusPort_array_port;
class GearboxCtrlStatusPort_array_portDefaultTypeInternal;
extern GearboxCtrlStatusPort_array_portDefaultTypeInternal _GearboxCtrlStatusPort_array_port_default_instance_;
}  // namespace gearbox_ctrl_status_port
}  // namespace ap_commonvehsigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort* Arena::CreateMaybeMessage<::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort>(Arena*);
template<> ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort_array_port* Arena::CreateMaybeMessage<::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_commonvehsigprovider {
namespace gearbox_ctrl_status_port {

// ===================================================================

class GearboxCtrlStatusPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort) */ {
 public:
  GearboxCtrlStatusPort();
  virtual ~GearboxCtrlStatusPort();

  GearboxCtrlStatusPort(const GearboxCtrlStatusPort& from);
  GearboxCtrlStatusPort(GearboxCtrlStatusPort&& from) noexcept
    : GearboxCtrlStatusPort() {
    *this = ::std::move(from);
  }

  inline GearboxCtrlStatusPort& operator=(const GearboxCtrlStatusPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline GearboxCtrlStatusPort& operator=(GearboxCtrlStatusPort&& from) noexcept {
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
  static const GearboxCtrlStatusPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const GearboxCtrlStatusPort* internal_default_instance() {
    return reinterpret_cast<const GearboxCtrlStatusPort*>(
               &_GearboxCtrlStatusPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(GearboxCtrlStatusPort& a, GearboxCtrlStatusPort& b) {
    a.Swap(&b);
  }
  inline void Swap(GearboxCtrlStatusPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline GearboxCtrlStatusPort* New() const final {
    return CreateMaybeMessage<GearboxCtrlStatusPort>(nullptr);
  }

  GearboxCtrlStatusPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<GearboxCtrlStatusPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const GearboxCtrlStatusPort& from);
  void MergeFrom(const GearboxCtrlStatusPort& from);
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
  void InternalSwap(GearboxCtrlStatusPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fcommonvehsigprovider_2fgearbox_5fctrl_5fstatus_5fport_2eproto);
    return ::descriptor_table_ap_5fcommonvehsigprovider_2fgearbox_5fctrl_5fstatus_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kGearInformationFieldNumber = 1426,
    kGearLeverInformationFieldNumber = 3204,
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

  // optional .pb.ap_commonvehsigprovider.gear_information.GearInformation gearInformation = 1426;
  bool has_gearinformation() const;
  private:
  bool _internal_has_gearinformation() const;
  public:
  void clear_gearinformation();
  const ::pb::ap_commonvehsigprovider::gear_information::GearInformation& gearinformation() const;
  ::pb::ap_commonvehsigprovider::gear_information::GearInformation* release_gearinformation();
  ::pb::ap_commonvehsigprovider::gear_information::GearInformation* mutable_gearinformation();
  void set_allocated_gearinformation(::pb::ap_commonvehsigprovider::gear_information::GearInformation* gearinformation);
  private:
  const ::pb::ap_commonvehsigprovider::gear_information::GearInformation& _internal_gearinformation() const;
  ::pb::ap_commonvehsigprovider::gear_information::GearInformation* _internal_mutable_gearinformation();
  public:

  // optional .pb.ap_commonvehsigprovider.gear_lever_information.GearLeverInformation gearLeverInformation = 3204;
  bool has_gearleverinformation() const;
  private:
  bool _internal_has_gearleverinformation() const;
  public:
  void clear_gearleverinformation();
  const ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation& gearleverinformation() const;
  ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation* release_gearleverinformation();
  ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation* mutable_gearleverinformation();
  void set_allocated_gearleverinformation(::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation* gearleverinformation);
  private:
  const ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation& _internal_gearleverinformation() const;
  ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation* _internal_mutable_gearleverinformation();
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

  // @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::pb::ap_commonvehsigprovider::gear_information::GearInformation* gearinformation_;
  ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation* gearleverinformation_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  friend struct ::TableStruct_ap_5fcommonvehsigprovider_2fgearbox_5fctrl_5fstatus_5fport_2eproto;
};
// -------------------------------------------------------------------

class GearboxCtrlStatusPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort_array_port) */ {
 public:
  GearboxCtrlStatusPort_array_port();
  virtual ~GearboxCtrlStatusPort_array_port();

  GearboxCtrlStatusPort_array_port(const GearboxCtrlStatusPort_array_port& from);
  GearboxCtrlStatusPort_array_port(GearboxCtrlStatusPort_array_port&& from) noexcept
    : GearboxCtrlStatusPort_array_port() {
    *this = ::std::move(from);
  }

  inline GearboxCtrlStatusPort_array_port& operator=(const GearboxCtrlStatusPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline GearboxCtrlStatusPort_array_port& operator=(GearboxCtrlStatusPort_array_port&& from) noexcept {
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
  static const GearboxCtrlStatusPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const GearboxCtrlStatusPort_array_port* internal_default_instance() {
    return reinterpret_cast<const GearboxCtrlStatusPort_array_port*>(
               &_GearboxCtrlStatusPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(GearboxCtrlStatusPort_array_port& a, GearboxCtrlStatusPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(GearboxCtrlStatusPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline GearboxCtrlStatusPort_array_port* New() const final {
    return CreateMaybeMessage<GearboxCtrlStatusPort_array_port>(nullptr);
  }

  GearboxCtrlStatusPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<GearboxCtrlStatusPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const GearboxCtrlStatusPort_array_port& from);
  void MergeFrom(const GearboxCtrlStatusPort_array_port& from);
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
  void InternalSwap(GearboxCtrlStatusPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fcommonvehsigprovider_2fgearbox_5fctrl_5fstatus_5fport_2eproto);
    return ::descriptor_table_ap_5fcommonvehsigprovider_2fgearbox_5fctrl_5fstatus_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3624,
  };
  // repeated .pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort data = 3624;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort >*
      mutable_data();
  private:
  const ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort& _internal_data(int index) const;
  ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort* _internal_add_data();
  public:
  const ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort& data(int index) const;
  ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort > data_;
  friend struct ::TableStruct_ap_5fcommonvehsigprovider_2fgearbox_5fctrl_5fstatus_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// GearboxCtrlStatusPort

// optional uint32 uiVersionNumber = 2124;
inline bool GearboxCtrlStatusPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool GearboxCtrlStatusPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void GearboxCtrlStatusPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 GearboxCtrlStatusPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 GearboxCtrlStatusPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void GearboxCtrlStatusPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  uiversionnumber_ = value;
}
inline void GearboxCtrlStatusPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool GearboxCtrlStatusPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool GearboxCtrlStatusPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& GearboxCtrlStatusPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& GearboxCtrlStatusPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* GearboxCtrlStatusPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* GearboxCtrlStatusPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* GearboxCtrlStatusPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void GearboxCtrlStatusPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.sSigHeader)
}

// optional .pb.ap_commonvehsigprovider.gear_information.GearInformation gearInformation = 1426;
inline bool GearboxCtrlStatusPort::_internal_has_gearinformation() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || gearinformation_ != nullptr);
  return value;
}
inline bool GearboxCtrlStatusPort::has_gearinformation() const {
  return _internal_has_gearinformation();
}
inline const ::pb::ap_commonvehsigprovider::gear_information::GearInformation& GearboxCtrlStatusPort::_internal_gearinformation() const {
  const ::pb::ap_commonvehsigprovider::gear_information::GearInformation* p = gearinformation_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::ap_commonvehsigprovider::gear_information::GearInformation*>(
      &::pb::ap_commonvehsigprovider::gear_information::_GearInformation_default_instance_);
}
inline const ::pb::ap_commonvehsigprovider::gear_information::GearInformation& GearboxCtrlStatusPort::gearinformation() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.gearInformation)
  return _internal_gearinformation();
}
inline ::pb::ap_commonvehsigprovider::gear_information::GearInformation* GearboxCtrlStatusPort::release_gearinformation() {
  // @@protoc_insertion_point(field_release:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.gearInformation)
  _has_bits_[0] &= ~0x00000002u;
  ::pb::ap_commonvehsigprovider::gear_information::GearInformation* temp = gearinformation_;
  gearinformation_ = nullptr;
  return temp;
}
inline ::pb::ap_commonvehsigprovider::gear_information::GearInformation* GearboxCtrlStatusPort::_internal_mutable_gearinformation() {
  _has_bits_[0] |= 0x00000002u;
  if (gearinformation_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::ap_commonvehsigprovider::gear_information::GearInformation>(GetArenaNoVirtual());
    gearinformation_ = p;
  }
  return gearinformation_;
}
inline ::pb::ap_commonvehsigprovider::gear_information::GearInformation* GearboxCtrlStatusPort::mutable_gearinformation() {
  // @@protoc_insertion_point(field_mutable:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.gearInformation)
  return _internal_mutable_gearinformation();
}
inline void GearboxCtrlStatusPort::set_allocated_gearinformation(::pb::ap_commonvehsigprovider::gear_information::GearInformation* gearinformation) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(gearinformation_);
  }
  if (gearinformation) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      gearinformation = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, gearinformation, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  gearinformation_ = gearinformation;
  // @@protoc_insertion_point(field_set_allocated:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.gearInformation)
}

// optional .pb.ap_commonvehsigprovider.gear_lever_information.GearLeverInformation gearLeverInformation = 3204;
inline bool GearboxCtrlStatusPort::_internal_has_gearleverinformation() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  PROTOBUF_ASSUME(!value || gearleverinformation_ != nullptr);
  return value;
}
inline bool GearboxCtrlStatusPort::has_gearleverinformation() const {
  return _internal_has_gearleverinformation();
}
inline const ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation& GearboxCtrlStatusPort::_internal_gearleverinformation() const {
  const ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation* p = gearleverinformation_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation*>(
      &::pb::ap_commonvehsigprovider::gear_lever_information::_GearLeverInformation_default_instance_);
}
inline const ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation& GearboxCtrlStatusPort::gearleverinformation() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.gearLeverInformation)
  return _internal_gearleverinformation();
}
inline ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation* GearboxCtrlStatusPort::release_gearleverinformation() {
  // @@protoc_insertion_point(field_release:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.gearLeverInformation)
  _has_bits_[0] &= ~0x00000004u;
  ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation* temp = gearleverinformation_;
  gearleverinformation_ = nullptr;
  return temp;
}
inline ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation* GearboxCtrlStatusPort::_internal_mutable_gearleverinformation() {
  _has_bits_[0] |= 0x00000004u;
  if (gearleverinformation_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation>(GetArenaNoVirtual());
    gearleverinformation_ = p;
  }
  return gearleverinformation_;
}
inline ::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation* GearboxCtrlStatusPort::mutable_gearleverinformation() {
  // @@protoc_insertion_point(field_mutable:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.gearLeverInformation)
  return _internal_mutable_gearleverinformation();
}
inline void GearboxCtrlStatusPort::set_allocated_gearleverinformation(::pb::ap_commonvehsigprovider::gear_lever_information::GearLeverInformation* gearleverinformation) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(gearleverinformation_);
  }
  if (gearleverinformation) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      gearleverinformation = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, gearleverinformation, submessage_arena);
    }
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  gearleverinformation_ = gearleverinformation;
  // @@protoc_insertion_point(field_set_allocated:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort.gearLeverInformation)
}

// -------------------------------------------------------------------

// GearboxCtrlStatusPort_array_port

// repeated .pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort data = 3624;
inline int GearboxCtrlStatusPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int GearboxCtrlStatusPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void GearboxCtrlStatusPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort* GearboxCtrlStatusPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort >*
GearboxCtrlStatusPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort_array_port.data)
  return &data_;
}
inline const ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort& GearboxCtrlStatusPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort& GearboxCtrlStatusPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort* GearboxCtrlStatusPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort* GearboxCtrlStatusPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::gearbox_ctrl_status_port::GearboxCtrlStatusPort >&
GearboxCtrlStatusPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_commonvehsigprovider.gearbox_ctrl_status_port.GearboxCtrlStatusPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace gearbox_ctrl_status_port
}  // namespace ap_commonvehsigprovider
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fcommonvehsigprovider_2fgearbox_5fctrl_5fstatus_5fport_2eproto
