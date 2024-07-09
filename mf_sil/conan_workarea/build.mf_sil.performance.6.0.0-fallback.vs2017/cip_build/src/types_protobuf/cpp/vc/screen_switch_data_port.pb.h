// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vc/screen_switch_data_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_vc_2fscreen_5fswitch_5fdata_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_vc_2fscreen_5fswitch_5fdata_5fport_2eproto

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
#include "ap_hmitoap/screen_types.pb.h"
#include "vc/blind_spot_view_status.pb.h"
#include "vc/transparency_preset.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_vc_2fscreen_5fswitch_5fdata_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_vc_2fscreen_5fswitch_5fdata_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto;
namespace pb {
namespace vc {
namespace screen_switch_data_port {
class ScreenSwitchDataPort;
class ScreenSwitchDataPortDefaultTypeInternal;
extern ScreenSwitchDataPortDefaultTypeInternal _ScreenSwitchDataPort_default_instance_;
class ScreenSwitchDataPort_array_port;
class ScreenSwitchDataPort_array_portDefaultTypeInternal;
extern ScreenSwitchDataPort_array_portDefaultTypeInternal _ScreenSwitchDataPort_array_port_default_instance_;
}  // namespace screen_switch_data_port
}  // namespace vc
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort* Arena::CreateMaybeMessage<::pb::vc::screen_switch_data_port::ScreenSwitchDataPort>(Arena*);
template<> ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort_array_port* Arena::CreateMaybeMessage<::pb::vc::screen_switch_data_port::ScreenSwitchDataPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace vc {
namespace screen_switch_data_port {

// ===================================================================

class ScreenSwitchDataPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.vc.screen_switch_data_port.ScreenSwitchDataPort) */ {
 public:
  ScreenSwitchDataPort();
  virtual ~ScreenSwitchDataPort();

  ScreenSwitchDataPort(const ScreenSwitchDataPort& from);
  ScreenSwitchDataPort(ScreenSwitchDataPort&& from) noexcept
    : ScreenSwitchDataPort() {
    *this = ::std::move(from);
  }

  inline ScreenSwitchDataPort& operator=(const ScreenSwitchDataPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline ScreenSwitchDataPort& operator=(ScreenSwitchDataPort&& from) noexcept {
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
  static const ScreenSwitchDataPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ScreenSwitchDataPort* internal_default_instance() {
    return reinterpret_cast<const ScreenSwitchDataPort*>(
               &_ScreenSwitchDataPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ScreenSwitchDataPort& a, ScreenSwitchDataPort& b) {
    a.Swap(&b);
  }
  inline void Swap(ScreenSwitchDataPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ScreenSwitchDataPort* New() const final {
    return CreateMaybeMessage<ScreenSwitchDataPort>(nullptr);
  }

  ScreenSwitchDataPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ScreenSwitchDataPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ScreenSwitchDataPort& from);
  void MergeFrom(const ScreenSwitchDataPort& from);
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
  void InternalSwap(ScreenSwitchDataPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.vc.screen_switch_data_port.ScreenSwitchDataPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto);
    return ::descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kDeactivateViewFieldNumber = 3791,
    kTransparencyPresetFieldNumber = 366,
    kHmiOutUserActScreenReqU8FieldNumber = 626,
    kBlindSpotViewTypeFieldNumber = 883,
    kUiVersionNumberFieldNumber = 2124,
    kCurrentViewModeFieldNumber = 2805,
    kClusterScreenResponseNuU8FieldNumber = 3156,
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

  // optional bool deactivateView = 3791;
  bool has_deactivateview() const;
  private:
  bool _internal_has_deactivateview() const;
  public:
  void clear_deactivateview();
  bool deactivateview() const;
  void set_deactivateview(bool value);
  private:
  bool _internal_deactivateview() const;
  void _internal_set_deactivateview(bool value);
  public:

  // optional .pb.vc.transparency_preset.TransparencyPreset transparencyPreset = 366;
  bool has_transparencypreset() const;
  private:
  bool _internal_has_transparencypreset() const;
  public:
  void clear_transparencypreset();
  ::pb::vc::transparency_preset::TransparencyPreset transparencypreset() const;
  void set_transparencypreset(::pb::vc::transparency_preset::TransparencyPreset value);
  private:
  ::pb::vc::transparency_preset::TransparencyPreset _internal_transparencypreset() const;
  void _internal_set_transparencypreset(::pb::vc::transparency_preset::TransparencyPreset value);
  public:

  // optional .pb.ap_hmitoap.screen_types.ScreenTypes HmiOutUserActScreenReq_u8 = 626;
  bool has_hmioutuseractscreenreq_u8() const;
  private:
  bool _internal_has_hmioutuseractscreenreq_u8() const;
  public:
  void clear_hmioutuseractscreenreq_u8();
  ::pb::ap_hmitoap::screen_types::ScreenTypes hmioutuseractscreenreq_u8() const;
  void set_hmioutuseractscreenreq_u8(::pb::ap_hmitoap::screen_types::ScreenTypes value);
  private:
  ::pb::ap_hmitoap::screen_types::ScreenTypes _internal_hmioutuseractscreenreq_u8() const;
  void _internal_set_hmioutuseractscreenreq_u8(::pb::ap_hmitoap::screen_types::ScreenTypes value);
  public:

  // optional .pb.vc.blind_spot_view_status.BlindSpotViewStatus blindSpotViewType = 883;
  bool has_blindspotviewtype() const;
  private:
  bool _internal_has_blindspotviewtype() const;
  public:
  void clear_blindspotviewtype();
  ::pb::vc::blind_spot_view_status::BlindSpotViewStatus blindspotviewtype() const;
  void set_blindspotviewtype(::pb::vc::blind_spot_view_status::BlindSpotViewStatus value);
  private:
  ::pb::vc::blind_spot_view_status::BlindSpotViewStatus _internal_blindspotviewtype() const;
  void _internal_set_blindspotviewtype(::pb::vc::blind_spot_view_status::BlindSpotViewStatus value);
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

  // optional .pb.ap_hmitoap.screen_types.ScreenTypes currentViewMode = 2805;
  bool has_currentviewmode() const;
  private:
  bool _internal_has_currentviewmode() const;
  public:
  void clear_currentviewmode();
  ::pb::ap_hmitoap::screen_types::ScreenTypes currentviewmode() const;
  void set_currentviewmode(::pb::ap_hmitoap::screen_types::ScreenTypes value);
  private:
  ::pb::ap_hmitoap::screen_types::ScreenTypes _internal_currentviewmode() const;
  void _internal_set_currentviewmode(::pb::ap_hmitoap::screen_types::ScreenTypes value);
  public:

  // optional uint32 ClusterScreenResponse_nu_u8 = 3156;
  bool has_clusterscreenresponse_nu_u8() const;
  private:
  bool _internal_has_clusterscreenresponse_nu_u8() const;
  public:
  void clear_clusterscreenresponse_nu_u8();
  ::PROTOBUF_NAMESPACE_ID::uint32 clusterscreenresponse_nu_u8() const;
  void set_clusterscreenresponse_nu_u8(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_clusterscreenresponse_nu_u8() const;
  void _internal_set_clusterscreenresponse_nu_u8(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  bool deactivateview_;
  int transparencypreset_;
  int hmioutuseractscreenreq_u8_;
  int blindspotviewtype_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  int currentviewmode_;
  ::PROTOBUF_NAMESPACE_ID::uint32 clusterscreenresponse_nu_u8_;
  friend struct ::TableStruct_vc_2fscreen_5fswitch_5fdata_5fport_2eproto;
};
// -------------------------------------------------------------------

class ScreenSwitchDataPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port) */ {
 public:
  ScreenSwitchDataPort_array_port();
  virtual ~ScreenSwitchDataPort_array_port();

  ScreenSwitchDataPort_array_port(const ScreenSwitchDataPort_array_port& from);
  ScreenSwitchDataPort_array_port(ScreenSwitchDataPort_array_port&& from) noexcept
    : ScreenSwitchDataPort_array_port() {
    *this = ::std::move(from);
  }

  inline ScreenSwitchDataPort_array_port& operator=(const ScreenSwitchDataPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline ScreenSwitchDataPort_array_port& operator=(ScreenSwitchDataPort_array_port&& from) noexcept {
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
  static const ScreenSwitchDataPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ScreenSwitchDataPort_array_port* internal_default_instance() {
    return reinterpret_cast<const ScreenSwitchDataPort_array_port*>(
               &_ScreenSwitchDataPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(ScreenSwitchDataPort_array_port& a, ScreenSwitchDataPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(ScreenSwitchDataPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ScreenSwitchDataPort_array_port* New() const final {
    return CreateMaybeMessage<ScreenSwitchDataPort_array_port>(nullptr);
  }

  ScreenSwitchDataPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ScreenSwitchDataPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ScreenSwitchDataPort_array_port& from);
  void MergeFrom(const ScreenSwitchDataPort_array_port& from);
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
  void InternalSwap(ScreenSwitchDataPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto);
    return ::descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3959,
  };
  // repeated .pb.vc.screen_switch_data_port.ScreenSwitchDataPort data = 3959;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort >*
      mutable_data();
  private:
  const ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort& _internal_data(int index) const;
  ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort* _internal_add_data();
  public:
  const ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort& data(int index) const;
  ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort > data_;
  friend struct ::TableStruct_vc_2fscreen_5fswitch_5fdata_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ScreenSwitchDataPort

// optional uint32 uiVersionNumber = 2124;
inline bool ScreenSwitchDataPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool ScreenSwitchDataPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void ScreenSwitchDataPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ScreenSwitchDataPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ScreenSwitchDataPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void ScreenSwitchDataPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  uiversionnumber_ = value;
}
inline void ScreenSwitchDataPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool ScreenSwitchDataPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool ScreenSwitchDataPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& ScreenSwitchDataPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& ScreenSwitchDataPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* ScreenSwitchDataPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* ScreenSwitchDataPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* ScreenSwitchDataPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void ScreenSwitchDataPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.sSigHeader)
}

// optional .pb.ap_hmitoap.screen_types.ScreenTypes HmiOutUserActScreenReq_u8 = 626;
inline bool ScreenSwitchDataPort::_internal_has_hmioutuseractscreenreq_u8() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool ScreenSwitchDataPort::has_hmioutuseractscreenreq_u8() const {
  return _internal_has_hmioutuseractscreenreq_u8();
}
inline void ScreenSwitchDataPort::clear_hmioutuseractscreenreq_u8() {
  hmioutuseractscreenreq_u8_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::pb::ap_hmitoap::screen_types::ScreenTypes ScreenSwitchDataPort::_internal_hmioutuseractscreenreq_u8() const {
  return static_cast< ::pb::ap_hmitoap::screen_types::ScreenTypes >(hmioutuseractscreenreq_u8_);
}
inline ::pb::ap_hmitoap::screen_types::ScreenTypes ScreenSwitchDataPort::hmioutuseractscreenreq_u8() const {
  // @@protoc_insertion_point(field_get:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.HmiOutUserActScreenReq_u8)
  return _internal_hmioutuseractscreenreq_u8();
}
inline void ScreenSwitchDataPort::_internal_set_hmioutuseractscreenreq_u8(::pb::ap_hmitoap::screen_types::ScreenTypes value) {
  assert(::pb::ap_hmitoap::screen_types::ScreenTypes_IsValid(value));
  _has_bits_[0] |= 0x00000008u;
  hmioutuseractscreenreq_u8_ = value;
}
inline void ScreenSwitchDataPort::set_hmioutuseractscreenreq_u8(::pb::ap_hmitoap::screen_types::ScreenTypes value) {
  _internal_set_hmioutuseractscreenreq_u8(value);
  // @@protoc_insertion_point(field_set:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.HmiOutUserActScreenReq_u8)
}

// optional .pb.vc.blind_spot_view_status.BlindSpotViewStatus blindSpotViewType = 883;
inline bool ScreenSwitchDataPort::_internal_has_blindspotviewtype() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool ScreenSwitchDataPort::has_blindspotviewtype() const {
  return _internal_has_blindspotviewtype();
}
inline void ScreenSwitchDataPort::clear_blindspotviewtype() {
  blindspotviewtype_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::pb::vc::blind_spot_view_status::BlindSpotViewStatus ScreenSwitchDataPort::_internal_blindspotviewtype() const {
  return static_cast< ::pb::vc::blind_spot_view_status::BlindSpotViewStatus >(blindspotviewtype_);
}
inline ::pb::vc::blind_spot_view_status::BlindSpotViewStatus ScreenSwitchDataPort::blindspotviewtype() const {
  // @@protoc_insertion_point(field_get:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.blindSpotViewType)
  return _internal_blindspotviewtype();
}
inline void ScreenSwitchDataPort::_internal_set_blindspotviewtype(::pb::vc::blind_spot_view_status::BlindSpotViewStatus value) {
  assert(::pb::vc::blind_spot_view_status::BlindSpotViewStatus_IsValid(value));
  _has_bits_[0] |= 0x00000010u;
  blindspotviewtype_ = value;
}
inline void ScreenSwitchDataPort::set_blindspotviewtype(::pb::vc::blind_spot_view_status::BlindSpotViewStatus value) {
  _internal_set_blindspotviewtype(value);
  // @@protoc_insertion_point(field_set:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.blindSpotViewType)
}

// optional .pb.ap_hmitoap.screen_types.ScreenTypes currentViewMode = 2805;
inline bool ScreenSwitchDataPort::_internal_has_currentviewmode() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool ScreenSwitchDataPort::has_currentviewmode() const {
  return _internal_has_currentviewmode();
}
inline void ScreenSwitchDataPort::clear_currentviewmode() {
  currentviewmode_ = 0;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::pb::ap_hmitoap::screen_types::ScreenTypes ScreenSwitchDataPort::_internal_currentviewmode() const {
  return static_cast< ::pb::ap_hmitoap::screen_types::ScreenTypes >(currentviewmode_);
}
inline ::pb::ap_hmitoap::screen_types::ScreenTypes ScreenSwitchDataPort::currentviewmode() const {
  // @@protoc_insertion_point(field_get:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.currentViewMode)
  return _internal_currentviewmode();
}
inline void ScreenSwitchDataPort::_internal_set_currentviewmode(::pb::ap_hmitoap::screen_types::ScreenTypes value) {
  assert(::pb::ap_hmitoap::screen_types::ScreenTypes_IsValid(value));
  _has_bits_[0] |= 0x00000040u;
  currentviewmode_ = value;
}
inline void ScreenSwitchDataPort::set_currentviewmode(::pb::ap_hmitoap::screen_types::ScreenTypes value) {
  _internal_set_currentviewmode(value);
  // @@protoc_insertion_point(field_set:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.currentViewMode)
}

// optional uint32 ClusterScreenResponse_nu_u8 = 3156;
inline bool ScreenSwitchDataPort::_internal_has_clusterscreenresponse_nu_u8() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool ScreenSwitchDataPort::has_clusterscreenresponse_nu_u8() const {
  return _internal_has_clusterscreenresponse_nu_u8();
}
inline void ScreenSwitchDataPort::clear_clusterscreenresponse_nu_u8() {
  clusterscreenresponse_nu_u8_ = 0u;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ScreenSwitchDataPort::_internal_clusterscreenresponse_nu_u8() const {
  return clusterscreenresponse_nu_u8_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ScreenSwitchDataPort::clusterscreenresponse_nu_u8() const {
  // @@protoc_insertion_point(field_get:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.ClusterScreenResponse_nu_u8)
  return _internal_clusterscreenresponse_nu_u8();
}
inline void ScreenSwitchDataPort::_internal_set_clusterscreenresponse_nu_u8(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000080u;
  clusterscreenresponse_nu_u8_ = value;
}
inline void ScreenSwitchDataPort::set_clusterscreenresponse_nu_u8(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_clusterscreenresponse_nu_u8(value);
  // @@protoc_insertion_point(field_set:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.ClusterScreenResponse_nu_u8)
}

// optional bool deactivateView = 3791;
inline bool ScreenSwitchDataPort::_internal_has_deactivateview() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool ScreenSwitchDataPort::has_deactivateview() const {
  return _internal_has_deactivateview();
}
inline void ScreenSwitchDataPort::clear_deactivateview() {
  deactivateview_ = false;
  _has_bits_[0] &= ~0x00000002u;
}
inline bool ScreenSwitchDataPort::_internal_deactivateview() const {
  return deactivateview_;
}
inline bool ScreenSwitchDataPort::deactivateview() const {
  // @@protoc_insertion_point(field_get:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.deactivateView)
  return _internal_deactivateview();
}
inline void ScreenSwitchDataPort::_internal_set_deactivateview(bool value) {
  _has_bits_[0] |= 0x00000002u;
  deactivateview_ = value;
}
inline void ScreenSwitchDataPort::set_deactivateview(bool value) {
  _internal_set_deactivateview(value);
  // @@protoc_insertion_point(field_set:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.deactivateView)
}

// optional .pb.vc.transparency_preset.TransparencyPreset transparencyPreset = 366;
inline bool ScreenSwitchDataPort::_internal_has_transparencypreset() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool ScreenSwitchDataPort::has_transparencypreset() const {
  return _internal_has_transparencypreset();
}
inline void ScreenSwitchDataPort::clear_transparencypreset() {
  transparencypreset_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::vc::transparency_preset::TransparencyPreset ScreenSwitchDataPort::_internal_transparencypreset() const {
  return static_cast< ::pb::vc::transparency_preset::TransparencyPreset >(transparencypreset_);
}
inline ::pb::vc::transparency_preset::TransparencyPreset ScreenSwitchDataPort::transparencypreset() const {
  // @@protoc_insertion_point(field_get:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.transparencyPreset)
  return _internal_transparencypreset();
}
inline void ScreenSwitchDataPort::_internal_set_transparencypreset(::pb::vc::transparency_preset::TransparencyPreset value) {
  assert(::pb::vc::transparency_preset::TransparencyPreset_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  transparencypreset_ = value;
}
inline void ScreenSwitchDataPort::set_transparencypreset(::pb::vc::transparency_preset::TransparencyPreset value) {
  _internal_set_transparencypreset(value);
  // @@protoc_insertion_point(field_set:pb.vc.screen_switch_data_port.ScreenSwitchDataPort.transparencyPreset)
}

// -------------------------------------------------------------------

// ScreenSwitchDataPort_array_port

// repeated .pb.vc.screen_switch_data_port.ScreenSwitchDataPort data = 3959;
inline int ScreenSwitchDataPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int ScreenSwitchDataPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void ScreenSwitchDataPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort* ScreenSwitchDataPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort >*
ScreenSwitchDataPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port.data)
  return &data_;
}
inline const ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort& ScreenSwitchDataPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort& ScreenSwitchDataPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort* ScreenSwitchDataPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort* ScreenSwitchDataPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort >&
ScreenSwitchDataPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace screen_switch_data_port
}  // namespace vc
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_vc_2fscreen_5fswitch_5fdata_5fport_2eproto
