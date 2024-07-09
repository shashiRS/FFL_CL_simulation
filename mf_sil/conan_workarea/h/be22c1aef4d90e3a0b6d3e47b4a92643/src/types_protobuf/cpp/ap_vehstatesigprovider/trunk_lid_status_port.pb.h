// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/trunk_lid_status_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2ftrunk_5flid_5fstatus_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2ftrunk_5flid_5fstatus_5fport_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2ftrunk_5flid_5fstatus_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fvehstatesigprovider_2ftrunk_5flid_5fstatus_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2ftrunk_5flid_5fstatus_5fport_2eproto;
namespace pb {
namespace ap_vehstatesigprovider {
namespace trunk_lid_status_port {
class TrunkLidStatusPort;
class TrunkLidStatusPortDefaultTypeInternal;
extern TrunkLidStatusPortDefaultTypeInternal _TrunkLidStatusPort_default_instance_;
class TrunkLidStatusPort_array_port;
class TrunkLidStatusPort_array_portDefaultTypeInternal;
extern TrunkLidStatusPort_array_portDefaultTypeInternal _TrunkLidStatusPort_array_port_default_instance_;
}  // namespace trunk_lid_status_port
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort>(Arena*);
template<> ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort_array_port* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_vehstatesigprovider {
namespace trunk_lid_status_port {

// ===================================================================

class TrunkLidStatusPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort) */ {
 public:
  TrunkLidStatusPort();
  virtual ~TrunkLidStatusPort();

  TrunkLidStatusPort(const TrunkLidStatusPort& from);
  TrunkLidStatusPort(TrunkLidStatusPort&& from) noexcept
    : TrunkLidStatusPort() {
    *this = ::std::move(from);
  }

  inline TrunkLidStatusPort& operator=(const TrunkLidStatusPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline TrunkLidStatusPort& operator=(TrunkLidStatusPort&& from) noexcept {
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
  static const TrunkLidStatusPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TrunkLidStatusPort* internal_default_instance() {
    return reinterpret_cast<const TrunkLidStatusPort*>(
               &_TrunkLidStatusPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(TrunkLidStatusPort& a, TrunkLidStatusPort& b) {
    a.Swap(&b);
  }
  inline void Swap(TrunkLidStatusPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline TrunkLidStatusPort* New() const final {
    return CreateMaybeMessage<TrunkLidStatusPort>(nullptr);
  }

  TrunkLidStatusPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<TrunkLidStatusPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const TrunkLidStatusPort& from);
  void MergeFrom(const TrunkLidStatusPort& from);
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
  void InternalSwap(TrunkLidStatusPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2ftrunk_5flid_5fstatus_5fport_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2ftrunk_5flid_5fstatus_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kUiVersionNumberFieldNumber = 2124,
    kOpenNuFieldNumber = 3468,
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

  // optional bool open_nu = 3468;
  bool has_open_nu() const;
  private:
  bool _internal_has_open_nu() const;
  public:
  void clear_open_nu();
  bool open_nu() const;
  void set_open_nu(bool value);
  private:
  bool _internal_open_nu() const;
  void _internal_set_open_nu(bool value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  bool open_nu_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2ftrunk_5flid_5fstatus_5fport_2eproto;
};
// -------------------------------------------------------------------

class TrunkLidStatusPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort_array_port) */ {
 public:
  TrunkLidStatusPort_array_port();
  virtual ~TrunkLidStatusPort_array_port();

  TrunkLidStatusPort_array_port(const TrunkLidStatusPort_array_port& from);
  TrunkLidStatusPort_array_port(TrunkLidStatusPort_array_port&& from) noexcept
    : TrunkLidStatusPort_array_port() {
    *this = ::std::move(from);
  }

  inline TrunkLidStatusPort_array_port& operator=(const TrunkLidStatusPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline TrunkLidStatusPort_array_port& operator=(TrunkLidStatusPort_array_port&& from) noexcept {
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
  static const TrunkLidStatusPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TrunkLidStatusPort_array_port* internal_default_instance() {
    return reinterpret_cast<const TrunkLidStatusPort_array_port*>(
               &_TrunkLidStatusPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(TrunkLidStatusPort_array_port& a, TrunkLidStatusPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(TrunkLidStatusPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline TrunkLidStatusPort_array_port* New() const final {
    return CreateMaybeMessage<TrunkLidStatusPort_array_port>(nullptr);
  }

  TrunkLidStatusPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<TrunkLidStatusPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const TrunkLidStatusPort_array_port& from);
  void MergeFrom(const TrunkLidStatusPort_array_port& from);
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
  void InternalSwap(TrunkLidStatusPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2ftrunk_5flid_5fstatus_5fport_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2ftrunk_5flid_5fstatus_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1045,
  };
  // repeated .pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort data = 1045;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort >*
      mutable_data();
  private:
  const ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort& _internal_data(int index) const;
  ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort* _internal_add_data();
  public:
  const ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort& data(int index) const;
  ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort > data_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2ftrunk_5flid_5fstatus_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// TrunkLidStatusPort

// optional uint32 uiVersionNumber = 2124;
inline bool TrunkLidStatusPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool TrunkLidStatusPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void TrunkLidStatusPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TrunkLidStatusPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TrunkLidStatusPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void TrunkLidStatusPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  uiversionnumber_ = value;
}
inline void TrunkLidStatusPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool TrunkLidStatusPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool TrunkLidStatusPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& TrunkLidStatusPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& TrunkLidStatusPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* TrunkLidStatusPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* TrunkLidStatusPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* TrunkLidStatusPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void TrunkLidStatusPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort.sSigHeader)
}

// optional bool open_nu = 3468;
inline bool TrunkLidStatusPort::_internal_has_open_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool TrunkLidStatusPort::has_open_nu() const {
  return _internal_has_open_nu();
}
inline void TrunkLidStatusPort::clear_open_nu() {
  open_nu_ = false;
  _has_bits_[0] &= ~0x00000004u;
}
inline bool TrunkLidStatusPort::_internal_open_nu() const {
  return open_nu_;
}
inline bool TrunkLidStatusPort::open_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort.open_nu)
  return _internal_open_nu();
}
inline void TrunkLidStatusPort::_internal_set_open_nu(bool value) {
  _has_bits_[0] |= 0x00000004u;
  open_nu_ = value;
}
inline void TrunkLidStatusPort::set_open_nu(bool value) {
  _internal_set_open_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort.open_nu)
}

// -------------------------------------------------------------------

// TrunkLidStatusPort_array_port

// repeated .pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort data = 1045;
inline int TrunkLidStatusPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int TrunkLidStatusPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void TrunkLidStatusPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort* TrunkLidStatusPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort >*
TrunkLidStatusPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort_array_port.data)
  return &data_;
}
inline const ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort& TrunkLidStatusPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort& TrunkLidStatusPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort* TrunkLidStatusPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort* TrunkLidStatusPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::trunk_lid_status_port::TrunkLidStatusPort >&
TrunkLidStatusPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_vehstatesigprovider.trunk_lid_status_port.TrunkLidStatusPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace trunk_lid_status_port
}  // namespace ap_vehstatesigprovider
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2ftrunk_5flid_5fstatus_5fport_2eproto
