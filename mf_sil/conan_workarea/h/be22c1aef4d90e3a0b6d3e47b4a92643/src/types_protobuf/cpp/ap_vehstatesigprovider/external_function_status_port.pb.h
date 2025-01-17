// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/external_function_status_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fexternal_5ffunction_5fstatus_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fexternal_5ffunction_5fstatus_5fport_2eproto

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
#include "ap_vehstatesigprovider/rctastatus.pb.h"
#include "ap_vehstatesigprovider/ebastatus.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2fexternal_5ffunction_5fstatus_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fvehstatesigprovider_2fexternal_5ffunction_5fstatus_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2fexternal_5ffunction_5fstatus_5fport_2eproto;
namespace pb {
namespace ap_vehstatesigprovider {
namespace external_function_status_port {
class ExternalFunctionStatusPort;
class ExternalFunctionStatusPortDefaultTypeInternal;
extern ExternalFunctionStatusPortDefaultTypeInternal _ExternalFunctionStatusPort_default_instance_;
class ExternalFunctionStatusPort_array_port;
class ExternalFunctionStatusPort_array_portDefaultTypeInternal;
extern ExternalFunctionStatusPort_array_portDefaultTypeInternal _ExternalFunctionStatusPort_array_port_default_instance_;
}  // namespace external_function_status_port
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort>(Arena*);
template<> ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort_array_port* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_vehstatesigprovider {
namespace external_function_status_port {

// ===================================================================

class ExternalFunctionStatusPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort) */ {
 public:
  ExternalFunctionStatusPort();
  virtual ~ExternalFunctionStatusPort();

  ExternalFunctionStatusPort(const ExternalFunctionStatusPort& from);
  ExternalFunctionStatusPort(ExternalFunctionStatusPort&& from) noexcept
    : ExternalFunctionStatusPort() {
    *this = ::std::move(from);
  }

  inline ExternalFunctionStatusPort& operator=(const ExternalFunctionStatusPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline ExternalFunctionStatusPort& operator=(ExternalFunctionStatusPort&& from) noexcept {
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
  static const ExternalFunctionStatusPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ExternalFunctionStatusPort* internal_default_instance() {
    return reinterpret_cast<const ExternalFunctionStatusPort*>(
               &_ExternalFunctionStatusPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ExternalFunctionStatusPort& a, ExternalFunctionStatusPort& b) {
    a.Swap(&b);
  }
  inline void Swap(ExternalFunctionStatusPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ExternalFunctionStatusPort* New() const final {
    return CreateMaybeMessage<ExternalFunctionStatusPort>(nullptr);
  }

  ExternalFunctionStatusPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ExternalFunctionStatusPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ExternalFunctionStatusPort& from);
  void MergeFrom(const ExternalFunctionStatusPort& from);
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
  void InternalSwap(ExternalFunctionStatusPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fexternal_5ffunction_5fstatus_5fport_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fexternal_5ffunction_5fstatus_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kEbaStatusNuFieldNumber = 3337,
    kRctaStatusNuFieldNumber = 1866,
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

  // optional .pb.ap_vehstatesigprovider.ebastatus.EBAStatus ebaStatus_nu = 3337;
  bool has_ebastatus_nu() const;
  private:
  bool _internal_has_ebastatus_nu() const;
  public:
  void clear_ebastatus_nu();
  ::pb::ap_vehstatesigprovider::ebastatus::EBAStatus ebastatus_nu() const;
  void set_ebastatus_nu(::pb::ap_vehstatesigprovider::ebastatus::EBAStatus value);
  private:
  ::pb::ap_vehstatesigprovider::ebastatus::EBAStatus _internal_ebastatus_nu() const;
  void _internal_set_ebastatus_nu(::pb::ap_vehstatesigprovider::ebastatus::EBAStatus value);
  public:

  // optional .pb.ap_vehstatesigprovider.rctastatus.RCTAStatus rctaStatus_nu = 1866;
  bool has_rctastatus_nu() const;
  private:
  bool _internal_has_rctastatus_nu() const;
  public:
  void clear_rctastatus_nu();
  ::pb::ap_vehstatesigprovider::rctastatus::RCTAStatus rctastatus_nu() const;
  void set_rctastatus_nu(::pb::ap_vehstatesigprovider::rctastatus::RCTAStatus value);
  private:
  ::pb::ap_vehstatesigprovider::rctastatus::RCTAStatus _internal_rctastatus_nu() const;
  void _internal_set_rctastatus_nu(::pb::ap_vehstatesigprovider::rctastatus::RCTAStatus value);
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

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  int ebastatus_nu_;
  int rctastatus_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fexternal_5ffunction_5fstatus_5fport_2eproto;
};
// -------------------------------------------------------------------

class ExternalFunctionStatusPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort_array_port) */ {
 public:
  ExternalFunctionStatusPort_array_port();
  virtual ~ExternalFunctionStatusPort_array_port();

  ExternalFunctionStatusPort_array_port(const ExternalFunctionStatusPort_array_port& from);
  ExternalFunctionStatusPort_array_port(ExternalFunctionStatusPort_array_port&& from) noexcept
    : ExternalFunctionStatusPort_array_port() {
    *this = ::std::move(from);
  }

  inline ExternalFunctionStatusPort_array_port& operator=(const ExternalFunctionStatusPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline ExternalFunctionStatusPort_array_port& operator=(ExternalFunctionStatusPort_array_port&& from) noexcept {
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
  static const ExternalFunctionStatusPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ExternalFunctionStatusPort_array_port* internal_default_instance() {
    return reinterpret_cast<const ExternalFunctionStatusPort_array_port*>(
               &_ExternalFunctionStatusPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(ExternalFunctionStatusPort_array_port& a, ExternalFunctionStatusPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(ExternalFunctionStatusPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ExternalFunctionStatusPort_array_port* New() const final {
    return CreateMaybeMessage<ExternalFunctionStatusPort_array_port>(nullptr);
  }

  ExternalFunctionStatusPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ExternalFunctionStatusPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ExternalFunctionStatusPort_array_port& from);
  void MergeFrom(const ExternalFunctionStatusPort_array_port& from);
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
  void InternalSwap(ExternalFunctionStatusPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fexternal_5ffunction_5fstatus_5fport_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fexternal_5ffunction_5fstatus_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3336,
  };
  // repeated .pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort data = 3336;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort >*
      mutable_data();
  private:
  const ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort& _internal_data(int index) const;
  ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort* _internal_add_data();
  public:
  const ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort& data(int index) const;
  ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort > data_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fexternal_5ffunction_5fstatus_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ExternalFunctionStatusPort

// optional uint32 uiVersionNumber = 2124;
inline bool ExternalFunctionStatusPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool ExternalFunctionStatusPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void ExternalFunctionStatusPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ExternalFunctionStatusPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ExternalFunctionStatusPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void ExternalFunctionStatusPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  uiversionnumber_ = value;
}
inline void ExternalFunctionStatusPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool ExternalFunctionStatusPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool ExternalFunctionStatusPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& ExternalFunctionStatusPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& ExternalFunctionStatusPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* ExternalFunctionStatusPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* ExternalFunctionStatusPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* ExternalFunctionStatusPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void ExternalFunctionStatusPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort.sSigHeader)
}

// optional .pb.ap_vehstatesigprovider.rctastatus.RCTAStatus rctaStatus_nu = 1866;
inline bool ExternalFunctionStatusPort::_internal_has_rctastatus_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool ExternalFunctionStatusPort::has_rctastatus_nu() const {
  return _internal_has_rctastatus_nu();
}
inline void ExternalFunctionStatusPort::clear_rctastatus_nu() {
  rctastatus_nu_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::ap_vehstatesigprovider::rctastatus::RCTAStatus ExternalFunctionStatusPort::_internal_rctastatus_nu() const {
  return static_cast< ::pb::ap_vehstatesigprovider::rctastatus::RCTAStatus >(rctastatus_nu_);
}
inline ::pb::ap_vehstatesigprovider::rctastatus::RCTAStatus ExternalFunctionStatusPort::rctastatus_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort.rctaStatus_nu)
  return _internal_rctastatus_nu();
}
inline void ExternalFunctionStatusPort::_internal_set_rctastatus_nu(::pb::ap_vehstatesigprovider::rctastatus::RCTAStatus value) {
  assert(::pb::ap_vehstatesigprovider::rctastatus::RCTAStatus_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  rctastatus_nu_ = value;
}
inline void ExternalFunctionStatusPort::set_rctastatus_nu(::pb::ap_vehstatesigprovider::rctastatus::RCTAStatus value) {
  _internal_set_rctastatus_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort.rctaStatus_nu)
}

// optional .pb.ap_vehstatesigprovider.ebastatus.EBAStatus ebaStatus_nu = 3337;
inline bool ExternalFunctionStatusPort::_internal_has_ebastatus_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool ExternalFunctionStatusPort::has_ebastatus_nu() const {
  return _internal_has_ebastatus_nu();
}
inline void ExternalFunctionStatusPort::clear_ebastatus_nu() {
  ebastatus_nu_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::pb::ap_vehstatesigprovider::ebastatus::EBAStatus ExternalFunctionStatusPort::_internal_ebastatus_nu() const {
  return static_cast< ::pb::ap_vehstatesigprovider::ebastatus::EBAStatus >(ebastatus_nu_);
}
inline ::pb::ap_vehstatesigprovider::ebastatus::EBAStatus ExternalFunctionStatusPort::ebastatus_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort.ebaStatus_nu)
  return _internal_ebastatus_nu();
}
inline void ExternalFunctionStatusPort::_internal_set_ebastatus_nu(::pb::ap_vehstatesigprovider::ebastatus::EBAStatus value) {
  assert(::pb::ap_vehstatesigprovider::ebastatus::EBAStatus_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  ebastatus_nu_ = value;
}
inline void ExternalFunctionStatusPort::set_ebastatus_nu(::pb::ap_vehstatesigprovider::ebastatus::EBAStatus value) {
  _internal_set_ebastatus_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort.ebaStatus_nu)
}

// -------------------------------------------------------------------

// ExternalFunctionStatusPort_array_port

// repeated .pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort data = 3336;
inline int ExternalFunctionStatusPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int ExternalFunctionStatusPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void ExternalFunctionStatusPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort* ExternalFunctionStatusPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort >*
ExternalFunctionStatusPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort_array_port.data)
  return &data_;
}
inline const ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort& ExternalFunctionStatusPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort& ExternalFunctionStatusPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort* ExternalFunctionStatusPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort* ExternalFunctionStatusPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::external_function_status_port::ExternalFunctionStatusPort >&
ExternalFunctionStatusPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_vehstatesigprovider.external_function_status_port.ExternalFunctionStatusPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace external_function_status_port
}  // namespace ap_vehstatesigprovider
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fexternal_5ffunction_5fstatus_5fport_2eproto
