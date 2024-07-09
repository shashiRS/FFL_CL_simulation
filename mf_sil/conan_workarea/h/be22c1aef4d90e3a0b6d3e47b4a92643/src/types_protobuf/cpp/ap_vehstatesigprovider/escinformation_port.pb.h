// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/escinformation_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fescinformation_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fescinformation_5fport_2eproto

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
#include "ap_vehstatesigprovider/tcsstate.pb.h"
#include "ap_vehstatesigprovider/escstate.pb.h"
#include "ap_vehstatesigprovider/absstate.pb.h"
#include "ap_vehstatesigprovider/ebdstate.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2fescinformation_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fvehstatesigprovider_2fescinformation_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2fescinformation_5fport_2eproto;
namespace pb {
namespace ap_vehstatesigprovider {
namespace escinformation_port {
class ESCInformationPort;
class ESCInformationPortDefaultTypeInternal;
extern ESCInformationPortDefaultTypeInternal _ESCInformationPort_default_instance_;
class ESCInformationPort_array_port;
class ESCInformationPort_array_portDefaultTypeInternal;
extern ESCInformationPort_array_portDefaultTypeInternal _ESCInformationPort_array_port_default_instance_;
}  // namespace escinformation_port
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort>(Arena*);
template<> ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort_array_port* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_vehstatesigprovider {
namespace escinformation_port {

// ===================================================================

class ESCInformationPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort) */ {
 public:
  ESCInformationPort();
  virtual ~ESCInformationPort();

  ESCInformationPort(const ESCInformationPort& from);
  ESCInformationPort(ESCInformationPort&& from) noexcept
    : ESCInformationPort() {
    *this = ::std::move(from);
  }

  inline ESCInformationPort& operator=(const ESCInformationPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline ESCInformationPort& operator=(ESCInformationPort&& from) noexcept {
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
  static const ESCInformationPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ESCInformationPort* internal_default_instance() {
    return reinterpret_cast<const ESCInformationPort*>(
               &_ESCInformationPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ESCInformationPort& a, ESCInformationPort& b) {
    a.Swap(&b);
  }
  inline void Swap(ESCInformationPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ESCInformationPort* New() const final {
    return CreateMaybeMessage<ESCInformationPort>(nullptr);
  }

  ESCInformationPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ESCInformationPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ESCInformationPort& from);
  void MergeFrom(const ESCInformationPort& from);
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
  void InternalSwap(ESCInformationPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fescinformation_5fport_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fescinformation_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kTcsStateNuFieldNumber = 3766,
    kBrakePressureGradientBarpsFieldNumber = 677,
    kBrakePressureDriverBarFieldNumber = 785,
    kAbsStateNuFieldNumber = 1013,
    kUiVersionNumberFieldNumber = 2124,
    kEscStateNuFieldNumber = 2242,
    kEbdStateNuFieldNumber = 2953,
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

  // optional .pb.ap_vehstatesigprovider.tcsstate.TCSState tcsState_nu = 3766;
  bool has_tcsstate_nu() const;
  private:
  bool _internal_has_tcsstate_nu() const;
  public:
  void clear_tcsstate_nu();
  ::pb::ap_vehstatesigprovider::tcsstate::TCSState tcsstate_nu() const;
  void set_tcsstate_nu(::pb::ap_vehstatesigprovider::tcsstate::TCSState value);
  private:
  ::pb::ap_vehstatesigprovider::tcsstate::TCSState _internal_tcsstate_nu() const;
  void _internal_set_tcsstate_nu(::pb::ap_vehstatesigprovider::tcsstate::TCSState value);
  public:

  // optional float brakePressureGradient_barps = 677;
  bool has_brakepressuregradient_barps() const;
  private:
  bool _internal_has_brakepressuregradient_barps() const;
  public:
  void clear_brakepressuregradient_barps();
  float brakepressuregradient_barps() const;
  void set_brakepressuregradient_barps(float value);
  private:
  float _internal_brakepressuregradient_barps() const;
  void _internal_set_brakepressuregradient_barps(float value);
  public:

  // optional float brakePressureDriver_bar = 785;
  bool has_brakepressuredriver_bar() const;
  private:
  bool _internal_has_brakepressuredriver_bar() const;
  public:
  void clear_brakepressuredriver_bar();
  float brakepressuredriver_bar() const;
  void set_brakepressuredriver_bar(float value);
  private:
  float _internal_brakepressuredriver_bar() const;
  void _internal_set_brakepressuredriver_bar(float value);
  public:

  // optional .pb.ap_vehstatesigprovider.absstate.ABSState absState_nu = 1013;
  bool has_absstate_nu() const;
  private:
  bool _internal_has_absstate_nu() const;
  public:
  void clear_absstate_nu();
  ::pb::ap_vehstatesigprovider::absstate::ABSState absstate_nu() const;
  void set_absstate_nu(::pb::ap_vehstatesigprovider::absstate::ABSState value);
  private:
  ::pb::ap_vehstatesigprovider::absstate::ABSState _internal_absstate_nu() const;
  void _internal_set_absstate_nu(::pb::ap_vehstatesigprovider::absstate::ABSState value);
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

  // optional .pb.ap_vehstatesigprovider.escstate.ESCState escState_nu = 2242;
  bool has_escstate_nu() const;
  private:
  bool _internal_has_escstate_nu() const;
  public:
  void clear_escstate_nu();
  ::pb::ap_vehstatesigprovider::escstate::ESCState escstate_nu() const;
  void set_escstate_nu(::pb::ap_vehstatesigprovider::escstate::ESCState value);
  private:
  ::pb::ap_vehstatesigprovider::escstate::ESCState _internal_escstate_nu() const;
  void _internal_set_escstate_nu(::pb::ap_vehstatesigprovider::escstate::ESCState value);
  public:

  // optional .pb.ap_vehstatesigprovider.ebdstate.EBDState ebdState_nu = 2953;
  bool has_ebdstate_nu() const;
  private:
  bool _internal_has_ebdstate_nu() const;
  public:
  void clear_ebdstate_nu();
  ::pb::ap_vehstatesigprovider::ebdstate::EBDState ebdstate_nu() const;
  void set_ebdstate_nu(::pb::ap_vehstatesigprovider::ebdstate::EBDState value);
  private:
  ::pb::ap_vehstatesigprovider::ebdstate::EBDState _internal_ebdstate_nu() const;
  void _internal_set_ebdstate_nu(::pb::ap_vehstatesigprovider::ebdstate::EBDState value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  int tcsstate_nu_;
  float brakepressuregradient_barps_;
  float brakepressuredriver_bar_;
  int absstate_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  int escstate_nu_;
  int ebdstate_nu_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fescinformation_5fport_2eproto;
};
// -------------------------------------------------------------------

class ESCInformationPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort_array_port) */ {
 public:
  ESCInformationPort_array_port();
  virtual ~ESCInformationPort_array_port();

  ESCInformationPort_array_port(const ESCInformationPort_array_port& from);
  ESCInformationPort_array_port(ESCInformationPort_array_port&& from) noexcept
    : ESCInformationPort_array_port() {
    *this = ::std::move(from);
  }

  inline ESCInformationPort_array_port& operator=(const ESCInformationPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline ESCInformationPort_array_port& operator=(ESCInformationPort_array_port&& from) noexcept {
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
  static const ESCInformationPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ESCInformationPort_array_port* internal_default_instance() {
    return reinterpret_cast<const ESCInformationPort_array_port*>(
               &_ESCInformationPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(ESCInformationPort_array_port& a, ESCInformationPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(ESCInformationPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ESCInformationPort_array_port* New() const final {
    return CreateMaybeMessage<ESCInformationPort_array_port>(nullptr);
  }

  ESCInformationPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ESCInformationPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ESCInformationPort_array_port& from);
  void MergeFrom(const ESCInformationPort_array_port& from);
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
  void InternalSwap(ESCInformationPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fescinformation_5fport_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fescinformation_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2763,
  };
  // repeated .pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort data = 2763;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort >*
      mutable_data();
  private:
  const ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort& _internal_data(int index) const;
  ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort* _internal_add_data();
  public:
  const ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort& data(int index) const;
  ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort > data_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fescinformation_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ESCInformationPort

// optional uint32 uiVersionNumber = 2124;
inline bool ESCInformationPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool ESCInformationPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void ESCInformationPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ESCInformationPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ESCInformationPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void ESCInformationPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  uiversionnumber_ = value;
}
inline void ESCInformationPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool ESCInformationPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool ESCInformationPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& ESCInformationPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& ESCInformationPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* ESCInformationPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* ESCInformationPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* ESCInformationPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void ESCInformationPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.sSigHeader)
}

// optional float brakePressureGradient_barps = 677;
inline bool ESCInformationPort::_internal_has_brakepressuregradient_barps() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool ESCInformationPort::has_brakepressuregradient_barps() const {
  return _internal_has_brakepressuregradient_barps();
}
inline void ESCInformationPort::clear_brakepressuregradient_barps() {
  brakepressuregradient_barps_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float ESCInformationPort::_internal_brakepressuregradient_barps() const {
  return brakepressuregradient_barps_;
}
inline float ESCInformationPort::brakepressuregradient_barps() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.brakePressureGradient_barps)
  return _internal_brakepressuregradient_barps();
}
inline void ESCInformationPort::_internal_set_brakepressuregradient_barps(float value) {
  _has_bits_[0] |= 0x00000004u;
  brakepressuregradient_barps_ = value;
}
inline void ESCInformationPort::set_brakepressuregradient_barps(float value) {
  _internal_set_brakepressuregradient_barps(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.brakePressureGradient_barps)
}

// optional float brakePressureDriver_bar = 785;
inline bool ESCInformationPort::_internal_has_brakepressuredriver_bar() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool ESCInformationPort::has_brakepressuredriver_bar() const {
  return _internal_has_brakepressuredriver_bar();
}
inline void ESCInformationPort::clear_brakepressuredriver_bar() {
  brakepressuredriver_bar_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float ESCInformationPort::_internal_brakepressuredriver_bar() const {
  return brakepressuredriver_bar_;
}
inline float ESCInformationPort::brakepressuredriver_bar() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.brakePressureDriver_bar)
  return _internal_brakepressuredriver_bar();
}
inline void ESCInformationPort::_internal_set_brakepressuredriver_bar(float value) {
  _has_bits_[0] |= 0x00000008u;
  brakepressuredriver_bar_ = value;
}
inline void ESCInformationPort::set_brakepressuredriver_bar(float value) {
  _internal_set_brakepressuredriver_bar(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.brakePressureDriver_bar)
}

// optional .pb.ap_vehstatesigprovider.tcsstate.TCSState tcsState_nu = 3766;
inline bool ESCInformationPort::_internal_has_tcsstate_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool ESCInformationPort::has_tcsstate_nu() const {
  return _internal_has_tcsstate_nu();
}
inline void ESCInformationPort::clear_tcsstate_nu() {
  tcsstate_nu_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::pb::ap_vehstatesigprovider::tcsstate::TCSState ESCInformationPort::_internal_tcsstate_nu() const {
  return static_cast< ::pb::ap_vehstatesigprovider::tcsstate::TCSState >(tcsstate_nu_);
}
inline ::pb::ap_vehstatesigprovider::tcsstate::TCSState ESCInformationPort::tcsstate_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.tcsState_nu)
  return _internal_tcsstate_nu();
}
inline void ESCInformationPort::_internal_set_tcsstate_nu(::pb::ap_vehstatesigprovider::tcsstate::TCSState value) {
  assert(::pb::ap_vehstatesigprovider::tcsstate::TCSState_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  tcsstate_nu_ = value;
}
inline void ESCInformationPort::set_tcsstate_nu(::pb::ap_vehstatesigprovider::tcsstate::TCSState value) {
  _internal_set_tcsstate_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.tcsState_nu)
}

// optional .pb.ap_vehstatesigprovider.escstate.ESCState escState_nu = 2242;
inline bool ESCInformationPort::_internal_has_escstate_nu() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool ESCInformationPort::has_escstate_nu() const {
  return _internal_has_escstate_nu();
}
inline void ESCInformationPort::clear_escstate_nu() {
  escstate_nu_ = 0;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::pb::ap_vehstatesigprovider::escstate::ESCState ESCInformationPort::_internal_escstate_nu() const {
  return static_cast< ::pb::ap_vehstatesigprovider::escstate::ESCState >(escstate_nu_);
}
inline ::pb::ap_vehstatesigprovider::escstate::ESCState ESCInformationPort::escstate_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.escState_nu)
  return _internal_escstate_nu();
}
inline void ESCInformationPort::_internal_set_escstate_nu(::pb::ap_vehstatesigprovider::escstate::ESCState value) {
  assert(::pb::ap_vehstatesigprovider::escstate::ESCState_IsValid(value));
  _has_bits_[0] |= 0x00000040u;
  escstate_nu_ = value;
}
inline void ESCInformationPort::set_escstate_nu(::pb::ap_vehstatesigprovider::escstate::ESCState value) {
  _internal_set_escstate_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.escState_nu)
}

// optional .pb.ap_vehstatesigprovider.absstate.ABSState absState_nu = 1013;
inline bool ESCInformationPort::_internal_has_absstate_nu() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool ESCInformationPort::has_absstate_nu() const {
  return _internal_has_absstate_nu();
}
inline void ESCInformationPort::clear_absstate_nu() {
  absstate_nu_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::pb::ap_vehstatesigprovider::absstate::ABSState ESCInformationPort::_internal_absstate_nu() const {
  return static_cast< ::pb::ap_vehstatesigprovider::absstate::ABSState >(absstate_nu_);
}
inline ::pb::ap_vehstatesigprovider::absstate::ABSState ESCInformationPort::absstate_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.absState_nu)
  return _internal_absstate_nu();
}
inline void ESCInformationPort::_internal_set_absstate_nu(::pb::ap_vehstatesigprovider::absstate::ABSState value) {
  assert(::pb::ap_vehstatesigprovider::absstate::ABSState_IsValid(value));
  _has_bits_[0] |= 0x00000010u;
  absstate_nu_ = value;
}
inline void ESCInformationPort::set_absstate_nu(::pb::ap_vehstatesigprovider::absstate::ABSState value) {
  _internal_set_absstate_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.absState_nu)
}

// optional .pb.ap_vehstatesigprovider.ebdstate.EBDState ebdState_nu = 2953;
inline bool ESCInformationPort::_internal_has_ebdstate_nu() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool ESCInformationPort::has_ebdstate_nu() const {
  return _internal_has_ebdstate_nu();
}
inline void ESCInformationPort::clear_ebdstate_nu() {
  ebdstate_nu_ = 0;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::pb::ap_vehstatesigprovider::ebdstate::EBDState ESCInformationPort::_internal_ebdstate_nu() const {
  return static_cast< ::pb::ap_vehstatesigprovider::ebdstate::EBDState >(ebdstate_nu_);
}
inline ::pb::ap_vehstatesigprovider::ebdstate::EBDState ESCInformationPort::ebdstate_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.ebdState_nu)
  return _internal_ebdstate_nu();
}
inline void ESCInformationPort::_internal_set_ebdstate_nu(::pb::ap_vehstatesigprovider::ebdstate::EBDState value) {
  assert(::pb::ap_vehstatesigprovider::ebdstate::EBDState_IsValid(value));
  _has_bits_[0] |= 0x00000080u;
  ebdstate_nu_ = value;
}
inline void ESCInformationPort::set_ebdstate_nu(::pb::ap_vehstatesigprovider::ebdstate::EBDState value) {
  _internal_set_ebdstate_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort.ebdState_nu)
}

// -------------------------------------------------------------------

// ESCInformationPort_array_port

// repeated .pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort data = 2763;
inline int ESCInformationPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int ESCInformationPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void ESCInformationPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort* ESCInformationPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort >*
ESCInformationPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort_array_port.data)
  return &data_;
}
inline const ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort& ESCInformationPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort& ESCInformationPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort* ESCInformationPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort* ESCInformationPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::escinformation_port::ESCInformationPort >&
ESCInformationPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_vehstatesigprovider.escinformation_port.ESCInformationPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace escinformation_port
}  // namespace ap_vehstatesigprovider
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fescinformation_5fport_2eproto