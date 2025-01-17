// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_lsca/lsca_status_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5flsca_2flsca_5fstatus_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5flsca_2flsca_5fstatus_5fport_2eproto

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
#include "mf_lsca/lsca_state.pb.h"
#include "mf_lsca/lsca_mode.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5flsca_2flsca_5fstatus_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5flsca_2flsca_5fstatus_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5flsca_2flsca_5fstatus_5fport_2eproto;
namespace pb {
namespace mf_lsca {
namespace lsca_status_port {
class LscaStatusPort;
class LscaStatusPortDefaultTypeInternal;
extern LscaStatusPortDefaultTypeInternal _LscaStatusPort_default_instance_;
class LscaStatusPort_array_port;
class LscaStatusPort_array_portDefaultTypeInternal;
extern LscaStatusPort_array_portDefaultTypeInternal _LscaStatusPort_array_port_default_instance_;
}  // namespace lsca_status_port
}  // namespace mf_lsca
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_lsca::lsca_status_port::LscaStatusPort* Arena::CreateMaybeMessage<::pb::mf_lsca::lsca_status_port::LscaStatusPort>(Arena*);
template<> ::pb::mf_lsca::lsca_status_port::LscaStatusPort_array_port* Arena::CreateMaybeMessage<::pb::mf_lsca::lsca_status_port::LscaStatusPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_lsca {
namespace lsca_status_port {

// ===================================================================

class LscaStatusPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_lsca.lsca_status_port.LscaStatusPort) */ {
 public:
  LscaStatusPort();
  virtual ~LscaStatusPort();

  LscaStatusPort(const LscaStatusPort& from);
  LscaStatusPort(LscaStatusPort&& from) noexcept
    : LscaStatusPort() {
    *this = ::std::move(from);
  }

  inline LscaStatusPort& operator=(const LscaStatusPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline LscaStatusPort& operator=(LscaStatusPort&& from) noexcept {
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
  static const LscaStatusPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LscaStatusPort* internal_default_instance() {
    return reinterpret_cast<const LscaStatusPort*>(
               &_LscaStatusPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LscaStatusPort& a, LscaStatusPort& b) {
    a.Swap(&b);
  }
  inline void Swap(LscaStatusPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LscaStatusPort* New() const final {
    return CreateMaybeMessage<LscaStatusPort>(nullptr);
  }

  LscaStatusPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LscaStatusPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LscaStatusPort& from);
  void MergeFrom(const LscaStatusPort& from);
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
  void InternalSwap(LscaStatusPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_lsca.lsca_status_port.LscaStatusPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5flsca_2flsca_5fstatus_5fport_2eproto);
    return ::descriptor_table_mf_5flsca_2flsca_5fstatus_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kSteeringResistanceModuleStateNuFieldNumber = 1272,
    kPmpModuleStateNuFieldNumber = 1273,
    kUiVersionNumberFieldNumber = 2124,
    kRctaModuleStateNuFieldNumber = 2236,
    kDoorProtectionModuleStateNuFieldNumber = 2526,
    kLscaOverallModeNuFieldNumber = 3249,
    kSteeringProposalModuleStateNuFieldNumber = 3481,
    kBrakingModuleStateNuFieldNumber = 3551,
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

  // optional .pb.mf_lsca.lsca_state.LSCA_STATE steeringResistanceModuleState_nu = 1272;
  bool has_steeringresistancemodulestate_nu() const;
  private:
  bool _internal_has_steeringresistancemodulestate_nu() const;
  public:
  void clear_steeringresistancemodulestate_nu();
  ::pb::mf_lsca::lsca_state::LSCA_STATE steeringresistancemodulestate_nu() const;
  void set_steeringresistancemodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value);
  private:
  ::pb::mf_lsca::lsca_state::LSCA_STATE _internal_steeringresistancemodulestate_nu() const;
  void _internal_set_steeringresistancemodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value);
  public:

  // optional .pb.mf_lsca.lsca_state.LSCA_STATE PmpModuleState_nu = 1273;
  bool has_pmpmodulestate_nu() const;
  private:
  bool _internal_has_pmpmodulestate_nu() const;
  public:
  void clear_pmpmodulestate_nu();
  ::pb::mf_lsca::lsca_state::LSCA_STATE pmpmodulestate_nu() const;
  void set_pmpmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value);
  private:
  ::pb::mf_lsca::lsca_state::LSCA_STATE _internal_pmpmodulestate_nu() const;
  void _internal_set_pmpmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value);
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

  // optional .pb.mf_lsca.lsca_state.LSCA_STATE rctaModuleState_nu = 2236;
  bool has_rctamodulestate_nu() const;
  private:
  bool _internal_has_rctamodulestate_nu() const;
  public:
  void clear_rctamodulestate_nu();
  ::pb::mf_lsca::lsca_state::LSCA_STATE rctamodulestate_nu() const;
  void set_rctamodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value);
  private:
  ::pb::mf_lsca::lsca_state::LSCA_STATE _internal_rctamodulestate_nu() const;
  void _internal_set_rctamodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value);
  public:

  // optional .pb.mf_lsca.lsca_state.LSCA_STATE doorProtectionModuleState_nu = 2526;
  bool has_doorprotectionmodulestate_nu() const;
  private:
  bool _internal_has_doorprotectionmodulestate_nu() const;
  public:
  void clear_doorprotectionmodulestate_nu();
  ::pb::mf_lsca::lsca_state::LSCA_STATE doorprotectionmodulestate_nu() const;
  void set_doorprotectionmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value);
  private:
  ::pb::mf_lsca::lsca_state::LSCA_STATE _internal_doorprotectionmodulestate_nu() const;
  void _internal_set_doorprotectionmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value);
  public:

  // optional .pb.mf_lsca.lsca_mode.LSCA_MODE lscaOverallMode_nu = 3249;
  bool has_lscaoverallmode_nu() const;
  private:
  bool _internal_has_lscaoverallmode_nu() const;
  public:
  void clear_lscaoverallmode_nu();
  ::pb::mf_lsca::lsca_mode::LSCA_MODE lscaoverallmode_nu() const;
  void set_lscaoverallmode_nu(::pb::mf_lsca::lsca_mode::LSCA_MODE value);
  private:
  ::pb::mf_lsca::lsca_mode::LSCA_MODE _internal_lscaoverallmode_nu() const;
  void _internal_set_lscaoverallmode_nu(::pb::mf_lsca::lsca_mode::LSCA_MODE value);
  public:

  // optional .pb.mf_lsca.lsca_state.LSCA_STATE steeringProposalModuleState_nu = 3481;
  bool has_steeringproposalmodulestate_nu() const;
  private:
  bool _internal_has_steeringproposalmodulestate_nu() const;
  public:
  void clear_steeringproposalmodulestate_nu();
  ::pb::mf_lsca::lsca_state::LSCA_STATE steeringproposalmodulestate_nu() const;
  void set_steeringproposalmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value);
  private:
  ::pb::mf_lsca::lsca_state::LSCA_STATE _internal_steeringproposalmodulestate_nu() const;
  void _internal_set_steeringproposalmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value);
  public:

  // optional .pb.mf_lsca.lsca_state.LSCA_STATE brakingModuleState_nu = 3551;
  bool has_brakingmodulestate_nu() const;
  private:
  bool _internal_has_brakingmodulestate_nu() const;
  public:
  void clear_brakingmodulestate_nu();
  ::pb::mf_lsca::lsca_state::LSCA_STATE brakingmodulestate_nu() const;
  void set_brakingmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value);
  private:
  ::pb::mf_lsca::lsca_state::LSCA_STATE _internal_brakingmodulestate_nu() const;
  void _internal_set_brakingmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_status_port.LscaStatusPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  int steeringresistancemodulestate_nu_;
  int pmpmodulestate_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  int rctamodulestate_nu_;
  int doorprotectionmodulestate_nu_;
  int lscaoverallmode_nu_;
  int steeringproposalmodulestate_nu_;
  int brakingmodulestate_nu_;
  friend struct ::TableStruct_mf_5flsca_2flsca_5fstatus_5fport_2eproto;
};
// -------------------------------------------------------------------

class LscaStatusPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_lsca.lsca_status_port.LscaStatusPort_array_port) */ {
 public:
  LscaStatusPort_array_port();
  virtual ~LscaStatusPort_array_port();

  LscaStatusPort_array_port(const LscaStatusPort_array_port& from);
  LscaStatusPort_array_port(LscaStatusPort_array_port&& from) noexcept
    : LscaStatusPort_array_port() {
    *this = ::std::move(from);
  }

  inline LscaStatusPort_array_port& operator=(const LscaStatusPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline LscaStatusPort_array_port& operator=(LscaStatusPort_array_port&& from) noexcept {
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
  static const LscaStatusPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LscaStatusPort_array_port* internal_default_instance() {
    return reinterpret_cast<const LscaStatusPort_array_port*>(
               &_LscaStatusPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(LscaStatusPort_array_port& a, LscaStatusPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(LscaStatusPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LscaStatusPort_array_port* New() const final {
    return CreateMaybeMessage<LscaStatusPort_array_port>(nullptr);
  }

  LscaStatusPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LscaStatusPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LscaStatusPort_array_port& from);
  void MergeFrom(const LscaStatusPort_array_port& from);
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
  void InternalSwap(LscaStatusPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_lsca.lsca_status_port.LscaStatusPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5flsca_2flsca_5fstatus_5fport_2eproto);
    return ::descriptor_table_mf_5flsca_2flsca_5fstatus_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 607,
  };
  // repeated .pb.mf_lsca.lsca_status_port.LscaStatusPort data = 607;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_lsca::lsca_status_port::LscaStatusPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_status_port::LscaStatusPort >*
      mutable_data();
  private:
  const ::pb::mf_lsca::lsca_status_port::LscaStatusPort& _internal_data(int index) const;
  ::pb::mf_lsca::lsca_status_port::LscaStatusPort* _internal_add_data();
  public:
  const ::pb::mf_lsca::lsca_status_port::LscaStatusPort& data(int index) const;
  ::pb::mf_lsca::lsca_status_port::LscaStatusPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_status_port::LscaStatusPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_status_port.LscaStatusPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_status_port::LscaStatusPort > data_;
  friend struct ::TableStruct_mf_5flsca_2flsca_5fstatus_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LscaStatusPort

// optional uint32 uiVersionNumber = 2124;
inline bool LscaStatusPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool LscaStatusPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void LscaStatusPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 LscaStatusPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 LscaStatusPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_status_port.LscaStatusPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void LscaStatusPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  uiversionnumber_ = value;
}
inline void LscaStatusPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.mf_lsca.lsca_status_port.LscaStatusPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool LscaStatusPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool LscaStatusPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& LscaStatusPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& LscaStatusPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_status_port.LscaStatusPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* LscaStatusPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.mf_lsca.lsca_status_port.LscaStatusPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* LscaStatusPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* LscaStatusPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.mf_lsca.lsca_status_port.LscaStatusPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void LscaStatusPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.mf_lsca.lsca_status_port.LscaStatusPort.sSigHeader)
}

// optional .pb.mf_lsca.lsca_state.LSCA_STATE brakingModuleState_nu = 3551;
inline bool LscaStatusPort::_internal_has_brakingmodulestate_nu() const {
  bool value = (_has_bits_[0] & 0x00000100u) != 0;
  return value;
}
inline bool LscaStatusPort::has_brakingmodulestate_nu() const {
  return _internal_has_brakingmodulestate_nu();
}
inline void LscaStatusPort::clear_brakingmodulestate_nu() {
  brakingmodulestate_nu_ = 0;
  _has_bits_[0] &= ~0x00000100u;
}
inline ::pb::mf_lsca::lsca_state::LSCA_STATE LscaStatusPort::_internal_brakingmodulestate_nu() const {
  return static_cast< ::pb::mf_lsca::lsca_state::LSCA_STATE >(brakingmodulestate_nu_);
}
inline ::pb::mf_lsca::lsca_state::LSCA_STATE LscaStatusPort::brakingmodulestate_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_status_port.LscaStatusPort.brakingModuleState_nu)
  return _internal_brakingmodulestate_nu();
}
inline void LscaStatusPort::_internal_set_brakingmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value) {
  assert(::pb::mf_lsca::lsca_state::LSCA_STATE_IsValid(value));
  _has_bits_[0] |= 0x00000100u;
  brakingmodulestate_nu_ = value;
}
inline void LscaStatusPort::set_brakingmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value) {
  _internal_set_brakingmodulestate_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_lsca.lsca_status_port.LscaStatusPort.brakingModuleState_nu)
}

// optional .pb.mf_lsca.lsca_state.LSCA_STATE doorProtectionModuleState_nu = 2526;
inline bool LscaStatusPort::_internal_has_doorprotectionmodulestate_nu() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool LscaStatusPort::has_doorprotectionmodulestate_nu() const {
  return _internal_has_doorprotectionmodulestate_nu();
}
inline void LscaStatusPort::clear_doorprotectionmodulestate_nu() {
  doorprotectionmodulestate_nu_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::pb::mf_lsca::lsca_state::LSCA_STATE LscaStatusPort::_internal_doorprotectionmodulestate_nu() const {
  return static_cast< ::pb::mf_lsca::lsca_state::LSCA_STATE >(doorprotectionmodulestate_nu_);
}
inline ::pb::mf_lsca::lsca_state::LSCA_STATE LscaStatusPort::doorprotectionmodulestate_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_status_port.LscaStatusPort.doorProtectionModuleState_nu)
  return _internal_doorprotectionmodulestate_nu();
}
inline void LscaStatusPort::_internal_set_doorprotectionmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value) {
  assert(::pb::mf_lsca::lsca_state::LSCA_STATE_IsValid(value));
  _has_bits_[0] |= 0x00000020u;
  doorprotectionmodulestate_nu_ = value;
}
inline void LscaStatusPort::set_doorprotectionmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value) {
  _internal_set_doorprotectionmodulestate_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_lsca.lsca_status_port.LscaStatusPort.doorProtectionModuleState_nu)
}

// optional .pb.mf_lsca.lsca_state.LSCA_STATE rctaModuleState_nu = 2236;
inline bool LscaStatusPort::_internal_has_rctamodulestate_nu() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool LscaStatusPort::has_rctamodulestate_nu() const {
  return _internal_has_rctamodulestate_nu();
}
inline void LscaStatusPort::clear_rctamodulestate_nu() {
  rctamodulestate_nu_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::pb::mf_lsca::lsca_state::LSCA_STATE LscaStatusPort::_internal_rctamodulestate_nu() const {
  return static_cast< ::pb::mf_lsca::lsca_state::LSCA_STATE >(rctamodulestate_nu_);
}
inline ::pb::mf_lsca::lsca_state::LSCA_STATE LscaStatusPort::rctamodulestate_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_status_port.LscaStatusPort.rctaModuleState_nu)
  return _internal_rctamodulestate_nu();
}
inline void LscaStatusPort::_internal_set_rctamodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value) {
  assert(::pb::mf_lsca::lsca_state::LSCA_STATE_IsValid(value));
  _has_bits_[0] |= 0x00000010u;
  rctamodulestate_nu_ = value;
}
inline void LscaStatusPort::set_rctamodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value) {
  _internal_set_rctamodulestate_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_lsca.lsca_status_port.LscaStatusPort.rctaModuleState_nu)
}

// optional .pb.mf_lsca.lsca_state.LSCA_STATE PmpModuleState_nu = 1273;
inline bool LscaStatusPort::_internal_has_pmpmodulestate_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool LscaStatusPort::has_pmpmodulestate_nu() const {
  return _internal_has_pmpmodulestate_nu();
}
inline void LscaStatusPort::clear_pmpmodulestate_nu() {
  pmpmodulestate_nu_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::mf_lsca::lsca_state::LSCA_STATE LscaStatusPort::_internal_pmpmodulestate_nu() const {
  return static_cast< ::pb::mf_lsca::lsca_state::LSCA_STATE >(pmpmodulestate_nu_);
}
inline ::pb::mf_lsca::lsca_state::LSCA_STATE LscaStatusPort::pmpmodulestate_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_status_port.LscaStatusPort.PmpModuleState_nu)
  return _internal_pmpmodulestate_nu();
}
inline void LscaStatusPort::_internal_set_pmpmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value) {
  assert(::pb::mf_lsca::lsca_state::LSCA_STATE_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  pmpmodulestate_nu_ = value;
}
inline void LscaStatusPort::set_pmpmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value) {
  _internal_set_pmpmodulestate_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_lsca.lsca_status_port.LscaStatusPort.PmpModuleState_nu)
}

// optional .pb.mf_lsca.lsca_state.LSCA_STATE steeringResistanceModuleState_nu = 1272;
inline bool LscaStatusPort::_internal_has_steeringresistancemodulestate_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool LscaStatusPort::has_steeringresistancemodulestate_nu() const {
  return _internal_has_steeringresistancemodulestate_nu();
}
inline void LscaStatusPort::clear_steeringresistancemodulestate_nu() {
  steeringresistancemodulestate_nu_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::pb::mf_lsca::lsca_state::LSCA_STATE LscaStatusPort::_internal_steeringresistancemodulestate_nu() const {
  return static_cast< ::pb::mf_lsca::lsca_state::LSCA_STATE >(steeringresistancemodulestate_nu_);
}
inline ::pb::mf_lsca::lsca_state::LSCA_STATE LscaStatusPort::steeringresistancemodulestate_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_status_port.LscaStatusPort.steeringResistanceModuleState_nu)
  return _internal_steeringresistancemodulestate_nu();
}
inline void LscaStatusPort::_internal_set_steeringresistancemodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value) {
  assert(::pb::mf_lsca::lsca_state::LSCA_STATE_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  steeringresistancemodulestate_nu_ = value;
}
inline void LscaStatusPort::set_steeringresistancemodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value) {
  _internal_set_steeringresistancemodulestate_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_lsca.lsca_status_port.LscaStatusPort.steeringResistanceModuleState_nu)
}

// optional .pb.mf_lsca.lsca_state.LSCA_STATE steeringProposalModuleState_nu = 3481;
inline bool LscaStatusPort::_internal_has_steeringproposalmodulestate_nu() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool LscaStatusPort::has_steeringproposalmodulestate_nu() const {
  return _internal_has_steeringproposalmodulestate_nu();
}
inline void LscaStatusPort::clear_steeringproposalmodulestate_nu() {
  steeringproposalmodulestate_nu_ = 0;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::pb::mf_lsca::lsca_state::LSCA_STATE LscaStatusPort::_internal_steeringproposalmodulestate_nu() const {
  return static_cast< ::pb::mf_lsca::lsca_state::LSCA_STATE >(steeringproposalmodulestate_nu_);
}
inline ::pb::mf_lsca::lsca_state::LSCA_STATE LscaStatusPort::steeringproposalmodulestate_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_status_port.LscaStatusPort.steeringProposalModuleState_nu)
  return _internal_steeringproposalmodulestate_nu();
}
inline void LscaStatusPort::_internal_set_steeringproposalmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value) {
  assert(::pb::mf_lsca::lsca_state::LSCA_STATE_IsValid(value));
  _has_bits_[0] |= 0x00000080u;
  steeringproposalmodulestate_nu_ = value;
}
inline void LscaStatusPort::set_steeringproposalmodulestate_nu(::pb::mf_lsca::lsca_state::LSCA_STATE value) {
  _internal_set_steeringproposalmodulestate_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_lsca.lsca_status_port.LscaStatusPort.steeringProposalModuleState_nu)
}

// optional .pb.mf_lsca.lsca_mode.LSCA_MODE lscaOverallMode_nu = 3249;
inline bool LscaStatusPort::_internal_has_lscaoverallmode_nu() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool LscaStatusPort::has_lscaoverallmode_nu() const {
  return _internal_has_lscaoverallmode_nu();
}
inline void LscaStatusPort::clear_lscaoverallmode_nu() {
  lscaoverallmode_nu_ = 0;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::pb::mf_lsca::lsca_mode::LSCA_MODE LscaStatusPort::_internal_lscaoverallmode_nu() const {
  return static_cast< ::pb::mf_lsca::lsca_mode::LSCA_MODE >(lscaoverallmode_nu_);
}
inline ::pb::mf_lsca::lsca_mode::LSCA_MODE LscaStatusPort::lscaoverallmode_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_status_port.LscaStatusPort.lscaOverallMode_nu)
  return _internal_lscaoverallmode_nu();
}
inline void LscaStatusPort::_internal_set_lscaoverallmode_nu(::pb::mf_lsca::lsca_mode::LSCA_MODE value) {
  assert(::pb::mf_lsca::lsca_mode::LSCA_MODE_IsValid(value));
  _has_bits_[0] |= 0x00000040u;
  lscaoverallmode_nu_ = value;
}
inline void LscaStatusPort::set_lscaoverallmode_nu(::pb::mf_lsca::lsca_mode::LSCA_MODE value) {
  _internal_set_lscaoverallmode_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_lsca.lsca_status_port.LscaStatusPort.lscaOverallMode_nu)
}

// -------------------------------------------------------------------

// LscaStatusPort_array_port

// repeated .pb.mf_lsca.lsca_status_port.LscaStatusPort data = 607;
inline int LscaStatusPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int LscaStatusPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void LscaStatusPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_lsca::lsca_status_port::LscaStatusPort* LscaStatusPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_lsca.lsca_status_port.LscaStatusPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_status_port::LscaStatusPort >*
LscaStatusPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_lsca.lsca_status_port.LscaStatusPort_array_port.data)
  return &data_;
}
inline const ::pb::mf_lsca::lsca_status_port::LscaStatusPort& LscaStatusPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_lsca::lsca_status_port::LscaStatusPort& LscaStatusPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_status_port.LscaStatusPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_lsca::lsca_status_port::LscaStatusPort* LscaStatusPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_lsca::lsca_status_port::LscaStatusPort* LscaStatusPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_lsca.lsca_status_port.LscaStatusPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_status_port::LscaStatusPort >&
LscaStatusPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_lsca.lsca_status_port.LscaStatusPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace lsca_status_port
}  // namespace mf_lsca
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5flsca_2flsca_5fstatus_5fport_2eproto
