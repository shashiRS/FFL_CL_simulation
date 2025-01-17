// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: eco/algo_comp_state.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_eco_2falgo_5fcomp_5fstate_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_eco_2falgo_5fcomp_5fstate_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_eco_2falgo_5fcomp_5fstate_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_eco_2falgo_5fcomp_5fstate_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_eco_2falgo_5fcomp_5fstate_2eproto;
namespace pb {
namespace eco {
namespace algo_comp_state {
class AlgoCompState;
class AlgoCompStateDefaultTypeInternal;
extern AlgoCompStateDefaultTypeInternal _AlgoCompState_default_instance_;
class AlgoCompState_array_port;
class AlgoCompState_array_portDefaultTypeInternal;
extern AlgoCompState_array_portDefaultTypeInternal _AlgoCompState_array_port_default_instance_;
}  // namespace algo_comp_state
}  // namespace eco
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::eco::algo_comp_state::AlgoCompState* Arena::CreateMaybeMessage<::pb::eco::algo_comp_state::AlgoCompState>(Arena*);
template<> ::pb::eco::algo_comp_state::AlgoCompState_array_port* Arena::CreateMaybeMessage<::pb::eco::algo_comp_state::AlgoCompState_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace eco {
namespace algo_comp_state {

// ===================================================================

class AlgoCompState :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.eco.algo_comp_state.AlgoCompState) */ {
 public:
  AlgoCompState();
  virtual ~AlgoCompState();

  AlgoCompState(const AlgoCompState& from);
  AlgoCompState(AlgoCompState&& from) noexcept
    : AlgoCompState() {
    *this = ::std::move(from);
  }

  inline AlgoCompState& operator=(const AlgoCompState& from) {
    CopyFrom(from);
    return *this;
  }
  inline AlgoCompState& operator=(AlgoCompState&& from) noexcept {
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
  static const AlgoCompState& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const AlgoCompState* internal_default_instance() {
    return reinterpret_cast<const AlgoCompState*>(
               &_AlgoCompState_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(AlgoCompState& a, AlgoCompState& b) {
    a.Swap(&b);
  }
  inline void Swap(AlgoCompState* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline AlgoCompState* New() const final {
    return CreateMaybeMessage<AlgoCompState>(nullptr);
  }

  AlgoCompState* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<AlgoCompState>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const AlgoCompState& from);
  void MergeFrom(const AlgoCompState& from);
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
  void InternalSwap(AlgoCompState* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.eco.algo_comp_state.AlgoCompState";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_eco_2falgo_5fcomp_5fstate_2eproto);
    return ::descriptor_table_eco_2falgo_5fcomp_5fstate_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kEErrorCodeFieldNumber = 37,
    kUiAlgoVersionNumberFieldNumber = 620,
    kECompStateFieldNumber = 1083,
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

  // optional uint32 eErrorCode = 37;
  bool has_eerrorcode() const;
  private:
  bool _internal_has_eerrorcode() const;
  public:
  void clear_eerrorcode();
  ::PROTOBUF_NAMESPACE_ID::uint32 eerrorcode() const;
  void set_eerrorcode(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_eerrorcode() const;
  void _internal_set_eerrorcode(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 uiAlgoVersionNumber = 620;
  bool has_uialgoversionnumber() const;
  private:
  bool _internal_has_uialgoversionnumber() const;
  public:
  void clear_uialgoversionnumber();
  ::PROTOBUF_NAMESPACE_ID::uint32 uialgoversionnumber() const;
  void set_uialgoversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_uialgoversionnumber() const;
  void _internal_set_uialgoversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 eCompState = 1083;
  bool has_ecompstate() const;
  private:
  bool _internal_has_ecompstate() const;
  public:
  void clear_ecompstate();
  ::PROTOBUF_NAMESPACE_ID::uint32 ecompstate() const;
  void set_ecompstate(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_ecompstate() const;
  void _internal_set_ecompstate(::PROTOBUF_NAMESPACE_ID::uint32 value);
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

  // @@protoc_insertion_point(class_scope:pb.eco.algo_comp_state.AlgoCompState)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint32 eerrorcode_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uialgoversionnumber_;
  ::PROTOBUF_NAMESPACE_ID::uint32 ecompstate_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  friend struct ::TableStruct_eco_2falgo_5fcomp_5fstate_2eproto;
};
// -------------------------------------------------------------------

class AlgoCompState_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.eco.algo_comp_state.AlgoCompState_array_port) */ {
 public:
  AlgoCompState_array_port();
  virtual ~AlgoCompState_array_port();

  AlgoCompState_array_port(const AlgoCompState_array_port& from);
  AlgoCompState_array_port(AlgoCompState_array_port&& from) noexcept
    : AlgoCompState_array_port() {
    *this = ::std::move(from);
  }

  inline AlgoCompState_array_port& operator=(const AlgoCompState_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline AlgoCompState_array_port& operator=(AlgoCompState_array_port&& from) noexcept {
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
  static const AlgoCompState_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const AlgoCompState_array_port* internal_default_instance() {
    return reinterpret_cast<const AlgoCompState_array_port*>(
               &_AlgoCompState_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(AlgoCompState_array_port& a, AlgoCompState_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(AlgoCompState_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline AlgoCompState_array_port* New() const final {
    return CreateMaybeMessage<AlgoCompState_array_port>(nullptr);
  }

  AlgoCompState_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<AlgoCompState_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const AlgoCompState_array_port& from);
  void MergeFrom(const AlgoCompState_array_port& from);
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
  void InternalSwap(AlgoCompState_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.eco.algo_comp_state.AlgoCompState_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_eco_2falgo_5fcomp_5fstate_2eproto);
    return ::descriptor_table_eco_2falgo_5fcomp_5fstate_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 491,
  };
  // repeated .pb.eco.algo_comp_state.AlgoCompState data = 491;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::eco::algo_comp_state::AlgoCompState* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::algo_comp_state::AlgoCompState >*
      mutable_data();
  private:
  const ::pb::eco::algo_comp_state::AlgoCompState& _internal_data(int index) const;
  ::pb::eco::algo_comp_state::AlgoCompState* _internal_add_data();
  public:
  const ::pb::eco::algo_comp_state::AlgoCompState& data(int index) const;
  ::pb::eco::algo_comp_state::AlgoCompState* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::algo_comp_state::AlgoCompState >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.eco.algo_comp_state.AlgoCompState_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::algo_comp_state::AlgoCompState > data_;
  friend struct ::TableStruct_eco_2falgo_5fcomp_5fstate_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// AlgoCompState

// optional uint32 uiVersionNumber = 2124;
inline bool AlgoCompState::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool AlgoCompState::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void AlgoCompState::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 AlgoCompState::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 AlgoCompState::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.eco.algo_comp_state.AlgoCompState.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void AlgoCompState::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  uiversionnumber_ = value;
}
inline void AlgoCompState::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.eco.algo_comp_state.AlgoCompState.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool AlgoCompState::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool AlgoCompState::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& AlgoCompState::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& AlgoCompState::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.eco.algo_comp_state.AlgoCompState.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* AlgoCompState::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.eco.algo_comp_state.AlgoCompState.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* AlgoCompState::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* AlgoCompState::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.eco.algo_comp_state.AlgoCompState.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void AlgoCompState::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.eco.algo_comp_state.AlgoCompState.sSigHeader)
}

// optional uint32 uiAlgoVersionNumber = 620;
inline bool AlgoCompState::_internal_has_uialgoversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool AlgoCompState::has_uialgoversionnumber() const {
  return _internal_has_uialgoversionnumber();
}
inline void AlgoCompState::clear_uialgoversionnumber() {
  uialgoversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 AlgoCompState::_internal_uialgoversionnumber() const {
  return uialgoversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 AlgoCompState::uialgoversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.eco.algo_comp_state.AlgoCompState.uiAlgoVersionNumber)
  return _internal_uialgoversionnumber();
}
inline void AlgoCompState::_internal_set_uialgoversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  uialgoversionnumber_ = value;
}
inline void AlgoCompState::set_uialgoversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uialgoversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.eco.algo_comp_state.AlgoCompState.uiAlgoVersionNumber)
}

// optional uint32 eCompState = 1083;
inline bool AlgoCompState::_internal_has_ecompstate() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool AlgoCompState::has_ecompstate() const {
  return _internal_has_ecompstate();
}
inline void AlgoCompState::clear_ecompstate() {
  ecompstate_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 AlgoCompState::_internal_ecompstate() const {
  return ecompstate_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 AlgoCompState::ecompstate() const {
  // @@protoc_insertion_point(field_get:pb.eco.algo_comp_state.AlgoCompState.eCompState)
  return _internal_ecompstate();
}
inline void AlgoCompState::_internal_set_ecompstate(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  ecompstate_ = value;
}
inline void AlgoCompState::set_ecompstate(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_ecompstate(value);
  // @@protoc_insertion_point(field_set:pb.eco.algo_comp_state.AlgoCompState.eCompState)
}

// optional uint32 eErrorCode = 37;
inline bool AlgoCompState::_internal_has_eerrorcode() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool AlgoCompState::has_eerrorcode() const {
  return _internal_has_eerrorcode();
}
inline void AlgoCompState::clear_eerrorcode() {
  eerrorcode_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 AlgoCompState::_internal_eerrorcode() const {
  return eerrorcode_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 AlgoCompState::eerrorcode() const {
  // @@protoc_insertion_point(field_get:pb.eco.algo_comp_state.AlgoCompState.eErrorCode)
  return _internal_eerrorcode();
}
inline void AlgoCompState::_internal_set_eerrorcode(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  eerrorcode_ = value;
}
inline void AlgoCompState::set_eerrorcode(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_eerrorcode(value);
  // @@protoc_insertion_point(field_set:pb.eco.algo_comp_state.AlgoCompState.eErrorCode)
}

// -------------------------------------------------------------------

// AlgoCompState_array_port

// repeated .pb.eco.algo_comp_state.AlgoCompState data = 491;
inline int AlgoCompState_array_port::_internal_data_size() const {
  return data_.size();
}
inline int AlgoCompState_array_port::data_size() const {
  return _internal_data_size();
}
inline void AlgoCompState_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::eco::algo_comp_state::AlgoCompState* AlgoCompState_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.eco.algo_comp_state.AlgoCompState_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::algo_comp_state::AlgoCompState >*
AlgoCompState_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.eco.algo_comp_state.AlgoCompState_array_port.data)
  return &data_;
}
inline const ::pb::eco::algo_comp_state::AlgoCompState& AlgoCompState_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::eco::algo_comp_state::AlgoCompState& AlgoCompState_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.eco.algo_comp_state.AlgoCompState_array_port.data)
  return _internal_data(index);
}
inline ::pb::eco::algo_comp_state::AlgoCompState* AlgoCompState_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::eco::algo_comp_state::AlgoCompState* AlgoCompState_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.eco.algo_comp_state.AlgoCompState_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::algo_comp_state::AlgoCompState >&
AlgoCompState_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.eco.algo_comp_state.AlgoCompState_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace algo_comp_state
}  // namespace eco
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_eco_2falgo_5fcomp_5fstate_2eproto
