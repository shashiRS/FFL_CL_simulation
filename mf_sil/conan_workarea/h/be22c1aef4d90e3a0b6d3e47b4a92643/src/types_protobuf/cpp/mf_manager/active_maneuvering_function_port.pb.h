// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_manager/active_maneuvering_function_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fmanager_2factive_5fmaneuvering_5ffunction_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fmanager_2factive_5fmaneuvering_5ffunction_5fport_2eproto

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
#include "mf_manager/active_maneuvering_function.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fmanager_2factive_5fmaneuvering_5ffunction_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fmanager_2factive_5fmaneuvering_5ffunction_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_5fport_2eproto;
namespace pb {
namespace mf_manager {
namespace active_maneuvering_function_port {
class ActiveManeuveringFunctionPort;
class ActiveManeuveringFunctionPortDefaultTypeInternal;
extern ActiveManeuveringFunctionPortDefaultTypeInternal _ActiveManeuveringFunctionPort_default_instance_;
class ActiveManeuveringFunctionPort_array_port;
class ActiveManeuveringFunctionPort_array_portDefaultTypeInternal;
extern ActiveManeuveringFunctionPort_array_portDefaultTypeInternal _ActiveManeuveringFunctionPort_array_port_default_instance_;
}  // namespace active_maneuvering_function_port
}  // namespace mf_manager
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort* Arena::CreateMaybeMessage<::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort>(Arena*);
template<> ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort_array_port* Arena::CreateMaybeMessage<::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_manager {
namespace active_maneuvering_function_port {

// ===================================================================

class ActiveManeuveringFunctionPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort) */ {
 public:
  ActiveManeuveringFunctionPort();
  virtual ~ActiveManeuveringFunctionPort();

  ActiveManeuveringFunctionPort(const ActiveManeuveringFunctionPort& from);
  ActiveManeuveringFunctionPort(ActiveManeuveringFunctionPort&& from) noexcept
    : ActiveManeuveringFunctionPort() {
    *this = ::std::move(from);
  }

  inline ActiveManeuveringFunctionPort& operator=(const ActiveManeuveringFunctionPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline ActiveManeuveringFunctionPort& operator=(ActiveManeuveringFunctionPort&& from) noexcept {
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
  static const ActiveManeuveringFunctionPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ActiveManeuveringFunctionPort* internal_default_instance() {
    return reinterpret_cast<const ActiveManeuveringFunctionPort*>(
               &_ActiveManeuveringFunctionPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ActiveManeuveringFunctionPort& a, ActiveManeuveringFunctionPort& b) {
    a.Swap(&b);
  }
  inline void Swap(ActiveManeuveringFunctionPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ActiveManeuveringFunctionPort* New() const final {
    return CreateMaybeMessage<ActiveManeuveringFunctionPort>(nullptr);
  }

  ActiveManeuveringFunctionPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ActiveManeuveringFunctionPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ActiveManeuveringFunctionPort& from);
  void MergeFrom(const ActiveManeuveringFunctionPort& from);
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
  void InternalSwap(ActiveManeuveringFunctionPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_5fport_2eproto);
    return ::descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kUiVersionNumberFieldNumber = 2124,
    kLaCtrlRequestOriginFieldNumber = 1269,
    kLoCtrlRequestOriginFieldNumber = 1406,
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

  // optional .pb.mf_manager.active_maneuvering_function.ActiveManeuveringFunction laCtrlRequestOrigin = 1269;
  bool has_lactrlrequestorigin() const;
  private:
  bool _internal_has_lactrlrequestorigin() const;
  public:
  void clear_lactrlrequestorigin();
  ::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction lactrlrequestorigin() const;
  void set_lactrlrequestorigin(::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction value);
  private:
  ::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction _internal_lactrlrequestorigin() const;
  void _internal_set_lactrlrequestorigin(::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction value);
  public:

  // optional .pb.mf_manager.active_maneuvering_function.ActiveManeuveringFunction loCtrlRequestOrigin = 1406;
  bool has_loctrlrequestorigin() const;
  private:
  bool _internal_has_loctrlrequestorigin() const;
  public:
  void clear_loctrlrequestorigin();
  ::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction loctrlrequestorigin() const;
  void set_loctrlrequestorigin(::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction value);
  private:
  ::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction _internal_loctrlrequestorigin() const;
  void _internal_set_loctrlrequestorigin(::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  int lactrlrequestorigin_;
  int loctrlrequestorigin_;
  friend struct ::TableStruct_mf_5fmanager_2factive_5fmaneuvering_5ffunction_5fport_2eproto;
};
// -------------------------------------------------------------------

class ActiveManeuveringFunctionPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort_array_port) */ {
 public:
  ActiveManeuveringFunctionPort_array_port();
  virtual ~ActiveManeuveringFunctionPort_array_port();

  ActiveManeuveringFunctionPort_array_port(const ActiveManeuveringFunctionPort_array_port& from);
  ActiveManeuveringFunctionPort_array_port(ActiveManeuveringFunctionPort_array_port&& from) noexcept
    : ActiveManeuveringFunctionPort_array_port() {
    *this = ::std::move(from);
  }

  inline ActiveManeuveringFunctionPort_array_port& operator=(const ActiveManeuveringFunctionPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline ActiveManeuveringFunctionPort_array_port& operator=(ActiveManeuveringFunctionPort_array_port&& from) noexcept {
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
  static const ActiveManeuveringFunctionPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ActiveManeuveringFunctionPort_array_port* internal_default_instance() {
    return reinterpret_cast<const ActiveManeuveringFunctionPort_array_port*>(
               &_ActiveManeuveringFunctionPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(ActiveManeuveringFunctionPort_array_port& a, ActiveManeuveringFunctionPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(ActiveManeuveringFunctionPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ActiveManeuveringFunctionPort_array_port* New() const final {
    return CreateMaybeMessage<ActiveManeuveringFunctionPort_array_port>(nullptr);
  }

  ActiveManeuveringFunctionPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ActiveManeuveringFunctionPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ActiveManeuveringFunctionPort_array_port& from);
  void MergeFrom(const ActiveManeuveringFunctionPort_array_port& from);
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
  void InternalSwap(ActiveManeuveringFunctionPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_5fport_2eproto);
    return ::descriptor_table_mf_5fmanager_2factive_5fmaneuvering_5ffunction_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1543,
  };
  // repeated .pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort data = 1543;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort >*
      mutable_data();
  private:
  const ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort& _internal_data(int index) const;
  ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort* _internal_add_data();
  public:
  const ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort& data(int index) const;
  ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort > data_;
  friend struct ::TableStruct_mf_5fmanager_2factive_5fmaneuvering_5ffunction_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ActiveManeuveringFunctionPort

// optional uint32 uiVersionNumber = 2124;
inline bool ActiveManeuveringFunctionPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool ActiveManeuveringFunctionPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void ActiveManeuveringFunctionPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ActiveManeuveringFunctionPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ActiveManeuveringFunctionPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void ActiveManeuveringFunctionPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  uiversionnumber_ = value;
}
inline void ActiveManeuveringFunctionPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool ActiveManeuveringFunctionPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool ActiveManeuveringFunctionPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& ActiveManeuveringFunctionPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& ActiveManeuveringFunctionPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* ActiveManeuveringFunctionPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* ActiveManeuveringFunctionPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* ActiveManeuveringFunctionPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void ActiveManeuveringFunctionPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort.sSigHeader)
}

// optional .pb.mf_manager.active_maneuvering_function.ActiveManeuveringFunction loCtrlRequestOrigin = 1406;
inline bool ActiveManeuveringFunctionPort::_internal_has_loctrlrequestorigin() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool ActiveManeuveringFunctionPort::has_loctrlrequestorigin() const {
  return _internal_has_loctrlrequestorigin();
}
inline void ActiveManeuveringFunctionPort::clear_loctrlrequestorigin() {
  loctrlrequestorigin_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction ActiveManeuveringFunctionPort::_internal_loctrlrequestorigin() const {
  return static_cast< ::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction >(loctrlrequestorigin_);
}
inline ::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction ActiveManeuveringFunctionPort::loctrlrequestorigin() const {
  // @@protoc_insertion_point(field_get:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort.loCtrlRequestOrigin)
  return _internal_loctrlrequestorigin();
}
inline void ActiveManeuveringFunctionPort::_internal_set_loctrlrequestorigin(::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction value) {
  assert(::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction_IsValid(value));
  _has_bits_[0] |= 0x00000008u;
  loctrlrequestorigin_ = value;
}
inline void ActiveManeuveringFunctionPort::set_loctrlrequestorigin(::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction value) {
  _internal_set_loctrlrequestorigin(value);
  // @@protoc_insertion_point(field_set:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort.loCtrlRequestOrigin)
}

// optional .pb.mf_manager.active_maneuvering_function.ActiveManeuveringFunction laCtrlRequestOrigin = 1269;
inline bool ActiveManeuveringFunctionPort::_internal_has_lactrlrequestorigin() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool ActiveManeuveringFunctionPort::has_lactrlrequestorigin() const {
  return _internal_has_lactrlrequestorigin();
}
inline void ActiveManeuveringFunctionPort::clear_lactrlrequestorigin() {
  lactrlrequestorigin_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction ActiveManeuveringFunctionPort::_internal_lactrlrequestorigin() const {
  return static_cast< ::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction >(lactrlrequestorigin_);
}
inline ::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction ActiveManeuveringFunctionPort::lactrlrequestorigin() const {
  // @@protoc_insertion_point(field_get:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort.laCtrlRequestOrigin)
  return _internal_lactrlrequestorigin();
}
inline void ActiveManeuveringFunctionPort::_internal_set_lactrlrequestorigin(::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction value) {
  assert(::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  lactrlrequestorigin_ = value;
}
inline void ActiveManeuveringFunctionPort::set_lactrlrequestorigin(::pb::mf_manager::active_maneuvering_function::ActiveManeuveringFunction value) {
  _internal_set_lactrlrequestorigin(value);
  // @@protoc_insertion_point(field_set:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort.laCtrlRequestOrigin)
}

// -------------------------------------------------------------------

// ActiveManeuveringFunctionPort_array_port

// repeated .pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort data = 1543;
inline int ActiveManeuveringFunctionPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int ActiveManeuveringFunctionPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void ActiveManeuveringFunctionPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort* ActiveManeuveringFunctionPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort >*
ActiveManeuveringFunctionPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort_array_port.data)
  return &data_;
}
inline const ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort& ActiveManeuveringFunctionPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort& ActiveManeuveringFunctionPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort* ActiveManeuveringFunctionPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort* ActiveManeuveringFunctionPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::active_maneuvering_function_port::ActiveManeuveringFunctionPort >&
ActiveManeuveringFunctionPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_manager.active_maneuvering_function_port.ActiveManeuveringFunctionPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace active_maneuvering_function_port
}  // namespace mf_manager
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fmanager_2factive_5fmaneuvering_5ffunction_5fport_2eproto
