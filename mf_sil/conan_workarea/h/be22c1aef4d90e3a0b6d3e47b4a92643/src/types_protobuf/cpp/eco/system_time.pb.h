// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: eco/system_time.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_eco_2fsystem_5ftime_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_eco_2fsystem_5ftime_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_eco_2fsystem_5ftime_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_eco_2fsystem_5ftime_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_eco_2fsystem_5ftime_2eproto;
namespace pb {
namespace eco {
namespace system_time {
class SystemTime;
class SystemTimeDefaultTypeInternal;
extern SystemTimeDefaultTypeInternal _SystemTime_default_instance_;
class SystemTime_array_port;
class SystemTime_array_portDefaultTypeInternal;
extern SystemTime_array_portDefaultTypeInternal _SystemTime_array_port_default_instance_;
}  // namespace system_time
}  // namespace eco
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::eco::system_time::SystemTime* Arena::CreateMaybeMessage<::pb::eco::system_time::SystemTime>(Arena*);
template<> ::pb::eco::system_time::SystemTime_array_port* Arena::CreateMaybeMessage<::pb::eco::system_time::SystemTime_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace eco {
namespace system_time {

// ===================================================================

class SystemTime :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.eco.system_time.SystemTime) */ {
 public:
  SystemTime();
  virtual ~SystemTime();

  SystemTime(const SystemTime& from);
  SystemTime(SystemTime&& from) noexcept
    : SystemTime() {
    *this = ::std::move(from);
  }

  inline SystemTime& operator=(const SystemTime& from) {
    CopyFrom(from);
    return *this;
  }
  inline SystemTime& operator=(SystemTime&& from) noexcept {
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
  static const SystemTime& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SystemTime* internal_default_instance() {
    return reinterpret_cast<const SystemTime*>(
               &_SystemTime_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(SystemTime& a, SystemTime& b) {
    a.Swap(&b);
  }
  inline void Swap(SystemTime* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SystemTime* New() const final {
    return CreateMaybeMessage<SystemTime>(nullptr);
  }

  SystemTime* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SystemTime>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SystemTime& from);
  void MergeFrom(const SystemTime& from);
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
  void InternalSwap(SystemTime* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.eco.system_time.SystemTime";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_eco_2fsystem_5ftime_2eproto);
    return ::descriptor_table_eco_2fsystem_5ftime_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSigHeaderFieldNumber = 1450,
    kSystemTimeFieldNumber = 3595,
  };
  // optional .pb.eco.signal_header.SignalHeader sigHeader = 1450;
  bool has_sigheader() const;
  private:
  bool _internal_has_sigheader() const;
  public:
  void clear_sigheader();
  const ::pb::eco::signal_header::SignalHeader& sigheader() const;
  ::pb::eco::signal_header::SignalHeader* release_sigheader();
  ::pb::eco::signal_header::SignalHeader* mutable_sigheader();
  void set_allocated_sigheader(::pb::eco::signal_header::SignalHeader* sigheader);
  private:
  const ::pb::eco::signal_header::SignalHeader& _internal_sigheader() const;
  ::pb::eco::signal_header::SignalHeader* _internal_mutable_sigheader();
  public:

  // optional uint64 systemTime = 3595;
  bool has_systemtime() const;
  private:
  bool _internal_has_systemtime() const;
  public:
  void clear_systemtime();
  ::PROTOBUF_NAMESPACE_ID::uint64 systemtime() const;
  void set_systemtime(::PROTOBUF_NAMESPACE_ID::uint64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint64 _internal_systemtime() const;
  void _internal_set_systemtime(::PROTOBUF_NAMESPACE_ID::uint64 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.eco.system_time.SystemTime)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* sigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint64 systemtime_;
  friend struct ::TableStruct_eco_2fsystem_5ftime_2eproto;
};
// -------------------------------------------------------------------

class SystemTime_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.eco.system_time.SystemTime_array_port) */ {
 public:
  SystemTime_array_port();
  virtual ~SystemTime_array_port();

  SystemTime_array_port(const SystemTime_array_port& from);
  SystemTime_array_port(SystemTime_array_port&& from) noexcept
    : SystemTime_array_port() {
    *this = ::std::move(from);
  }

  inline SystemTime_array_port& operator=(const SystemTime_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline SystemTime_array_port& operator=(SystemTime_array_port&& from) noexcept {
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
  static const SystemTime_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SystemTime_array_port* internal_default_instance() {
    return reinterpret_cast<const SystemTime_array_port*>(
               &_SystemTime_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(SystemTime_array_port& a, SystemTime_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(SystemTime_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SystemTime_array_port* New() const final {
    return CreateMaybeMessage<SystemTime_array_port>(nullptr);
  }

  SystemTime_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SystemTime_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SystemTime_array_port& from);
  void MergeFrom(const SystemTime_array_port& from);
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
  void InternalSwap(SystemTime_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.eco.system_time.SystemTime_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_eco_2fsystem_5ftime_2eproto);
    return ::descriptor_table_eco_2fsystem_5ftime_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 403,
  };
  // repeated .pb.eco.system_time.SystemTime data = 403;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::eco::system_time::SystemTime* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::system_time::SystemTime >*
      mutable_data();
  private:
  const ::pb::eco::system_time::SystemTime& _internal_data(int index) const;
  ::pb::eco::system_time::SystemTime* _internal_add_data();
  public:
  const ::pb::eco::system_time::SystemTime& data(int index) const;
  ::pb::eco::system_time::SystemTime* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::system_time::SystemTime >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.eco.system_time.SystemTime_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::system_time::SystemTime > data_;
  friend struct ::TableStruct_eco_2fsystem_5ftime_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SystemTime

// optional .pb.eco.signal_header.SignalHeader sigHeader = 1450;
inline bool SystemTime::_internal_has_sigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || sigheader_ != nullptr);
  return value;
}
inline bool SystemTime::has_sigheader() const {
  return _internal_has_sigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& SystemTime::_internal_sigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = sigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& SystemTime::sigheader() const {
  // @@protoc_insertion_point(field_get:pb.eco.system_time.SystemTime.sigHeader)
  return _internal_sigheader();
}
inline ::pb::eco::signal_header::SignalHeader* SystemTime::release_sigheader() {
  // @@protoc_insertion_point(field_release:pb.eco.system_time.SystemTime.sigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = sigheader_;
  sigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* SystemTime::_internal_mutable_sigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (sigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    sigheader_ = p;
  }
  return sigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* SystemTime::mutable_sigheader() {
  // @@protoc_insertion_point(field_mutable:pb.eco.system_time.SystemTime.sigHeader)
  return _internal_mutable_sigheader();
}
inline void SystemTime::set_allocated_sigheader(::pb::eco::signal_header::SignalHeader* sigheader) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(sigheader_);
  }
  if (sigheader) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      sigheader = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, sigheader, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  sigheader_ = sigheader;
  // @@protoc_insertion_point(field_set_allocated:pb.eco.system_time.SystemTime.sigHeader)
}

// optional uint64 systemTime = 3595;
inline bool SystemTime::_internal_has_systemtime() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool SystemTime::has_systemtime() const {
  return _internal_has_systemtime();
}
inline void SystemTime::clear_systemtime() {
  systemtime_ = PROTOBUF_ULONGLONG(0);
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 SystemTime::_internal_systemtime() const {
  return systemtime_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 SystemTime::systemtime() const {
  // @@protoc_insertion_point(field_get:pb.eco.system_time.SystemTime.systemTime)
  return _internal_systemtime();
}
inline void SystemTime::_internal_set_systemtime(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _has_bits_[0] |= 0x00000002u;
  systemtime_ = value;
}
inline void SystemTime::set_systemtime(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _internal_set_systemtime(value);
  // @@protoc_insertion_point(field_set:pb.eco.system_time.SystemTime.systemTime)
}

// -------------------------------------------------------------------

// SystemTime_array_port

// repeated .pb.eco.system_time.SystemTime data = 403;
inline int SystemTime_array_port::_internal_data_size() const {
  return data_.size();
}
inline int SystemTime_array_port::data_size() const {
  return _internal_data_size();
}
inline void SystemTime_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::eco::system_time::SystemTime* SystemTime_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.eco.system_time.SystemTime_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::system_time::SystemTime >*
SystemTime_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.eco.system_time.SystemTime_array_port.data)
  return &data_;
}
inline const ::pb::eco::system_time::SystemTime& SystemTime_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::eco::system_time::SystemTime& SystemTime_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.eco.system_time.SystemTime_array_port.data)
  return _internal_data(index);
}
inline ::pb::eco::system_time::SystemTime* SystemTime_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::eco::system_time::SystemTime* SystemTime_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.eco.system_time.SystemTime_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::system_time::SystemTime >&
SystemTime_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.eco.system_time.SystemTime_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace system_time
}  // namespace eco
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_eco_2fsystem_5ftime_2eproto
