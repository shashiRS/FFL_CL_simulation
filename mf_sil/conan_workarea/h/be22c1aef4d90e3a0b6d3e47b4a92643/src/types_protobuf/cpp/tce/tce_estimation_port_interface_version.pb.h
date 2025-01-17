// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: tce/tce_estimation_port_interface_version.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_tce_2ftce_5festimation_5fport_5finterface_5fversion_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_tce_2ftce_5festimation_5fport_5finterface_5fversion_2eproto

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
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_tce_2ftce_5festimation_5fport_5finterface_5fversion_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_tce_2ftce_5festimation_5fport_5finterface_5fversion_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_tce_2ftce_5festimation_5fport_5finterface_5fversion_2eproto;
namespace pb {
namespace tce {
namespace tce_estimation_port_interface_version {
class TceEstimationPort_InterfaceVersion;
class TceEstimationPort_InterfaceVersionDefaultTypeInternal;
extern TceEstimationPort_InterfaceVersionDefaultTypeInternal _TceEstimationPort_InterfaceVersion_default_instance_;
class TceEstimationPort_InterfaceVersion_array_port;
class TceEstimationPort_InterfaceVersion_array_portDefaultTypeInternal;
extern TceEstimationPort_InterfaceVersion_array_portDefaultTypeInternal _TceEstimationPort_InterfaceVersion_array_port_default_instance_;
}  // namespace tce_estimation_port_interface_version
}  // namespace tce
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion* Arena::CreateMaybeMessage<::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion>(Arena*);
template<> ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion_array_port* Arena::CreateMaybeMessage<::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace tce {
namespace tce_estimation_port_interface_version {

// ===================================================================

class TceEstimationPort_InterfaceVersion :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion) */ {
 public:
  TceEstimationPort_InterfaceVersion();
  virtual ~TceEstimationPort_InterfaceVersion();

  TceEstimationPort_InterfaceVersion(const TceEstimationPort_InterfaceVersion& from);
  TceEstimationPort_InterfaceVersion(TceEstimationPort_InterfaceVersion&& from) noexcept
    : TceEstimationPort_InterfaceVersion() {
    *this = ::std::move(from);
  }

  inline TceEstimationPort_InterfaceVersion& operator=(const TceEstimationPort_InterfaceVersion& from) {
    CopyFrom(from);
    return *this;
  }
  inline TceEstimationPort_InterfaceVersion& operator=(TceEstimationPort_InterfaceVersion&& from) noexcept {
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
  static const TceEstimationPort_InterfaceVersion& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TceEstimationPort_InterfaceVersion* internal_default_instance() {
    return reinterpret_cast<const TceEstimationPort_InterfaceVersion*>(
               &_TceEstimationPort_InterfaceVersion_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(TceEstimationPort_InterfaceVersion& a, TceEstimationPort_InterfaceVersion& b) {
    a.Swap(&b);
  }
  inline void Swap(TceEstimationPort_InterfaceVersion* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline TceEstimationPort_InterfaceVersion* New() const final {
    return CreateMaybeMessage<TceEstimationPort_InterfaceVersion>(nullptr);
  }

  TceEstimationPort_InterfaceVersion* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<TceEstimationPort_InterfaceVersion>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const TceEstimationPort_InterfaceVersion& from);
  void MergeFrom(const TceEstimationPort_InterfaceVersion& from);
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
  void InternalSwap(TceEstimationPort_InterfaceVersion* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_tce_2ftce_5festimation_5fport_5finterface_5fversion_2eproto);
    return ::descriptor_table_tce_2ftce_5festimation_5fport_5finterface_5fversion_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kTceEstimationPortVERSIONFieldNumber = 71,
  };
  // optional uint32 TceEstimationPort_VERSION = 71;
  bool has_tceestimationport_version() const;
  private:
  bool _internal_has_tceestimationport_version() const;
  public:
  void clear_tceestimationport_version();
  ::PROTOBUF_NAMESPACE_ID::uint32 tceestimationport_version() const;
  void set_tceestimationport_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_tceestimationport_version() const;
  void _internal_set_tceestimationport_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 tceestimationport_version_;
  friend struct ::TableStruct_tce_2ftce_5festimation_5fport_5finterface_5fversion_2eproto;
};
// -------------------------------------------------------------------

class TceEstimationPort_InterfaceVersion_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion_array_port) */ {
 public:
  TceEstimationPort_InterfaceVersion_array_port();
  virtual ~TceEstimationPort_InterfaceVersion_array_port();

  TceEstimationPort_InterfaceVersion_array_port(const TceEstimationPort_InterfaceVersion_array_port& from);
  TceEstimationPort_InterfaceVersion_array_port(TceEstimationPort_InterfaceVersion_array_port&& from) noexcept
    : TceEstimationPort_InterfaceVersion_array_port() {
    *this = ::std::move(from);
  }

  inline TceEstimationPort_InterfaceVersion_array_port& operator=(const TceEstimationPort_InterfaceVersion_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline TceEstimationPort_InterfaceVersion_array_port& operator=(TceEstimationPort_InterfaceVersion_array_port&& from) noexcept {
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
  static const TceEstimationPort_InterfaceVersion_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TceEstimationPort_InterfaceVersion_array_port* internal_default_instance() {
    return reinterpret_cast<const TceEstimationPort_InterfaceVersion_array_port*>(
               &_TceEstimationPort_InterfaceVersion_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(TceEstimationPort_InterfaceVersion_array_port& a, TceEstimationPort_InterfaceVersion_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(TceEstimationPort_InterfaceVersion_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline TceEstimationPort_InterfaceVersion_array_port* New() const final {
    return CreateMaybeMessage<TceEstimationPort_InterfaceVersion_array_port>(nullptr);
  }

  TceEstimationPort_InterfaceVersion_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<TceEstimationPort_InterfaceVersion_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const TceEstimationPort_InterfaceVersion_array_port& from);
  void MergeFrom(const TceEstimationPort_InterfaceVersion_array_port& from);
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
  void InternalSwap(TceEstimationPort_InterfaceVersion_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_tce_2ftce_5festimation_5fport_5finterface_5fversion_2eproto);
    return ::descriptor_table_tce_2ftce_5festimation_5fport_5finterface_5fversion_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3226,
  };
  // repeated .pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion data = 3226;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion >*
      mutable_data();
  private:
  const ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion& _internal_data(int index) const;
  ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion* _internal_add_data();
  public:
  const ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion& data(int index) const;
  ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion > data_;
  friend struct ::TableStruct_tce_2ftce_5festimation_5fport_5finterface_5fversion_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// TceEstimationPort_InterfaceVersion

// optional uint32 TceEstimationPort_VERSION = 71;
inline bool TceEstimationPort_InterfaceVersion::_internal_has_tceestimationport_version() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool TceEstimationPort_InterfaceVersion::has_tceestimationport_version() const {
  return _internal_has_tceestimationport_version();
}
inline void TceEstimationPort_InterfaceVersion::clear_tceestimationport_version() {
  tceestimationport_version_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TceEstimationPort_InterfaceVersion::_internal_tceestimationport_version() const {
  return tceestimationport_version_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TceEstimationPort_InterfaceVersion::tceestimationport_version() const {
  // @@protoc_insertion_point(field_get:pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion.TceEstimationPort_VERSION)
  return _internal_tceestimationport_version();
}
inline void TceEstimationPort_InterfaceVersion::_internal_set_tceestimationport_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  tceestimationport_version_ = value;
}
inline void TceEstimationPort_InterfaceVersion::set_tceestimationport_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_tceestimationport_version(value);
  // @@protoc_insertion_point(field_set:pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion.TceEstimationPort_VERSION)
}

// -------------------------------------------------------------------

// TceEstimationPort_InterfaceVersion_array_port

// repeated .pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion data = 3226;
inline int TceEstimationPort_InterfaceVersion_array_port::_internal_data_size() const {
  return data_.size();
}
inline int TceEstimationPort_InterfaceVersion_array_port::data_size() const {
  return _internal_data_size();
}
inline void TceEstimationPort_InterfaceVersion_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion* TceEstimationPort_InterfaceVersion_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion >*
TceEstimationPort_InterfaceVersion_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion_array_port.data)
  return &data_;
}
inline const ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion& TceEstimationPort_InterfaceVersion_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion& TceEstimationPort_InterfaceVersion_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion_array_port.data)
  return _internal_data(index);
}
inline ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion* TceEstimationPort_InterfaceVersion_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion* TceEstimationPort_InterfaceVersion_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::tce::tce_estimation_port_interface_version::TceEstimationPort_InterfaceVersion >&
TceEstimationPort_InterfaceVersion_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace tce_estimation_port_interface_version
}  // namespace tce
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_tce_2ftce_5festimation_5fport_5finterface_5fversion_2eproto
