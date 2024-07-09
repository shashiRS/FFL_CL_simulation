// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/coll_env_model_port_interface_version.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_si_2fcoll_5fenv_5fmodel_5fport_5finterface_5fversion_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_si_2fcoll_5fenv_5fmodel_5fport_5finterface_5fversion_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_si_2fcoll_5fenv_5fmodel_5fport_5finterface_5fversion_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_si_2fcoll_5fenv_5fmodel_5fport_5finterface_5fversion_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fcoll_5fenv_5fmodel_5fport_5finterface_5fversion_2eproto;
namespace pb {
namespace si {
namespace coll_env_model_port_interface_version {
class CollEnvModelPort_InterfaceVersion;
class CollEnvModelPort_InterfaceVersionDefaultTypeInternal;
extern CollEnvModelPort_InterfaceVersionDefaultTypeInternal _CollEnvModelPort_InterfaceVersion_default_instance_;
class CollEnvModelPort_InterfaceVersion_array_port;
class CollEnvModelPort_InterfaceVersion_array_portDefaultTypeInternal;
extern CollEnvModelPort_InterfaceVersion_array_portDefaultTypeInternal _CollEnvModelPort_InterfaceVersion_array_port_default_instance_;
}  // namespace coll_env_model_port_interface_version
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion* Arena::CreateMaybeMessage<::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion>(Arena*);
template<> ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion_array_port* Arena::CreateMaybeMessage<::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace si {
namespace coll_env_model_port_interface_version {

// ===================================================================

class CollEnvModelPort_InterfaceVersion :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion) */ {
 public:
  CollEnvModelPort_InterfaceVersion();
  virtual ~CollEnvModelPort_InterfaceVersion();

  CollEnvModelPort_InterfaceVersion(const CollEnvModelPort_InterfaceVersion& from);
  CollEnvModelPort_InterfaceVersion(CollEnvModelPort_InterfaceVersion&& from) noexcept
    : CollEnvModelPort_InterfaceVersion() {
    *this = ::std::move(from);
  }

  inline CollEnvModelPort_InterfaceVersion& operator=(const CollEnvModelPort_InterfaceVersion& from) {
    CopyFrom(from);
    return *this;
  }
  inline CollEnvModelPort_InterfaceVersion& operator=(CollEnvModelPort_InterfaceVersion&& from) noexcept {
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
  static const CollEnvModelPort_InterfaceVersion& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CollEnvModelPort_InterfaceVersion* internal_default_instance() {
    return reinterpret_cast<const CollEnvModelPort_InterfaceVersion*>(
               &_CollEnvModelPort_InterfaceVersion_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(CollEnvModelPort_InterfaceVersion& a, CollEnvModelPort_InterfaceVersion& b) {
    a.Swap(&b);
  }
  inline void Swap(CollEnvModelPort_InterfaceVersion* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline CollEnvModelPort_InterfaceVersion* New() const final {
    return CreateMaybeMessage<CollEnvModelPort_InterfaceVersion>(nullptr);
  }

  CollEnvModelPort_InterfaceVersion* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<CollEnvModelPort_InterfaceVersion>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const CollEnvModelPort_InterfaceVersion& from);
  void MergeFrom(const CollEnvModelPort_InterfaceVersion& from);
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
  void InternalSwap(CollEnvModelPort_InterfaceVersion* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fcoll_5fenv_5fmodel_5fport_5finterface_5fversion_2eproto);
    return ::descriptor_table_si_2fcoll_5fenv_5fmodel_5fport_5finterface_5fversion_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kCollEnvModelPortVERSIONFieldNumber = 2484,
  };
  // optional uint32 CollEnvModelPort_VERSION = 2484;
  bool has_collenvmodelport_version() const;
  private:
  bool _internal_has_collenvmodelport_version() const;
  public:
  void clear_collenvmodelport_version();
  ::PROTOBUF_NAMESPACE_ID::uint32 collenvmodelport_version() const;
  void set_collenvmodelport_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_collenvmodelport_version() const;
  void _internal_set_collenvmodelport_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 collenvmodelport_version_;
  friend struct ::TableStruct_si_2fcoll_5fenv_5fmodel_5fport_5finterface_5fversion_2eproto;
};
// -------------------------------------------------------------------

class CollEnvModelPort_InterfaceVersion_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion_array_port) */ {
 public:
  CollEnvModelPort_InterfaceVersion_array_port();
  virtual ~CollEnvModelPort_InterfaceVersion_array_port();

  CollEnvModelPort_InterfaceVersion_array_port(const CollEnvModelPort_InterfaceVersion_array_port& from);
  CollEnvModelPort_InterfaceVersion_array_port(CollEnvModelPort_InterfaceVersion_array_port&& from) noexcept
    : CollEnvModelPort_InterfaceVersion_array_port() {
    *this = ::std::move(from);
  }

  inline CollEnvModelPort_InterfaceVersion_array_port& operator=(const CollEnvModelPort_InterfaceVersion_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline CollEnvModelPort_InterfaceVersion_array_port& operator=(CollEnvModelPort_InterfaceVersion_array_port&& from) noexcept {
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
  static const CollEnvModelPort_InterfaceVersion_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CollEnvModelPort_InterfaceVersion_array_port* internal_default_instance() {
    return reinterpret_cast<const CollEnvModelPort_InterfaceVersion_array_port*>(
               &_CollEnvModelPort_InterfaceVersion_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(CollEnvModelPort_InterfaceVersion_array_port& a, CollEnvModelPort_InterfaceVersion_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(CollEnvModelPort_InterfaceVersion_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline CollEnvModelPort_InterfaceVersion_array_port* New() const final {
    return CreateMaybeMessage<CollEnvModelPort_InterfaceVersion_array_port>(nullptr);
  }

  CollEnvModelPort_InterfaceVersion_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<CollEnvModelPort_InterfaceVersion_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const CollEnvModelPort_InterfaceVersion_array_port& from);
  void MergeFrom(const CollEnvModelPort_InterfaceVersion_array_port& from);
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
  void InternalSwap(CollEnvModelPort_InterfaceVersion_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fcoll_5fenv_5fmodel_5fport_5finterface_5fversion_2eproto);
    return ::descriptor_table_si_2fcoll_5fenv_5fmodel_5fport_5finterface_5fversion_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2220,
  };
  // repeated .pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion data = 2220;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion >*
      mutable_data();
  private:
  const ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion& _internal_data(int index) const;
  ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion* _internal_add_data();
  public:
  const ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion& data(int index) const;
  ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion > data_;
  friend struct ::TableStruct_si_2fcoll_5fenv_5fmodel_5fport_5finterface_5fversion_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// CollEnvModelPort_InterfaceVersion

// optional uint32 CollEnvModelPort_VERSION = 2484;
inline bool CollEnvModelPort_InterfaceVersion::_internal_has_collenvmodelport_version() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool CollEnvModelPort_InterfaceVersion::has_collenvmodelport_version() const {
  return _internal_has_collenvmodelport_version();
}
inline void CollEnvModelPort_InterfaceVersion::clear_collenvmodelport_version() {
  collenvmodelport_version_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 CollEnvModelPort_InterfaceVersion::_internal_collenvmodelport_version() const {
  return collenvmodelport_version_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 CollEnvModelPort_InterfaceVersion::collenvmodelport_version() const {
  // @@protoc_insertion_point(field_get:pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion.CollEnvModelPort_VERSION)
  return _internal_collenvmodelport_version();
}
inline void CollEnvModelPort_InterfaceVersion::_internal_set_collenvmodelport_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  collenvmodelport_version_ = value;
}
inline void CollEnvModelPort_InterfaceVersion::set_collenvmodelport_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_collenvmodelport_version(value);
  // @@protoc_insertion_point(field_set:pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion.CollEnvModelPort_VERSION)
}

// -------------------------------------------------------------------

// CollEnvModelPort_InterfaceVersion_array_port

// repeated .pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion data = 2220;
inline int CollEnvModelPort_InterfaceVersion_array_port::_internal_data_size() const {
  return data_.size();
}
inline int CollEnvModelPort_InterfaceVersion_array_port::data_size() const {
  return _internal_data_size();
}
inline void CollEnvModelPort_InterfaceVersion_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion* CollEnvModelPort_InterfaceVersion_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion >*
CollEnvModelPort_InterfaceVersion_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion_array_port.data)
  return &data_;
}
inline const ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion& CollEnvModelPort_InterfaceVersion_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion& CollEnvModelPort_InterfaceVersion_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion_array_port.data)
  return _internal_data(index);
}
inline ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion* CollEnvModelPort_InterfaceVersion_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion* CollEnvModelPort_InterfaceVersion_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::coll_env_model_port_interface_version::CollEnvModelPort_InterfaceVersion >&
CollEnvModelPort_InterfaceVersion_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.si.coll_env_model_port_interface_version.CollEnvModelPort_InterfaceVersion_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace coll_env_model_port_interface_version
}  // namespace si
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_si_2fcoll_5fenv_5fmodel_5fport_5finterface_5fversion_2eproto