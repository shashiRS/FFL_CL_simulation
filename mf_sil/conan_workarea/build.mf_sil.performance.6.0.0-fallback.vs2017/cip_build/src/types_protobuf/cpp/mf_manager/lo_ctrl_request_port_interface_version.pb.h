// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_manager/lo_ctrl_request_port_interface_version.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fmanager_2flo_5fctrl_5frequest_5fport_5finterface_5fversion_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fmanager_2flo_5fctrl_5frequest_5fport_5finterface_5fversion_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_mf_5fmanager_2flo_5fctrl_5frequest_5fport_5finterface_5fversion_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fmanager_2flo_5fctrl_5frequest_5fport_5finterface_5fversion_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmanager_2flo_5fctrl_5frequest_5fport_5finterface_5fversion_2eproto;
namespace pb {
namespace mf_manager {
namespace lo_ctrl_request_port_interface_version {
class LoCtrlRequestPort_InterfaceVersion;
class LoCtrlRequestPort_InterfaceVersionDefaultTypeInternal;
extern LoCtrlRequestPort_InterfaceVersionDefaultTypeInternal _LoCtrlRequestPort_InterfaceVersion_default_instance_;
class LoCtrlRequestPort_InterfaceVersion_array_port;
class LoCtrlRequestPort_InterfaceVersion_array_portDefaultTypeInternal;
extern LoCtrlRequestPort_InterfaceVersion_array_portDefaultTypeInternal _LoCtrlRequestPort_InterfaceVersion_array_port_default_instance_;
}  // namespace lo_ctrl_request_port_interface_version
}  // namespace mf_manager
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion* Arena::CreateMaybeMessage<::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion>(Arena*);
template<> ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion_array_port* Arena::CreateMaybeMessage<::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_manager {
namespace lo_ctrl_request_port_interface_version {

// ===================================================================

class LoCtrlRequestPort_InterfaceVersion :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion) */ {
 public:
  LoCtrlRequestPort_InterfaceVersion();
  virtual ~LoCtrlRequestPort_InterfaceVersion();

  LoCtrlRequestPort_InterfaceVersion(const LoCtrlRequestPort_InterfaceVersion& from);
  LoCtrlRequestPort_InterfaceVersion(LoCtrlRequestPort_InterfaceVersion&& from) noexcept
    : LoCtrlRequestPort_InterfaceVersion() {
    *this = ::std::move(from);
  }

  inline LoCtrlRequestPort_InterfaceVersion& operator=(const LoCtrlRequestPort_InterfaceVersion& from) {
    CopyFrom(from);
    return *this;
  }
  inline LoCtrlRequestPort_InterfaceVersion& operator=(LoCtrlRequestPort_InterfaceVersion&& from) noexcept {
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
  static const LoCtrlRequestPort_InterfaceVersion& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LoCtrlRequestPort_InterfaceVersion* internal_default_instance() {
    return reinterpret_cast<const LoCtrlRequestPort_InterfaceVersion*>(
               &_LoCtrlRequestPort_InterfaceVersion_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LoCtrlRequestPort_InterfaceVersion& a, LoCtrlRequestPort_InterfaceVersion& b) {
    a.Swap(&b);
  }
  inline void Swap(LoCtrlRequestPort_InterfaceVersion* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LoCtrlRequestPort_InterfaceVersion* New() const final {
    return CreateMaybeMessage<LoCtrlRequestPort_InterfaceVersion>(nullptr);
  }

  LoCtrlRequestPort_InterfaceVersion* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LoCtrlRequestPort_InterfaceVersion>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LoCtrlRequestPort_InterfaceVersion& from);
  void MergeFrom(const LoCtrlRequestPort_InterfaceVersion& from);
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
  void InternalSwap(LoCtrlRequestPort_InterfaceVersion* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmanager_2flo_5fctrl_5frequest_5fport_5finterface_5fversion_2eproto);
    return ::descriptor_table_mf_5fmanager_2flo_5fctrl_5frequest_5fport_5finterface_5fversion_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kLoCtrlRequestPortVERSIONFieldNumber = 3065,
  };
  // optional uint32 LoCtrlRequestPort_VERSION = 3065;
  bool has_loctrlrequestport_version() const;
  private:
  bool _internal_has_loctrlrequestport_version() const;
  public:
  void clear_loctrlrequestport_version();
  ::PROTOBUF_NAMESPACE_ID::uint32 loctrlrequestport_version() const;
  void set_loctrlrequestport_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_loctrlrequestport_version() const;
  void _internal_set_loctrlrequestport_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 loctrlrequestport_version_;
  friend struct ::TableStruct_mf_5fmanager_2flo_5fctrl_5frequest_5fport_5finterface_5fversion_2eproto;
};
// -------------------------------------------------------------------

class LoCtrlRequestPort_InterfaceVersion_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion_array_port) */ {
 public:
  LoCtrlRequestPort_InterfaceVersion_array_port();
  virtual ~LoCtrlRequestPort_InterfaceVersion_array_port();

  LoCtrlRequestPort_InterfaceVersion_array_port(const LoCtrlRequestPort_InterfaceVersion_array_port& from);
  LoCtrlRequestPort_InterfaceVersion_array_port(LoCtrlRequestPort_InterfaceVersion_array_port&& from) noexcept
    : LoCtrlRequestPort_InterfaceVersion_array_port() {
    *this = ::std::move(from);
  }

  inline LoCtrlRequestPort_InterfaceVersion_array_port& operator=(const LoCtrlRequestPort_InterfaceVersion_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline LoCtrlRequestPort_InterfaceVersion_array_port& operator=(LoCtrlRequestPort_InterfaceVersion_array_port&& from) noexcept {
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
  static const LoCtrlRequestPort_InterfaceVersion_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LoCtrlRequestPort_InterfaceVersion_array_port* internal_default_instance() {
    return reinterpret_cast<const LoCtrlRequestPort_InterfaceVersion_array_port*>(
               &_LoCtrlRequestPort_InterfaceVersion_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(LoCtrlRequestPort_InterfaceVersion_array_port& a, LoCtrlRequestPort_InterfaceVersion_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(LoCtrlRequestPort_InterfaceVersion_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LoCtrlRequestPort_InterfaceVersion_array_port* New() const final {
    return CreateMaybeMessage<LoCtrlRequestPort_InterfaceVersion_array_port>(nullptr);
  }

  LoCtrlRequestPort_InterfaceVersion_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LoCtrlRequestPort_InterfaceVersion_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LoCtrlRequestPort_InterfaceVersion_array_port& from);
  void MergeFrom(const LoCtrlRequestPort_InterfaceVersion_array_port& from);
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
  void InternalSwap(LoCtrlRequestPort_InterfaceVersion_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmanager_2flo_5fctrl_5frequest_5fport_5finterface_5fversion_2eproto);
    return ::descriptor_table_mf_5fmanager_2flo_5fctrl_5frequest_5fport_5finterface_5fversion_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 610,
  };
  // repeated .pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion data = 610;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion >*
      mutable_data();
  private:
  const ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion& _internal_data(int index) const;
  ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion* _internal_add_data();
  public:
  const ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion& data(int index) const;
  ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion > data_;
  friend struct ::TableStruct_mf_5fmanager_2flo_5fctrl_5frequest_5fport_5finterface_5fversion_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LoCtrlRequestPort_InterfaceVersion

// optional uint32 LoCtrlRequestPort_VERSION = 3065;
inline bool LoCtrlRequestPort_InterfaceVersion::_internal_has_loctrlrequestport_version() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool LoCtrlRequestPort_InterfaceVersion::has_loctrlrequestport_version() const {
  return _internal_has_loctrlrequestport_version();
}
inline void LoCtrlRequestPort_InterfaceVersion::clear_loctrlrequestport_version() {
  loctrlrequestport_version_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 LoCtrlRequestPort_InterfaceVersion::_internal_loctrlrequestport_version() const {
  return loctrlrequestport_version_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 LoCtrlRequestPort_InterfaceVersion::loctrlrequestport_version() const {
  // @@protoc_insertion_point(field_get:pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion.LoCtrlRequestPort_VERSION)
  return _internal_loctrlrequestport_version();
}
inline void LoCtrlRequestPort_InterfaceVersion::_internal_set_loctrlrequestport_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  loctrlrequestport_version_ = value;
}
inline void LoCtrlRequestPort_InterfaceVersion::set_loctrlrequestport_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_loctrlrequestport_version(value);
  // @@protoc_insertion_point(field_set:pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion.LoCtrlRequestPort_VERSION)
}

// -------------------------------------------------------------------

// LoCtrlRequestPort_InterfaceVersion_array_port

// repeated .pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion data = 610;
inline int LoCtrlRequestPort_InterfaceVersion_array_port::_internal_data_size() const {
  return data_.size();
}
inline int LoCtrlRequestPort_InterfaceVersion_array_port::data_size() const {
  return _internal_data_size();
}
inline void LoCtrlRequestPort_InterfaceVersion_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion* LoCtrlRequestPort_InterfaceVersion_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion >*
LoCtrlRequestPort_InterfaceVersion_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion_array_port.data)
  return &data_;
}
inline const ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion& LoCtrlRequestPort_InterfaceVersion_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion& LoCtrlRequestPort_InterfaceVersion_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion* LoCtrlRequestPort_InterfaceVersion_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion* LoCtrlRequestPort_InterfaceVersion_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::lo_ctrl_request_port_interface_version::LoCtrlRequestPort_InterfaceVersion >&
LoCtrlRequestPort_InterfaceVersion_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_manager.lo_ctrl_request_port_interface_version.LoCtrlRequestPort_InterfaceVersion_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace lo_ctrl_request_port_interface_version
}  // namespace mf_manager
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fmanager_2flo_5fctrl_5frequest_5fport_5finterface_5fversion_2eproto
