// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_drv/us_drv_asic_cmd_interface_version.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fasic_5fcmd_5finterface_5fversion_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fasic_5fcmd_5finterface_5fversion_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fasic_5fcmd_5finterface_5fversion_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fdrv_2fus_5fdrv_5fasic_5fcmd_5finterface_5fversion_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fdrv_2fus_5fdrv_5fasic_5fcmd_5finterface_5fversion_2eproto;
namespace pb {
namespace us_drv {
namespace us_drv_asic_cmd_interface_version {
class UsDrvAsicCmd_InterfaceVersion;
class UsDrvAsicCmd_InterfaceVersionDefaultTypeInternal;
extern UsDrvAsicCmd_InterfaceVersionDefaultTypeInternal _UsDrvAsicCmd_InterfaceVersion_default_instance_;
class UsDrvAsicCmd_InterfaceVersion_array_port;
class UsDrvAsicCmd_InterfaceVersion_array_portDefaultTypeInternal;
extern UsDrvAsicCmd_InterfaceVersion_array_portDefaultTypeInternal _UsDrvAsicCmd_InterfaceVersion_array_port_default_instance_;
}  // namespace us_drv_asic_cmd_interface_version
}  // namespace us_drv
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion>(Arena*);
template<> ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion_array_port* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_drv {
namespace us_drv_asic_cmd_interface_version {

// ===================================================================

class UsDrvAsicCmd_InterfaceVersion :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion) */ {
 public:
  UsDrvAsicCmd_InterfaceVersion();
  virtual ~UsDrvAsicCmd_InterfaceVersion();

  UsDrvAsicCmd_InterfaceVersion(const UsDrvAsicCmd_InterfaceVersion& from);
  UsDrvAsicCmd_InterfaceVersion(UsDrvAsicCmd_InterfaceVersion&& from) noexcept
    : UsDrvAsicCmd_InterfaceVersion() {
    *this = ::std::move(from);
  }

  inline UsDrvAsicCmd_InterfaceVersion& operator=(const UsDrvAsicCmd_InterfaceVersion& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvAsicCmd_InterfaceVersion& operator=(UsDrvAsicCmd_InterfaceVersion&& from) noexcept {
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
  static const UsDrvAsicCmd_InterfaceVersion& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvAsicCmd_InterfaceVersion* internal_default_instance() {
    return reinterpret_cast<const UsDrvAsicCmd_InterfaceVersion*>(
               &_UsDrvAsicCmd_InterfaceVersion_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsDrvAsicCmd_InterfaceVersion& a, UsDrvAsicCmd_InterfaceVersion& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvAsicCmd_InterfaceVersion* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvAsicCmd_InterfaceVersion* New() const final {
    return CreateMaybeMessage<UsDrvAsicCmd_InterfaceVersion>(nullptr);
  }

  UsDrvAsicCmd_InterfaceVersion* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvAsicCmd_InterfaceVersion>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvAsicCmd_InterfaceVersion& from);
  void MergeFrom(const UsDrvAsicCmd_InterfaceVersion& from);
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
  void InternalSwap(UsDrvAsicCmd_InterfaceVersion* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fasic_5fcmd_5finterface_5fversion_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fasic_5fcmd_5finterface_5fversion_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kUsDrvAsicCmdVERSIONFieldNumber = 394,
  };
  // optional uint32 UsDrvAsicCmd_VERSION = 394;
  bool has_usdrvasiccmd_version() const;
  private:
  bool _internal_has_usdrvasiccmd_version() const;
  public:
  void clear_usdrvasiccmd_version();
  ::PROTOBUF_NAMESPACE_ID::uint32 usdrvasiccmd_version() const;
  void set_usdrvasiccmd_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_usdrvasiccmd_version() const;
  void _internal_set_usdrvasiccmd_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 usdrvasiccmd_version_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fasic_5fcmd_5finterface_5fversion_2eproto;
};
// -------------------------------------------------------------------

class UsDrvAsicCmd_InterfaceVersion_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion_array_port) */ {
 public:
  UsDrvAsicCmd_InterfaceVersion_array_port();
  virtual ~UsDrvAsicCmd_InterfaceVersion_array_port();

  UsDrvAsicCmd_InterfaceVersion_array_port(const UsDrvAsicCmd_InterfaceVersion_array_port& from);
  UsDrvAsicCmd_InterfaceVersion_array_port(UsDrvAsicCmd_InterfaceVersion_array_port&& from) noexcept
    : UsDrvAsicCmd_InterfaceVersion_array_port() {
    *this = ::std::move(from);
  }

  inline UsDrvAsicCmd_InterfaceVersion_array_port& operator=(const UsDrvAsicCmd_InterfaceVersion_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvAsicCmd_InterfaceVersion_array_port& operator=(UsDrvAsicCmd_InterfaceVersion_array_port&& from) noexcept {
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
  static const UsDrvAsicCmd_InterfaceVersion_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvAsicCmd_InterfaceVersion_array_port* internal_default_instance() {
    return reinterpret_cast<const UsDrvAsicCmd_InterfaceVersion_array_port*>(
               &_UsDrvAsicCmd_InterfaceVersion_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsDrvAsicCmd_InterfaceVersion_array_port& a, UsDrvAsicCmd_InterfaceVersion_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvAsicCmd_InterfaceVersion_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvAsicCmd_InterfaceVersion_array_port* New() const final {
    return CreateMaybeMessage<UsDrvAsicCmd_InterfaceVersion_array_port>(nullptr);
  }

  UsDrvAsicCmd_InterfaceVersion_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvAsicCmd_InterfaceVersion_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvAsicCmd_InterfaceVersion_array_port& from);
  void MergeFrom(const UsDrvAsicCmd_InterfaceVersion_array_port& from);
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
  void InternalSwap(UsDrvAsicCmd_InterfaceVersion_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fasic_5fcmd_5finterface_5fversion_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fasic_5fcmd_5finterface_5fversion_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2788,
  };
  // repeated .pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion data = 2788;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion >*
      mutable_data();
  private:
  const ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion& _internal_data(int index) const;
  ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion* _internal_add_data();
  public:
  const ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion& data(int index) const;
  ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion > data_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fasic_5fcmd_5finterface_5fversion_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsDrvAsicCmd_InterfaceVersion

// optional uint32 UsDrvAsicCmd_VERSION = 394;
inline bool UsDrvAsicCmd_InterfaceVersion::_internal_has_usdrvasiccmd_version() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool UsDrvAsicCmd_InterfaceVersion::has_usdrvasiccmd_version() const {
  return _internal_has_usdrvasiccmd_version();
}
inline void UsDrvAsicCmd_InterfaceVersion::clear_usdrvasiccmd_version() {
  usdrvasiccmd_version_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvAsicCmd_InterfaceVersion::_internal_usdrvasiccmd_version() const {
  return usdrvasiccmd_version_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvAsicCmd_InterfaceVersion::usdrvasiccmd_version() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion.UsDrvAsicCmd_VERSION)
  return _internal_usdrvasiccmd_version();
}
inline void UsDrvAsicCmd_InterfaceVersion::_internal_set_usdrvasiccmd_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  usdrvasiccmd_version_ = value;
}
inline void UsDrvAsicCmd_InterfaceVersion::set_usdrvasiccmd_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_usdrvasiccmd_version(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion.UsDrvAsicCmd_VERSION)
}

// -------------------------------------------------------------------

// UsDrvAsicCmd_InterfaceVersion_array_port

// repeated .pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion data = 2788;
inline int UsDrvAsicCmd_InterfaceVersion_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsDrvAsicCmd_InterfaceVersion_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsDrvAsicCmd_InterfaceVersion_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion* UsDrvAsicCmd_InterfaceVersion_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion >*
UsDrvAsicCmd_InterfaceVersion_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion_array_port.data)
  return &data_;
}
inline const ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion& UsDrvAsicCmd_InterfaceVersion_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion& UsDrvAsicCmd_InterfaceVersion_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion* UsDrvAsicCmd_InterfaceVersion_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion* UsDrvAsicCmd_InterfaceVersion_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_asic_cmd_interface_version::UsDrvAsicCmd_InterfaceVersion >&
UsDrvAsicCmd_InterfaceVersion_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_asic_cmd_interface_version.UsDrvAsicCmd_InterfaceVersion_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_drv_asic_cmd_interface_version
}  // namespace us_drv
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fasic_5fcmd_5finterface_5fversion_2eproto
