// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/virtual_line_serializable.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_si_2fvirtual_5fline_5fserializable_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_si_2fvirtual_5fline_5fserializable_2eproto

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
#include "si/virt_line_vertices_m_serializable.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_si_2fvirtual_5fline_5fserializable_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_si_2fvirtual_5fline_5fserializable_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fvirtual_5fline_5fserializable_2eproto;
namespace pb {
namespace si {
namespace virtual_line_serializable {
class VirtualLineSerializable;
class VirtualLineSerializableDefaultTypeInternal;
extern VirtualLineSerializableDefaultTypeInternal _VirtualLineSerializable_default_instance_;
class VirtualLineSerializable_array_port;
class VirtualLineSerializable_array_portDefaultTypeInternal;
extern VirtualLineSerializable_array_portDefaultTypeInternal _VirtualLineSerializable_array_port_default_instance_;
}  // namespace virtual_line_serializable
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::si::virtual_line_serializable::VirtualLineSerializable* Arena::CreateMaybeMessage<::pb::si::virtual_line_serializable::VirtualLineSerializable>(Arena*);
template<> ::pb::si::virtual_line_serializable::VirtualLineSerializable_array_port* Arena::CreateMaybeMessage<::pb::si::virtual_line_serializable::VirtualLineSerializable_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace si {
namespace virtual_line_serializable {

// ===================================================================

class VirtualLineSerializable :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.virtual_line_serializable.VirtualLineSerializable) */ {
 public:
  VirtualLineSerializable();
  virtual ~VirtualLineSerializable();

  VirtualLineSerializable(const VirtualLineSerializable& from);
  VirtualLineSerializable(VirtualLineSerializable&& from) noexcept
    : VirtualLineSerializable() {
    *this = ::std::move(from);
  }

  inline VirtualLineSerializable& operator=(const VirtualLineSerializable& from) {
    CopyFrom(from);
    return *this;
  }
  inline VirtualLineSerializable& operator=(VirtualLineSerializable&& from) noexcept {
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
  static const VirtualLineSerializable& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const VirtualLineSerializable* internal_default_instance() {
    return reinterpret_cast<const VirtualLineSerializable*>(
               &_VirtualLineSerializable_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(VirtualLineSerializable& a, VirtualLineSerializable& b) {
    a.Swap(&b);
  }
  inline void Swap(VirtualLineSerializable* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline VirtualLineSerializable* New() const final {
    return CreateMaybeMessage<VirtualLineSerializable>(nullptr);
  }

  VirtualLineSerializable* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<VirtualLineSerializable>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const VirtualLineSerializable& from);
  void MergeFrom(const VirtualLineSerializable& from);
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
  void InternalSwap(VirtualLineSerializable* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.virtual_line_serializable.VirtualLineSerializable";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fvirtual_5fline_5fserializable_2eproto);
    return ::descriptor_table_si_2fvirtual_5fline_5fserializable_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kVirtLineVerticesMFieldNumber = 3517,
  };
  // optional .pb.si.virt_line_vertices_m_serializable.VirtLineVertices_mSerializable virtLineVertices_m = 3517;
  bool has_virtlinevertices_m() const;
  private:
  bool _internal_has_virtlinevertices_m() const;
  public:
  void clear_virtlinevertices_m();
  const ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable& virtlinevertices_m() const;
  ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable* release_virtlinevertices_m();
  ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable* mutable_virtlinevertices_m();
  void set_allocated_virtlinevertices_m(::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable* virtlinevertices_m);
  private:
  const ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable& _internal_virtlinevertices_m() const;
  ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable* _internal_mutable_virtlinevertices_m();
  public:

  // @@protoc_insertion_point(class_scope:pb.si.virtual_line_serializable.VirtualLineSerializable)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable* virtlinevertices_m_;
  friend struct ::TableStruct_si_2fvirtual_5fline_5fserializable_2eproto;
};
// -------------------------------------------------------------------

class VirtualLineSerializable_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.virtual_line_serializable.VirtualLineSerializable_array_port) */ {
 public:
  VirtualLineSerializable_array_port();
  virtual ~VirtualLineSerializable_array_port();

  VirtualLineSerializable_array_port(const VirtualLineSerializable_array_port& from);
  VirtualLineSerializable_array_port(VirtualLineSerializable_array_port&& from) noexcept
    : VirtualLineSerializable_array_port() {
    *this = ::std::move(from);
  }

  inline VirtualLineSerializable_array_port& operator=(const VirtualLineSerializable_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline VirtualLineSerializable_array_port& operator=(VirtualLineSerializable_array_port&& from) noexcept {
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
  static const VirtualLineSerializable_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const VirtualLineSerializable_array_port* internal_default_instance() {
    return reinterpret_cast<const VirtualLineSerializable_array_port*>(
               &_VirtualLineSerializable_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(VirtualLineSerializable_array_port& a, VirtualLineSerializable_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(VirtualLineSerializable_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline VirtualLineSerializable_array_port* New() const final {
    return CreateMaybeMessage<VirtualLineSerializable_array_port>(nullptr);
  }

  VirtualLineSerializable_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<VirtualLineSerializable_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const VirtualLineSerializable_array_port& from);
  void MergeFrom(const VirtualLineSerializable_array_port& from);
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
  void InternalSwap(VirtualLineSerializable_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.virtual_line_serializable.VirtualLineSerializable_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fvirtual_5fline_5fserializable_2eproto);
    return ::descriptor_table_si_2fvirtual_5fline_5fserializable_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 876,
  };
  // repeated .pb.si.virtual_line_serializable.VirtualLineSerializable data = 876;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::si::virtual_line_serializable::VirtualLineSerializable* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::virtual_line_serializable::VirtualLineSerializable >*
      mutable_data();
  private:
  const ::pb::si::virtual_line_serializable::VirtualLineSerializable& _internal_data(int index) const;
  ::pb::si::virtual_line_serializable::VirtualLineSerializable* _internal_add_data();
  public:
  const ::pb::si::virtual_line_serializable::VirtualLineSerializable& data(int index) const;
  ::pb::si::virtual_line_serializable::VirtualLineSerializable* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::virtual_line_serializable::VirtualLineSerializable >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.si.virtual_line_serializable.VirtualLineSerializable_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::virtual_line_serializable::VirtualLineSerializable > data_;
  friend struct ::TableStruct_si_2fvirtual_5fline_5fserializable_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// VirtualLineSerializable

// optional .pb.si.virt_line_vertices_m_serializable.VirtLineVertices_mSerializable virtLineVertices_m = 3517;
inline bool VirtualLineSerializable::_internal_has_virtlinevertices_m() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || virtlinevertices_m_ != nullptr);
  return value;
}
inline bool VirtualLineSerializable::has_virtlinevertices_m() const {
  return _internal_has_virtlinevertices_m();
}
inline const ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable& VirtualLineSerializable::_internal_virtlinevertices_m() const {
  const ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable* p = virtlinevertices_m_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable*>(
      &::pb::si::virt_line_vertices_m_serializable::_VirtLineVertices_mSerializable_default_instance_);
}
inline const ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable& VirtualLineSerializable::virtlinevertices_m() const {
  // @@protoc_insertion_point(field_get:pb.si.virtual_line_serializable.VirtualLineSerializable.virtLineVertices_m)
  return _internal_virtlinevertices_m();
}
inline ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable* VirtualLineSerializable::release_virtlinevertices_m() {
  // @@protoc_insertion_point(field_release:pb.si.virtual_line_serializable.VirtualLineSerializable.virtLineVertices_m)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable* temp = virtlinevertices_m_;
  virtlinevertices_m_ = nullptr;
  return temp;
}
inline ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable* VirtualLineSerializable::_internal_mutable_virtlinevertices_m() {
  _has_bits_[0] |= 0x00000001u;
  if (virtlinevertices_m_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable>(GetArenaNoVirtual());
    virtlinevertices_m_ = p;
  }
  return virtlinevertices_m_;
}
inline ::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable* VirtualLineSerializable::mutable_virtlinevertices_m() {
  // @@protoc_insertion_point(field_mutable:pb.si.virtual_line_serializable.VirtualLineSerializable.virtLineVertices_m)
  return _internal_mutable_virtlinevertices_m();
}
inline void VirtualLineSerializable::set_allocated_virtlinevertices_m(::pb::si::virt_line_vertices_m_serializable::VirtLineVertices_mSerializable* virtlinevertices_m) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(virtlinevertices_m_);
  }
  if (virtlinevertices_m) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      virtlinevertices_m = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, virtlinevertices_m, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  virtlinevertices_m_ = virtlinevertices_m;
  // @@protoc_insertion_point(field_set_allocated:pb.si.virtual_line_serializable.VirtualLineSerializable.virtLineVertices_m)
}

// -------------------------------------------------------------------

// VirtualLineSerializable_array_port

// repeated .pb.si.virtual_line_serializable.VirtualLineSerializable data = 876;
inline int VirtualLineSerializable_array_port::_internal_data_size() const {
  return data_.size();
}
inline int VirtualLineSerializable_array_port::data_size() const {
  return _internal_data_size();
}
inline void VirtualLineSerializable_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::si::virtual_line_serializable::VirtualLineSerializable* VirtualLineSerializable_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.si.virtual_line_serializable.VirtualLineSerializable_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::virtual_line_serializable::VirtualLineSerializable >*
VirtualLineSerializable_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.si.virtual_line_serializable.VirtualLineSerializable_array_port.data)
  return &data_;
}
inline const ::pb::si::virtual_line_serializable::VirtualLineSerializable& VirtualLineSerializable_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::si::virtual_line_serializable::VirtualLineSerializable& VirtualLineSerializable_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.si.virtual_line_serializable.VirtualLineSerializable_array_port.data)
  return _internal_data(index);
}
inline ::pb::si::virtual_line_serializable::VirtualLineSerializable* VirtualLineSerializable_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::si::virtual_line_serializable::VirtualLineSerializable* VirtualLineSerializable_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.si.virtual_line_serializable.VirtualLineSerializable_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::virtual_line_serializable::VirtualLineSerializable >&
VirtualLineSerializable_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.si.virtual_line_serializable.VirtualLineSerializable_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace virtual_line_serializable
}  // namespace si
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_si_2fvirtual_5fline_5fserializable_2eproto
