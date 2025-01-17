// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/triangle_serializable.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_si_2ftriangle_5fserializable_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_si_2ftriangle_5fserializable_2eproto

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
#include "cml/vec2_df_pod.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_si_2ftriangle_5fserializable_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_si_2ftriangle_5fserializable_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2ftriangle_5fserializable_2eproto;
namespace pb {
namespace si {
namespace triangle_serializable {
class TriangleSerializable;
class TriangleSerializableDefaultTypeInternal;
extern TriangleSerializableDefaultTypeInternal _TriangleSerializable_default_instance_;
class TriangleSerializable_array_port;
class TriangleSerializable_array_portDefaultTypeInternal;
extern TriangleSerializable_array_portDefaultTypeInternal _TriangleSerializable_array_port_default_instance_;
}  // namespace triangle_serializable
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::si::triangle_serializable::TriangleSerializable* Arena::CreateMaybeMessage<::pb::si::triangle_serializable::TriangleSerializable>(Arena*);
template<> ::pb::si::triangle_serializable::TriangleSerializable_array_port* Arena::CreateMaybeMessage<::pb::si::triangle_serializable::TriangleSerializable_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace si {
namespace triangle_serializable {

// ===================================================================

class TriangleSerializable :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.triangle_serializable.TriangleSerializable) */ {
 public:
  TriangleSerializable();
  virtual ~TriangleSerializable();

  TriangleSerializable(const TriangleSerializable& from);
  TriangleSerializable(TriangleSerializable&& from) noexcept
    : TriangleSerializable() {
    *this = ::std::move(from);
  }

  inline TriangleSerializable& operator=(const TriangleSerializable& from) {
    CopyFrom(from);
    return *this;
  }
  inline TriangleSerializable& operator=(TriangleSerializable&& from) noexcept {
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
  static const TriangleSerializable& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TriangleSerializable* internal_default_instance() {
    return reinterpret_cast<const TriangleSerializable*>(
               &_TriangleSerializable_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(TriangleSerializable& a, TriangleSerializable& b) {
    a.Swap(&b);
  }
  inline void Swap(TriangleSerializable* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline TriangleSerializable* New() const final {
    return CreateMaybeMessage<TriangleSerializable>(nullptr);
  }

  TriangleSerializable* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<TriangleSerializable>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const TriangleSerializable& from);
  void MergeFrom(const TriangleSerializable& from);
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
  void InternalSwap(TriangleSerializable* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.triangle_serializable.TriangleSerializable";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2ftriangle_5fserializable_2eproto);
    return ::descriptor_table_si_2ftriangle_5fserializable_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kArrayFieldNumber = 3129,
    kActualSizeFieldNumber = 3009,
  };
  // repeated .pb.cml.vec2_df_pod.Vec2Df_POD array = 3129;
  int array_size() const;
  private:
  int _internal_array_size() const;
  public:
  void clear_array();
  ::pb::cml::vec2_df_pod::Vec2Df_POD* mutable_array(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD >*
      mutable_array();
  private:
  const ::pb::cml::vec2_df_pod::Vec2Df_POD& _internal_array(int index) const;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* _internal_add_array();
  public:
  const ::pb::cml::vec2_df_pod::Vec2Df_POD& array(int index) const;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* add_array();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD >&
      array() const;

  // optional uint32 actualSize = 3009;
  bool has_actualsize() const;
  private:
  bool _internal_has_actualsize() const;
  public:
  void clear_actualsize();
  ::PROTOBUF_NAMESPACE_ID::uint32 actualsize() const;
  void set_actualsize(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_actualsize() const;
  void _internal_set_actualsize(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.si.triangle_serializable.TriangleSerializable)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD > array_;
  ::PROTOBUF_NAMESPACE_ID::uint32 actualsize_;
  friend struct ::TableStruct_si_2ftriangle_5fserializable_2eproto;
};
// -------------------------------------------------------------------

class TriangleSerializable_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.triangle_serializable.TriangleSerializable_array_port) */ {
 public:
  TriangleSerializable_array_port();
  virtual ~TriangleSerializable_array_port();

  TriangleSerializable_array_port(const TriangleSerializable_array_port& from);
  TriangleSerializable_array_port(TriangleSerializable_array_port&& from) noexcept
    : TriangleSerializable_array_port() {
    *this = ::std::move(from);
  }

  inline TriangleSerializable_array_port& operator=(const TriangleSerializable_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline TriangleSerializable_array_port& operator=(TriangleSerializable_array_port&& from) noexcept {
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
  static const TriangleSerializable_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TriangleSerializable_array_port* internal_default_instance() {
    return reinterpret_cast<const TriangleSerializable_array_port*>(
               &_TriangleSerializable_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(TriangleSerializable_array_port& a, TriangleSerializable_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(TriangleSerializable_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline TriangleSerializable_array_port* New() const final {
    return CreateMaybeMessage<TriangleSerializable_array_port>(nullptr);
  }

  TriangleSerializable_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<TriangleSerializable_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const TriangleSerializable_array_port& from);
  void MergeFrom(const TriangleSerializable_array_port& from);
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
  void InternalSwap(TriangleSerializable_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.triangle_serializable.TriangleSerializable_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2ftriangle_5fserializable_2eproto);
    return ::descriptor_table_si_2ftriangle_5fserializable_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1696,
  };
  // repeated .pb.si.triangle_serializable.TriangleSerializable data = 1696;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::si::triangle_serializable::TriangleSerializable* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::triangle_serializable::TriangleSerializable >*
      mutable_data();
  private:
  const ::pb::si::triangle_serializable::TriangleSerializable& _internal_data(int index) const;
  ::pb::si::triangle_serializable::TriangleSerializable* _internal_add_data();
  public:
  const ::pb::si::triangle_serializable::TriangleSerializable& data(int index) const;
  ::pb::si::triangle_serializable::TriangleSerializable* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::triangle_serializable::TriangleSerializable >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.si.triangle_serializable.TriangleSerializable_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::triangle_serializable::TriangleSerializable > data_;
  friend struct ::TableStruct_si_2ftriangle_5fserializable_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// TriangleSerializable

// optional uint32 actualSize = 3009;
inline bool TriangleSerializable::_internal_has_actualsize() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool TriangleSerializable::has_actualsize() const {
  return _internal_has_actualsize();
}
inline void TriangleSerializable::clear_actualsize() {
  actualsize_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TriangleSerializable::_internal_actualsize() const {
  return actualsize_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TriangleSerializable::actualsize() const {
  // @@protoc_insertion_point(field_get:pb.si.triangle_serializable.TriangleSerializable.actualSize)
  return _internal_actualsize();
}
inline void TriangleSerializable::_internal_set_actualsize(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  actualsize_ = value;
}
inline void TriangleSerializable::set_actualsize(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_actualsize(value);
  // @@protoc_insertion_point(field_set:pb.si.triangle_serializable.TriangleSerializable.actualSize)
}

// repeated .pb.cml.vec2_df_pod.Vec2Df_POD array = 3129;
inline int TriangleSerializable::_internal_array_size() const {
  return array_.size();
}
inline int TriangleSerializable::array_size() const {
  return _internal_array_size();
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* TriangleSerializable::mutable_array(int index) {
  // @@protoc_insertion_point(field_mutable:pb.si.triangle_serializable.TriangleSerializable.array)
  return array_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD >*
TriangleSerializable::mutable_array() {
  // @@protoc_insertion_point(field_mutable_list:pb.si.triangle_serializable.TriangleSerializable.array)
  return &array_;
}
inline const ::pb::cml::vec2_df_pod::Vec2Df_POD& TriangleSerializable::_internal_array(int index) const {
  return array_.Get(index);
}
inline const ::pb::cml::vec2_df_pod::Vec2Df_POD& TriangleSerializable::array(int index) const {
  // @@protoc_insertion_point(field_get:pb.si.triangle_serializable.TriangleSerializable.array)
  return _internal_array(index);
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* TriangleSerializable::_internal_add_array() {
  return array_.Add();
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* TriangleSerializable::add_array() {
  // @@protoc_insertion_point(field_add:pb.si.triangle_serializable.TriangleSerializable.array)
  return _internal_add_array();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD >&
TriangleSerializable::array() const {
  // @@protoc_insertion_point(field_list:pb.si.triangle_serializable.TriangleSerializable.array)
  return array_;
}

// -------------------------------------------------------------------

// TriangleSerializable_array_port

// repeated .pb.si.triangle_serializable.TriangleSerializable data = 1696;
inline int TriangleSerializable_array_port::_internal_data_size() const {
  return data_.size();
}
inline int TriangleSerializable_array_port::data_size() const {
  return _internal_data_size();
}
inline void TriangleSerializable_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::si::triangle_serializable::TriangleSerializable* TriangleSerializable_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.si.triangle_serializable.TriangleSerializable_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::triangle_serializable::TriangleSerializable >*
TriangleSerializable_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.si.triangle_serializable.TriangleSerializable_array_port.data)
  return &data_;
}
inline const ::pb::si::triangle_serializable::TriangleSerializable& TriangleSerializable_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::si::triangle_serializable::TriangleSerializable& TriangleSerializable_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.si.triangle_serializable.TriangleSerializable_array_port.data)
  return _internal_data(index);
}
inline ::pb::si::triangle_serializable::TriangleSerializable* TriangleSerializable_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::si::triangle_serializable::TriangleSerializable* TriangleSerializable_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.si.triangle_serializable.TriangleSerializable_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::triangle_serializable::TriangleSerializable >&
TriangleSerializable_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.si.triangle_serializable.TriangleSerializable_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace triangle_serializable
}  // namespace si
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_si_2ftriangle_5fserializable_2eproto
