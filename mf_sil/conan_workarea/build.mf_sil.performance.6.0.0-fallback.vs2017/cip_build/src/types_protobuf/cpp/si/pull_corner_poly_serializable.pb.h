// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/pull_corner_poly_serializable.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_si_2fpull_5fcorner_5fpoly_5fserializable_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_si_2fpull_5fcorner_5fpoly_5fserializable_2eproto

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
#include "si/pull_corner_poly.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_si_2fpull_5fcorner_5fpoly_5fserializable_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_si_2fpull_5fcorner_5fpoly_5fserializable_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fpull_5fcorner_5fpoly_5fserializable_2eproto;
namespace pb {
namespace si {
namespace pull_corner_poly_serializable {
class PullCornerPolySerializable;
class PullCornerPolySerializableDefaultTypeInternal;
extern PullCornerPolySerializableDefaultTypeInternal _PullCornerPolySerializable_default_instance_;
class PullCornerPolySerializable_array_port;
class PullCornerPolySerializable_array_portDefaultTypeInternal;
extern PullCornerPolySerializable_array_portDefaultTypeInternal _PullCornerPolySerializable_array_port_default_instance_;
}  // namespace pull_corner_poly_serializable
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable* Arena::CreateMaybeMessage<::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable>(Arena*);
template<> ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable_array_port* Arena::CreateMaybeMessage<::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace si {
namespace pull_corner_poly_serializable {

// ===================================================================

class PullCornerPolySerializable :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable) */ {
 public:
  PullCornerPolySerializable();
  virtual ~PullCornerPolySerializable();

  PullCornerPolySerializable(const PullCornerPolySerializable& from);
  PullCornerPolySerializable(PullCornerPolySerializable&& from) noexcept
    : PullCornerPolySerializable() {
    *this = ::std::move(from);
  }

  inline PullCornerPolySerializable& operator=(const PullCornerPolySerializable& from) {
    CopyFrom(from);
    return *this;
  }
  inline PullCornerPolySerializable& operator=(PullCornerPolySerializable&& from) noexcept {
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
  static const PullCornerPolySerializable& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PullCornerPolySerializable* internal_default_instance() {
    return reinterpret_cast<const PullCornerPolySerializable*>(
               &_PullCornerPolySerializable_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PullCornerPolySerializable& a, PullCornerPolySerializable& b) {
    a.Swap(&b);
  }
  inline void Swap(PullCornerPolySerializable* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PullCornerPolySerializable* New() const final {
    return CreateMaybeMessage<PullCornerPolySerializable>(nullptr);
  }

  PullCornerPolySerializable* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PullCornerPolySerializable>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PullCornerPolySerializable& from);
  void MergeFrom(const PullCornerPolySerializable& from);
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
  void InternalSwap(PullCornerPolySerializable* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.pull_corner_poly_serializable.PullCornerPolySerializable";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fpull_5fcorner_5fpoly_5fserializable_2eproto);
    return ::descriptor_table_si_2fpull_5fcorner_5fpoly_5fserializable_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kArrayFieldNumber = 417,
    kActualSizeFieldNumber = 3009,
  };
  // repeated .pb.si.pull_corner_poly.PullCornerPoly array = 417;
  int array_size() const;
  private:
  int _internal_array_size() const;
  public:
  void clear_array();
  ::pb::si::pull_corner_poly::PullCornerPoly* mutable_array(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::pull_corner_poly::PullCornerPoly >*
      mutable_array();
  private:
  const ::pb::si::pull_corner_poly::PullCornerPoly& _internal_array(int index) const;
  ::pb::si::pull_corner_poly::PullCornerPoly* _internal_add_array();
  public:
  const ::pb::si::pull_corner_poly::PullCornerPoly& array(int index) const;
  ::pb::si::pull_corner_poly::PullCornerPoly* add_array();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::pull_corner_poly::PullCornerPoly >&
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

  // @@protoc_insertion_point(class_scope:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::pull_corner_poly::PullCornerPoly > array_;
  ::PROTOBUF_NAMESPACE_ID::uint32 actualsize_;
  friend struct ::TableStruct_si_2fpull_5fcorner_5fpoly_5fserializable_2eproto;
};
// -------------------------------------------------------------------

class PullCornerPolySerializable_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable_array_port) */ {
 public:
  PullCornerPolySerializable_array_port();
  virtual ~PullCornerPolySerializable_array_port();

  PullCornerPolySerializable_array_port(const PullCornerPolySerializable_array_port& from);
  PullCornerPolySerializable_array_port(PullCornerPolySerializable_array_port&& from) noexcept
    : PullCornerPolySerializable_array_port() {
    *this = ::std::move(from);
  }

  inline PullCornerPolySerializable_array_port& operator=(const PullCornerPolySerializable_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline PullCornerPolySerializable_array_port& operator=(PullCornerPolySerializable_array_port&& from) noexcept {
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
  static const PullCornerPolySerializable_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PullCornerPolySerializable_array_port* internal_default_instance() {
    return reinterpret_cast<const PullCornerPolySerializable_array_port*>(
               &_PullCornerPolySerializable_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(PullCornerPolySerializable_array_port& a, PullCornerPolySerializable_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(PullCornerPolySerializable_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PullCornerPolySerializable_array_port* New() const final {
    return CreateMaybeMessage<PullCornerPolySerializable_array_port>(nullptr);
  }

  PullCornerPolySerializable_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PullCornerPolySerializable_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PullCornerPolySerializable_array_port& from);
  void MergeFrom(const PullCornerPolySerializable_array_port& from);
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
  void InternalSwap(PullCornerPolySerializable_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.pull_corner_poly_serializable.PullCornerPolySerializable_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fpull_5fcorner_5fpoly_5fserializable_2eproto);
    return ::descriptor_table_si_2fpull_5fcorner_5fpoly_5fserializable_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 962,
  };
  // repeated .pb.si.pull_corner_poly_serializable.PullCornerPolySerializable data = 962;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable >*
      mutable_data();
  private:
  const ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable& _internal_data(int index) const;
  ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable* _internal_add_data();
  public:
  const ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable& data(int index) const;
  ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable > data_;
  friend struct ::TableStruct_si_2fpull_5fcorner_5fpoly_5fserializable_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PullCornerPolySerializable

// optional uint32 actualSize = 3009;
inline bool PullCornerPolySerializable::_internal_has_actualsize() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool PullCornerPolySerializable::has_actualsize() const {
  return _internal_has_actualsize();
}
inline void PullCornerPolySerializable::clear_actualsize() {
  actualsize_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PullCornerPolySerializable::_internal_actualsize() const {
  return actualsize_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PullCornerPolySerializable::actualsize() const {
  // @@protoc_insertion_point(field_get:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable.actualSize)
  return _internal_actualsize();
}
inline void PullCornerPolySerializable::_internal_set_actualsize(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  actualsize_ = value;
}
inline void PullCornerPolySerializable::set_actualsize(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_actualsize(value);
  // @@protoc_insertion_point(field_set:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable.actualSize)
}

// repeated .pb.si.pull_corner_poly.PullCornerPoly array = 417;
inline int PullCornerPolySerializable::_internal_array_size() const {
  return array_.size();
}
inline int PullCornerPolySerializable::array_size() const {
  return _internal_array_size();
}
inline ::pb::si::pull_corner_poly::PullCornerPoly* PullCornerPolySerializable::mutable_array(int index) {
  // @@protoc_insertion_point(field_mutable:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable.array)
  return array_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::pull_corner_poly::PullCornerPoly >*
PullCornerPolySerializable::mutable_array() {
  // @@protoc_insertion_point(field_mutable_list:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable.array)
  return &array_;
}
inline const ::pb::si::pull_corner_poly::PullCornerPoly& PullCornerPolySerializable::_internal_array(int index) const {
  return array_.Get(index);
}
inline const ::pb::si::pull_corner_poly::PullCornerPoly& PullCornerPolySerializable::array(int index) const {
  // @@protoc_insertion_point(field_get:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable.array)
  return _internal_array(index);
}
inline ::pb::si::pull_corner_poly::PullCornerPoly* PullCornerPolySerializable::_internal_add_array() {
  return array_.Add();
}
inline ::pb::si::pull_corner_poly::PullCornerPoly* PullCornerPolySerializable::add_array() {
  // @@protoc_insertion_point(field_add:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable.array)
  return _internal_add_array();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::pull_corner_poly::PullCornerPoly >&
PullCornerPolySerializable::array() const {
  // @@protoc_insertion_point(field_list:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable.array)
  return array_;
}

// -------------------------------------------------------------------

// PullCornerPolySerializable_array_port

// repeated .pb.si.pull_corner_poly_serializable.PullCornerPolySerializable data = 962;
inline int PullCornerPolySerializable_array_port::_internal_data_size() const {
  return data_.size();
}
inline int PullCornerPolySerializable_array_port::data_size() const {
  return _internal_data_size();
}
inline void PullCornerPolySerializable_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable* PullCornerPolySerializable_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable >*
PullCornerPolySerializable_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable_array_port.data)
  return &data_;
}
inline const ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable& PullCornerPolySerializable_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable& PullCornerPolySerializable_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable_array_port.data)
  return _internal_data(index);
}
inline ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable* PullCornerPolySerializable_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable* PullCornerPolySerializable_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::pull_corner_poly_serializable::PullCornerPolySerializable >&
PullCornerPolySerializable_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.si.pull_corner_poly_serializable.PullCornerPolySerializable_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace pull_corner_poly_serializable
}  // namespace si
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_si_2fpull_5fcorner_5fpoly_5fserializable_2eproto
