// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_lsca/lsca_ego_simple_body_shape_serializable.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5flsca_2flsca_5fego_5fsimple_5fbody_5fshape_5fserializable_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5flsca_2flsca_5fego_5fsimple_5fbody_5fshape_5fserializable_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_mf_5flsca_2flsca_5fego_5fsimple_5fbody_5fshape_5fserializable_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5flsca_2flsca_5fego_5fsimple_5fbody_5fshape_5fserializable_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5flsca_2flsca_5fego_5fsimple_5fbody_5fshape_5fserializable_2eproto;
namespace pb {
namespace mf_lsca {
namespace lsca_ego_simple_body_shape_serializable {
class LscaEgoSimpleBodyShapeSerializable;
class LscaEgoSimpleBodyShapeSerializableDefaultTypeInternal;
extern LscaEgoSimpleBodyShapeSerializableDefaultTypeInternal _LscaEgoSimpleBodyShapeSerializable_default_instance_;
class LscaEgoSimpleBodyShapeSerializable_array_port;
class LscaEgoSimpleBodyShapeSerializable_array_portDefaultTypeInternal;
extern LscaEgoSimpleBodyShapeSerializable_array_portDefaultTypeInternal _LscaEgoSimpleBodyShapeSerializable_array_port_default_instance_;
}  // namespace lsca_ego_simple_body_shape_serializable
}  // namespace mf_lsca
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable* Arena::CreateMaybeMessage<::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable>(Arena*);
template<> ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable_array_port* Arena::CreateMaybeMessage<::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_lsca {
namespace lsca_ego_simple_body_shape_serializable {

// ===================================================================

class LscaEgoSimpleBodyShapeSerializable :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable) */ {
 public:
  LscaEgoSimpleBodyShapeSerializable();
  virtual ~LscaEgoSimpleBodyShapeSerializable();

  LscaEgoSimpleBodyShapeSerializable(const LscaEgoSimpleBodyShapeSerializable& from);
  LscaEgoSimpleBodyShapeSerializable(LscaEgoSimpleBodyShapeSerializable&& from) noexcept
    : LscaEgoSimpleBodyShapeSerializable() {
    *this = ::std::move(from);
  }

  inline LscaEgoSimpleBodyShapeSerializable& operator=(const LscaEgoSimpleBodyShapeSerializable& from) {
    CopyFrom(from);
    return *this;
  }
  inline LscaEgoSimpleBodyShapeSerializable& operator=(LscaEgoSimpleBodyShapeSerializable&& from) noexcept {
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
  static const LscaEgoSimpleBodyShapeSerializable& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LscaEgoSimpleBodyShapeSerializable* internal_default_instance() {
    return reinterpret_cast<const LscaEgoSimpleBodyShapeSerializable*>(
               &_LscaEgoSimpleBodyShapeSerializable_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LscaEgoSimpleBodyShapeSerializable& a, LscaEgoSimpleBodyShapeSerializable& b) {
    a.Swap(&b);
  }
  inline void Swap(LscaEgoSimpleBodyShapeSerializable* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LscaEgoSimpleBodyShapeSerializable* New() const final {
    return CreateMaybeMessage<LscaEgoSimpleBodyShapeSerializable>(nullptr);
  }

  LscaEgoSimpleBodyShapeSerializable* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LscaEgoSimpleBodyShapeSerializable>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LscaEgoSimpleBodyShapeSerializable& from);
  void MergeFrom(const LscaEgoSimpleBodyShapeSerializable& from);
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
  void InternalSwap(LscaEgoSimpleBodyShapeSerializable* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5flsca_2flsca_5fego_5fsimple_5fbody_5fshape_5fserializable_2eproto);
    return ::descriptor_table_mf_5flsca_2flsca_5fego_5fsimple_5fbody_5fshape_5fserializable_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kPointsFieldNumber = 3855,
    kActualSizeFieldNumber = 3009,
  };
  // repeated .pb.cml.vec2_df_pod.Vec2Df_POD points = 3855;
  int points_size() const;
  private:
  int _internal_points_size() const;
  public:
  void clear_points();
  ::pb::cml::vec2_df_pod::Vec2Df_POD* mutable_points(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD >*
      mutable_points();
  private:
  const ::pb::cml::vec2_df_pod::Vec2Df_POD& _internal_points(int index) const;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* _internal_add_points();
  public:
  const ::pb::cml::vec2_df_pod::Vec2Df_POD& points(int index) const;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* add_points();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD >&
      points() const;

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

  // @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD > points_;
  ::PROTOBUF_NAMESPACE_ID::uint32 actualsize_;
  friend struct ::TableStruct_mf_5flsca_2flsca_5fego_5fsimple_5fbody_5fshape_5fserializable_2eproto;
};
// -------------------------------------------------------------------

class LscaEgoSimpleBodyShapeSerializable_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable_array_port) */ {
 public:
  LscaEgoSimpleBodyShapeSerializable_array_port();
  virtual ~LscaEgoSimpleBodyShapeSerializable_array_port();

  LscaEgoSimpleBodyShapeSerializable_array_port(const LscaEgoSimpleBodyShapeSerializable_array_port& from);
  LscaEgoSimpleBodyShapeSerializable_array_port(LscaEgoSimpleBodyShapeSerializable_array_port&& from) noexcept
    : LscaEgoSimpleBodyShapeSerializable_array_port() {
    *this = ::std::move(from);
  }

  inline LscaEgoSimpleBodyShapeSerializable_array_port& operator=(const LscaEgoSimpleBodyShapeSerializable_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline LscaEgoSimpleBodyShapeSerializable_array_port& operator=(LscaEgoSimpleBodyShapeSerializable_array_port&& from) noexcept {
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
  static const LscaEgoSimpleBodyShapeSerializable_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LscaEgoSimpleBodyShapeSerializable_array_port* internal_default_instance() {
    return reinterpret_cast<const LscaEgoSimpleBodyShapeSerializable_array_port*>(
               &_LscaEgoSimpleBodyShapeSerializable_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(LscaEgoSimpleBodyShapeSerializable_array_port& a, LscaEgoSimpleBodyShapeSerializable_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(LscaEgoSimpleBodyShapeSerializable_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LscaEgoSimpleBodyShapeSerializable_array_port* New() const final {
    return CreateMaybeMessage<LscaEgoSimpleBodyShapeSerializable_array_port>(nullptr);
  }

  LscaEgoSimpleBodyShapeSerializable_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LscaEgoSimpleBodyShapeSerializable_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LscaEgoSimpleBodyShapeSerializable_array_port& from);
  void MergeFrom(const LscaEgoSimpleBodyShapeSerializable_array_port& from);
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
  void InternalSwap(LscaEgoSimpleBodyShapeSerializable_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5flsca_2flsca_5fego_5fsimple_5fbody_5fshape_5fserializable_2eproto);
    return ::descriptor_table_mf_5flsca_2flsca_5fego_5fsimple_5fbody_5fshape_5fserializable_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1836,
  };
  // repeated .pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable data = 1836;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable >*
      mutable_data();
  private:
  const ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable& _internal_data(int index) const;
  ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable* _internal_add_data();
  public:
  const ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable& data(int index) const;
  ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable > data_;
  friend struct ::TableStruct_mf_5flsca_2flsca_5fego_5fsimple_5fbody_5fshape_5fserializable_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LscaEgoSimpleBodyShapeSerializable

// optional uint32 actualSize = 3009;
inline bool LscaEgoSimpleBodyShapeSerializable::_internal_has_actualsize() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool LscaEgoSimpleBodyShapeSerializable::has_actualsize() const {
  return _internal_has_actualsize();
}
inline void LscaEgoSimpleBodyShapeSerializable::clear_actualsize() {
  actualsize_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 LscaEgoSimpleBodyShapeSerializable::_internal_actualsize() const {
  return actualsize_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 LscaEgoSimpleBodyShapeSerializable::actualsize() const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable.actualSize)
  return _internal_actualsize();
}
inline void LscaEgoSimpleBodyShapeSerializable::_internal_set_actualsize(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  actualsize_ = value;
}
inline void LscaEgoSimpleBodyShapeSerializable::set_actualsize(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_actualsize(value);
  // @@protoc_insertion_point(field_set:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable.actualSize)
}

// repeated .pb.cml.vec2_df_pod.Vec2Df_POD points = 3855;
inline int LscaEgoSimpleBodyShapeSerializable::_internal_points_size() const {
  return points_.size();
}
inline int LscaEgoSimpleBodyShapeSerializable::points_size() const {
  return _internal_points_size();
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* LscaEgoSimpleBodyShapeSerializable::mutable_points(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable.points)
  return points_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD >*
LscaEgoSimpleBodyShapeSerializable::mutable_points() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable.points)
  return &points_;
}
inline const ::pb::cml::vec2_df_pod::Vec2Df_POD& LscaEgoSimpleBodyShapeSerializable::_internal_points(int index) const {
  return points_.Get(index);
}
inline const ::pb::cml::vec2_df_pod::Vec2Df_POD& LscaEgoSimpleBodyShapeSerializable::points(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable.points)
  return _internal_points(index);
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* LscaEgoSimpleBodyShapeSerializable::_internal_add_points() {
  return points_.Add();
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* LscaEgoSimpleBodyShapeSerializable::add_points() {
  // @@protoc_insertion_point(field_add:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable.points)
  return _internal_add_points();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD >&
LscaEgoSimpleBodyShapeSerializable::points() const {
  // @@protoc_insertion_point(field_list:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable.points)
  return points_;
}

// -------------------------------------------------------------------

// LscaEgoSimpleBodyShapeSerializable_array_port

// repeated .pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable data = 1836;
inline int LscaEgoSimpleBodyShapeSerializable_array_port::_internal_data_size() const {
  return data_.size();
}
inline int LscaEgoSimpleBodyShapeSerializable_array_port::data_size() const {
  return _internal_data_size();
}
inline void LscaEgoSimpleBodyShapeSerializable_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable* LscaEgoSimpleBodyShapeSerializable_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable >*
LscaEgoSimpleBodyShapeSerializable_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable_array_port.data)
  return &data_;
}
inline const ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable& LscaEgoSimpleBodyShapeSerializable_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable& LscaEgoSimpleBodyShapeSerializable_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable* LscaEgoSimpleBodyShapeSerializable_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable* LscaEgoSimpleBodyShapeSerializable_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_ego_simple_body_shape_serializable::LscaEgoSimpleBodyShapeSerializable >&
LscaEgoSimpleBodyShapeSerializable_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_lsca.lsca_ego_simple_body_shape_serializable.LscaEgoSimpleBodyShapeSerializable_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace lsca_ego_simple_body_shape_serializable
}  // namespace mf_lsca
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5flsca_2flsca_5fego_5fsimple_5fbody_5fshape_5fserializable_2eproto
