// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cml/vec2_df_pod.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_cml_2fvec2_5fdf_5fpod_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_cml_2fvec2_5fdf_5fpod_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_cml_2fvec2_5fdf_5fpod_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_cml_2fvec2_5fdf_5fpod_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cml_2fvec2_5fdf_5fpod_2eproto;
namespace pb {
namespace cml {
namespace vec2_df_pod {
class Vec2Df_POD;
class Vec2Df_PODDefaultTypeInternal;
extern Vec2Df_PODDefaultTypeInternal _Vec2Df_POD_default_instance_;
class Vec2Df_POD_array_port;
class Vec2Df_POD_array_portDefaultTypeInternal;
extern Vec2Df_POD_array_portDefaultTypeInternal _Vec2Df_POD_array_port_default_instance_;
}  // namespace vec2_df_pod
}  // namespace cml
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::cml::vec2_df_pod::Vec2Df_POD* Arena::CreateMaybeMessage<::pb::cml::vec2_df_pod::Vec2Df_POD>(Arena*);
template<> ::pb::cml::vec2_df_pod::Vec2Df_POD_array_port* Arena::CreateMaybeMessage<::pb::cml::vec2_df_pod::Vec2Df_POD_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace cml {
namespace vec2_df_pod {

// ===================================================================

class Vec2Df_POD :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.cml.vec2_df_pod.Vec2Df_POD) */ {
 public:
  Vec2Df_POD();
  virtual ~Vec2Df_POD();

  Vec2Df_POD(const Vec2Df_POD& from);
  Vec2Df_POD(Vec2Df_POD&& from) noexcept
    : Vec2Df_POD() {
    *this = ::std::move(from);
  }

  inline Vec2Df_POD& operator=(const Vec2Df_POD& from) {
    CopyFrom(from);
    return *this;
  }
  inline Vec2Df_POD& operator=(Vec2Df_POD&& from) noexcept {
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
  static const Vec2Df_POD& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Vec2Df_POD* internal_default_instance() {
    return reinterpret_cast<const Vec2Df_POD*>(
               &_Vec2Df_POD_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Vec2Df_POD& a, Vec2Df_POD& b) {
    a.Swap(&b);
  }
  inline void Swap(Vec2Df_POD* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Vec2Df_POD* New() const final {
    return CreateMaybeMessage<Vec2Df_POD>(nullptr);
  }

  Vec2Df_POD* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Vec2Df_POD>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Vec2Df_POD& from);
  void MergeFrom(const Vec2Df_POD& from);
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
  void InternalSwap(Vec2Df_POD* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.cml.vec2_df_pod.Vec2Df_POD";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_cml_2fvec2_5fdf_5fpod_2eproto);
    return ::descriptor_table_cml_2fvec2_5fdf_5fpod_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kXDirFieldNumber = 1650,
    kYDirFieldNumber = 3107,
  };
  // optional float x_dir = 1650;
  bool has_x_dir() const;
  private:
  bool _internal_has_x_dir() const;
  public:
  void clear_x_dir();
  float x_dir() const;
  void set_x_dir(float value);
  private:
  float _internal_x_dir() const;
  void _internal_set_x_dir(float value);
  public:

  // optional float y_dir = 3107;
  bool has_y_dir() const;
  private:
  bool _internal_has_y_dir() const;
  public:
  void clear_y_dir();
  float y_dir() const;
  void set_y_dir(float value);
  private:
  float _internal_y_dir() const;
  void _internal_set_y_dir(float value);
  public:

  // @@protoc_insertion_point(class_scope:pb.cml.vec2_df_pod.Vec2Df_POD)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  float x_dir_;
  float y_dir_;
  friend struct ::TableStruct_cml_2fvec2_5fdf_5fpod_2eproto;
};
// -------------------------------------------------------------------

class Vec2Df_POD_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.cml.vec2_df_pod.Vec2Df_POD_array_port) */ {
 public:
  Vec2Df_POD_array_port();
  virtual ~Vec2Df_POD_array_port();

  Vec2Df_POD_array_port(const Vec2Df_POD_array_port& from);
  Vec2Df_POD_array_port(Vec2Df_POD_array_port&& from) noexcept
    : Vec2Df_POD_array_port() {
    *this = ::std::move(from);
  }

  inline Vec2Df_POD_array_port& operator=(const Vec2Df_POD_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline Vec2Df_POD_array_port& operator=(Vec2Df_POD_array_port&& from) noexcept {
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
  static const Vec2Df_POD_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Vec2Df_POD_array_port* internal_default_instance() {
    return reinterpret_cast<const Vec2Df_POD_array_port*>(
               &_Vec2Df_POD_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(Vec2Df_POD_array_port& a, Vec2Df_POD_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(Vec2Df_POD_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Vec2Df_POD_array_port* New() const final {
    return CreateMaybeMessage<Vec2Df_POD_array_port>(nullptr);
  }

  Vec2Df_POD_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Vec2Df_POD_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Vec2Df_POD_array_port& from);
  void MergeFrom(const Vec2Df_POD_array_port& from);
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
  void InternalSwap(Vec2Df_POD_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.cml.vec2_df_pod.Vec2Df_POD_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_cml_2fvec2_5fdf_5fpod_2eproto);
    return ::descriptor_table_cml_2fvec2_5fdf_5fpod_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3375,
  };
  // repeated .pb.cml.vec2_df_pod.Vec2Df_POD data = 3375;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::cml::vec2_df_pod::Vec2Df_POD* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD >*
      mutable_data();
  private:
  const ::pb::cml::vec2_df_pod::Vec2Df_POD& _internal_data(int index) const;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* _internal_add_data();
  public:
  const ::pb::cml::vec2_df_pod::Vec2Df_POD& data(int index) const;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.cml.vec2_df_pod.Vec2Df_POD_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD > data_;
  friend struct ::TableStruct_cml_2fvec2_5fdf_5fpod_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Vec2Df_POD

// optional float x_dir = 1650;
inline bool Vec2Df_POD::_internal_has_x_dir() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool Vec2Df_POD::has_x_dir() const {
  return _internal_has_x_dir();
}
inline void Vec2Df_POD::clear_x_dir() {
  x_dir_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline float Vec2Df_POD::_internal_x_dir() const {
  return x_dir_;
}
inline float Vec2Df_POD::x_dir() const {
  // @@protoc_insertion_point(field_get:pb.cml.vec2_df_pod.Vec2Df_POD.x_dir)
  return _internal_x_dir();
}
inline void Vec2Df_POD::_internal_set_x_dir(float value) {
  _has_bits_[0] |= 0x00000001u;
  x_dir_ = value;
}
inline void Vec2Df_POD::set_x_dir(float value) {
  _internal_set_x_dir(value);
  // @@protoc_insertion_point(field_set:pb.cml.vec2_df_pod.Vec2Df_POD.x_dir)
}

// optional float y_dir = 3107;
inline bool Vec2Df_POD::_internal_has_y_dir() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool Vec2Df_POD::has_y_dir() const {
  return _internal_has_y_dir();
}
inline void Vec2Df_POD::clear_y_dir() {
  y_dir_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline float Vec2Df_POD::_internal_y_dir() const {
  return y_dir_;
}
inline float Vec2Df_POD::y_dir() const {
  // @@protoc_insertion_point(field_get:pb.cml.vec2_df_pod.Vec2Df_POD.y_dir)
  return _internal_y_dir();
}
inline void Vec2Df_POD::_internal_set_y_dir(float value) {
  _has_bits_[0] |= 0x00000002u;
  y_dir_ = value;
}
inline void Vec2Df_POD::set_y_dir(float value) {
  _internal_set_y_dir(value);
  // @@protoc_insertion_point(field_set:pb.cml.vec2_df_pod.Vec2Df_POD.y_dir)
}

// -------------------------------------------------------------------

// Vec2Df_POD_array_port

// repeated .pb.cml.vec2_df_pod.Vec2Df_POD data = 3375;
inline int Vec2Df_POD_array_port::_internal_data_size() const {
  return data_.size();
}
inline int Vec2Df_POD_array_port::data_size() const {
  return _internal_data_size();
}
inline void Vec2Df_POD_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* Vec2Df_POD_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.cml.vec2_df_pod.Vec2Df_POD_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD >*
Vec2Df_POD_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.cml.vec2_df_pod.Vec2Df_POD_array_port.data)
  return &data_;
}
inline const ::pb::cml::vec2_df_pod::Vec2Df_POD& Vec2Df_POD_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::cml::vec2_df_pod::Vec2Df_POD& Vec2Df_POD_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.cml.vec2_df_pod.Vec2Df_POD_array_port.data)
  return _internal_data(index);
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* Vec2Df_POD_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* Vec2Df_POD_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.cml.vec2_df_pod.Vec2Df_POD_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::cml::vec2_df_pod::Vec2Df_POD >&
Vec2Df_POD_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.cml.vec2_df_pod.Vec2Df_POD_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace vec2_df_pod
}  // namespace cml
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_cml_2fvec2_5fdf_5fpod_2eproto
