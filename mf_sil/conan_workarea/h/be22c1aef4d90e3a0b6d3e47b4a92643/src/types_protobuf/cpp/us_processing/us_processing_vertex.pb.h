// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_processing/us_processing_vertex.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fprocessing_2fus_5fprocessing_5fvertex_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fprocessing_2fus_5fprocessing_5fvertex_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_us_5fprocessing_2fus_5fprocessing_5fvertex_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fprocessing_2fus_5fprocessing_5fvertex_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fprocessing_2fus_5fprocessing_5fvertex_2eproto;
namespace pb {
namespace us_processing {
namespace us_processing_vertex {
class UsProcessingVertex;
class UsProcessingVertexDefaultTypeInternal;
extern UsProcessingVertexDefaultTypeInternal _UsProcessingVertex_default_instance_;
class UsProcessingVertex_array_port;
class UsProcessingVertex_array_portDefaultTypeInternal;
extern UsProcessingVertex_array_portDefaultTypeInternal _UsProcessingVertex_array_port_default_instance_;
}  // namespace us_processing_vertex
}  // namespace us_processing
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_processing::us_processing_vertex::UsProcessingVertex* Arena::CreateMaybeMessage<::pb::us_processing::us_processing_vertex::UsProcessingVertex>(Arena*);
template<> ::pb::us_processing::us_processing_vertex::UsProcessingVertex_array_port* Arena::CreateMaybeMessage<::pb::us_processing::us_processing_vertex::UsProcessingVertex_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_processing {
namespace us_processing_vertex {

// ===================================================================

class UsProcessingVertex :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_processing.us_processing_vertex.UsProcessingVertex) */ {
 public:
  UsProcessingVertex();
  virtual ~UsProcessingVertex();

  UsProcessingVertex(const UsProcessingVertex& from);
  UsProcessingVertex(UsProcessingVertex&& from) noexcept
    : UsProcessingVertex() {
    *this = ::std::move(from);
  }

  inline UsProcessingVertex& operator=(const UsProcessingVertex& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsProcessingVertex& operator=(UsProcessingVertex&& from) noexcept {
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
  static const UsProcessingVertex& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsProcessingVertex* internal_default_instance() {
    return reinterpret_cast<const UsProcessingVertex*>(
               &_UsProcessingVertex_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsProcessingVertex& a, UsProcessingVertex& b) {
    a.Swap(&b);
  }
  inline void Swap(UsProcessingVertex* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsProcessingVertex* New() const final {
    return CreateMaybeMessage<UsProcessingVertex>(nullptr);
  }

  UsProcessingVertex* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsProcessingVertex>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsProcessingVertex& from);
  void MergeFrom(const UsProcessingVertex& from);
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
  void InternalSwap(UsProcessingVertex* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_processing.us_processing_vertex.UsProcessingVertex";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fvertex_2eproto);
    return ::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fvertex_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kPosXVarMFieldNumber = 38,
    kPosXMFieldNumber = 287,
    kPosYVarMFieldNumber = 1414,
    kPosYMFieldNumber = 1583,
  };
  // optional float posXVar_m = 38;
  bool has_posxvar_m() const;
  private:
  bool _internal_has_posxvar_m() const;
  public:
  void clear_posxvar_m();
  float posxvar_m() const;
  void set_posxvar_m(float value);
  private:
  float _internal_posxvar_m() const;
  void _internal_set_posxvar_m(float value);
  public:

  // optional float posX_m = 287;
  bool has_posx_m() const;
  private:
  bool _internal_has_posx_m() const;
  public:
  void clear_posx_m();
  float posx_m() const;
  void set_posx_m(float value);
  private:
  float _internal_posx_m() const;
  void _internal_set_posx_m(float value);
  public:

  // optional float posYVar_m = 1414;
  bool has_posyvar_m() const;
  private:
  bool _internal_has_posyvar_m() const;
  public:
  void clear_posyvar_m();
  float posyvar_m() const;
  void set_posyvar_m(float value);
  private:
  float _internal_posyvar_m() const;
  void _internal_set_posyvar_m(float value);
  public:

  // optional float posY_m = 1583;
  bool has_posy_m() const;
  private:
  bool _internal_has_posy_m() const;
  public:
  void clear_posy_m();
  float posy_m() const;
  void set_posy_m(float value);
  private:
  float _internal_posy_m() const;
  void _internal_set_posy_m(float value);
  public:

  // @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_vertex.UsProcessingVertex)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  float posxvar_m_;
  float posx_m_;
  float posyvar_m_;
  float posy_m_;
  friend struct ::TableStruct_us_5fprocessing_2fus_5fprocessing_5fvertex_2eproto;
};
// -------------------------------------------------------------------

class UsProcessingVertex_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_processing.us_processing_vertex.UsProcessingVertex_array_port) */ {
 public:
  UsProcessingVertex_array_port();
  virtual ~UsProcessingVertex_array_port();

  UsProcessingVertex_array_port(const UsProcessingVertex_array_port& from);
  UsProcessingVertex_array_port(UsProcessingVertex_array_port&& from) noexcept
    : UsProcessingVertex_array_port() {
    *this = ::std::move(from);
  }

  inline UsProcessingVertex_array_port& operator=(const UsProcessingVertex_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsProcessingVertex_array_port& operator=(UsProcessingVertex_array_port&& from) noexcept {
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
  static const UsProcessingVertex_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsProcessingVertex_array_port* internal_default_instance() {
    return reinterpret_cast<const UsProcessingVertex_array_port*>(
               &_UsProcessingVertex_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsProcessingVertex_array_port& a, UsProcessingVertex_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsProcessingVertex_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsProcessingVertex_array_port* New() const final {
    return CreateMaybeMessage<UsProcessingVertex_array_port>(nullptr);
  }

  UsProcessingVertex_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsProcessingVertex_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsProcessingVertex_array_port& from);
  void MergeFrom(const UsProcessingVertex_array_port& from);
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
  void InternalSwap(UsProcessingVertex_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_processing.us_processing_vertex.UsProcessingVertex_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fvertex_2eproto);
    return ::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fvertex_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1893,
  };
  // repeated .pb.us_processing.us_processing_vertex.UsProcessingVertex data = 1893;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_processing::us_processing_vertex::UsProcessingVertex* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_vertex::UsProcessingVertex >*
      mutable_data();
  private:
  const ::pb::us_processing::us_processing_vertex::UsProcessingVertex& _internal_data(int index) const;
  ::pb::us_processing::us_processing_vertex::UsProcessingVertex* _internal_add_data();
  public:
  const ::pb::us_processing::us_processing_vertex::UsProcessingVertex& data(int index) const;
  ::pb::us_processing::us_processing_vertex::UsProcessingVertex* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_vertex::UsProcessingVertex >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_vertex.UsProcessingVertex_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_vertex::UsProcessingVertex > data_;
  friend struct ::TableStruct_us_5fprocessing_2fus_5fprocessing_5fvertex_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsProcessingVertex

// optional float posX_m = 287;
inline bool UsProcessingVertex::_internal_has_posx_m() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool UsProcessingVertex::has_posx_m() const {
  return _internal_has_posx_m();
}
inline void UsProcessingVertex::clear_posx_m() {
  posx_m_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline float UsProcessingVertex::_internal_posx_m() const {
  return posx_m_;
}
inline float UsProcessingVertex::posx_m() const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_vertex.UsProcessingVertex.posX_m)
  return _internal_posx_m();
}
inline void UsProcessingVertex::_internal_set_posx_m(float value) {
  _has_bits_[0] |= 0x00000002u;
  posx_m_ = value;
}
inline void UsProcessingVertex::set_posx_m(float value) {
  _internal_set_posx_m(value);
  // @@protoc_insertion_point(field_set:pb.us_processing.us_processing_vertex.UsProcessingVertex.posX_m)
}

// optional float posY_m = 1583;
inline bool UsProcessingVertex::_internal_has_posy_m() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool UsProcessingVertex::has_posy_m() const {
  return _internal_has_posy_m();
}
inline void UsProcessingVertex::clear_posy_m() {
  posy_m_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float UsProcessingVertex::_internal_posy_m() const {
  return posy_m_;
}
inline float UsProcessingVertex::posy_m() const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_vertex.UsProcessingVertex.posY_m)
  return _internal_posy_m();
}
inline void UsProcessingVertex::_internal_set_posy_m(float value) {
  _has_bits_[0] |= 0x00000008u;
  posy_m_ = value;
}
inline void UsProcessingVertex::set_posy_m(float value) {
  _internal_set_posy_m(value);
  // @@protoc_insertion_point(field_set:pb.us_processing.us_processing_vertex.UsProcessingVertex.posY_m)
}

// optional float posXVar_m = 38;
inline bool UsProcessingVertex::_internal_has_posxvar_m() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool UsProcessingVertex::has_posxvar_m() const {
  return _internal_has_posxvar_m();
}
inline void UsProcessingVertex::clear_posxvar_m() {
  posxvar_m_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline float UsProcessingVertex::_internal_posxvar_m() const {
  return posxvar_m_;
}
inline float UsProcessingVertex::posxvar_m() const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_vertex.UsProcessingVertex.posXVar_m)
  return _internal_posxvar_m();
}
inline void UsProcessingVertex::_internal_set_posxvar_m(float value) {
  _has_bits_[0] |= 0x00000001u;
  posxvar_m_ = value;
}
inline void UsProcessingVertex::set_posxvar_m(float value) {
  _internal_set_posxvar_m(value);
  // @@protoc_insertion_point(field_set:pb.us_processing.us_processing_vertex.UsProcessingVertex.posXVar_m)
}

// optional float posYVar_m = 1414;
inline bool UsProcessingVertex::_internal_has_posyvar_m() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool UsProcessingVertex::has_posyvar_m() const {
  return _internal_has_posyvar_m();
}
inline void UsProcessingVertex::clear_posyvar_m() {
  posyvar_m_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float UsProcessingVertex::_internal_posyvar_m() const {
  return posyvar_m_;
}
inline float UsProcessingVertex::posyvar_m() const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_vertex.UsProcessingVertex.posYVar_m)
  return _internal_posyvar_m();
}
inline void UsProcessingVertex::_internal_set_posyvar_m(float value) {
  _has_bits_[0] |= 0x00000004u;
  posyvar_m_ = value;
}
inline void UsProcessingVertex::set_posyvar_m(float value) {
  _internal_set_posyvar_m(value);
  // @@protoc_insertion_point(field_set:pb.us_processing.us_processing_vertex.UsProcessingVertex.posYVar_m)
}

// -------------------------------------------------------------------

// UsProcessingVertex_array_port

// repeated .pb.us_processing.us_processing_vertex.UsProcessingVertex data = 1893;
inline int UsProcessingVertex_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsProcessingVertex_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsProcessingVertex_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_processing::us_processing_vertex::UsProcessingVertex* UsProcessingVertex_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_processing.us_processing_vertex.UsProcessingVertex_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_vertex::UsProcessingVertex >*
UsProcessingVertex_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_processing.us_processing_vertex.UsProcessingVertex_array_port.data)
  return &data_;
}
inline const ::pb::us_processing::us_processing_vertex::UsProcessingVertex& UsProcessingVertex_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_processing::us_processing_vertex::UsProcessingVertex& UsProcessingVertex_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_vertex.UsProcessingVertex_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_processing::us_processing_vertex::UsProcessingVertex* UsProcessingVertex_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_processing::us_processing_vertex::UsProcessingVertex* UsProcessingVertex_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_processing.us_processing_vertex.UsProcessingVertex_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_vertex::UsProcessingVertex >&
UsProcessingVertex_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_processing.us_processing_vertex.UsProcessingVertex_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_processing_vertex
}  // namespace us_processing
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fprocessing_2fus_5fprocessing_5fvertex_2eproto
