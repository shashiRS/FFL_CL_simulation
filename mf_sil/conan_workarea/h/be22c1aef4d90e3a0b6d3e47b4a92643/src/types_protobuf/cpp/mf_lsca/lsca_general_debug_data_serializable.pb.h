// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_lsca/lsca_general_debug_data_serializable.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5flsca_2flsca_5fgeneral_5fdebug_5fdata_5fserializable_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5flsca_2flsca_5fgeneral_5fdebug_5fdata_5fserializable_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_mf_5flsca_2flsca_5fgeneral_5fdebug_5fdata_5fserializable_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5flsca_2flsca_5fgeneral_5fdebug_5fdata_5fserializable_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5flsca_2flsca_5fgeneral_5fdebug_5fdata_5fserializable_2eproto;
namespace pb {
namespace mf_lsca {
namespace lsca_general_debug_data_serializable {
class LscaGeneralDebugDataSerializable;
class LscaGeneralDebugDataSerializableDefaultTypeInternal;
extern LscaGeneralDebugDataSerializableDefaultTypeInternal _LscaGeneralDebugDataSerializable_default_instance_;
class LscaGeneralDebugDataSerializable_array_port;
class LscaGeneralDebugDataSerializable_array_portDefaultTypeInternal;
extern LscaGeneralDebugDataSerializable_array_portDefaultTypeInternal _LscaGeneralDebugDataSerializable_array_port_default_instance_;
}  // namespace lsca_general_debug_data_serializable
}  // namespace mf_lsca
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable* Arena::CreateMaybeMessage<::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable>(Arena*);
template<> ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable_array_port* Arena::CreateMaybeMessage<::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_lsca {
namespace lsca_general_debug_data_serializable {

// ===================================================================

class LscaGeneralDebugDataSerializable :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable) */ {
 public:
  LscaGeneralDebugDataSerializable();
  virtual ~LscaGeneralDebugDataSerializable();

  LscaGeneralDebugDataSerializable(const LscaGeneralDebugDataSerializable& from);
  LscaGeneralDebugDataSerializable(LscaGeneralDebugDataSerializable&& from) noexcept
    : LscaGeneralDebugDataSerializable() {
    *this = ::std::move(from);
  }

  inline LscaGeneralDebugDataSerializable& operator=(const LscaGeneralDebugDataSerializable& from) {
    CopyFrom(from);
    return *this;
  }
  inline LscaGeneralDebugDataSerializable& operator=(LscaGeneralDebugDataSerializable&& from) noexcept {
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
  static const LscaGeneralDebugDataSerializable& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LscaGeneralDebugDataSerializable* internal_default_instance() {
    return reinterpret_cast<const LscaGeneralDebugDataSerializable*>(
               &_LscaGeneralDebugDataSerializable_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LscaGeneralDebugDataSerializable& a, LscaGeneralDebugDataSerializable& b) {
    a.Swap(&b);
  }
  inline void Swap(LscaGeneralDebugDataSerializable* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LscaGeneralDebugDataSerializable* New() const final {
    return CreateMaybeMessage<LscaGeneralDebugDataSerializable>(nullptr);
  }

  LscaGeneralDebugDataSerializable* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LscaGeneralDebugDataSerializable>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LscaGeneralDebugDataSerializable& from);
  void MergeFrom(const LscaGeneralDebugDataSerializable& from);
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
  void InternalSwap(LscaGeneralDebugDataSerializable* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5flsca_2flsca_5fgeneral_5fdebug_5fdata_5fserializable_2eproto);
    return ::descriptor_table_mf_5flsca_2flsca_5fgeneral_5fdebug_5fdata_5fserializable_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kGasPedalOverrideFieldNumber = 2512,
  };
  // optional bool gasPedalOverride = 2512;
  bool has_gaspedaloverride() const;
  private:
  bool _internal_has_gaspedaloverride() const;
  public:
  void clear_gaspedaloverride();
  bool gaspedaloverride() const;
  void set_gaspedaloverride(bool value);
  private:
  bool _internal_gaspedaloverride() const;
  void _internal_set_gaspedaloverride(bool value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  bool gaspedaloverride_;
  friend struct ::TableStruct_mf_5flsca_2flsca_5fgeneral_5fdebug_5fdata_5fserializable_2eproto;
};
// -------------------------------------------------------------------

class LscaGeneralDebugDataSerializable_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable_array_port) */ {
 public:
  LscaGeneralDebugDataSerializable_array_port();
  virtual ~LscaGeneralDebugDataSerializable_array_port();

  LscaGeneralDebugDataSerializable_array_port(const LscaGeneralDebugDataSerializable_array_port& from);
  LscaGeneralDebugDataSerializable_array_port(LscaGeneralDebugDataSerializable_array_port&& from) noexcept
    : LscaGeneralDebugDataSerializable_array_port() {
    *this = ::std::move(from);
  }

  inline LscaGeneralDebugDataSerializable_array_port& operator=(const LscaGeneralDebugDataSerializable_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline LscaGeneralDebugDataSerializable_array_port& operator=(LscaGeneralDebugDataSerializable_array_port&& from) noexcept {
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
  static const LscaGeneralDebugDataSerializable_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LscaGeneralDebugDataSerializable_array_port* internal_default_instance() {
    return reinterpret_cast<const LscaGeneralDebugDataSerializable_array_port*>(
               &_LscaGeneralDebugDataSerializable_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(LscaGeneralDebugDataSerializable_array_port& a, LscaGeneralDebugDataSerializable_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(LscaGeneralDebugDataSerializable_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LscaGeneralDebugDataSerializable_array_port* New() const final {
    return CreateMaybeMessage<LscaGeneralDebugDataSerializable_array_port>(nullptr);
  }

  LscaGeneralDebugDataSerializable_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LscaGeneralDebugDataSerializable_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LscaGeneralDebugDataSerializable_array_port& from);
  void MergeFrom(const LscaGeneralDebugDataSerializable_array_port& from);
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
  void InternalSwap(LscaGeneralDebugDataSerializable_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5flsca_2flsca_5fgeneral_5fdebug_5fdata_5fserializable_2eproto);
    return ::descriptor_table_mf_5flsca_2flsca_5fgeneral_5fdebug_5fdata_5fserializable_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1567,
  };
  // repeated .pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable data = 1567;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable >*
      mutable_data();
  private:
  const ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable& _internal_data(int index) const;
  ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable* _internal_add_data();
  public:
  const ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable& data(int index) const;
  ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable > data_;
  friend struct ::TableStruct_mf_5flsca_2flsca_5fgeneral_5fdebug_5fdata_5fserializable_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LscaGeneralDebugDataSerializable

// optional bool gasPedalOverride = 2512;
inline bool LscaGeneralDebugDataSerializable::_internal_has_gaspedaloverride() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool LscaGeneralDebugDataSerializable::has_gaspedaloverride() const {
  return _internal_has_gaspedaloverride();
}
inline void LscaGeneralDebugDataSerializable::clear_gaspedaloverride() {
  gaspedaloverride_ = false;
  _has_bits_[0] &= ~0x00000001u;
}
inline bool LscaGeneralDebugDataSerializable::_internal_gaspedaloverride() const {
  return gaspedaloverride_;
}
inline bool LscaGeneralDebugDataSerializable::gaspedaloverride() const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable.gasPedalOverride)
  return _internal_gaspedaloverride();
}
inline void LscaGeneralDebugDataSerializable::_internal_set_gaspedaloverride(bool value) {
  _has_bits_[0] |= 0x00000001u;
  gaspedaloverride_ = value;
}
inline void LscaGeneralDebugDataSerializable::set_gaspedaloverride(bool value) {
  _internal_set_gaspedaloverride(value);
  // @@protoc_insertion_point(field_set:pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable.gasPedalOverride)
}

// -------------------------------------------------------------------

// LscaGeneralDebugDataSerializable_array_port

// repeated .pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable data = 1567;
inline int LscaGeneralDebugDataSerializable_array_port::_internal_data_size() const {
  return data_.size();
}
inline int LscaGeneralDebugDataSerializable_array_port::data_size() const {
  return _internal_data_size();
}
inline void LscaGeneralDebugDataSerializable_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable* LscaGeneralDebugDataSerializable_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable >*
LscaGeneralDebugDataSerializable_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable_array_port.data)
  return &data_;
}
inline const ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable& LscaGeneralDebugDataSerializable_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable& LscaGeneralDebugDataSerializable_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable* LscaGeneralDebugDataSerializable_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable* LscaGeneralDebugDataSerializable_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_lsca::lsca_general_debug_data_serializable::LscaGeneralDebugDataSerializable >&
LscaGeneralDebugDataSerializable_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace lsca_general_debug_data_serializable
}  // namespace mf_lsca
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5flsca_2flsca_5fgeneral_5fdebug_5fdata_5fserializable_2eproto
