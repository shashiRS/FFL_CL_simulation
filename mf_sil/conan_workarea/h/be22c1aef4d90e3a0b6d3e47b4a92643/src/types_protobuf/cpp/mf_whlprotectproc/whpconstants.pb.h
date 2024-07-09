// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_whlprotectproc/whpconstants.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fwhlprotectproc_2fwhpconstants_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fwhlprotectproc_2fwhpconstants_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_mf_5fwhlprotectproc_2fwhpconstants_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fwhlprotectproc_2fwhpconstants_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fwhlprotectproc_2fwhpconstants_2eproto;
namespace pb {
namespace mf_whlprotectproc {
namespace whpconstants {
class WHPConstants;
class WHPConstantsDefaultTypeInternal;
extern WHPConstantsDefaultTypeInternal _WHPConstants_default_instance_;
class WHPConstants_array_port;
class WHPConstants_array_portDefaultTypeInternal;
extern WHPConstants_array_portDefaultTypeInternal _WHPConstants_array_port_default_instance_;
}  // namespace whpconstants
}  // namespace mf_whlprotectproc
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_whlprotectproc::whpconstants::WHPConstants* Arena::CreateMaybeMessage<::pb::mf_whlprotectproc::whpconstants::WHPConstants>(Arena*);
template<> ::pb::mf_whlprotectproc::whpconstants::WHPConstants_array_port* Arena::CreateMaybeMessage<::pb::mf_whlprotectproc::whpconstants::WHPConstants_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_whlprotectproc {
namespace whpconstants {

// ===================================================================

class WHPConstants :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_whlprotectproc.whpconstants.WHPConstants) */ {
 public:
  WHPConstants();
  virtual ~WHPConstants();

  WHPConstants(const WHPConstants& from);
  WHPConstants(WHPConstants&& from) noexcept
    : WHPConstants() {
    *this = ::std::move(from);
  }

  inline WHPConstants& operator=(const WHPConstants& from) {
    CopyFrom(from);
    return *this;
  }
  inline WHPConstants& operator=(WHPConstants&& from) noexcept {
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
  static const WHPConstants& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const WHPConstants* internal_default_instance() {
    return reinterpret_cast<const WHPConstants*>(
               &_WHPConstants_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(WHPConstants& a, WHPConstants& b) {
    a.Swap(&b);
  }
  inline void Swap(WHPConstants* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline WHPConstants* New() const final {
    return CreateMaybeMessage<WHPConstants>(nullptr);
  }

  WHPConstants* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<WHPConstants>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const WHPConstants& from);
  void MergeFrom(const WHPConstants& from);
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
  void InternalSwap(WHPConstants* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_whlprotectproc.whpconstants.WHPConstants";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fwhlprotectproc_2fwhpconstants_2eproto);
    return ::descriptor_table_mf_5fwhlprotectproc_2fwhpconstants_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kNUMMTSDEBUGFREESPACEWHPPFieldNumber = 245,
  };
  // optional uint32 NUM_MTS_DEBUG_FREESPACE_WHPP = 245;
  bool has_num_mts_debug_freespace_whpp() const;
  private:
  bool _internal_has_num_mts_debug_freespace_whpp() const;
  public:
  void clear_num_mts_debug_freespace_whpp();
  ::PROTOBUF_NAMESPACE_ID::uint32 num_mts_debug_freespace_whpp() const;
  void set_num_mts_debug_freespace_whpp(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_num_mts_debug_freespace_whpp() const;
  void _internal_set_num_mts_debug_freespace_whpp(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_whlprotectproc.whpconstants.WHPConstants)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 num_mts_debug_freespace_whpp_;
  friend struct ::TableStruct_mf_5fwhlprotectproc_2fwhpconstants_2eproto;
};
// -------------------------------------------------------------------

class WHPConstants_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_whlprotectproc.whpconstants.WHPConstants_array_port) */ {
 public:
  WHPConstants_array_port();
  virtual ~WHPConstants_array_port();

  WHPConstants_array_port(const WHPConstants_array_port& from);
  WHPConstants_array_port(WHPConstants_array_port&& from) noexcept
    : WHPConstants_array_port() {
    *this = ::std::move(from);
  }

  inline WHPConstants_array_port& operator=(const WHPConstants_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline WHPConstants_array_port& operator=(WHPConstants_array_port&& from) noexcept {
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
  static const WHPConstants_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const WHPConstants_array_port* internal_default_instance() {
    return reinterpret_cast<const WHPConstants_array_port*>(
               &_WHPConstants_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(WHPConstants_array_port& a, WHPConstants_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(WHPConstants_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline WHPConstants_array_port* New() const final {
    return CreateMaybeMessage<WHPConstants_array_port>(nullptr);
  }

  WHPConstants_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<WHPConstants_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const WHPConstants_array_port& from);
  void MergeFrom(const WHPConstants_array_port& from);
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
  void InternalSwap(WHPConstants_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_whlprotectproc.whpconstants.WHPConstants_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fwhlprotectproc_2fwhpconstants_2eproto);
    return ::descriptor_table_mf_5fwhlprotectproc_2fwhpconstants_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3185,
  };
  // repeated .pb.mf_whlprotectproc.whpconstants.WHPConstants data = 3185;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_whlprotectproc::whpconstants::WHPConstants* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_whlprotectproc::whpconstants::WHPConstants >*
      mutable_data();
  private:
  const ::pb::mf_whlprotectproc::whpconstants::WHPConstants& _internal_data(int index) const;
  ::pb::mf_whlprotectproc::whpconstants::WHPConstants* _internal_add_data();
  public:
  const ::pb::mf_whlprotectproc::whpconstants::WHPConstants& data(int index) const;
  ::pb::mf_whlprotectproc::whpconstants::WHPConstants* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_whlprotectproc::whpconstants::WHPConstants >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_whlprotectproc.whpconstants.WHPConstants_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_whlprotectproc::whpconstants::WHPConstants > data_;
  friend struct ::TableStruct_mf_5fwhlprotectproc_2fwhpconstants_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// WHPConstants

// optional uint32 NUM_MTS_DEBUG_FREESPACE_WHPP = 245;
inline bool WHPConstants::_internal_has_num_mts_debug_freespace_whpp() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool WHPConstants::has_num_mts_debug_freespace_whpp() const {
  return _internal_has_num_mts_debug_freespace_whpp();
}
inline void WHPConstants::clear_num_mts_debug_freespace_whpp() {
  num_mts_debug_freespace_whpp_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 WHPConstants::_internal_num_mts_debug_freespace_whpp() const {
  return num_mts_debug_freespace_whpp_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 WHPConstants::num_mts_debug_freespace_whpp() const {
  // @@protoc_insertion_point(field_get:pb.mf_whlprotectproc.whpconstants.WHPConstants.NUM_MTS_DEBUG_FREESPACE_WHPP)
  return _internal_num_mts_debug_freespace_whpp();
}
inline void WHPConstants::_internal_set_num_mts_debug_freespace_whpp(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  num_mts_debug_freespace_whpp_ = value;
}
inline void WHPConstants::set_num_mts_debug_freespace_whpp(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_num_mts_debug_freespace_whpp(value);
  // @@protoc_insertion_point(field_set:pb.mf_whlprotectproc.whpconstants.WHPConstants.NUM_MTS_DEBUG_FREESPACE_WHPP)
}

// -------------------------------------------------------------------

// WHPConstants_array_port

// repeated .pb.mf_whlprotectproc.whpconstants.WHPConstants data = 3185;
inline int WHPConstants_array_port::_internal_data_size() const {
  return data_.size();
}
inline int WHPConstants_array_port::data_size() const {
  return _internal_data_size();
}
inline void WHPConstants_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_whlprotectproc::whpconstants::WHPConstants* WHPConstants_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_whlprotectproc.whpconstants.WHPConstants_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_whlprotectproc::whpconstants::WHPConstants >*
WHPConstants_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_whlprotectproc.whpconstants.WHPConstants_array_port.data)
  return &data_;
}
inline const ::pb::mf_whlprotectproc::whpconstants::WHPConstants& WHPConstants_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_whlprotectproc::whpconstants::WHPConstants& WHPConstants_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_whlprotectproc.whpconstants.WHPConstants_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_whlprotectproc::whpconstants::WHPConstants* WHPConstants_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_whlprotectproc::whpconstants::WHPConstants* WHPConstants_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_whlprotectproc.whpconstants.WHPConstants_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_whlprotectproc::whpconstants::WHPConstants >&
WHPConstants_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_whlprotectproc.whpconstants.WHPConstants_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace whpconstants
}  // namespace mf_whlprotectproc
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fwhlprotectproc_2fwhpconstants_2eproto
