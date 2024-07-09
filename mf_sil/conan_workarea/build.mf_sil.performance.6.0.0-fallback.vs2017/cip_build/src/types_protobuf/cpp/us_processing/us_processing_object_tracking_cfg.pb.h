// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_processing/us_processing_object_tracking_cfg.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fprocessing_2fus_5fprocessing_5fobject_5ftracking_5fcfg_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fprocessing_2fus_5fprocessing_5fobject_5ftracking_5fcfg_2eproto

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
#include "us_processing/us_processing_neighboring_filter_cfg.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_us_5fprocessing_2fus_5fprocessing_5fobject_5ftracking_5fcfg_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fprocessing_2fus_5fprocessing_5fobject_5ftracking_5fcfg_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fprocessing_2fus_5fprocessing_5fobject_5ftracking_5fcfg_2eproto;
namespace pb {
namespace us_processing {
namespace us_processing_object_tracking_cfg {
class UsProcessingObjectTrackingCfg;
class UsProcessingObjectTrackingCfgDefaultTypeInternal;
extern UsProcessingObjectTrackingCfgDefaultTypeInternal _UsProcessingObjectTrackingCfg_default_instance_;
class UsProcessingObjectTrackingCfg_array_port;
class UsProcessingObjectTrackingCfg_array_portDefaultTypeInternal;
extern UsProcessingObjectTrackingCfg_array_portDefaultTypeInternal _UsProcessingObjectTrackingCfg_array_port_default_instance_;
}  // namespace us_processing_object_tracking_cfg
}  // namespace us_processing
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg* Arena::CreateMaybeMessage<::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg>(Arena*);
template<> ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg_array_port* Arena::CreateMaybeMessage<::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_processing {
namespace us_processing_object_tracking_cfg {

// ===================================================================

class UsProcessingObjectTrackingCfg :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg) */ {
 public:
  UsProcessingObjectTrackingCfg();
  virtual ~UsProcessingObjectTrackingCfg();

  UsProcessingObjectTrackingCfg(const UsProcessingObjectTrackingCfg& from);
  UsProcessingObjectTrackingCfg(UsProcessingObjectTrackingCfg&& from) noexcept
    : UsProcessingObjectTrackingCfg() {
    *this = ::std::move(from);
  }

  inline UsProcessingObjectTrackingCfg& operator=(const UsProcessingObjectTrackingCfg& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsProcessingObjectTrackingCfg& operator=(UsProcessingObjectTrackingCfg&& from) noexcept {
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
  static const UsProcessingObjectTrackingCfg& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsProcessingObjectTrackingCfg* internal_default_instance() {
    return reinterpret_cast<const UsProcessingObjectTrackingCfg*>(
               &_UsProcessingObjectTrackingCfg_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsProcessingObjectTrackingCfg& a, UsProcessingObjectTrackingCfg& b) {
    a.Swap(&b);
  }
  inline void Swap(UsProcessingObjectTrackingCfg* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsProcessingObjectTrackingCfg* New() const final {
    return CreateMaybeMessage<UsProcessingObjectTrackingCfg>(nullptr);
  }

  UsProcessingObjectTrackingCfg* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsProcessingObjectTrackingCfg>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsProcessingObjectTrackingCfg& from);
  void MergeFrom(const UsProcessingObjectTrackingCfg& from);
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
  void InternalSwap(UsProcessingObjectTrackingCfg* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fobject_5ftracking_5fcfg_2eproto);
    return ::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fobject_5ftracking_5fcfg_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kUsnfParamsFieldNumber = 176,
  };
  // optional .pb.us_processing.us_processing_neighboring_filter_cfg.UsProcessingNeighboringFilterCfg usnfParams = 176;
  bool has_usnfparams() const;
  private:
  bool _internal_has_usnfparams() const;
  public:
  void clear_usnfparams();
  const ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg& usnfparams() const;
  ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg* release_usnfparams();
  ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg* mutable_usnfparams();
  void set_allocated_usnfparams(::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg* usnfparams);
  private:
  const ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg& _internal_usnfparams() const;
  ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg* _internal_mutable_usnfparams();
  public:

  // @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg* usnfparams_;
  friend struct ::TableStruct_us_5fprocessing_2fus_5fprocessing_5fobject_5ftracking_5fcfg_2eproto;
};
// -------------------------------------------------------------------

class UsProcessingObjectTrackingCfg_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg_array_port) */ {
 public:
  UsProcessingObjectTrackingCfg_array_port();
  virtual ~UsProcessingObjectTrackingCfg_array_port();

  UsProcessingObjectTrackingCfg_array_port(const UsProcessingObjectTrackingCfg_array_port& from);
  UsProcessingObjectTrackingCfg_array_port(UsProcessingObjectTrackingCfg_array_port&& from) noexcept
    : UsProcessingObjectTrackingCfg_array_port() {
    *this = ::std::move(from);
  }

  inline UsProcessingObjectTrackingCfg_array_port& operator=(const UsProcessingObjectTrackingCfg_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsProcessingObjectTrackingCfg_array_port& operator=(UsProcessingObjectTrackingCfg_array_port&& from) noexcept {
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
  static const UsProcessingObjectTrackingCfg_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsProcessingObjectTrackingCfg_array_port* internal_default_instance() {
    return reinterpret_cast<const UsProcessingObjectTrackingCfg_array_port*>(
               &_UsProcessingObjectTrackingCfg_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsProcessingObjectTrackingCfg_array_port& a, UsProcessingObjectTrackingCfg_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsProcessingObjectTrackingCfg_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsProcessingObjectTrackingCfg_array_port* New() const final {
    return CreateMaybeMessage<UsProcessingObjectTrackingCfg_array_port>(nullptr);
  }

  UsProcessingObjectTrackingCfg_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsProcessingObjectTrackingCfg_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsProcessingObjectTrackingCfg_array_port& from);
  void MergeFrom(const UsProcessingObjectTrackingCfg_array_port& from);
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
  void InternalSwap(UsProcessingObjectTrackingCfg_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fobject_5ftracking_5fcfg_2eproto);
    return ::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fobject_5ftracking_5fcfg_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3923,
  };
  // repeated .pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg data = 3923;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg >*
      mutable_data();
  private:
  const ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg& _internal_data(int index) const;
  ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg* _internal_add_data();
  public:
  const ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg& data(int index) const;
  ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg > data_;
  friend struct ::TableStruct_us_5fprocessing_2fus_5fprocessing_5fobject_5ftracking_5fcfg_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsProcessingObjectTrackingCfg

// optional .pb.us_processing.us_processing_neighboring_filter_cfg.UsProcessingNeighboringFilterCfg usnfParams = 176;
inline bool UsProcessingObjectTrackingCfg::_internal_has_usnfparams() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || usnfparams_ != nullptr);
  return value;
}
inline bool UsProcessingObjectTrackingCfg::has_usnfparams() const {
  return _internal_has_usnfparams();
}
inline const ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg& UsProcessingObjectTrackingCfg::_internal_usnfparams() const {
  const ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg* p = usnfparams_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg*>(
      &::pb::us_processing::us_processing_neighboring_filter_cfg::_UsProcessingNeighboringFilterCfg_default_instance_);
}
inline const ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg& UsProcessingObjectTrackingCfg::usnfparams() const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg.usnfParams)
  return _internal_usnfparams();
}
inline ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg* UsProcessingObjectTrackingCfg::release_usnfparams() {
  // @@protoc_insertion_point(field_release:pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg.usnfParams)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg* temp = usnfparams_;
  usnfparams_ = nullptr;
  return temp;
}
inline ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg* UsProcessingObjectTrackingCfg::_internal_mutable_usnfparams() {
  _has_bits_[0] |= 0x00000001u;
  if (usnfparams_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg>(GetArenaNoVirtual());
    usnfparams_ = p;
  }
  return usnfparams_;
}
inline ::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg* UsProcessingObjectTrackingCfg::mutable_usnfparams() {
  // @@protoc_insertion_point(field_mutable:pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg.usnfParams)
  return _internal_mutable_usnfparams();
}
inline void UsProcessingObjectTrackingCfg::set_allocated_usnfparams(::pb::us_processing::us_processing_neighboring_filter_cfg::UsProcessingNeighboringFilterCfg* usnfparams) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(usnfparams_);
  }
  if (usnfparams) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      usnfparams = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, usnfparams, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  usnfparams_ = usnfparams;
  // @@protoc_insertion_point(field_set_allocated:pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg.usnfParams)
}

// -------------------------------------------------------------------

// UsProcessingObjectTrackingCfg_array_port

// repeated .pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg data = 3923;
inline int UsProcessingObjectTrackingCfg_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsProcessingObjectTrackingCfg_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsProcessingObjectTrackingCfg_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg* UsProcessingObjectTrackingCfg_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg >*
UsProcessingObjectTrackingCfg_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg_array_port.data)
  return &data_;
}
inline const ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg& UsProcessingObjectTrackingCfg_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg& UsProcessingObjectTrackingCfg_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg* UsProcessingObjectTrackingCfg_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg* UsProcessingObjectTrackingCfg_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_object_tracking_cfg::UsProcessingObjectTrackingCfg >&
UsProcessingObjectTrackingCfg_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_processing_object_tracking_cfg
}  // namespace us_processing
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fprocessing_2fus_5fprocessing_5fobject_5ftracking_5fcfg_2eproto