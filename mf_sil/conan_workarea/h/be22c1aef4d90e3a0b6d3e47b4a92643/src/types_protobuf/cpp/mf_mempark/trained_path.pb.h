// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_mempark/trained_path.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2ftrained_5fpath_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2ftrained_5fpath_2eproto

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
#include "mf_mempark/localization_status.pb.h"
#include "mf_mempark/gpssilent_offering_enabled.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fmempark_2ftrained_5fpath_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fmempark_2ftrained_5fpath_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmempark_2ftrained_5fpath_2eproto;
namespace pb {
namespace mf_mempark {
namespace trained_path {
class TrainedPath;
class TrainedPathDefaultTypeInternal;
extern TrainedPathDefaultTypeInternal _TrainedPath_default_instance_;
class TrainedPath_array_port;
class TrainedPath_array_portDefaultTypeInternal;
extern TrainedPath_array_portDefaultTypeInternal _TrainedPath_array_port_default_instance_;
}  // namespace trained_path
}  // namespace mf_mempark
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_mempark::trained_path::TrainedPath* Arena::CreateMaybeMessage<::pb::mf_mempark::trained_path::TrainedPath>(Arena*);
template<> ::pb::mf_mempark::trained_path::TrainedPath_array_port* Arena::CreateMaybeMessage<::pb::mf_mempark::trained_path::TrainedPath_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_mempark {
namespace trained_path {

// ===================================================================

class TrainedPath :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.trained_path.TrainedPath) */ {
 public:
  TrainedPath();
  virtual ~TrainedPath();

  TrainedPath(const TrainedPath& from);
  TrainedPath(TrainedPath&& from) noexcept
    : TrainedPath() {
    *this = ::std::move(from);
  }

  inline TrainedPath& operator=(const TrainedPath& from) {
    CopyFrom(from);
    return *this;
  }
  inline TrainedPath& operator=(TrainedPath&& from) noexcept {
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
  static const TrainedPath& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TrainedPath* internal_default_instance() {
    return reinterpret_cast<const TrainedPath*>(
               &_TrainedPath_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(TrainedPath& a, TrainedPath& b) {
    a.Swap(&b);
  }
  inline void Swap(TrainedPath* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline TrainedPath* New() const final {
    return CreateMaybeMessage<TrainedPath>(nullptr);
  }

  TrainedPath* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<TrainedPath>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const TrainedPath& from);
  void MergeFrom(const TrainedPath& from);
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
  void InternalSwap(TrainedPath* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.trained_path.TrainedPath";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2ftrained_5fpath_2eproto);
    return ::descriptor_table_mf_5fmempark_2ftrained_5fpath_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kIDFieldNumber = 3354,
    kGpsSilentOfferingEnabledFieldNumber = 1883,
    kRelocalizationStatusFieldNumber = 2813,
  };
  // optional uint32 ID = 3354;
  bool has_id() const;
  private:
  bool _internal_has_id() const;
  public:
  void clear_id();
  ::PROTOBUF_NAMESPACE_ID::uint32 id() const;
  void set_id(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_id() const;
  void _internal_set_id(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional .pb.mf_mempark.gpssilent_offering_enabled.GPSSilentOfferingEnabled gpsSilentOfferingEnabled = 1883;
  bool has_gpssilentofferingenabled() const;
  private:
  bool _internal_has_gpssilentofferingenabled() const;
  public:
  void clear_gpssilentofferingenabled();
  ::pb::mf_mempark::gpssilent_offering_enabled::GPSSilentOfferingEnabled gpssilentofferingenabled() const;
  void set_gpssilentofferingenabled(::pb::mf_mempark::gpssilent_offering_enabled::GPSSilentOfferingEnabled value);
  private:
  ::pb::mf_mempark::gpssilent_offering_enabled::GPSSilentOfferingEnabled _internal_gpssilentofferingenabled() const;
  void _internal_set_gpssilentofferingenabled(::pb::mf_mempark::gpssilent_offering_enabled::GPSSilentOfferingEnabled value);
  public:

  // optional .pb.mf_mempark.localization_status.LocalizationStatus relocalizationStatus = 2813;
  bool has_relocalizationstatus() const;
  private:
  bool _internal_has_relocalizationstatus() const;
  public:
  void clear_relocalizationstatus();
  ::pb::mf_mempark::localization_status::LocalizationStatus relocalizationstatus() const;
  void set_relocalizationstatus(::pb::mf_mempark::localization_status::LocalizationStatus value);
  private:
  ::pb::mf_mempark::localization_status::LocalizationStatus _internal_relocalizationstatus() const;
  void _internal_set_relocalizationstatus(::pb::mf_mempark::localization_status::LocalizationStatus value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.trained_path.TrainedPath)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 id_;
  int gpssilentofferingenabled_;
  int relocalizationstatus_;
  friend struct ::TableStruct_mf_5fmempark_2ftrained_5fpath_2eproto;
};
// -------------------------------------------------------------------

class TrainedPath_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.trained_path.TrainedPath_array_port) */ {
 public:
  TrainedPath_array_port();
  virtual ~TrainedPath_array_port();

  TrainedPath_array_port(const TrainedPath_array_port& from);
  TrainedPath_array_port(TrainedPath_array_port&& from) noexcept
    : TrainedPath_array_port() {
    *this = ::std::move(from);
  }

  inline TrainedPath_array_port& operator=(const TrainedPath_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline TrainedPath_array_port& operator=(TrainedPath_array_port&& from) noexcept {
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
  static const TrainedPath_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TrainedPath_array_port* internal_default_instance() {
    return reinterpret_cast<const TrainedPath_array_port*>(
               &_TrainedPath_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(TrainedPath_array_port& a, TrainedPath_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(TrainedPath_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline TrainedPath_array_port* New() const final {
    return CreateMaybeMessage<TrainedPath_array_port>(nullptr);
  }

  TrainedPath_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<TrainedPath_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const TrainedPath_array_port& from);
  void MergeFrom(const TrainedPath_array_port& from);
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
  void InternalSwap(TrainedPath_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.trained_path.TrainedPath_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2ftrained_5fpath_2eproto);
    return ::descriptor_table_mf_5fmempark_2ftrained_5fpath_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2042,
  };
  // repeated .pb.mf_mempark.trained_path.TrainedPath data = 2042;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_mempark::trained_path::TrainedPath* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::trained_path::TrainedPath >*
      mutable_data();
  private:
  const ::pb::mf_mempark::trained_path::TrainedPath& _internal_data(int index) const;
  ::pb::mf_mempark::trained_path::TrainedPath* _internal_add_data();
  public:
  const ::pb::mf_mempark::trained_path::TrainedPath& data(int index) const;
  ::pb::mf_mempark::trained_path::TrainedPath* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::trained_path::TrainedPath >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.trained_path.TrainedPath_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::trained_path::TrainedPath > data_;
  friend struct ::TableStruct_mf_5fmempark_2ftrained_5fpath_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// TrainedPath

// optional uint32 ID = 3354;
inline bool TrainedPath::_internal_has_id() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool TrainedPath::has_id() const {
  return _internal_has_id();
}
inline void TrainedPath::clear_id() {
  id_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TrainedPath::_internal_id() const {
  return id_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TrainedPath::id() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.trained_path.TrainedPath.ID)
  return _internal_id();
}
inline void TrainedPath::_internal_set_id(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  id_ = value;
}
inline void TrainedPath::set_id(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_id(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.trained_path.TrainedPath.ID)
}

// optional .pb.mf_mempark.localization_status.LocalizationStatus relocalizationStatus = 2813;
inline bool TrainedPath::_internal_has_relocalizationstatus() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool TrainedPath::has_relocalizationstatus() const {
  return _internal_has_relocalizationstatus();
}
inline void TrainedPath::clear_relocalizationstatus() {
  relocalizationstatus_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::mf_mempark::localization_status::LocalizationStatus TrainedPath::_internal_relocalizationstatus() const {
  return static_cast< ::pb::mf_mempark::localization_status::LocalizationStatus >(relocalizationstatus_);
}
inline ::pb::mf_mempark::localization_status::LocalizationStatus TrainedPath::relocalizationstatus() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.trained_path.TrainedPath.relocalizationStatus)
  return _internal_relocalizationstatus();
}
inline void TrainedPath::_internal_set_relocalizationstatus(::pb::mf_mempark::localization_status::LocalizationStatus value) {
  assert(::pb::mf_mempark::localization_status::LocalizationStatus_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  relocalizationstatus_ = value;
}
inline void TrainedPath::set_relocalizationstatus(::pb::mf_mempark::localization_status::LocalizationStatus value) {
  _internal_set_relocalizationstatus(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.trained_path.TrainedPath.relocalizationStatus)
}

// optional .pb.mf_mempark.gpssilent_offering_enabled.GPSSilentOfferingEnabled gpsSilentOfferingEnabled = 1883;
inline bool TrainedPath::_internal_has_gpssilentofferingenabled() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool TrainedPath::has_gpssilentofferingenabled() const {
  return _internal_has_gpssilentofferingenabled();
}
inline void TrainedPath::clear_gpssilentofferingenabled() {
  gpssilentofferingenabled_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::pb::mf_mempark::gpssilent_offering_enabled::GPSSilentOfferingEnabled TrainedPath::_internal_gpssilentofferingenabled() const {
  return static_cast< ::pb::mf_mempark::gpssilent_offering_enabled::GPSSilentOfferingEnabled >(gpssilentofferingenabled_);
}
inline ::pb::mf_mempark::gpssilent_offering_enabled::GPSSilentOfferingEnabled TrainedPath::gpssilentofferingenabled() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.trained_path.TrainedPath.gpsSilentOfferingEnabled)
  return _internal_gpssilentofferingenabled();
}
inline void TrainedPath::_internal_set_gpssilentofferingenabled(::pb::mf_mempark::gpssilent_offering_enabled::GPSSilentOfferingEnabled value) {
  assert(::pb::mf_mempark::gpssilent_offering_enabled::GPSSilentOfferingEnabled_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  gpssilentofferingenabled_ = value;
}
inline void TrainedPath::set_gpssilentofferingenabled(::pb::mf_mempark::gpssilent_offering_enabled::GPSSilentOfferingEnabled value) {
  _internal_set_gpssilentofferingenabled(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.trained_path.TrainedPath.gpsSilentOfferingEnabled)
}

// -------------------------------------------------------------------

// TrainedPath_array_port

// repeated .pb.mf_mempark.trained_path.TrainedPath data = 2042;
inline int TrainedPath_array_port::_internal_data_size() const {
  return data_.size();
}
inline int TrainedPath_array_port::data_size() const {
  return _internal_data_size();
}
inline void TrainedPath_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_mempark::trained_path::TrainedPath* TrainedPath_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.trained_path.TrainedPath_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::trained_path::TrainedPath >*
TrainedPath_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_mempark.trained_path.TrainedPath_array_port.data)
  return &data_;
}
inline const ::pb::mf_mempark::trained_path::TrainedPath& TrainedPath_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_mempark::trained_path::TrainedPath& TrainedPath_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.trained_path.TrainedPath_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_mempark::trained_path::TrainedPath* TrainedPath_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_mempark::trained_path::TrainedPath* TrainedPath_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_mempark.trained_path.TrainedPath_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::trained_path::TrainedPath >&
TrainedPath_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_mempark.trained_path.TrainedPath_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace trained_path
}  // namespace mf_mempark
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2ftrained_5fpath_2eproto