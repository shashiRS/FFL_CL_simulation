// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/delimiter_zones.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_si_2fdelimiter_5fzones_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_si_2fdelimiter_5fzones_2eproto

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
#include "si/quadrilateral_serializable.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_si_2fdelimiter_5fzones_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_si_2fdelimiter_5fzones_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fdelimiter_5fzones_2eproto;
namespace pb {
namespace si {
namespace delimiter_zones {
class DelimiterZones;
class DelimiterZonesDefaultTypeInternal;
extern DelimiterZonesDefaultTypeInternal _DelimiterZones_default_instance_;
class DelimiterZones_array_port;
class DelimiterZones_array_portDefaultTypeInternal;
extern DelimiterZones_array_portDefaultTypeInternal _DelimiterZones_array_port_default_instance_;
}  // namespace delimiter_zones
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::si::delimiter_zones::DelimiterZones* Arena::CreateMaybeMessage<::pb::si::delimiter_zones::DelimiterZones>(Arena*);
template<> ::pb::si::delimiter_zones::DelimiterZones_array_port* Arena::CreateMaybeMessage<::pb::si::delimiter_zones::DelimiterZones_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace si {
namespace delimiter_zones {

// ===================================================================

class DelimiterZones :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.delimiter_zones.DelimiterZones) */ {
 public:
  DelimiterZones();
  virtual ~DelimiterZones();

  DelimiterZones(const DelimiterZones& from);
  DelimiterZones(DelimiterZones&& from) noexcept
    : DelimiterZones() {
    *this = ::std::move(from);
  }

  inline DelimiterZones& operator=(const DelimiterZones& from) {
    CopyFrom(from);
    return *this;
  }
  inline DelimiterZones& operator=(DelimiterZones&& from) noexcept {
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
  static const DelimiterZones& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DelimiterZones* internal_default_instance() {
    return reinterpret_cast<const DelimiterZones*>(
               &_DelimiterZones_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(DelimiterZones& a, DelimiterZones& b) {
    a.Swap(&b);
  }
  inline void Swap(DelimiterZones* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline DelimiterZones* New() const final {
    return CreateMaybeMessage<DelimiterZones>(nullptr);
  }

  DelimiterZones* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<DelimiterZones>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const DelimiterZones& from);
  void MergeFrom(const DelimiterZones& from);
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
  void InternalSwap(DelimiterZones* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.delimiter_zones.DelimiterZones";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fdelimiter_5fzones_2eproto);
    return ::descriptor_table_si_2fdelimiter_5fzones_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kRightZoneFieldNumber = 408,
    kLeftZoneFieldNumber = 1670,
    kRoadZoneFieldNumber = 2678,
    kInsideZoneFieldNumber = 2791,
    kAllFieldNumber = 2879,
    kCurbZoneFieldNumber = 3525,
    kSlotIdNuFieldNumber = 3547,
  };
  // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable rightZone = 408;
  bool has_rightzone() const;
  private:
  bool _internal_has_rightzone() const;
  public:
  void clear_rightzone();
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& rightzone() const;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* release_rightzone();
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* mutable_rightzone();
  void set_allocated_rightzone(::pb::si::quadrilateral_serializable::QuadrilateralSerializable* rightzone);
  private:
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& _internal_rightzone() const;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* _internal_mutable_rightzone();
  public:

  // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable leftZone = 1670;
  bool has_leftzone() const;
  private:
  bool _internal_has_leftzone() const;
  public:
  void clear_leftzone();
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& leftzone() const;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* release_leftzone();
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* mutable_leftzone();
  void set_allocated_leftzone(::pb::si::quadrilateral_serializable::QuadrilateralSerializable* leftzone);
  private:
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& _internal_leftzone() const;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* _internal_mutable_leftzone();
  public:

  // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable roadZone = 2678;
  bool has_roadzone() const;
  private:
  bool _internal_has_roadzone() const;
  public:
  void clear_roadzone();
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& roadzone() const;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* release_roadzone();
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* mutable_roadzone();
  void set_allocated_roadzone(::pb::si::quadrilateral_serializable::QuadrilateralSerializable* roadzone);
  private:
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& _internal_roadzone() const;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* _internal_mutable_roadzone();
  public:

  // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable insideZone = 2791;
  bool has_insidezone() const;
  private:
  bool _internal_has_insidezone() const;
  public:
  void clear_insidezone();
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& insidezone() const;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* release_insidezone();
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* mutable_insidezone();
  void set_allocated_insidezone(::pb::si::quadrilateral_serializable::QuadrilateralSerializable* insidezone);
  private:
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& _internal_insidezone() const;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* _internal_mutable_insidezone();
  public:

  // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable all = 2879;
  bool has_all() const;
  private:
  bool _internal_has_all() const;
  public:
  void clear_all();
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& all() const;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* release_all();
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* mutable_all();
  void set_allocated_all(::pb::si::quadrilateral_serializable::QuadrilateralSerializable* all);
  private:
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& _internal_all() const;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* _internal_mutable_all();
  public:

  // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable curbZone = 3525;
  bool has_curbzone() const;
  private:
  bool _internal_has_curbzone() const;
  public:
  void clear_curbzone();
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& curbzone() const;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* release_curbzone();
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* mutable_curbzone();
  void set_allocated_curbzone(::pb::si::quadrilateral_serializable::QuadrilateralSerializable* curbzone);
  private:
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& _internal_curbzone() const;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* _internal_mutable_curbzone();
  public:

  // optional uint32 slotId_nu = 3547;
  bool has_slotid_nu() const;
  private:
  bool _internal_has_slotid_nu() const;
  public:
  void clear_slotid_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 slotid_nu() const;
  void set_slotid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_slotid_nu() const;
  void _internal_set_slotid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.si.delimiter_zones.DelimiterZones)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* rightzone_;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* leftzone_;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* roadzone_;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* insidezone_;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* all_;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* curbzone_;
  ::PROTOBUF_NAMESPACE_ID::uint32 slotid_nu_;
  friend struct ::TableStruct_si_2fdelimiter_5fzones_2eproto;
};
// -------------------------------------------------------------------

class DelimiterZones_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.delimiter_zones.DelimiterZones_array_port) */ {
 public:
  DelimiterZones_array_port();
  virtual ~DelimiterZones_array_port();

  DelimiterZones_array_port(const DelimiterZones_array_port& from);
  DelimiterZones_array_port(DelimiterZones_array_port&& from) noexcept
    : DelimiterZones_array_port() {
    *this = ::std::move(from);
  }

  inline DelimiterZones_array_port& operator=(const DelimiterZones_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline DelimiterZones_array_port& operator=(DelimiterZones_array_port&& from) noexcept {
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
  static const DelimiterZones_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DelimiterZones_array_port* internal_default_instance() {
    return reinterpret_cast<const DelimiterZones_array_port*>(
               &_DelimiterZones_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(DelimiterZones_array_port& a, DelimiterZones_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(DelimiterZones_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline DelimiterZones_array_port* New() const final {
    return CreateMaybeMessage<DelimiterZones_array_port>(nullptr);
  }

  DelimiterZones_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<DelimiterZones_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const DelimiterZones_array_port& from);
  void MergeFrom(const DelimiterZones_array_port& from);
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
  void InternalSwap(DelimiterZones_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.delimiter_zones.DelimiterZones_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fdelimiter_5fzones_2eproto);
    return ::descriptor_table_si_2fdelimiter_5fzones_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3950,
  };
  // repeated .pb.si.delimiter_zones.DelimiterZones data = 3950;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::si::delimiter_zones::DelimiterZones* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::delimiter_zones::DelimiterZones >*
      mutable_data();
  private:
  const ::pb::si::delimiter_zones::DelimiterZones& _internal_data(int index) const;
  ::pb::si::delimiter_zones::DelimiterZones* _internal_add_data();
  public:
  const ::pb::si::delimiter_zones::DelimiterZones& data(int index) const;
  ::pb::si::delimiter_zones::DelimiterZones* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::delimiter_zones::DelimiterZones >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.si.delimiter_zones.DelimiterZones_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::delimiter_zones::DelimiterZones > data_;
  friend struct ::TableStruct_si_2fdelimiter_5fzones_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// DelimiterZones

// optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable curbZone = 3525;
inline bool DelimiterZones::_internal_has_curbzone() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  PROTOBUF_ASSUME(!value || curbzone_ != nullptr);
  return value;
}
inline bool DelimiterZones::has_curbzone() const {
  return _internal_has_curbzone();
}
inline const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& DelimiterZones::_internal_curbzone() const {
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* p = curbzone_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable*>(
      &::pb::si::quadrilateral_serializable::_QuadrilateralSerializable_default_instance_);
}
inline const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& DelimiterZones::curbzone() const {
  // @@protoc_insertion_point(field_get:pb.si.delimiter_zones.DelimiterZones.curbZone)
  return _internal_curbzone();
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::release_curbzone() {
  // @@protoc_insertion_point(field_release:pb.si.delimiter_zones.DelimiterZones.curbZone)
  _has_bits_[0] &= ~0x00000020u;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* temp = curbzone_;
  curbzone_ = nullptr;
  return temp;
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::_internal_mutable_curbzone() {
  _has_bits_[0] |= 0x00000020u;
  if (curbzone_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::quadrilateral_serializable::QuadrilateralSerializable>(GetArenaNoVirtual());
    curbzone_ = p;
  }
  return curbzone_;
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::mutable_curbzone() {
  // @@protoc_insertion_point(field_mutable:pb.si.delimiter_zones.DelimiterZones.curbZone)
  return _internal_mutable_curbzone();
}
inline void DelimiterZones::set_allocated_curbzone(::pb::si::quadrilateral_serializable::QuadrilateralSerializable* curbzone) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(curbzone_);
  }
  if (curbzone) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      curbzone = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, curbzone, submessage_arena);
    }
    _has_bits_[0] |= 0x00000020u;
  } else {
    _has_bits_[0] &= ~0x00000020u;
  }
  curbzone_ = curbzone;
  // @@protoc_insertion_point(field_set_allocated:pb.si.delimiter_zones.DelimiterZones.curbZone)
}

// optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable roadZone = 2678;
inline bool DelimiterZones::_internal_has_roadzone() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  PROTOBUF_ASSUME(!value || roadzone_ != nullptr);
  return value;
}
inline bool DelimiterZones::has_roadzone() const {
  return _internal_has_roadzone();
}
inline const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& DelimiterZones::_internal_roadzone() const {
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* p = roadzone_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable*>(
      &::pb::si::quadrilateral_serializable::_QuadrilateralSerializable_default_instance_);
}
inline const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& DelimiterZones::roadzone() const {
  // @@protoc_insertion_point(field_get:pb.si.delimiter_zones.DelimiterZones.roadZone)
  return _internal_roadzone();
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::release_roadzone() {
  // @@protoc_insertion_point(field_release:pb.si.delimiter_zones.DelimiterZones.roadZone)
  _has_bits_[0] &= ~0x00000004u;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* temp = roadzone_;
  roadzone_ = nullptr;
  return temp;
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::_internal_mutable_roadzone() {
  _has_bits_[0] |= 0x00000004u;
  if (roadzone_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::quadrilateral_serializable::QuadrilateralSerializable>(GetArenaNoVirtual());
    roadzone_ = p;
  }
  return roadzone_;
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::mutable_roadzone() {
  // @@protoc_insertion_point(field_mutable:pb.si.delimiter_zones.DelimiterZones.roadZone)
  return _internal_mutable_roadzone();
}
inline void DelimiterZones::set_allocated_roadzone(::pb::si::quadrilateral_serializable::QuadrilateralSerializable* roadzone) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(roadzone_);
  }
  if (roadzone) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      roadzone = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, roadzone, submessage_arena);
    }
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  roadzone_ = roadzone;
  // @@protoc_insertion_point(field_set_allocated:pb.si.delimiter_zones.DelimiterZones.roadZone)
}

// optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable leftZone = 1670;
inline bool DelimiterZones::_internal_has_leftzone() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || leftzone_ != nullptr);
  return value;
}
inline bool DelimiterZones::has_leftzone() const {
  return _internal_has_leftzone();
}
inline const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& DelimiterZones::_internal_leftzone() const {
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* p = leftzone_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable*>(
      &::pb::si::quadrilateral_serializable::_QuadrilateralSerializable_default_instance_);
}
inline const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& DelimiterZones::leftzone() const {
  // @@protoc_insertion_point(field_get:pb.si.delimiter_zones.DelimiterZones.leftZone)
  return _internal_leftzone();
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::release_leftzone() {
  // @@protoc_insertion_point(field_release:pb.si.delimiter_zones.DelimiterZones.leftZone)
  _has_bits_[0] &= ~0x00000002u;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* temp = leftzone_;
  leftzone_ = nullptr;
  return temp;
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::_internal_mutable_leftzone() {
  _has_bits_[0] |= 0x00000002u;
  if (leftzone_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::quadrilateral_serializable::QuadrilateralSerializable>(GetArenaNoVirtual());
    leftzone_ = p;
  }
  return leftzone_;
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::mutable_leftzone() {
  // @@protoc_insertion_point(field_mutable:pb.si.delimiter_zones.DelimiterZones.leftZone)
  return _internal_mutable_leftzone();
}
inline void DelimiterZones::set_allocated_leftzone(::pb::si::quadrilateral_serializable::QuadrilateralSerializable* leftzone) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(leftzone_);
  }
  if (leftzone) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      leftzone = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, leftzone, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  leftzone_ = leftzone;
  // @@protoc_insertion_point(field_set_allocated:pb.si.delimiter_zones.DelimiterZones.leftZone)
}

// optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable rightZone = 408;
inline bool DelimiterZones::_internal_has_rightzone() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || rightzone_ != nullptr);
  return value;
}
inline bool DelimiterZones::has_rightzone() const {
  return _internal_has_rightzone();
}
inline const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& DelimiterZones::_internal_rightzone() const {
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* p = rightzone_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable*>(
      &::pb::si::quadrilateral_serializable::_QuadrilateralSerializable_default_instance_);
}
inline const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& DelimiterZones::rightzone() const {
  // @@protoc_insertion_point(field_get:pb.si.delimiter_zones.DelimiterZones.rightZone)
  return _internal_rightzone();
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::release_rightzone() {
  // @@protoc_insertion_point(field_release:pb.si.delimiter_zones.DelimiterZones.rightZone)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* temp = rightzone_;
  rightzone_ = nullptr;
  return temp;
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::_internal_mutable_rightzone() {
  _has_bits_[0] |= 0x00000001u;
  if (rightzone_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::quadrilateral_serializable::QuadrilateralSerializable>(GetArenaNoVirtual());
    rightzone_ = p;
  }
  return rightzone_;
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::mutable_rightzone() {
  // @@protoc_insertion_point(field_mutable:pb.si.delimiter_zones.DelimiterZones.rightZone)
  return _internal_mutable_rightzone();
}
inline void DelimiterZones::set_allocated_rightzone(::pb::si::quadrilateral_serializable::QuadrilateralSerializable* rightzone) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(rightzone_);
  }
  if (rightzone) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      rightzone = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, rightzone, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  rightzone_ = rightzone;
  // @@protoc_insertion_point(field_set_allocated:pb.si.delimiter_zones.DelimiterZones.rightZone)
}

// optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable insideZone = 2791;
inline bool DelimiterZones::_internal_has_insidezone() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  PROTOBUF_ASSUME(!value || insidezone_ != nullptr);
  return value;
}
inline bool DelimiterZones::has_insidezone() const {
  return _internal_has_insidezone();
}
inline const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& DelimiterZones::_internal_insidezone() const {
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* p = insidezone_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable*>(
      &::pb::si::quadrilateral_serializable::_QuadrilateralSerializable_default_instance_);
}
inline const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& DelimiterZones::insidezone() const {
  // @@protoc_insertion_point(field_get:pb.si.delimiter_zones.DelimiterZones.insideZone)
  return _internal_insidezone();
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::release_insidezone() {
  // @@protoc_insertion_point(field_release:pb.si.delimiter_zones.DelimiterZones.insideZone)
  _has_bits_[0] &= ~0x00000008u;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* temp = insidezone_;
  insidezone_ = nullptr;
  return temp;
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::_internal_mutable_insidezone() {
  _has_bits_[0] |= 0x00000008u;
  if (insidezone_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::quadrilateral_serializable::QuadrilateralSerializable>(GetArenaNoVirtual());
    insidezone_ = p;
  }
  return insidezone_;
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::mutable_insidezone() {
  // @@protoc_insertion_point(field_mutable:pb.si.delimiter_zones.DelimiterZones.insideZone)
  return _internal_mutable_insidezone();
}
inline void DelimiterZones::set_allocated_insidezone(::pb::si::quadrilateral_serializable::QuadrilateralSerializable* insidezone) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(insidezone_);
  }
  if (insidezone) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      insidezone = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, insidezone, submessage_arena);
    }
    _has_bits_[0] |= 0x00000008u;
  } else {
    _has_bits_[0] &= ~0x00000008u;
  }
  insidezone_ = insidezone;
  // @@protoc_insertion_point(field_set_allocated:pb.si.delimiter_zones.DelimiterZones.insideZone)
}

// optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable all = 2879;
inline bool DelimiterZones::_internal_has_all() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  PROTOBUF_ASSUME(!value || all_ != nullptr);
  return value;
}
inline bool DelimiterZones::has_all() const {
  return _internal_has_all();
}
inline const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& DelimiterZones::_internal_all() const {
  const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* p = all_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable*>(
      &::pb::si::quadrilateral_serializable::_QuadrilateralSerializable_default_instance_);
}
inline const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& DelimiterZones::all() const {
  // @@protoc_insertion_point(field_get:pb.si.delimiter_zones.DelimiterZones.all)
  return _internal_all();
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::release_all() {
  // @@protoc_insertion_point(field_release:pb.si.delimiter_zones.DelimiterZones.all)
  _has_bits_[0] &= ~0x00000010u;
  ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* temp = all_;
  all_ = nullptr;
  return temp;
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::_internal_mutable_all() {
  _has_bits_[0] |= 0x00000010u;
  if (all_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::quadrilateral_serializable::QuadrilateralSerializable>(GetArenaNoVirtual());
    all_ = p;
  }
  return all_;
}
inline ::pb::si::quadrilateral_serializable::QuadrilateralSerializable* DelimiterZones::mutable_all() {
  // @@protoc_insertion_point(field_mutable:pb.si.delimiter_zones.DelimiterZones.all)
  return _internal_mutable_all();
}
inline void DelimiterZones::set_allocated_all(::pb::si::quadrilateral_serializable::QuadrilateralSerializable* all) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(all_);
  }
  if (all) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      all = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, all, submessage_arena);
    }
    _has_bits_[0] |= 0x00000010u;
  } else {
    _has_bits_[0] &= ~0x00000010u;
  }
  all_ = all;
  // @@protoc_insertion_point(field_set_allocated:pb.si.delimiter_zones.DelimiterZones.all)
}

// optional uint32 slotId_nu = 3547;
inline bool DelimiterZones::_internal_has_slotid_nu() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool DelimiterZones::has_slotid_nu() const {
  return _internal_has_slotid_nu();
}
inline void DelimiterZones::clear_slotid_nu() {
  slotid_nu_ = 0u;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DelimiterZones::_internal_slotid_nu() const {
  return slotid_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DelimiterZones::slotid_nu() const {
  // @@protoc_insertion_point(field_get:pb.si.delimiter_zones.DelimiterZones.slotId_nu)
  return _internal_slotid_nu();
}
inline void DelimiterZones::_internal_set_slotid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000040u;
  slotid_nu_ = value;
}
inline void DelimiterZones::set_slotid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_slotid_nu(value);
  // @@protoc_insertion_point(field_set:pb.si.delimiter_zones.DelimiterZones.slotId_nu)
}

// -------------------------------------------------------------------

// DelimiterZones_array_port

// repeated .pb.si.delimiter_zones.DelimiterZones data = 3950;
inline int DelimiterZones_array_port::_internal_data_size() const {
  return data_.size();
}
inline int DelimiterZones_array_port::data_size() const {
  return _internal_data_size();
}
inline void DelimiterZones_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::si::delimiter_zones::DelimiterZones* DelimiterZones_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.si.delimiter_zones.DelimiterZones_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::delimiter_zones::DelimiterZones >*
DelimiterZones_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.si.delimiter_zones.DelimiterZones_array_port.data)
  return &data_;
}
inline const ::pb::si::delimiter_zones::DelimiterZones& DelimiterZones_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::si::delimiter_zones::DelimiterZones& DelimiterZones_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.si.delimiter_zones.DelimiterZones_array_port.data)
  return _internal_data(index);
}
inline ::pb::si::delimiter_zones::DelimiterZones* DelimiterZones_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::si::delimiter_zones::DelimiterZones* DelimiterZones_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.si.delimiter_zones.DelimiterZones_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::delimiter_zones::DelimiterZones >&
DelimiterZones_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.si.delimiter_zones.DelimiterZones_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace delimiter_zones
}  // namespace si
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_si_2fdelimiter_5fzones_2eproto
