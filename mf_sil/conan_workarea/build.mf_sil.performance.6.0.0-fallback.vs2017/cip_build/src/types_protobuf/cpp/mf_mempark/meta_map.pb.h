// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_mempark/meta_map.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmeta_5fmap_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmeta_5fmap_2eproto

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
#include "eco/signal_header.pb.h"
#include "mf_mempark/parking_slot.pb.h"
#include "mf_mempark/parking_trajectory.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fmempark_2fmeta_5fmap_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fmempark_2fmeta_5fmap_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmempark_2fmeta_5fmap_2eproto;
namespace pb {
namespace mf_mempark {
namespace meta_map {
class MetaMap;
class MetaMapDefaultTypeInternal;
extern MetaMapDefaultTypeInternal _MetaMap_default_instance_;
class MetaMap_array_port;
class MetaMap_array_portDefaultTypeInternal;
extern MetaMap_array_portDefaultTypeInternal _MetaMap_array_port_default_instance_;
}  // namespace meta_map
}  // namespace mf_mempark
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_mempark::meta_map::MetaMap* Arena::CreateMaybeMessage<::pb::mf_mempark::meta_map::MetaMap>(Arena*);
template<> ::pb::mf_mempark::meta_map::MetaMap_array_port* Arena::CreateMaybeMessage<::pb::mf_mempark::meta_map::MetaMap_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_mempark {
namespace meta_map {

// ===================================================================

class MetaMap :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.meta_map.MetaMap) */ {
 public:
  MetaMap();
  virtual ~MetaMap();

  MetaMap(const MetaMap& from);
  MetaMap(MetaMap&& from) noexcept
    : MetaMap() {
    *this = ::std::move(from);
  }

  inline MetaMap& operator=(const MetaMap& from) {
    CopyFrom(from);
    return *this;
  }
  inline MetaMap& operator=(MetaMap&& from) noexcept {
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
  static const MetaMap& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MetaMap* internal_default_instance() {
    return reinterpret_cast<const MetaMap*>(
               &_MetaMap_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MetaMap& a, MetaMap& b) {
    a.Swap(&b);
  }
  inline void Swap(MetaMap* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MetaMap* New() const final {
    return CreateMaybeMessage<MetaMap>(nullptr);
  }

  MetaMap* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MetaMap>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MetaMap& from);
  void MergeFrom(const MetaMap& from);
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
  void InternalSwap(MetaMap* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.meta_map.MetaMap";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2fmeta_5fmap_2eproto);
    return ::descriptor_table_mf_5fmempark_2fmeta_5fmap_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kParkingTrajectoriesFieldNumber = 1106,
    kParkingSlotsFieldNumber = 2267,
    kSSigHeaderFieldNumber = 1033,
    kNumValidParkingSlotsFieldNumber = 595,
    kUiVersionNumberFieldNumber = 2124,
    kCorrespondingMapIDFieldNumber = 2794,
    kNumValidTrajectoriesFieldNumber = 3897,
  };
  // repeated .pb.mf_mempark.parking_trajectory.ParkingTrajectory parkingTrajectories = 1106;
  int parkingtrajectories_size() const;
  private:
  int _internal_parkingtrajectories_size() const;
  public:
  void clear_parkingtrajectories();
  ::pb::mf_mempark::parking_trajectory::ParkingTrajectory* mutable_parkingtrajectories(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::parking_trajectory::ParkingTrajectory >*
      mutable_parkingtrajectories();
  private:
  const ::pb::mf_mempark::parking_trajectory::ParkingTrajectory& _internal_parkingtrajectories(int index) const;
  ::pb::mf_mempark::parking_trajectory::ParkingTrajectory* _internal_add_parkingtrajectories();
  public:
  const ::pb::mf_mempark::parking_trajectory::ParkingTrajectory& parkingtrajectories(int index) const;
  ::pb::mf_mempark::parking_trajectory::ParkingTrajectory* add_parkingtrajectories();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::parking_trajectory::ParkingTrajectory >&
      parkingtrajectories() const;

  // repeated .pb.mf_mempark.parking_slot.ParkingSlot parkingSlots = 2267;
  int parkingslots_size() const;
  private:
  int _internal_parkingslots_size() const;
  public:
  void clear_parkingslots();
  ::pb::mf_mempark::parking_slot::ParkingSlot* mutable_parkingslots(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::parking_slot::ParkingSlot >*
      mutable_parkingslots();
  private:
  const ::pb::mf_mempark::parking_slot::ParkingSlot& _internal_parkingslots(int index) const;
  ::pb::mf_mempark::parking_slot::ParkingSlot* _internal_add_parkingslots();
  public:
  const ::pb::mf_mempark::parking_slot::ParkingSlot& parkingslots(int index) const;
  ::pb::mf_mempark::parking_slot::ParkingSlot* add_parkingslots();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::parking_slot::ParkingSlot >&
      parkingslots() const;

  // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
  bool has_ssigheader() const;
  private:
  bool _internal_has_ssigheader() const;
  public:
  void clear_ssigheader();
  const ::pb::eco::signal_header::SignalHeader& ssigheader() const;
  ::pb::eco::signal_header::SignalHeader* release_ssigheader();
  ::pb::eco::signal_header::SignalHeader* mutable_ssigheader();
  void set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader);
  private:
  const ::pb::eco::signal_header::SignalHeader& _internal_ssigheader() const;
  ::pb::eco::signal_header::SignalHeader* _internal_mutable_ssigheader();
  public:

  // optional uint32 numValidParkingSlots = 595;
  bool has_numvalidparkingslots() const;
  private:
  bool _internal_has_numvalidparkingslots() const;
  public:
  void clear_numvalidparkingslots();
  ::PROTOBUF_NAMESPACE_ID::uint32 numvalidparkingslots() const;
  void set_numvalidparkingslots(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_numvalidparkingslots() const;
  void _internal_set_numvalidparkingslots(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 uiVersionNumber = 2124;
  bool has_uiversionnumber() const;
  private:
  bool _internal_has_uiversionnumber() const;
  public:
  void clear_uiversionnumber();
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber() const;
  void set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_uiversionnumber() const;
  void _internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 correspondingMapID = 2794;
  bool has_correspondingmapid() const;
  private:
  bool _internal_has_correspondingmapid() const;
  public:
  void clear_correspondingmapid();
  ::PROTOBUF_NAMESPACE_ID::uint32 correspondingmapid() const;
  void set_correspondingmapid(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_correspondingmapid() const;
  void _internal_set_correspondingmapid(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 numValidTrajectories = 3897;
  bool has_numvalidtrajectories() const;
  private:
  bool _internal_has_numvalidtrajectories() const;
  public:
  void clear_numvalidtrajectories();
  ::PROTOBUF_NAMESPACE_ID::uint32 numvalidtrajectories() const;
  void set_numvalidtrajectories(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_numvalidtrajectories() const;
  void _internal_set_numvalidtrajectories(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.meta_map.MetaMap)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::parking_trajectory::ParkingTrajectory > parkingtrajectories_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::parking_slot::ParkingSlot > parkingslots_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint32 numvalidparkingslots_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  ::PROTOBUF_NAMESPACE_ID::uint32 correspondingmapid_;
  ::PROTOBUF_NAMESPACE_ID::uint32 numvalidtrajectories_;
  friend struct ::TableStruct_mf_5fmempark_2fmeta_5fmap_2eproto;
};
// -------------------------------------------------------------------

class MetaMap_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.meta_map.MetaMap_array_port) */ {
 public:
  MetaMap_array_port();
  virtual ~MetaMap_array_port();

  MetaMap_array_port(const MetaMap_array_port& from);
  MetaMap_array_port(MetaMap_array_port&& from) noexcept
    : MetaMap_array_port() {
    *this = ::std::move(from);
  }

  inline MetaMap_array_port& operator=(const MetaMap_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline MetaMap_array_port& operator=(MetaMap_array_port&& from) noexcept {
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
  static const MetaMap_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MetaMap_array_port* internal_default_instance() {
    return reinterpret_cast<const MetaMap_array_port*>(
               &_MetaMap_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(MetaMap_array_port& a, MetaMap_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(MetaMap_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MetaMap_array_port* New() const final {
    return CreateMaybeMessage<MetaMap_array_port>(nullptr);
  }

  MetaMap_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MetaMap_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MetaMap_array_port& from);
  void MergeFrom(const MetaMap_array_port& from);
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
  void InternalSwap(MetaMap_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.meta_map.MetaMap_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2fmeta_5fmap_2eproto);
    return ::descriptor_table_mf_5fmempark_2fmeta_5fmap_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 675,
  };
  // repeated .pb.mf_mempark.meta_map.MetaMap data = 675;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_mempark::meta_map::MetaMap* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::meta_map::MetaMap >*
      mutable_data();
  private:
  const ::pb::mf_mempark::meta_map::MetaMap& _internal_data(int index) const;
  ::pb::mf_mempark::meta_map::MetaMap* _internal_add_data();
  public:
  const ::pb::mf_mempark::meta_map::MetaMap& data(int index) const;
  ::pb::mf_mempark::meta_map::MetaMap* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::meta_map::MetaMap >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.meta_map.MetaMap_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::meta_map::MetaMap > data_;
  friend struct ::TableStruct_mf_5fmempark_2fmeta_5fmap_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MetaMap

// optional uint32 uiVersionNumber = 2124;
inline bool MetaMap::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool MetaMap::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void MetaMap::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MetaMap::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MetaMap::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.meta_map.MetaMap.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void MetaMap::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  uiversionnumber_ = value;
}
inline void MetaMap::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.meta_map.MetaMap.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool MetaMap::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool MetaMap::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& MetaMap::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& MetaMap::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.meta_map.MetaMap.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* MetaMap::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.mf_mempark.meta_map.MetaMap.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* MetaMap::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* MetaMap::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.meta_map.MetaMap.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void MetaMap::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(ssigheader_);
  }
  if (ssigheader) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      ssigheader = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, ssigheader, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  ssigheader_ = ssigheader;
  // @@protoc_insertion_point(field_set_allocated:pb.mf_mempark.meta_map.MetaMap.sSigHeader)
}

// optional uint32 correspondingMapID = 2794;
inline bool MetaMap::_internal_has_correspondingmapid() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool MetaMap::has_correspondingmapid() const {
  return _internal_has_correspondingmapid();
}
inline void MetaMap::clear_correspondingmapid() {
  correspondingmapid_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MetaMap::_internal_correspondingmapid() const {
  return correspondingmapid_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MetaMap::correspondingmapid() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.meta_map.MetaMap.correspondingMapID)
  return _internal_correspondingmapid();
}
inline void MetaMap::_internal_set_correspondingmapid(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  correspondingmapid_ = value;
}
inline void MetaMap::set_correspondingmapid(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_correspondingmapid(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.meta_map.MetaMap.correspondingMapID)
}

// optional uint32 numValidParkingSlots = 595;
inline bool MetaMap::_internal_has_numvalidparkingslots() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool MetaMap::has_numvalidparkingslots() const {
  return _internal_has_numvalidparkingslots();
}
inline void MetaMap::clear_numvalidparkingslots() {
  numvalidparkingslots_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MetaMap::_internal_numvalidparkingslots() const {
  return numvalidparkingslots_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MetaMap::numvalidparkingslots() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.meta_map.MetaMap.numValidParkingSlots)
  return _internal_numvalidparkingslots();
}
inline void MetaMap::_internal_set_numvalidparkingslots(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  numvalidparkingslots_ = value;
}
inline void MetaMap::set_numvalidparkingslots(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_numvalidparkingslots(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.meta_map.MetaMap.numValidParkingSlots)
}

// repeated .pb.mf_mempark.parking_slot.ParkingSlot parkingSlots = 2267;
inline int MetaMap::_internal_parkingslots_size() const {
  return parkingslots_.size();
}
inline int MetaMap::parkingslots_size() const {
  return _internal_parkingslots_size();
}
inline ::pb::mf_mempark::parking_slot::ParkingSlot* MetaMap::mutable_parkingslots(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.meta_map.MetaMap.parkingSlots)
  return parkingslots_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::parking_slot::ParkingSlot >*
MetaMap::mutable_parkingslots() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_mempark.meta_map.MetaMap.parkingSlots)
  return &parkingslots_;
}
inline const ::pb::mf_mempark::parking_slot::ParkingSlot& MetaMap::_internal_parkingslots(int index) const {
  return parkingslots_.Get(index);
}
inline const ::pb::mf_mempark::parking_slot::ParkingSlot& MetaMap::parkingslots(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.meta_map.MetaMap.parkingSlots)
  return _internal_parkingslots(index);
}
inline ::pb::mf_mempark::parking_slot::ParkingSlot* MetaMap::_internal_add_parkingslots() {
  return parkingslots_.Add();
}
inline ::pb::mf_mempark::parking_slot::ParkingSlot* MetaMap::add_parkingslots() {
  // @@protoc_insertion_point(field_add:pb.mf_mempark.meta_map.MetaMap.parkingSlots)
  return _internal_add_parkingslots();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::parking_slot::ParkingSlot >&
MetaMap::parkingslots() const {
  // @@protoc_insertion_point(field_list:pb.mf_mempark.meta_map.MetaMap.parkingSlots)
  return parkingslots_;
}

// optional uint32 numValidTrajectories = 3897;
inline bool MetaMap::_internal_has_numvalidtrajectories() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool MetaMap::has_numvalidtrajectories() const {
  return _internal_has_numvalidtrajectories();
}
inline void MetaMap::clear_numvalidtrajectories() {
  numvalidtrajectories_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MetaMap::_internal_numvalidtrajectories() const {
  return numvalidtrajectories_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MetaMap::numvalidtrajectories() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.meta_map.MetaMap.numValidTrajectories)
  return _internal_numvalidtrajectories();
}
inline void MetaMap::_internal_set_numvalidtrajectories(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  numvalidtrajectories_ = value;
}
inline void MetaMap::set_numvalidtrajectories(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_numvalidtrajectories(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.meta_map.MetaMap.numValidTrajectories)
}

// repeated .pb.mf_mempark.parking_trajectory.ParkingTrajectory parkingTrajectories = 1106;
inline int MetaMap::_internal_parkingtrajectories_size() const {
  return parkingtrajectories_.size();
}
inline int MetaMap::parkingtrajectories_size() const {
  return _internal_parkingtrajectories_size();
}
inline ::pb::mf_mempark::parking_trajectory::ParkingTrajectory* MetaMap::mutable_parkingtrajectories(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.meta_map.MetaMap.parkingTrajectories)
  return parkingtrajectories_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::parking_trajectory::ParkingTrajectory >*
MetaMap::mutable_parkingtrajectories() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_mempark.meta_map.MetaMap.parkingTrajectories)
  return &parkingtrajectories_;
}
inline const ::pb::mf_mempark::parking_trajectory::ParkingTrajectory& MetaMap::_internal_parkingtrajectories(int index) const {
  return parkingtrajectories_.Get(index);
}
inline const ::pb::mf_mempark::parking_trajectory::ParkingTrajectory& MetaMap::parkingtrajectories(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.meta_map.MetaMap.parkingTrajectories)
  return _internal_parkingtrajectories(index);
}
inline ::pb::mf_mempark::parking_trajectory::ParkingTrajectory* MetaMap::_internal_add_parkingtrajectories() {
  return parkingtrajectories_.Add();
}
inline ::pb::mf_mempark::parking_trajectory::ParkingTrajectory* MetaMap::add_parkingtrajectories() {
  // @@protoc_insertion_point(field_add:pb.mf_mempark.meta_map.MetaMap.parkingTrajectories)
  return _internal_add_parkingtrajectories();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::parking_trajectory::ParkingTrajectory >&
MetaMap::parkingtrajectories() const {
  // @@protoc_insertion_point(field_list:pb.mf_mempark.meta_map.MetaMap.parkingTrajectories)
  return parkingtrajectories_;
}

// -------------------------------------------------------------------

// MetaMap_array_port

// repeated .pb.mf_mempark.meta_map.MetaMap data = 675;
inline int MetaMap_array_port::_internal_data_size() const {
  return data_.size();
}
inline int MetaMap_array_port::data_size() const {
  return _internal_data_size();
}
inline void MetaMap_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_mempark::meta_map::MetaMap* MetaMap_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.meta_map.MetaMap_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::meta_map::MetaMap >*
MetaMap_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_mempark.meta_map.MetaMap_array_port.data)
  return &data_;
}
inline const ::pb::mf_mempark::meta_map::MetaMap& MetaMap_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_mempark::meta_map::MetaMap& MetaMap_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.meta_map.MetaMap_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_mempark::meta_map::MetaMap* MetaMap_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_mempark::meta_map::MetaMap* MetaMap_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_mempark.meta_map.MetaMap_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::meta_map::MetaMap >&
MetaMap_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_mempark.meta_map.MetaMap_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace meta_map
}  // namespace mf_mempark
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmeta_5fmap_2eproto