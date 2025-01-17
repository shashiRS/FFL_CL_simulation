// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_mempark/maps_to_meta_maps.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmaps_5fto_5fmeta_5fmaps_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmaps_5fto_5fmeta_5fmaps_2eproto

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
#include "mf_mempark/map_idto_meta_map.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fmempark_2fmaps_5fto_5fmeta_5fmaps_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fmempark_2fmaps_5fto_5fmeta_5fmaps_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmempark_2fmaps_5fto_5fmeta_5fmaps_2eproto;
namespace pb {
namespace mf_mempark {
namespace maps_to_meta_maps {
class MapsToMetaMaps;
class MapsToMetaMapsDefaultTypeInternal;
extern MapsToMetaMapsDefaultTypeInternal _MapsToMetaMaps_default_instance_;
class MapsToMetaMaps_array_port;
class MapsToMetaMaps_array_portDefaultTypeInternal;
extern MapsToMetaMaps_array_portDefaultTypeInternal _MapsToMetaMaps_array_port_default_instance_;
}  // namespace maps_to_meta_maps
}  // namespace mf_mempark
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps* Arena::CreateMaybeMessage<::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps>(Arena*);
template<> ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps_array_port* Arena::CreateMaybeMessage<::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_mempark {
namespace maps_to_meta_maps {

// ===================================================================

class MapsToMetaMaps :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps) */ {
 public:
  MapsToMetaMaps();
  virtual ~MapsToMetaMaps();

  MapsToMetaMaps(const MapsToMetaMaps& from);
  MapsToMetaMaps(MapsToMetaMaps&& from) noexcept
    : MapsToMetaMaps() {
    *this = ::std::move(from);
  }

  inline MapsToMetaMaps& operator=(const MapsToMetaMaps& from) {
    CopyFrom(from);
    return *this;
  }
  inline MapsToMetaMaps& operator=(MapsToMetaMaps&& from) noexcept {
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
  static const MapsToMetaMaps& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MapsToMetaMaps* internal_default_instance() {
    return reinterpret_cast<const MapsToMetaMaps*>(
               &_MapsToMetaMaps_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MapsToMetaMaps& a, MapsToMetaMaps& b) {
    a.Swap(&b);
  }
  inline void Swap(MapsToMetaMaps* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MapsToMetaMaps* New() const final {
    return CreateMaybeMessage<MapsToMetaMaps>(nullptr);
  }

  MapsToMetaMaps* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MapsToMetaMaps>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MapsToMetaMaps& from);
  void MergeFrom(const MapsToMetaMaps& from);
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
  void InternalSwap(MapsToMetaMaps* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2fmaps_5fto_5fmeta_5fmaps_2eproto);
    return ::descriptor_table_mf_5fmempark_2fmaps_5fto_5fmeta_5fmaps_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kMapsMetaMapsFieldNumber = 1338,
    kSSigHeaderFieldNumber = 1033,
    kNumValidMapsFieldNumber = 853,
    kUiVersionNumberFieldNumber = 2124,
  };
  // repeated .pb.mf_mempark.map_idto_meta_map.MapIDToMetaMap mapsMetaMaps = 1338;
  int mapsmetamaps_size() const;
  private:
  int _internal_mapsmetamaps_size() const;
  public:
  void clear_mapsmetamaps();
  ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap* mutable_mapsmetamaps(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap >*
      mutable_mapsmetamaps();
  private:
  const ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap& _internal_mapsmetamaps(int index) const;
  ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap* _internal_add_mapsmetamaps();
  public:
  const ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap& mapsmetamaps(int index) const;
  ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap* add_mapsmetamaps();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap >&
      mapsmetamaps() const;

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

  // optional uint32 numValidMaps = 853;
  bool has_numvalidmaps() const;
  private:
  bool _internal_has_numvalidmaps() const;
  public:
  void clear_numvalidmaps();
  ::PROTOBUF_NAMESPACE_ID::uint32 numvalidmaps() const;
  void set_numvalidmaps(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_numvalidmaps() const;
  void _internal_set_numvalidmaps(::PROTOBUF_NAMESPACE_ID::uint32 value);
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

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap > mapsmetamaps_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint32 numvalidmaps_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  friend struct ::TableStruct_mf_5fmempark_2fmaps_5fto_5fmeta_5fmaps_2eproto;
};
// -------------------------------------------------------------------

class MapsToMetaMaps_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps_array_port) */ {
 public:
  MapsToMetaMaps_array_port();
  virtual ~MapsToMetaMaps_array_port();

  MapsToMetaMaps_array_port(const MapsToMetaMaps_array_port& from);
  MapsToMetaMaps_array_port(MapsToMetaMaps_array_port&& from) noexcept
    : MapsToMetaMaps_array_port() {
    *this = ::std::move(from);
  }

  inline MapsToMetaMaps_array_port& operator=(const MapsToMetaMaps_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline MapsToMetaMaps_array_port& operator=(MapsToMetaMaps_array_port&& from) noexcept {
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
  static const MapsToMetaMaps_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MapsToMetaMaps_array_port* internal_default_instance() {
    return reinterpret_cast<const MapsToMetaMaps_array_port*>(
               &_MapsToMetaMaps_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(MapsToMetaMaps_array_port& a, MapsToMetaMaps_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(MapsToMetaMaps_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MapsToMetaMaps_array_port* New() const final {
    return CreateMaybeMessage<MapsToMetaMaps_array_port>(nullptr);
  }

  MapsToMetaMaps_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MapsToMetaMaps_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MapsToMetaMaps_array_port& from);
  void MergeFrom(const MapsToMetaMaps_array_port& from);
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
  void InternalSwap(MapsToMetaMaps_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2fmaps_5fto_5fmeta_5fmaps_2eproto);
    return ::descriptor_table_mf_5fmempark_2fmaps_5fto_5fmeta_5fmaps_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 300,
  };
  // repeated .pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps data = 300;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps >*
      mutable_data();
  private:
  const ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps& _internal_data(int index) const;
  ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps* _internal_add_data();
  public:
  const ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps& data(int index) const;
  ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps > data_;
  friend struct ::TableStruct_mf_5fmempark_2fmaps_5fto_5fmeta_5fmaps_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MapsToMetaMaps

// optional uint32 uiVersionNumber = 2124;
inline bool MapsToMetaMaps::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool MapsToMetaMaps::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void MapsToMetaMaps::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MapsToMetaMaps::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MapsToMetaMaps::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void MapsToMetaMaps::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  uiversionnumber_ = value;
}
inline void MapsToMetaMaps::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool MapsToMetaMaps::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool MapsToMetaMaps::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& MapsToMetaMaps::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& MapsToMetaMaps::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* MapsToMetaMaps::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* MapsToMetaMaps::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* MapsToMetaMaps::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void MapsToMetaMaps::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps.sSigHeader)
}

// optional uint32 numValidMaps = 853;
inline bool MapsToMetaMaps::_internal_has_numvalidmaps() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool MapsToMetaMaps::has_numvalidmaps() const {
  return _internal_has_numvalidmaps();
}
inline void MapsToMetaMaps::clear_numvalidmaps() {
  numvalidmaps_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MapsToMetaMaps::_internal_numvalidmaps() const {
  return numvalidmaps_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MapsToMetaMaps::numvalidmaps() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps.numValidMaps)
  return _internal_numvalidmaps();
}
inline void MapsToMetaMaps::_internal_set_numvalidmaps(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  numvalidmaps_ = value;
}
inline void MapsToMetaMaps::set_numvalidmaps(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_numvalidmaps(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps.numValidMaps)
}

// repeated .pb.mf_mempark.map_idto_meta_map.MapIDToMetaMap mapsMetaMaps = 1338;
inline int MapsToMetaMaps::_internal_mapsmetamaps_size() const {
  return mapsmetamaps_.size();
}
inline int MapsToMetaMaps::mapsmetamaps_size() const {
  return _internal_mapsmetamaps_size();
}
inline ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap* MapsToMetaMaps::mutable_mapsmetamaps(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps.mapsMetaMaps)
  return mapsmetamaps_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap >*
MapsToMetaMaps::mutable_mapsmetamaps() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps.mapsMetaMaps)
  return &mapsmetamaps_;
}
inline const ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap& MapsToMetaMaps::_internal_mapsmetamaps(int index) const {
  return mapsmetamaps_.Get(index);
}
inline const ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap& MapsToMetaMaps::mapsmetamaps(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps.mapsMetaMaps)
  return _internal_mapsmetamaps(index);
}
inline ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap* MapsToMetaMaps::_internal_add_mapsmetamaps() {
  return mapsmetamaps_.Add();
}
inline ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap* MapsToMetaMaps::add_mapsmetamaps() {
  // @@protoc_insertion_point(field_add:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps.mapsMetaMaps)
  return _internal_add_mapsmetamaps();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::map_idto_meta_map::MapIDToMetaMap >&
MapsToMetaMaps::mapsmetamaps() const {
  // @@protoc_insertion_point(field_list:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps.mapsMetaMaps)
  return mapsmetamaps_;
}

// -------------------------------------------------------------------

// MapsToMetaMaps_array_port

// repeated .pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps data = 300;
inline int MapsToMetaMaps_array_port::_internal_data_size() const {
  return data_.size();
}
inline int MapsToMetaMaps_array_port::data_size() const {
  return _internal_data_size();
}
inline void MapsToMetaMaps_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps* MapsToMetaMaps_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps >*
MapsToMetaMaps_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps_array_port.data)
  return &data_;
}
inline const ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps& MapsToMetaMaps_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps& MapsToMetaMaps_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps* MapsToMetaMaps_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps* MapsToMetaMaps_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::maps_to_meta_maps::MapsToMetaMaps >&
MapsToMetaMaps_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_mempark.maps_to_meta_maps.MapsToMetaMaps_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace maps_to_meta_maps
}  // namespace mf_mempark
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmaps_5fto_5fmeta_5fmaps_2eproto
