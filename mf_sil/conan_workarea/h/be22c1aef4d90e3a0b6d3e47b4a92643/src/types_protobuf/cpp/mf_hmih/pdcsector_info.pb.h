// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_hmih/pdcsector_info.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fpdcsector_5finfo_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fpdcsector_5finfo_2eproto

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
#include "pdcp/criticality_level.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fhmih_2fpdcsector_5finfo_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fhmih_2fpdcsector_5finfo_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fhmih_2fpdcsector_5finfo_2eproto;
namespace pb {
namespace mf_hmih {
namespace pdcsector_info {
class PDCSectorInfo;
class PDCSectorInfoDefaultTypeInternal;
extern PDCSectorInfoDefaultTypeInternal _PDCSectorInfo_default_instance_;
class PDCSectorInfo_array_port;
class PDCSectorInfo_array_portDefaultTypeInternal;
extern PDCSectorInfo_array_portDefaultTypeInternal _PDCSectorInfo_array_port_default_instance_;
}  // namespace pdcsector_info
}  // namespace mf_hmih
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* Arena::CreateMaybeMessage<::pb::mf_hmih::pdcsector_info::PDCSectorInfo>(Arena*);
template<> ::pb::mf_hmih::pdcsector_info::PDCSectorInfo_array_port* Arena::CreateMaybeMessage<::pb::mf_hmih::pdcsector_info::PDCSectorInfo_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_hmih {
namespace pdcsector_info {

// ===================================================================

class PDCSectorInfo :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.pdcsector_info.PDCSectorInfo) */ {
 public:
  PDCSectorInfo();
  virtual ~PDCSectorInfo();

  PDCSectorInfo(const PDCSectorInfo& from);
  PDCSectorInfo(PDCSectorInfo&& from) noexcept
    : PDCSectorInfo() {
    *this = ::std::move(from);
  }

  inline PDCSectorInfo& operator=(const PDCSectorInfo& from) {
    CopyFrom(from);
    return *this;
  }
  inline PDCSectorInfo& operator=(PDCSectorInfo&& from) noexcept {
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
  static const PDCSectorInfo& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PDCSectorInfo* internal_default_instance() {
    return reinterpret_cast<const PDCSectorInfo*>(
               &_PDCSectorInfo_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PDCSectorInfo& a, PDCSectorInfo& b) {
    a.Swap(&b);
  }
  inline void Swap(PDCSectorInfo* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PDCSectorInfo* New() const final {
    return CreateMaybeMessage<PDCSectorInfo>(nullptr);
  }

  PDCSectorInfo* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PDCSectorInfo>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PDCSectorInfo& from);
  void MergeFrom(const PDCSectorInfo& from);
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
  void InternalSwap(PDCSectorInfo* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.pdcsector_info.PDCSectorInfo";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fpdcsector_5finfo_2eproto);
    return ::descriptor_table_mf_5fhmih_2fpdcsector_5finfo_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSectorIDNuFieldNumber = 3810,
    kSliceNuFieldNumber = 883,
    kSmallestDistanceMFieldNumber = 899,
    kIntersectsDrvTubeNuFieldNumber = 1308,
    kScannedNuFieldNumber = 1046,
    kCriticalityLevelNuFieldNumber = 1238,
  };
  // optional uint32 sectorID_nu = 3810;
  bool has_sectorid_nu() const;
  private:
  bool _internal_has_sectorid_nu() const;
  public:
  void clear_sectorid_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 sectorid_nu() const;
  void set_sectorid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_sectorid_nu() const;
  void _internal_set_sectorid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 slice_nu = 883;
  bool has_slice_nu() const;
  private:
  bool _internal_has_slice_nu() const;
  public:
  void clear_slice_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 slice_nu() const;
  void set_slice_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_slice_nu() const;
  void _internal_set_slice_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional float smallestDistance_m = 899;
  bool has_smallestdistance_m() const;
  private:
  bool _internal_has_smallestdistance_m() const;
  public:
  void clear_smallestdistance_m();
  float smallestdistance_m() const;
  void set_smallestdistance_m(float value);
  private:
  float _internal_smallestdistance_m() const;
  void _internal_set_smallestdistance_m(float value);
  public:

  // optional bool intersectsDrvTube_nu = 1308;
  bool has_intersectsdrvtube_nu() const;
  private:
  bool _internal_has_intersectsdrvtube_nu() const;
  public:
  void clear_intersectsdrvtube_nu();
  bool intersectsdrvtube_nu() const;
  void set_intersectsdrvtube_nu(bool value);
  private:
  bool _internal_intersectsdrvtube_nu() const;
  void _internal_set_intersectsdrvtube_nu(bool value);
  public:

  // optional bool scanned_nu = 1046;
  bool has_scanned_nu() const;
  private:
  bool _internal_has_scanned_nu() const;
  public:
  void clear_scanned_nu();
  bool scanned_nu() const;
  void set_scanned_nu(bool value);
  private:
  bool _internal_scanned_nu() const;
  void _internal_set_scanned_nu(bool value);
  public:

  // optional .pb.pdcp.criticality_level.CriticalityLevel criticalityLevel_nu = 1238;
  bool has_criticalitylevel_nu() const;
  private:
  bool _internal_has_criticalitylevel_nu() const;
  public:
  void clear_criticalitylevel_nu();
  ::pb::pdcp::criticality_level::CriticalityLevel criticalitylevel_nu() const;
  void set_criticalitylevel_nu(::pb::pdcp::criticality_level::CriticalityLevel value);
  private:
  ::pb::pdcp::criticality_level::CriticalityLevel _internal_criticalitylevel_nu() const;
  void _internal_set_criticalitylevel_nu(::pb::pdcp::criticality_level::CriticalityLevel value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.pdcsector_info.PDCSectorInfo)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 sectorid_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 slice_nu_;
  float smallestdistance_m_;
  bool intersectsdrvtube_nu_;
  bool scanned_nu_;
  int criticalitylevel_nu_;
  friend struct ::TableStruct_mf_5fhmih_2fpdcsector_5finfo_2eproto;
};
// -------------------------------------------------------------------

class PDCSectorInfo_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.pdcsector_info.PDCSectorInfo_array_port) */ {
 public:
  PDCSectorInfo_array_port();
  virtual ~PDCSectorInfo_array_port();

  PDCSectorInfo_array_port(const PDCSectorInfo_array_port& from);
  PDCSectorInfo_array_port(PDCSectorInfo_array_port&& from) noexcept
    : PDCSectorInfo_array_port() {
    *this = ::std::move(from);
  }

  inline PDCSectorInfo_array_port& operator=(const PDCSectorInfo_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline PDCSectorInfo_array_port& operator=(PDCSectorInfo_array_port&& from) noexcept {
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
  static const PDCSectorInfo_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PDCSectorInfo_array_port* internal_default_instance() {
    return reinterpret_cast<const PDCSectorInfo_array_port*>(
               &_PDCSectorInfo_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(PDCSectorInfo_array_port& a, PDCSectorInfo_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(PDCSectorInfo_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PDCSectorInfo_array_port* New() const final {
    return CreateMaybeMessage<PDCSectorInfo_array_port>(nullptr);
  }

  PDCSectorInfo_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PDCSectorInfo_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PDCSectorInfo_array_port& from);
  void MergeFrom(const PDCSectorInfo_array_port& from);
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
  void InternalSwap(PDCSectorInfo_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.pdcsector_info.PDCSectorInfo_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fpdcsector_5finfo_2eproto);
    return ::descriptor_table_mf_5fhmih_2fpdcsector_5finfo_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3828,
  };
  // repeated .pb.mf_hmih.pdcsector_info.PDCSectorInfo data = 3828;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >*
      mutable_data();
  private:
  const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& _internal_data(int index) const;
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* _internal_add_data();
  public:
  const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& data(int index) const;
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.pdcsector_info.PDCSectorInfo_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo > data_;
  friend struct ::TableStruct_mf_5fhmih_2fpdcsector_5finfo_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PDCSectorInfo

// optional float smallestDistance_m = 899;
inline bool PDCSectorInfo::_internal_has_smallestdistance_m() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool PDCSectorInfo::has_smallestdistance_m() const {
  return _internal_has_smallestdistance_m();
}
inline void PDCSectorInfo::clear_smallestdistance_m() {
  smallestdistance_m_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float PDCSectorInfo::_internal_smallestdistance_m() const {
  return smallestdistance_m_;
}
inline float PDCSectorInfo::smallestdistance_m() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsector_info.PDCSectorInfo.smallestDistance_m)
  return _internal_smallestdistance_m();
}
inline void PDCSectorInfo::_internal_set_smallestdistance_m(float value) {
  _has_bits_[0] |= 0x00000004u;
  smallestdistance_m_ = value;
}
inline void PDCSectorInfo::set_smallestdistance_m(float value) {
  _internal_set_smallestdistance_m(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.pdcsector_info.PDCSectorInfo.smallestDistance_m)
}

// optional .pb.pdcp.criticality_level.CriticalityLevel criticalityLevel_nu = 1238;
inline bool PDCSectorInfo::_internal_has_criticalitylevel_nu() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool PDCSectorInfo::has_criticalitylevel_nu() const {
  return _internal_has_criticalitylevel_nu();
}
inline void PDCSectorInfo::clear_criticalitylevel_nu() {
  criticalitylevel_nu_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::pb::pdcp::criticality_level::CriticalityLevel PDCSectorInfo::_internal_criticalitylevel_nu() const {
  return static_cast< ::pb::pdcp::criticality_level::CriticalityLevel >(criticalitylevel_nu_);
}
inline ::pb::pdcp::criticality_level::CriticalityLevel PDCSectorInfo::criticalitylevel_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsector_info.PDCSectorInfo.criticalityLevel_nu)
  return _internal_criticalitylevel_nu();
}
inline void PDCSectorInfo::_internal_set_criticalitylevel_nu(::pb::pdcp::criticality_level::CriticalityLevel value) {
  assert(::pb::pdcp::criticality_level::CriticalityLevel_IsValid(value));
  _has_bits_[0] |= 0x00000020u;
  criticalitylevel_nu_ = value;
}
inline void PDCSectorInfo::set_criticalitylevel_nu(::pb::pdcp::criticality_level::CriticalityLevel value) {
  _internal_set_criticalitylevel_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.pdcsector_info.PDCSectorInfo.criticalityLevel_nu)
}

// optional uint32 slice_nu = 883;
inline bool PDCSectorInfo::_internal_has_slice_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool PDCSectorInfo::has_slice_nu() const {
  return _internal_has_slice_nu();
}
inline void PDCSectorInfo::clear_slice_nu() {
  slice_nu_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PDCSectorInfo::_internal_slice_nu() const {
  return slice_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PDCSectorInfo::slice_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsector_info.PDCSectorInfo.slice_nu)
  return _internal_slice_nu();
}
inline void PDCSectorInfo::_internal_set_slice_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  slice_nu_ = value;
}
inline void PDCSectorInfo::set_slice_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_slice_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.pdcsector_info.PDCSectorInfo.slice_nu)
}

// optional uint32 sectorID_nu = 3810;
inline bool PDCSectorInfo::_internal_has_sectorid_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool PDCSectorInfo::has_sectorid_nu() const {
  return _internal_has_sectorid_nu();
}
inline void PDCSectorInfo::clear_sectorid_nu() {
  sectorid_nu_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PDCSectorInfo::_internal_sectorid_nu() const {
  return sectorid_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PDCSectorInfo::sectorid_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsector_info.PDCSectorInfo.sectorID_nu)
  return _internal_sectorid_nu();
}
inline void PDCSectorInfo::_internal_set_sectorid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  sectorid_nu_ = value;
}
inline void PDCSectorInfo::set_sectorid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_sectorid_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.pdcsector_info.PDCSectorInfo.sectorID_nu)
}

// optional bool intersectsDrvTube_nu = 1308;
inline bool PDCSectorInfo::_internal_has_intersectsdrvtube_nu() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool PDCSectorInfo::has_intersectsdrvtube_nu() const {
  return _internal_has_intersectsdrvtube_nu();
}
inline void PDCSectorInfo::clear_intersectsdrvtube_nu() {
  intersectsdrvtube_nu_ = false;
  _has_bits_[0] &= ~0x00000008u;
}
inline bool PDCSectorInfo::_internal_intersectsdrvtube_nu() const {
  return intersectsdrvtube_nu_;
}
inline bool PDCSectorInfo::intersectsdrvtube_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsector_info.PDCSectorInfo.intersectsDrvTube_nu)
  return _internal_intersectsdrvtube_nu();
}
inline void PDCSectorInfo::_internal_set_intersectsdrvtube_nu(bool value) {
  _has_bits_[0] |= 0x00000008u;
  intersectsdrvtube_nu_ = value;
}
inline void PDCSectorInfo::set_intersectsdrvtube_nu(bool value) {
  _internal_set_intersectsdrvtube_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.pdcsector_info.PDCSectorInfo.intersectsDrvTube_nu)
}

// optional bool scanned_nu = 1046;
inline bool PDCSectorInfo::_internal_has_scanned_nu() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool PDCSectorInfo::has_scanned_nu() const {
  return _internal_has_scanned_nu();
}
inline void PDCSectorInfo::clear_scanned_nu() {
  scanned_nu_ = false;
  _has_bits_[0] &= ~0x00000010u;
}
inline bool PDCSectorInfo::_internal_scanned_nu() const {
  return scanned_nu_;
}
inline bool PDCSectorInfo::scanned_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsector_info.PDCSectorInfo.scanned_nu)
  return _internal_scanned_nu();
}
inline void PDCSectorInfo::_internal_set_scanned_nu(bool value) {
  _has_bits_[0] |= 0x00000010u;
  scanned_nu_ = value;
}
inline void PDCSectorInfo::set_scanned_nu(bool value) {
  _internal_set_scanned_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.pdcsector_info.PDCSectorInfo.scanned_nu)
}

// -------------------------------------------------------------------

// PDCSectorInfo_array_port

// repeated .pb.mf_hmih.pdcsector_info.PDCSectorInfo data = 3828;
inline int PDCSectorInfo_array_port::_internal_data_size() const {
  return data_.size();
}
inline int PDCSectorInfo_array_port::data_size() const {
  return _internal_data_size();
}
inline void PDCSectorInfo_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectorInfo_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.pdcsector_info.PDCSectorInfo_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >*
PDCSectorInfo_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.pdcsector_info.PDCSectorInfo_array_port.data)
  return &data_;
}
inline const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& PDCSectorInfo_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& PDCSectorInfo_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsector_info.PDCSectorInfo_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectorInfo_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectorInfo_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_hmih.pdcsector_info.PDCSectorInfo_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >&
PDCSectorInfo_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.pdcsector_info.PDCSectorInfo_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace pdcsector_info
}  // namespace mf_hmih
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fpdcsector_5finfo_2eproto
