// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_mempark/mf_mem_park_consts.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmf_5fmem_5fpark_5fconsts_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmf_5fmem_5fpark_5fconsts_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_mf_5fmempark_2fmf_5fmem_5fpark_5fconsts_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fmempark_2fmf_5fmem_5fpark_5fconsts_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmempark_2fmf_5fmem_5fpark_5fconsts_2eproto;
namespace pb {
namespace mf_mempark {
namespace mf_mem_park_consts {
class MF_MemPark_Consts;
class MF_MemPark_ConstsDefaultTypeInternal;
extern MF_MemPark_ConstsDefaultTypeInternal _MF_MemPark_Consts_default_instance_;
class MF_MemPark_Consts_array_port;
class MF_MemPark_Consts_array_portDefaultTypeInternal;
extern MF_MemPark_Consts_array_portDefaultTypeInternal _MF_MemPark_Consts_array_port_default_instance_;
}  // namespace mf_mem_park_consts
}  // namespace mf_mempark
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts* Arena::CreateMaybeMessage<::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts>(Arena*);
template<> ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts_array_port* Arena::CreateMaybeMessage<::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_mempark {
namespace mf_mem_park_consts {

// ===================================================================

class MF_MemPark_Consts :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts) */ {
 public:
  MF_MemPark_Consts();
  virtual ~MF_MemPark_Consts();

  MF_MemPark_Consts(const MF_MemPark_Consts& from);
  MF_MemPark_Consts(MF_MemPark_Consts&& from) noexcept
    : MF_MemPark_Consts() {
    *this = ::std::move(from);
  }

  inline MF_MemPark_Consts& operator=(const MF_MemPark_Consts& from) {
    CopyFrom(from);
    return *this;
  }
  inline MF_MemPark_Consts& operator=(MF_MemPark_Consts&& from) noexcept {
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
  static const MF_MemPark_Consts& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MF_MemPark_Consts* internal_default_instance() {
    return reinterpret_cast<const MF_MemPark_Consts*>(
               &_MF_MemPark_Consts_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MF_MemPark_Consts& a, MF_MemPark_Consts& b) {
    a.Swap(&b);
  }
  inline void Swap(MF_MemPark_Consts* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MF_MemPark_Consts* New() const final {
    return CreateMaybeMessage<MF_MemPark_Consts>(nullptr);
  }

  MF_MemPark_Consts* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MF_MemPark_Consts>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MF_MemPark_Consts& from);
  void MergeFrom(const MF_MemPark_Consts& from);
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
  void InternalSwap(MF_MemPark_Consts* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2fmf_5fmem_5fpark_5fconsts_2eproto);
    return ::descriptor_table_mf_5fmempark_2fmf_5fmem_5fpark_5fconsts_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kMAXNUMTRAJECTORYNUFieldNumber = 952,
    kMAXSIZEOFPOINTSFieldNumber = 1276,
    kMAXNUMPARKINGSLOTSNUFieldNumber = 1630,
    kINVALIDMAPIDFieldNumber = 2190,
    kMAXSLOTSPERMETAMAPNUFieldNumber = 2287,
    kMAXNUMMAPSSTOREDNUFieldNumber = 2452,
    kMAXTRAJECTORIESPERMETAMAPNUFieldNumber = 2668,
    kMAXNUMREQUESTABLERELOCALIZATIONSLOTSFieldNumber = 3021,
  };
  // optional uint32 MAX_NUM_TRAJECTORY_NU = 952;
  bool has_max_num_trajectory_nu() const;
  private:
  bool _internal_has_max_num_trajectory_nu() const;
  public:
  void clear_max_num_trajectory_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 max_num_trajectory_nu() const;
  void set_max_num_trajectory_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_max_num_trajectory_nu() const;
  void _internal_set_max_num_trajectory_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 MAX_SIZE_OF_POINTS = 1276;
  bool has_max_size_of_points() const;
  private:
  bool _internal_has_max_size_of_points() const;
  public:
  void clear_max_size_of_points();
  ::PROTOBUF_NAMESPACE_ID::uint32 max_size_of_points() const;
  void set_max_size_of_points(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_max_size_of_points() const;
  void _internal_set_max_size_of_points(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 MAX_NUM_PARKING_SLOTS_NU = 1630;
  bool has_max_num_parking_slots_nu() const;
  private:
  bool _internal_has_max_num_parking_slots_nu() const;
  public:
  void clear_max_num_parking_slots_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 max_num_parking_slots_nu() const;
  void set_max_num_parking_slots_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_max_num_parking_slots_nu() const;
  void _internal_set_max_num_parking_slots_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 INVALID_MAP_ID = 2190;
  bool has_invalid_map_id() const;
  private:
  bool _internal_has_invalid_map_id() const;
  public:
  void clear_invalid_map_id();
  ::PROTOBUF_NAMESPACE_ID::uint32 invalid_map_id() const;
  void set_invalid_map_id(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_invalid_map_id() const;
  void _internal_set_invalid_map_id(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 MAX_SLOTS_PER_METAMAP_NU = 2287;
  bool has_max_slots_per_metamap_nu() const;
  private:
  bool _internal_has_max_slots_per_metamap_nu() const;
  public:
  void clear_max_slots_per_metamap_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 max_slots_per_metamap_nu() const;
  void set_max_slots_per_metamap_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_max_slots_per_metamap_nu() const;
  void _internal_set_max_slots_per_metamap_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 MAX_NUM_MAPS_STORED_NU = 2452;
  bool has_max_num_maps_stored_nu() const;
  private:
  bool _internal_has_max_num_maps_stored_nu() const;
  public:
  void clear_max_num_maps_stored_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 max_num_maps_stored_nu() const;
  void set_max_num_maps_stored_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_max_num_maps_stored_nu() const;
  void _internal_set_max_num_maps_stored_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 MAX_TRAJECTORIES_PER_METAMAP_NU = 2668;
  bool has_max_trajectories_per_metamap_nu() const;
  private:
  bool _internal_has_max_trajectories_per_metamap_nu() const;
  public:
  void clear_max_trajectories_per_metamap_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 max_trajectories_per_metamap_nu() const;
  void set_max_trajectories_per_metamap_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_max_trajectories_per_metamap_nu() const;
  void _internal_set_max_trajectories_per_metamap_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 MAX_NUM_REQUESTABLE_RELOCALIZATION_SLOTS = 3021;
  bool has_max_num_requestable_relocalization_slots() const;
  private:
  bool _internal_has_max_num_requestable_relocalization_slots() const;
  public:
  void clear_max_num_requestable_relocalization_slots();
  ::PROTOBUF_NAMESPACE_ID::uint32 max_num_requestable_relocalization_slots() const;
  void set_max_num_requestable_relocalization_slots(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_max_num_requestable_relocalization_slots() const;
  void _internal_set_max_num_requestable_relocalization_slots(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 max_num_trajectory_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 max_size_of_points_;
  ::PROTOBUF_NAMESPACE_ID::uint32 max_num_parking_slots_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 invalid_map_id_;
  ::PROTOBUF_NAMESPACE_ID::uint32 max_slots_per_metamap_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 max_num_maps_stored_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 max_trajectories_per_metamap_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 max_num_requestable_relocalization_slots_;
  friend struct ::TableStruct_mf_5fmempark_2fmf_5fmem_5fpark_5fconsts_2eproto;
};
// -------------------------------------------------------------------

class MF_MemPark_Consts_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts_array_port) */ {
 public:
  MF_MemPark_Consts_array_port();
  virtual ~MF_MemPark_Consts_array_port();

  MF_MemPark_Consts_array_port(const MF_MemPark_Consts_array_port& from);
  MF_MemPark_Consts_array_port(MF_MemPark_Consts_array_port&& from) noexcept
    : MF_MemPark_Consts_array_port() {
    *this = ::std::move(from);
  }

  inline MF_MemPark_Consts_array_port& operator=(const MF_MemPark_Consts_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline MF_MemPark_Consts_array_port& operator=(MF_MemPark_Consts_array_port&& from) noexcept {
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
  static const MF_MemPark_Consts_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MF_MemPark_Consts_array_port* internal_default_instance() {
    return reinterpret_cast<const MF_MemPark_Consts_array_port*>(
               &_MF_MemPark_Consts_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(MF_MemPark_Consts_array_port& a, MF_MemPark_Consts_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(MF_MemPark_Consts_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MF_MemPark_Consts_array_port* New() const final {
    return CreateMaybeMessage<MF_MemPark_Consts_array_port>(nullptr);
  }

  MF_MemPark_Consts_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MF_MemPark_Consts_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MF_MemPark_Consts_array_port& from);
  void MergeFrom(const MF_MemPark_Consts_array_port& from);
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
  void InternalSwap(MF_MemPark_Consts_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2fmf_5fmem_5fpark_5fconsts_2eproto);
    return ::descriptor_table_mf_5fmempark_2fmf_5fmem_5fpark_5fconsts_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 434,
  };
  // repeated .pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts data = 434;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts >*
      mutable_data();
  private:
  const ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts& _internal_data(int index) const;
  ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts* _internal_add_data();
  public:
  const ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts& data(int index) const;
  ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts > data_;
  friend struct ::TableStruct_mf_5fmempark_2fmf_5fmem_5fpark_5fconsts_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MF_MemPark_Consts

// optional uint32 MAX_NUM_PARKING_SLOTS_NU = 1630;
inline bool MF_MemPark_Consts::_internal_has_max_num_parking_slots_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool MF_MemPark_Consts::has_max_num_parking_slots_nu() const {
  return _internal_has_max_num_parking_slots_nu();
}
inline void MF_MemPark_Consts::clear_max_num_parking_slots_nu() {
  max_num_parking_slots_nu_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::_internal_max_num_parking_slots_nu() const {
  return max_num_parking_slots_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::max_num_parking_slots_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_NUM_PARKING_SLOTS_NU)
  return _internal_max_num_parking_slots_nu();
}
inline void MF_MemPark_Consts::_internal_set_max_num_parking_slots_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  max_num_parking_slots_nu_ = value;
}
inline void MF_MemPark_Consts::set_max_num_parking_slots_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_max_num_parking_slots_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_NUM_PARKING_SLOTS_NU)
}

// optional uint32 MAX_NUM_TRAJECTORY_NU = 952;
inline bool MF_MemPark_Consts::_internal_has_max_num_trajectory_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool MF_MemPark_Consts::has_max_num_trajectory_nu() const {
  return _internal_has_max_num_trajectory_nu();
}
inline void MF_MemPark_Consts::clear_max_num_trajectory_nu() {
  max_num_trajectory_nu_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::_internal_max_num_trajectory_nu() const {
  return max_num_trajectory_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::max_num_trajectory_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_NUM_TRAJECTORY_NU)
  return _internal_max_num_trajectory_nu();
}
inline void MF_MemPark_Consts::_internal_set_max_num_trajectory_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  max_num_trajectory_nu_ = value;
}
inline void MF_MemPark_Consts::set_max_num_trajectory_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_max_num_trajectory_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_NUM_TRAJECTORY_NU)
}

// optional uint32 MAX_NUM_REQUESTABLE_RELOCALIZATION_SLOTS = 3021;
inline bool MF_MemPark_Consts::_internal_has_max_num_requestable_relocalization_slots() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool MF_MemPark_Consts::has_max_num_requestable_relocalization_slots() const {
  return _internal_has_max_num_requestable_relocalization_slots();
}
inline void MF_MemPark_Consts::clear_max_num_requestable_relocalization_slots() {
  max_num_requestable_relocalization_slots_ = 0u;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::_internal_max_num_requestable_relocalization_slots() const {
  return max_num_requestable_relocalization_slots_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::max_num_requestable_relocalization_slots() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_NUM_REQUESTABLE_RELOCALIZATION_SLOTS)
  return _internal_max_num_requestable_relocalization_slots();
}
inline void MF_MemPark_Consts::_internal_set_max_num_requestable_relocalization_slots(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000080u;
  max_num_requestable_relocalization_slots_ = value;
}
inline void MF_MemPark_Consts::set_max_num_requestable_relocalization_slots(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_max_num_requestable_relocalization_slots(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_NUM_REQUESTABLE_RELOCALIZATION_SLOTS)
}

// optional uint32 MAX_SIZE_OF_POINTS = 1276;
inline bool MF_MemPark_Consts::_internal_has_max_size_of_points() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool MF_MemPark_Consts::has_max_size_of_points() const {
  return _internal_has_max_size_of_points();
}
inline void MF_MemPark_Consts::clear_max_size_of_points() {
  max_size_of_points_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::_internal_max_size_of_points() const {
  return max_size_of_points_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::max_size_of_points() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_SIZE_OF_POINTS)
  return _internal_max_size_of_points();
}
inline void MF_MemPark_Consts::_internal_set_max_size_of_points(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  max_size_of_points_ = value;
}
inline void MF_MemPark_Consts::set_max_size_of_points(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_max_size_of_points(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_SIZE_OF_POINTS)
}

// optional uint32 MAX_SLOTS_PER_METAMAP_NU = 2287;
inline bool MF_MemPark_Consts::_internal_has_max_slots_per_metamap_nu() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool MF_MemPark_Consts::has_max_slots_per_metamap_nu() const {
  return _internal_has_max_slots_per_metamap_nu();
}
inline void MF_MemPark_Consts::clear_max_slots_per_metamap_nu() {
  max_slots_per_metamap_nu_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::_internal_max_slots_per_metamap_nu() const {
  return max_slots_per_metamap_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::max_slots_per_metamap_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_SLOTS_PER_METAMAP_NU)
  return _internal_max_slots_per_metamap_nu();
}
inline void MF_MemPark_Consts::_internal_set_max_slots_per_metamap_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  max_slots_per_metamap_nu_ = value;
}
inline void MF_MemPark_Consts::set_max_slots_per_metamap_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_max_slots_per_metamap_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_SLOTS_PER_METAMAP_NU)
}

// optional uint32 MAX_TRAJECTORIES_PER_METAMAP_NU = 2668;
inline bool MF_MemPark_Consts::_internal_has_max_trajectories_per_metamap_nu() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool MF_MemPark_Consts::has_max_trajectories_per_metamap_nu() const {
  return _internal_has_max_trajectories_per_metamap_nu();
}
inline void MF_MemPark_Consts::clear_max_trajectories_per_metamap_nu() {
  max_trajectories_per_metamap_nu_ = 0u;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::_internal_max_trajectories_per_metamap_nu() const {
  return max_trajectories_per_metamap_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::max_trajectories_per_metamap_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_TRAJECTORIES_PER_METAMAP_NU)
  return _internal_max_trajectories_per_metamap_nu();
}
inline void MF_MemPark_Consts::_internal_set_max_trajectories_per_metamap_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000040u;
  max_trajectories_per_metamap_nu_ = value;
}
inline void MF_MemPark_Consts::set_max_trajectories_per_metamap_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_max_trajectories_per_metamap_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_TRAJECTORIES_PER_METAMAP_NU)
}

// optional uint32 MAX_NUM_MAPS_STORED_NU = 2452;
inline bool MF_MemPark_Consts::_internal_has_max_num_maps_stored_nu() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool MF_MemPark_Consts::has_max_num_maps_stored_nu() const {
  return _internal_has_max_num_maps_stored_nu();
}
inline void MF_MemPark_Consts::clear_max_num_maps_stored_nu() {
  max_num_maps_stored_nu_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::_internal_max_num_maps_stored_nu() const {
  return max_num_maps_stored_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::max_num_maps_stored_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_NUM_MAPS_STORED_NU)
  return _internal_max_num_maps_stored_nu();
}
inline void MF_MemPark_Consts::_internal_set_max_num_maps_stored_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  max_num_maps_stored_nu_ = value;
}
inline void MF_MemPark_Consts::set_max_num_maps_stored_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_max_num_maps_stored_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.MAX_NUM_MAPS_STORED_NU)
}

// optional uint32 INVALID_MAP_ID = 2190;
inline bool MF_MemPark_Consts::_internal_has_invalid_map_id() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool MF_MemPark_Consts::has_invalid_map_id() const {
  return _internal_has_invalid_map_id();
}
inline void MF_MemPark_Consts::clear_invalid_map_id() {
  invalid_map_id_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::_internal_invalid_map_id() const {
  return invalid_map_id_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MemPark_Consts::invalid_map_id() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.INVALID_MAP_ID)
  return _internal_invalid_map_id();
}
inline void MF_MemPark_Consts::_internal_set_invalid_map_id(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  invalid_map_id_ = value;
}
inline void MF_MemPark_Consts::set_invalid_map_id(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_invalid_map_id(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts.INVALID_MAP_ID)
}

// -------------------------------------------------------------------

// MF_MemPark_Consts_array_port

// repeated .pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts data = 434;
inline int MF_MemPark_Consts_array_port::_internal_data_size() const {
  return data_.size();
}
inline int MF_MemPark_Consts_array_port::data_size() const {
  return _internal_data_size();
}
inline void MF_MemPark_Consts_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts* MF_MemPark_Consts_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts >*
MF_MemPark_Consts_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts_array_port.data)
  return &data_;
}
inline const ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts& MF_MemPark_Consts_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts& MF_MemPark_Consts_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts* MF_MemPark_Consts_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts* MF_MemPark_Consts_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::mf_mem_park_consts::MF_MemPark_Consts >&
MF_MemPark_Consts_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_mempark.mf_mem_park_consts.MF_MemPark_Consts_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace mf_mem_park_consts
}  // namespace mf_mempark
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmf_5fmem_5fpark_5fconsts_2eproto