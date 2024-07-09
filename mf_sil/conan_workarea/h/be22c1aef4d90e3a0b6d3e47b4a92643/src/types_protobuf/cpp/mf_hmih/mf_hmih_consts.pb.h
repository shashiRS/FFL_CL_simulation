// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_hmih/mf_hmih_consts.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fmf_5fhmih_5fconsts_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fmf_5fhmih_5fconsts_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_mf_5fhmih_2fmf_5fhmih_5fconsts_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fhmih_2fmf_5fhmih_5fconsts_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fhmih_2fmf_5fhmih_5fconsts_2eproto;
namespace pb {
namespace mf_hmih {
namespace mf_hmih_consts {
class MF_HMIH_Consts;
class MF_HMIH_ConstsDefaultTypeInternal;
extern MF_HMIH_ConstsDefaultTypeInternal _MF_HMIH_Consts_default_instance_;
class MF_HMIH_Consts_array_port;
class MF_HMIH_Consts_array_portDefaultTypeInternal;
extern MF_HMIH_Consts_array_portDefaultTypeInternal _MF_HMIH_Consts_array_port_default_instance_;
}  // namespace mf_hmih_consts
}  // namespace mf_hmih
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts* Arena::CreateMaybeMessage<::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts>(Arena*);
template<> ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts_array_port* Arena::CreateMaybeMessage<::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_hmih {
namespace mf_hmih_consts {

// ===================================================================

class MF_HMIH_Consts :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts) */ {
 public:
  MF_HMIH_Consts();
  virtual ~MF_HMIH_Consts();

  MF_HMIH_Consts(const MF_HMIH_Consts& from);
  MF_HMIH_Consts(MF_HMIH_Consts&& from) noexcept
    : MF_HMIH_Consts() {
    *this = ::std::move(from);
  }

  inline MF_HMIH_Consts& operator=(const MF_HMIH_Consts& from) {
    CopyFrom(from);
    return *this;
  }
  inline MF_HMIH_Consts& operator=(MF_HMIH_Consts&& from) noexcept {
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
  static const MF_HMIH_Consts& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MF_HMIH_Consts* internal_default_instance() {
    return reinterpret_cast<const MF_HMIH_Consts*>(
               &_MF_HMIH_Consts_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MF_HMIH_Consts& a, MF_HMIH_Consts& b) {
    a.Swap(&b);
  }
  inline void Swap(MF_HMIH_Consts* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MF_HMIH_Consts* New() const final {
    return CreateMaybeMessage<MF_HMIH_Consts>(nullptr);
  }

  MF_HMIH_Consts* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MF_HMIH_Consts>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MF_HMIH_Consts& from);
  void MergeFrom(const MF_HMIH_Consts& from);
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
  void InternalSwap(MF_HMIH_Consts* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fmf_5fhmih_5fconsts_2eproto);
    return ::descriptor_table_mf_5fhmih_2fmf_5fhmih_5fconsts_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kAPHMAXNUMPLACEHOLDERNUFieldNumber = 1626,
    kAPHMAXNUMPARKINGPOSESNUFieldNumber = 2410,
    kAPHMAXNUMVISIBILITYTAGSNUFieldNumber = 2634,
    kMAXNUMSECTORSPERSIDEHMIHFieldNumber = 3725,
    kNUMMTSDEBUGFREESPACEHMIHFieldNumber = 3742,
    kAPHMAXNUMSLOTSSIDENUFieldNumber = 3780,
  };
  // optional uint32 AP_H_MAX_NUM_PLACEHOLDER_NU = 1626;
  bool has_ap_h_max_num_placeholder_nu() const;
  private:
  bool _internal_has_ap_h_max_num_placeholder_nu() const;
  public:
  void clear_ap_h_max_num_placeholder_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 ap_h_max_num_placeholder_nu() const;
  void set_ap_h_max_num_placeholder_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_ap_h_max_num_placeholder_nu() const;
  void _internal_set_ap_h_max_num_placeholder_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 AP_H_MAX_NUM_PARKING_POSES_NU = 2410;
  bool has_ap_h_max_num_parking_poses_nu() const;
  private:
  bool _internal_has_ap_h_max_num_parking_poses_nu() const;
  public:
  void clear_ap_h_max_num_parking_poses_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 ap_h_max_num_parking_poses_nu() const;
  void set_ap_h_max_num_parking_poses_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_ap_h_max_num_parking_poses_nu() const;
  void _internal_set_ap_h_max_num_parking_poses_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 AP_H_MAX_NUM_VISIBILITY_TAGS_NU = 2634;
  bool has_ap_h_max_num_visibility_tags_nu() const;
  private:
  bool _internal_has_ap_h_max_num_visibility_tags_nu() const;
  public:
  void clear_ap_h_max_num_visibility_tags_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 ap_h_max_num_visibility_tags_nu() const;
  void set_ap_h_max_num_visibility_tags_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_ap_h_max_num_visibility_tags_nu() const;
  void _internal_set_ap_h_max_num_visibility_tags_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 MAX_NUM_SECTORS_PER_SIDE_HMIH = 3725;
  bool has_max_num_sectors_per_side_hmih() const;
  private:
  bool _internal_has_max_num_sectors_per_side_hmih() const;
  public:
  void clear_max_num_sectors_per_side_hmih();
  ::PROTOBUF_NAMESPACE_ID::uint32 max_num_sectors_per_side_hmih() const;
  void set_max_num_sectors_per_side_hmih(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_max_num_sectors_per_side_hmih() const;
  void _internal_set_max_num_sectors_per_side_hmih(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 NUM_MTS_DEBUG_FREESPACE_HMIH = 3742;
  bool has_num_mts_debug_freespace_hmih() const;
  private:
  bool _internal_has_num_mts_debug_freespace_hmih() const;
  public:
  void clear_num_mts_debug_freespace_hmih();
  ::PROTOBUF_NAMESPACE_ID::uint32 num_mts_debug_freespace_hmih() const;
  void set_num_mts_debug_freespace_hmih(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_num_mts_debug_freespace_hmih() const;
  void _internal_set_num_mts_debug_freespace_hmih(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 AP_H_MAX_NUM_SLOTS_SIDE_NU = 3780;
  bool has_ap_h_max_num_slots_side_nu() const;
  private:
  bool _internal_has_ap_h_max_num_slots_side_nu() const;
  public:
  void clear_ap_h_max_num_slots_side_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 ap_h_max_num_slots_side_nu() const;
  void set_ap_h_max_num_slots_side_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_ap_h_max_num_slots_side_nu() const;
  void _internal_set_ap_h_max_num_slots_side_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 ap_h_max_num_placeholder_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 ap_h_max_num_parking_poses_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 ap_h_max_num_visibility_tags_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 max_num_sectors_per_side_hmih_;
  ::PROTOBUF_NAMESPACE_ID::uint32 num_mts_debug_freespace_hmih_;
  ::PROTOBUF_NAMESPACE_ID::uint32 ap_h_max_num_slots_side_nu_;
  friend struct ::TableStruct_mf_5fhmih_2fmf_5fhmih_5fconsts_2eproto;
};
// -------------------------------------------------------------------

class MF_HMIH_Consts_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts_array_port) */ {
 public:
  MF_HMIH_Consts_array_port();
  virtual ~MF_HMIH_Consts_array_port();

  MF_HMIH_Consts_array_port(const MF_HMIH_Consts_array_port& from);
  MF_HMIH_Consts_array_port(MF_HMIH_Consts_array_port&& from) noexcept
    : MF_HMIH_Consts_array_port() {
    *this = ::std::move(from);
  }

  inline MF_HMIH_Consts_array_port& operator=(const MF_HMIH_Consts_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline MF_HMIH_Consts_array_port& operator=(MF_HMIH_Consts_array_port&& from) noexcept {
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
  static const MF_HMIH_Consts_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MF_HMIH_Consts_array_port* internal_default_instance() {
    return reinterpret_cast<const MF_HMIH_Consts_array_port*>(
               &_MF_HMIH_Consts_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(MF_HMIH_Consts_array_port& a, MF_HMIH_Consts_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(MF_HMIH_Consts_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MF_HMIH_Consts_array_port* New() const final {
    return CreateMaybeMessage<MF_HMIH_Consts_array_port>(nullptr);
  }

  MF_HMIH_Consts_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MF_HMIH_Consts_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MF_HMIH_Consts_array_port& from);
  void MergeFrom(const MF_HMIH_Consts_array_port& from);
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
  void InternalSwap(MF_HMIH_Consts_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fmf_5fhmih_5fconsts_2eproto);
    return ::descriptor_table_mf_5fhmih_2fmf_5fhmih_5fconsts_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 145,
  };
  // repeated .pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts data = 145;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts >*
      mutable_data();
  private:
  const ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts& _internal_data(int index) const;
  ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts* _internal_add_data();
  public:
  const ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts& data(int index) const;
  ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts > data_;
  friend struct ::TableStruct_mf_5fhmih_2fmf_5fhmih_5fconsts_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MF_HMIH_Consts

// optional uint32 AP_H_MAX_NUM_SLOTS_SIDE_NU = 3780;
inline bool MF_HMIH_Consts::_internal_has_ap_h_max_num_slots_side_nu() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool MF_HMIH_Consts::has_ap_h_max_num_slots_side_nu() const {
  return _internal_has_ap_h_max_num_slots_side_nu();
}
inline void MF_HMIH_Consts::clear_ap_h_max_num_slots_side_nu() {
  ap_h_max_num_slots_side_nu_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_HMIH_Consts::_internal_ap_h_max_num_slots_side_nu() const {
  return ap_h_max_num_slots_side_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_HMIH_Consts::ap_h_max_num_slots_side_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts.AP_H_MAX_NUM_SLOTS_SIDE_NU)
  return _internal_ap_h_max_num_slots_side_nu();
}
inline void MF_HMIH_Consts::_internal_set_ap_h_max_num_slots_side_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  ap_h_max_num_slots_side_nu_ = value;
}
inline void MF_HMIH_Consts::set_ap_h_max_num_slots_side_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_ap_h_max_num_slots_side_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts.AP_H_MAX_NUM_SLOTS_SIDE_NU)
}

// optional uint32 AP_H_MAX_NUM_PLACEHOLDER_NU = 1626;
inline bool MF_HMIH_Consts::_internal_has_ap_h_max_num_placeholder_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool MF_HMIH_Consts::has_ap_h_max_num_placeholder_nu() const {
  return _internal_has_ap_h_max_num_placeholder_nu();
}
inline void MF_HMIH_Consts::clear_ap_h_max_num_placeholder_nu() {
  ap_h_max_num_placeholder_nu_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_HMIH_Consts::_internal_ap_h_max_num_placeholder_nu() const {
  return ap_h_max_num_placeholder_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_HMIH_Consts::ap_h_max_num_placeholder_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts.AP_H_MAX_NUM_PLACEHOLDER_NU)
  return _internal_ap_h_max_num_placeholder_nu();
}
inline void MF_HMIH_Consts::_internal_set_ap_h_max_num_placeholder_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  ap_h_max_num_placeholder_nu_ = value;
}
inline void MF_HMIH_Consts::set_ap_h_max_num_placeholder_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_ap_h_max_num_placeholder_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts.AP_H_MAX_NUM_PLACEHOLDER_NU)
}

// optional uint32 AP_H_MAX_NUM_PARKING_POSES_NU = 2410;
inline bool MF_HMIH_Consts::_internal_has_ap_h_max_num_parking_poses_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool MF_HMIH_Consts::has_ap_h_max_num_parking_poses_nu() const {
  return _internal_has_ap_h_max_num_parking_poses_nu();
}
inline void MF_HMIH_Consts::clear_ap_h_max_num_parking_poses_nu() {
  ap_h_max_num_parking_poses_nu_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_HMIH_Consts::_internal_ap_h_max_num_parking_poses_nu() const {
  return ap_h_max_num_parking_poses_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_HMIH_Consts::ap_h_max_num_parking_poses_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts.AP_H_MAX_NUM_PARKING_POSES_NU)
  return _internal_ap_h_max_num_parking_poses_nu();
}
inline void MF_HMIH_Consts::_internal_set_ap_h_max_num_parking_poses_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  ap_h_max_num_parking_poses_nu_ = value;
}
inline void MF_HMIH_Consts::set_ap_h_max_num_parking_poses_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_ap_h_max_num_parking_poses_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts.AP_H_MAX_NUM_PARKING_POSES_NU)
}

// optional uint32 AP_H_MAX_NUM_VISIBILITY_TAGS_NU = 2634;
inline bool MF_HMIH_Consts::_internal_has_ap_h_max_num_visibility_tags_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool MF_HMIH_Consts::has_ap_h_max_num_visibility_tags_nu() const {
  return _internal_has_ap_h_max_num_visibility_tags_nu();
}
inline void MF_HMIH_Consts::clear_ap_h_max_num_visibility_tags_nu() {
  ap_h_max_num_visibility_tags_nu_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_HMIH_Consts::_internal_ap_h_max_num_visibility_tags_nu() const {
  return ap_h_max_num_visibility_tags_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_HMIH_Consts::ap_h_max_num_visibility_tags_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts.AP_H_MAX_NUM_VISIBILITY_TAGS_NU)
  return _internal_ap_h_max_num_visibility_tags_nu();
}
inline void MF_HMIH_Consts::_internal_set_ap_h_max_num_visibility_tags_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  ap_h_max_num_visibility_tags_nu_ = value;
}
inline void MF_HMIH_Consts::set_ap_h_max_num_visibility_tags_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_ap_h_max_num_visibility_tags_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts.AP_H_MAX_NUM_VISIBILITY_TAGS_NU)
}

// optional uint32 MAX_NUM_SECTORS_PER_SIDE_HMIH = 3725;
inline bool MF_HMIH_Consts::_internal_has_max_num_sectors_per_side_hmih() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool MF_HMIH_Consts::has_max_num_sectors_per_side_hmih() const {
  return _internal_has_max_num_sectors_per_side_hmih();
}
inline void MF_HMIH_Consts::clear_max_num_sectors_per_side_hmih() {
  max_num_sectors_per_side_hmih_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_HMIH_Consts::_internal_max_num_sectors_per_side_hmih() const {
  return max_num_sectors_per_side_hmih_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_HMIH_Consts::max_num_sectors_per_side_hmih() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts.MAX_NUM_SECTORS_PER_SIDE_HMIH)
  return _internal_max_num_sectors_per_side_hmih();
}
inline void MF_HMIH_Consts::_internal_set_max_num_sectors_per_side_hmih(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  max_num_sectors_per_side_hmih_ = value;
}
inline void MF_HMIH_Consts::set_max_num_sectors_per_side_hmih(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_max_num_sectors_per_side_hmih(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts.MAX_NUM_SECTORS_PER_SIDE_HMIH)
}

// optional uint32 NUM_MTS_DEBUG_FREESPACE_HMIH = 3742;
inline bool MF_HMIH_Consts::_internal_has_num_mts_debug_freespace_hmih() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool MF_HMIH_Consts::has_num_mts_debug_freespace_hmih() const {
  return _internal_has_num_mts_debug_freespace_hmih();
}
inline void MF_HMIH_Consts::clear_num_mts_debug_freespace_hmih() {
  num_mts_debug_freespace_hmih_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_HMIH_Consts::_internal_num_mts_debug_freespace_hmih() const {
  return num_mts_debug_freespace_hmih_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_HMIH_Consts::num_mts_debug_freespace_hmih() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts.NUM_MTS_DEBUG_FREESPACE_HMIH)
  return _internal_num_mts_debug_freespace_hmih();
}
inline void MF_HMIH_Consts::_internal_set_num_mts_debug_freespace_hmih(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  num_mts_debug_freespace_hmih_ = value;
}
inline void MF_HMIH_Consts::set_num_mts_debug_freespace_hmih(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_num_mts_debug_freespace_hmih(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts.NUM_MTS_DEBUG_FREESPACE_HMIH)
}

// -------------------------------------------------------------------

// MF_HMIH_Consts_array_port

// repeated .pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts data = 145;
inline int MF_HMIH_Consts_array_port::_internal_data_size() const {
  return data_.size();
}
inline int MF_HMIH_Consts_array_port::data_size() const {
  return _internal_data_size();
}
inline void MF_HMIH_Consts_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts* MF_HMIH_Consts_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts >*
MF_HMIH_Consts_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts_array_port.data)
  return &data_;
}
inline const ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts& MF_HMIH_Consts_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts& MF_HMIH_Consts_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts* MF_HMIH_Consts_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts* MF_HMIH_Consts_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::mf_hmih_consts::MF_HMIH_Consts >&
MF_HMIH_Consts_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.mf_hmih_consts.MF_HMIH_Consts_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace mf_hmih_consts
}  // namespace mf_hmih
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fmf_5fhmih_5fconsts_2eproto
