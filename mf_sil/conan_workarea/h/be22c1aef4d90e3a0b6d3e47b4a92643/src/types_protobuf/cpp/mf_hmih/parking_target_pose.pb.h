// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_hmih/parking_target_pose.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fparking_5ftarget_5fpose_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fparking_5ftarget_5fpose_2eproto

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
#include "lsm_geoml/pose_pod.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fhmih_2fparking_5ftarget_5fpose_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fhmih_2fparking_5ftarget_5fpose_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fhmih_2fparking_5ftarget_5fpose_2eproto;
namespace pb {
namespace mf_hmih {
namespace parking_target_pose {
class ParkingTargetPose;
class ParkingTargetPoseDefaultTypeInternal;
extern ParkingTargetPoseDefaultTypeInternal _ParkingTargetPose_default_instance_;
class ParkingTargetPose_array_port;
class ParkingTargetPose_array_portDefaultTypeInternal;
extern ParkingTargetPose_array_portDefaultTypeInternal _ParkingTargetPose_array_port_default_instance_;
}  // namespace parking_target_pose
}  // namespace mf_hmih
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_hmih::parking_target_pose::ParkingTargetPose* Arena::CreateMaybeMessage<::pb::mf_hmih::parking_target_pose::ParkingTargetPose>(Arena*);
template<> ::pb::mf_hmih::parking_target_pose::ParkingTargetPose_array_port* Arena::CreateMaybeMessage<::pb::mf_hmih::parking_target_pose::ParkingTargetPose_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_hmih {
namespace parking_target_pose {

// ===================================================================

class ParkingTargetPose :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.parking_target_pose.ParkingTargetPose) */ {
 public:
  ParkingTargetPose();
  virtual ~ParkingTargetPose();

  ParkingTargetPose(const ParkingTargetPose& from);
  ParkingTargetPose(ParkingTargetPose&& from) noexcept
    : ParkingTargetPose() {
    *this = ::std::move(from);
  }

  inline ParkingTargetPose& operator=(const ParkingTargetPose& from) {
    CopyFrom(from);
    return *this;
  }
  inline ParkingTargetPose& operator=(ParkingTargetPose&& from) noexcept {
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
  static const ParkingTargetPose& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ParkingTargetPose* internal_default_instance() {
    return reinterpret_cast<const ParkingTargetPose*>(
               &_ParkingTargetPose_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ParkingTargetPose& a, ParkingTargetPose& b) {
    a.Swap(&b);
  }
  inline void Swap(ParkingTargetPose* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ParkingTargetPose* New() const final {
    return CreateMaybeMessage<ParkingTargetPose>(nullptr);
  }

  ParkingTargetPose* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ParkingTargetPose>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ParkingTargetPose& from);
  void MergeFrom(const ParkingTargetPose& from);
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
  void InternalSwap(ParkingTargetPose* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.parking_target_pose.ParkingTargetPose";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fparking_5ftarget_5fpose_2eproto);
    return ::descriptor_table_mf_5fhmih_2fparking_5ftarget_5fpose_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kPoseNuFieldNumber = 151,
    kIdNuFieldNumber = 111,
    kIsFreeNuFieldNumber = 1388,
    kIsScannedNuFieldNumber = 230,
    kIsSelectedNuFieldNumber = 61,
    kIsSwitchableNuFieldNumber = 387,
  };
  // optional .pb.lsm_geoml.pose_pod.Pose_POD pose_nu = 151;
  bool has_pose_nu() const;
  private:
  bool _internal_has_pose_nu() const;
  public:
  void clear_pose_nu();
  const ::pb::lsm_geoml::pose_pod::Pose_POD& pose_nu() const;
  ::pb::lsm_geoml::pose_pod::Pose_POD* release_pose_nu();
  ::pb::lsm_geoml::pose_pod::Pose_POD* mutable_pose_nu();
  void set_allocated_pose_nu(::pb::lsm_geoml::pose_pod::Pose_POD* pose_nu);
  private:
  const ::pb::lsm_geoml::pose_pod::Pose_POD& _internal_pose_nu() const;
  ::pb::lsm_geoml::pose_pod::Pose_POD* _internal_mutable_pose_nu();
  public:

  // optional uint32 id_nu = 111;
  bool has_id_nu() const;
  private:
  bool _internal_has_id_nu() const;
  public:
  void clear_id_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 id_nu() const;
  void set_id_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_id_nu() const;
  void _internal_set_id_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional bool isFree_nu = 1388;
  bool has_isfree_nu() const;
  private:
  bool _internal_has_isfree_nu() const;
  public:
  void clear_isfree_nu();
  bool isfree_nu() const;
  void set_isfree_nu(bool value);
  private:
  bool _internal_isfree_nu() const;
  void _internal_set_isfree_nu(bool value);
  public:

  // optional bool isScanned_nu = 230;
  bool has_isscanned_nu() const;
  private:
  bool _internal_has_isscanned_nu() const;
  public:
  void clear_isscanned_nu();
  bool isscanned_nu() const;
  void set_isscanned_nu(bool value);
  private:
  bool _internal_isscanned_nu() const;
  void _internal_set_isscanned_nu(bool value);
  public:

  // optional bool isSelected_nu = 61;
  bool has_isselected_nu() const;
  private:
  bool _internal_has_isselected_nu() const;
  public:
  void clear_isselected_nu();
  bool isselected_nu() const;
  void set_isselected_nu(bool value);
  private:
  bool _internal_isselected_nu() const;
  void _internal_set_isselected_nu(bool value);
  public:

  // optional bool isSwitchable_nu = 387;
  bool has_isswitchable_nu() const;
  private:
  bool _internal_has_isswitchable_nu() const;
  public:
  void clear_isswitchable_nu();
  bool isswitchable_nu() const;
  void set_isswitchable_nu(bool value);
  private:
  bool _internal_isswitchable_nu() const;
  void _internal_set_isswitchable_nu(bool value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.parking_target_pose.ParkingTargetPose)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::lsm_geoml::pose_pod::Pose_POD* pose_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 id_nu_;
  bool isfree_nu_;
  bool isscanned_nu_;
  bool isselected_nu_;
  bool isswitchable_nu_;
  friend struct ::TableStruct_mf_5fhmih_2fparking_5ftarget_5fpose_2eproto;
};
// -------------------------------------------------------------------

class ParkingTargetPose_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.parking_target_pose.ParkingTargetPose_array_port) */ {
 public:
  ParkingTargetPose_array_port();
  virtual ~ParkingTargetPose_array_port();

  ParkingTargetPose_array_port(const ParkingTargetPose_array_port& from);
  ParkingTargetPose_array_port(ParkingTargetPose_array_port&& from) noexcept
    : ParkingTargetPose_array_port() {
    *this = ::std::move(from);
  }

  inline ParkingTargetPose_array_port& operator=(const ParkingTargetPose_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline ParkingTargetPose_array_port& operator=(ParkingTargetPose_array_port&& from) noexcept {
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
  static const ParkingTargetPose_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ParkingTargetPose_array_port* internal_default_instance() {
    return reinterpret_cast<const ParkingTargetPose_array_port*>(
               &_ParkingTargetPose_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(ParkingTargetPose_array_port& a, ParkingTargetPose_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(ParkingTargetPose_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ParkingTargetPose_array_port* New() const final {
    return CreateMaybeMessage<ParkingTargetPose_array_port>(nullptr);
  }

  ParkingTargetPose_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ParkingTargetPose_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ParkingTargetPose_array_port& from);
  void MergeFrom(const ParkingTargetPose_array_port& from);
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
  void InternalSwap(ParkingTargetPose_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.parking_target_pose.ParkingTargetPose_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fparking_5ftarget_5fpose_2eproto);
    return ::descriptor_table_mf_5fhmih_2fparking_5ftarget_5fpose_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3660,
  };
  // repeated .pb.mf_hmih.parking_target_pose.ParkingTargetPose data = 3660;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_hmih::parking_target_pose::ParkingTargetPose* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::parking_target_pose::ParkingTargetPose >*
      mutable_data();
  private:
  const ::pb::mf_hmih::parking_target_pose::ParkingTargetPose& _internal_data(int index) const;
  ::pb::mf_hmih::parking_target_pose::ParkingTargetPose* _internal_add_data();
  public:
  const ::pb::mf_hmih::parking_target_pose::ParkingTargetPose& data(int index) const;
  ::pb::mf_hmih::parking_target_pose::ParkingTargetPose* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::parking_target_pose::ParkingTargetPose >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.parking_target_pose.ParkingTargetPose_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::parking_target_pose::ParkingTargetPose > data_;
  friend struct ::TableStruct_mf_5fhmih_2fparking_5ftarget_5fpose_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ParkingTargetPose

// optional uint32 id_nu = 111;
inline bool ParkingTargetPose::_internal_has_id_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool ParkingTargetPose::has_id_nu() const {
  return _internal_has_id_nu();
}
inline void ParkingTargetPose::clear_id_nu() {
  id_nu_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ParkingTargetPose::_internal_id_nu() const {
  return id_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ParkingTargetPose::id_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.parking_target_pose.ParkingTargetPose.id_nu)
  return _internal_id_nu();
}
inline void ParkingTargetPose::_internal_set_id_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  id_nu_ = value;
}
inline void ParkingTargetPose::set_id_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_id_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.parking_target_pose.ParkingTargetPose.id_nu)
}

// optional bool isFree_nu = 1388;
inline bool ParkingTargetPose::_internal_has_isfree_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool ParkingTargetPose::has_isfree_nu() const {
  return _internal_has_isfree_nu();
}
inline void ParkingTargetPose::clear_isfree_nu() {
  isfree_nu_ = false;
  _has_bits_[0] &= ~0x00000004u;
}
inline bool ParkingTargetPose::_internal_isfree_nu() const {
  return isfree_nu_;
}
inline bool ParkingTargetPose::isfree_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.parking_target_pose.ParkingTargetPose.isFree_nu)
  return _internal_isfree_nu();
}
inline void ParkingTargetPose::_internal_set_isfree_nu(bool value) {
  _has_bits_[0] |= 0x00000004u;
  isfree_nu_ = value;
}
inline void ParkingTargetPose::set_isfree_nu(bool value) {
  _internal_set_isfree_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.parking_target_pose.ParkingTargetPose.isFree_nu)
}

// optional bool isScanned_nu = 230;
inline bool ParkingTargetPose::_internal_has_isscanned_nu() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool ParkingTargetPose::has_isscanned_nu() const {
  return _internal_has_isscanned_nu();
}
inline void ParkingTargetPose::clear_isscanned_nu() {
  isscanned_nu_ = false;
  _has_bits_[0] &= ~0x00000008u;
}
inline bool ParkingTargetPose::_internal_isscanned_nu() const {
  return isscanned_nu_;
}
inline bool ParkingTargetPose::isscanned_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.parking_target_pose.ParkingTargetPose.isScanned_nu)
  return _internal_isscanned_nu();
}
inline void ParkingTargetPose::_internal_set_isscanned_nu(bool value) {
  _has_bits_[0] |= 0x00000008u;
  isscanned_nu_ = value;
}
inline void ParkingTargetPose::set_isscanned_nu(bool value) {
  _internal_set_isscanned_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.parking_target_pose.ParkingTargetPose.isScanned_nu)
}

// optional bool isSelected_nu = 61;
inline bool ParkingTargetPose::_internal_has_isselected_nu() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool ParkingTargetPose::has_isselected_nu() const {
  return _internal_has_isselected_nu();
}
inline void ParkingTargetPose::clear_isselected_nu() {
  isselected_nu_ = false;
  _has_bits_[0] &= ~0x00000010u;
}
inline bool ParkingTargetPose::_internal_isselected_nu() const {
  return isselected_nu_;
}
inline bool ParkingTargetPose::isselected_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.parking_target_pose.ParkingTargetPose.isSelected_nu)
  return _internal_isselected_nu();
}
inline void ParkingTargetPose::_internal_set_isselected_nu(bool value) {
  _has_bits_[0] |= 0x00000010u;
  isselected_nu_ = value;
}
inline void ParkingTargetPose::set_isselected_nu(bool value) {
  _internal_set_isselected_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.parking_target_pose.ParkingTargetPose.isSelected_nu)
}

// optional bool isSwitchable_nu = 387;
inline bool ParkingTargetPose::_internal_has_isswitchable_nu() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool ParkingTargetPose::has_isswitchable_nu() const {
  return _internal_has_isswitchable_nu();
}
inline void ParkingTargetPose::clear_isswitchable_nu() {
  isswitchable_nu_ = false;
  _has_bits_[0] &= ~0x00000020u;
}
inline bool ParkingTargetPose::_internal_isswitchable_nu() const {
  return isswitchable_nu_;
}
inline bool ParkingTargetPose::isswitchable_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.parking_target_pose.ParkingTargetPose.isSwitchable_nu)
  return _internal_isswitchable_nu();
}
inline void ParkingTargetPose::_internal_set_isswitchable_nu(bool value) {
  _has_bits_[0] |= 0x00000020u;
  isswitchable_nu_ = value;
}
inline void ParkingTargetPose::set_isswitchable_nu(bool value) {
  _internal_set_isswitchable_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.parking_target_pose.ParkingTargetPose.isSwitchable_nu)
}

// optional .pb.lsm_geoml.pose_pod.Pose_POD pose_nu = 151;
inline bool ParkingTargetPose::_internal_has_pose_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || pose_nu_ != nullptr);
  return value;
}
inline bool ParkingTargetPose::has_pose_nu() const {
  return _internal_has_pose_nu();
}
inline const ::pb::lsm_geoml::pose_pod::Pose_POD& ParkingTargetPose::_internal_pose_nu() const {
  const ::pb::lsm_geoml::pose_pod::Pose_POD* p = pose_nu_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::lsm_geoml::pose_pod::Pose_POD*>(
      &::pb::lsm_geoml::pose_pod::_Pose_POD_default_instance_);
}
inline const ::pb::lsm_geoml::pose_pod::Pose_POD& ParkingTargetPose::pose_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.parking_target_pose.ParkingTargetPose.pose_nu)
  return _internal_pose_nu();
}
inline ::pb::lsm_geoml::pose_pod::Pose_POD* ParkingTargetPose::release_pose_nu() {
  // @@protoc_insertion_point(field_release:pb.mf_hmih.parking_target_pose.ParkingTargetPose.pose_nu)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::lsm_geoml::pose_pod::Pose_POD* temp = pose_nu_;
  pose_nu_ = nullptr;
  return temp;
}
inline ::pb::lsm_geoml::pose_pod::Pose_POD* ParkingTargetPose::_internal_mutable_pose_nu() {
  _has_bits_[0] |= 0x00000001u;
  if (pose_nu_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::lsm_geoml::pose_pod::Pose_POD>(GetArenaNoVirtual());
    pose_nu_ = p;
  }
  return pose_nu_;
}
inline ::pb::lsm_geoml::pose_pod::Pose_POD* ParkingTargetPose::mutable_pose_nu() {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.parking_target_pose.ParkingTargetPose.pose_nu)
  return _internal_mutable_pose_nu();
}
inline void ParkingTargetPose::set_allocated_pose_nu(::pb::lsm_geoml::pose_pod::Pose_POD* pose_nu) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(pose_nu_);
  }
  if (pose_nu) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      pose_nu = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, pose_nu, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  pose_nu_ = pose_nu;
  // @@protoc_insertion_point(field_set_allocated:pb.mf_hmih.parking_target_pose.ParkingTargetPose.pose_nu)
}

// -------------------------------------------------------------------

// ParkingTargetPose_array_port

// repeated .pb.mf_hmih.parking_target_pose.ParkingTargetPose data = 3660;
inline int ParkingTargetPose_array_port::_internal_data_size() const {
  return data_.size();
}
inline int ParkingTargetPose_array_port::data_size() const {
  return _internal_data_size();
}
inline void ParkingTargetPose_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_hmih::parking_target_pose::ParkingTargetPose* ParkingTargetPose_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.parking_target_pose.ParkingTargetPose_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::parking_target_pose::ParkingTargetPose >*
ParkingTargetPose_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.parking_target_pose.ParkingTargetPose_array_port.data)
  return &data_;
}
inline const ::pb::mf_hmih::parking_target_pose::ParkingTargetPose& ParkingTargetPose_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_hmih::parking_target_pose::ParkingTargetPose& ParkingTargetPose_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.parking_target_pose.ParkingTargetPose_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_hmih::parking_target_pose::ParkingTargetPose* ParkingTargetPose_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_hmih::parking_target_pose::ParkingTargetPose* ParkingTargetPose_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_hmih.parking_target_pose.ParkingTargetPose_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::parking_target_pose::ParkingTargetPose >&
ParkingTargetPose_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.parking_target_pose.ParkingTargetPose_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace parking_target_pose
}  // namespace mf_hmih
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fparking_5ftarget_5fpose_2eproto
