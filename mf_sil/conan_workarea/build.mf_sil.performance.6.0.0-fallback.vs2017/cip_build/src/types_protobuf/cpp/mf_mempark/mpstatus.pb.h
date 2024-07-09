// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_mempark/mpstatus.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmpstatus_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmpstatus_2eproto

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
#include "mf_mempark/memorized_slot.pb.h"
#include "mf_mempark/memorized_parking_status.pb.h"
#include "mf_mempark/training_status.pb.h"
#include "mf_mempark/localization_status.pb.h"
#include "mf_mempark/user_update_request_status.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fmempark_2fmpstatus_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fmempark_2fmpstatus_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmempark_2fmpstatus_2eproto;
namespace pb {
namespace mf_mempark {
namespace mpstatus {
class MPStatus;
class MPStatusDefaultTypeInternal;
extern MPStatusDefaultTypeInternal _MPStatus_default_instance_;
class MPStatus_array_port;
class MPStatus_array_portDefaultTypeInternal;
extern MPStatus_array_portDefaultTypeInternal _MPStatus_array_port_default_instance_;
}  // namespace mpstatus
}  // namespace mf_mempark
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_mempark::mpstatus::MPStatus* Arena::CreateMaybeMessage<::pb::mf_mempark::mpstatus::MPStatus>(Arena*);
template<> ::pb::mf_mempark::mpstatus::MPStatus_array_port* Arena::CreateMaybeMessage<::pb::mf_mempark::mpstatus::MPStatus_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_mempark {
namespace mpstatus {

// ===================================================================

class MPStatus :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.mpstatus.MPStatus) */ {
 public:
  MPStatus();
  virtual ~MPStatus();

  MPStatus(const MPStatus& from);
  MPStatus(MPStatus&& from) noexcept
    : MPStatus() {
    *this = ::std::move(from);
  }

  inline MPStatus& operator=(const MPStatus& from) {
    CopyFrom(from);
    return *this;
  }
  inline MPStatus& operator=(MPStatus&& from) noexcept {
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
  static const MPStatus& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MPStatus* internal_default_instance() {
    return reinterpret_cast<const MPStatus*>(
               &_MPStatus_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MPStatus& a, MPStatus& b) {
    a.Swap(&b);
  }
  inline void Swap(MPStatus* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MPStatus* New() const final {
    return CreateMaybeMessage<MPStatus>(nullptr);
  }

  MPStatus* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MPStatus>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MPStatus& from);
  void MergeFrom(const MPStatus& from);
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
  void InternalSwap(MPStatus* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.mpstatus.MPStatus";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2fmpstatus_2eproto);
    return ::descriptor_table_mf_5fmempark_2fmpstatus_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kMemorizedParkingSlotsFieldNumber = 3661,
    kNumStoredMemoryParkingSlotsNuFieldNumber = 2935,
    kTrainingStatusFieldNumber = 612,
    kMemoryParkingStateFieldNumber = 866,
    kUserUpdateRequestStatusFieldNumber = 1635,
    kLocalizationStatusFieldNumber = 1648,
  };
  // repeated .pb.mf_mempark.memorized_slot.MemorizedSlot memorizedParkingSlots = 3661;
  int memorizedparkingslots_size() const;
  private:
  int _internal_memorizedparkingslots_size() const;
  public:
  void clear_memorizedparkingslots();
  ::pb::mf_mempark::memorized_slot::MemorizedSlot* mutable_memorizedparkingslots(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::memorized_slot::MemorizedSlot >*
      mutable_memorizedparkingslots();
  private:
  const ::pb::mf_mempark::memorized_slot::MemorizedSlot& _internal_memorizedparkingslots(int index) const;
  ::pb::mf_mempark::memorized_slot::MemorizedSlot* _internal_add_memorizedparkingslots();
  public:
  const ::pb::mf_mempark::memorized_slot::MemorizedSlot& memorizedparkingslots(int index) const;
  ::pb::mf_mempark::memorized_slot::MemorizedSlot* add_memorizedparkingslots();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::memorized_slot::MemorizedSlot >&
      memorizedparkingslots() const;

  // optional uint32 numStoredMemoryParkingSlots_nu = 2935;
  bool has_numstoredmemoryparkingslots_nu() const;
  private:
  bool _internal_has_numstoredmemoryparkingslots_nu() const;
  public:
  void clear_numstoredmemoryparkingslots_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 numstoredmemoryparkingslots_nu() const;
  void set_numstoredmemoryparkingslots_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_numstoredmemoryparkingslots_nu() const;
  void _internal_set_numstoredmemoryparkingslots_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional .pb.mf_mempark.training_status.TrainingStatus trainingStatus = 612;
  bool has_trainingstatus() const;
  private:
  bool _internal_has_trainingstatus() const;
  public:
  void clear_trainingstatus();
  ::pb::mf_mempark::training_status::TrainingStatus trainingstatus() const;
  void set_trainingstatus(::pb::mf_mempark::training_status::TrainingStatus value);
  private:
  ::pb::mf_mempark::training_status::TrainingStatus _internal_trainingstatus() const;
  void _internal_set_trainingstatus(::pb::mf_mempark::training_status::TrainingStatus value);
  public:

  // optional .pb.mf_mempark.memorized_parking_status.MemorizedParkingStatus memoryParkingState = 866;
  bool has_memoryparkingstate() const;
  private:
  bool _internal_has_memoryparkingstate() const;
  public:
  void clear_memoryparkingstate();
  ::pb::mf_mempark::memorized_parking_status::MemorizedParkingStatus memoryparkingstate() const;
  void set_memoryparkingstate(::pb::mf_mempark::memorized_parking_status::MemorizedParkingStatus value);
  private:
  ::pb::mf_mempark::memorized_parking_status::MemorizedParkingStatus _internal_memoryparkingstate() const;
  void _internal_set_memoryparkingstate(::pb::mf_mempark::memorized_parking_status::MemorizedParkingStatus value);
  public:

  // optional .pb.mf_mempark.user_update_request_status.UserUpdateRequestStatus userUpdateRequestStatus = 1635;
  bool has_userupdaterequeststatus() const;
  private:
  bool _internal_has_userupdaterequeststatus() const;
  public:
  void clear_userupdaterequeststatus();
  ::pb::mf_mempark::user_update_request_status::UserUpdateRequestStatus userupdaterequeststatus() const;
  void set_userupdaterequeststatus(::pb::mf_mempark::user_update_request_status::UserUpdateRequestStatus value);
  private:
  ::pb::mf_mempark::user_update_request_status::UserUpdateRequestStatus _internal_userupdaterequeststatus() const;
  void _internal_set_userupdaterequeststatus(::pb::mf_mempark::user_update_request_status::UserUpdateRequestStatus value);
  public:

  // optional .pb.mf_mempark.localization_status.LocalizationStatus localizationStatus = 1648;
  bool has_localizationstatus() const;
  private:
  bool _internal_has_localizationstatus() const;
  public:
  void clear_localizationstatus();
  ::pb::mf_mempark::localization_status::LocalizationStatus localizationstatus() const;
  void set_localizationstatus(::pb::mf_mempark::localization_status::LocalizationStatus value);
  private:
  ::pb::mf_mempark::localization_status::LocalizationStatus _internal_localizationstatus() const;
  void _internal_set_localizationstatus(::pb::mf_mempark::localization_status::LocalizationStatus value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.mpstatus.MPStatus)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::memorized_slot::MemorizedSlot > memorizedparkingslots_;
  ::PROTOBUF_NAMESPACE_ID::uint32 numstoredmemoryparkingslots_nu_;
  int trainingstatus_;
  int memoryparkingstate_;
  int userupdaterequeststatus_;
  int localizationstatus_;
  friend struct ::TableStruct_mf_5fmempark_2fmpstatus_2eproto;
};
// -------------------------------------------------------------------

class MPStatus_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.mpstatus.MPStatus_array_port) */ {
 public:
  MPStatus_array_port();
  virtual ~MPStatus_array_port();

  MPStatus_array_port(const MPStatus_array_port& from);
  MPStatus_array_port(MPStatus_array_port&& from) noexcept
    : MPStatus_array_port() {
    *this = ::std::move(from);
  }

  inline MPStatus_array_port& operator=(const MPStatus_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline MPStatus_array_port& operator=(MPStatus_array_port&& from) noexcept {
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
  static const MPStatus_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MPStatus_array_port* internal_default_instance() {
    return reinterpret_cast<const MPStatus_array_port*>(
               &_MPStatus_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(MPStatus_array_port& a, MPStatus_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(MPStatus_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MPStatus_array_port* New() const final {
    return CreateMaybeMessage<MPStatus_array_port>(nullptr);
  }

  MPStatus_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MPStatus_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MPStatus_array_port& from);
  void MergeFrom(const MPStatus_array_port& from);
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
  void InternalSwap(MPStatus_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.mpstatus.MPStatus_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2fmpstatus_2eproto);
    return ::descriptor_table_mf_5fmempark_2fmpstatus_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1796,
  };
  // repeated .pb.mf_mempark.mpstatus.MPStatus data = 1796;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_mempark::mpstatus::MPStatus* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::mpstatus::MPStatus >*
      mutable_data();
  private:
  const ::pb::mf_mempark::mpstatus::MPStatus& _internal_data(int index) const;
  ::pb::mf_mempark::mpstatus::MPStatus* _internal_add_data();
  public:
  const ::pb::mf_mempark::mpstatus::MPStatus& data(int index) const;
  ::pb::mf_mempark::mpstatus::MPStatus* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::mpstatus::MPStatus >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.mpstatus.MPStatus_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::mpstatus::MPStatus > data_;
  friend struct ::TableStruct_mf_5fmempark_2fmpstatus_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MPStatus

// optional uint32 numStoredMemoryParkingSlots_nu = 2935;
inline bool MPStatus::_internal_has_numstoredmemoryparkingslots_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool MPStatus::has_numstoredmemoryparkingslots_nu() const {
  return _internal_has_numstoredmemoryparkingslots_nu();
}
inline void MPStatus::clear_numstoredmemoryparkingslots_nu() {
  numstoredmemoryparkingslots_nu_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MPStatus::_internal_numstoredmemoryparkingslots_nu() const {
  return numstoredmemoryparkingslots_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MPStatus::numstoredmemoryparkingslots_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mpstatus.MPStatus.numStoredMemoryParkingSlots_nu)
  return _internal_numstoredmemoryparkingslots_nu();
}
inline void MPStatus::_internal_set_numstoredmemoryparkingslots_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  numstoredmemoryparkingslots_nu_ = value;
}
inline void MPStatus::set_numstoredmemoryparkingslots_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_numstoredmemoryparkingslots_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.mpstatus.MPStatus.numStoredMemoryParkingSlots_nu)
}

// repeated .pb.mf_mempark.memorized_slot.MemorizedSlot memorizedParkingSlots = 3661;
inline int MPStatus::_internal_memorizedparkingslots_size() const {
  return memorizedparkingslots_.size();
}
inline int MPStatus::memorizedparkingslots_size() const {
  return _internal_memorizedparkingslots_size();
}
inline ::pb::mf_mempark::memorized_slot::MemorizedSlot* MPStatus::mutable_memorizedparkingslots(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.mpstatus.MPStatus.memorizedParkingSlots)
  return memorizedparkingslots_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::memorized_slot::MemorizedSlot >*
MPStatus::mutable_memorizedparkingslots() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_mempark.mpstatus.MPStatus.memorizedParkingSlots)
  return &memorizedparkingslots_;
}
inline const ::pb::mf_mempark::memorized_slot::MemorizedSlot& MPStatus::_internal_memorizedparkingslots(int index) const {
  return memorizedparkingslots_.Get(index);
}
inline const ::pb::mf_mempark::memorized_slot::MemorizedSlot& MPStatus::memorizedparkingslots(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mpstatus.MPStatus.memorizedParkingSlots)
  return _internal_memorizedparkingslots(index);
}
inline ::pb::mf_mempark::memorized_slot::MemorizedSlot* MPStatus::_internal_add_memorizedparkingslots() {
  return memorizedparkingslots_.Add();
}
inline ::pb::mf_mempark::memorized_slot::MemorizedSlot* MPStatus::add_memorizedparkingslots() {
  // @@protoc_insertion_point(field_add:pb.mf_mempark.mpstatus.MPStatus.memorizedParkingSlots)
  return _internal_add_memorizedparkingslots();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::memorized_slot::MemorizedSlot >&
MPStatus::memorizedparkingslots() const {
  // @@protoc_insertion_point(field_list:pb.mf_mempark.mpstatus.MPStatus.memorizedParkingSlots)
  return memorizedparkingslots_;
}

// optional .pb.mf_mempark.memorized_parking_status.MemorizedParkingStatus memoryParkingState = 866;
inline bool MPStatus::_internal_has_memoryparkingstate() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool MPStatus::has_memoryparkingstate() const {
  return _internal_has_memoryparkingstate();
}
inline void MPStatus::clear_memoryparkingstate() {
  memoryparkingstate_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::mf_mempark::memorized_parking_status::MemorizedParkingStatus MPStatus::_internal_memoryparkingstate() const {
  return static_cast< ::pb::mf_mempark::memorized_parking_status::MemorizedParkingStatus >(memoryparkingstate_);
}
inline ::pb::mf_mempark::memorized_parking_status::MemorizedParkingStatus MPStatus::memoryparkingstate() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mpstatus.MPStatus.memoryParkingState)
  return _internal_memoryparkingstate();
}
inline void MPStatus::_internal_set_memoryparkingstate(::pb::mf_mempark::memorized_parking_status::MemorizedParkingStatus value) {
  assert(::pb::mf_mempark::memorized_parking_status::MemorizedParkingStatus_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  memoryparkingstate_ = value;
}
inline void MPStatus::set_memoryparkingstate(::pb::mf_mempark::memorized_parking_status::MemorizedParkingStatus value) {
  _internal_set_memoryparkingstate(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.mpstatus.MPStatus.memoryParkingState)
}

// optional .pb.mf_mempark.training_status.TrainingStatus trainingStatus = 612;
inline bool MPStatus::_internal_has_trainingstatus() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool MPStatus::has_trainingstatus() const {
  return _internal_has_trainingstatus();
}
inline void MPStatus::clear_trainingstatus() {
  trainingstatus_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::pb::mf_mempark::training_status::TrainingStatus MPStatus::_internal_trainingstatus() const {
  return static_cast< ::pb::mf_mempark::training_status::TrainingStatus >(trainingstatus_);
}
inline ::pb::mf_mempark::training_status::TrainingStatus MPStatus::trainingstatus() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mpstatus.MPStatus.trainingStatus)
  return _internal_trainingstatus();
}
inline void MPStatus::_internal_set_trainingstatus(::pb::mf_mempark::training_status::TrainingStatus value) {
  assert(::pb::mf_mempark::training_status::TrainingStatus_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  trainingstatus_ = value;
}
inline void MPStatus::set_trainingstatus(::pb::mf_mempark::training_status::TrainingStatus value) {
  _internal_set_trainingstatus(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.mpstatus.MPStatus.trainingStatus)
}

// optional .pb.mf_mempark.localization_status.LocalizationStatus localizationStatus = 1648;
inline bool MPStatus::_internal_has_localizationstatus() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool MPStatus::has_localizationstatus() const {
  return _internal_has_localizationstatus();
}
inline void MPStatus::clear_localizationstatus() {
  localizationstatus_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::pb::mf_mempark::localization_status::LocalizationStatus MPStatus::_internal_localizationstatus() const {
  return static_cast< ::pb::mf_mempark::localization_status::LocalizationStatus >(localizationstatus_);
}
inline ::pb::mf_mempark::localization_status::LocalizationStatus MPStatus::localizationstatus() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mpstatus.MPStatus.localizationStatus)
  return _internal_localizationstatus();
}
inline void MPStatus::_internal_set_localizationstatus(::pb::mf_mempark::localization_status::LocalizationStatus value) {
  assert(::pb::mf_mempark::localization_status::LocalizationStatus_IsValid(value));
  _has_bits_[0] |= 0x00000010u;
  localizationstatus_ = value;
}
inline void MPStatus::set_localizationstatus(::pb::mf_mempark::localization_status::LocalizationStatus value) {
  _internal_set_localizationstatus(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.mpstatus.MPStatus.localizationStatus)
}

// optional .pb.mf_mempark.user_update_request_status.UserUpdateRequestStatus userUpdateRequestStatus = 1635;
inline bool MPStatus::_internal_has_userupdaterequeststatus() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool MPStatus::has_userupdaterequeststatus() const {
  return _internal_has_userupdaterequeststatus();
}
inline void MPStatus::clear_userupdaterequeststatus() {
  userupdaterequeststatus_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::pb::mf_mempark::user_update_request_status::UserUpdateRequestStatus MPStatus::_internal_userupdaterequeststatus() const {
  return static_cast< ::pb::mf_mempark::user_update_request_status::UserUpdateRequestStatus >(userupdaterequeststatus_);
}
inline ::pb::mf_mempark::user_update_request_status::UserUpdateRequestStatus MPStatus::userupdaterequeststatus() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mpstatus.MPStatus.userUpdateRequestStatus)
  return _internal_userupdaterequeststatus();
}
inline void MPStatus::_internal_set_userupdaterequeststatus(::pb::mf_mempark::user_update_request_status::UserUpdateRequestStatus value) {
  assert(::pb::mf_mempark::user_update_request_status::UserUpdateRequestStatus_IsValid(value));
  _has_bits_[0] |= 0x00000008u;
  userupdaterequeststatus_ = value;
}
inline void MPStatus::set_userupdaterequeststatus(::pb::mf_mempark::user_update_request_status::UserUpdateRequestStatus value) {
  _internal_set_userupdaterequeststatus(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.mpstatus.MPStatus.userUpdateRequestStatus)
}

// -------------------------------------------------------------------

// MPStatus_array_port

// repeated .pb.mf_mempark.mpstatus.MPStatus data = 1796;
inline int MPStatus_array_port::_internal_data_size() const {
  return data_.size();
}
inline int MPStatus_array_port::data_size() const {
  return _internal_data_size();
}
inline void MPStatus_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_mempark::mpstatus::MPStatus* MPStatus_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.mpstatus.MPStatus_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::mpstatus::MPStatus >*
MPStatus_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_mempark.mpstatus.MPStatus_array_port.data)
  return &data_;
}
inline const ::pb::mf_mempark::mpstatus::MPStatus& MPStatus_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_mempark::mpstatus::MPStatus& MPStatus_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.mpstatus.MPStatus_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_mempark::mpstatus::MPStatus* MPStatus_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_mempark::mpstatus::MPStatus* MPStatus_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_mempark.mpstatus.MPStatus_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::mpstatus::MPStatus >&
MPStatus_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_mempark.mpstatus.MPStatus_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace mpstatus
}  // namespace mf_mempark
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmpstatus_2eproto