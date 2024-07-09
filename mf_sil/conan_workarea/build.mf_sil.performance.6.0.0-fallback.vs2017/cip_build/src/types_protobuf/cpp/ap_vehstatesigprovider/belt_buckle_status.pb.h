// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/belt_buckle_status.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto

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
#include "ap_vehstatesigprovider/belt_buckle.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto;
namespace pb {
namespace ap_vehstatesigprovider {
namespace belt_buckle_status {
class BeltBuckleStatus;
class BeltBuckleStatusDefaultTypeInternal;
extern BeltBuckleStatusDefaultTypeInternal _BeltBuckleStatus_default_instance_;
class BeltBuckleStatus_array_port;
class BeltBuckleStatus_array_portDefaultTypeInternal;
extern BeltBuckleStatus_array_portDefaultTypeInternal _BeltBuckleStatus_array_port_default_instance_;
}  // namespace belt_buckle_status
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus>(Arena*);
template<> ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus_array_port* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_vehstatesigprovider {
namespace belt_buckle_status {

// ===================================================================

class BeltBuckleStatus :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus) */ {
 public:
  BeltBuckleStatus();
  virtual ~BeltBuckleStatus();

  BeltBuckleStatus(const BeltBuckleStatus& from);
  BeltBuckleStatus(BeltBuckleStatus&& from) noexcept
    : BeltBuckleStatus() {
    *this = ::std::move(from);
  }

  inline BeltBuckleStatus& operator=(const BeltBuckleStatus& from) {
    CopyFrom(from);
    return *this;
  }
  inline BeltBuckleStatus& operator=(BeltBuckleStatus&& from) noexcept {
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
  static const BeltBuckleStatus& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const BeltBuckleStatus* internal_default_instance() {
    return reinterpret_cast<const BeltBuckleStatus*>(
               &_BeltBuckleStatus_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(BeltBuckleStatus& a, BeltBuckleStatus& b) {
    a.Swap(&b);
  }
  inline void Swap(BeltBuckleStatus* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline BeltBuckleStatus* New() const final {
    return CreateMaybeMessage<BeltBuckleStatus>(nullptr);
  }

  BeltBuckleStatus* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<BeltBuckleStatus>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const BeltBuckleStatus& from);
  void MergeFrom(const BeltBuckleStatus& from);
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
  void InternalSwap(BeltBuckleStatus* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kBackrowLeftNuFieldNumber = 2374,
    kDriverNuFieldNumber = 3425,
    kBackrowRightNuFieldNumber = 3725,
    kFrontPsgrNuFieldNumber = 4002,
  };
  // optional .pb.ap_vehstatesigprovider.belt_buckle.BeltBuckle backrowLeft_nu = 2374;
  bool has_backrowleft_nu() const;
  private:
  bool _internal_has_backrowleft_nu() const;
  public:
  void clear_backrowleft_nu();
  ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle backrowleft_nu() const;
  void set_backrowleft_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value);
  private:
  ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle _internal_backrowleft_nu() const;
  void _internal_set_backrowleft_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value);
  public:

  // optional .pb.ap_vehstatesigprovider.belt_buckle.BeltBuckle driver_nu = 3425;
  bool has_driver_nu() const;
  private:
  bool _internal_has_driver_nu() const;
  public:
  void clear_driver_nu();
  ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle driver_nu() const;
  void set_driver_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value);
  private:
  ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle _internal_driver_nu() const;
  void _internal_set_driver_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value);
  public:

  // optional .pb.ap_vehstatesigprovider.belt_buckle.BeltBuckle backrowRight_nu = 3725;
  bool has_backrowright_nu() const;
  private:
  bool _internal_has_backrowright_nu() const;
  public:
  void clear_backrowright_nu();
  ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle backrowright_nu() const;
  void set_backrowright_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value);
  private:
  ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle _internal_backrowright_nu() const;
  void _internal_set_backrowright_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value);
  public:

  // optional .pb.ap_vehstatesigprovider.belt_buckle.BeltBuckle frontPsgr_nu = 4002;
  bool has_frontpsgr_nu() const;
  private:
  bool _internal_has_frontpsgr_nu() const;
  public:
  void clear_frontpsgr_nu();
  ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle frontpsgr_nu() const;
  void set_frontpsgr_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value);
  private:
  ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle _internal_frontpsgr_nu() const;
  void _internal_set_frontpsgr_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int backrowleft_nu_;
  int driver_nu_;
  int backrowright_nu_;
  int frontpsgr_nu_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto;
};
// -------------------------------------------------------------------

class BeltBuckleStatus_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus_array_port) */ {
 public:
  BeltBuckleStatus_array_port();
  virtual ~BeltBuckleStatus_array_port();

  BeltBuckleStatus_array_port(const BeltBuckleStatus_array_port& from);
  BeltBuckleStatus_array_port(BeltBuckleStatus_array_port&& from) noexcept
    : BeltBuckleStatus_array_port() {
    *this = ::std::move(from);
  }

  inline BeltBuckleStatus_array_port& operator=(const BeltBuckleStatus_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline BeltBuckleStatus_array_port& operator=(BeltBuckleStatus_array_port&& from) noexcept {
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
  static const BeltBuckleStatus_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const BeltBuckleStatus_array_port* internal_default_instance() {
    return reinterpret_cast<const BeltBuckleStatus_array_port*>(
               &_BeltBuckleStatus_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(BeltBuckleStatus_array_port& a, BeltBuckleStatus_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(BeltBuckleStatus_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline BeltBuckleStatus_array_port* New() const final {
    return CreateMaybeMessage<BeltBuckleStatus_array_port>(nullptr);
  }

  BeltBuckleStatus_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<BeltBuckleStatus_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const BeltBuckleStatus_array_port& from);
  void MergeFrom(const BeltBuckleStatus_array_port& from);
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
  void InternalSwap(BeltBuckleStatus_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3348,
  };
  // repeated .pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus data = 3348;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus >*
      mutable_data();
  private:
  const ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus& _internal_data(int index) const;
  ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus* _internal_add_data();
  public:
  const ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus& data(int index) const;
  ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus > data_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// BeltBuckleStatus

// optional .pb.ap_vehstatesigprovider.belt_buckle.BeltBuckle frontPsgr_nu = 4002;
inline bool BeltBuckleStatus::_internal_has_frontpsgr_nu() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool BeltBuckleStatus::has_frontpsgr_nu() const {
  return _internal_has_frontpsgr_nu();
}
inline void BeltBuckleStatus::clear_frontpsgr_nu() {
  frontpsgr_nu_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle BeltBuckleStatus::_internal_frontpsgr_nu() const {
  return static_cast< ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle >(frontpsgr_nu_);
}
inline ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle BeltBuckleStatus::frontpsgr_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus.frontPsgr_nu)
  return _internal_frontpsgr_nu();
}
inline void BeltBuckleStatus::_internal_set_frontpsgr_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value) {
  assert(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle_IsValid(value));
  _has_bits_[0] |= 0x00000008u;
  frontpsgr_nu_ = value;
}
inline void BeltBuckleStatus::set_frontpsgr_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value) {
  _internal_set_frontpsgr_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus.frontPsgr_nu)
}

// optional .pb.ap_vehstatesigprovider.belt_buckle.BeltBuckle driver_nu = 3425;
inline bool BeltBuckleStatus::_internal_has_driver_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool BeltBuckleStatus::has_driver_nu() const {
  return _internal_has_driver_nu();
}
inline void BeltBuckleStatus::clear_driver_nu() {
  driver_nu_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle BeltBuckleStatus::_internal_driver_nu() const {
  return static_cast< ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle >(driver_nu_);
}
inline ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle BeltBuckleStatus::driver_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus.driver_nu)
  return _internal_driver_nu();
}
inline void BeltBuckleStatus::_internal_set_driver_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value) {
  assert(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  driver_nu_ = value;
}
inline void BeltBuckleStatus::set_driver_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value) {
  _internal_set_driver_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus.driver_nu)
}

// optional .pb.ap_vehstatesigprovider.belt_buckle.BeltBuckle backrowRight_nu = 3725;
inline bool BeltBuckleStatus::_internal_has_backrowright_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool BeltBuckleStatus::has_backrowright_nu() const {
  return _internal_has_backrowright_nu();
}
inline void BeltBuckleStatus::clear_backrowright_nu() {
  backrowright_nu_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle BeltBuckleStatus::_internal_backrowright_nu() const {
  return static_cast< ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle >(backrowright_nu_);
}
inline ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle BeltBuckleStatus::backrowright_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus.backrowRight_nu)
  return _internal_backrowright_nu();
}
inline void BeltBuckleStatus::_internal_set_backrowright_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value) {
  assert(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  backrowright_nu_ = value;
}
inline void BeltBuckleStatus::set_backrowright_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value) {
  _internal_set_backrowright_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus.backrowRight_nu)
}

// optional .pb.ap_vehstatesigprovider.belt_buckle.BeltBuckle backrowLeft_nu = 2374;
inline bool BeltBuckleStatus::_internal_has_backrowleft_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool BeltBuckleStatus::has_backrowleft_nu() const {
  return _internal_has_backrowleft_nu();
}
inline void BeltBuckleStatus::clear_backrowleft_nu() {
  backrowleft_nu_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle BeltBuckleStatus::_internal_backrowleft_nu() const {
  return static_cast< ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle >(backrowleft_nu_);
}
inline ::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle BeltBuckleStatus::backrowleft_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus.backrowLeft_nu)
  return _internal_backrowleft_nu();
}
inline void BeltBuckleStatus::_internal_set_backrowleft_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value) {
  assert(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  backrowleft_nu_ = value;
}
inline void BeltBuckleStatus::set_backrowleft_nu(::pb::ap_vehstatesigprovider::belt_buckle::BeltBuckle value) {
  _internal_set_backrowleft_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus.backrowLeft_nu)
}

// -------------------------------------------------------------------

// BeltBuckleStatus_array_port

// repeated .pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus data = 3348;
inline int BeltBuckleStatus_array_port::_internal_data_size() const {
  return data_.size();
}
inline int BeltBuckleStatus_array_port::data_size() const {
  return _internal_data_size();
}
inline void BeltBuckleStatus_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus* BeltBuckleStatus_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus >*
BeltBuckleStatus_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus_array_port.data)
  return &data_;
}
inline const ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus& BeltBuckleStatus_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus& BeltBuckleStatus_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus* BeltBuckleStatus_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus* BeltBuckleStatus_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus >&
BeltBuckleStatus_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace belt_buckle_status
}  // namespace ap_vehstatesigprovider
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto