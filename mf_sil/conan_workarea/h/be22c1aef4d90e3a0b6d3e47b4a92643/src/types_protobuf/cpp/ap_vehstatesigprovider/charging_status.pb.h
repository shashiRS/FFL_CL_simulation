// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/charging_status.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto

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
#include "ap_vehstatesigprovider/charging_connector_status.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto;
namespace pb {
namespace ap_vehstatesigprovider {
namespace charging_status {
class ChargingStatus;
class ChargingStatusDefaultTypeInternal;
extern ChargingStatusDefaultTypeInternal _ChargingStatus_default_instance_;
class ChargingStatus_array_port;
class ChargingStatus_array_portDefaultTypeInternal;
extern ChargingStatus_array_portDefaultTypeInternal _ChargingStatus_array_port_default_instance_;
}  // namespace charging_status
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::charging_status::ChargingStatus>(Arena*);
template<> ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus_array_port* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::charging_status::ChargingStatus_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_vehstatesigprovider {
namespace charging_status {

// ===================================================================

class ChargingStatus :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.charging_status.ChargingStatus) */ {
 public:
  ChargingStatus();
  virtual ~ChargingStatus();

  ChargingStatus(const ChargingStatus& from);
  ChargingStatus(ChargingStatus&& from) noexcept
    : ChargingStatus() {
    *this = ::std::move(from);
  }

  inline ChargingStatus& operator=(const ChargingStatus& from) {
    CopyFrom(from);
    return *this;
  }
  inline ChargingStatus& operator=(ChargingStatus&& from) noexcept {
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
  static const ChargingStatus& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ChargingStatus* internal_default_instance() {
    return reinterpret_cast<const ChargingStatus*>(
               &_ChargingStatus_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ChargingStatus& a, ChargingStatus& b) {
    a.Swap(&b);
  }
  inline void Swap(ChargingStatus* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ChargingStatus* New() const final {
    return CreateMaybeMessage<ChargingStatus>(nullptr);
  }

  ChargingStatus* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ChargingStatus>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ChargingStatus& from);
  void MergeFrom(const ChargingStatus& from);
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
  void InternalSwap(ChargingStatus* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.charging_status.ChargingStatus";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kChargingConnectorStatusNuFieldNumber = 1062,
    kEvChargingIsInstalledNuFieldNumber = 1425,
  };
  // optional .pb.ap_vehstatesigprovider.charging_connector_status.ChargingConnectorStatus chargingConnectorStatus_nu = 1062;
  bool has_chargingconnectorstatus_nu() const;
  private:
  bool _internal_has_chargingconnectorstatus_nu() const;
  public:
  void clear_chargingconnectorstatus_nu();
  ::pb::ap_vehstatesigprovider::charging_connector_status::ChargingConnectorStatus chargingconnectorstatus_nu() const;
  void set_chargingconnectorstatus_nu(::pb::ap_vehstatesigprovider::charging_connector_status::ChargingConnectorStatus value);
  private:
  ::pb::ap_vehstatesigprovider::charging_connector_status::ChargingConnectorStatus _internal_chargingconnectorstatus_nu() const;
  void _internal_set_chargingconnectorstatus_nu(::pb::ap_vehstatesigprovider::charging_connector_status::ChargingConnectorStatus value);
  public:

  // optional bool ev_charging_is_installed_nu = 1425;
  bool has_ev_charging_is_installed_nu() const;
  private:
  bool _internal_has_ev_charging_is_installed_nu() const;
  public:
  void clear_ev_charging_is_installed_nu();
  bool ev_charging_is_installed_nu() const;
  void set_ev_charging_is_installed_nu(bool value);
  private:
  bool _internal_ev_charging_is_installed_nu() const;
  void _internal_set_ev_charging_is_installed_nu(bool value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int chargingconnectorstatus_nu_;
  bool ev_charging_is_installed_nu_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto;
};
// -------------------------------------------------------------------

class ChargingStatus_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port) */ {
 public:
  ChargingStatus_array_port();
  virtual ~ChargingStatus_array_port();

  ChargingStatus_array_port(const ChargingStatus_array_port& from);
  ChargingStatus_array_port(ChargingStatus_array_port&& from) noexcept
    : ChargingStatus_array_port() {
    *this = ::std::move(from);
  }

  inline ChargingStatus_array_port& operator=(const ChargingStatus_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline ChargingStatus_array_port& operator=(ChargingStatus_array_port&& from) noexcept {
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
  static const ChargingStatus_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ChargingStatus_array_port* internal_default_instance() {
    return reinterpret_cast<const ChargingStatus_array_port*>(
               &_ChargingStatus_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(ChargingStatus_array_port& a, ChargingStatus_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(ChargingStatus_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ChargingStatus_array_port* New() const final {
    return CreateMaybeMessage<ChargingStatus_array_port>(nullptr);
  }

  ChargingStatus_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ChargingStatus_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ChargingStatus_array_port& from);
  void MergeFrom(const ChargingStatus_array_port& from);
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
  void InternalSwap(ChargingStatus_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3831,
  };
  // repeated .pb.ap_vehstatesigprovider.charging_status.ChargingStatus data = 3831;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus >*
      mutable_data();
  private:
  const ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus& _internal_data(int index) const;
  ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus* _internal_add_data();
  public:
  const ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus& data(int index) const;
  ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus > data_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ChargingStatus

// optional bool ev_charging_is_installed_nu = 1425;
inline bool ChargingStatus::_internal_has_ev_charging_is_installed_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool ChargingStatus::has_ev_charging_is_installed_nu() const {
  return _internal_has_ev_charging_is_installed_nu();
}
inline void ChargingStatus::clear_ev_charging_is_installed_nu() {
  ev_charging_is_installed_nu_ = false;
  _has_bits_[0] &= ~0x00000002u;
}
inline bool ChargingStatus::_internal_ev_charging_is_installed_nu() const {
  return ev_charging_is_installed_nu_;
}
inline bool ChargingStatus::ev_charging_is_installed_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.charging_status.ChargingStatus.ev_charging_is_installed_nu)
  return _internal_ev_charging_is_installed_nu();
}
inline void ChargingStatus::_internal_set_ev_charging_is_installed_nu(bool value) {
  _has_bits_[0] |= 0x00000002u;
  ev_charging_is_installed_nu_ = value;
}
inline void ChargingStatus::set_ev_charging_is_installed_nu(bool value) {
  _internal_set_ev_charging_is_installed_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.charging_status.ChargingStatus.ev_charging_is_installed_nu)
}

// optional .pb.ap_vehstatesigprovider.charging_connector_status.ChargingConnectorStatus chargingConnectorStatus_nu = 1062;
inline bool ChargingStatus::_internal_has_chargingconnectorstatus_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool ChargingStatus::has_chargingconnectorstatus_nu() const {
  return _internal_has_chargingconnectorstatus_nu();
}
inline void ChargingStatus::clear_chargingconnectorstatus_nu() {
  chargingconnectorstatus_nu_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::pb::ap_vehstatesigprovider::charging_connector_status::ChargingConnectorStatus ChargingStatus::_internal_chargingconnectorstatus_nu() const {
  return static_cast< ::pb::ap_vehstatesigprovider::charging_connector_status::ChargingConnectorStatus >(chargingconnectorstatus_nu_);
}
inline ::pb::ap_vehstatesigprovider::charging_connector_status::ChargingConnectorStatus ChargingStatus::chargingconnectorstatus_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.charging_status.ChargingStatus.chargingConnectorStatus_nu)
  return _internal_chargingconnectorstatus_nu();
}
inline void ChargingStatus::_internal_set_chargingconnectorstatus_nu(::pb::ap_vehstatesigprovider::charging_connector_status::ChargingConnectorStatus value) {
  assert(::pb::ap_vehstatesigprovider::charging_connector_status::ChargingConnectorStatus_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  chargingconnectorstatus_nu_ = value;
}
inline void ChargingStatus::set_chargingconnectorstatus_nu(::pb::ap_vehstatesigprovider::charging_connector_status::ChargingConnectorStatus value) {
  _internal_set_chargingconnectorstatus_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.charging_status.ChargingStatus.chargingConnectorStatus_nu)
}

// -------------------------------------------------------------------

// ChargingStatus_array_port

// repeated .pb.ap_vehstatesigprovider.charging_status.ChargingStatus data = 3831;
inline int ChargingStatus_array_port::_internal_data_size() const {
  return data_.size();
}
inline int ChargingStatus_array_port::data_size() const {
  return _internal_data_size();
}
inline void ChargingStatus_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus* ChargingStatus_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus >*
ChargingStatus_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port.data)
  return &data_;
}
inline const ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus& ChargingStatus_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus& ChargingStatus_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus* ChargingStatus_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus* ChargingStatus_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus >&
ChargingStatus_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace charging_status
}  // namespace ap_vehstatesigprovider
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto
