// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/chassis_axle_height_port_interface_version.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_5finterface_5fversion_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_5finterface_5fversion_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_5finterface_5fversion_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_5finterface_5fversion_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_5finterface_5fversion_2eproto;
namespace pb {
namespace ap_vehstatesigprovider {
namespace chassis_axle_height_port_interface_version {
class ChassisAxleHeightPort_InterfaceVersion;
class ChassisAxleHeightPort_InterfaceVersionDefaultTypeInternal;
extern ChassisAxleHeightPort_InterfaceVersionDefaultTypeInternal _ChassisAxleHeightPort_InterfaceVersion_default_instance_;
class ChassisAxleHeightPort_InterfaceVersion_array_port;
class ChassisAxleHeightPort_InterfaceVersion_array_portDefaultTypeInternal;
extern ChassisAxleHeightPort_InterfaceVersion_array_portDefaultTypeInternal _ChassisAxleHeightPort_InterfaceVersion_array_port_default_instance_;
}  // namespace chassis_axle_height_port_interface_version
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion>(Arena*);
template<> ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion_array_port* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_vehstatesigprovider {
namespace chassis_axle_height_port_interface_version {

// ===================================================================

class ChassisAxleHeightPort_InterfaceVersion :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion) */ {
 public:
  ChassisAxleHeightPort_InterfaceVersion();
  virtual ~ChassisAxleHeightPort_InterfaceVersion();

  ChassisAxleHeightPort_InterfaceVersion(const ChassisAxleHeightPort_InterfaceVersion& from);
  ChassisAxleHeightPort_InterfaceVersion(ChassisAxleHeightPort_InterfaceVersion&& from) noexcept
    : ChassisAxleHeightPort_InterfaceVersion() {
    *this = ::std::move(from);
  }

  inline ChassisAxleHeightPort_InterfaceVersion& operator=(const ChassisAxleHeightPort_InterfaceVersion& from) {
    CopyFrom(from);
    return *this;
  }
  inline ChassisAxleHeightPort_InterfaceVersion& operator=(ChassisAxleHeightPort_InterfaceVersion&& from) noexcept {
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
  static const ChassisAxleHeightPort_InterfaceVersion& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ChassisAxleHeightPort_InterfaceVersion* internal_default_instance() {
    return reinterpret_cast<const ChassisAxleHeightPort_InterfaceVersion*>(
               &_ChassisAxleHeightPort_InterfaceVersion_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ChassisAxleHeightPort_InterfaceVersion& a, ChassisAxleHeightPort_InterfaceVersion& b) {
    a.Swap(&b);
  }
  inline void Swap(ChassisAxleHeightPort_InterfaceVersion* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ChassisAxleHeightPort_InterfaceVersion* New() const final {
    return CreateMaybeMessage<ChassisAxleHeightPort_InterfaceVersion>(nullptr);
  }

  ChassisAxleHeightPort_InterfaceVersion* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ChassisAxleHeightPort_InterfaceVersion>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ChassisAxleHeightPort_InterfaceVersion& from);
  void MergeFrom(const ChassisAxleHeightPort_InterfaceVersion& from);
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
  void InternalSwap(ChassisAxleHeightPort_InterfaceVersion* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_5finterface_5fversion_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_5finterface_5fversion_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kChassisAxleHeightPortVERSIONFieldNumber = 1088,
  };
  // optional uint32 ChassisAxleHeightPort_VERSION = 1088;
  bool has_chassisaxleheightport_version() const;
  private:
  bool _internal_has_chassisaxleheightport_version() const;
  public:
  void clear_chassisaxleheightport_version();
  ::PROTOBUF_NAMESPACE_ID::uint32 chassisaxleheightport_version() const;
  void set_chassisaxleheightport_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_chassisaxleheightport_version() const;
  void _internal_set_chassisaxleheightport_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 chassisaxleheightport_version_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_5finterface_5fversion_2eproto;
};
// -------------------------------------------------------------------

class ChassisAxleHeightPort_InterfaceVersion_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion_array_port) */ {
 public:
  ChassisAxleHeightPort_InterfaceVersion_array_port();
  virtual ~ChassisAxleHeightPort_InterfaceVersion_array_port();

  ChassisAxleHeightPort_InterfaceVersion_array_port(const ChassisAxleHeightPort_InterfaceVersion_array_port& from);
  ChassisAxleHeightPort_InterfaceVersion_array_port(ChassisAxleHeightPort_InterfaceVersion_array_port&& from) noexcept
    : ChassisAxleHeightPort_InterfaceVersion_array_port() {
    *this = ::std::move(from);
  }

  inline ChassisAxleHeightPort_InterfaceVersion_array_port& operator=(const ChassisAxleHeightPort_InterfaceVersion_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline ChassisAxleHeightPort_InterfaceVersion_array_port& operator=(ChassisAxleHeightPort_InterfaceVersion_array_port&& from) noexcept {
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
  static const ChassisAxleHeightPort_InterfaceVersion_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ChassisAxleHeightPort_InterfaceVersion_array_port* internal_default_instance() {
    return reinterpret_cast<const ChassisAxleHeightPort_InterfaceVersion_array_port*>(
               &_ChassisAxleHeightPort_InterfaceVersion_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(ChassisAxleHeightPort_InterfaceVersion_array_port& a, ChassisAxleHeightPort_InterfaceVersion_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(ChassisAxleHeightPort_InterfaceVersion_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ChassisAxleHeightPort_InterfaceVersion_array_port* New() const final {
    return CreateMaybeMessage<ChassisAxleHeightPort_InterfaceVersion_array_port>(nullptr);
  }

  ChassisAxleHeightPort_InterfaceVersion_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ChassisAxleHeightPort_InterfaceVersion_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ChassisAxleHeightPort_InterfaceVersion_array_port& from);
  void MergeFrom(const ChassisAxleHeightPort_InterfaceVersion_array_port& from);
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
  void InternalSwap(ChassisAxleHeightPort_InterfaceVersion_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_5finterface_5fversion_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_5finterface_5fversion_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 573,
  };
  // repeated .pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion data = 573;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion >*
      mutable_data();
  private:
  const ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion& _internal_data(int index) const;
  ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion* _internal_add_data();
  public:
  const ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion& data(int index) const;
  ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion > data_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_5finterface_5fversion_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ChassisAxleHeightPort_InterfaceVersion

// optional uint32 ChassisAxleHeightPort_VERSION = 1088;
inline bool ChassisAxleHeightPort_InterfaceVersion::_internal_has_chassisaxleheightport_version() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool ChassisAxleHeightPort_InterfaceVersion::has_chassisaxleheightport_version() const {
  return _internal_has_chassisaxleheightport_version();
}
inline void ChassisAxleHeightPort_InterfaceVersion::clear_chassisaxleheightport_version() {
  chassisaxleheightport_version_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ChassisAxleHeightPort_InterfaceVersion::_internal_chassisaxleheightport_version() const {
  return chassisaxleheightport_version_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ChassisAxleHeightPort_InterfaceVersion::chassisaxleheightport_version() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion.ChassisAxleHeightPort_VERSION)
  return _internal_chassisaxleheightport_version();
}
inline void ChassisAxleHeightPort_InterfaceVersion::_internal_set_chassisaxleheightport_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  chassisaxleheightport_version_ = value;
}
inline void ChassisAxleHeightPort_InterfaceVersion::set_chassisaxleheightport_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_chassisaxleheightport_version(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion.ChassisAxleHeightPort_VERSION)
}

// -------------------------------------------------------------------

// ChassisAxleHeightPort_InterfaceVersion_array_port

// repeated .pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion data = 573;
inline int ChassisAxleHeightPort_InterfaceVersion_array_port::_internal_data_size() const {
  return data_.size();
}
inline int ChassisAxleHeightPort_InterfaceVersion_array_port::data_size() const {
  return _internal_data_size();
}
inline void ChassisAxleHeightPort_InterfaceVersion_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion* ChassisAxleHeightPort_InterfaceVersion_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion >*
ChassisAxleHeightPort_InterfaceVersion_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion_array_port.data)
  return &data_;
}
inline const ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion& ChassisAxleHeightPort_InterfaceVersion_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion& ChassisAxleHeightPort_InterfaceVersion_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion* ChassisAxleHeightPort_InterfaceVersion_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion* ChassisAxleHeightPort_InterfaceVersion_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::chassis_axle_height_port_interface_version::ChassisAxleHeightPort_InterfaceVersion >&
ChassisAxleHeightPort_InterfaceVersion_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_vehstatesigprovider.chassis_axle_height_port_interface_version.ChassisAxleHeightPort_InterfaceVersion_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace chassis_axle_height_port_interface_version
}  // namespace ap_vehstatesigprovider
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_5finterface_5fversion_2eproto
