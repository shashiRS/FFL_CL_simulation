// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/chassis_axle_height_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_2eproto

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
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_2eproto;
namespace pb {
namespace ap_vehstatesigprovider {
namespace chassis_axle_height_port {
class ChassisAxleHeightPort;
class ChassisAxleHeightPortDefaultTypeInternal;
extern ChassisAxleHeightPortDefaultTypeInternal _ChassisAxleHeightPort_default_instance_;
class ChassisAxleHeightPort_array_port;
class ChassisAxleHeightPort_array_portDefaultTypeInternal;
extern ChassisAxleHeightPort_array_portDefaultTypeInternal _ChassisAxleHeightPort_array_port_default_instance_;
}  // namespace chassis_axle_height_port
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort>(Arena*);
template<> ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort_array_port* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_vehstatesigprovider {
namespace chassis_axle_height_port {

// ===================================================================

class ChassisAxleHeightPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort) */ {
 public:
  ChassisAxleHeightPort();
  virtual ~ChassisAxleHeightPort();

  ChassisAxleHeightPort(const ChassisAxleHeightPort& from);
  ChassisAxleHeightPort(ChassisAxleHeightPort&& from) noexcept
    : ChassisAxleHeightPort() {
    *this = ::std::move(from);
  }

  inline ChassisAxleHeightPort& operator=(const ChassisAxleHeightPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline ChassisAxleHeightPort& operator=(ChassisAxleHeightPort&& from) noexcept {
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
  static const ChassisAxleHeightPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ChassisAxleHeightPort* internal_default_instance() {
    return reinterpret_cast<const ChassisAxleHeightPort*>(
               &_ChassisAxleHeightPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ChassisAxleHeightPort& a, ChassisAxleHeightPort& b) {
    a.Swap(&b);
  }
  inline void Swap(ChassisAxleHeightPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ChassisAxleHeightPort* New() const final {
    return CreateMaybeMessage<ChassisAxleHeightPort>(nullptr);
  }

  ChassisAxleHeightPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ChassisAxleHeightPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ChassisAxleHeightPort& from);
  void MergeFrom(const ChassisAxleHeightPort& from);
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
  void InternalSwap(ChassisAxleHeightPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kUiVersionNumberFieldNumber = 2124,
    kChassisAxleHeightFrontMmFieldNumber = 77,
    kChassisAxleHeightRearMmFieldNumber = 1224,
  };
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

  // optional float chassisAxleHeightFront_mm = 77;
  bool has_chassisaxleheightfront_mm() const;
  private:
  bool _internal_has_chassisaxleheightfront_mm() const;
  public:
  void clear_chassisaxleheightfront_mm();
  float chassisaxleheightfront_mm() const;
  void set_chassisaxleheightfront_mm(float value);
  private:
  float _internal_chassisaxleheightfront_mm() const;
  void _internal_set_chassisaxleheightfront_mm(float value);
  public:

  // optional float chassisAxleHeightRear_mm = 1224;
  bool has_chassisaxleheightrear_mm() const;
  private:
  bool _internal_has_chassisaxleheightrear_mm() const;
  public:
  void clear_chassisaxleheightrear_mm();
  float chassisaxleheightrear_mm() const;
  void set_chassisaxleheightrear_mm(float value);
  private:
  float _internal_chassisaxleheightrear_mm() const;
  void _internal_set_chassisaxleheightrear_mm(float value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  float chassisaxleheightfront_mm_;
  float chassisaxleheightrear_mm_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_2eproto;
};
// -------------------------------------------------------------------

class ChassisAxleHeightPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort_array_port) */ {
 public:
  ChassisAxleHeightPort_array_port();
  virtual ~ChassisAxleHeightPort_array_port();

  ChassisAxleHeightPort_array_port(const ChassisAxleHeightPort_array_port& from);
  ChassisAxleHeightPort_array_port(ChassisAxleHeightPort_array_port&& from) noexcept
    : ChassisAxleHeightPort_array_port() {
    *this = ::std::move(from);
  }

  inline ChassisAxleHeightPort_array_port& operator=(const ChassisAxleHeightPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline ChassisAxleHeightPort_array_port& operator=(ChassisAxleHeightPort_array_port&& from) noexcept {
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
  static const ChassisAxleHeightPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ChassisAxleHeightPort_array_port* internal_default_instance() {
    return reinterpret_cast<const ChassisAxleHeightPort_array_port*>(
               &_ChassisAxleHeightPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(ChassisAxleHeightPort_array_port& a, ChassisAxleHeightPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(ChassisAxleHeightPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ChassisAxleHeightPort_array_port* New() const final {
    return CreateMaybeMessage<ChassisAxleHeightPort_array_port>(nullptr);
  }

  ChassisAxleHeightPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ChassisAxleHeightPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ChassisAxleHeightPort_array_port& from);
  void MergeFrom(const ChassisAxleHeightPort_array_port& from);
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
  void InternalSwap(ChassisAxleHeightPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1088,
  };
  // repeated .pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort data = 1088;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort >*
      mutable_data();
  private:
  const ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort& _internal_data(int index) const;
  ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort* _internal_add_data();
  public:
  const ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort& data(int index) const;
  ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort > data_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ChassisAxleHeightPort

// optional uint32 uiVersionNumber = 2124;
inline bool ChassisAxleHeightPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool ChassisAxleHeightPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void ChassisAxleHeightPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ChassisAxleHeightPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ChassisAxleHeightPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void ChassisAxleHeightPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  uiversionnumber_ = value;
}
inline void ChassisAxleHeightPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool ChassisAxleHeightPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool ChassisAxleHeightPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& ChassisAxleHeightPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& ChassisAxleHeightPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* ChassisAxleHeightPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* ChassisAxleHeightPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* ChassisAxleHeightPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void ChassisAxleHeightPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort.sSigHeader)
}

// optional float chassisAxleHeightFront_mm = 77;
inline bool ChassisAxleHeightPort::_internal_has_chassisaxleheightfront_mm() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool ChassisAxleHeightPort::has_chassisaxleheightfront_mm() const {
  return _internal_has_chassisaxleheightfront_mm();
}
inline void ChassisAxleHeightPort::clear_chassisaxleheightfront_mm() {
  chassisaxleheightfront_mm_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float ChassisAxleHeightPort::_internal_chassisaxleheightfront_mm() const {
  return chassisaxleheightfront_mm_;
}
inline float ChassisAxleHeightPort::chassisaxleheightfront_mm() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort.chassisAxleHeightFront_mm)
  return _internal_chassisaxleheightfront_mm();
}
inline void ChassisAxleHeightPort::_internal_set_chassisaxleheightfront_mm(float value) {
  _has_bits_[0] |= 0x00000004u;
  chassisaxleheightfront_mm_ = value;
}
inline void ChassisAxleHeightPort::set_chassisaxleheightfront_mm(float value) {
  _internal_set_chassisaxleheightfront_mm(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort.chassisAxleHeightFront_mm)
}

// optional float chassisAxleHeightRear_mm = 1224;
inline bool ChassisAxleHeightPort::_internal_has_chassisaxleheightrear_mm() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool ChassisAxleHeightPort::has_chassisaxleheightrear_mm() const {
  return _internal_has_chassisaxleheightrear_mm();
}
inline void ChassisAxleHeightPort::clear_chassisaxleheightrear_mm() {
  chassisaxleheightrear_mm_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float ChassisAxleHeightPort::_internal_chassisaxleheightrear_mm() const {
  return chassisaxleheightrear_mm_;
}
inline float ChassisAxleHeightPort::chassisaxleheightrear_mm() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort.chassisAxleHeightRear_mm)
  return _internal_chassisaxleheightrear_mm();
}
inline void ChassisAxleHeightPort::_internal_set_chassisaxleheightrear_mm(float value) {
  _has_bits_[0] |= 0x00000008u;
  chassisaxleheightrear_mm_ = value;
}
inline void ChassisAxleHeightPort::set_chassisaxleheightrear_mm(float value) {
  _internal_set_chassisaxleheightrear_mm(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort.chassisAxleHeightRear_mm)
}

// -------------------------------------------------------------------

// ChassisAxleHeightPort_array_port

// repeated .pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort data = 1088;
inline int ChassisAxleHeightPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int ChassisAxleHeightPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void ChassisAxleHeightPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort* ChassisAxleHeightPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort >*
ChassisAxleHeightPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort_array_port.data)
  return &data_;
}
inline const ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort& ChassisAxleHeightPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort& ChassisAxleHeightPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort* ChassisAxleHeightPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort* ChassisAxleHeightPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::chassis_axle_height_port::ChassisAxleHeightPort >&
ChassisAxleHeightPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_vehstatesigprovider.chassis_axle_height_port.ChassisAxleHeightPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace chassis_axle_height_port
}  // namespace ap_vehstatesigprovider
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fchassis_5faxle_5fheight_5fport_2eproto
