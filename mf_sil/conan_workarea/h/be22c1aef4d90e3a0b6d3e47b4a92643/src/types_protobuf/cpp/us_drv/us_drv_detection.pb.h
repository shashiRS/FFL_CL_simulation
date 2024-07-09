// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_drv/us_drv_detection.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fdetection_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fdetection_2eproto

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
#include "us_drv/us_drv_detection_type.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fdetection_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fdrv_2fus_5fdrv_5fdetection_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fdrv_2fus_5fdrv_5fdetection_2eproto;
namespace pb {
namespace us_drv {
namespace us_drv_detection {
class UsDrvDetection;
class UsDrvDetectionDefaultTypeInternal;
extern UsDrvDetectionDefaultTypeInternal _UsDrvDetection_default_instance_;
class UsDrvDetection_array_port;
class UsDrvDetection_array_portDefaultTypeInternal;
extern UsDrvDetection_array_portDefaultTypeInternal _UsDrvDetection_array_port_default_instance_;
}  // namespace us_drv_detection
}  // namespace us_drv
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_drv::us_drv_detection::UsDrvDetection* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_detection::UsDrvDetection>(Arena*);
template<> ::pb::us_drv::us_drv_detection::UsDrvDetection_array_port* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_detection::UsDrvDetection_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_drv {
namespace us_drv_detection {

// ===================================================================

class UsDrvDetection :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_detection.UsDrvDetection) */ {
 public:
  UsDrvDetection();
  virtual ~UsDrvDetection();

  UsDrvDetection(const UsDrvDetection& from);
  UsDrvDetection(UsDrvDetection&& from) noexcept
    : UsDrvDetection() {
    *this = ::std::move(from);
  }

  inline UsDrvDetection& operator=(const UsDrvDetection& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvDetection& operator=(UsDrvDetection&& from) noexcept {
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
  static const UsDrvDetection& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvDetection* internal_default_instance() {
    return reinterpret_cast<const UsDrvDetection*>(
               &_UsDrvDetection_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsDrvDetection& a, UsDrvDetection& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvDetection* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvDetection* New() const final {
    return CreateMaybeMessage<UsDrvDetection>(nullptr);
  }

  UsDrvDetection* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvDetection>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvDetection& from);
  void MergeFrom(const UsDrvDetection& from);
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
  void InternalSwap(UsDrvDetection* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_detection.UsDrvDetection";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fdetection_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fdetection_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSyncCntMsFieldNumber = 1171,
    kSensorIdFieldNumber = 1529,
    kAmplitudeFieldNumber = 2603,
    kRelEcuTimestampUsFieldNumber = 2612,
    kPhaseDerivativeFieldNumber = 3592,
    kSensorTimestampUsFieldNumber = 3782,
    kDetectionTypeFieldNumber = 2224,
  };
  // optional uint32 syncCnt_ms = 1171;
  bool has_synccnt_ms() const;
  private:
  bool _internal_has_synccnt_ms() const;
  public:
  void clear_synccnt_ms();
  ::PROTOBUF_NAMESPACE_ID::uint32 synccnt_ms() const;
  void set_synccnt_ms(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_synccnt_ms() const;
  void _internal_set_synccnt_ms(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 sensorId = 1529;
  bool has_sensorid() const;
  private:
  bool _internal_has_sensorid() const;
  public:
  void clear_sensorid();
  ::PROTOBUF_NAMESPACE_ID::uint32 sensorid() const;
  void set_sensorid(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_sensorid() const;
  void _internal_set_sensorid(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 amplitude = 2603;
  bool has_amplitude() const;
  private:
  bool _internal_has_amplitude() const;
  public:
  void clear_amplitude();
  ::PROTOBUF_NAMESPACE_ID::uint32 amplitude() const;
  void set_amplitude(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_amplitude() const;
  void _internal_set_amplitude(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional sint32 relEcuTimestamp_us = 2612;
  bool has_relecutimestamp_us() const;
  private:
  bool _internal_has_relecutimestamp_us() const;
  public:
  void clear_relecutimestamp_us();
  ::PROTOBUF_NAMESPACE_ID::int32 relecutimestamp_us() const;
  void set_relecutimestamp_us(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_relecutimestamp_us() const;
  void _internal_set_relecutimestamp_us(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // optional sint32 phaseDerivative = 3592;
  bool has_phasederivative() const;
  private:
  bool _internal_has_phasederivative() const;
  public:
  void clear_phasederivative();
  ::PROTOBUF_NAMESPACE_ID::int32 phasederivative() const;
  void set_phasederivative(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_phasederivative() const;
  void _internal_set_phasederivative(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // optional uint32 sensorTimestamp_us = 3782;
  bool has_sensortimestamp_us() const;
  private:
  bool _internal_has_sensortimestamp_us() const;
  public:
  void clear_sensortimestamp_us();
  ::PROTOBUF_NAMESPACE_ID::uint32 sensortimestamp_us() const;
  void set_sensortimestamp_us(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_sensortimestamp_us() const;
  void _internal_set_sensortimestamp_us(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional .pb.us_drv.us_drv_detection_type.UsDrvDetectionType detectionType = 2224;
  bool has_detectiontype() const;
  private:
  bool _internal_has_detectiontype() const;
  public:
  void clear_detectiontype();
  ::pb::us_drv::us_drv_detection_type::UsDrvDetectionType detectiontype() const;
  void set_detectiontype(::pb::us_drv::us_drv_detection_type::UsDrvDetectionType value);
  private:
  ::pb::us_drv::us_drv_detection_type::UsDrvDetectionType _internal_detectiontype() const;
  void _internal_set_detectiontype(::pb::us_drv::us_drv_detection_type::UsDrvDetectionType value);
  public:

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_detection.UsDrvDetection)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 synccnt_ms_;
  ::PROTOBUF_NAMESPACE_ID::uint32 sensorid_;
  ::PROTOBUF_NAMESPACE_ID::uint32 amplitude_;
  ::PROTOBUF_NAMESPACE_ID::int32 relecutimestamp_us_;
  ::PROTOBUF_NAMESPACE_ID::int32 phasederivative_;
  ::PROTOBUF_NAMESPACE_ID::uint32 sensortimestamp_us_;
  int detectiontype_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fdetection_2eproto;
};
// -------------------------------------------------------------------

class UsDrvDetection_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_detection.UsDrvDetection_array_port) */ {
 public:
  UsDrvDetection_array_port();
  virtual ~UsDrvDetection_array_port();

  UsDrvDetection_array_port(const UsDrvDetection_array_port& from);
  UsDrvDetection_array_port(UsDrvDetection_array_port&& from) noexcept
    : UsDrvDetection_array_port() {
    *this = ::std::move(from);
  }

  inline UsDrvDetection_array_port& operator=(const UsDrvDetection_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvDetection_array_port& operator=(UsDrvDetection_array_port&& from) noexcept {
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
  static const UsDrvDetection_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvDetection_array_port* internal_default_instance() {
    return reinterpret_cast<const UsDrvDetection_array_port*>(
               &_UsDrvDetection_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsDrvDetection_array_port& a, UsDrvDetection_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvDetection_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvDetection_array_port* New() const final {
    return CreateMaybeMessage<UsDrvDetection_array_port>(nullptr);
  }

  UsDrvDetection_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvDetection_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvDetection_array_port& from);
  void MergeFrom(const UsDrvDetection_array_port& from);
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
  void InternalSwap(UsDrvDetection_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_detection.UsDrvDetection_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fdetection_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fdetection_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3565,
  };
  // repeated .pb.us_drv.us_drv_detection.UsDrvDetection data = 3565;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_drv::us_drv_detection::UsDrvDetection* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_detection::UsDrvDetection >*
      mutable_data();
  private:
  const ::pb::us_drv::us_drv_detection::UsDrvDetection& _internal_data(int index) const;
  ::pb::us_drv::us_drv_detection::UsDrvDetection* _internal_add_data();
  public:
  const ::pb::us_drv::us_drv_detection::UsDrvDetection& data(int index) const;
  ::pb::us_drv::us_drv_detection::UsDrvDetection* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_detection::UsDrvDetection >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_detection.UsDrvDetection_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_detection::UsDrvDetection > data_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fdetection_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsDrvDetection

// optional uint32 sensorId = 1529;
inline bool UsDrvDetection::_internal_has_sensorid() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool UsDrvDetection::has_sensorid() const {
  return _internal_has_sensorid();
}
inline void UsDrvDetection::clear_sensorid() {
  sensorid_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDetection::_internal_sensorid() const {
  return sensorid_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDetection::sensorid() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_detection.UsDrvDetection.sensorId)
  return _internal_sensorid();
}
inline void UsDrvDetection::_internal_set_sensorid(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  sensorid_ = value;
}
inline void UsDrvDetection::set_sensorid(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_sensorid(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_detection.UsDrvDetection.sensorId)
}

// optional .pb.us_drv.us_drv_detection_type.UsDrvDetectionType detectionType = 2224;
inline bool UsDrvDetection::_internal_has_detectiontype() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool UsDrvDetection::has_detectiontype() const {
  return _internal_has_detectiontype();
}
inline void UsDrvDetection::clear_detectiontype() {
  detectiontype_ = 15;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::pb::us_drv::us_drv_detection_type::UsDrvDetectionType UsDrvDetection::_internal_detectiontype() const {
  return static_cast< ::pb::us_drv::us_drv_detection_type::UsDrvDetectionType >(detectiontype_);
}
inline ::pb::us_drv::us_drv_detection_type::UsDrvDetectionType UsDrvDetection::detectiontype() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_detection.UsDrvDetection.detectionType)
  return _internal_detectiontype();
}
inline void UsDrvDetection::_internal_set_detectiontype(::pb::us_drv::us_drv_detection_type::UsDrvDetectionType value) {
  assert(::pb::us_drv::us_drv_detection_type::UsDrvDetectionType_IsValid(value));
  _has_bits_[0] |= 0x00000040u;
  detectiontype_ = value;
}
inline void UsDrvDetection::set_detectiontype(::pb::us_drv::us_drv_detection_type::UsDrvDetectionType value) {
  _internal_set_detectiontype(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_detection.UsDrvDetection.detectionType)
}

// optional uint32 amplitude = 2603;
inline bool UsDrvDetection::_internal_has_amplitude() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool UsDrvDetection::has_amplitude() const {
  return _internal_has_amplitude();
}
inline void UsDrvDetection::clear_amplitude() {
  amplitude_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDetection::_internal_amplitude() const {
  return amplitude_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDetection::amplitude() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_detection.UsDrvDetection.amplitude)
  return _internal_amplitude();
}
inline void UsDrvDetection::_internal_set_amplitude(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  amplitude_ = value;
}
inline void UsDrvDetection::set_amplitude(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_amplitude(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_detection.UsDrvDetection.amplitude)
}

// optional sint32 phaseDerivative = 3592;
inline bool UsDrvDetection::_internal_has_phasederivative() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool UsDrvDetection::has_phasederivative() const {
  return _internal_has_phasederivative();
}
inline void UsDrvDetection::clear_phasederivative() {
  phasederivative_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 UsDrvDetection::_internal_phasederivative() const {
  return phasederivative_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 UsDrvDetection::phasederivative() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_detection.UsDrvDetection.phaseDerivative)
  return _internal_phasederivative();
}
inline void UsDrvDetection::_internal_set_phasederivative(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000010u;
  phasederivative_ = value;
}
inline void UsDrvDetection::set_phasederivative(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_phasederivative(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_detection.UsDrvDetection.phaseDerivative)
}

// optional uint32 syncCnt_ms = 1171;
inline bool UsDrvDetection::_internal_has_synccnt_ms() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool UsDrvDetection::has_synccnt_ms() const {
  return _internal_has_synccnt_ms();
}
inline void UsDrvDetection::clear_synccnt_ms() {
  synccnt_ms_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDetection::_internal_synccnt_ms() const {
  return synccnt_ms_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDetection::synccnt_ms() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_detection.UsDrvDetection.syncCnt_ms)
  return _internal_synccnt_ms();
}
inline void UsDrvDetection::_internal_set_synccnt_ms(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  synccnt_ms_ = value;
}
inline void UsDrvDetection::set_synccnt_ms(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_synccnt_ms(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_detection.UsDrvDetection.syncCnt_ms)
}

// optional uint32 sensorTimestamp_us = 3782;
inline bool UsDrvDetection::_internal_has_sensortimestamp_us() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool UsDrvDetection::has_sensortimestamp_us() const {
  return _internal_has_sensortimestamp_us();
}
inline void UsDrvDetection::clear_sensortimestamp_us() {
  sensortimestamp_us_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDetection::_internal_sensortimestamp_us() const {
  return sensortimestamp_us_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDetection::sensortimestamp_us() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_detection.UsDrvDetection.sensorTimestamp_us)
  return _internal_sensortimestamp_us();
}
inline void UsDrvDetection::_internal_set_sensortimestamp_us(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  sensortimestamp_us_ = value;
}
inline void UsDrvDetection::set_sensortimestamp_us(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_sensortimestamp_us(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_detection.UsDrvDetection.sensorTimestamp_us)
}

// optional sint32 relEcuTimestamp_us = 2612;
inline bool UsDrvDetection::_internal_has_relecutimestamp_us() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool UsDrvDetection::has_relecutimestamp_us() const {
  return _internal_has_relecutimestamp_us();
}
inline void UsDrvDetection::clear_relecutimestamp_us() {
  relecutimestamp_us_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 UsDrvDetection::_internal_relecutimestamp_us() const {
  return relecutimestamp_us_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 UsDrvDetection::relecutimestamp_us() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_detection.UsDrvDetection.relEcuTimestamp_us)
  return _internal_relecutimestamp_us();
}
inline void UsDrvDetection::_internal_set_relecutimestamp_us(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000008u;
  relecutimestamp_us_ = value;
}
inline void UsDrvDetection::set_relecutimestamp_us(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_relecutimestamp_us(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_detection.UsDrvDetection.relEcuTimestamp_us)
}

// -------------------------------------------------------------------

// UsDrvDetection_array_port

// repeated .pb.us_drv.us_drv_detection.UsDrvDetection data = 3565;
inline int UsDrvDetection_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsDrvDetection_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsDrvDetection_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_drv::us_drv_detection::UsDrvDetection* UsDrvDetection_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_detection.UsDrvDetection_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_detection::UsDrvDetection >*
UsDrvDetection_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_detection.UsDrvDetection_array_port.data)
  return &data_;
}
inline const ::pb::us_drv::us_drv_detection::UsDrvDetection& UsDrvDetection_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_drv::us_drv_detection::UsDrvDetection& UsDrvDetection_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_detection.UsDrvDetection_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_drv::us_drv_detection::UsDrvDetection* UsDrvDetection_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_drv::us_drv_detection::UsDrvDetection* UsDrvDetection_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_detection.UsDrvDetection_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_detection::UsDrvDetection >&
UsDrvDetection_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_detection.UsDrvDetection_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_drv_detection
}  // namespace us_drv
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fdetection_2eproto