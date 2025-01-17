// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_drv/us_drv_diag_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto

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
#include "us_drv/us_drv_variant_data.pb.h"
#include "us_drv/us_drv_sw_errors.pb.h"
#include "us_drv/us_drv_asic_errors.pb.h"
#include "us_drv/us_drv_sensor_errors.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto;
namespace pb {
namespace us_drv {
namespace us_drv_diag_port {
class UsDrvDiagPort;
class UsDrvDiagPortDefaultTypeInternal;
extern UsDrvDiagPortDefaultTypeInternal _UsDrvDiagPort_default_instance_;
class UsDrvDiagPort_array_port;
class UsDrvDiagPort_array_portDefaultTypeInternal;
extern UsDrvDiagPort_array_portDefaultTypeInternal _UsDrvDiagPort_array_port_default_instance_;
}  // namespace us_drv_diag_port
}  // namespace us_drv
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_diag_port::UsDrvDiagPort>(Arena*);
template<> ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort_array_port* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_diag_port::UsDrvDiagPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_drv {
namespace us_drv_diag_port {

// ===================================================================

class UsDrvDiagPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_diag_port.UsDrvDiagPort) */ {
 public:
  UsDrvDiagPort();
  virtual ~UsDrvDiagPort();

  UsDrvDiagPort(const UsDrvDiagPort& from);
  UsDrvDiagPort(UsDrvDiagPort&& from) noexcept
    : UsDrvDiagPort() {
    *this = ::std::move(from);
  }

  inline UsDrvDiagPort& operator=(const UsDrvDiagPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvDiagPort& operator=(UsDrvDiagPort&& from) noexcept {
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
  static const UsDrvDiagPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvDiagPort* internal_default_instance() {
    return reinterpret_cast<const UsDrvDiagPort*>(
               &_UsDrvDiagPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsDrvDiagPort& a, UsDrvDiagPort& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvDiagPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvDiagPort* New() const final {
    return CreateMaybeMessage<UsDrvDiagPort>(nullptr);
  }

  UsDrvDiagPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvDiagPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvDiagPort& from);
  void MergeFrom(const UsDrvDiagPort& from);
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
  void InternalSwap(UsDrvDiagPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_diag_port.UsDrvDiagPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSensorErrorsFieldNumber = 155,
    kAsicErrorsFieldNumber = 3675,
    kSSigHeaderFieldNumber = 1033,
    kUsDriverVariantDataFieldNumber = 2993,
    kUsDriverSwErrorsFieldNumber = 3589,
    kUiVersionNumberFieldNumber = 2124,
  };
  // repeated .pb.us_drv.us_drv_sensor_errors.UsDrvSensorErrors sensorErrors = 155;
  int sensorerrors_size() const;
  private:
  int _internal_sensorerrors_size() const;
  public:
  void clear_sensorerrors();
  ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors* mutable_sensorerrors(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors >*
      mutable_sensorerrors();
  private:
  const ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors& _internal_sensorerrors(int index) const;
  ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors* _internal_add_sensorerrors();
  public:
  const ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors& sensorerrors(int index) const;
  ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors* add_sensorerrors();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors >&
      sensorerrors() const;

  // repeated .pb.us_drv.us_drv_asic_errors.UsDrvAsicErrors asicErrors = 3675;
  int asicerrors_size() const;
  private:
  int _internal_asicerrors_size() const;
  public:
  void clear_asicerrors();
  ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors* mutable_asicerrors(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors >*
      mutable_asicerrors();
  private:
  const ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors& _internal_asicerrors(int index) const;
  ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors* _internal_add_asicerrors();
  public:
  const ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors& asicerrors(int index) const;
  ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors* add_asicerrors();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors >&
      asicerrors() const;

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

  // optional .pb.us_drv.us_drv_variant_data.UsDrvVariantData usDriverVariantData = 2993;
  bool has_usdrivervariantdata() const;
  private:
  bool _internal_has_usdrivervariantdata() const;
  public:
  void clear_usdrivervariantdata();
  const ::pb::us_drv::us_drv_variant_data::UsDrvVariantData& usdrivervariantdata() const;
  ::pb::us_drv::us_drv_variant_data::UsDrvVariantData* release_usdrivervariantdata();
  ::pb::us_drv::us_drv_variant_data::UsDrvVariantData* mutable_usdrivervariantdata();
  void set_allocated_usdrivervariantdata(::pb::us_drv::us_drv_variant_data::UsDrvVariantData* usdrivervariantdata);
  private:
  const ::pb::us_drv::us_drv_variant_data::UsDrvVariantData& _internal_usdrivervariantdata() const;
  ::pb::us_drv::us_drv_variant_data::UsDrvVariantData* _internal_mutable_usdrivervariantdata();
  public:

  // optional .pb.us_drv.us_drv_sw_errors.UsDrvSwErrors usDriverSwErrors = 3589;
  bool has_usdriverswerrors() const;
  private:
  bool _internal_has_usdriverswerrors() const;
  public:
  void clear_usdriverswerrors();
  const ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors& usdriverswerrors() const;
  ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors* release_usdriverswerrors();
  ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors* mutable_usdriverswerrors();
  void set_allocated_usdriverswerrors(::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors* usdriverswerrors);
  private:
  const ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors& _internal_usdriverswerrors() const;
  ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors* _internal_mutable_usdriverswerrors();
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

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors > sensorerrors_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors > asicerrors_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::pb::us_drv::us_drv_variant_data::UsDrvVariantData* usdrivervariantdata_;
  ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors* usdriverswerrors_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto;
};
// -------------------------------------------------------------------

class UsDrvDiagPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port) */ {
 public:
  UsDrvDiagPort_array_port();
  virtual ~UsDrvDiagPort_array_port();

  UsDrvDiagPort_array_port(const UsDrvDiagPort_array_port& from);
  UsDrvDiagPort_array_port(UsDrvDiagPort_array_port&& from) noexcept
    : UsDrvDiagPort_array_port() {
    *this = ::std::move(from);
  }

  inline UsDrvDiagPort_array_port& operator=(const UsDrvDiagPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvDiagPort_array_port& operator=(UsDrvDiagPort_array_port&& from) noexcept {
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
  static const UsDrvDiagPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvDiagPort_array_port* internal_default_instance() {
    return reinterpret_cast<const UsDrvDiagPort_array_port*>(
               &_UsDrvDiagPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsDrvDiagPort_array_port& a, UsDrvDiagPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvDiagPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvDiagPort_array_port* New() const final {
    return CreateMaybeMessage<UsDrvDiagPort_array_port>(nullptr);
  }

  UsDrvDiagPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvDiagPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvDiagPort_array_port& from);
  void MergeFrom(const UsDrvDiagPort_array_port& from);
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
  void InternalSwap(UsDrvDiagPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2394,
  };
  // repeated .pb.us_drv.us_drv_diag_port.UsDrvDiagPort data = 2394;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort >*
      mutable_data();
  private:
  const ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort& _internal_data(int index) const;
  ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort* _internal_add_data();
  public:
  const ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort& data(int index) const;
  ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort > data_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsDrvDiagPort

// optional uint32 uiVersionNumber = 2124;
inline bool UsDrvDiagPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool UsDrvDiagPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void UsDrvDiagPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDiagPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDiagPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void UsDrvDiagPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  uiversionnumber_ = value;
}
inline void UsDrvDiagPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool UsDrvDiagPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool UsDrvDiagPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& UsDrvDiagPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& UsDrvDiagPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* UsDrvDiagPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* UsDrvDiagPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* UsDrvDiagPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void UsDrvDiagPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.sSigHeader)
}

// optional .pb.us_drv.us_drv_variant_data.UsDrvVariantData usDriverVariantData = 2993;
inline bool UsDrvDiagPort::_internal_has_usdrivervariantdata() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || usdrivervariantdata_ != nullptr);
  return value;
}
inline bool UsDrvDiagPort::has_usdrivervariantdata() const {
  return _internal_has_usdrivervariantdata();
}
inline const ::pb::us_drv::us_drv_variant_data::UsDrvVariantData& UsDrvDiagPort::_internal_usdrivervariantdata() const {
  const ::pb::us_drv::us_drv_variant_data::UsDrvVariantData* p = usdrivervariantdata_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::us_drv::us_drv_variant_data::UsDrvVariantData*>(
      &::pb::us_drv::us_drv_variant_data::_UsDrvVariantData_default_instance_);
}
inline const ::pb::us_drv::us_drv_variant_data::UsDrvVariantData& UsDrvDiagPort::usdrivervariantdata() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.usDriverVariantData)
  return _internal_usdrivervariantdata();
}
inline ::pb::us_drv::us_drv_variant_data::UsDrvVariantData* UsDrvDiagPort::release_usdrivervariantdata() {
  // @@protoc_insertion_point(field_release:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.usDriverVariantData)
  _has_bits_[0] &= ~0x00000002u;
  ::pb::us_drv::us_drv_variant_data::UsDrvVariantData* temp = usdrivervariantdata_;
  usdrivervariantdata_ = nullptr;
  return temp;
}
inline ::pb::us_drv::us_drv_variant_data::UsDrvVariantData* UsDrvDiagPort::_internal_mutable_usdrivervariantdata() {
  _has_bits_[0] |= 0x00000002u;
  if (usdrivervariantdata_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::us_drv::us_drv_variant_data::UsDrvVariantData>(GetArenaNoVirtual());
    usdrivervariantdata_ = p;
  }
  return usdrivervariantdata_;
}
inline ::pb::us_drv::us_drv_variant_data::UsDrvVariantData* UsDrvDiagPort::mutable_usdrivervariantdata() {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.usDriverVariantData)
  return _internal_mutable_usdrivervariantdata();
}
inline void UsDrvDiagPort::set_allocated_usdrivervariantdata(::pb::us_drv::us_drv_variant_data::UsDrvVariantData* usdrivervariantdata) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(usdrivervariantdata_);
  }
  if (usdrivervariantdata) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      usdrivervariantdata = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, usdrivervariantdata, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  usdrivervariantdata_ = usdrivervariantdata;
  // @@protoc_insertion_point(field_set_allocated:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.usDriverVariantData)
}

// optional .pb.us_drv.us_drv_sw_errors.UsDrvSwErrors usDriverSwErrors = 3589;
inline bool UsDrvDiagPort::_internal_has_usdriverswerrors() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  PROTOBUF_ASSUME(!value || usdriverswerrors_ != nullptr);
  return value;
}
inline bool UsDrvDiagPort::has_usdriverswerrors() const {
  return _internal_has_usdriverswerrors();
}
inline const ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors& UsDrvDiagPort::_internal_usdriverswerrors() const {
  const ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors* p = usdriverswerrors_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors*>(
      &::pb::us_drv::us_drv_sw_errors::_UsDrvSwErrors_default_instance_);
}
inline const ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors& UsDrvDiagPort::usdriverswerrors() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.usDriverSwErrors)
  return _internal_usdriverswerrors();
}
inline ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors* UsDrvDiagPort::release_usdriverswerrors() {
  // @@protoc_insertion_point(field_release:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.usDriverSwErrors)
  _has_bits_[0] &= ~0x00000004u;
  ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors* temp = usdriverswerrors_;
  usdriverswerrors_ = nullptr;
  return temp;
}
inline ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors* UsDrvDiagPort::_internal_mutable_usdriverswerrors() {
  _has_bits_[0] |= 0x00000004u;
  if (usdriverswerrors_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors>(GetArenaNoVirtual());
    usdriverswerrors_ = p;
  }
  return usdriverswerrors_;
}
inline ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors* UsDrvDiagPort::mutable_usdriverswerrors() {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.usDriverSwErrors)
  return _internal_mutable_usdriverswerrors();
}
inline void UsDrvDiagPort::set_allocated_usdriverswerrors(::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors* usdriverswerrors) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(usdriverswerrors_);
  }
  if (usdriverswerrors) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      usdriverswerrors = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, usdriverswerrors, submessage_arena);
    }
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  usdriverswerrors_ = usdriverswerrors;
  // @@protoc_insertion_point(field_set_allocated:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.usDriverSwErrors)
}

// repeated .pb.us_drv.us_drv_asic_errors.UsDrvAsicErrors asicErrors = 3675;
inline int UsDrvDiagPort::_internal_asicerrors_size() const {
  return asicerrors_.size();
}
inline int UsDrvDiagPort::asicerrors_size() const {
  return _internal_asicerrors_size();
}
inline ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors* UsDrvDiagPort::mutable_asicerrors(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.asicErrors)
  return asicerrors_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors >*
UsDrvDiagPort::mutable_asicerrors() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.asicErrors)
  return &asicerrors_;
}
inline const ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors& UsDrvDiagPort::_internal_asicerrors(int index) const {
  return asicerrors_.Get(index);
}
inline const ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors& UsDrvDiagPort::asicerrors(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.asicErrors)
  return _internal_asicerrors(index);
}
inline ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors* UsDrvDiagPort::_internal_add_asicerrors() {
  return asicerrors_.Add();
}
inline ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors* UsDrvDiagPort::add_asicerrors() {
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.asicErrors)
  return _internal_add_asicerrors();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_asic_errors::UsDrvAsicErrors >&
UsDrvDiagPort::asicerrors() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.asicErrors)
  return asicerrors_;
}

// repeated .pb.us_drv.us_drv_sensor_errors.UsDrvSensorErrors sensorErrors = 155;
inline int UsDrvDiagPort::_internal_sensorerrors_size() const {
  return sensorerrors_.size();
}
inline int UsDrvDiagPort::sensorerrors_size() const {
  return _internal_sensorerrors_size();
}
inline ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors* UsDrvDiagPort::mutable_sensorerrors(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.sensorErrors)
  return sensorerrors_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors >*
UsDrvDiagPort::mutable_sensorerrors() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.sensorErrors)
  return &sensorerrors_;
}
inline const ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors& UsDrvDiagPort::_internal_sensorerrors(int index) const {
  return sensorerrors_.Get(index);
}
inline const ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors& UsDrvDiagPort::sensorerrors(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.sensorErrors)
  return _internal_sensorerrors(index);
}
inline ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors* UsDrvDiagPort::_internal_add_sensorerrors() {
  return sensorerrors_.Add();
}
inline ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors* UsDrvDiagPort::add_sensorerrors() {
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.sensorErrors)
  return _internal_add_sensorerrors();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_sensor_errors::UsDrvSensorErrors >&
UsDrvDiagPort::sensorerrors() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_diag_port.UsDrvDiagPort.sensorErrors)
  return sensorerrors_;
}

// -------------------------------------------------------------------

// UsDrvDiagPort_array_port

// repeated .pb.us_drv.us_drv_diag_port.UsDrvDiagPort data = 2394;
inline int UsDrvDiagPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsDrvDiagPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsDrvDiagPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort* UsDrvDiagPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort >*
UsDrvDiagPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port.data)
  return &data_;
}
inline const ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort& UsDrvDiagPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort& UsDrvDiagPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort* UsDrvDiagPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort* UsDrvDiagPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort >&
UsDrvDiagPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_drv_diag_port
}  // namespace us_drv
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto
