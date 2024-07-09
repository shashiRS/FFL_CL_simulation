// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_drv/us_drv_envelope_data.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto

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
#include "us_drv/us_drv_sensor_state.pb.h"
#include "us_drv/us_drv_envelope_signal_path.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto;
namespace pb {
namespace us_drv {
namespace us_drv_envelope_data {
class UsDrvEnvelopeData;
class UsDrvEnvelopeDataDefaultTypeInternal;
extern UsDrvEnvelopeDataDefaultTypeInternal _UsDrvEnvelopeData_default_instance_;
class UsDrvEnvelopeData_array_port;
class UsDrvEnvelopeData_array_portDefaultTypeInternal;
extern UsDrvEnvelopeData_array_portDefaultTypeInternal _UsDrvEnvelopeData_array_port_default_instance_;
}  // namespace us_drv_envelope_data
}  // namespace us_drv
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData>(Arena*);
template<> ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData_array_port* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_drv {
namespace us_drv_envelope_data {

// ===================================================================

class UsDrvEnvelopeData :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData) */ {
 public:
  UsDrvEnvelopeData();
  virtual ~UsDrvEnvelopeData();

  UsDrvEnvelopeData(const UsDrvEnvelopeData& from);
  UsDrvEnvelopeData(UsDrvEnvelopeData&& from) noexcept
    : UsDrvEnvelopeData() {
    *this = ::std::move(from);
  }

  inline UsDrvEnvelopeData& operator=(const UsDrvEnvelopeData& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvEnvelopeData& operator=(UsDrvEnvelopeData&& from) noexcept {
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
  static const UsDrvEnvelopeData& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvEnvelopeData* internal_default_instance() {
    return reinterpret_cast<const UsDrvEnvelopeData*>(
               &_UsDrvEnvelopeData_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsDrvEnvelopeData& a, UsDrvEnvelopeData& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvEnvelopeData* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvEnvelopeData* New() const final {
    return CreateMaybeMessage<UsDrvEnvelopeData>(nullptr);
  }

  UsDrvEnvelopeData* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvEnvelopeData>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvEnvelopeData& from);
  void MergeFrom(const UsDrvEnvelopeData& from);
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
  void InternalSwap(UsDrvEnvelopeData* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSamplesFieldNumber = 22,
    kSignalPathsFieldNumber = 1240,
    kSensorStateFieldNumber = 3135,
    kSSigHeaderFieldNumber = 1033,
    kUiVersionNumberFieldNumber = 2124,
    kNumSamplesFieldNumber = 993,
    kNumSignalPathsFieldNumber = 1888,
  };
  // repeated uint32 samples = 22;
  int samples_size() const;
  private:
  int _internal_samples_size() const;
  public:
  void clear_samples();
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_samples(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >&
      _internal_samples() const;
  void _internal_add_samples(::PROTOBUF_NAMESPACE_ID::uint32 value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >*
      _internal_mutable_samples();
  public:
  ::PROTOBUF_NAMESPACE_ID::uint32 samples(int index) const;
  void set_samples(int index, ::PROTOBUF_NAMESPACE_ID::uint32 value);
  void add_samples(::PROTOBUF_NAMESPACE_ID::uint32 value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >&
      samples() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >*
      mutable_samples();

  // repeated .pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath signalPaths = 1240;
  int signalpaths_size() const;
  private:
  int _internal_signalpaths_size() const;
  public:
  void clear_signalpaths();
  ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath* mutable_signalpaths(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath >*
      mutable_signalpaths();
  private:
  const ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath& _internal_signalpaths(int index) const;
  ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath* _internal_add_signalpaths();
  public:
  const ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath& signalpaths(int index) const;
  ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath* add_signalpaths();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath >&
      signalpaths() const;

  // repeated .pb.us_drv.us_drv_sensor_state.UsDrvSensorState sensorState = 3135;
  int sensorstate_size() const;
  private:
  int _internal_sensorstate_size() const;
  public:
  void clear_sensorstate();
  private:
  ::pb::us_drv::us_drv_sensor_state::UsDrvSensorState _internal_sensorstate(int index) const;
  void _internal_add_sensorstate(::pb::us_drv::us_drv_sensor_state::UsDrvSensorState value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>* _internal_mutable_sensorstate();
  public:
  ::pb::us_drv::us_drv_sensor_state::UsDrvSensorState sensorstate(int index) const;
  void set_sensorstate(int index, ::pb::us_drv::us_drv_sensor_state::UsDrvSensorState value);
  void add_sensorstate(::pb::us_drv::us_drv_sensor_state::UsDrvSensorState value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>& sensorstate() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>* mutable_sensorstate();

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

  // optional uint32 numSamples = 993;
  bool has_numsamples() const;
  private:
  bool _internal_has_numsamples() const;
  public:
  void clear_numsamples();
  ::PROTOBUF_NAMESPACE_ID::uint32 numsamples() const;
  void set_numsamples(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_numsamples() const;
  void _internal_set_numsamples(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 numSignalPaths = 1888;
  bool has_numsignalpaths() const;
  private:
  bool _internal_has_numsignalpaths() const;
  public:
  void clear_numsignalpaths();
  ::PROTOBUF_NAMESPACE_ID::uint32 numsignalpaths() const;
  void set_numsignalpaths(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_numsignalpaths() const;
  void _internal_set_numsignalpaths(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 > samples_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath > signalpaths_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField<int> sensorstate_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  ::PROTOBUF_NAMESPACE_ID::uint32 numsamples_;
  ::PROTOBUF_NAMESPACE_ID::uint32 numsignalpaths_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto;
};
// -------------------------------------------------------------------

class UsDrvEnvelopeData_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port) */ {
 public:
  UsDrvEnvelopeData_array_port();
  virtual ~UsDrvEnvelopeData_array_port();

  UsDrvEnvelopeData_array_port(const UsDrvEnvelopeData_array_port& from);
  UsDrvEnvelopeData_array_port(UsDrvEnvelopeData_array_port&& from) noexcept
    : UsDrvEnvelopeData_array_port() {
    *this = ::std::move(from);
  }

  inline UsDrvEnvelopeData_array_port& operator=(const UsDrvEnvelopeData_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvEnvelopeData_array_port& operator=(UsDrvEnvelopeData_array_port&& from) noexcept {
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
  static const UsDrvEnvelopeData_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvEnvelopeData_array_port* internal_default_instance() {
    return reinterpret_cast<const UsDrvEnvelopeData_array_port*>(
               &_UsDrvEnvelopeData_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsDrvEnvelopeData_array_port& a, UsDrvEnvelopeData_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvEnvelopeData_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvEnvelopeData_array_port* New() const final {
    return CreateMaybeMessage<UsDrvEnvelopeData_array_port>(nullptr);
  }

  UsDrvEnvelopeData_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvEnvelopeData_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvEnvelopeData_array_port& from);
  void MergeFrom(const UsDrvEnvelopeData_array_port& from);
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
  void InternalSwap(UsDrvEnvelopeData_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3478,
  };
  // repeated .pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData data = 3478;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData >*
      mutable_data();
  private:
  const ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData& _internal_data(int index) const;
  ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData* _internal_add_data();
  public:
  const ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData& data(int index) const;
  ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData > data_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsDrvEnvelopeData

// optional uint32 uiVersionNumber = 2124;
inline bool UsDrvEnvelopeData::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool UsDrvEnvelopeData::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void UsDrvEnvelopeData::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvEnvelopeData::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvEnvelopeData::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void UsDrvEnvelopeData::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  uiversionnumber_ = value;
}
inline void UsDrvEnvelopeData::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool UsDrvEnvelopeData::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool UsDrvEnvelopeData::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& UsDrvEnvelopeData::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& UsDrvEnvelopeData::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* UsDrvEnvelopeData::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* UsDrvEnvelopeData::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* UsDrvEnvelopeData::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void UsDrvEnvelopeData::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.sSigHeader)
}

// repeated .pb.us_drv.us_drv_sensor_state.UsDrvSensorState sensorState = 3135;
inline int UsDrvEnvelopeData::_internal_sensorstate_size() const {
  return sensorstate_.size();
}
inline int UsDrvEnvelopeData::sensorstate_size() const {
  return _internal_sensorstate_size();
}
inline void UsDrvEnvelopeData::clear_sensorstate() {
  sensorstate_.Clear();
}
inline ::pb::us_drv::us_drv_sensor_state::UsDrvSensorState UsDrvEnvelopeData::_internal_sensorstate(int index) const {
  return static_cast< ::pb::us_drv::us_drv_sensor_state::UsDrvSensorState >(sensorstate_.Get(index));
}
inline ::pb::us_drv::us_drv_sensor_state::UsDrvSensorState UsDrvEnvelopeData::sensorstate(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.sensorState)
  return _internal_sensorstate(index);
}
inline void UsDrvEnvelopeData::set_sensorstate(int index, ::pb::us_drv::us_drv_sensor_state::UsDrvSensorState value) {
  assert(::pb::us_drv::us_drv_sensor_state::UsDrvSensorState_IsValid(value));
  sensorstate_.Set(index, value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.sensorState)
}
inline void UsDrvEnvelopeData::_internal_add_sensorstate(::pb::us_drv::us_drv_sensor_state::UsDrvSensorState value) {
  assert(::pb::us_drv::us_drv_sensor_state::UsDrvSensorState_IsValid(value));
  sensorstate_.Add(value);
}
inline void UsDrvEnvelopeData::add_sensorstate(::pb::us_drv::us_drv_sensor_state::UsDrvSensorState value) {
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.sensorState)
  _internal_add_sensorstate(value);
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>&
UsDrvEnvelopeData::sensorstate() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.sensorState)
  return sensorstate_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>*
UsDrvEnvelopeData::_internal_mutable_sensorstate() {
  return &sensorstate_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>*
UsDrvEnvelopeData::mutable_sensorstate() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.sensorState)
  return _internal_mutable_sensorstate();
}

// optional uint32 numSignalPaths = 1888;
inline bool UsDrvEnvelopeData::_internal_has_numsignalpaths() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool UsDrvEnvelopeData::has_numsignalpaths() const {
  return _internal_has_numsignalpaths();
}
inline void UsDrvEnvelopeData::clear_numsignalpaths() {
  numsignalpaths_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvEnvelopeData::_internal_numsignalpaths() const {
  return numsignalpaths_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvEnvelopeData::numsignalpaths() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.numSignalPaths)
  return _internal_numsignalpaths();
}
inline void UsDrvEnvelopeData::_internal_set_numsignalpaths(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  numsignalpaths_ = value;
}
inline void UsDrvEnvelopeData::set_numsignalpaths(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_numsignalpaths(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.numSignalPaths)
}

// repeated .pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath signalPaths = 1240;
inline int UsDrvEnvelopeData::_internal_signalpaths_size() const {
  return signalpaths_.size();
}
inline int UsDrvEnvelopeData::signalpaths_size() const {
  return _internal_signalpaths_size();
}
inline ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath* UsDrvEnvelopeData::mutable_signalpaths(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.signalPaths)
  return signalpaths_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath >*
UsDrvEnvelopeData::mutable_signalpaths() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.signalPaths)
  return &signalpaths_;
}
inline const ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath& UsDrvEnvelopeData::_internal_signalpaths(int index) const {
  return signalpaths_.Get(index);
}
inline const ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath& UsDrvEnvelopeData::signalpaths(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.signalPaths)
  return _internal_signalpaths(index);
}
inline ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath* UsDrvEnvelopeData::_internal_add_signalpaths() {
  return signalpaths_.Add();
}
inline ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath* UsDrvEnvelopeData::add_signalpaths() {
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.signalPaths)
  return _internal_add_signalpaths();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_envelope_signal_path::UsDrvEnvelopeSignalPath >&
UsDrvEnvelopeData::signalpaths() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.signalPaths)
  return signalpaths_;
}

// optional uint32 numSamples = 993;
inline bool UsDrvEnvelopeData::_internal_has_numsamples() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool UsDrvEnvelopeData::has_numsamples() const {
  return _internal_has_numsamples();
}
inline void UsDrvEnvelopeData::clear_numsamples() {
  numsamples_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvEnvelopeData::_internal_numsamples() const {
  return numsamples_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvEnvelopeData::numsamples() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.numSamples)
  return _internal_numsamples();
}
inline void UsDrvEnvelopeData::_internal_set_numsamples(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  numsamples_ = value;
}
inline void UsDrvEnvelopeData::set_numsamples(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_numsamples(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.numSamples)
}

// repeated uint32 samples = 22;
inline int UsDrvEnvelopeData::_internal_samples_size() const {
  return samples_.size();
}
inline int UsDrvEnvelopeData::samples_size() const {
  return _internal_samples_size();
}
inline void UsDrvEnvelopeData::clear_samples() {
  samples_.Clear();
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvEnvelopeData::_internal_samples(int index) const {
  return samples_.Get(index);
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvEnvelopeData::samples(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.samples)
  return _internal_samples(index);
}
inline void UsDrvEnvelopeData::set_samples(int index, ::PROTOBUF_NAMESPACE_ID::uint32 value) {
  samples_.Set(index, value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.samples)
}
inline void UsDrvEnvelopeData::_internal_add_samples(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  samples_.Add(value);
}
inline void UsDrvEnvelopeData::add_samples(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_add_samples(value);
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.samples)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >&
UsDrvEnvelopeData::_internal_samples() const {
  return samples_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >&
UsDrvEnvelopeData::samples() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.samples)
  return _internal_samples();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >*
UsDrvEnvelopeData::_internal_mutable_samples() {
  return &samples_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >*
UsDrvEnvelopeData::mutable_samples() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.samples)
  return _internal_mutable_samples();
}

// -------------------------------------------------------------------

// UsDrvEnvelopeData_array_port

// repeated .pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData data = 3478;
inline int UsDrvEnvelopeData_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsDrvEnvelopeData_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsDrvEnvelopeData_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData* UsDrvEnvelopeData_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData >*
UsDrvEnvelopeData_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port.data)
  return &data_;
}
inline const ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData& UsDrvEnvelopeData_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData& UsDrvEnvelopeData_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData* UsDrvEnvelopeData_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData* UsDrvEnvelopeData_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData >&
UsDrvEnvelopeData_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_drv_envelope_data
}  // namespace us_drv
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto
