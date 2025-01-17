// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_processing/us_processing_data_integrity.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fprocessing_2fus_5fprocessing_5fdata_5fintegrity_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fprocessing_2fus_5fprocessing_5fdata_5fintegrity_2eproto

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
#include "us_processing/us_processing_sensor_running_mode.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_us_5fprocessing_2fus_5fprocessing_5fdata_5fintegrity_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fprocessing_2fus_5fprocessing_5fdata_5fintegrity_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fprocessing_2fus_5fprocessing_5fdata_5fintegrity_2eproto;
namespace pb {
namespace us_processing {
namespace us_processing_data_integrity {
class UsProcessingDataIntegrity;
class UsProcessingDataIntegrityDefaultTypeInternal;
extern UsProcessingDataIntegrityDefaultTypeInternal _UsProcessingDataIntegrity_default_instance_;
class UsProcessingDataIntegrity_array_port;
class UsProcessingDataIntegrity_array_portDefaultTypeInternal;
extern UsProcessingDataIntegrity_array_portDefaultTypeInternal _UsProcessingDataIntegrity_array_port_default_instance_;
}  // namespace us_processing_data_integrity
}  // namespace us_processing
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity* Arena::CreateMaybeMessage<::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity>(Arena*);
template<> ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity_array_port* Arena::CreateMaybeMessage<::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_processing {
namespace us_processing_data_integrity {

// ===================================================================

class UsProcessingDataIntegrity :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity) */ {
 public:
  UsProcessingDataIntegrity();
  virtual ~UsProcessingDataIntegrity();

  UsProcessingDataIntegrity(const UsProcessingDataIntegrity& from);
  UsProcessingDataIntegrity(UsProcessingDataIntegrity&& from) noexcept
    : UsProcessingDataIntegrity() {
    *this = ::std::move(from);
  }

  inline UsProcessingDataIntegrity& operator=(const UsProcessingDataIntegrity& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsProcessingDataIntegrity& operator=(UsProcessingDataIntegrity&& from) noexcept {
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
  static const UsProcessingDataIntegrity& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsProcessingDataIntegrity* internal_default_instance() {
    return reinterpret_cast<const UsProcessingDataIntegrity*>(
               &_UsProcessingDataIntegrity_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsProcessingDataIntegrity& a, UsProcessingDataIntegrity& b) {
    a.Swap(&b);
  }
  inline void Swap(UsProcessingDataIntegrity* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsProcessingDataIntegrity* New() const final {
    return CreateMaybeMessage<UsProcessingDataIntegrity>(nullptr);
  }

  UsProcessingDataIntegrity* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsProcessingDataIntegrity>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsProcessingDataIntegrity& from);
  void MergeFrom(const UsProcessingDataIntegrity& from);
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
  void InternalSwap(UsProcessingDataIntegrity* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fdata_5fintegrity_2eproto);
    return ::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fdata_5fintegrity_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSensorRunningModeFieldNumber = 466,
    kFiltDistMFieldNumber = 1519,
    kRawEchoCountFieldNumber = 1635,
    kSSigHeaderFieldNumber = 1033,
    kIsInputCommOkFieldNumber = 1573,
    kUiVersionNumberFieldNumber = 2124,
    kInputTemperatureFieldNumber = 2315,
    kInputVelocityFieldNumber = 3655,
  };
  // repeated .pb.us_processing.us_processing_sensor_running_mode.UsProcessingSensorRunningMode sensorRunningMode = 466;
  int sensorrunningmode_size() const;
  private:
  int _internal_sensorrunningmode_size() const;
  public:
  void clear_sensorrunningmode();
  private:
  ::pb::us_processing::us_processing_sensor_running_mode::UsProcessingSensorRunningMode _internal_sensorrunningmode(int index) const;
  void _internal_add_sensorrunningmode(::pb::us_processing::us_processing_sensor_running_mode::UsProcessingSensorRunningMode value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>* _internal_mutable_sensorrunningmode();
  public:
  ::pb::us_processing::us_processing_sensor_running_mode::UsProcessingSensorRunningMode sensorrunningmode(int index) const;
  void set_sensorrunningmode(int index, ::pb::us_processing::us_processing_sensor_running_mode::UsProcessingSensorRunningMode value);
  void add_sensorrunningmode(::pb::us_processing::us_processing_sensor_running_mode::UsProcessingSensorRunningMode value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>& sensorrunningmode() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>* mutable_sensorrunningmode();

  // repeated float filtDist_m = 1519;
  int filtdist_m_size() const;
  private:
  int _internal_filtdist_m_size() const;
  public:
  void clear_filtdist_m();
  private:
  float _internal_filtdist_m(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_filtdist_m() const;
  void _internal_add_filtdist_m(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_filtdist_m();
  public:
  float filtdist_m(int index) const;
  void set_filtdist_m(int index, float value);
  void add_filtdist_m(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      filtdist_m() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_filtdist_m();

  // repeated uint32 rawEchoCount = 1635;
  int rawechocount_size() const;
  private:
  int _internal_rawechocount_size() const;
  public:
  void clear_rawechocount();
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_rawechocount(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >&
      _internal_rawechocount() const;
  void _internal_add_rawechocount(::PROTOBUF_NAMESPACE_ID::uint32 value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >*
      _internal_mutable_rawechocount();
  public:
  ::PROTOBUF_NAMESPACE_ID::uint32 rawechocount(int index) const;
  void set_rawechocount(int index, ::PROTOBUF_NAMESPACE_ID::uint32 value);
  void add_rawechocount(::PROTOBUF_NAMESPACE_ID::uint32 value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >&
      rawechocount() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >*
      mutable_rawechocount();

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

  // optional bool isInputCommOk = 1573;
  bool has_isinputcommok() const;
  private:
  bool _internal_has_isinputcommok() const;
  public:
  void clear_isinputcommok();
  bool isinputcommok() const;
  void set_isinputcommok(bool value);
  private:
  bool _internal_isinputcommok() const;
  void _internal_set_isinputcommok(bool value);
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

  // optional float inputTemperature = 2315;
  bool has_inputtemperature() const;
  private:
  bool _internal_has_inputtemperature() const;
  public:
  void clear_inputtemperature();
  float inputtemperature() const;
  void set_inputtemperature(float value);
  private:
  float _internal_inputtemperature() const;
  void _internal_set_inputtemperature(float value);
  public:

  // optional float inputVelocity = 3655;
  bool has_inputvelocity() const;
  private:
  bool _internal_has_inputvelocity() const;
  public:
  void clear_inputvelocity();
  float inputvelocity() const;
  void set_inputvelocity(float value);
  private:
  float _internal_inputvelocity() const;
  void _internal_set_inputvelocity(float value);
  public:

  // @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField<int> sensorrunningmode_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > filtdist_m_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 > rawechocount_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  bool isinputcommok_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  float inputtemperature_;
  float inputvelocity_;
  friend struct ::TableStruct_us_5fprocessing_2fus_5fprocessing_5fdata_5fintegrity_2eproto;
};
// -------------------------------------------------------------------

class UsProcessingDataIntegrity_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity_array_port) */ {
 public:
  UsProcessingDataIntegrity_array_port();
  virtual ~UsProcessingDataIntegrity_array_port();

  UsProcessingDataIntegrity_array_port(const UsProcessingDataIntegrity_array_port& from);
  UsProcessingDataIntegrity_array_port(UsProcessingDataIntegrity_array_port&& from) noexcept
    : UsProcessingDataIntegrity_array_port() {
    *this = ::std::move(from);
  }

  inline UsProcessingDataIntegrity_array_port& operator=(const UsProcessingDataIntegrity_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsProcessingDataIntegrity_array_port& operator=(UsProcessingDataIntegrity_array_port&& from) noexcept {
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
  static const UsProcessingDataIntegrity_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsProcessingDataIntegrity_array_port* internal_default_instance() {
    return reinterpret_cast<const UsProcessingDataIntegrity_array_port*>(
               &_UsProcessingDataIntegrity_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsProcessingDataIntegrity_array_port& a, UsProcessingDataIntegrity_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsProcessingDataIntegrity_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsProcessingDataIntegrity_array_port* New() const final {
    return CreateMaybeMessage<UsProcessingDataIntegrity_array_port>(nullptr);
  }

  UsProcessingDataIntegrity_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsProcessingDataIntegrity_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsProcessingDataIntegrity_array_port& from);
  void MergeFrom(const UsProcessingDataIntegrity_array_port& from);
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
  void InternalSwap(UsProcessingDataIntegrity_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fdata_5fintegrity_2eproto);
    return ::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fdata_5fintegrity_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2694,
  };
  // repeated .pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity data = 2694;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity >*
      mutable_data();
  private:
  const ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity& _internal_data(int index) const;
  ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity* _internal_add_data();
  public:
  const ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity& data(int index) const;
  ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity > data_;
  friend struct ::TableStruct_us_5fprocessing_2fus_5fprocessing_5fdata_5fintegrity_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsProcessingDataIntegrity

// optional uint32 uiVersionNumber = 2124;
inline bool UsProcessingDataIntegrity::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool UsProcessingDataIntegrity::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void UsProcessingDataIntegrity::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsProcessingDataIntegrity::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsProcessingDataIntegrity::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void UsProcessingDataIntegrity::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  uiversionnumber_ = value;
}
inline void UsProcessingDataIntegrity::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool UsProcessingDataIntegrity::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool UsProcessingDataIntegrity::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& UsProcessingDataIntegrity::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& UsProcessingDataIntegrity::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* UsProcessingDataIntegrity::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* UsProcessingDataIntegrity::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* UsProcessingDataIntegrity::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void UsProcessingDataIntegrity::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.sSigHeader)
}

// repeated .pb.us_processing.us_processing_sensor_running_mode.UsProcessingSensorRunningMode sensorRunningMode = 466;
inline int UsProcessingDataIntegrity::_internal_sensorrunningmode_size() const {
  return sensorrunningmode_.size();
}
inline int UsProcessingDataIntegrity::sensorrunningmode_size() const {
  return _internal_sensorrunningmode_size();
}
inline void UsProcessingDataIntegrity::clear_sensorrunningmode() {
  sensorrunningmode_.Clear();
}
inline ::pb::us_processing::us_processing_sensor_running_mode::UsProcessingSensorRunningMode UsProcessingDataIntegrity::_internal_sensorrunningmode(int index) const {
  return static_cast< ::pb::us_processing::us_processing_sensor_running_mode::UsProcessingSensorRunningMode >(sensorrunningmode_.Get(index));
}
inline ::pb::us_processing::us_processing_sensor_running_mode::UsProcessingSensorRunningMode UsProcessingDataIntegrity::sensorrunningmode(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.sensorRunningMode)
  return _internal_sensorrunningmode(index);
}
inline void UsProcessingDataIntegrity::set_sensorrunningmode(int index, ::pb::us_processing::us_processing_sensor_running_mode::UsProcessingSensorRunningMode value) {
  assert(::pb::us_processing::us_processing_sensor_running_mode::UsProcessingSensorRunningMode_IsValid(value));
  sensorrunningmode_.Set(index, value);
  // @@protoc_insertion_point(field_set:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.sensorRunningMode)
}
inline void UsProcessingDataIntegrity::_internal_add_sensorrunningmode(::pb::us_processing::us_processing_sensor_running_mode::UsProcessingSensorRunningMode value) {
  assert(::pb::us_processing::us_processing_sensor_running_mode::UsProcessingSensorRunningMode_IsValid(value));
  sensorrunningmode_.Add(value);
}
inline void UsProcessingDataIntegrity::add_sensorrunningmode(::pb::us_processing::us_processing_sensor_running_mode::UsProcessingSensorRunningMode value) {
  // @@protoc_insertion_point(field_add:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.sensorRunningMode)
  _internal_add_sensorrunningmode(value);
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>&
UsProcessingDataIntegrity::sensorrunningmode() const {
  // @@protoc_insertion_point(field_list:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.sensorRunningMode)
  return sensorrunningmode_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>*
UsProcessingDataIntegrity::_internal_mutable_sensorrunningmode() {
  return &sensorrunningmode_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>*
UsProcessingDataIntegrity::mutable_sensorrunningmode() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.sensorRunningMode)
  return _internal_mutable_sensorrunningmode();
}

// repeated float filtDist_m = 1519;
inline int UsProcessingDataIntegrity::_internal_filtdist_m_size() const {
  return filtdist_m_.size();
}
inline int UsProcessingDataIntegrity::filtdist_m_size() const {
  return _internal_filtdist_m_size();
}
inline void UsProcessingDataIntegrity::clear_filtdist_m() {
  filtdist_m_.Clear();
}
inline float UsProcessingDataIntegrity::_internal_filtdist_m(int index) const {
  return filtdist_m_.Get(index);
}
inline float UsProcessingDataIntegrity::filtdist_m(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.filtDist_m)
  return _internal_filtdist_m(index);
}
inline void UsProcessingDataIntegrity::set_filtdist_m(int index, float value) {
  filtdist_m_.Set(index, value);
  // @@protoc_insertion_point(field_set:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.filtDist_m)
}
inline void UsProcessingDataIntegrity::_internal_add_filtdist_m(float value) {
  filtdist_m_.Add(value);
}
inline void UsProcessingDataIntegrity::add_filtdist_m(float value) {
  _internal_add_filtdist_m(value);
  // @@protoc_insertion_point(field_add:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.filtDist_m)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
UsProcessingDataIntegrity::_internal_filtdist_m() const {
  return filtdist_m_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
UsProcessingDataIntegrity::filtdist_m() const {
  // @@protoc_insertion_point(field_list:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.filtDist_m)
  return _internal_filtdist_m();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
UsProcessingDataIntegrity::_internal_mutable_filtdist_m() {
  return &filtdist_m_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
UsProcessingDataIntegrity::mutable_filtdist_m() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.filtDist_m)
  return _internal_mutable_filtdist_m();
}

// repeated uint32 rawEchoCount = 1635;
inline int UsProcessingDataIntegrity::_internal_rawechocount_size() const {
  return rawechocount_.size();
}
inline int UsProcessingDataIntegrity::rawechocount_size() const {
  return _internal_rawechocount_size();
}
inline void UsProcessingDataIntegrity::clear_rawechocount() {
  rawechocount_.Clear();
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsProcessingDataIntegrity::_internal_rawechocount(int index) const {
  return rawechocount_.Get(index);
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsProcessingDataIntegrity::rawechocount(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.rawEchoCount)
  return _internal_rawechocount(index);
}
inline void UsProcessingDataIntegrity::set_rawechocount(int index, ::PROTOBUF_NAMESPACE_ID::uint32 value) {
  rawechocount_.Set(index, value);
  // @@protoc_insertion_point(field_set:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.rawEchoCount)
}
inline void UsProcessingDataIntegrity::_internal_add_rawechocount(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  rawechocount_.Add(value);
}
inline void UsProcessingDataIntegrity::add_rawechocount(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_add_rawechocount(value);
  // @@protoc_insertion_point(field_add:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.rawEchoCount)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >&
UsProcessingDataIntegrity::_internal_rawechocount() const {
  return rawechocount_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >&
UsProcessingDataIntegrity::rawechocount() const {
  // @@protoc_insertion_point(field_list:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.rawEchoCount)
  return _internal_rawechocount();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >*
UsProcessingDataIntegrity::_internal_mutable_rawechocount() {
  return &rawechocount_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::uint32 >*
UsProcessingDataIntegrity::mutable_rawechocount() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.rawEchoCount)
  return _internal_mutable_rawechocount();
}

// optional float inputVelocity = 3655;
inline bool UsProcessingDataIntegrity::_internal_has_inputvelocity() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool UsProcessingDataIntegrity::has_inputvelocity() const {
  return _internal_has_inputvelocity();
}
inline void UsProcessingDataIntegrity::clear_inputvelocity() {
  inputvelocity_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline float UsProcessingDataIntegrity::_internal_inputvelocity() const {
  return inputvelocity_;
}
inline float UsProcessingDataIntegrity::inputvelocity() const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.inputVelocity)
  return _internal_inputvelocity();
}
inline void UsProcessingDataIntegrity::_internal_set_inputvelocity(float value) {
  _has_bits_[0] |= 0x00000010u;
  inputvelocity_ = value;
}
inline void UsProcessingDataIntegrity::set_inputvelocity(float value) {
  _internal_set_inputvelocity(value);
  // @@protoc_insertion_point(field_set:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.inputVelocity)
}

// optional float inputTemperature = 2315;
inline bool UsProcessingDataIntegrity::_internal_has_inputtemperature() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool UsProcessingDataIntegrity::has_inputtemperature() const {
  return _internal_has_inputtemperature();
}
inline void UsProcessingDataIntegrity::clear_inputtemperature() {
  inputtemperature_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float UsProcessingDataIntegrity::_internal_inputtemperature() const {
  return inputtemperature_;
}
inline float UsProcessingDataIntegrity::inputtemperature() const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.inputTemperature)
  return _internal_inputtemperature();
}
inline void UsProcessingDataIntegrity::_internal_set_inputtemperature(float value) {
  _has_bits_[0] |= 0x00000008u;
  inputtemperature_ = value;
}
inline void UsProcessingDataIntegrity::set_inputtemperature(float value) {
  _internal_set_inputtemperature(value);
  // @@protoc_insertion_point(field_set:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.inputTemperature)
}

// optional bool isInputCommOk = 1573;
inline bool UsProcessingDataIntegrity::_internal_has_isinputcommok() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool UsProcessingDataIntegrity::has_isinputcommok() const {
  return _internal_has_isinputcommok();
}
inline void UsProcessingDataIntegrity::clear_isinputcommok() {
  isinputcommok_ = false;
  _has_bits_[0] &= ~0x00000002u;
}
inline bool UsProcessingDataIntegrity::_internal_isinputcommok() const {
  return isinputcommok_;
}
inline bool UsProcessingDataIntegrity::isinputcommok() const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.isInputCommOk)
  return _internal_isinputcommok();
}
inline void UsProcessingDataIntegrity::_internal_set_isinputcommok(bool value) {
  _has_bits_[0] |= 0x00000002u;
  isinputcommok_ = value;
}
inline void UsProcessingDataIntegrity::set_isinputcommok(bool value) {
  _internal_set_isinputcommok(value);
  // @@protoc_insertion_point(field_set:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity.isInputCommOk)
}

// -------------------------------------------------------------------

// UsProcessingDataIntegrity_array_port

// repeated .pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity data = 2694;
inline int UsProcessingDataIntegrity_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsProcessingDataIntegrity_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsProcessingDataIntegrity_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity* UsProcessingDataIntegrity_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity >*
UsProcessingDataIntegrity_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity_array_port.data)
  return &data_;
}
inline const ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity& UsProcessingDataIntegrity_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity& UsProcessingDataIntegrity_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity* UsProcessingDataIntegrity_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity* UsProcessingDataIntegrity_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_data_integrity::UsProcessingDataIntegrity >&
UsProcessingDataIntegrity_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_processing.us_processing_data_integrity.UsProcessingDataIntegrity_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_processing_data_integrity
}  // namespace us_processing
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fprocessing_2fus_5fprocessing_5fdata_5fintegrity_2eproto
