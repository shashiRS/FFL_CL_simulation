// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_em/us_em_params.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fem_2fus_5fem_5fparams_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fem_2fus_5fem_5fparams_2eproto

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
#include "us_em/us_em_neighboring_filter_cfg_intern.pb.h"
#include "us_em/us_em_detection_zone_cfg.pb.h"
#include "us_em/us_em_sensor_parameters_intern.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_us_5fem_2fus_5fem_5fparams_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fem_2fus_5fem_5fparams_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fem_2fus_5fem_5fparams_2eproto;
namespace pb {
namespace us_em {
namespace us_em_params {
class UsEmParams;
class UsEmParamsDefaultTypeInternal;
extern UsEmParamsDefaultTypeInternal _UsEmParams_default_instance_;
class UsEmParams_array_port;
class UsEmParams_array_portDefaultTypeInternal;
extern UsEmParams_array_portDefaultTypeInternal _UsEmParams_array_port_default_instance_;
}  // namespace us_em_params
}  // namespace us_em
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_em::us_em_params::UsEmParams* Arena::CreateMaybeMessage<::pb::us_em::us_em_params::UsEmParams>(Arena*);
template<> ::pb::us_em::us_em_params::UsEmParams_array_port* Arena::CreateMaybeMessage<::pb::us_em::us_em_params::UsEmParams_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_em {
namespace us_em_params {

// ===================================================================

class UsEmParams :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_em.us_em_params.UsEmParams) */ {
 public:
  UsEmParams();
  virtual ~UsEmParams();

  UsEmParams(const UsEmParams& from);
  UsEmParams(UsEmParams&& from) noexcept
    : UsEmParams() {
    *this = ::std::move(from);
  }

  inline UsEmParams& operator=(const UsEmParams& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsEmParams& operator=(UsEmParams&& from) noexcept {
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
  static const UsEmParams& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsEmParams* internal_default_instance() {
    return reinterpret_cast<const UsEmParams*>(
               &_UsEmParams_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsEmParams& a, UsEmParams& b) {
    a.Swap(&b);
  }
  inline void Swap(UsEmParams* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsEmParams* New() const final {
    return CreateMaybeMessage<UsEmParams>(nullptr);
  }

  UsEmParams* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsEmParams>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsEmParams& from);
  void MergeFrom(const UsEmParams& from);
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
  void InternalSwap(UsEmParams* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_em.us_em_params.UsEmParams";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fem_2fus_5fem_5fparams_2eproto);
    return ::descriptor_table_us_5fem_2fus_5fem_5fparams_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDetZoneParamsFieldNumber = 89,
    kSSigHeaderFieldNumber = 1033,
    kUspcParamsFieldNumber = 1754,
    kUsSensorParamsFieldNumber = 3105,
    kUiVersionNumberFieldNumber = 2124,
  };
  // optional .pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg detZoneParams = 89;
  bool has_detzoneparams() const;
  private:
  bool _internal_has_detzoneparams() const;
  public:
  void clear_detzoneparams();
  const ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg& detzoneparams() const;
  ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* release_detzoneparams();
  ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* mutable_detzoneparams();
  void set_allocated_detzoneparams(::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* detzoneparams);
  private:
  const ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg& _internal_detzoneparams() const;
  ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* _internal_mutable_detzoneparams();
  public:

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

  // optional .pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern uspcParams = 1754;
  bool has_uspcparams() const;
  private:
  bool _internal_has_uspcparams() const;
  public:
  void clear_uspcparams();
  const ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern& uspcparams() const;
  ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern* release_uspcparams();
  ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern* mutable_uspcparams();
  void set_allocated_uspcparams(::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern* uspcparams);
  private:
  const ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern& _internal_uspcparams() const;
  ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern* _internal_mutable_uspcparams();
  public:

  // optional .pb.us_em.us_em_sensor_parameters_intern.UsEmSensorParametersIntern usSensorParams = 3105;
  bool has_ussensorparams() const;
  private:
  bool _internal_has_ussensorparams() const;
  public:
  void clear_ussensorparams();
  const ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern& ussensorparams() const;
  ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern* release_ussensorparams();
  ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern* mutable_ussensorparams();
  void set_allocated_ussensorparams(::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern* ussensorparams);
  private:
  const ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern& _internal_ussensorparams() const;
  ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern* _internal_mutable_ussensorparams();
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

  // @@protoc_insertion_point(class_scope:pb.us_em.us_em_params.UsEmParams)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* detzoneparams_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern* uspcparams_;
  ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern* ussensorparams_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  friend struct ::TableStruct_us_5fem_2fus_5fem_5fparams_2eproto;
};
// -------------------------------------------------------------------

class UsEmParams_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_em.us_em_params.UsEmParams_array_port) */ {
 public:
  UsEmParams_array_port();
  virtual ~UsEmParams_array_port();

  UsEmParams_array_port(const UsEmParams_array_port& from);
  UsEmParams_array_port(UsEmParams_array_port&& from) noexcept
    : UsEmParams_array_port() {
    *this = ::std::move(from);
  }

  inline UsEmParams_array_port& operator=(const UsEmParams_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsEmParams_array_port& operator=(UsEmParams_array_port&& from) noexcept {
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
  static const UsEmParams_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsEmParams_array_port* internal_default_instance() {
    return reinterpret_cast<const UsEmParams_array_port*>(
               &_UsEmParams_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsEmParams_array_port& a, UsEmParams_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsEmParams_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsEmParams_array_port* New() const final {
    return CreateMaybeMessage<UsEmParams_array_port>(nullptr);
  }

  UsEmParams_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsEmParams_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsEmParams_array_port& from);
  void MergeFrom(const UsEmParams_array_port& from);
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
  void InternalSwap(UsEmParams_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_em.us_em_params.UsEmParams_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fem_2fus_5fem_5fparams_2eproto);
    return ::descriptor_table_us_5fem_2fus_5fem_5fparams_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2277,
  };
  // repeated .pb.us_em.us_em_params.UsEmParams data = 2277;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_em::us_em_params::UsEmParams* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_params::UsEmParams >*
      mutable_data();
  private:
  const ::pb::us_em::us_em_params::UsEmParams& _internal_data(int index) const;
  ::pb::us_em::us_em_params::UsEmParams* _internal_add_data();
  public:
  const ::pb::us_em::us_em_params::UsEmParams& data(int index) const;
  ::pb::us_em::us_em_params::UsEmParams* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_params::UsEmParams >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_em.us_em_params.UsEmParams_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_params::UsEmParams > data_;
  friend struct ::TableStruct_us_5fem_2fus_5fem_5fparams_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsEmParams

// optional uint32 uiVersionNumber = 2124;
inline bool UsEmParams::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool UsEmParams::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void UsEmParams::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEmParams::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEmParams::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_params.UsEmParams.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void UsEmParams::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  uiversionnumber_ = value;
}
inline void UsEmParams::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_em_params.UsEmParams.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool UsEmParams::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool UsEmParams::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& UsEmParams::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& UsEmParams::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_params.UsEmParams.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* UsEmParams::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.us_em.us_em_params.UsEmParams.sSigHeader)
  _has_bits_[0] &= ~0x00000002u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* UsEmParams::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000002u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* UsEmParams::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_em_params.UsEmParams.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void UsEmParams::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  ssigheader_ = ssigheader;
  // @@protoc_insertion_point(field_set_allocated:pb.us_em.us_em_params.UsEmParams.sSigHeader)
}

// optional .pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern uspcParams = 1754;
inline bool UsEmParams::_internal_has_uspcparams() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  PROTOBUF_ASSUME(!value || uspcparams_ != nullptr);
  return value;
}
inline bool UsEmParams::has_uspcparams() const {
  return _internal_has_uspcparams();
}
inline const ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern& UsEmParams::_internal_uspcparams() const {
  const ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern* p = uspcparams_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern*>(
      &::pb::us_em::us_em_neighboring_filter_cfg_intern::_UsEmNeighboringFilterCfgIntern_default_instance_);
}
inline const ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern& UsEmParams::uspcparams() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_params.UsEmParams.uspcParams)
  return _internal_uspcparams();
}
inline ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern* UsEmParams::release_uspcparams() {
  // @@protoc_insertion_point(field_release:pb.us_em.us_em_params.UsEmParams.uspcParams)
  _has_bits_[0] &= ~0x00000004u;
  ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern* temp = uspcparams_;
  uspcparams_ = nullptr;
  return temp;
}
inline ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern* UsEmParams::_internal_mutable_uspcparams() {
  _has_bits_[0] |= 0x00000004u;
  if (uspcparams_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern>(GetArenaNoVirtual());
    uspcparams_ = p;
  }
  return uspcparams_;
}
inline ::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern* UsEmParams::mutable_uspcparams() {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_em_params.UsEmParams.uspcParams)
  return _internal_mutable_uspcparams();
}
inline void UsEmParams::set_allocated_uspcparams(::pb::us_em::us_em_neighboring_filter_cfg_intern::UsEmNeighboringFilterCfgIntern* uspcparams) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(uspcparams_);
  }
  if (uspcparams) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      uspcparams = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, uspcparams, submessage_arena);
    }
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  uspcparams_ = uspcparams;
  // @@protoc_insertion_point(field_set_allocated:pb.us_em.us_em_params.UsEmParams.uspcParams)
}

// optional .pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg detZoneParams = 89;
inline bool UsEmParams::_internal_has_detzoneparams() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || detzoneparams_ != nullptr);
  return value;
}
inline bool UsEmParams::has_detzoneparams() const {
  return _internal_has_detzoneparams();
}
inline const ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg& UsEmParams::_internal_detzoneparams() const {
  const ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* p = detzoneparams_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg*>(
      &::pb::us_em::us_em_detection_zone_cfg::_UsEmDetectionZoneCfg_default_instance_);
}
inline const ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg& UsEmParams::detzoneparams() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_params.UsEmParams.detZoneParams)
  return _internal_detzoneparams();
}
inline ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* UsEmParams::release_detzoneparams() {
  // @@protoc_insertion_point(field_release:pb.us_em.us_em_params.UsEmParams.detZoneParams)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* temp = detzoneparams_;
  detzoneparams_ = nullptr;
  return temp;
}
inline ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* UsEmParams::_internal_mutable_detzoneparams() {
  _has_bits_[0] |= 0x00000001u;
  if (detzoneparams_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg>(GetArenaNoVirtual());
    detzoneparams_ = p;
  }
  return detzoneparams_;
}
inline ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* UsEmParams::mutable_detzoneparams() {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_em_params.UsEmParams.detZoneParams)
  return _internal_mutable_detzoneparams();
}
inline void UsEmParams::set_allocated_detzoneparams(::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* detzoneparams) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(detzoneparams_);
  }
  if (detzoneparams) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      detzoneparams = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, detzoneparams, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  detzoneparams_ = detzoneparams;
  // @@protoc_insertion_point(field_set_allocated:pb.us_em.us_em_params.UsEmParams.detZoneParams)
}

// optional .pb.us_em.us_em_sensor_parameters_intern.UsEmSensorParametersIntern usSensorParams = 3105;
inline bool UsEmParams::_internal_has_ussensorparams() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  PROTOBUF_ASSUME(!value || ussensorparams_ != nullptr);
  return value;
}
inline bool UsEmParams::has_ussensorparams() const {
  return _internal_has_ussensorparams();
}
inline const ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern& UsEmParams::_internal_ussensorparams() const {
  const ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern* p = ussensorparams_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern*>(
      &::pb::us_em::us_em_sensor_parameters_intern::_UsEmSensorParametersIntern_default_instance_);
}
inline const ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern& UsEmParams::ussensorparams() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_params.UsEmParams.usSensorParams)
  return _internal_ussensorparams();
}
inline ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern* UsEmParams::release_ussensorparams() {
  // @@protoc_insertion_point(field_release:pb.us_em.us_em_params.UsEmParams.usSensorParams)
  _has_bits_[0] &= ~0x00000008u;
  ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern* temp = ussensorparams_;
  ussensorparams_ = nullptr;
  return temp;
}
inline ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern* UsEmParams::_internal_mutable_ussensorparams() {
  _has_bits_[0] |= 0x00000008u;
  if (ussensorparams_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern>(GetArenaNoVirtual());
    ussensorparams_ = p;
  }
  return ussensorparams_;
}
inline ::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern* UsEmParams::mutable_ussensorparams() {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_em_params.UsEmParams.usSensorParams)
  return _internal_mutable_ussensorparams();
}
inline void UsEmParams::set_allocated_ussensorparams(::pb::us_em::us_em_sensor_parameters_intern::UsEmSensorParametersIntern* ussensorparams) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(ussensorparams_);
  }
  if (ussensorparams) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      ussensorparams = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, ussensorparams, submessage_arena);
    }
    _has_bits_[0] |= 0x00000008u;
  } else {
    _has_bits_[0] &= ~0x00000008u;
  }
  ussensorparams_ = ussensorparams;
  // @@protoc_insertion_point(field_set_allocated:pb.us_em.us_em_params.UsEmParams.usSensorParams)
}

// -------------------------------------------------------------------

// UsEmParams_array_port

// repeated .pb.us_em.us_em_params.UsEmParams data = 2277;
inline int UsEmParams_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsEmParams_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsEmParams_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_em::us_em_params::UsEmParams* UsEmParams_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_em_params.UsEmParams_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_params::UsEmParams >*
UsEmParams_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_em.us_em_params.UsEmParams_array_port.data)
  return &data_;
}
inline const ::pb::us_em::us_em_params::UsEmParams& UsEmParams_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_em::us_em_params::UsEmParams& UsEmParams_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_params.UsEmParams_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_em::us_em_params::UsEmParams* UsEmParams_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_em::us_em_params::UsEmParams* UsEmParams_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_em.us_em_params.UsEmParams_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_params::UsEmParams >&
UsEmParams_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_em.us_em_params.UsEmParams_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_em_params
}  // namespace us_em
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fem_2fus_5fem_5fparams_2eproto
