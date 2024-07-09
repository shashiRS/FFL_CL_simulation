// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_common/fc_trjpla_vehicle_params.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fcommon_2ffc_5ftrjpla_5fvehicle_5fparams_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fcommon_2ffc_5ftrjpla_5fvehicle_5fparams_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ap_5fcommon_2ffc_5ftrjpla_5fvehicle_5fparams_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fcommon_2ffc_5ftrjpla_5fvehicle_5fparams_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fcommon_2ffc_5ftrjpla_5fvehicle_5fparams_2eproto;
namespace pb {
namespace ap_common {
namespace fc_trjpla_vehicle_params {
class FC_TRJPLA_Vehicle_Params;
class FC_TRJPLA_Vehicle_ParamsDefaultTypeInternal;
extern FC_TRJPLA_Vehicle_ParamsDefaultTypeInternal _FC_TRJPLA_Vehicle_Params_default_instance_;
class FC_TRJPLA_Vehicle_Params_array_port;
class FC_TRJPLA_Vehicle_Params_array_portDefaultTypeInternal;
extern FC_TRJPLA_Vehicle_Params_array_portDefaultTypeInternal _FC_TRJPLA_Vehicle_Params_array_port_default_instance_;
}  // namespace fc_trjpla_vehicle_params
}  // namespace ap_common
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params* Arena::CreateMaybeMessage<::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params>(Arena*);
template<> ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params_array_port* Arena::CreateMaybeMessage<::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_common {
namespace fc_trjpla_vehicle_params {

// ===================================================================

class FC_TRJPLA_Vehicle_Params :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params) */ {
 public:
  FC_TRJPLA_Vehicle_Params();
  virtual ~FC_TRJPLA_Vehicle_Params();

  FC_TRJPLA_Vehicle_Params(const FC_TRJPLA_Vehicle_Params& from);
  FC_TRJPLA_Vehicle_Params(FC_TRJPLA_Vehicle_Params&& from) noexcept
    : FC_TRJPLA_Vehicle_Params() {
    *this = ::std::move(from);
  }

  inline FC_TRJPLA_Vehicle_Params& operator=(const FC_TRJPLA_Vehicle_Params& from) {
    CopyFrom(from);
    return *this;
  }
  inline FC_TRJPLA_Vehicle_Params& operator=(FC_TRJPLA_Vehicle_Params&& from) noexcept {
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
  static const FC_TRJPLA_Vehicle_Params& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const FC_TRJPLA_Vehicle_Params* internal_default_instance() {
    return reinterpret_cast<const FC_TRJPLA_Vehicle_Params*>(
               &_FC_TRJPLA_Vehicle_Params_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(FC_TRJPLA_Vehicle_Params& a, FC_TRJPLA_Vehicle_Params& b) {
    a.Swap(&b);
  }
  inline void Swap(FC_TRJPLA_Vehicle_Params* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline FC_TRJPLA_Vehicle_Params* New() const final {
    return CreateMaybeMessage<FC_TRJPLA_Vehicle_Params>(nullptr);
  }

  FC_TRJPLA_Vehicle_Params* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<FC_TRJPLA_Vehicle_Params>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const FC_TRJPLA_Vehicle_Params& from);
  void MergeFrom(const FC_TRJPLA_Vehicle_Params& from);
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
  void InternalSwap(FC_TRJPLA_Vehicle_Params* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fcommon_2ffc_5ftrjpla_5fvehicle_5fparams_2eproto);
    return ::descriptor_table_ap_5fcommon_2ffc_5ftrjpla_5fvehicle_5fparams_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kAPVMINSAFETYDISTMFieldNumber = 2866,
    kAPVMAXROBUSTDISTMFieldNumber = 1469,
    kUiVersionNumberFieldNumber = 2124,
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

  // optional float AP_V_MIN_SAFETY_DIST_M = 2866;
  bool has_ap_v_min_safety_dist_m() const;
  private:
  bool _internal_has_ap_v_min_safety_dist_m() const;
  public:
  void clear_ap_v_min_safety_dist_m();
  float ap_v_min_safety_dist_m() const;
  void set_ap_v_min_safety_dist_m(float value);
  private:
  float _internal_ap_v_min_safety_dist_m() const;
  void _internal_set_ap_v_min_safety_dist_m(float value);
  public:

  // optional float AP_V_MAX_ROBUST_DIST_M = 1469;
  bool has_ap_v_max_robust_dist_m() const;
  private:
  bool _internal_has_ap_v_max_robust_dist_m() const;
  public:
  void clear_ap_v_max_robust_dist_m();
  float ap_v_max_robust_dist_m() const;
  void set_ap_v_max_robust_dist_m(float value);
  private:
  float _internal_ap_v_max_robust_dist_m() const;
  void _internal_set_ap_v_max_robust_dist_m(float value);
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

  // @@protoc_insertion_point(class_scope:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  float ap_v_min_safety_dist_m_;
  float ap_v_max_robust_dist_m_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  friend struct ::TableStruct_ap_5fcommon_2ffc_5ftrjpla_5fvehicle_5fparams_2eproto;
};
// -------------------------------------------------------------------

class FC_TRJPLA_Vehicle_Params_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params_array_port) */ {
 public:
  FC_TRJPLA_Vehicle_Params_array_port();
  virtual ~FC_TRJPLA_Vehicle_Params_array_port();

  FC_TRJPLA_Vehicle_Params_array_port(const FC_TRJPLA_Vehicle_Params_array_port& from);
  FC_TRJPLA_Vehicle_Params_array_port(FC_TRJPLA_Vehicle_Params_array_port&& from) noexcept
    : FC_TRJPLA_Vehicle_Params_array_port() {
    *this = ::std::move(from);
  }

  inline FC_TRJPLA_Vehicle_Params_array_port& operator=(const FC_TRJPLA_Vehicle_Params_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline FC_TRJPLA_Vehicle_Params_array_port& operator=(FC_TRJPLA_Vehicle_Params_array_port&& from) noexcept {
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
  static const FC_TRJPLA_Vehicle_Params_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const FC_TRJPLA_Vehicle_Params_array_port* internal_default_instance() {
    return reinterpret_cast<const FC_TRJPLA_Vehicle_Params_array_port*>(
               &_FC_TRJPLA_Vehicle_Params_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(FC_TRJPLA_Vehicle_Params_array_port& a, FC_TRJPLA_Vehicle_Params_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(FC_TRJPLA_Vehicle_Params_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline FC_TRJPLA_Vehicle_Params_array_port* New() const final {
    return CreateMaybeMessage<FC_TRJPLA_Vehicle_Params_array_port>(nullptr);
  }

  FC_TRJPLA_Vehicle_Params_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<FC_TRJPLA_Vehicle_Params_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const FC_TRJPLA_Vehicle_Params_array_port& from);
  void MergeFrom(const FC_TRJPLA_Vehicle_Params_array_port& from);
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
  void InternalSwap(FC_TRJPLA_Vehicle_Params_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fcommon_2ffc_5ftrjpla_5fvehicle_5fparams_2eproto);
    return ::descriptor_table_ap_5fcommon_2ffc_5ftrjpla_5fvehicle_5fparams_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3581,
  };
  // repeated .pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params data = 3581;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params >*
      mutable_data();
  private:
  const ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params& _internal_data(int index) const;
  ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params* _internal_add_data();
  public:
  const ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params& data(int index) const;
  ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params > data_;
  friend struct ::TableStruct_ap_5fcommon_2ffc_5ftrjpla_5fvehicle_5fparams_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// FC_TRJPLA_Vehicle_Params

// optional uint32 uiVersionNumber = 2124;
inline bool FC_TRJPLA_Vehicle_Params::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool FC_TRJPLA_Vehicle_Params::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void FC_TRJPLA_Vehicle_Params::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 FC_TRJPLA_Vehicle_Params::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 FC_TRJPLA_Vehicle_Params::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void FC_TRJPLA_Vehicle_Params::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  uiversionnumber_ = value;
}
inline void FC_TRJPLA_Vehicle_Params::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool FC_TRJPLA_Vehicle_Params::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool FC_TRJPLA_Vehicle_Params::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& FC_TRJPLA_Vehicle_Params::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& FC_TRJPLA_Vehicle_Params::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* FC_TRJPLA_Vehicle_Params::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* FC_TRJPLA_Vehicle_Params::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* FC_TRJPLA_Vehicle_Params::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void FC_TRJPLA_Vehicle_Params::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.sSigHeader)
}

// optional float AP_V_MIN_SAFETY_DIST_M = 2866;
inline bool FC_TRJPLA_Vehicle_Params::_internal_has_ap_v_min_safety_dist_m() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool FC_TRJPLA_Vehicle_Params::has_ap_v_min_safety_dist_m() const {
  return _internal_has_ap_v_min_safety_dist_m();
}
inline void FC_TRJPLA_Vehicle_Params::clear_ap_v_min_safety_dist_m() {
  ap_v_min_safety_dist_m_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline float FC_TRJPLA_Vehicle_Params::_internal_ap_v_min_safety_dist_m() const {
  return ap_v_min_safety_dist_m_;
}
inline float FC_TRJPLA_Vehicle_Params::ap_v_min_safety_dist_m() const {
  // @@protoc_insertion_point(field_get:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.AP_V_MIN_SAFETY_DIST_M)
  return _internal_ap_v_min_safety_dist_m();
}
inline void FC_TRJPLA_Vehicle_Params::_internal_set_ap_v_min_safety_dist_m(float value) {
  _has_bits_[0] |= 0x00000002u;
  ap_v_min_safety_dist_m_ = value;
}
inline void FC_TRJPLA_Vehicle_Params::set_ap_v_min_safety_dist_m(float value) {
  _internal_set_ap_v_min_safety_dist_m(value);
  // @@protoc_insertion_point(field_set:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.AP_V_MIN_SAFETY_DIST_M)
}

// optional float AP_V_MAX_ROBUST_DIST_M = 1469;
inline bool FC_TRJPLA_Vehicle_Params::_internal_has_ap_v_max_robust_dist_m() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool FC_TRJPLA_Vehicle_Params::has_ap_v_max_robust_dist_m() const {
  return _internal_has_ap_v_max_robust_dist_m();
}
inline void FC_TRJPLA_Vehicle_Params::clear_ap_v_max_robust_dist_m() {
  ap_v_max_robust_dist_m_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float FC_TRJPLA_Vehicle_Params::_internal_ap_v_max_robust_dist_m() const {
  return ap_v_max_robust_dist_m_;
}
inline float FC_TRJPLA_Vehicle_Params::ap_v_max_robust_dist_m() const {
  // @@protoc_insertion_point(field_get:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.AP_V_MAX_ROBUST_DIST_M)
  return _internal_ap_v_max_robust_dist_m();
}
inline void FC_TRJPLA_Vehicle_Params::_internal_set_ap_v_max_robust_dist_m(float value) {
  _has_bits_[0] |= 0x00000004u;
  ap_v_max_robust_dist_m_ = value;
}
inline void FC_TRJPLA_Vehicle_Params::set_ap_v_max_robust_dist_m(float value) {
  _internal_set_ap_v_max_robust_dist_m(value);
  // @@protoc_insertion_point(field_set:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.AP_V_MAX_ROBUST_DIST_M)
}

// -------------------------------------------------------------------

// FC_TRJPLA_Vehicle_Params_array_port

// repeated .pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params data = 3581;
inline int FC_TRJPLA_Vehicle_Params_array_port::_internal_data_size() const {
  return data_.size();
}
inline int FC_TRJPLA_Vehicle_Params_array_port::data_size() const {
  return _internal_data_size();
}
inline void FC_TRJPLA_Vehicle_Params_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params* FC_TRJPLA_Vehicle_Params_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params >*
FC_TRJPLA_Vehicle_Params_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params_array_port.data)
  return &data_;
}
inline const ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params& FC_TRJPLA_Vehicle_Params_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params& FC_TRJPLA_Vehicle_Params_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params* FC_TRJPLA_Vehicle_Params_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params* FC_TRJPLA_Vehicle_Params_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_common::fc_trjpla_vehicle_params::FC_TRJPLA_Vehicle_Params >&
FC_TRJPLA_Vehicle_Params_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace fc_trjpla_vehicle_params
}  // namespace ap_common
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fcommon_2ffc_5ftrjpla_5fvehicle_5fparams_2eproto