// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_trjctl/mf_control_t_long_ctrl_req.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5ftrjctl_2fmf_5fcontrol_5ft_5flong_5fctrl_5freq_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5ftrjctl_2fmf_5fcontrol_5ft_5flong_5fctrl_5freq_2eproto

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
#include "ap_trjctl/mf_control_te_long_ctrl_mode.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5ftrjctl_2fmf_5fcontrol_5ft_5flong_5fctrl_5freq_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5ftrjctl_2fmf_5fcontrol_5ft_5flong_5fctrl_5freq_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftrjctl_2fmf_5fcontrol_5ft_5flong_5fctrl_5freq_2eproto;
namespace pb {
namespace ap_trjctl {
namespace mf_control_t_long_ctrl_req {
class MF_CONTROL_t_LongCtrlReq;
class MF_CONTROL_t_LongCtrlReqDefaultTypeInternal;
extern MF_CONTROL_t_LongCtrlReqDefaultTypeInternal _MF_CONTROL_t_LongCtrlReq_default_instance_;
class MF_CONTROL_t_LongCtrlReq_array_port;
class MF_CONTROL_t_LongCtrlReq_array_portDefaultTypeInternal;
extern MF_CONTROL_t_LongCtrlReq_array_portDefaultTypeInternal _MF_CONTROL_t_LongCtrlReq_array_port_default_instance_;
}  // namespace mf_control_t_long_ctrl_req
}  // namespace ap_trjctl
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq* Arena::CreateMaybeMessage<::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq>(Arena*);
template<> ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq_array_port* Arena::CreateMaybeMessage<::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_trjctl {
namespace mf_control_t_long_ctrl_req {

// ===================================================================

class MF_CONTROL_t_LongCtrlReq :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq) */ {
 public:
  MF_CONTROL_t_LongCtrlReq();
  virtual ~MF_CONTROL_t_LongCtrlReq();

  MF_CONTROL_t_LongCtrlReq(const MF_CONTROL_t_LongCtrlReq& from);
  MF_CONTROL_t_LongCtrlReq(MF_CONTROL_t_LongCtrlReq&& from) noexcept
    : MF_CONTROL_t_LongCtrlReq() {
    *this = ::std::move(from);
  }

  inline MF_CONTROL_t_LongCtrlReq& operator=(const MF_CONTROL_t_LongCtrlReq& from) {
    CopyFrom(from);
    return *this;
  }
  inline MF_CONTROL_t_LongCtrlReq& operator=(MF_CONTROL_t_LongCtrlReq&& from) noexcept {
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
  static const MF_CONTROL_t_LongCtrlReq& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MF_CONTROL_t_LongCtrlReq* internal_default_instance() {
    return reinterpret_cast<const MF_CONTROL_t_LongCtrlReq*>(
               &_MF_CONTROL_t_LongCtrlReq_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MF_CONTROL_t_LongCtrlReq& a, MF_CONTROL_t_LongCtrlReq& b) {
    a.Swap(&b);
  }
  inline void Swap(MF_CONTROL_t_LongCtrlReq* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MF_CONTROL_t_LongCtrlReq* New() const final {
    return CreateMaybeMessage<MF_CONTROL_t_LongCtrlReq>(nullptr);
  }

  MF_CONTROL_t_LongCtrlReq* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MF_CONTROL_t_LongCtrlReq>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MF_CONTROL_t_LongCtrlReq& from);
  void MergeFrom(const MF_CONTROL_t_LongCtrlReq& from);
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
  void InternalSwap(MF_CONTROL_t_LongCtrlReq* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftrjctl_2fmf_5fcontrol_5ft_5flong_5fctrl_5freq_2eproto);
    return ::descriptor_table_ap_5ftrjctl_2fmf_5fcontrol_5ft_5flong_5fctrl_5freq_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kDistanceToStopReqMFieldNumber = 3322,
    kLongCtrlModeNuFieldNumber = 10,
    kVeloLimMaxMpsFieldNumber = 1919,
    kUiVersionNumberFieldNumber = 2124,
    kAccelReqMps2FieldNumber = 2960,
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

  // optional float distanceToStopReq_m = 3322;
  bool has_distancetostopreq_m() const;
  private:
  bool _internal_has_distancetostopreq_m() const;
  public:
  void clear_distancetostopreq_m();
  float distancetostopreq_m() const;
  void set_distancetostopreq_m(float value);
  private:
  float _internal_distancetostopreq_m() const;
  void _internal_set_distancetostopreq_m(float value);
  public:

  // optional .pb.ap_trjctl.mf_control_te_long_ctrl_mode.MF_CONTROL_te_LongCtrlMode longCtrlMode_nu = 10;
  bool has_longctrlmode_nu() const;
  private:
  bool _internal_has_longctrlmode_nu() const;
  public:
  void clear_longctrlmode_nu();
  ::pb::ap_trjctl::mf_control_te_long_ctrl_mode::MF_CONTROL_te_LongCtrlMode longctrlmode_nu() const;
  void set_longctrlmode_nu(::pb::ap_trjctl::mf_control_te_long_ctrl_mode::MF_CONTROL_te_LongCtrlMode value);
  private:
  ::pb::ap_trjctl::mf_control_te_long_ctrl_mode::MF_CONTROL_te_LongCtrlMode _internal_longctrlmode_nu() const;
  void _internal_set_longctrlmode_nu(::pb::ap_trjctl::mf_control_te_long_ctrl_mode::MF_CONTROL_te_LongCtrlMode value);
  public:

  // optional float veloLimMax_mps = 1919;
  bool has_velolimmax_mps() const;
  private:
  bool _internal_has_velolimmax_mps() const;
  public:
  void clear_velolimmax_mps();
  float velolimmax_mps() const;
  void set_velolimmax_mps(float value);
  private:
  float _internal_velolimmax_mps() const;
  void _internal_set_velolimmax_mps(float value);
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

  // optional float accelReq_mps2 = 2960;
  bool has_accelreq_mps2() const;
  private:
  bool _internal_has_accelreq_mps2() const;
  public:
  void clear_accelreq_mps2();
  float accelreq_mps2() const;
  void set_accelreq_mps2(float value);
  private:
  float _internal_accelreq_mps2() const;
  void _internal_set_accelreq_mps2(float value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  float distancetostopreq_m_;
  int longctrlmode_nu_;
  float velolimmax_mps_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  float accelreq_mps2_;
  friend struct ::TableStruct_ap_5ftrjctl_2fmf_5fcontrol_5ft_5flong_5fctrl_5freq_2eproto;
};
// -------------------------------------------------------------------

class MF_CONTROL_t_LongCtrlReq_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq_array_port) */ {
 public:
  MF_CONTROL_t_LongCtrlReq_array_port();
  virtual ~MF_CONTROL_t_LongCtrlReq_array_port();

  MF_CONTROL_t_LongCtrlReq_array_port(const MF_CONTROL_t_LongCtrlReq_array_port& from);
  MF_CONTROL_t_LongCtrlReq_array_port(MF_CONTROL_t_LongCtrlReq_array_port&& from) noexcept
    : MF_CONTROL_t_LongCtrlReq_array_port() {
    *this = ::std::move(from);
  }

  inline MF_CONTROL_t_LongCtrlReq_array_port& operator=(const MF_CONTROL_t_LongCtrlReq_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline MF_CONTROL_t_LongCtrlReq_array_port& operator=(MF_CONTROL_t_LongCtrlReq_array_port&& from) noexcept {
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
  static const MF_CONTROL_t_LongCtrlReq_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MF_CONTROL_t_LongCtrlReq_array_port* internal_default_instance() {
    return reinterpret_cast<const MF_CONTROL_t_LongCtrlReq_array_port*>(
               &_MF_CONTROL_t_LongCtrlReq_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(MF_CONTROL_t_LongCtrlReq_array_port& a, MF_CONTROL_t_LongCtrlReq_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(MF_CONTROL_t_LongCtrlReq_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MF_CONTROL_t_LongCtrlReq_array_port* New() const final {
    return CreateMaybeMessage<MF_CONTROL_t_LongCtrlReq_array_port>(nullptr);
  }

  MF_CONTROL_t_LongCtrlReq_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MF_CONTROL_t_LongCtrlReq_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MF_CONTROL_t_LongCtrlReq_array_port& from);
  void MergeFrom(const MF_CONTROL_t_LongCtrlReq_array_port& from);
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
  void InternalSwap(MF_CONTROL_t_LongCtrlReq_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftrjctl_2fmf_5fcontrol_5ft_5flong_5fctrl_5freq_2eproto);
    return ::descriptor_table_ap_5ftrjctl_2fmf_5fcontrol_5ft_5flong_5fctrl_5freq_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3600,
  };
  // repeated .pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq data = 3600;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq >*
      mutable_data();
  private:
  const ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq& _internal_data(int index) const;
  ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq* _internal_add_data();
  public:
  const ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq& data(int index) const;
  ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq > data_;
  friend struct ::TableStruct_ap_5ftrjctl_2fmf_5fcontrol_5ft_5flong_5fctrl_5freq_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MF_CONTROL_t_LongCtrlReq

// optional uint32 uiVersionNumber = 2124;
inline bool MF_CONTROL_t_LongCtrlReq::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool MF_CONTROL_t_LongCtrlReq::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void MF_CONTROL_t_LongCtrlReq::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_CONTROL_t_LongCtrlReq::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_CONTROL_t_LongCtrlReq::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void MF_CONTROL_t_LongCtrlReq::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  uiversionnumber_ = value;
}
inline void MF_CONTROL_t_LongCtrlReq::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool MF_CONTROL_t_LongCtrlReq::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool MF_CONTROL_t_LongCtrlReq::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& MF_CONTROL_t_LongCtrlReq::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& MF_CONTROL_t_LongCtrlReq::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* MF_CONTROL_t_LongCtrlReq::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* MF_CONTROL_t_LongCtrlReq::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* MF_CONTROL_t_LongCtrlReq::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void MF_CONTROL_t_LongCtrlReq::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.sSigHeader)
}

// optional float distanceToStopReq_m = 3322;
inline bool MF_CONTROL_t_LongCtrlReq::_internal_has_distancetostopreq_m() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool MF_CONTROL_t_LongCtrlReq::has_distancetostopreq_m() const {
  return _internal_has_distancetostopreq_m();
}
inline void MF_CONTROL_t_LongCtrlReq::clear_distancetostopreq_m() {
  distancetostopreq_m_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline float MF_CONTROL_t_LongCtrlReq::_internal_distancetostopreq_m() const {
  return distancetostopreq_m_;
}
inline float MF_CONTROL_t_LongCtrlReq::distancetostopreq_m() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.distanceToStopReq_m)
  return _internal_distancetostopreq_m();
}
inline void MF_CONTROL_t_LongCtrlReq::_internal_set_distancetostopreq_m(float value) {
  _has_bits_[0] |= 0x00000002u;
  distancetostopreq_m_ = value;
}
inline void MF_CONTROL_t_LongCtrlReq::set_distancetostopreq_m(float value) {
  _internal_set_distancetostopreq_m(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.distanceToStopReq_m)
}

// optional float veloLimMax_mps = 1919;
inline bool MF_CONTROL_t_LongCtrlReq::_internal_has_velolimmax_mps() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool MF_CONTROL_t_LongCtrlReq::has_velolimmax_mps() const {
  return _internal_has_velolimmax_mps();
}
inline void MF_CONTROL_t_LongCtrlReq::clear_velolimmax_mps() {
  velolimmax_mps_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float MF_CONTROL_t_LongCtrlReq::_internal_velolimmax_mps() const {
  return velolimmax_mps_;
}
inline float MF_CONTROL_t_LongCtrlReq::velolimmax_mps() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.veloLimMax_mps)
  return _internal_velolimmax_mps();
}
inline void MF_CONTROL_t_LongCtrlReq::_internal_set_velolimmax_mps(float value) {
  _has_bits_[0] |= 0x00000008u;
  velolimmax_mps_ = value;
}
inline void MF_CONTROL_t_LongCtrlReq::set_velolimmax_mps(float value) {
  _internal_set_velolimmax_mps(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.veloLimMax_mps)
}

// optional float accelReq_mps2 = 2960;
inline bool MF_CONTROL_t_LongCtrlReq::_internal_has_accelreq_mps2() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool MF_CONTROL_t_LongCtrlReq::has_accelreq_mps2() const {
  return _internal_has_accelreq_mps2();
}
inline void MF_CONTROL_t_LongCtrlReq::clear_accelreq_mps2() {
  accelreq_mps2_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline float MF_CONTROL_t_LongCtrlReq::_internal_accelreq_mps2() const {
  return accelreq_mps2_;
}
inline float MF_CONTROL_t_LongCtrlReq::accelreq_mps2() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.accelReq_mps2)
  return _internal_accelreq_mps2();
}
inline void MF_CONTROL_t_LongCtrlReq::_internal_set_accelreq_mps2(float value) {
  _has_bits_[0] |= 0x00000020u;
  accelreq_mps2_ = value;
}
inline void MF_CONTROL_t_LongCtrlReq::set_accelreq_mps2(float value) {
  _internal_set_accelreq_mps2(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.accelReq_mps2)
}

// optional .pb.ap_trjctl.mf_control_te_long_ctrl_mode.MF_CONTROL_te_LongCtrlMode longCtrlMode_nu = 10;
inline bool MF_CONTROL_t_LongCtrlReq::_internal_has_longctrlmode_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool MF_CONTROL_t_LongCtrlReq::has_longctrlmode_nu() const {
  return _internal_has_longctrlmode_nu();
}
inline void MF_CONTROL_t_LongCtrlReq::clear_longctrlmode_nu() {
  longctrlmode_nu_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::ap_trjctl::mf_control_te_long_ctrl_mode::MF_CONTROL_te_LongCtrlMode MF_CONTROL_t_LongCtrlReq::_internal_longctrlmode_nu() const {
  return static_cast< ::pb::ap_trjctl::mf_control_te_long_ctrl_mode::MF_CONTROL_te_LongCtrlMode >(longctrlmode_nu_);
}
inline ::pb::ap_trjctl::mf_control_te_long_ctrl_mode::MF_CONTROL_te_LongCtrlMode MF_CONTROL_t_LongCtrlReq::longctrlmode_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.longCtrlMode_nu)
  return _internal_longctrlmode_nu();
}
inline void MF_CONTROL_t_LongCtrlReq::_internal_set_longctrlmode_nu(::pb::ap_trjctl::mf_control_te_long_ctrl_mode::MF_CONTROL_te_LongCtrlMode value) {
  assert(::pb::ap_trjctl::mf_control_te_long_ctrl_mode::MF_CONTROL_te_LongCtrlMode_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  longctrlmode_nu_ = value;
}
inline void MF_CONTROL_t_LongCtrlReq::set_longctrlmode_nu(::pb::ap_trjctl::mf_control_te_long_ctrl_mode::MF_CONTROL_te_LongCtrlMode value) {
  _internal_set_longctrlmode_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq.longCtrlMode_nu)
}

// -------------------------------------------------------------------

// MF_CONTROL_t_LongCtrlReq_array_port

// repeated .pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq data = 3600;
inline int MF_CONTROL_t_LongCtrlReq_array_port::_internal_data_size() const {
  return data_.size();
}
inline int MF_CONTROL_t_LongCtrlReq_array_port::data_size() const {
  return _internal_data_size();
}
inline void MF_CONTROL_t_LongCtrlReq_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq* MF_CONTROL_t_LongCtrlReq_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq >*
MF_CONTROL_t_LongCtrlReq_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq_array_port.data)
  return &data_;
}
inline const ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq& MF_CONTROL_t_LongCtrlReq_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq& MF_CONTROL_t_LongCtrlReq_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq* MF_CONTROL_t_LongCtrlReq_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq* MF_CONTROL_t_LongCtrlReq_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::mf_control_t_long_ctrl_req::MF_CONTROL_t_LongCtrlReq >&
MF_CONTROL_t_LongCtrlReq_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_trjctl.mf_control_t_long_ctrl_req.MF_CONTROL_t_LongCtrlReq_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace mf_control_t_long_ctrl_req
}  // namespace ap_trjctl
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5ftrjctl_2fmf_5fcontrol_5ft_5flong_5fctrl_5freq_2eproto