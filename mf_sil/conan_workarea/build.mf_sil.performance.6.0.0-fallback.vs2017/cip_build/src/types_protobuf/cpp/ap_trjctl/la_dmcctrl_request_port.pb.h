// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_trjctl/la_dmcctrl_request_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5ftrjctl_2fla_5fdmcctrl_5frequest_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5ftrjctl_2fla_5fdmcctrl_5frequest_5fport_2eproto

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
#include "ap_trjctl/la_dmcctrl_request_interface_type.pb.h"
#include "ap_trjctl/la_dmcctrl_request_source_type.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5ftrjctl_2fla_5fdmcctrl_5frequest_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5ftrjctl_2fla_5fdmcctrl_5frequest_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftrjctl_2fla_5fdmcctrl_5frequest_5fport_2eproto;
namespace pb {
namespace ap_trjctl {
namespace la_dmcctrl_request_port {
class LaDMCCtrlRequestPort;
class LaDMCCtrlRequestPortDefaultTypeInternal;
extern LaDMCCtrlRequestPortDefaultTypeInternal _LaDMCCtrlRequestPort_default_instance_;
class LaDMCCtrlRequestPort_array_port;
class LaDMCCtrlRequestPort_array_portDefaultTypeInternal;
extern LaDMCCtrlRequestPort_array_portDefaultTypeInternal _LaDMCCtrlRequestPort_array_port_default_instance_;
}  // namespace la_dmcctrl_request_port
}  // namespace ap_trjctl
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort* Arena::CreateMaybeMessage<::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort>(Arena*);
template<> ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort_array_port* Arena::CreateMaybeMessage<::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_trjctl {
namespace la_dmcctrl_request_port {

// ===================================================================

class LaDMCCtrlRequestPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort) */ {
 public:
  LaDMCCtrlRequestPort();
  virtual ~LaDMCCtrlRequestPort();

  LaDMCCtrlRequestPort(const LaDMCCtrlRequestPort& from);
  LaDMCCtrlRequestPort(LaDMCCtrlRequestPort&& from) noexcept
    : LaDMCCtrlRequestPort() {
    *this = ::std::move(from);
  }

  inline LaDMCCtrlRequestPort& operator=(const LaDMCCtrlRequestPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline LaDMCCtrlRequestPort& operator=(LaDMCCtrlRequestPort&& from) noexcept {
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
  static const LaDMCCtrlRequestPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LaDMCCtrlRequestPort* internal_default_instance() {
    return reinterpret_cast<const LaDMCCtrlRequestPort*>(
               &_LaDMCCtrlRequestPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LaDMCCtrlRequestPort& a, LaDMCCtrlRequestPort& b) {
    a.Swap(&b);
  }
  inline void Swap(LaDMCCtrlRequestPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LaDMCCtrlRequestPort* New() const final {
    return CreateMaybeMessage<LaDMCCtrlRequestPort>(nullptr);
  }

  LaDMCCtrlRequestPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LaDMCCtrlRequestPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LaDMCCtrlRequestPort& from);
  void MergeFrom(const LaDMCCtrlRequestPort& from);
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
  void InternalSwap(LaDMCCtrlRequestPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftrjctl_2fla_5fdmcctrl_5frequest_5fport_2eproto);
    return ::descriptor_table_ap_5ftrjctl_2fla_5fdmcctrl_5frequest_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kLaDMCCtrlRequestNuFieldNumber = 632,
    kLaDMCCtrlRequestInterfaceNuFieldNumber = 1740,
    kUiVersionNumberFieldNumber = 2124,
    kLaDMCCtrlRequestSourceNuFieldNumber = 2339,
    kSteerWheelAngReqRadFieldNumber = 2606,
    kRearSteerTorqueReqNmFieldNumber = 2765,
    kCurvatureReq1PmFieldNumber = 3031,
    kFrontSteerAngReqRadFieldNumber = 3183,
    kFrontSteerTorqueReqNmFieldNumber = 3580,
    kRearSteerAngReqRadFieldNumber = 3617,
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

  // optional bool laDMCCtrlRequest_nu = 632;
  bool has_ladmcctrlrequest_nu() const;
  private:
  bool _internal_has_ladmcctrlrequest_nu() const;
  public:
  void clear_ladmcctrlrequest_nu();
  bool ladmcctrlrequest_nu() const;
  void set_ladmcctrlrequest_nu(bool value);
  private:
  bool _internal_ladmcctrlrequest_nu() const;
  void _internal_set_ladmcctrlrequest_nu(bool value);
  public:

  // optional .pb.ap_trjctl.la_dmcctrl_request_interface_type.LaDMCCtrlRequestInterfaceType laDMCCtrlRequestInterface_nu = 1740;
  bool has_ladmcctrlrequestinterface_nu() const;
  private:
  bool _internal_has_ladmcctrlrequestinterface_nu() const;
  public:
  void clear_ladmcctrlrequestinterface_nu();
  ::pb::ap_trjctl::la_dmcctrl_request_interface_type::LaDMCCtrlRequestInterfaceType ladmcctrlrequestinterface_nu() const;
  void set_ladmcctrlrequestinterface_nu(::pb::ap_trjctl::la_dmcctrl_request_interface_type::LaDMCCtrlRequestInterfaceType value);
  private:
  ::pb::ap_trjctl::la_dmcctrl_request_interface_type::LaDMCCtrlRequestInterfaceType _internal_ladmcctrlrequestinterface_nu() const;
  void _internal_set_ladmcctrlrequestinterface_nu(::pb::ap_trjctl::la_dmcctrl_request_interface_type::LaDMCCtrlRequestInterfaceType value);
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

  // optional .pb.ap_trjctl.la_dmcctrl_request_source_type.LaDMCCtrlRequestSourceType laDMCCtrlRequestSource_nu = 2339;
  bool has_ladmcctrlrequestsource_nu() const;
  private:
  bool _internal_has_ladmcctrlrequestsource_nu() const;
  public:
  void clear_ladmcctrlrequestsource_nu();
  ::pb::ap_trjctl::la_dmcctrl_request_source_type::LaDMCCtrlRequestSourceType ladmcctrlrequestsource_nu() const;
  void set_ladmcctrlrequestsource_nu(::pb::ap_trjctl::la_dmcctrl_request_source_type::LaDMCCtrlRequestSourceType value);
  private:
  ::pb::ap_trjctl::la_dmcctrl_request_source_type::LaDMCCtrlRequestSourceType _internal_ladmcctrlrequestsource_nu() const;
  void _internal_set_ladmcctrlrequestsource_nu(::pb::ap_trjctl::la_dmcctrl_request_source_type::LaDMCCtrlRequestSourceType value);
  public:

  // optional float steerWheelAngReq_rad = 2606;
  bool has_steerwheelangreq_rad() const;
  private:
  bool _internal_has_steerwheelangreq_rad() const;
  public:
  void clear_steerwheelangreq_rad();
  float steerwheelangreq_rad() const;
  void set_steerwheelangreq_rad(float value);
  private:
  float _internal_steerwheelangreq_rad() const;
  void _internal_set_steerwheelangreq_rad(float value);
  public:

  // optional float rearSteerTorqueReq_Nm = 2765;
  bool has_rearsteertorquereq_nm() const;
  private:
  bool _internal_has_rearsteertorquereq_nm() const;
  public:
  void clear_rearsteertorquereq_nm();
  float rearsteertorquereq_nm() const;
  void set_rearsteertorquereq_nm(float value);
  private:
  float _internal_rearsteertorquereq_nm() const;
  void _internal_set_rearsteertorquereq_nm(float value);
  public:

  // optional float curvatureReq_1pm = 3031;
  bool has_curvaturereq_1pm() const;
  private:
  bool _internal_has_curvaturereq_1pm() const;
  public:
  void clear_curvaturereq_1pm();
  float curvaturereq_1pm() const;
  void set_curvaturereq_1pm(float value);
  private:
  float _internal_curvaturereq_1pm() const;
  void _internal_set_curvaturereq_1pm(float value);
  public:

  // optional float frontSteerAngReq_rad = 3183;
  bool has_frontsteerangreq_rad() const;
  private:
  bool _internal_has_frontsteerangreq_rad() const;
  public:
  void clear_frontsteerangreq_rad();
  float frontsteerangreq_rad() const;
  void set_frontsteerangreq_rad(float value);
  private:
  float _internal_frontsteerangreq_rad() const;
  void _internal_set_frontsteerangreq_rad(float value);
  public:

  // optional float frontSteerTorqueReq_Nm = 3580;
  bool has_frontsteertorquereq_nm() const;
  private:
  bool _internal_has_frontsteertorquereq_nm() const;
  public:
  void clear_frontsteertorquereq_nm();
  float frontsteertorquereq_nm() const;
  void set_frontsteertorquereq_nm(float value);
  private:
  float _internal_frontsteertorquereq_nm() const;
  void _internal_set_frontsteertorquereq_nm(float value);
  public:

  // optional float rearSteerAngReq_rad = 3617;
  bool has_rearsteerangreq_rad() const;
  private:
  bool _internal_has_rearsteerangreq_rad() const;
  public:
  void clear_rearsteerangreq_rad();
  float rearsteerangreq_rad() const;
  void set_rearsteerangreq_rad(float value);
  private:
  float _internal_rearsteerangreq_rad() const;
  void _internal_set_rearsteerangreq_rad(float value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  bool ladmcctrlrequest_nu_;
  int ladmcctrlrequestinterface_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  int ladmcctrlrequestsource_nu_;
  float steerwheelangreq_rad_;
  float rearsteertorquereq_nm_;
  float curvaturereq_1pm_;
  float frontsteerangreq_rad_;
  float frontsteertorquereq_nm_;
  float rearsteerangreq_rad_;
  friend struct ::TableStruct_ap_5ftrjctl_2fla_5fdmcctrl_5frequest_5fport_2eproto;
};
// -------------------------------------------------------------------

class LaDMCCtrlRequestPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort_array_port) */ {
 public:
  LaDMCCtrlRequestPort_array_port();
  virtual ~LaDMCCtrlRequestPort_array_port();

  LaDMCCtrlRequestPort_array_port(const LaDMCCtrlRequestPort_array_port& from);
  LaDMCCtrlRequestPort_array_port(LaDMCCtrlRequestPort_array_port&& from) noexcept
    : LaDMCCtrlRequestPort_array_port() {
    *this = ::std::move(from);
  }

  inline LaDMCCtrlRequestPort_array_port& operator=(const LaDMCCtrlRequestPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline LaDMCCtrlRequestPort_array_port& operator=(LaDMCCtrlRequestPort_array_port&& from) noexcept {
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
  static const LaDMCCtrlRequestPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LaDMCCtrlRequestPort_array_port* internal_default_instance() {
    return reinterpret_cast<const LaDMCCtrlRequestPort_array_port*>(
               &_LaDMCCtrlRequestPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(LaDMCCtrlRequestPort_array_port& a, LaDMCCtrlRequestPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(LaDMCCtrlRequestPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LaDMCCtrlRequestPort_array_port* New() const final {
    return CreateMaybeMessage<LaDMCCtrlRequestPort_array_port>(nullptr);
  }

  LaDMCCtrlRequestPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LaDMCCtrlRequestPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LaDMCCtrlRequestPort_array_port& from);
  void MergeFrom(const LaDMCCtrlRequestPort_array_port& from);
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
  void InternalSwap(LaDMCCtrlRequestPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftrjctl_2fla_5fdmcctrl_5frequest_5fport_2eproto);
    return ::descriptor_table_ap_5ftrjctl_2fla_5fdmcctrl_5frequest_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2648,
  };
  // repeated .pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort data = 2648;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort >*
      mutable_data();
  private:
  const ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort& _internal_data(int index) const;
  ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort* _internal_add_data();
  public:
  const ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort& data(int index) const;
  ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort > data_;
  friend struct ::TableStruct_ap_5ftrjctl_2fla_5fdmcctrl_5frequest_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LaDMCCtrlRequestPort

// optional uint32 uiVersionNumber = 2124;
inline bool LaDMCCtrlRequestPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool LaDMCCtrlRequestPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void LaDMCCtrlRequestPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 LaDMCCtrlRequestPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 LaDMCCtrlRequestPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void LaDMCCtrlRequestPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  uiversionnumber_ = value;
}
inline void LaDMCCtrlRequestPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool LaDMCCtrlRequestPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool LaDMCCtrlRequestPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& LaDMCCtrlRequestPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& LaDMCCtrlRequestPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* LaDMCCtrlRequestPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* LaDMCCtrlRequestPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* LaDMCCtrlRequestPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void LaDMCCtrlRequestPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.sSigHeader)
}

// optional float steerWheelAngReq_rad = 2606;
inline bool LaDMCCtrlRequestPort::_internal_has_steerwheelangreq_rad() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool LaDMCCtrlRequestPort::has_steerwheelangreq_rad() const {
  return _internal_has_steerwheelangreq_rad();
}
inline void LaDMCCtrlRequestPort::clear_steerwheelangreq_rad() {
  steerwheelangreq_rad_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline float LaDMCCtrlRequestPort::_internal_steerwheelangreq_rad() const {
  return steerwheelangreq_rad_;
}
inline float LaDMCCtrlRequestPort::steerwheelangreq_rad() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.steerWheelAngReq_rad)
  return _internal_steerwheelangreq_rad();
}
inline void LaDMCCtrlRequestPort::_internal_set_steerwheelangreq_rad(float value) {
  _has_bits_[0] |= 0x00000020u;
  steerwheelangreq_rad_ = value;
}
inline void LaDMCCtrlRequestPort::set_steerwheelangreq_rad(float value) {
  _internal_set_steerwheelangreq_rad(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.steerWheelAngReq_rad)
}

// optional float frontSteerAngReq_rad = 3183;
inline bool LaDMCCtrlRequestPort::_internal_has_frontsteerangreq_rad() const {
  bool value = (_has_bits_[0] & 0x00000100u) != 0;
  return value;
}
inline bool LaDMCCtrlRequestPort::has_frontsteerangreq_rad() const {
  return _internal_has_frontsteerangreq_rad();
}
inline void LaDMCCtrlRequestPort::clear_frontsteerangreq_rad() {
  frontsteerangreq_rad_ = 0;
  _has_bits_[0] &= ~0x00000100u;
}
inline float LaDMCCtrlRequestPort::_internal_frontsteerangreq_rad() const {
  return frontsteerangreq_rad_;
}
inline float LaDMCCtrlRequestPort::frontsteerangreq_rad() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.frontSteerAngReq_rad)
  return _internal_frontsteerangreq_rad();
}
inline void LaDMCCtrlRequestPort::_internal_set_frontsteerangreq_rad(float value) {
  _has_bits_[0] |= 0x00000100u;
  frontsteerangreq_rad_ = value;
}
inline void LaDMCCtrlRequestPort::set_frontsteerangreq_rad(float value) {
  _internal_set_frontsteerangreq_rad(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.frontSteerAngReq_rad)
}

// optional float rearSteerAngReq_rad = 3617;
inline bool LaDMCCtrlRequestPort::_internal_has_rearsteerangreq_rad() const {
  bool value = (_has_bits_[0] & 0x00000400u) != 0;
  return value;
}
inline bool LaDMCCtrlRequestPort::has_rearsteerangreq_rad() const {
  return _internal_has_rearsteerangreq_rad();
}
inline void LaDMCCtrlRequestPort::clear_rearsteerangreq_rad() {
  rearsteerangreq_rad_ = 0;
  _has_bits_[0] &= ~0x00000400u;
}
inline float LaDMCCtrlRequestPort::_internal_rearsteerangreq_rad() const {
  return rearsteerangreq_rad_;
}
inline float LaDMCCtrlRequestPort::rearsteerangreq_rad() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.rearSteerAngReq_rad)
  return _internal_rearsteerangreq_rad();
}
inline void LaDMCCtrlRequestPort::_internal_set_rearsteerangreq_rad(float value) {
  _has_bits_[0] |= 0x00000400u;
  rearsteerangreq_rad_ = value;
}
inline void LaDMCCtrlRequestPort::set_rearsteerangreq_rad(float value) {
  _internal_set_rearsteerangreq_rad(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.rearSteerAngReq_rad)
}

// optional float frontSteerTorqueReq_Nm = 3580;
inline bool LaDMCCtrlRequestPort::_internal_has_frontsteertorquereq_nm() const {
  bool value = (_has_bits_[0] & 0x00000200u) != 0;
  return value;
}
inline bool LaDMCCtrlRequestPort::has_frontsteertorquereq_nm() const {
  return _internal_has_frontsteertorquereq_nm();
}
inline void LaDMCCtrlRequestPort::clear_frontsteertorquereq_nm() {
  frontsteertorquereq_nm_ = 0;
  _has_bits_[0] &= ~0x00000200u;
}
inline float LaDMCCtrlRequestPort::_internal_frontsteertorquereq_nm() const {
  return frontsteertorquereq_nm_;
}
inline float LaDMCCtrlRequestPort::frontsteertorquereq_nm() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.frontSteerTorqueReq_Nm)
  return _internal_frontsteertorquereq_nm();
}
inline void LaDMCCtrlRequestPort::_internal_set_frontsteertorquereq_nm(float value) {
  _has_bits_[0] |= 0x00000200u;
  frontsteertorquereq_nm_ = value;
}
inline void LaDMCCtrlRequestPort::set_frontsteertorquereq_nm(float value) {
  _internal_set_frontsteertorquereq_nm(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.frontSteerTorqueReq_Nm)
}

// optional float rearSteerTorqueReq_Nm = 2765;
inline bool LaDMCCtrlRequestPort::_internal_has_rearsteertorquereq_nm() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool LaDMCCtrlRequestPort::has_rearsteertorquereq_nm() const {
  return _internal_has_rearsteertorquereq_nm();
}
inline void LaDMCCtrlRequestPort::clear_rearsteertorquereq_nm() {
  rearsteertorquereq_nm_ = 0;
  _has_bits_[0] &= ~0x00000040u;
}
inline float LaDMCCtrlRequestPort::_internal_rearsteertorquereq_nm() const {
  return rearsteertorquereq_nm_;
}
inline float LaDMCCtrlRequestPort::rearsteertorquereq_nm() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.rearSteerTorqueReq_Nm)
  return _internal_rearsteertorquereq_nm();
}
inline void LaDMCCtrlRequestPort::_internal_set_rearsteertorquereq_nm(float value) {
  _has_bits_[0] |= 0x00000040u;
  rearsteertorquereq_nm_ = value;
}
inline void LaDMCCtrlRequestPort::set_rearsteertorquereq_nm(float value) {
  _internal_set_rearsteertorquereq_nm(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.rearSteerTorqueReq_Nm)
}

// optional float curvatureReq_1pm = 3031;
inline bool LaDMCCtrlRequestPort::_internal_has_curvaturereq_1pm() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool LaDMCCtrlRequestPort::has_curvaturereq_1pm() const {
  return _internal_has_curvaturereq_1pm();
}
inline void LaDMCCtrlRequestPort::clear_curvaturereq_1pm() {
  curvaturereq_1pm_ = 0;
  _has_bits_[0] &= ~0x00000080u;
}
inline float LaDMCCtrlRequestPort::_internal_curvaturereq_1pm() const {
  return curvaturereq_1pm_;
}
inline float LaDMCCtrlRequestPort::curvaturereq_1pm() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.curvatureReq_1pm)
  return _internal_curvaturereq_1pm();
}
inline void LaDMCCtrlRequestPort::_internal_set_curvaturereq_1pm(float value) {
  _has_bits_[0] |= 0x00000080u;
  curvaturereq_1pm_ = value;
}
inline void LaDMCCtrlRequestPort::set_curvaturereq_1pm(float value) {
  _internal_set_curvaturereq_1pm(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.curvatureReq_1pm)
}

// optional .pb.ap_trjctl.la_dmcctrl_request_interface_type.LaDMCCtrlRequestInterfaceType laDMCCtrlRequestInterface_nu = 1740;
inline bool LaDMCCtrlRequestPort::_internal_has_ladmcctrlrequestinterface_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool LaDMCCtrlRequestPort::has_ladmcctrlrequestinterface_nu() const {
  return _internal_has_ladmcctrlrequestinterface_nu();
}
inline void LaDMCCtrlRequestPort::clear_ladmcctrlrequestinterface_nu() {
  ladmcctrlrequestinterface_nu_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::ap_trjctl::la_dmcctrl_request_interface_type::LaDMCCtrlRequestInterfaceType LaDMCCtrlRequestPort::_internal_ladmcctrlrequestinterface_nu() const {
  return static_cast< ::pb::ap_trjctl::la_dmcctrl_request_interface_type::LaDMCCtrlRequestInterfaceType >(ladmcctrlrequestinterface_nu_);
}
inline ::pb::ap_trjctl::la_dmcctrl_request_interface_type::LaDMCCtrlRequestInterfaceType LaDMCCtrlRequestPort::ladmcctrlrequestinterface_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.laDMCCtrlRequestInterface_nu)
  return _internal_ladmcctrlrequestinterface_nu();
}
inline void LaDMCCtrlRequestPort::_internal_set_ladmcctrlrequestinterface_nu(::pb::ap_trjctl::la_dmcctrl_request_interface_type::LaDMCCtrlRequestInterfaceType value) {
  assert(::pb::ap_trjctl::la_dmcctrl_request_interface_type::LaDMCCtrlRequestInterfaceType_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  ladmcctrlrequestinterface_nu_ = value;
}
inline void LaDMCCtrlRequestPort::set_ladmcctrlrequestinterface_nu(::pb::ap_trjctl::la_dmcctrl_request_interface_type::LaDMCCtrlRequestInterfaceType value) {
  _internal_set_ladmcctrlrequestinterface_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.laDMCCtrlRequestInterface_nu)
}

// optional .pb.ap_trjctl.la_dmcctrl_request_source_type.LaDMCCtrlRequestSourceType laDMCCtrlRequestSource_nu = 2339;
inline bool LaDMCCtrlRequestPort::_internal_has_ladmcctrlrequestsource_nu() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool LaDMCCtrlRequestPort::has_ladmcctrlrequestsource_nu() const {
  return _internal_has_ladmcctrlrequestsource_nu();
}
inline void LaDMCCtrlRequestPort::clear_ladmcctrlrequestsource_nu() {
  ladmcctrlrequestsource_nu_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::pb::ap_trjctl::la_dmcctrl_request_source_type::LaDMCCtrlRequestSourceType LaDMCCtrlRequestPort::_internal_ladmcctrlrequestsource_nu() const {
  return static_cast< ::pb::ap_trjctl::la_dmcctrl_request_source_type::LaDMCCtrlRequestSourceType >(ladmcctrlrequestsource_nu_);
}
inline ::pb::ap_trjctl::la_dmcctrl_request_source_type::LaDMCCtrlRequestSourceType LaDMCCtrlRequestPort::ladmcctrlrequestsource_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.laDMCCtrlRequestSource_nu)
  return _internal_ladmcctrlrequestsource_nu();
}
inline void LaDMCCtrlRequestPort::_internal_set_ladmcctrlrequestsource_nu(::pb::ap_trjctl::la_dmcctrl_request_source_type::LaDMCCtrlRequestSourceType value) {
  assert(::pb::ap_trjctl::la_dmcctrl_request_source_type::LaDMCCtrlRequestSourceType_IsValid(value));
  _has_bits_[0] |= 0x00000010u;
  ladmcctrlrequestsource_nu_ = value;
}
inline void LaDMCCtrlRequestPort::set_ladmcctrlrequestsource_nu(::pb::ap_trjctl::la_dmcctrl_request_source_type::LaDMCCtrlRequestSourceType value) {
  _internal_set_ladmcctrlrequestsource_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.laDMCCtrlRequestSource_nu)
}

// optional bool laDMCCtrlRequest_nu = 632;
inline bool LaDMCCtrlRequestPort::_internal_has_ladmcctrlrequest_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool LaDMCCtrlRequestPort::has_ladmcctrlrequest_nu() const {
  return _internal_has_ladmcctrlrequest_nu();
}
inline void LaDMCCtrlRequestPort::clear_ladmcctrlrequest_nu() {
  ladmcctrlrequest_nu_ = false;
  _has_bits_[0] &= ~0x00000002u;
}
inline bool LaDMCCtrlRequestPort::_internal_ladmcctrlrequest_nu() const {
  return ladmcctrlrequest_nu_;
}
inline bool LaDMCCtrlRequestPort::ladmcctrlrequest_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.laDMCCtrlRequest_nu)
  return _internal_ladmcctrlrequest_nu();
}
inline void LaDMCCtrlRequestPort::_internal_set_ladmcctrlrequest_nu(bool value) {
  _has_bits_[0] |= 0x00000002u;
  ladmcctrlrequest_nu_ = value;
}
inline void LaDMCCtrlRequestPort::set_ladmcctrlrequest_nu(bool value) {
  _internal_set_ladmcctrlrequest_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort.laDMCCtrlRequest_nu)
}

// -------------------------------------------------------------------

// LaDMCCtrlRequestPort_array_port

// repeated .pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort data = 2648;
inline int LaDMCCtrlRequestPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int LaDMCCtrlRequestPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void LaDMCCtrlRequestPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort* LaDMCCtrlRequestPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort >*
LaDMCCtrlRequestPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort_array_port.data)
  return &data_;
}
inline const ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort& LaDMCCtrlRequestPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort& LaDMCCtrlRequestPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort* LaDMCCtrlRequestPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort* LaDMCCtrlRequestPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_trjctl::la_dmcctrl_request_port::LaDMCCtrlRequestPort >&
LaDMCCtrlRequestPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_trjctl.la_dmcctrl_request_port.LaDMCCtrlRequestPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace la_dmcctrl_request_port
}  // namespace ap_trjctl
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5ftrjctl_2fla_5fdmcctrl_5frequest_5fport_2eproto
