// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: pdcp/pdcpdriving_tube_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_pdcp_2fpdcpdriving_5ftube_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_pdcp_2fpdcpdriving_5ftube_5fport_2eproto

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
#include "cml/vec2_df_pod.pb.h"
#include "pdcp/drv_tube_display.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_pdcp_2fpdcpdriving_5ftube_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_pdcp_2fpdcpdriving_5ftube_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_pdcp_2fpdcpdriving_5ftube_5fport_2eproto;
namespace pb {
namespace pdcp {
namespace pdcpdriving_tube_port {
class PDCPDrivingTubePort;
class PDCPDrivingTubePortDefaultTypeInternal;
extern PDCPDrivingTubePortDefaultTypeInternal _PDCPDrivingTubePort_default_instance_;
class PDCPDrivingTubePort_array_port;
class PDCPDrivingTubePort_array_portDefaultTypeInternal;
extern PDCPDrivingTubePort_array_portDefaultTypeInternal _PDCPDrivingTubePort_array_port_default_instance_;
}  // namespace pdcpdriving_tube_port
}  // namespace pdcp
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort* Arena::CreateMaybeMessage<::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort>(Arena*);
template<> ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort_array_port* Arena::CreateMaybeMessage<::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace pdcp {
namespace pdcpdriving_tube_port {

// ===================================================================

class PDCPDrivingTubePort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort) */ {
 public:
  PDCPDrivingTubePort();
  virtual ~PDCPDrivingTubePort();

  PDCPDrivingTubePort(const PDCPDrivingTubePort& from);
  PDCPDrivingTubePort(PDCPDrivingTubePort&& from) noexcept
    : PDCPDrivingTubePort() {
    *this = ::std::move(from);
  }

  inline PDCPDrivingTubePort& operator=(const PDCPDrivingTubePort& from) {
    CopyFrom(from);
    return *this;
  }
  inline PDCPDrivingTubePort& operator=(PDCPDrivingTubePort&& from) noexcept {
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
  static const PDCPDrivingTubePort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PDCPDrivingTubePort* internal_default_instance() {
    return reinterpret_cast<const PDCPDrivingTubePort*>(
               &_PDCPDrivingTubePort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PDCPDrivingTubePort& a, PDCPDrivingTubePort& b) {
    a.Swap(&b);
  }
  inline void Swap(PDCPDrivingTubePort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PDCPDrivingTubePort* New() const final {
    return CreateMaybeMessage<PDCPDrivingTubePort>(nullptr);
  }

  PDCPDrivingTubePort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PDCPDrivingTubePort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PDCPDrivingTubePort& from);
  void MergeFrom(const PDCPDrivingTubePort& from);
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
  void InternalSwap(PDCPDrivingTubePort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_pdcp_2fpdcpdriving_5ftube_5fport_2eproto);
    return ::descriptor_table_pdcp_2fpdcpdriving_5ftube_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kTurningCircleCenterNuFieldNumber = 2688,
    kRearRadiusMFieldNumber = 3176,
    kFrontRadiusMFieldNumber = 380,
    kStraightDrvTubeNuFieldNumber = 1575,
    kUiVersionNumberFieldNumber = 2124,
    kDrvTubeDisplayNuFieldNumber = 3130,
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

  // optional .pb.cml.vec2_df_pod.Vec2Df_POD turningCircleCenter_nu = 2688;
  bool has_turningcirclecenter_nu() const;
  private:
  bool _internal_has_turningcirclecenter_nu() const;
  public:
  void clear_turningcirclecenter_nu();
  const ::pb::cml::vec2_df_pod::Vec2Df_POD& turningcirclecenter_nu() const;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* release_turningcirclecenter_nu();
  ::pb::cml::vec2_df_pod::Vec2Df_POD* mutable_turningcirclecenter_nu();
  void set_allocated_turningcirclecenter_nu(::pb::cml::vec2_df_pod::Vec2Df_POD* turningcirclecenter_nu);
  private:
  const ::pb::cml::vec2_df_pod::Vec2Df_POD& _internal_turningcirclecenter_nu() const;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* _internal_mutable_turningcirclecenter_nu();
  public:

  // optional float rearRadius_m = 3176;
  bool has_rearradius_m() const;
  private:
  bool _internal_has_rearradius_m() const;
  public:
  void clear_rearradius_m();
  float rearradius_m() const;
  void set_rearradius_m(float value);
  private:
  float _internal_rearradius_m() const;
  void _internal_set_rearradius_m(float value);
  public:

  // optional float frontRadius_m = 380;
  bool has_frontradius_m() const;
  private:
  bool _internal_has_frontradius_m() const;
  public:
  void clear_frontradius_m();
  float frontradius_m() const;
  void set_frontradius_m(float value);
  private:
  float _internal_frontradius_m() const;
  void _internal_set_frontradius_m(float value);
  public:

  // optional bool straightDrvTube_nu = 1575;
  bool has_straightdrvtube_nu() const;
  private:
  bool _internal_has_straightdrvtube_nu() const;
  public:
  void clear_straightdrvtube_nu();
  bool straightdrvtube_nu() const;
  void set_straightdrvtube_nu(bool value);
  private:
  bool _internal_straightdrvtube_nu() const;
  void _internal_set_straightdrvtube_nu(bool value);
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

  // optional .pb.pdcp.drv_tube_display.DrvTubeDisplay drvTubeDisplay_nu = 3130;
  bool has_drvtubedisplay_nu() const;
  private:
  bool _internal_has_drvtubedisplay_nu() const;
  public:
  void clear_drvtubedisplay_nu();
  ::pb::pdcp::drv_tube_display::DrvTubeDisplay drvtubedisplay_nu() const;
  void set_drvtubedisplay_nu(::pb::pdcp::drv_tube_display::DrvTubeDisplay value);
  private:
  ::pb::pdcp::drv_tube_display::DrvTubeDisplay _internal_drvtubedisplay_nu() const;
  void _internal_set_drvtubedisplay_nu(::pb::pdcp::drv_tube_display::DrvTubeDisplay value);
  public:

  // @@protoc_insertion_point(class_scope:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* turningcirclecenter_nu_;
  float rearradius_m_;
  float frontradius_m_;
  bool straightdrvtube_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  int drvtubedisplay_nu_;
  friend struct ::TableStruct_pdcp_2fpdcpdriving_5ftube_5fport_2eproto;
};
// -------------------------------------------------------------------

class PDCPDrivingTubePort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort_array_port) */ {
 public:
  PDCPDrivingTubePort_array_port();
  virtual ~PDCPDrivingTubePort_array_port();

  PDCPDrivingTubePort_array_port(const PDCPDrivingTubePort_array_port& from);
  PDCPDrivingTubePort_array_port(PDCPDrivingTubePort_array_port&& from) noexcept
    : PDCPDrivingTubePort_array_port() {
    *this = ::std::move(from);
  }

  inline PDCPDrivingTubePort_array_port& operator=(const PDCPDrivingTubePort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline PDCPDrivingTubePort_array_port& operator=(PDCPDrivingTubePort_array_port&& from) noexcept {
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
  static const PDCPDrivingTubePort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PDCPDrivingTubePort_array_port* internal_default_instance() {
    return reinterpret_cast<const PDCPDrivingTubePort_array_port*>(
               &_PDCPDrivingTubePort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(PDCPDrivingTubePort_array_port& a, PDCPDrivingTubePort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(PDCPDrivingTubePort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PDCPDrivingTubePort_array_port* New() const final {
    return CreateMaybeMessage<PDCPDrivingTubePort_array_port>(nullptr);
  }

  PDCPDrivingTubePort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PDCPDrivingTubePort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PDCPDrivingTubePort_array_port& from);
  void MergeFrom(const PDCPDrivingTubePort_array_port& from);
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
  void InternalSwap(PDCPDrivingTubePort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_pdcp_2fpdcpdriving_5ftube_5fport_2eproto);
    return ::descriptor_table_pdcp_2fpdcpdriving_5ftube_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1484,
  };
  // repeated .pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort data = 1484;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort >*
      mutable_data();
  private:
  const ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort& _internal_data(int index) const;
  ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort* _internal_add_data();
  public:
  const ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort& data(int index) const;
  ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort > data_;
  friend struct ::TableStruct_pdcp_2fpdcpdriving_5ftube_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PDCPDrivingTubePort

// optional uint32 uiVersionNumber = 2124;
inline bool PDCPDrivingTubePort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool PDCPDrivingTubePort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void PDCPDrivingTubePort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PDCPDrivingTubePort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PDCPDrivingTubePort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void PDCPDrivingTubePort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  uiversionnumber_ = value;
}
inline void PDCPDrivingTubePort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool PDCPDrivingTubePort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool PDCPDrivingTubePort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& PDCPDrivingTubePort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& PDCPDrivingTubePort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* PDCPDrivingTubePort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* PDCPDrivingTubePort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* PDCPDrivingTubePort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void PDCPDrivingTubePort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.sSigHeader)
}

// optional float frontRadius_m = 380;
inline bool PDCPDrivingTubePort::_internal_has_frontradius_m() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool PDCPDrivingTubePort::has_frontradius_m() const {
  return _internal_has_frontradius_m();
}
inline void PDCPDrivingTubePort::clear_frontradius_m() {
  frontradius_m_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float PDCPDrivingTubePort::_internal_frontradius_m() const {
  return frontradius_m_;
}
inline float PDCPDrivingTubePort::frontradius_m() const {
  // @@protoc_insertion_point(field_get:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.frontRadius_m)
  return _internal_frontradius_m();
}
inline void PDCPDrivingTubePort::_internal_set_frontradius_m(float value) {
  _has_bits_[0] |= 0x00000008u;
  frontradius_m_ = value;
}
inline void PDCPDrivingTubePort::set_frontradius_m(float value) {
  _internal_set_frontradius_m(value);
  // @@protoc_insertion_point(field_set:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.frontRadius_m)
}

// optional float rearRadius_m = 3176;
inline bool PDCPDrivingTubePort::_internal_has_rearradius_m() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool PDCPDrivingTubePort::has_rearradius_m() const {
  return _internal_has_rearradius_m();
}
inline void PDCPDrivingTubePort::clear_rearradius_m() {
  rearradius_m_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float PDCPDrivingTubePort::_internal_rearradius_m() const {
  return rearradius_m_;
}
inline float PDCPDrivingTubePort::rearradius_m() const {
  // @@protoc_insertion_point(field_get:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.rearRadius_m)
  return _internal_rearradius_m();
}
inline void PDCPDrivingTubePort::_internal_set_rearradius_m(float value) {
  _has_bits_[0] |= 0x00000004u;
  rearradius_m_ = value;
}
inline void PDCPDrivingTubePort::set_rearradius_m(float value) {
  _internal_set_rearradius_m(value);
  // @@protoc_insertion_point(field_set:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.rearRadius_m)
}

// optional .pb.cml.vec2_df_pod.Vec2Df_POD turningCircleCenter_nu = 2688;
inline bool PDCPDrivingTubePort::_internal_has_turningcirclecenter_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || turningcirclecenter_nu_ != nullptr);
  return value;
}
inline bool PDCPDrivingTubePort::has_turningcirclecenter_nu() const {
  return _internal_has_turningcirclecenter_nu();
}
inline const ::pb::cml::vec2_df_pod::Vec2Df_POD& PDCPDrivingTubePort::_internal_turningcirclecenter_nu() const {
  const ::pb::cml::vec2_df_pod::Vec2Df_POD* p = turningcirclecenter_nu_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::cml::vec2_df_pod::Vec2Df_POD*>(
      &::pb::cml::vec2_df_pod::_Vec2Df_POD_default_instance_);
}
inline const ::pb::cml::vec2_df_pod::Vec2Df_POD& PDCPDrivingTubePort::turningcirclecenter_nu() const {
  // @@protoc_insertion_point(field_get:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.turningCircleCenter_nu)
  return _internal_turningcirclecenter_nu();
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* PDCPDrivingTubePort::release_turningcirclecenter_nu() {
  // @@protoc_insertion_point(field_release:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.turningCircleCenter_nu)
  _has_bits_[0] &= ~0x00000002u;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* temp = turningcirclecenter_nu_;
  turningcirclecenter_nu_ = nullptr;
  return temp;
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* PDCPDrivingTubePort::_internal_mutable_turningcirclecenter_nu() {
  _has_bits_[0] |= 0x00000002u;
  if (turningcirclecenter_nu_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::cml::vec2_df_pod::Vec2Df_POD>(GetArenaNoVirtual());
    turningcirclecenter_nu_ = p;
  }
  return turningcirclecenter_nu_;
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* PDCPDrivingTubePort::mutable_turningcirclecenter_nu() {
  // @@protoc_insertion_point(field_mutable:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.turningCircleCenter_nu)
  return _internal_mutable_turningcirclecenter_nu();
}
inline void PDCPDrivingTubePort::set_allocated_turningcirclecenter_nu(::pb::cml::vec2_df_pod::Vec2Df_POD* turningcirclecenter_nu) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(turningcirclecenter_nu_);
  }
  if (turningcirclecenter_nu) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      turningcirclecenter_nu = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, turningcirclecenter_nu, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  turningcirclecenter_nu_ = turningcirclecenter_nu;
  // @@protoc_insertion_point(field_set_allocated:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.turningCircleCenter_nu)
}

// optional .pb.pdcp.drv_tube_display.DrvTubeDisplay drvTubeDisplay_nu = 3130;
inline bool PDCPDrivingTubePort::_internal_has_drvtubedisplay_nu() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool PDCPDrivingTubePort::has_drvtubedisplay_nu() const {
  return _internal_has_drvtubedisplay_nu();
}
inline void PDCPDrivingTubePort::clear_drvtubedisplay_nu() {
  drvtubedisplay_nu_ = 0;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::pb::pdcp::drv_tube_display::DrvTubeDisplay PDCPDrivingTubePort::_internal_drvtubedisplay_nu() const {
  return static_cast< ::pb::pdcp::drv_tube_display::DrvTubeDisplay >(drvtubedisplay_nu_);
}
inline ::pb::pdcp::drv_tube_display::DrvTubeDisplay PDCPDrivingTubePort::drvtubedisplay_nu() const {
  // @@protoc_insertion_point(field_get:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.drvTubeDisplay_nu)
  return _internal_drvtubedisplay_nu();
}
inline void PDCPDrivingTubePort::_internal_set_drvtubedisplay_nu(::pb::pdcp::drv_tube_display::DrvTubeDisplay value) {
  assert(::pb::pdcp::drv_tube_display::DrvTubeDisplay_IsValid(value));
  _has_bits_[0] |= 0x00000040u;
  drvtubedisplay_nu_ = value;
}
inline void PDCPDrivingTubePort::set_drvtubedisplay_nu(::pb::pdcp::drv_tube_display::DrvTubeDisplay value) {
  _internal_set_drvtubedisplay_nu(value);
  // @@protoc_insertion_point(field_set:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.drvTubeDisplay_nu)
}

// optional bool straightDrvTube_nu = 1575;
inline bool PDCPDrivingTubePort::_internal_has_straightdrvtube_nu() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool PDCPDrivingTubePort::has_straightdrvtube_nu() const {
  return _internal_has_straightdrvtube_nu();
}
inline void PDCPDrivingTubePort::clear_straightdrvtube_nu() {
  straightdrvtube_nu_ = false;
  _has_bits_[0] &= ~0x00000010u;
}
inline bool PDCPDrivingTubePort::_internal_straightdrvtube_nu() const {
  return straightdrvtube_nu_;
}
inline bool PDCPDrivingTubePort::straightdrvtube_nu() const {
  // @@protoc_insertion_point(field_get:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.straightDrvTube_nu)
  return _internal_straightdrvtube_nu();
}
inline void PDCPDrivingTubePort::_internal_set_straightdrvtube_nu(bool value) {
  _has_bits_[0] |= 0x00000010u;
  straightdrvtube_nu_ = value;
}
inline void PDCPDrivingTubePort::set_straightdrvtube_nu(bool value) {
  _internal_set_straightdrvtube_nu(value);
  // @@protoc_insertion_point(field_set:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort.straightDrvTube_nu)
}

// -------------------------------------------------------------------

// PDCPDrivingTubePort_array_port

// repeated .pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort data = 1484;
inline int PDCPDrivingTubePort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int PDCPDrivingTubePort_array_port::data_size() const {
  return _internal_data_size();
}
inline void PDCPDrivingTubePort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort* PDCPDrivingTubePort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort >*
PDCPDrivingTubePort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort_array_port.data)
  return &data_;
}
inline const ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort& PDCPDrivingTubePort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort& PDCPDrivingTubePort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort_array_port.data)
  return _internal_data(index);
}
inline ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort* PDCPDrivingTubePort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort* PDCPDrivingTubePort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::pdcp::pdcpdriving_tube_port::PDCPDrivingTubePort >&
PDCPDrivingTubePort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.pdcp.pdcpdriving_tube_port.PDCPDrivingTubePort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace pdcpdriving_tube_port
}  // namespace pdcp
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_pdcp_2fpdcpdriving_5ftube_5fport_2eproto