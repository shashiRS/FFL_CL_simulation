// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_tp/driven_path_data_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto

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
#include "ap_tp/stored_waypoint_data.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto;
namespace pb {
namespace ap_tp {
namespace driven_path_data_port {
class DrivenPathDataPort;
class DrivenPathDataPortDefaultTypeInternal;
extern DrivenPathDataPortDefaultTypeInternal _DrivenPathDataPort_default_instance_;
class DrivenPathDataPort_array_port;
class DrivenPathDataPort_array_portDefaultTypeInternal;
extern DrivenPathDataPort_array_portDefaultTypeInternal _DrivenPathDataPort_array_port_default_instance_;
}  // namespace driven_path_data_port
}  // namespace ap_tp
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort* Arena::CreateMaybeMessage<::pb::ap_tp::driven_path_data_port::DrivenPathDataPort>(Arena*);
template<> ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort_array_port* Arena::CreateMaybeMessage<::pb::ap_tp::driven_path_data_port::DrivenPathDataPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_tp {
namespace driven_path_data_port {

// ===================================================================

class DrivenPathDataPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_tp.driven_path_data_port.DrivenPathDataPort) */ {
 public:
  DrivenPathDataPort();
  virtual ~DrivenPathDataPort();

  DrivenPathDataPort(const DrivenPathDataPort& from);
  DrivenPathDataPort(DrivenPathDataPort&& from) noexcept
    : DrivenPathDataPort() {
    *this = ::std::move(from);
  }

  inline DrivenPathDataPort& operator=(const DrivenPathDataPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline DrivenPathDataPort& operator=(DrivenPathDataPort&& from) noexcept {
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
  static const DrivenPathDataPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DrivenPathDataPort* internal_default_instance() {
    return reinterpret_cast<const DrivenPathDataPort*>(
               &_DrivenPathDataPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(DrivenPathDataPort& a, DrivenPathDataPort& b) {
    a.Swap(&b);
  }
  inline void Swap(DrivenPathDataPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline DrivenPathDataPort* New() const final {
    return CreateMaybeMessage<DrivenPathDataPort>(nullptr);
  }

  DrivenPathDataPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<DrivenPathDataPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const DrivenPathDataPort& from);
  void MergeFrom(const DrivenPathDataPort& from);
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
  void InternalSwap(DrivenPathDataPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_tp.driven_path_data_port.DrivenPathDataPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto);
    return ::descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kStoredDrivenPathFieldNumber = 647,
    kBufferFieldNumber = 2818,
    kSSigHeaderFieldNumber = 1033,
    kHasValidDataFieldNumber = 2506,
    kNumElementsInBufferFieldNumber = 1546,
    kNumElementsInDrivenPathFieldNumber = 1772,
    kSaveCounterFieldNumber = 2002,
    kUiVersionNumberFieldNumber = 2124,
  };
  // repeated .pb.ap_tp.stored_waypoint_data.StoredWaypointData storedDrivenPath = 647;
  int storeddrivenpath_size() const;
  private:
  int _internal_storeddrivenpath_size() const;
  public:
  void clear_storeddrivenpath();
  ::pb::ap_tp::stored_waypoint_data::StoredWaypointData* mutable_storeddrivenpath(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::stored_waypoint_data::StoredWaypointData >*
      mutable_storeddrivenpath();
  private:
  const ::pb::ap_tp::stored_waypoint_data::StoredWaypointData& _internal_storeddrivenpath(int index) const;
  ::pb::ap_tp::stored_waypoint_data::StoredWaypointData* _internal_add_storeddrivenpath();
  public:
  const ::pb::ap_tp::stored_waypoint_data::StoredWaypointData& storeddrivenpath(int index) const;
  ::pb::ap_tp::stored_waypoint_data::StoredWaypointData* add_storeddrivenpath();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::stored_waypoint_data::StoredWaypointData >&
      storeddrivenpath() const;

  // repeated .pb.ap_tp.stored_waypoint_data.StoredWaypointData buffer = 2818;
  int buffer_size() const;
  private:
  int _internal_buffer_size() const;
  public:
  void clear_buffer();
  ::pb::ap_tp::stored_waypoint_data::StoredWaypointData* mutable_buffer(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::stored_waypoint_data::StoredWaypointData >*
      mutable_buffer();
  private:
  const ::pb::ap_tp::stored_waypoint_data::StoredWaypointData& _internal_buffer(int index) const;
  ::pb::ap_tp::stored_waypoint_data::StoredWaypointData* _internal_add_buffer();
  public:
  const ::pb::ap_tp::stored_waypoint_data::StoredWaypointData& buffer(int index) const;
  ::pb::ap_tp::stored_waypoint_data::StoredWaypointData* add_buffer();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::stored_waypoint_data::StoredWaypointData >&
      buffer() const;

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

  // optional bool hasValidData = 2506;
  bool has_hasvaliddata() const;
  private:
  bool _internal_has_hasvaliddata() const;
  public:
  void clear_hasvaliddata();
  bool hasvaliddata() const;
  void set_hasvaliddata(bool value);
  private:
  bool _internal_hasvaliddata() const;
  void _internal_set_hasvaliddata(bool value);
  public:

  // optional uint32 numElementsInBuffer = 1546;
  bool has_numelementsinbuffer() const;
  private:
  bool _internal_has_numelementsinbuffer() const;
  public:
  void clear_numelementsinbuffer();
  ::PROTOBUF_NAMESPACE_ID::uint32 numelementsinbuffer() const;
  void set_numelementsinbuffer(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_numelementsinbuffer() const;
  void _internal_set_numelementsinbuffer(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 numElementsInDrivenPath = 1772;
  bool has_numelementsindrivenpath() const;
  private:
  bool _internal_has_numelementsindrivenpath() const;
  public:
  void clear_numelementsindrivenpath();
  ::PROTOBUF_NAMESPACE_ID::uint32 numelementsindrivenpath() const;
  void set_numelementsindrivenpath(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_numelementsindrivenpath() const;
  void _internal_set_numelementsindrivenpath(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 saveCounter = 2002;
  bool has_savecounter() const;
  private:
  bool _internal_has_savecounter() const;
  public:
  void clear_savecounter();
  ::PROTOBUF_NAMESPACE_ID::uint32 savecounter() const;
  void set_savecounter(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_savecounter() const;
  void _internal_set_savecounter(::PROTOBUF_NAMESPACE_ID::uint32 value);
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

  // @@protoc_insertion_point(class_scope:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::stored_waypoint_data::StoredWaypointData > storeddrivenpath_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::stored_waypoint_data::StoredWaypointData > buffer_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  bool hasvaliddata_;
  ::PROTOBUF_NAMESPACE_ID::uint32 numelementsinbuffer_;
  ::PROTOBUF_NAMESPACE_ID::uint32 numelementsindrivenpath_;
  ::PROTOBUF_NAMESPACE_ID::uint32 savecounter_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  friend struct ::TableStruct_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto;
};
// -------------------------------------------------------------------

class DrivenPathDataPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port) */ {
 public:
  DrivenPathDataPort_array_port();
  virtual ~DrivenPathDataPort_array_port();

  DrivenPathDataPort_array_port(const DrivenPathDataPort_array_port& from);
  DrivenPathDataPort_array_port(DrivenPathDataPort_array_port&& from) noexcept
    : DrivenPathDataPort_array_port() {
    *this = ::std::move(from);
  }

  inline DrivenPathDataPort_array_port& operator=(const DrivenPathDataPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline DrivenPathDataPort_array_port& operator=(DrivenPathDataPort_array_port&& from) noexcept {
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
  static const DrivenPathDataPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DrivenPathDataPort_array_port* internal_default_instance() {
    return reinterpret_cast<const DrivenPathDataPort_array_port*>(
               &_DrivenPathDataPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(DrivenPathDataPort_array_port& a, DrivenPathDataPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(DrivenPathDataPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline DrivenPathDataPort_array_port* New() const final {
    return CreateMaybeMessage<DrivenPathDataPort_array_port>(nullptr);
  }

  DrivenPathDataPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<DrivenPathDataPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const DrivenPathDataPort_array_port& from);
  void MergeFrom(const DrivenPathDataPort_array_port& from);
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
  void InternalSwap(DrivenPathDataPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto);
    return ::descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3266,
  };
  // repeated .pb.ap_tp.driven_path_data_port.DrivenPathDataPort data = 3266;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort >*
      mutable_data();
  private:
  const ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort& _internal_data(int index) const;
  ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort* _internal_add_data();
  public:
  const ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort& data(int index) const;
  ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort > data_;
  friend struct ::TableStruct_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// DrivenPathDataPort

// optional uint32 uiVersionNumber = 2124;
inline bool DrivenPathDataPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool DrivenPathDataPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void DrivenPathDataPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DrivenPathDataPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DrivenPathDataPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void DrivenPathDataPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  uiversionnumber_ = value;
}
inline void DrivenPathDataPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool DrivenPathDataPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool DrivenPathDataPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& DrivenPathDataPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& DrivenPathDataPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* DrivenPathDataPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* DrivenPathDataPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* DrivenPathDataPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void DrivenPathDataPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.sSigHeader)
}

// optional uint32 saveCounter = 2002;
inline bool DrivenPathDataPort::_internal_has_savecounter() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool DrivenPathDataPort::has_savecounter() const {
  return _internal_has_savecounter();
}
inline void DrivenPathDataPort::clear_savecounter() {
  savecounter_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DrivenPathDataPort::_internal_savecounter() const {
  return savecounter_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DrivenPathDataPort::savecounter() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.saveCounter)
  return _internal_savecounter();
}
inline void DrivenPathDataPort::_internal_set_savecounter(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  savecounter_ = value;
}
inline void DrivenPathDataPort::set_savecounter(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_savecounter(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.saveCounter)
}

// optional bool hasValidData = 2506;
inline bool DrivenPathDataPort::_internal_has_hasvaliddata() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool DrivenPathDataPort::has_hasvaliddata() const {
  return _internal_has_hasvaliddata();
}
inline void DrivenPathDataPort::clear_hasvaliddata() {
  hasvaliddata_ = false;
  _has_bits_[0] &= ~0x00000002u;
}
inline bool DrivenPathDataPort::_internal_hasvaliddata() const {
  return hasvaliddata_;
}
inline bool DrivenPathDataPort::hasvaliddata() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.hasValidData)
  return _internal_hasvaliddata();
}
inline void DrivenPathDataPort::_internal_set_hasvaliddata(bool value) {
  _has_bits_[0] |= 0x00000002u;
  hasvaliddata_ = value;
}
inline void DrivenPathDataPort::set_hasvaliddata(bool value) {
  _internal_set_hasvaliddata(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.hasValidData)
}

// repeated .pb.ap_tp.stored_waypoint_data.StoredWaypointData storedDrivenPath = 647;
inline int DrivenPathDataPort::_internal_storeddrivenpath_size() const {
  return storeddrivenpath_.size();
}
inline int DrivenPathDataPort::storeddrivenpath_size() const {
  return _internal_storeddrivenpath_size();
}
inline ::pb::ap_tp::stored_waypoint_data::StoredWaypointData* DrivenPathDataPort::mutable_storeddrivenpath(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.storedDrivenPath)
  return storeddrivenpath_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::stored_waypoint_data::StoredWaypointData >*
DrivenPathDataPort::mutable_storeddrivenpath() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.storedDrivenPath)
  return &storeddrivenpath_;
}
inline const ::pb::ap_tp::stored_waypoint_data::StoredWaypointData& DrivenPathDataPort::_internal_storeddrivenpath(int index) const {
  return storeddrivenpath_.Get(index);
}
inline const ::pb::ap_tp::stored_waypoint_data::StoredWaypointData& DrivenPathDataPort::storeddrivenpath(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.storedDrivenPath)
  return _internal_storeddrivenpath(index);
}
inline ::pb::ap_tp::stored_waypoint_data::StoredWaypointData* DrivenPathDataPort::_internal_add_storeddrivenpath() {
  return storeddrivenpath_.Add();
}
inline ::pb::ap_tp::stored_waypoint_data::StoredWaypointData* DrivenPathDataPort::add_storeddrivenpath() {
  // @@protoc_insertion_point(field_add:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.storedDrivenPath)
  return _internal_add_storeddrivenpath();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::stored_waypoint_data::StoredWaypointData >&
DrivenPathDataPort::storeddrivenpath() const {
  // @@protoc_insertion_point(field_list:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.storedDrivenPath)
  return storeddrivenpath_;
}

// optional uint32 numElementsInDrivenPath = 1772;
inline bool DrivenPathDataPort::_internal_has_numelementsindrivenpath() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool DrivenPathDataPort::has_numelementsindrivenpath() const {
  return _internal_has_numelementsindrivenpath();
}
inline void DrivenPathDataPort::clear_numelementsindrivenpath() {
  numelementsindrivenpath_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DrivenPathDataPort::_internal_numelementsindrivenpath() const {
  return numelementsindrivenpath_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DrivenPathDataPort::numelementsindrivenpath() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.numElementsInDrivenPath)
  return _internal_numelementsindrivenpath();
}
inline void DrivenPathDataPort::_internal_set_numelementsindrivenpath(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  numelementsindrivenpath_ = value;
}
inline void DrivenPathDataPort::set_numelementsindrivenpath(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_numelementsindrivenpath(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.numElementsInDrivenPath)
}

// repeated .pb.ap_tp.stored_waypoint_data.StoredWaypointData buffer = 2818;
inline int DrivenPathDataPort::_internal_buffer_size() const {
  return buffer_.size();
}
inline int DrivenPathDataPort::buffer_size() const {
  return _internal_buffer_size();
}
inline ::pb::ap_tp::stored_waypoint_data::StoredWaypointData* DrivenPathDataPort::mutable_buffer(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.buffer)
  return buffer_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::stored_waypoint_data::StoredWaypointData >*
DrivenPathDataPort::mutable_buffer() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.buffer)
  return &buffer_;
}
inline const ::pb::ap_tp::stored_waypoint_data::StoredWaypointData& DrivenPathDataPort::_internal_buffer(int index) const {
  return buffer_.Get(index);
}
inline const ::pb::ap_tp::stored_waypoint_data::StoredWaypointData& DrivenPathDataPort::buffer(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.buffer)
  return _internal_buffer(index);
}
inline ::pb::ap_tp::stored_waypoint_data::StoredWaypointData* DrivenPathDataPort::_internal_add_buffer() {
  return buffer_.Add();
}
inline ::pb::ap_tp::stored_waypoint_data::StoredWaypointData* DrivenPathDataPort::add_buffer() {
  // @@protoc_insertion_point(field_add:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.buffer)
  return _internal_add_buffer();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::stored_waypoint_data::StoredWaypointData >&
DrivenPathDataPort::buffer() const {
  // @@protoc_insertion_point(field_list:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.buffer)
  return buffer_;
}

// optional uint32 numElementsInBuffer = 1546;
inline bool DrivenPathDataPort::_internal_has_numelementsinbuffer() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool DrivenPathDataPort::has_numelementsinbuffer() const {
  return _internal_has_numelementsinbuffer();
}
inline void DrivenPathDataPort::clear_numelementsinbuffer() {
  numelementsinbuffer_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DrivenPathDataPort::_internal_numelementsinbuffer() const {
  return numelementsinbuffer_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DrivenPathDataPort::numelementsinbuffer() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.numElementsInBuffer)
  return _internal_numelementsinbuffer();
}
inline void DrivenPathDataPort::_internal_set_numelementsinbuffer(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  numelementsinbuffer_ = value;
}
inline void DrivenPathDataPort::set_numelementsinbuffer(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_numelementsinbuffer(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.driven_path_data_port.DrivenPathDataPort.numElementsInBuffer)
}

// -------------------------------------------------------------------

// DrivenPathDataPort_array_port

// repeated .pb.ap_tp.driven_path_data_port.DrivenPathDataPort data = 3266;
inline int DrivenPathDataPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int DrivenPathDataPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void DrivenPathDataPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort* DrivenPathDataPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort >*
DrivenPathDataPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port.data)
  return &data_;
}
inline const ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort& DrivenPathDataPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort& DrivenPathDataPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort* DrivenPathDataPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort* DrivenPathDataPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort >&
DrivenPathDataPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace driven_path_data_port
}  // namespace ap_tp
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto