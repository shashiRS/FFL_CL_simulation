// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_em/us_em_debug_output_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fem_2fus_5fem_5fdebug_5foutput_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fem_2fus_5fem_5fdebug_5foutput_5fport_2eproto

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
#include "us_em/us_em_point.pb.h"
#include "us_em/us_em_position.pb.h"
#include "us_em/us_em_dec_data.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_us_5fem_2fus_5fem_5fdebug_5foutput_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fem_2fus_5fem_5fdebug_5foutput_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fem_2fus_5fem_5fdebug_5foutput_5fport_2eproto;
namespace pb {
namespace us_em {
namespace us_em_debug_output_port {
class UsEmDebugOutputPort;
class UsEmDebugOutputPortDefaultTypeInternal;
extern UsEmDebugOutputPortDefaultTypeInternal _UsEmDebugOutputPort_default_instance_;
class UsEmDebugOutputPort_array_port;
class UsEmDebugOutputPort_array_portDefaultTypeInternal;
extern UsEmDebugOutputPort_array_portDefaultTypeInternal _UsEmDebugOutputPort_array_port_default_instance_;
}  // namespace us_em_debug_output_port
}  // namespace us_em
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort* Arena::CreateMaybeMessage<::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort>(Arena*);
template<> ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort_array_port* Arena::CreateMaybeMessage<::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_em {
namespace us_em_debug_output_port {

// ===================================================================

class UsEmDebugOutputPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort) */ {
 public:
  UsEmDebugOutputPort();
  virtual ~UsEmDebugOutputPort();

  UsEmDebugOutputPort(const UsEmDebugOutputPort& from);
  UsEmDebugOutputPort(UsEmDebugOutputPort&& from) noexcept
    : UsEmDebugOutputPort() {
    *this = ::std::move(from);
  }

  inline UsEmDebugOutputPort& operator=(const UsEmDebugOutputPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsEmDebugOutputPort& operator=(UsEmDebugOutputPort&& from) noexcept {
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
  static const UsEmDebugOutputPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsEmDebugOutputPort* internal_default_instance() {
    return reinterpret_cast<const UsEmDebugOutputPort*>(
               &_UsEmDebugOutputPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsEmDebugOutputPort& a, UsEmDebugOutputPort& b) {
    a.Swap(&b);
  }
  inline void Swap(UsEmDebugOutputPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsEmDebugOutputPort* New() const final {
    return CreateMaybeMessage<UsEmDebugOutputPort>(nullptr);
  }

  UsEmDebugOutputPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsEmDebugOutputPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsEmDebugOutputPort& from);
  void MergeFrom(const UsEmDebugOutputPort& from);
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
  void InternalSwap(UsEmDebugOutputPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fem_2fus_5fem_5fdebug_5foutput_5fport_2eproto);
    return ::descriptor_table_us_5fem_2fus_5fem_5fdebug_5foutput_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDecayDataFieldNumber = 274,
    kVehicleBoundingBoxesFieldNumber = 2777,
    kUsEmPointListFieldNumber = 3132,
    kSSigHeaderFieldNumber = 1033,
    kNumberOfPointsFieldNumber = 3290,
    kCycleCounterFieldNumber = 1989,
    kUiVersionNumberFieldNumber = 2124,
  };
  // repeated .pb.us_em.us_em_dec_data.UsEmDecData decayData = 274;
  int decaydata_size() const;
  private:
  int _internal_decaydata_size() const;
  public:
  void clear_decaydata();
  ::pb::us_em::us_em_dec_data::UsEmDecData* mutable_decaydata(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_dec_data::UsEmDecData >*
      mutable_decaydata();
  private:
  const ::pb::us_em::us_em_dec_data::UsEmDecData& _internal_decaydata(int index) const;
  ::pb::us_em::us_em_dec_data::UsEmDecData* _internal_add_decaydata();
  public:
  const ::pb::us_em::us_em_dec_data::UsEmDecData& decaydata(int index) const;
  ::pb::us_em::us_em_dec_data::UsEmDecData* add_decaydata();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_dec_data::UsEmDecData >&
      decaydata() const;

  // repeated .pb.us_em.us_em_position.UsEmPosition vehicleBoundingBoxes = 2777;
  int vehicleboundingboxes_size() const;
  private:
  int _internal_vehicleboundingboxes_size() const;
  public:
  void clear_vehicleboundingboxes();
  ::pb::us_em::us_em_position::UsEmPosition* mutable_vehicleboundingboxes(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_position::UsEmPosition >*
      mutable_vehicleboundingboxes();
  private:
  const ::pb::us_em::us_em_position::UsEmPosition& _internal_vehicleboundingboxes(int index) const;
  ::pb::us_em::us_em_position::UsEmPosition* _internal_add_vehicleboundingboxes();
  public:
  const ::pb::us_em::us_em_position::UsEmPosition& vehicleboundingboxes(int index) const;
  ::pb::us_em::us_em_position::UsEmPosition* add_vehicleboundingboxes();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_position::UsEmPosition >&
      vehicleboundingboxes() const;

  // repeated .pb.us_em.us_em_point.UsEmPoint usEmPointList = 3132;
  int usempointlist_size() const;
  private:
  int _internal_usempointlist_size() const;
  public:
  void clear_usempointlist();
  ::pb::us_em::us_em_point::UsEmPoint* mutable_usempointlist(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_point::UsEmPoint >*
      mutable_usempointlist();
  private:
  const ::pb::us_em::us_em_point::UsEmPoint& _internal_usempointlist(int index) const;
  ::pb::us_em::us_em_point::UsEmPoint* _internal_add_usempointlist();
  public:
  const ::pb::us_em::us_em_point::UsEmPoint& usempointlist(int index) const;
  ::pb::us_em::us_em_point::UsEmPoint* add_usempointlist();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_point::UsEmPoint >&
      usempointlist() const;

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

  // optional uint32 numberOfPoints = 3290;
  bool has_numberofpoints() const;
  private:
  bool _internal_has_numberofpoints() const;
  public:
  void clear_numberofpoints();
  ::PROTOBUF_NAMESPACE_ID::uint32 numberofpoints() const;
  void set_numberofpoints(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_numberofpoints() const;
  void _internal_set_numberofpoints(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 cycleCounter = 1989;
  bool has_cyclecounter() const;
  private:
  bool _internal_has_cyclecounter() const;
  public:
  void clear_cyclecounter();
  ::PROTOBUF_NAMESPACE_ID::uint32 cyclecounter() const;
  void set_cyclecounter(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_cyclecounter() const;
  void _internal_set_cyclecounter(::PROTOBUF_NAMESPACE_ID::uint32 value);
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

  // @@protoc_insertion_point(class_scope:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_dec_data::UsEmDecData > decaydata_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_position::UsEmPosition > vehicleboundingboxes_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_point::UsEmPoint > usempointlist_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint32 numberofpoints_;
  ::PROTOBUF_NAMESPACE_ID::uint32 cyclecounter_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  friend struct ::TableStruct_us_5fem_2fus_5fem_5fdebug_5foutput_5fport_2eproto;
};
// -------------------------------------------------------------------

class UsEmDebugOutputPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort_array_port) */ {
 public:
  UsEmDebugOutputPort_array_port();
  virtual ~UsEmDebugOutputPort_array_port();

  UsEmDebugOutputPort_array_port(const UsEmDebugOutputPort_array_port& from);
  UsEmDebugOutputPort_array_port(UsEmDebugOutputPort_array_port&& from) noexcept
    : UsEmDebugOutputPort_array_port() {
    *this = ::std::move(from);
  }

  inline UsEmDebugOutputPort_array_port& operator=(const UsEmDebugOutputPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsEmDebugOutputPort_array_port& operator=(UsEmDebugOutputPort_array_port&& from) noexcept {
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
  static const UsEmDebugOutputPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsEmDebugOutputPort_array_port* internal_default_instance() {
    return reinterpret_cast<const UsEmDebugOutputPort_array_port*>(
               &_UsEmDebugOutputPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsEmDebugOutputPort_array_port& a, UsEmDebugOutputPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsEmDebugOutputPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsEmDebugOutputPort_array_port* New() const final {
    return CreateMaybeMessage<UsEmDebugOutputPort_array_port>(nullptr);
  }

  UsEmDebugOutputPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsEmDebugOutputPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsEmDebugOutputPort_array_port& from);
  void MergeFrom(const UsEmDebugOutputPort_array_port& from);
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
  void InternalSwap(UsEmDebugOutputPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fem_2fus_5fem_5fdebug_5foutput_5fport_2eproto);
    return ::descriptor_table_us_5fem_2fus_5fem_5fdebug_5foutput_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 4005,
  };
  // repeated .pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort data = 4005;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort >*
      mutable_data();
  private:
  const ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort& _internal_data(int index) const;
  ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort* _internal_add_data();
  public:
  const ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort& data(int index) const;
  ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort > data_;
  friend struct ::TableStruct_us_5fem_2fus_5fem_5fdebug_5foutput_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsEmDebugOutputPort

// optional uint32 uiVersionNumber = 2124;
inline bool UsEmDebugOutputPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool UsEmDebugOutputPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void UsEmDebugOutputPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEmDebugOutputPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEmDebugOutputPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void UsEmDebugOutputPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  uiversionnumber_ = value;
}
inline void UsEmDebugOutputPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool UsEmDebugOutputPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool UsEmDebugOutputPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& UsEmDebugOutputPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& UsEmDebugOutputPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* UsEmDebugOutputPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* UsEmDebugOutputPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* UsEmDebugOutputPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void UsEmDebugOutputPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.sSigHeader)
}

// optional uint32 numberOfPoints = 3290;
inline bool UsEmDebugOutputPort::_internal_has_numberofpoints() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool UsEmDebugOutputPort::has_numberofpoints() const {
  return _internal_has_numberofpoints();
}
inline void UsEmDebugOutputPort::clear_numberofpoints() {
  numberofpoints_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEmDebugOutputPort::_internal_numberofpoints() const {
  return numberofpoints_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEmDebugOutputPort::numberofpoints() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.numberOfPoints)
  return _internal_numberofpoints();
}
inline void UsEmDebugOutputPort::_internal_set_numberofpoints(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  numberofpoints_ = value;
}
inline void UsEmDebugOutputPort::set_numberofpoints(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_numberofpoints(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.numberOfPoints)
}

// repeated .pb.us_em.us_em_point.UsEmPoint usEmPointList = 3132;
inline int UsEmDebugOutputPort::_internal_usempointlist_size() const {
  return usempointlist_.size();
}
inline int UsEmDebugOutputPort::usempointlist_size() const {
  return _internal_usempointlist_size();
}
inline ::pb::us_em::us_em_point::UsEmPoint* UsEmDebugOutputPort::mutable_usempointlist(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.usEmPointList)
  return usempointlist_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_point::UsEmPoint >*
UsEmDebugOutputPort::mutable_usempointlist() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.usEmPointList)
  return &usempointlist_;
}
inline const ::pb::us_em::us_em_point::UsEmPoint& UsEmDebugOutputPort::_internal_usempointlist(int index) const {
  return usempointlist_.Get(index);
}
inline const ::pb::us_em::us_em_point::UsEmPoint& UsEmDebugOutputPort::usempointlist(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.usEmPointList)
  return _internal_usempointlist(index);
}
inline ::pb::us_em::us_em_point::UsEmPoint* UsEmDebugOutputPort::_internal_add_usempointlist() {
  return usempointlist_.Add();
}
inline ::pb::us_em::us_em_point::UsEmPoint* UsEmDebugOutputPort::add_usempointlist() {
  // @@protoc_insertion_point(field_add:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.usEmPointList)
  return _internal_add_usempointlist();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_point::UsEmPoint >&
UsEmDebugOutputPort::usempointlist() const {
  // @@protoc_insertion_point(field_list:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.usEmPointList)
  return usempointlist_;
}

// repeated .pb.us_em.us_em_position.UsEmPosition vehicleBoundingBoxes = 2777;
inline int UsEmDebugOutputPort::_internal_vehicleboundingboxes_size() const {
  return vehicleboundingboxes_.size();
}
inline int UsEmDebugOutputPort::vehicleboundingboxes_size() const {
  return _internal_vehicleboundingboxes_size();
}
inline ::pb::us_em::us_em_position::UsEmPosition* UsEmDebugOutputPort::mutable_vehicleboundingboxes(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.vehicleBoundingBoxes)
  return vehicleboundingboxes_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_position::UsEmPosition >*
UsEmDebugOutputPort::mutable_vehicleboundingboxes() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.vehicleBoundingBoxes)
  return &vehicleboundingboxes_;
}
inline const ::pb::us_em::us_em_position::UsEmPosition& UsEmDebugOutputPort::_internal_vehicleboundingboxes(int index) const {
  return vehicleboundingboxes_.Get(index);
}
inline const ::pb::us_em::us_em_position::UsEmPosition& UsEmDebugOutputPort::vehicleboundingboxes(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.vehicleBoundingBoxes)
  return _internal_vehicleboundingboxes(index);
}
inline ::pb::us_em::us_em_position::UsEmPosition* UsEmDebugOutputPort::_internal_add_vehicleboundingboxes() {
  return vehicleboundingboxes_.Add();
}
inline ::pb::us_em::us_em_position::UsEmPosition* UsEmDebugOutputPort::add_vehicleboundingboxes() {
  // @@protoc_insertion_point(field_add:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.vehicleBoundingBoxes)
  return _internal_add_vehicleboundingboxes();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_position::UsEmPosition >&
UsEmDebugOutputPort::vehicleboundingboxes() const {
  // @@protoc_insertion_point(field_list:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.vehicleBoundingBoxes)
  return vehicleboundingboxes_;
}

// repeated .pb.us_em.us_em_dec_data.UsEmDecData decayData = 274;
inline int UsEmDebugOutputPort::_internal_decaydata_size() const {
  return decaydata_.size();
}
inline int UsEmDebugOutputPort::decaydata_size() const {
  return _internal_decaydata_size();
}
inline ::pb::us_em::us_em_dec_data::UsEmDecData* UsEmDebugOutputPort::mutable_decaydata(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.decayData)
  return decaydata_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_dec_data::UsEmDecData >*
UsEmDebugOutputPort::mutable_decaydata() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.decayData)
  return &decaydata_;
}
inline const ::pb::us_em::us_em_dec_data::UsEmDecData& UsEmDebugOutputPort::_internal_decaydata(int index) const {
  return decaydata_.Get(index);
}
inline const ::pb::us_em::us_em_dec_data::UsEmDecData& UsEmDebugOutputPort::decaydata(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.decayData)
  return _internal_decaydata(index);
}
inline ::pb::us_em::us_em_dec_data::UsEmDecData* UsEmDebugOutputPort::_internal_add_decaydata() {
  return decaydata_.Add();
}
inline ::pb::us_em::us_em_dec_data::UsEmDecData* UsEmDebugOutputPort::add_decaydata() {
  // @@protoc_insertion_point(field_add:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.decayData)
  return _internal_add_decaydata();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_dec_data::UsEmDecData >&
UsEmDebugOutputPort::decaydata() const {
  // @@protoc_insertion_point(field_list:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.decayData)
  return decaydata_;
}

// optional uint32 cycleCounter = 1989;
inline bool UsEmDebugOutputPort::_internal_has_cyclecounter() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool UsEmDebugOutputPort::has_cyclecounter() const {
  return _internal_has_cyclecounter();
}
inline void UsEmDebugOutputPort::clear_cyclecounter() {
  cyclecounter_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEmDebugOutputPort::_internal_cyclecounter() const {
  return cyclecounter_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEmDebugOutputPort::cyclecounter() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.cycleCounter)
  return _internal_cyclecounter();
}
inline void UsEmDebugOutputPort::_internal_set_cyclecounter(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  cyclecounter_ = value;
}
inline void UsEmDebugOutputPort::set_cyclecounter(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_cyclecounter(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort.cycleCounter)
}

// -------------------------------------------------------------------

// UsEmDebugOutputPort_array_port

// repeated .pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort data = 4005;
inline int UsEmDebugOutputPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsEmDebugOutputPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsEmDebugOutputPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort* UsEmDebugOutputPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort >*
UsEmDebugOutputPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort_array_port.data)
  return &data_;
}
inline const ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort& UsEmDebugOutputPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort& UsEmDebugOutputPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort* UsEmDebugOutputPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort* UsEmDebugOutputPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_debug_output_port::UsEmDebugOutputPort >&
UsEmDebugOutputPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_em.us_em_debug_output_port.UsEmDebugOutputPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_em_debug_output_port
}  // namespace us_em
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fem_2fus_5fem_5fdebug_5foutput_5fport_2eproto
