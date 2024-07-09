// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/diagnostic_event.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_si_2fdiagnostic_5fevent_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_si_2fdiagnostic_5fevent_2eproto

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
#include "eco/diagnosis_event_status.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_si_2fdiagnostic_5fevent_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_si_2fdiagnostic_5fevent_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fdiagnostic_5fevent_2eproto;
namespace pb {
namespace si {
namespace diagnostic_event {
class DiagnosticEvent;
class DiagnosticEventDefaultTypeInternal;
extern DiagnosticEventDefaultTypeInternal _DiagnosticEvent_default_instance_;
class DiagnosticEvent_array_port;
class DiagnosticEvent_array_portDefaultTypeInternal;
extern DiagnosticEvent_array_portDefaultTypeInternal _DiagnosticEvent_array_port_default_instance_;
}  // namespace diagnostic_event
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::si::diagnostic_event::DiagnosticEvent* Arena::CreateMaybeMessage<::pb::si::diagnostic_event::DiagnosticEvent>(Arena*);
template<> ::pb::si::diagnostic_event::DiagnosticEvent_array_port* Arena::CreateMaybeMessage<::pb::si::diagnostic_event::DiagnosticEvent_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace si {
namespace diagnostic_event {

// ===================================================================

class DiagnosticEvent :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.diagnostic_event.DiagnosticEvent) */ {
 public:
  DiagnosticEvent();
  virtual ~DiagnosticEvent();

  DiagnosticEvent(const DiagnosticEvent& from);
  DiagnosticEvent(DiagnosticEvent&& from) noexcept
    : DiagnosticEvent() {
    *this = ::std::move(from);
  }

  inline DiagnosticEvent& operator=(const DiagnosticEvent& from) {
    CopyFrom(from);
    return *this;
  }
  inline DiagnosticEvent& operator=(DiagnosticEvent&& from) noexcept {
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
  static const DiagnosticEvent& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DiagnosticEvent* internal_default_instance() {
    return reinterpret_cast<const DiagnosticEvent*>(
               &_DiagnosticEvent_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(DiagnosticEvent& a, DiagnosticEvent& b) {
    a.Swap(&b);
  }
  inline void Swap(DiagnosticEvent* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline DiagnosticEvent* New() const final {
    return CreateMaybeMessage<DiagnosticEvent>(nullptr);
  }

  DiagnosticEvent* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<DiagnosticEvent>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const DiagnosticEvent& from);
  void MergeFrom(const DiagnosticEvent& from);
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
  void InternalSwap(DiagnosticEvent* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.diagnostic_event.DiagnosticEvent";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fdiagnostic_5fevent_2eproto);
    return ::descriptor_table_si_2fdiagnostic_5fevent_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDiagnosisEventStatusFieldNumber = 870,
    kDiagnosisEventIDFieldNumber = 3237,
  };
  // optional .pb.eco.diagnosis_event_status.DiagnosisEventStatus diagnosisEventStatus = 870;
  bool has_diagnosiseventstatus() const;
  private:
  bool _internal_has_diagnosiseventstatus() const;
  public:
  void clear_diagnosiseventstatus();
  ::pb::eco::diagnosis_event_status::DiagnosisEventStatus diagnosiseventstatus() const;
  void set_diagnosiseventstatus(::pb::eco::diagnosis_event_status::DiagnosisEventStatus value);
  private:
  ::pb::eco::diagnosis_event_status::DiagnosisEventStatus _internal_diagnosiseventstatus() const;
  void _internal_set_diagnosiseventstatus(::pb::eco::diagnosis_event_status::DiagnosisEventStatus value);
  public:

  // optional uint32 diagnosisEventID = 3237;
  bool has_diagnosiseventid() const;
  private:
  bool _internal_has_diagnosiseventid() const;
  public:
  void clear_diagnosiseventid();
  ::PROTOBUF_NAMESPACE_ID::uint32 diagnosiseventid() const;
  void set_diagnosiseventid(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_diagnosiseventid() const;
  void _internal_set_diagnosiseventid(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.si.diagnostic_event.DiagnosticEvent)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int diagnosiseventstatus_;
  ::PROTOBUF_NAMESPACE_ID::uint32 diagnosiseventid_;
  friend struct ::TableStruct_si_2fdiagnostic_5fevent_2eproto;
};
// -------------------------------------------------------------------

class DiagnosticEvent_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.diagnostic_event.DiagnosticEvent_array_port) */ {
 public:
  DiagnosticEvent_array_port();
  virtual ~DiagnosticEvent_array_port();

  DiagnosticEvent_array_port(const DiagnosticEvent_array_port& from);
  DiagnosticEvent_array_port(DiagnosticEvent_array_port&& from) noexcept
    : DiagnosticEvent_array_port() {
    *this = ::std::move(from);
  }

  inline DiagnosticEvent_array_port& operator=(const DiagnosticEvent_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline DiagnosticEvent_array_port& operator=(DiagnosticEvent_array_port&& from) noexcept {
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
  static const DiagnosticEvent_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DiagnosticEvent_array_port* internal_default_instance() {
    return reinterpret_cast<const DiagnosticEvent_array_port*>(
               &_DiagnosticEvent_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(DiagnosticEvent_array_port& a, DiagnosticEvent_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(DiagnosticEvent_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline DiagnosticEvent_array_port* New() const final {
    return CreateMaybeMessage<DiagnosticEvent_array_port>(nullptr);
  }

  DiagnosticEvent_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<DiagnosticEvent_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const DiagnosticEvent_array_port& from);
  void MergeFrom(const DiagnosticEvent_array_port& from);
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
  void InternalSwap(DiagnosticEvent_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.diagnostic_event.DiagnosticEvent_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fdiagnostic_5fevent_2eproto);
    return ::descriptor_table_si_2fdiagnostic_5fevent_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1456,
  };
  // repeated .pb.si.diagnostic_event.DiagnosticEvent data = 1456;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::si::diagnostic_event::DiagnosticEvent* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::diagnostic_event::DiagnosticEvent >*
      mutable_data();
  private:
  const ::pb::si::diagnostic_event::DiagnosticEvent& _internal_data(int index) const;
  ::pb::si::diagnostic_event::DiagnosticEvent* _internal_add_data();
  public:
  const ::pb::si::diagnostic_event::DiagnosticEvent& data(int index) const;
  ::pb::si::diagnostic_event::DiagnosticEvent* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::diagnostic_event::DiagnosticEvent >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.si.diagnostic_event.DiagnosticEvent_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::diagnostic_event::DiagnosticEvent > data_;
  friend struct ::TableStruct_si_2fdiagnostic_5fevent_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// DiagnosticEvent

// optional uint32 diagnosisEventID = 3237;
inline bool DiagnosticEvent::_internal_has_diagnosiseventid() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool DiagnosticEvent::has_diagnosiseventid() const {
  return _internal_has_diagnosiseventid();
}
inline void DiagnosticEvent::clear_diagnosiseventid() {
  diagnosiseventid_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DiagnosticEvent::_internal_diagnosiseventid() const {
  return diagnosiseventid_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DiagnosticEvent::diagnosiseventid() const {
  // @@protoc_insertion_point(field_get:pb.si.diagnostic_event.DiagnosticEvent.diagnosisEventID)
  return _internal_diagnosiseventid();
}
inline void DiagnosticEvent::_internal_set_diagnosiseventid(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  diagnosiseventid_ = value;
}
inline void DiagnosticEvent::set_diagnosiseventid(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_diagnosiseventid(value);
  // @@protoc_insertion_point(field_set:pb.si.diagnostic_event.DiagnosticEvent.diagnosisEventID)
}

// optional .pb.eco.diagnosis_event_status.DiagnosisEventStatus diagnosisEventStatus = 870;
inline bool DiagnosticEvent::_internal_has_diagnosiseventstatus() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool DiagnosticEvent::has_diagnosiseventstatus() const {
  return _internal_has_diagnosiseventstatus();
}
inline void DiagnosticEvent::clear_diagnosiseventstatus() {
  diagnosiseventstatus_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::pb::eco::diagnosis_event_status::DiagnosisEventStatus DiagnosticEvent::_internal_diagnosiseventstatus() const {
  return static_cast< ::pb::eco::diagnosis_event_status::DiagnosisEventStatus >(diagnosiseventstatus_);
}
inline ::pb::eco::diagnosis_event_status::DiagnosisEventStatus DiagnosticEvent::diagnosiseventstatus() const {
  // @@protoc_insertion_point(field_get:pb.si.diagnostic_event.DiagnosticEvent.diagnosisEventStatus)
  return _internal_diagnosiseventstatus();
}
inline void DiagnosticEvent::_internal_set_diagnosiseventstatus(::pb::eco::diagnosis_event_status::DiagnosisEventStatus value) {
  assert(::pb::eco::diagnosis_event_status::DiagnosisEventStatus_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  diagnosiseventstatus_ = value;
}
inline void DiagnosticEvent::set_diagnosiseventstatus(::pb::eco::diagnosis_event_status::DiagnosisEventStatus value) {
  _internal_set_diagnosiseventstatus(value);
  // @@protoc_insertion_point(field_set:pb.si.diagnostic_event.DiagnosticEvent.diagnosisEventStatus)
}

// -------------------------------------------------------------------

// DiagnosticEvent_array_port

// repeated .pb.si.diagnostic_event.DiagnosticEvent data = 1456;
inline int DiagnosticEvent_array_port::_internal_data_size() const {
  return data_.size();
}
inline int DiagnosticEvent_array_port::data_size() const {
  return _internal_data_size();
}
inline void DiagnosticEvent_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::si::diagnostic_event::DiagnosticEvent* DiagnosticEvent_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.si.diagnostic_event.DiagnosticEvent_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::diagnostic_event::DiagnosticEvent >*
DiagnosticEvent_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.si.diagnostic_event.DiagnosticEvent_array_port.data)
  return &data_;
}
inline const ::pb::si::diagnostic_event::DiagnosticEvent& DiagnosticEvent_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::si::diagnostic_event::DiagnosticEvent& DiagnosticEvent_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.si.diagnostic_event.DiagnosticEvent_array_port.data)
  return _internal_data(index);
}
inline ::pb::si::diagnostic_event::DiagnosticEvent* DiagnosticEvent_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::si::diagnostic_event::DiagnosticEvent* DiagnosticEvent_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.si.diagnostic_event.DiagnosticEvent_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::diagnostic_event::DiagnosticEvent >&
DiagnosticEvent_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.si.diagnostic_event.DiagnosticEvent_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace diagnostic_event
}  // namespace si
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_si_2fdiagnostic_5fevent_2eproto
