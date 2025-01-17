// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_processing/us_processing_diag_output.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fprocessing_2fus_5fprocessing_5fdiag_5foutput_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fprocessing_2fus_5fprocessing_5fdiag_5foutput_2eproto

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
#include "us_processing/us_processing_sensor_status.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_us_5fprocessing_2fus_5fprocessing_5fdiag_5foutput_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fprocessing_2fus_5fprocessing_5fdiag_5foutput_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fprocessing_2fus_5fprocessing_5fdiag_5foutput_2eproto;
namespace pb {
namespace us_processing {
namespace us_processing_diag_output {
class UsProcessingDiagOutput;
class UsProcessingDiagOutputDefaultTypeInternal;
extern UsProcessingDiagOutputDefaultTypeInternal _UsProcessingDiagOutput_default_instance_;
class UsProcessingDiagOutput_array_port;
class UsProcessingDiagOutput_array_portDefaultTypeInternal;
extern UsProcessingDiagOutput_array_portDefaultTypeInternal _UsProcessingDiagOutput_array_port_default_instance_;
}  // namespace us_processing_diag_output
}  // namespace us_processing
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput* Arena::CreateMaybeMessage<::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput>(Arena*);
template<> ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput_array_port* Arena::CreateMaybeMessage<::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_processing {
namespace us_processing_diag_output {

// ===================================================================

class UsProcessingDiagOutput :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput) */ {
 public:
  UsProcessingDiagOutput();
  virtual ~UsProcessingDiagOutput();

  UsProcessingDiagOutput(const UsProcessingDiagOutput& from);
  UsProcessingDiagOutput(UsProcessingDiagOutput&& from) noexcept
    : UsProcessingDiagOutput() {
    *this = ::std::move(from);
  }

  inline UsProcessingDiagOutput& operator=(const UsProcessingDiagOutput& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsProcessingDiagOutput& operator=(UsProcessingDiagOutput&& from) noexcept {
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
  static const UsProcessingDiagOutput& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsProcessingDiagOutput* internal_default_instance() {
    return reinterpret_cast<const UsProcessingDiagOutput*>(
               &_UsProcessingDiagOutput_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsProcessingDiagOutput& a, UsProcessingDiagOutput& b) {
    a.Swap(&b);
  }
  inline void Swap(UsProcessingDiagOutput* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsProcessingDiagOutput* New() const final {
    return CreateMaybeMessage<UsProcessingDiagOutput>(nullptr);
  }

  UsProcessingDiagOutput* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsProcessingDiagOutput>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsProcessingDiagOutput& from);
  void MergeFrom(const UsProcessingDiagOutput& from);
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
  void InternalSwap(UsProcessingDiagOutput* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fdiag_5foutput_2eproto);
    return ::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fdiag_5foutput_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSensorStatusFieldNumber = 403,
    kSSigHeaderFieldNumber = 1033,
    kUiVersionNumberFieldNumber = 2124,
  };
  // repeated .pb.us_processing.us_processing_sensor_status.UsProcessingSensorStatus sensorStatus = 403;
  int sensorstatus_size() const;
  private:
  int _internal_sensorstatus_size() const;
  public:
  void clear_sensorstatus();
  private:
  ::pb::us_processing::us_processing_sensor_status::UsProcessingSensorStatus _internal_sensorstatus(int index) const;
  void _internal_add_sensorstatus(::pb::us_processing::us_processing_sensor_status::UsProcessingSensorStatus value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>* _internal_mutable_sensorstatus();
  public:
  ::pb::us_processing::us_processing_sensor_status::UsProcessingSensorStatus sensorstatus(int index) const;
  void set_sensorstatus(int index, ::pb::us_processing::us_processing_sensor_status::UsProcessingSensorStatus value);
  void add_sensorstatus(::pb::us_processing::us_processing_sensor_status::UsProcessingSensorStatus value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>& sensorstatus() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>* mutable_sensorstatus();

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

  // @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField<int> sensorstatus_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  friend struct ::TableStruct_us_5fprocessing_2fus_5fprocessing_5fdiag_5foutput_2eproto;
};
// -------------------------------------------------------------------

class UsProcessingDiagOutput_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput_array_port) */ {
 public:
  UsProcessingDiagOutput_array_port();
  virtual ~UsProcessingDiagOutput_array_port();

  UsProcessingDiagOutput_array_port(const UsProcessingDiagOutput_array_port& from);
  UsProcessingDiagOutput_array_port(UsProcessingDiagOutput_array_port&& from) noexcept
    : UsProcessingDiagOutput_array_port() {
    *this = ::std::move(from);
  }

  inline UsProcessingDiagOutput_array_port& operator=(const UsProcessingDiagOutput_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsProcessingDiagOutput_array_port& operator=(UsProcessingDiagOutput_array_port&& from) noexcept {
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
  static const UsProcessingDiagOutput_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsProcessingDiagOutput_array_port* internal_default_instance() {
    return reinterpret_cast<const UsProcessingDiagOutput_array_port*>(
               &_UsProcessingDiagOutput_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsProcessingDiagOutput_array_port& a, UsProcessingDiagOutput_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsProcessingDiagOutput_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsProcessingDiagOutput_array_port* New() const final {
    return CreateMaybeMessage<UsProcessingDiagOutput_array_port>(nullptr);
  }

  UsProcessingDiagOutput_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsProcessingDiagOutput_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsProcessingDiagOutput_array_port& from);
  void MergeFrom(const UsProcessingDiagOutput_array_port& from);
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
  void InternalSwap(UsProcessingDiagOutput_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fdiag_5foutput_2eproto);
    return ::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fdiag_5foutput_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2599,
  };
  // repeated .pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput data = 2599;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput >*
      mutable_data();
  private:
  const ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput& _internal_data(int index) const;
  ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput* _internal_add_data();
  public:
  const ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput& data(int index) const;
  ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput > data_;
  friend struct ::TableStruct_us_5fprocessing_2fus_5fprocessing_5fdiag_5foutput_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsProcessingDiagOutput

// optional uint32 uiVersionNumber = 2124;
inline bool UsProcessingDiagOutput::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool UsProcessingDiagOutput::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void UsProcessingDiagOutput::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsProcessingDiagOutput::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsProcessingDiagOutput::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void UsProcessingDiagOutput::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  uiversionnumber_ = value;
}
inline void UsProcessingDiagOutput::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool UsProcessingDiagOutput::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool UsProcessingDiagOutput::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& UsProcessingDiagOutput::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& UsProcessingDiagOutput::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* UsProcessingDiagOutput::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* UsProcessingDiagOutput::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* UsProcessingDiagOutput::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void UsProcessingDiagOutput::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.sSigHeader)
}

// repeated .pb.us_processing.us_processing_sensor_status.UsProcessingSensorStatus sensorStatus = 403;
inline int UsProcessingDiagOutput::_internal_sensorstatus_size() const {
  return sensorstatus_.size();
}
inline int UsProcessingDiagOutput::sensorstatus_size() const {
  return _internal_sensorstatus_size();
}
inline void UsProcessingDiagOutput::clear_sensorstatus() {
  sensorstatus_.Clear();
}
inline ::pb::us_processing::us_processing_sensor_status::UsProcessingSensorStatus UsProcessingDiagOutput::_internal_sensorstatus(int index) const {
  return static_cast< ::pb::us_processing::us_processing_sensor_status::UsProcessingSensorStatus >(sensorstatus_.Get(index));
}
inline ::pb::us_processing::us_processing_sensor_status::UsProcessingSensorStatus UsProcessingDiagOutput::sensorstatus(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.sensorStatus)
  return _internal_sensorstatus(index);
}
inline void UsProcessingDiagOutput::set_sensorstatus(int index, ::pb::us_processing::us_processing_sensor_status::UsProcessingSensorStatus value) {
  assert(::pb::us_processing::us_processing_sensor_status::UsProcessingSensorStatus_IsValid(value));
  sensorstatus_.Set(index, value);
  // @@protoc_insertion_point(field_set:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.sensorStatus)
}
inline void UsProcessingDiagOutput::_internal_add_sensorstatus(::pb::us_processing::us_processing_sensor_status::UsProcessingSensorStatus value) {
  assert(::pb::us_processing::us_processing_sensor_status::UsProcessingSensorStatus_IsValid(value));
  sensorstatus_.Add(value);
}
inline void UsProcessingDiagOutput::add_sensorstatus(::pb::us_processing::us_processing_sensor_status::UsProcessingSensorStatus value) {
  // @@protoc_insertion_point(field_add:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.sensorStatus)
  _internal_add_sensorstatus(value);
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>&
UsProcessingDiagOutput::sensorstatus() const {
  // @@protoc_insertion_point(field_list:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.sensorStatus)
  return sensorstatus_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>*
UsProcessingDiagOutput::_internal_mutable_sensorstatus() {
  return &sensorstatus_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>*
UsProcessingDiagOutput::mutable_sensorstatus() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.sensorStatus)
  return _internal_mutable_sensorstatus();
}

// -------------------------------------------------------------------

// UsProcessingDiagOutput_array_port

// repeated .pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput data = 2599;
inline int UsProcessingDiagOutput_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsProcessingDiagOutput_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsProcessingDiagOutput_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput* UsProcessingDiagOutput_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput >*
UsProcessingDiagOutput_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput_array_port.data)
  return &data_;
}
inline const ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput& UsProcessingDiagOutput_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput& UsProcessingDiagOutput_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput* UsProcessingDiagOutput_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput* UsProcessingDiagOutput_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_processing::us_processing_diag_output::UsProcessingDiagOutput >&
UsProcessingDiagOutput_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_processing_diag_output
}  // namespace us_processing
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fprocessing_2fus_5fprocessing_5fdiag_5foutput_2eproto
