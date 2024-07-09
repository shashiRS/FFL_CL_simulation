// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_mempark/memory_parking_data_result_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmemory_5fparking_5fdata_5fresult_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmemory_5fparking_5fdata_5fresult_5fport_2eproto

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
#include "mf_mempark/storing_status.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fmempark_2fmemory_5fparking_5fdata_5fresult_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fmempark_2fmemory_5fparking_5fdata_5fresult_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmempark_2fmemory_5fparking_5fdata_5fresult_5fport_2eproto;
namespace pb {
namespace mf_mempark {
namespace memory_parking_data_result_port {
class MemoryParkingDataResultPort;
class MemoryParkingDataResultPortDefaultTypeInternal;
extern MemoryParkingDataResultPortDefaultTypeInternal _MemoryParkingDataResultPort_default_instance_;
class MemoryParkingDataResultPort_array_port;
class MemoryParkingDataResultPort_array_portDefaultTypeInternal;
extern MemoryParkingDataResultPort_array_portDefaultTypeInternal _MemoryParkingDataResultPort_array_port_default_instance_;
}  // namespace memory_parking_data_result_port
}  // namespace mf_mempark
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort* Arena::CreateMaybeMessage<::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort>(Arena*);
template<> ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort_array_port* Arena::CreateMaybeMessage<::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_mempark {
namespace memory_parking_data_result_port {

// ===================================================================

class MemoryParkingDataResultPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort) */ {
 public:
  MemoryParkingDataResultPort();
  virtual ~MemoryParkingDataResultPort();

  MemoryParkingDataResultPort(const MemoryParkingDataResultPort& from);
  MemoryParkingDataResultPort(MemoryParkingDataResultPort&& from) noexcept
    : MemoryParkingDataResultPort() {
    *this = ::std::move(from);
  }

  inline MemoryParkingDataResultPort& operator=(const MemoryParkingDataResultPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline MemoryParkingDataResultPort& operator=(MemoryParkingDataResultPort&& from) noexcept {
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
  static const MemoryParkingDataResultPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MemoryParkingDataResultPort* internal_default_instance() {
    return reinterpret_cast<const MemoryParkingDataResultPort*>(
               &_MemoryParkingDataResultPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MemoryParkingDataResultPort& a, MemoryParkingDataResultPort& b) {
    a.Swap(&b);
  }
  inline void Swap(MemoryParkingDataResultPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MemoryParkingDataResultPort* New() const final {
    return CreateMaybeMessage<MemoryParkingDataResultPort>(nullptr);
  }

  MemoryParkingDataResultPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MemoryParkingDataResultPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MemoryParkingDataResultPort& from);
  void MergeFrom(const MemoryParkingDataResultPort& from);
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
  void InternalSwap(MemoryParkingDataResultPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2fmemory_5fparking_5fdata_5fresult_5fport_2eproto);
    return ::descriptor_table_mf_5fmempark_2fmemory_5fparking_5fdata_5fresult_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kMetaMapIdDeletingFieldNumber = 2749,
    kMetaMapIdLoadingFieldNumber = 52,
    kDeletingStatusFieldNumber = 1430,
    kSavingStatusFieldNumber = 1540,
    kMetaMapIdSavingFieldNumber = 2092,
    kUiVersionNumberFieldNumber = 2124,
    kLoadingStatusFieldNumber = 2336,
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

  // optional uint32 metaMapIdDeleting = 2749;
  bool has_metamapiddeleting() const;
  private:
  bool _internal_has_metamapiddeleting() const;
  public:
  void clear_metamapiddeleting();
  ::PROTOBUF_NAMESPACE_ID::uint32 metamapiddeleting() const;
  void set_metamapiddeleting(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_metamapiddeleting() const;
  void _internal_set_metamapiddeleting(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 metaMapIdLoading = 52;
  bool has_metamapidloading() const;
  private:
  bool _internal_has_metamapidloading() const;
  public:
  void clear_metamapidloading();
  ::PROTOBUF_NAMESPACE_ID::uint32 metamapidloading() const;
  void set_metamapidloading(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_metamapidloading() const;
  void _internal_set_metamapidloading(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional .pb.mf_mempark.storing_status.StoringStatus deletingStatus = 1430;
  bool has_deletingstatus() const;
  private:
  bool _internal_has_deletingstatus() const;
  public:
  void clear_deletingstatus();
  ::pb::mf_mempark::storing_status::StoringStatus deletingstatus() const;
  void set_deletingstatus(::pb::mf_mempark::storing_status::StoringStatus value);
  private:
  ::pb::mf_mempark::storing_status::StoringStatus _internal_deletingstatus() const;
  void _internal_set_deletingstatus(::pb::mf_mempark::storing_status::StoringStatus value);
  public:

  // optional .pb.mf_mempark.storing_status.StoringStatus savingStatus = 1540;
  bool has_savingstatus() const;
  private:
  bool _internal_has_savingstatus() const;
  public:
  void clear_savingstatus();
  ::pb::mf_mempark::storing_status::StoringStatus savingstatus() const;
  void set_savingstatus(::pb::mf_mempark::storing_status::StoringStatus value);
  private:
  ::pb::mf_mempark::storing_status::StoringStatus _internal_savingstatus() const;
  void _internal_set_savingstatus(::pb::mf_mempark::storing_status::StoringStatus value);
  public:

  // optional uint32 metaMapIdSaving = 2092;
  bool has_metamapidsaving() const;
  private:
  bool _internal_has_metamapidsaving() const;
  public:
  void clear_metamapidsaving();
  ::PROTOBUF_NAMESPACE_ID::uint32 metamapidsaving() const;
  void set_metamapidsaving(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_metamapidsaving() const;
  void _internal_set_metamapidsaving(::PROTOBUF_NAMESPACE_ID::uint32 value);
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

  // optional .pb.mf_mempark.storing_status.StoringStatus loadingStatus = 2336;
  bool has_loadingstatus() const;
  private:
  bool _internal_has_loadingstatus() const;
  public:
  void clear_loadingstatus();
  ::pb::mf_mempark::storing_status::StoringStatus loadingstatus() const;
  void set_loadingstatus(::pb::mf_mempark::storing_status::StoringStatus value);
  private:
  ::pb::mf_mempark::storing_status::StoringStatus _internal_loadingstatus() const;
  void _internal_set_loadingstatus(::pb::mf_mempark::storing_status::StoringStatus value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint32 metamapiddeleting_;
  ::PROTOBUF_NAMESPACE_ID::uint32 metamapidloading_;
  int deletingstatus_;
  int savingstatus_;
  ::PROTOBUF_NAMESPACE_ID::uint32 metamapidsaving_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  int loadingstatus_;
  friend struct ::TableStruct_mf_5fmempark_2fmemory_5fparking_5fdata_5fresult_5fport_2eproto;
};
// -------------------------------------------------------------------

class MemoryParkingDataResultPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort_array_port) */ {
 public:
  MemoryParkingDataResultPort_array_port();
  virtual ~MemoryParkingDataResultPort_array_port();

  MemoryParkingDataResultPort_array_port(const MemoryParkingDataResultPort_array_port& from);
  MemoryParkingDataResultPort_array_port(MemoryParkingDataResultPort_array_port&& from) noexcept
    : MemoryParkingDataResultPort_array_port() {
    *this = ::std::move(from);
  }

  inline MemoryParkingDataResultPort_array_port& operator=(const MemoryParkingDataResultPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline MemoryParkingDataResultPort_array_port& operator=(MemoryParkingDataResultPort_array_port&& from) noexcept {
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
  static const MemoryParkingDataResultPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MemoryParkingDataResultPort_array_port* internal_default_instance() {
    return reinterpret_cast<const MemoryParkingDataResultPort_array_port*>(
               &_MemoryParkingDataResultPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(MemoryParkingDataResultPort_array_port& a, MemoryParkingDataResultPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(MemoryParkingDataResultPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MemoryParkingDataResultPort_array_port* New() const final {
    return CreateMaybeMessage<MemoryParkingDataResultPort_array_port>(nullptr);
  }

  MemoryParkingDataResultPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MemoryParkingDataResultPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MemoryParkingDataResultPort_array_port& from);
  void MergeFrom(const MemoryParkingDataResultPort_array_port& from);
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
  void InternalSwap(MemoryParkingDataResultPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2fmemory_5fparking_5fdata_5fresult_5fport_2eproto);
    return ::descriptor_table_mf_5fmempark_2fmemory_5fparking_5fdata_5fresult_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3563,
  };
  // repeated .pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort data = 3563;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort >*
      mutable_data();
  private:
  const ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort& _internal_data(int index) const;
  ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort* _internal_add_data();
  public:
  const ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort& data(int index) const;
  ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort > data_;
  friend struct ::TableStruct_mf_5fmempark_2fmemory_5fparking_5fdata_5fresult_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MemoryParkingDataResultPort

// optional uint32 uiVersionNumber = 2124;
inline bool MemoryParkingDataResultPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool MemoryParkingDataResultPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void MemoryParkingDataResultPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MemoryParkingDataResultPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MemoryParkingDataResultPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void MemoryParkingDataResultPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000040u;
  uiversionnumber_ = value;
}
inline void MemoryParkingDataResultPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool MemoryParkingDataResultPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool MemoryParkingDataResultPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& MemoryParkingDataResultPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& MemoryParkingDataResultPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* MemoryParkingDataResultPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* MemoryParkingDataResultPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* MemoryParkingDataResultPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void MemoryParkingDataResultPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.sSigHeader)
}

// optional uint32 metaMapIdSaving = 2092;
inline bool MemoryParkingDataResultPort::_internal_has_metamapidsaving() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool MemoryParkingDataResultPort::has_metamapidsaving() const {
  return _internal_has_metamapidsaving();
}
inline void MemoryParkingDataResultPort::clear_metamapidsaving() {
  metamapidsaving_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MemoryParkingDataResultPort::_internal_metamapidsaving() const {
  return metamapidsaving_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MemoryParkingDataResultPort::metamapidsaving() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.metaMapIdSaving)
  return _internal_metamapidsaving();
}
inline void MemoryParkingDataResultPort::_internal_set_metamapidsaving(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  metamapidsaving_ = value;
}
inline void MemoryParkingDataResultPort::set_metamapidsaving(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_metamapidsaving(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.metaMapIdSaving)
}

// optional .pb.mf_mempark.storing_status.StoringStatus savingStatus = 1540;
inline bool MemoryParkingDataResultPort::_internal_has_savingstatus() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool MemoryParkingDataResultPort::has_savingstatus() const {
  return _internal_has_savingstatus();
}
inline void MemoryParkingDataResultPort::clear_savingstatus() {
  savingstatus_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::pb::mf_mempark::storing_status::StoringStatus MemoryParkingDataResultPort::_internal_savingstatus() const {
  return static_cast< ::pb::mf_mempark::storing_status::StoringStatus >(savingstatus_);
}
inline ::pb::mf_mempark::storing_status::StoringStatus MemoryParkingDataResultPort::savingstatus() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.savingStatus)
  return _internal_savingstatus();
}
inline void MemoryParkingDataResultPort::_internal_set_savingstatus(::pb::mf_mempark::storing_status::StoringStatus value) {
  assert(::pb::mf_mempark::storing_status::StoringStatus_IsValid(value));
  _has_bits_[0] |= 0x00000010u;
  savingstatus_ = value;
}
inline void MemoryParkingDataResultPort::set_savingstatus(::pb::mf_mempark::storing_status::StoringStatus value) {
  _internal_set_savingstatus(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.savingStatus)
}

// optional uint32 metaMapIdLoading = 52;
inline bool MemoryParkingDataResultPort::_internal_has_metamapidloading() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool MemoryParkingDataResultPort::has_metamapidloading() const {
  return _internal_has_metamapidloading();
}
inline void MemoryParkingDataResultPort::clear_metamapidloading() {
  metamapidloading_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MemoryParkingDataResultPort::_internal_metamapidloading() const {
  return metamapidloading_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MemoryParkingDataResultPort::metamapidloading() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.metaMapIdLoading)
  return _internal_metamapidloading();
}
inline void MemoryParkingDataResultPort::_internal_set_metamapidloading(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  metamapidloading_ = value;
}
inline void MemoryParkingDataResultPort::set_metamapidloading(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_metamapidloading(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.metaMapIdLoading)
}

// optional .pb.mf_mempark.storing_status.StoringStatus loadingStatus = 2336;
inline bool MemoryParkingDataResultPort::_internal_has_loadingstatus() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool MemoryParkingDataResultPort::has_loadingstatus() const {
  return _internal_has_loadingstatus();
}
inline void MemoryParkingDataResultPort::clear_loadingstatus() {
  loadingstatus_ = 0;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::pb::mf_mempark::storing_status::StoringStatus MemoryParkingDataResultPort::_internal_loadingstatus() const {
  return static_cast< ::pb::mf_mempark::storing_status::StoringStatus >(loadingstatus_);
}
inline ::pb::mf_mempark::storing_status::StoringStatus MemoryParkingDataResultPort::loadingstatus() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.loadingStatus)
  return _internal_loadingstatus();
}
inline void MemoryParkingDataResultPort::_internal_set_loadingstatus(::pb::mf_mempark::storing_status::StoringStatus value) {
  assert(::pb::mf_mempark::storing_status::StoringStatus_IsValid(value));
  _has_bits_[0] |= 0x00000080u;
  loadingstatus_ = value;
}
inline void MemoryParkingDataResultPort::set_loadingstatus(::pb::mf_mempark::storing_status::StoringStatus value) {
  _internal_set_loadingstatus(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.loadingStatus)
}

// optional uint32 metaMapIdDeleting = 2749;
inline bool MemoryParkingDataResultPort::_internal_has_metamapiddeleting() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool MemoryParkingDataResultPort::has_metamapiddeleting() const {
  return _internal_has_metamapiddeleting();
}
inline void MemoryParkingDataResultPort::clear_metamapiddeleting() {
  metamapiddeleting_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MemoryParkingDataResultPort::_internal_metamapiddeleting() const {
  return metamapiddeleting_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MemoryParkingDataResultPort::metamapiddeleting() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.metaMapIdDeleting)
  return _internal_metamapiddeleting();
}
inline void MemoryParkingDataResultPort::_internal_set_metamapiddeleting(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  metamapiddeleting_ = value;
}
inline void MemoryParkingDataResultPort::set_metamapiddeleting(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_metamapiddeleting(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.metaMapIdDeleting)
}

// optional .pb.mf_mempark.storing_status.StoringStatus deletingStatus = 1430;
inline bool MemoryParkingDataResultPort::_internal_has_deletingstatus() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool MemoryParkingDataResultPort::has_deletingstatus() const {
  return _internal_has_deletingstatus();
}
inline void MemoryParkingDataResultPort::clear_deletingstatus() {
  deletingstatus_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::pb::mf_mempark::storing_status::StoringStatus MemoryParkingDataResultPort::_internal_deletingstatus() const {
  return static_cast< ::pb::mf_mempark::storing_status::StoringStatus >(deletingstatus_);
}
inline ::pb::mf_mempark::storing_status::StoringStatus MemoryParkingDataResultPort::deletingstatus() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.deletingStatus)
  return _internal_deletingstatus();
}
inline void MemoryParkingDataResultPort::_internal_set_deletingstatus(::pb::mf_mempark::storing_status::StoringStatus value) {
  assert(::pb::mf_mempark::storing_status::StoringStatus_IsValid(value));
  _has_bits_[0] |= 0x00000008u;
  deletingstatus_ = value;
}
inline void MemoryParkingDataResultPort::set_deletingstatus(::pb::mf_mempark::storing_status::StoringStatus value) {
  _internal_set_deletingstatus(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort.deletingStatus)
}

// -------------------------------------------------------------------

// MemoryParkingDataResultPort_array_port

// repeated .pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort data = 3563;
inline int MemoryParkingDataResultPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int MemoryParkingDataResultPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void MemoryParkingDataResultPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort* MemoryParkingDataResultPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort >*
MemoryParkingDataResultPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort_array_port.data)
  return &data_;
}
inline const ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort& MemoryParkingDataResultPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort& MemoryParkingDataResultPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort* MemoryParkingDataResultPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort* MemoryParkingDataResultPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::memory_parking_data_result_port::MemoryParkingDataResultPort >&
MemoryParkingDataResultPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_mempark.memory_parking_data_result_port.MemoryParkingDataResultPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace memory_parking_data_result_port
}  // namespace mf_mempark
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fmemory_5fparking_5fdata_5fresult_5fport_2eproto
