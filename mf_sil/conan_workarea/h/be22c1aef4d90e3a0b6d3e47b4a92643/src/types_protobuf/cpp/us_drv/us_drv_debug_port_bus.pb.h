// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_drv/us_drv_debug_port_bus.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fdebug_5fport_5fbus_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fdebug_5fport_5fbus_2eproto

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
#include "us_drv/us_drv_crm_status_counts.pb.h"
#include "us_drv/us_drv_pdcm_status_counts.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fdebug_5fport_5fbus_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fdrv_2fus_5fdrv_5fdebug_5fport_5fbus_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fdrv_2fus_5fdrv_5fdebug_5fport_5fbus_2eproto;
namespace pb {
namespace us_drv {
namespace us_drv_debug_port_bus {
class UsDrvDebugPortBus;
class UsDrvDebugPortBusDefaultTypeInternal;
extern UsDrvDebugPortBusDefaultTypeInternal _UsDrvDebugPortBus_default_instance_;
class UsDrvDebugPortBus_array_port;
class UsDrvDebugPortBus_array_portDefaultTypeInternal;
extern UsDrvDebugPortBus_array_portDefaultTypeInternal _UsDrvDebugPortBus_array_port_default_instance_;
}  // namespace us_drv_debug_port_bus
}  // namespace us_drv
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus>(Arena*);
template<> ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus_array_port* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_drv {
namespace us_drv_debug_port_bus {

// ===================================================================

class UsDrvDebugPortBus :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus) */ {
 public:
  UsDrvDebugPortBus();
  virtual ~UsDrvDebugPortBus();

  UsDrvDebugPortBus(const UsDrvDebugPortBus& from);
  UsDrvDebugPortBus(UsDrvDebugPortBus&& from) noexcept
    : UsDrvDebugPortBus() {
    *this = ::std::move(from);
  }

  inline UsDrvDebugPortBus& operator=(const UsDrvDebugPortBus& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvDebugPortBus& operator=(UsDrvDebugPortBus&& from) noexcept {
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
  static const UsDrvDebugPortBus& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvDebugPortBus* internal_default_instance() {
    return reinterpret_cast<const UsDrvDebugPortBus*>(
               &_UsDrvDebugPortBus_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsDrvDebugPortBus& a, UsDrvDebugPortBus& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvDebugPortBus* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvDebugPortBus* New() const final {
    return CreateMaybeMessage<UsDrvDebugPortBus>(nullptr);
  }

  UsDrvDebugPortBus* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvDebugPortBus>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvDebugPortBus& from);
  void MergeFrom(const UsDrvDebugPortBus& from);
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
  void InternalSwap(UsDrvDebugPortBus* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fdebug_5fport_5fbus_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fdebug_5fport_5fbus_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kPdcmStatusCountsFieldNumber = 604,
    kCrmStatusCountsFieldNumber = 1593,
    kCrmCountFieldNumber = 3244,
    kPdcmErrorCountFieldNumber = 1449,
    kPdcmCountFieldNumber = 2015,
    kCrmErrorCountFieldNumber = 2715,
    kPdcmSizeMismatchCountFieldNumber = 2811,
  };
  // optional .pb.us_drv.us_drv_pdcm_status_counts.UsDrvPdcmStatusCounts pdcmStatusCounts = 604;
  bool has_pdcmstatuscounts() const;
  private:
  bool _internal_has_pdcmstatuscounts() const;
  public:
  void clear_pdcmstatuscounts();
  const ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts& pdcmstatuscounts() const;
  ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts* release_pdcmstatuscounts();
  ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts* mutable_pdcmstatuscounts();
  void set_allocated_pdcmstatuscounts(::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts* pdcmstatuscounts);
  private:
  const ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts& _internal_pdcmstatuscounts() const;
  ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts* _internal_mutable_pdcmstatuscounts();
  public:

  // optional .pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts crmStatusCounts = 1593;
  bool has_crmstatuscounts() const;
  private:
  bool _internal_has_crmstatuscounts() const;
  public:
  void clear_crmstatuscounts();
  const ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts& crmstatuscounts() const;
  ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* release_crmstatuscounts();
  ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* mutable_crmstatuscounts();
  void set_allocated_crmstatuscounts(::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* crmstatuscounts);
  private:
  const ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts& _internal_crmstatuscounts() const;
  ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* _internal_mutable_crmstatuscounts();
  public:

  // optional uint32 crmCount = 3244;
  bool has_crmcount() const;
  private:
  bool _internal_has_crmcount() const;
  public:
  void clear_crmcount();
  ::PROTOBUF_NAMESPACE_ID::uint32 crmcount() const;
  void set_crmcount(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_crmcount() const;
  void _internal_set_crmcount(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 pdcmErrorCount = 1449;
  bool has_pdcmerrorcount() const;
  private:
  bool _internal_has_pdcmerrorcount() const;
  public:
  void clear_pdcmerrorcount();
  ::PROTOBUF_NAMESPACE_ID::uint32 pdcmerrorcount() const;
  void set_pdcmerrorcount(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_pdcmerrorcount() const;
  void _internal_set_pdcmerrorcount(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 pdcmCount = 2015;
  bool has_pdcmcount() const;
  private:
  bool _internal_has_pdcmcount() const;
  public:
  void clear_pdcmcount();
  ::PROTOBUF_NAMESPACE_ID::uint32 pdcmcount() const;
  void set_pdcmcount(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_pdcmcount() const;
  void _internal_set_pdcmcount(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 crmErrorCount = 2715;
  bool has_crmerrorcount() const;
  private:
  bool _internal_has_crmerrorcount() const;
  public:
  void clear_crmerrorcount();
  ::PROTOBUF_NAMESPACE_ID::uint32 crmerrorcount() const;
  void set_crmerrorcount(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_crmerrorcount() const;
  void _internal_set_crmerrorcount(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 pdcmSizeMismatchCount = 2811;
  bool has_pdcmsizemismatchcount() const;
  private:
  bool _internal_has_pdcmsizemismatchcount() const;
  public:
  void clear_pdcmsizemismatchcount();
  ::PROTOBUF_NAMESPACE_ID::uint32 pdcmsizemismatchcount() const;
  void set_pdcmsizemismatchcount(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_pdcmsizemismatchcount() const;
  void _internal_set_pdcmsizemismatchcount(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts* pdcmstatuscounts_;
  ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* crmstatuscounts_;
  ::PROTOBUF_NAMESPACE_ID::uint32 crmcount_;
  ::PROTOBUF_NAMESPACE_ID::uint32 pdcmerrorcount_;
  ::PROTOBUF_NAMESPACE_ID::uint32 pdcmcount_;
  ::PROTOBUF_NAMESPACE_ID::uint32 crmerrorcount_;
  ::PROTOBUF_NAMESPACE_ID::uint32 pdcmsizemismatchcount_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fdebug_5fport_5fbus_2eproto;
};
// -------------------------------------------------------------------

class UsDrvDebugPortBus_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus_array_port) */ {
 public:
  UsDrvDebugPortBus_array_port();
  virtual ~UsDrvDebugPortBus_array_port();

  UsDrvDebugPortBus_array_port(const UsDrvDebugPortBus_array_port& from);
  UsDrvDebugPortBus_array_port(UsDrvDebugPortBus_array_port&& from) noexcept
    : UsDrvDebugPortBus_array_port() {
    *this = ::std::move(from);
  }

  inline UsDrvDebugPortBus_array_port& operator=(const UsDrvDebugPortBus_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvDebugPortBus_array_port& operator=(UsDrvDebugPortBus_array_port&& from) noexcept {
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
  static const UsDrvDebugPortBus_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvDebugPortBus_array_port* internal_default_instance() {
    return reinterpret_cast<const UsDrvDebugPortBus_array_port*>(
               &_UsDrvDebugPortBus_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsDrvDebugPortBus_array_port& a, UsDrvDebugPortBus_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvDebugPortBus_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvDebugPortBus_array_port* New() const final {
    return CreateMaybeMessage<UsDrvDebugPortBus_array_port>(nullptr);
  }

  UsDrvDebugPortBus_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvDebugPortBus_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvDebugPortBus_array_port& from);
  void MergeFrom(const UsDrvDebugPortBus_array_port& from);
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
  void InternalSwap(UsDrvDebugPortBus_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fdebug_5fport_5fbus_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fdebug_5fport_5fbus_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3783,
  };
  // repeated .pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus data = 3783;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus >*
      mutable_data();
  private:
  const ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus& _internal_data(int index) const;
  ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus* _internal_add_data();
  public:
  const ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus& data(int index) const;
  ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus > data_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fdebug_5fport_5fbus_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsDrvDebugPortBus

// optional uint32 crmCount = 3244;
inline bool UsDrvDebugPortBus::_internal_has_crmcount() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool UsDrvDebugPortBus::has_crmcount() const {
  return _internal_has_crmcount();
}
inline void UsDrvDebugPortBus::clear_crmcount() {
  crmcount_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDebugPortBus::_internal_crmcount() const {
  return crmcount_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDebugPortBus::crmcount() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.crmCount)
  return _internal_crmcount();
}
inline void UsDrvDebugPortBus::_internal_set_crmcount(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  crmcount_ = value;
}
inline void UsDrvDebugPortBus::set_crmcount(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_crmcount(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.crmCount)
}

// optional uint32 crmErrorCount = 2715;
inline bool UsDrvDebugPortBus::_internal_has_crmerrorcount() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool UsDrvDebugPortBus::has_crmerrorcount() const {
  return _internal_has_crmerrorcount();
}
inline void UsDrvDebugPortBus::clear_crmerrorcount() {
  crmerrorcount_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDebugPortBus::_internal_crmerrorcount() const {
  return crmerrorcount_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDebugPortBus::crmerrorcount() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.crmErrorCount)
  return _internal_crmerrorcount();
}
inline void UsDrvDebugPortBus::_internal_set_crmerrorcount(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  crmerrorcount_ = value;
}
inline void UsDrvDebugPortBus::set_crmerrorcount(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_crmerrorcount(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.crmErrorCount)
}

// optional .pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts crmStatusCounts = 1593;
inline bool UsDrvDebugPortBus::_internal_has_crmstatuscounts() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || crmstatuscounts_ != nullptr);
  return value;
}
inline bool UsDrvDebugPortBus::has_crmstatuscounts() const {
  return _internal_has_crmstatuscounts();
}
inline const ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts& UsDrvDebugPortBus::_internal_crmstatuscounts() const {
  const ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* p = crmstatuscounts_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts*>(
      &::pb::us_drv::us_drv_crm_status_counts::_UsDrvCrmStatusCounts_default_instance_);
}
inline const ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts& UsDrvDebugPortBus::crmstatuscounts() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.crmStatusCounts)
  return _internal_crmstatuscounts();
}
inline ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* UsDrvDebugPortBus::release_crmstatuscounts() {
  // @@protoc_insertion_point(field_release:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.crmStatusCounts)
  _has_bits_[0] &= ~0x00000002u;
  ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* temp = crmstatuscounts_;
  crmstatuscounts_ = nullptr;
  return temp;
}
inline ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* UsDrvDebugPortBus::_internal_mutable_crmstatuscounts() {
  _has_bits_[0] |= 0x00000002u;
  if (crmstatuscounts_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts>(GetArenaNoVirtual());
    crmstatuscounts_ = p;
  }
  return crmstatuscounts_;
}
inline ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* UsDrvDebugPortBus::mutable_crmstatuscounts() {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.crmStatusCounts)
  return _internal_mutable_crmstatuscounts();
}
inline void UsDrvDebugPortBus::set_allocated_crmstatuscounts(::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* crmstatuscounts) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(crmstatuscounts_);
  }
  if (crmstatuscounts) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      crmstatuscounts = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, crmstatuscounts, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  crmstatuscounts_ = crmstatuscounts;
  // @@protoc_insertion_point(field_set_allocated:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.crmStatusCounts)
}

// optional uint32 pdcmCount = 2015;
inline bool UsDrvDebugPortBus::_internal_has_pdcmcount() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool UsDrvDebugPortBus::has_pdcmcount() const {
  return _internal_has_pdcmcount();
}
inline void UsDrvDebugPortBus::clear_pdcmcount() {
  pdcmcount_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDebugPortBus::_internal_pdcmcount() const {
  return pdcmcount_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDebugPortBus::pdcmcount() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.pdcmCount)
  return _internal_pdcmcount();
}
inline void UsDrvDebugPortBus::_internal_set_pdcmcount(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  pdcmcount_ = value;
}
inline void UsDrvDebugPortBus::set_pdcmcount(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_pdcmcount(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.pdcmCount)
}

// optional uint32 pdcmErrorCount = 1449;
inline bool UsDrvDebugPortBus::_internal_has_pdcmerrorcount() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool UsDrvDebugPortBus::has_pdcmerrorcount() const {
  return _internal_has_pdcmerrorcount();
}
inline void UsDrvDebugPortBus::clear_pdcmerrorcount() {
  pdcmerrorcount_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDebugPortBus::_internal_pdcmerrorcount() const {
  return pdcmerrorcount_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDebugPortBus::pdcmerrorcount() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.pdcmErrorCount)
  return _internal_pdcmerrorcount();
}
inline void UsDrvDebugPortBus::_internal_set_pdcmerrorcount(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  pdcmerrorcount_ = value;
}
inline void UsDrvDebugPortBus::set_pdcmerrorcount(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_pdcmerrorcount(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.pdcmErrorCount)
}

// optional uint32 pdcmSizeMismatchCount = 2811;
inline bool UsDrvDebugPortBus::_internal_has_pdcmsizemismatchcount() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool UsDrvDebugPortBus::has_pdcmsizemismatchcount() const {
  return _internal_has_pdcmsizemismatchcount();
}
inline void UsDrvDebugPortBus::clear_pdcmsizemismatchcount() {
  pdcmsizemismatchcount_ = 0u;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDebugPortBus::_internal_pdcmsizemismatchcount() const {
  return pdcmsizemismatchcount_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvDebugPortBus::pdcmsizemismatchcount() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.pdcmSizeMismatchCount)
  return _internal_pdcmsizemismatchcount();
}
inline void UsDrvDebugPortBus::_internal_set_pdcmsizemismatchcount(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000040u;
  pdcmsizemismatchcount_ = value;
}
inline void UsDrvDebugPortBus::set_pdcmsizemismatchcount(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_pdcmsizemismatchcount(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.pdcmSizeMismatchCount)
}

// optional .pb.us_drv.us_drv_pdcm_status_counts.UsDrvPdcmStatusCounts pdcmStatusCounts = 604;
inline bool UsDrvDebugPortBus::_internal_has_pdcmstatuscounts() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || pdcmstatuscounts_ != nullptr);
  return value;
}
inline bool UsDrvDebugPortBus::has_pdcmstatuscounts() const {
  return _internal_has_pdcmstatuscounts();
}
inline const ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts& UsDrvDebugPortBus::_internal_pdcmstatuscounts() const {
  const ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts* p = pdcmstatuscounts_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts*>(
      &::pb::us_drv::us_drv_pdcm_status_counts::_UsDrvPdcmStatusCounts_default_instance_);
}
inline const ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts& UsDrvDebugPortBus::pdcmstatuscounts() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.pdcmStatusCounts)
  return _internal_pdcmstatuscounts();
}
inline ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts* UsDrvDebugPortBus::release_pdcmstatuscounts() {
  // @@protoc_insertion_point(field_release:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.pdcmStatusCounts)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts* temp = pdcmstatuscounts_;
  pdcmstatuscounts_ = nullptr;
  return temp;
}
inline ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts* UsDrvDebugPortBus::_internal_mutable_pdcmstatuscounts() {
  _has_bits_[0] |= 0x00000001u;
  if (pdcmstatuscounts_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts>(GetArenaNoVirtual());
    pdcmstatuscounts_ = p;
  }
  return pdcmstatuscounts_;
}
inline ::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts* UsDrvDebugPortBus::mutable_pdcmstatuscounts() {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.pdcmStatusCounts)
  return _internal_mutable_pdcmstatuscounts();
}
inline void UsDrvDebugPortBus::set_allocated_pdcmstatuscounts(::pb::us_drv::us_drv_pdcm_status_counts::UsDrvPdcmStatusCounts* pdcmstatuscounts) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(pdcmstatuscounts_);
  }
  if (pdcmstatuscounts) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      pdcmstatuscounts = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, pdcmstatuscounts, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  pdcmstatuscounts_ = pdcmstatuscounts;
  // @@protoc_insertion_point(field_set_allocated:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus.pdcmStatusCounts)
}

// -------------------------------------------------------------------

// UsDrvDebugPortBus_array_port

// repeated .pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus data = 3783;
inline int UsDrvDebugPortBus_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsDrvDebugPortBus_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsDrvDebugPortBus_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus* UsDrvDebugPortBus_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus >*
UsDrvDebugPortBus_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus_array_port.data)
  return &data_;
}
inline const ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus& UsDrvDebugPortBus_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus& UsDrvDebugPortBus_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus* UsDrvDebugPortBus_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus* UsDrvDebugPortBus_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_debug_port_bus::UsDrvDebugPortBus >&
UsDrvDebugPortBus_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_drv_debug_port_bus
}  // namespace us_drv
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fdebug_5fport_5fbus_2eproto