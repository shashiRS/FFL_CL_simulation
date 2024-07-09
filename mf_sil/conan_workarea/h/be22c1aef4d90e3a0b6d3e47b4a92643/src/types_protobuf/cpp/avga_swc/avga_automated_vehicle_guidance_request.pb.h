// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: avga_swc/avga_automated_vehicle_guidance_request.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_avga_5fswc_2favga_5fautomated_5fvehicle_5fguidance_5frequest_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_avga_5fswc_2favga_5fautomated_5fvehicle_5fguidance_5frequest_2eproto

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
#include "avga_swc/automated_vehicle_guidance_request_type.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_avga_5fswc_2favga_5fautomated_5fvehicle_5fguidance_5frequest_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_avga_5fswc_2favga_5fautomated_5fvehicle_5fguidance_5frequest_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_avga_5fswc_2favga_5fautomated_5fvehicle_5fguidance_5frequest_2eproto;
namespace pb {
namespace avga_swc {
namespace avga_automated_vehicle_guidance_request {
class AVGA_AutomatedVehicleGuidanceRequest;
class AVGA_AutomatedVehicleGuidanceRequestDefaultTypeInternal;
extern AVGA_AutomatedVehicleGuidanceRequestDefaultTypeInternal _AVGA_AutomatedVehicleGuidanceRequest_default_instance_;
class AVGA_AutomatedVehicleGuidanceRequest_array_port;
class AVGA_AutomatedVehicleGuidanceRequest_array_portDefaultTypeInternal;
extern AVGA_AutomatedVehicleGuidanceRequest_array_portDefaultTypeInternal _AVGA_AutomatedVehicleGuidanceRequest_array_port_default_instance_;
}  // namespace avga_automated_vehicle_guidance_request
}  // namespace avga_swc
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest* Arena::CreateMaybeMessage<::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest>(Arena*);
template<> ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest_array_port* Arena::CreateMaybeMessage<::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace avga_swc {
namespace avga_automated_vehicle_guidance_request {

// ===================================================================

class AVGA_AutomatedVehicleGuidanceRequest :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest) */ {
 public:
  AVGA_AutomatedVehicleGuidanceRequest();
  virtual ~AVGA_AutomatedVehicleGuidanceRequest();

  AVGA_AutomatedVehicleGuidanceRequest(const AVGA_AutomatedVehicleGuidanceRequest& from);
  AVGA_AutomatedVehicleGuidanceRequest(AVGA_AutomatedVehicleGuidanceRequest&& from) noexcept
    : AVGA_AutomatedVehicleGuidanceRequest() {
    *this = ::std::move(from);
  }

  inline AVGA_AutomatedVehicleGuidanceRequest& operator=(const AVGA_AutomatedVehicleGuidanceRequest& from) {
    CopyFrom(from);
    return *this;
  }
  inline AVGA_AutomatedVehicleGuidanceRequest& operator=(AVGA_AutomatedVehicleGuidanceRequest&& from) noexcept {
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
  static const AVGA_AutomatedVehicleGuidanceRequest& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const AVGA_AutomatedVehicleGuidanceRequest* internal_default_instance() {
    return reinterpret_cast<const AVGA_AutomatedVehicleGuidanceRequest*>(
               &_AVGA_AutomatedVehicleGuidanceRequest_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(AVGA_AutomatedVehicleGuidanceRequest& a, AVGA_AutomatedVehicleGuidanceRequest& b) {
    a.Swap(&b);
  }
  inline void Swap(AVGA_AutomatedVehicleGuidanceRequest* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline AVGA_AutomatedVehicleGuidanceRequest* New() const final {
    return CreateMaybeMessage<AVGA_AutomatedVehicleGuidanceRequest>(nullptr);
  }

  AVGA_AutomatedVehicleGuidanceRequest* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<AVGA_AutomatedVehicleGuidanceRequest>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const AVGA_AutomatedVehicleGuidanceRequest& from);
  void MergeFrom(const AVGA_AutomatedVehicleGuidanceRequest& from);
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
  void InternalSwap(AVGA_AutomatedVehicleGuidanceRequest* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_avga_5fswc_2favga_5fautomated_5fvehicle_5fguidance_5frequest_2eproto);
    return ::descriptor_table_avga_5fswc_2favga_5fautomated_5fvehicle_5fguidance_5frequest_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kAutomatedVehicleGuidanceRequestFieldNumber = 3651,
  };
  // optional .pb.avga_swc.automated_vehicle_guidance_request_type.AutomatedVehicleGuidanceRequestType automatedVehicleGuidanceRequest = 3651;
  bool has_automatedvehicleguidancerequest() const;
  private:
  bool _internal_has_automatedvehicleguidancerequest() const;
  public:
  void clear_automatedvehicleguidancerequest();
  ::pb::avga_swc::automated_vehicle_guidance_request_type::AutomatedVehicleGuidanceRequestType automatedvehicleguidancerequest() const;
  void set_automatedvehicleguidancerequest(::pb::avga_swc::automated_vehicle_guidance_request_type::AutomatedVehicleGuidanceRequestType value);
  private:
  ::pb::avga_swc::automated_vehicle_guidance_request_type::AutomatedVehicleGuidanceRequestType _internal_automatedvehicleguidancerequest() const;
  void _internal_set_automatedvehicleguidancerequest(::pb::avga_swc::automated_vehicle_guidance_request_type::AutomatedVehicleGuidanceRequestType value);
  public:

  // @@protoc_insertion_point(class_scope:pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int automatedvehicleguidancerequest_;
  friend struct ::TableStruct_avga_5fswc_2favga_5fautomated_5fvehicle_5fguidance_5frequest_2eproto;
};
// -------------------------------------------------------------------

class AVGA_AutomatedVehicleGuidanceRequest_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest_array_port) */ {
 public:
  AVGA_AutomatedVehicleGuidanceRequest_array_port();
  virtual ~AVGA_AutomatedVehicleGuidanceRequest_array_port();

  AVGA_AutomatedVehicleGuidanceRequest_array_port(const AVGA_AutomatedVehicleGuidanceRequest_array_port& from);
  AVGA_AutomatedVehicleGuidanceRequest_array_port(AVGA_AutomatedVehicleGuidanceRequest_array_port&& from) noexcept
    : AVGA_AutomatedVehicleGuidanceRequest_array_port() {
    *this = ::std::move(from);
  }

  inline AVGA_AutomatedVehicleGuidanceRequest_array_port& operator=(const AVGA_AutomatedVehicleGuidanceRequest_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline AVGA_AutomatedVehicleGuidanceRequest_array_port& operator=(AVGA_AutomatedVehicleGuidanceRequest_array_port&& from) noexcept {
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
  static const AVGA_AutomatedVehicleGuidanceRequest_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const AVGA_AutomatedVehicleGuidanceRequest_array_port* internal_default_instance() {
    return reinterpret_cast<const AVGA_AutomatedVehicleGuidanceRequest_array_port*>(
               &_AVGA_AutomatedVehicleGuidanceRequest_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(AVGA_AutomatedVehicleGuidanceRequest_array_port& a, AVGA_AutomatedVehicleGuidanceRequest_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(AVGA_AutomatedVehicleGuidanceRequest_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline AVGA_AutomatedVehicleGuidanceRequest_array_port* New() const final {
    return CreateMaybeMessage<AVGA_AutomatedVehicleGuidanceRequest_array_port>(nullptr);
  }

  AVGA_AutomatedVehicleGuidanceRequest_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<AVGA_AutomatedVehicleGuidanceRequest_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const AVGA_AutomatedVehicleGuidanceRequest_array_port& from);
  void MergeFrom(const AVGA_AutomatedVehicleGuidanceRequest_array_port& from);
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
  void InternalSwap(AVGA_AutomatedVehicleGuidanceRequest_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_avga_5fswc_2favga_5fautomated_5fvehicle_5fguidance_5frequest_2eproto);
    return ::descriptor_table_avga_5fswc_2favga_5fautomated_5fvehicle_5fguidance_5frequest_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2066,
  };
  // repeated .pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest data = 2066;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest >*
      mutable_data();
  private:
  const ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest& _internal_data(int index) const;
  ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest* _internal_add_data();
  public:
  const ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest& data(int index) const;
  ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest > data_;
  friend struct ::TableStruct_avga_5fswc_2favga_5fautomated_5fvehicle_5fguidance_5frequest_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// AVGA_AutomatedVehicleGuidanceRequest

// optional .pb.avga_swc.automated_vehicle_guidance_request_type.AutomatedVehicleGuidanceRequestType automatedVehicleGuidanceRequest = 3651;
inline bool AVGA_AutomatedVehicleGuidanceRequest::_internal_has_automatedvehicleguidancerequest() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool AVGA_AutomatedVehicleGuidanceRequest::has_automatedvehicleguidancerequest() const {
  return _internal_has_automatedvehicleguidancerequest();
}
inline void AVGA_AutomatedVehicleGuidanceRequest::clear_automatedvehicleguidancerequest() {
  automatedvehicleguidancerequest_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::pb::avga_swc::automated_vehicle_guidance_request_type::AutomatedVehicleGuidanceRequestType AVGA_AutomatedVehicleGuidanceRequest::_internal_automatedvehicleguidancerequest() const {
  return static_cast< ::pb::avga_swc::automated_vehicle_guidance_request_type::AutomatedVehicleGuidanceRequestType >(automatedvehicleguidancerequest_);
}
inline ::pb::avga_swc::automated_vehicle_guidance_request_type::AutomatedVehicleGuidanceRequestType AVGA_AutomatedVehicleGuidanceRequest::automatedvehicleguidancerequest() const {
  // @@protoc_insertion_point(field_get:pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest.automatedVehicleGuidanceRequest)
  return _internal_automatedvehicleguidancerequest();
}
inline void AVGA_AutomatedVehicleGuidanceRequest::_internal_set_automatedvehicleguidancerequest(::pb::avga_swc::automated_vehicle_guidance_request_type::AutomatedVehicleGuidanceRequestType value) {
  assert(::pb::avga_swc::automated_vehicle_guidance_request_type::AutomatedVehicleGuidanceRequestType_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  automatedvehicleguidancerequest_ = value;
}
inline void AVGA_AutomatedVehicleGuidanceRequest::set_automatedvehicleguidancerequest(::pb::avga_swc::automated_vehicle_guidance_request_type::AutomatedVehicleGuidanceRequestType value) {
  _internal_set_automatedvehicleguidancerequest(value);
  // @@protoc_insertion_point(field_set:pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest.automatedVehicleGuidanceRequest)
}

// -------------------------------------------------------------------

// AVGA_AutomatedVehicleGuidanceRequest_array_port

// repeated .pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest data = 2066;
inline int AVGA_AutomatedVehicleGuidanceRequest_array_port::_internal_data_size() const {
  return data_.size();
}
inline int AVGA_AutomatedVehicleGuidanceRequest_array_port::data_size() const {
  return _internal_data_size();
}
inline void AVGA_AutomatedVehicleGuidanceRequest_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest* AVGA_AutomatedVehicleGuidanceRequest_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest >*
AVGA_AutomatedVehicleGuidanceRequest_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest_array_port.data)
  return &data_;
}
inline const ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest& AVGA_AutomatedVehicleGuidanceRequest_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest& AVGA_AutomatedVehicleGuidanceRequest_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest_array_port.data)
  return _internal_data(index);
}
inline ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest* AVGA_AutomatedVehicleGuidanceRequest_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest* AVGA_AutomatedVehicleGuidanceRequest_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::avga_swc::avga_automated_vehicle_guidance_request::AVGA_AutomatedVehicleGuidanceRequest >&
AVGA_AutomatedVehicleGuidanceRequest_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.avga_swc.avga_automated_vehicle_guidance_request.AVGA_AutomatedVehicleGuidanceRequest_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace avga_automated_vehicle_guidance_request
}  // namespace avga_swc
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_avga_5fswc_2favga_5fautomated_5fvehicle_5fguidance_5frequest_2eproto
