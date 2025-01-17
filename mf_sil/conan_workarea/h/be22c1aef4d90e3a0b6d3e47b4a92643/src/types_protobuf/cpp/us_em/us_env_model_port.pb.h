// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_em/us_env_model_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fem_2fus_5fenv_5fmodel_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fem_2fus_5fenv_5fmodel_5fport_2eproto

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
#include "us_em/dynamic_object_serializable.pb.h"
#include "us_em/static_object_serializable.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_us_5fem_2fus_5fenv_5fmodel_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fem_2fus_5fenv_5fmodel_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fem_2fus_5fenv_5fmodel_5fport_2eproto;
namespace pb {
namespace us_em {
namespace us_env_model_port {
class UsEnvModelPort;
class UsEnvModelPortDefaultTypeInternal;
extern UsEnvModelPortDefaultTypeInternal _UsEnvModelPort_default_instance_;
class UsEnvModelPort_array_port;
class UsEnvModelPort_array_portDefaultTypeInternal;
extern UsEnvModelPort_array_portDefaultTypeInternal _UsEnvModelPort_array_port_default_instance_;
}  // namespace us_env_model_port
}  // namespace us_em
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_em::us_env_model_port::UsEnvModelPort* Arena::CreateMaybeMessage<::pb::us_em::us_env_model_port::UsEnvModelPort>(Arena*);
template<> ::pb::us_em::us_env_model_port::UsEnvModelPort_array_port* Arena::CreateMaybeMessage<::pb::us_em::us_env_model_port::UsEnvModelPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_em {
namespace us_env_model_port {

// ===================================================================

class UsEnvModelPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_em.us_env_model_port.UsEnvModelPort) */ {
 public:
  UsEnvModelPort();
  virtual ~UsEnvModelPort();

  UsEnvModelPort(const UsEnvModelPort& from);
  UsEnvModelPort(UsEnvModelPort&& from) noexcept
    : UsEnvModelPort() {
    *this = ::std::move(from);
  }

  inline UsEnvModelPort& operator=(const UsEnvModelPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsEnvModelPort& operator=(UsEnvModelPort&& from) noexcept {
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
  static const UsEnvModelPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsEnvModelPort* internal_default_instance() {
    return reinterpret_cast<const UsEnvModelPort*>(
               &_UsEnvModelPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsEnvModelPort& a, UsEnvModelPort& b) {
    a.Swap(&b);
  }
  inline void Swap(UsEnvModelPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsEnvModelPort* New() const final {
    return CreateMaybeMessage<UsEnvModelPort>(nullptr);
  }

  UsEnvModelPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsEnvModelPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsEnvModelPort& from);
  void MergeFrom(const UsEnvModelPort& from);
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
  void InternalSwap(UsEnvModelPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_em.us_env_model_port.UsEnvModelPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fem_2fus_5fenv_5fmodel_5fport_2eproto);
    return ::descriptor_table_us_5fem_2fus_5fenv_5fmodel_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDynamicObjectsFieldNumber = 482,
    kStaticObjectsFieldNumber = 3098,
    kSSigHeaderFieldNumber = 1033,
    kNumberOfDynamicObjectsU8FieldNumber = 3825,
    kNumberOfStaticObjectsU8FieldNumber = 186,
    kFirstDynObjOutDetZoneIdxU8FieldNumber = 1726,
    kUiVersionNumberFieldNumber = 2124,
    kFirstStatObjOutDetZoneIdxU8FieldNumber = 2215,
  };
  // repeated .pb.us_em.dynamic_object_serializable.DynamicObjectSerializable dynamicObjects = 482;
  int dynamicobjects_size() const;
  private:
  int _internal_dynamicobjects_size() const;
  public:
  void clear_dynamicobjects();
  ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable* mutable_dynamicobjects(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable >*
      mutable_dynamicobjects();
  private:
  const ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable& _internal_dynamicobjects(int index) const;
  ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable* _internal_add_dynamicobjects();
  public:
  const ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable& dynamicobjects(int index) const;
  ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable* add_dynamicobjects();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable >&
      dynamicobjects() const;

  // repeated .pb.us_em.static_object_serializable.StaticObjectSerializable staticObjects = 3098;
  int staticobjects_size() const;
  private:
  int _internal_staticobjects_size() const;
  public:
  void clear_staticobjects();
  ::pb::us_em::static_object_serializable::StaticObjectSerializable* mutable_staticobjects(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::static_object_serializable::StaticObjectSerializable >*
      mutable_staticobjects();
  private:
  const ::pb::us_em::static_object_serializable::StaticObjectSerializable& _internal_staticobjects(int index) const;
  ::pb::us_em::static_object_serializable::StaticObjectSerializable* _internal_add_staticobjects();
  public:
  const ::pb::us_em::static_object_serializable::StaticObjectSerializable& staticobjects(int index) const;
  ::pb::us_em::static_object_serializable::StaticObjectSerializable* add_staticobjects();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::static_object_serializable::StaticObjectSerializable >&
      staticobjects() const;

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

  // optional uint32 numberOfDynamicObjects_u8 = 3825;
  bool has_numberofdynamicobjects_u8() const;
  private:
  bool _internal_has_numberofdynamicobjects_u8() const;
  public:
  void clear_numberofdynamicobjects_u8();
  ::PROTOBUF_NAMESPACE_ID::uint32 numberofdynamicobjects_u8() const;
  void set_numberofdynamicobjects_u8(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_numberofdynamicobjects_u8() const;
  void _internal_set_numberofdynamicobjects_u8(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 numberOfStaticObjects_u8 = 186;
  bool has_numberofstaticobjects_u8() const;
  private:
  bool _internal_has_numberofstaticobjects_u8() const;
  public:
  void clear_numberofstaticobjects_u8();
  ::PROTOBUF_NAMESPACE_ID::uint32 numberofstaticobjects_u8() const;
  void set_numberofstaticobjects_u8(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_numberofstaticobjects_u8() const;
  void _internal_set_numberofstaticobjects_u8(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 firstDynObjOutDetZoneIdx_u8 = 1726;
  bool has_firstdynobjoutdetzoneidx_u8() const;
  private:
  bool _internal_has_firstdynobjoutdetzoneidx_u8() const;
  public:
  void clear_firstdynobjoutdetzoneidx_u8();
  ::PROTOBUF_NAMESPACE_ID::uint32 firstdynobjoutdetzoneidx_u8() const;
  void set_firstdynobjoutdetzoneidx_u8(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_firstdynobjoutdetzoneidx_u8() const;
  void _internal_set_firstdynobjoutdetzoneidx_u8(::PROTOBUF_NAMESPACE_ID::uint32 value);
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

  // optional uint32 firstStatObjOutDetZoneIdx_u8 = 2215;
  bool has_firststatobjoutdetzoneidx_u8() const;
  private:
  bool _internal_has_firststatobjoutdetzoneidx_u8() const;
  public:
  void clear_firststatobjoutdetzoneidx_u8();
  ::PROTOBUF_NAMESPACE_ID::uint32 firststatobjoutdetzoneidx_u8() const;
  void set_firststatobjoutdetzoneidx_u8(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_firststatobjoutdetzoneidx_u8() const;
  void _internal_set_firststatobjoutdetzoneidx_u8(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.us_em.us_env_model_port.UsEnvModelPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable > dynamicobjects_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::static_object_serializable::StaticObjectSerializable > staticobjects_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint32 numberofdynamicobjects_u8_;
  ::PROTOBUF_NAMESPACE_ID::uint32 numberofstaticobjects_u8_;
  ::PROTOBUF_NAMESPACE_ID::uint32 firstdynobjoutdetzoneidx_u8_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  ::PROTOBUF_NAMESPACE_ID::uint32 firststatobjoutdetzoneidx_u8_;
  friend struct ::TableStruct_us_5fem_2fus_5fenv_5fmodel_5fport_2eproto;
};
// -------------------------------------------------------------------

class UsEnvModelPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_em.us_env_model_port.UsEnvModelPort_array_port) */ {
 public:
  UsEnvModelPort_array_port();
  virtual ~UsEnvModelPort_array_port();

  UsEnvModelPort_array_port(const UsEnvModelPort_array_port& from);
  UsEnvModelPort_array_port(UsEnvModelPort_array_port&& from) noexcept
    : UsEnvModelPort_array_port() {
    *this = ::std::move(from);
  }

  inline UsEnvModelPort_array_port& operator=(const UsEnvModelPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsEnvModelPort_array_port& operator=(UsEnvModelPort_array_port&& from) noexcept {
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
  static const UsEnvModelPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsEnvModelPort_array_port* internal_default_instance() {
    return reinterpret_cast<const UsEnvModelPort_array_port*>(
               &_UsEnvModelPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsEnvModelPort_array_port& a, UsEnvModelPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsEnvModelPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsEnvModelPort_array_port* New() const final {
    return CreateMaybeMessage<UsEnvModelPort_array_port>(nullptr);
  }

  UsEnvModelPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsEnvModelPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsEnvModelPort_array_port& from);
  void MergeFrom(const UsEnvModelPort_array_port& from);
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
  void InternalSwap(UsEnvModelPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_em.us_env_model_port.UsEnvModelPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fem_2fus_5fenv_5fmodel_5fport_2eproto);
    return ::descriptor_table_us_5fem_2fus_5fenv_5fmodel_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3989,
  };
  // repeated .pb.us_em.us_env_model_port.UsEnvModelPort data = 3989;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_em::us_env_model_port::UsEnvModelPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_env_model_port::UsEnvModelPort >*
      mutable_data();
  private:
  const ::pb::us_em::us_env_model_port::UsEnvModelPort& _internal_data(int index) const;
  ::pb::us_em::us_env_model_port::UsEnvModelPort* _internal_add_data();
  public:
  const ::pb::us_em::us_env_model_port::UsEnvModelPort& data(int index) const;
  ::pb::us_em::us_env_model_port::UsEnvModelPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_env_model_port::UsEnvModelPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_em.us_env_model_port.UsEnvModelPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_env_model_port::UsEnvModelPort > data_;
  friend struct ::TableStruct_us_5fem_2fus_5fenv_5fmodel_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsEnvModelPort

// optional uint32 uiVersionNumber = 2124;
inline bool UsEnvModelPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool UsEnvModelPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void UsEnvModelPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEnvModelPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEnvModelPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_env_model_port.UsEnvModelPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void UsEnvModelPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  uiversionnumber_ = value;
}
inline void UsEnvModelPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_env_model_port.UsEnvModelPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool UsEnvModelPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool UsEnvModelPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& UsEnvModelPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& UsEnvModelPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_env_model_port.UsEnvModelPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* UsEnvModelPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.us_em.us_env_model_port.UsEnvModelPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* UsEnvModelPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* UsEnvModelPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_env_model_port.UsEnvModelPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void UsEnvModelPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.us_em.us_env_model_port.UsEnvModelPort.sSigHeader)
}

// optional uint32 numberOfStaticObjects_u8 = 186;
inline bool UsEnvModelPort::_internal_has_numberofstaticobjects_u8() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool UsEnvModelPort::has_numberofstaticobjects_u8() const {
  return _internal_has_numberofstaticobjects_u8();
}
inline void UsEnvModelPort::clear_numberofstaticobjects_u8() {
  numberofstaticobjects_u8_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEnvModelPort::_internal_numberofstaticobjects_u8() const {
  return numberofstaticobjects_u8_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEnvModelPort::numberofstaticobjects_u8() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_env_model_port.UsEnvModelPort.numberOfStaticObjects_u8)
  return _internal_numberofstaticobjects_u8();
}
inline void UsEnvModelPort::_internal_set_numberofstaticobjects_u8(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  numberofstaticobjects_u8_ = value;
}
inline void UsEnvModelPort::set_numberofstaticobjects_u8(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_numberofstaticobjects_u8(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_env_model_port.UsEnvModelPort.numberOfStaticObjects_u8)
}

// optional uint32 numberOfDynamicObjects_u8 = 3825;
inline bool UsEnvModelPort::_internal_has_numberofdynamicobjects_u8() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool UsEnvModelPort::has_numberofdynamicobjects_u8() const {
  return _internal_has_numberofdynamicobjects_u8();
}
inline void UsEnvModelPort::clear_numberofdynamicobjects_u8() {
  numberofdynamicobjects_u8_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEnvModelPort::_internal_numberofdynamicobjects_u8() const {
  return numberofdynamicobjects_u8_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEnvModelPort::numberofdynamicobjects_u8() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_env_model_port.UsEnvModelPort.numberOfDynamicObjects_u8)
  return _internal_numberofdynamicobjects_u8();
}
inline void UsEnvModelPort::_internal_set_numberofdynamicobjects_u8(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  numberofdynamicobjects_u8_ = value;
}
inline void UsEnvModelPort::set_numberofdynamicobjects_u8(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_numberofdynamicobjects_u8(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_env_model_port.UsEnvModelPort.numberOfDynamicObjects_u8)
}

// optional uint32 firstStatObjOutDetZoneIdx_u8 = 2215;
inline bool UsEnvModelPort::_internal_has_firststatobjoutdetzoneidx_u8() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool UsEnvModelPort::has_firststatobjoutdetzoneidx_u8() const {
  return _internal_has_firststatobjoutdetzoneidx_u8();
}
inline void UsEnvModelPort::clear_firststatobjoutdetzoneidx_u8() {
  firststatobjoutdetzoneidx_u8_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEnvModelPort::_internal_firststatobjoutdetzoneidx_u8() const {
  return firststatobjoutdetzoneidx_u8_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEnvModelPort::firststatobjoutdetzoneidx_u8() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_env_model_port.UsEnvModelPort.firstStatObjOutDetZoneIdx_u8)
  return _internal_firststatobjoutdetzoneidx_u8();
}
inline void UsEnvModelPort::_internal_set_firststatobjoutdetzoneidx_u8(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  firststatobjoutdetzoneidx_u8_ = value;
}
inline void UsEnvModelPort::set_firststatobjoutdetzoneidx_u8(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_firststatobjoutdetzoneidx_u8(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_env_model_port.UsEnvModelPort.firstStatObjOutDetZoneIdx_u8)
}

// optional uint32 firstDynObjOutDetZoneIdx_u8 = 1726;
inline bool UsEnvModelPort::_internal_has_firstdynobjoutdetzoneidx_u8() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool UsEnvModelPort::has_firstdynobjoutdetzoneidx_u8() const {
  return _internal_has_firstdynobjoutdetzoneidx_u8();
}
inline void UsEnvModelPort::clear_firstdynobjoutdetzoneidx_u8() {
  firstdynobjoutdetzoneidx_u8_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEnvModelPort::_internal_firstdynobjoutdetzoneidx_u8() const {
  return firstdynobjoutdetzoneidx_u8_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsEnvModelPort::firstdynobjoutdetzoneidx_u8() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_env_model_port.UsEnvModelPort.firstDynObjOutDetZoneIdx_u8)
  return _internal_firstdynobjoutdetzoneidx_u8();
}
inline void UsEnvModelPort::_internal_set_firstdynobjoutdetzoneidx_u8(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  firstdynobjoutdetzoneidx_u8_ = value;
}
inline void UsEnvModelPort::set_firstdynobjoutdetzoneidx_u8(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_firstdynobjoutdetzoneidx_u8(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_env_model_port.UsEnvModelPort.firstDynObjOutDetZoneIdx_u8)
}

// repeated .pb.us_em.dynamic_object_serializable.DynamicObjectSerializable dynamicObjects = 482;
inline int UsEnvModelPort::_internal_dynamicobjects_size() const {
  return dynamicobjects_.size();
}
inline int UsEnvModelPort::dynamicobjects_size() const {
  return _internal_dynamicobjects_size();
}
inline ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable* UsEnvModelPort::mutable_dynamicobjects(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_env_model_port.UsEnvModelPort.dynamicObjects)
  return dynamicobjects_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable >*
UsEnvModelPort::mutable_dynamicobjects() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_em.us_env_model_port.UsEnvModelPort.dynamicObjects)
  return &dynamicobjects_;
}
inline const ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable& UsEnvModelPort::_internal_dynamicobjects(int index) const {
  return dynamicobjects_.Get(index);
}
inline const ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable& UsEnvModelPort::dynamicobjects(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_env_model_port.UsEnvModelPort.dynamicObjects)
  return _internal_dynamicobjects(index);
}
inline ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable* UsEnvModelPort::_internal_add_dynamicobjects() {
  return dynamicobjects_.Add();
}
inline ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable* UsEnvModelPort::add_dynamicobjects() {
  // @@protoc_insertion_point(field_add:pb.us_em.us_env_model_port.UsEnvModelPort.dynamicObjects)
  return _internal_add_dynamicobjects();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::dynamic_object_serializable::DynamicObjectSerializable >&
UsEnvModelPort::dynamicobjects() const {
  // @@protoc_insertion_point(field_list:pb.us_em.us_env_model_port.UsEnvModelPort.dynamicObjects)
  return dynamicobjects_;
}

// repeated .pb.us_em.static_object_serializable.StaticObjectSerializable staticObjects = 3098;
inline int UsEnvModelPort::_internal_staticobjects_size() const {
  return staticobjects_.size();
}
inline int UsEnvModelPort::staticobjects_size() const {
  return _internal_staticobjects_size();
}
inline ::pb::us_em::static_object_serializable::StaticObjectSerializable* UsEnvModelPort::mutable_staticobjects(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_env_model_port.UsEnvModelPort.staticObjects)
  return staticobjects_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::static_object_serializable::StaticObjectSerializable >*
UsEnvModelPort::mutable_staticobjects() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_em.us_env_model_port.UsEnvModelPort.staticObjects)
  return &staticobjects_;
}
inline const ::pb::us_em::static_object_serializable::StaticObjectSerializable& UsEnvModelPort::_internal_staticobjects(int index) const {
  return staticobjects_.Get(index);
}
inline const ::pb::us_em::static_object_serializable::StaticObjectSerializable& UsEnvModelPort::staticobjects(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_env_model_port.UsEnvModelPort.staticObjects)
  return _internal_staticobjects(index);
}
inline ::pb::us_em::static_object_serializable::StaticObjectSerializable* UsEnvModelPort::_internal_add_staticobjects() {
  return staticobjects_.Add();
}
inline ::pb::us_em::static_object_serializable::StaticObjectSerializable* UsEnvModelPort::add_staticobjects() {
  // @@protoc_insertion_point(field_add:pb.us_em.us_env_model_port.UsEnvModelPort.staticObjects)
  return _internal_add_staticobjects();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::static_object_serializable::StaticObjectSerializable >&
UsEnvModelPort::staticobjects() const {
  // @@protoc_insertion_point(field_list:pb.us_em.us_env_model_port.UsEnvModelPort.staticObjects)
  return staticobjects_;
}

// -------------------------------------------------------------------

// UsEnvModelPort_array_port

// repeated .pb.us_em.us_env_model_port.UsEnvModelPort data = 3989;
inline int UsEnvModelPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsEnvModelPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsEnvModelPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_em::us_env_model_port::UsEnvModelPort* UsEnvModelPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_env_model_port.UsEnvModelPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_env_model_port::UsEnvModelPort >*
UsEnvModelPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_em.us_env_model_port.UsEnvModelPort_array_port.data)
  return &data_;
}
inline const ::pb::us_em::us_env_model_port::UsEnvModelPort& UsEnvModelPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_em::us_env_model_port::UsEnvModelPort& UsEnvModelPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_env_model_port.UsEnvModelPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_em::us_env_model_port::UsEnvModelPort* UsEnvModelPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_em::us_env_model_port::UsEnvModelPort* UsEnvModelPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_em.us_env_model_port.UsEnvModelPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_env_model_port::UsEnvModelPort >&
UsEnvModelPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_em.us_env_model_port.UsEnvModelPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_env_model_port
}  // namespace us_em
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fem_2fus_5fenv_5fmodel_5fport_2eproto
