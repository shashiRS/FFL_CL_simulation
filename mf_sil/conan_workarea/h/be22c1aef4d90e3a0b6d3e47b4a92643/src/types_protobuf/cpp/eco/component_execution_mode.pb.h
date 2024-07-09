// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: eco/component_execution_mode.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_eco_2fcomponent_5fexecution_5fmode_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_eco_2fcomponent_5fexecution_5fmode_2eproto

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
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_eco_2fcomponent_5fexecution_5fmode_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_eco_2fcomponent_5fexecution_5fmode_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_eco_2fcomponent_5fexecution_5fmode_2eproto;
namespace pb {
namespace eco {
namespace component_execution_mode {
class ComponentExecutionMode;
class ComponentExecutionModeDefaultTypeInternal;
extern ComponentExecutionModeDefaultTypeInternal _ComponentExecutionMode_default_instance_;
class ComponentExecutionMode_array_port;
class ComponentExecutionMode_array_portDefaultTypeInternal;
extern ComponentExecutionMode_array_portDefaultTypeInternal _ComponentExecutionMode_array_port_default_instance_;
}  // namespace component_execution_mode
}  // namespace eco
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::eco::component_execution_mode::ComponentExecutionMode* Arena::CreateMaybeMessage<::pb::eco::component_execution_mode::ComponentExecutionMode>(Arena*);
template<> ::pb::eco::component_execution_mode::ComponentExecutionMode_array_port* Arena::CreateMaybeMessage<::pb::eco::component_execution_mode::ComponentExecutionMode_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace eco {
namespace component_execution_mode {

// ===================================================================

class ComponentExecutionMode :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.eco.component_execution_mode.ComponentExecutionMode) */ {
 public:
  ComponentExecutionMode();
  virtual ~ComponentExecutionMode();

  ComponentExecutionMode(const ComponentExecutionMode& from);
  ComponentExecutionMode(ComponentExecutionMode&& from) noexcept
    : ComponentExecutionMode() {
    *this = ::std::move(from);
  }

  inline ComponentExecutionMode& operator=(const ComponentExecutionMode& from) {
    CopyFrom(from);
    return *this;
  }
  inline ComponentExecutionMode& operator=(ComponentExecutionMode&& from) noexcept {
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
  static const ComponentExecutionMode& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ComponentExecutionMode* internal_default_instance() {
    return reinterpret_cast<const ComponentExecutionMode*>(
               &_ComponentExecutionMode_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ComponentExecutionMode& a, ComponentExecutionMode& b) {
    a.Swap(&b);
  }
  inline void Swap(ComponentExecutionMode* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ComponentExecutionMode* New() const final {
    return CreateMaybeMessage<ComponentExecutionMode>(nullptr);
  }

  ComponentExecutionMode* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ComponentExecutionMode>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ComponentExecutionMode& from);
  void MergeFrom(const ComponentExecutionMode& from);
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
  void InternalSwap(ComponentExecutionMode* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.eco.component_execution_mode.ComponentExecutionMode";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_eco_2fcomponent_5fexecution_5fmode_2eproto);
    return ::descriptor_table_eco_2fcomponent_5fexecution_5fmode_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSubConfigurationIDFieldNumber = 116,
    kConfigurationIDFieldNumber = 1030,
  };
  // optional uint32 subConfigurationID = 116;
  bool has_subconfigurationid() const;
  private:
  bool _internal_has_subconfigurationid() const;
  public:
  void clear_subconfigurationid();
  ::PROTOBUF_NAMESPACE_ID::uint32 subconfigurationid() const;
  void set_subconfigurationid(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_subconfigurationid() const;
  void _internal_set_subconfigurationid(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 configurationID = 1030;
  bool has_configurationid() const;
  private:
  bool _internal_has_configurationid() const;
  public:
  void clear_configurationid();
  ::PROTOBUF_NAMESPACE_ID::uint32 configurationid() const;
  void set_configurationid(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_configurationid() const;
  void _internal_set_configurationid(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.eco.component_execution_mode.ComponentExecutionMode)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 subconfigurationid_;
  ::PROTOBUF_NAMESPACE_ID::uint32 configurationid_;
  friend struct ::TableStruct_eco_2fcomponent_5fexecution_5fmode_2eproto;
};
// -------------------------------------------------------------------

class ComponentExecutionMode_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.eco.component_execution_mode.ComponentExecutionMode_array_port) */ {
 public:
  ComponentExecutionMode_array_port();
  virtual ~ComponentExecutionMode_array_port();

  ComponentExecutionMode_array_port(const ComponentExecutionMode_array_port& from);
  ComponentExecutionMode_array_port(ComponentExecutionMode_array_port&& from) noexcept
    : ComponentExecutionMode_array_port() {
    *this = ::std::move(from);
  }

  inline ComponentExecutionMode_array_port& operator=(const ComponentExecutionMode_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline ComponentExecutionMode_array_port& operator=(ComponentExecutionMode_array_port&& from) noexcept {
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
  static const ComponentExecutionMode_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ComponentExecutionMode_array_port* internal_default_instance() {
    return reinterpret_cast<const ComponentExecutionMode_array_port*>(
               &_ComponentExecutionMode_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(ComponentExecutionMode_array_port& a, ComponentExecutionMode_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(ComponentExecutionMode_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ComponentExecutionMode_array_port* New() const final {
    return CreateMaybeMessage<ComponentExecutionMode_array_port>(nullptr);
  }

  ComponentExecutionMode_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ComponentExecutionMode_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ComponentExecutionMode_array_port& from);
  void MergeFrom(const ComponentExecutionMode_array_port& from);
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
  void InternalSwap(ComponentExecutionMode_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.eco.component_execution_mode.ComponentExecutionMode_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_eco_2fcomponent_5fexecution_5fmode_2eproto);
    return ::descriptor_table_eco_2fcomponent_5fexecution_5fmode_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3335,
  };
  // repeated .pb.eco.component_execution_mode.ComponentExecutionMode data = 3335;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::eco::component_execution_mode::ComponentExecutionMode* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::component_execution_mode::ComponentExecutionMode >*
      mutable_data();
  private:
  const ::pb::eco::component_execution_mode::ComponentExecutionMode& _internal_data(int index) const;
  ::pb::eco::component_execution_mode::ComponentExecutionMode* _internal_add_data();
  public:
  const ::pb::eco::component_execution_mode::ComponentExecutionMode& data(int index) const;
  ::pb::eco::component_execution_mode::ComponentExecutionMode* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::component_execution_mode::ComponentExecutionMode >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.eco.component_execution_mode.ComponentExecutionMode_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::component_execution_mode::ComponentExecutionMode > data_;
  friend struct ::TableStruct_eco_2fcomponent_5fexecution_5fmode_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ComponentExecutionMode

// optional uint32 configurationID = 1030;
inline bool ComponentExecutionMode::_internal_has_configurationid() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool ComponentExecutionMode::has_configurationid() const {
  return _internal_has_configurationid();
}
inline void ComponentExecutionMode::clear_configurationid() {
  configurationid_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ComponentExecutionMode::_internal_configurationid() const {
  return configurationid_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ComponentExecutionMode::configurationid() const {
  // @@protoc_insertion_point(field_get:pb.eco.component_execution_mode.ComponentExecutionMode.configurationID)
  return _internal_configurationid();
}
inline void ComponentExecutionMode::_internal_set_configurationid(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  configurationid_ = value;
}
inline void ComponentExecutionMode::set_configurationid(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_configurationid(value);
  // @@protoc_insertion_point(field_set:pb.eco.component_execution_mode.ComponentExecutionMode.configurationID)
}

// optional uint32 subConfigurationID = 116;
inline bool ComponentExecutionMode::_internal_has_subconfigurationid() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool ComponentExecutionMode::has_subconfigurationid() const {
  return _internal_has_subconfigurationid();
}
inline void ComponentExecutionMode::clear_subconfigurationid() {
  subconfigurationid_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ComponentExecutionMode::_internal_subconfigurationid() const {
  return subconfigurationid_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 ComponentExecutionMode::subconfigurationid() const {
  // @@protoc_insertion_point(field_get:pb.eco.component_execution_mode.ComponentExecutionMode.subConfigurationID)
  return _internal_subconfigurationid();
}
inline void ComponentExecutionMode::_internal_set_subconfigurationid(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  subconfigurationid_ = value;
}
inline void ComponentExecutionMode::set_subconfigurationid(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_subconfigurationid(value);
  // @@protoc_insertion_point(field_set:pb.eco.component_execution_mode.ComponentExecutionMode.subConfigurationID)
}

// -------------------------------------------------------------------

// ComponentExecutionMode_array_port

// repeated .pb.eco.component_execution_mode.ComponentExecutionMode data = 3335;
inline int ComponentExecutionMode_array_port::_internal_data_size() const {
  return data_.size();
}
inline int ComponentExecutionMode_array_port::data_size() const {
  return _internal_data_size();
}
inline void ComponentExecutionMode_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::eco::component_execution_mode::ComponentExecutionMode* ComponentExecutionMode_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.eco.component_execution_mode.ComponentExecutionMode_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::component_execution_mode::ComponentExecutionMode >*
ComponentExecutionMode_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.eco.component_execution_mode.ComponentExecutionMode_array_port.data)
  return &data_;
}
inline const ::pb::eco::component_execution_mode::ComponentExecutionMode& ComponentExecutionMode_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::eco::component_execution_mode::ComponentExecutionMode& ComponentExecutionMode_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.eco.component_execution_mode.ComponentExecutionMode_array_port.data)
  return _internal_data(index);
}
inline ::pb::eco::component_execution_mode::ComponentExecutionMode* ComponentExecutionMode_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::eco::component_execution_mode::ComponentExecutionMode* ComponentExecutionMode_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.eco.component_execution_mode.ComponentExecutionMode_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::eco::component_execution_mode::ComponentExecutionMode >&
ComponentExecutionMode_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.eco.component_execution_mode.ComponentExecutionMode_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace component_execution_mode
}  // namespace eco
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_eco_2fcomponent_5fexecution_5fmode_2eproto