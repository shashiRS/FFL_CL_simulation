// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_hmih/head_unit_interaction.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fhead_5funit_5finteraction_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fhead_5funit_5finteraction_2eproto

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
#include "mf_hmih/apuser_action_head_unit.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fhmih_2fhead_5funit_5finteraction_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fhmih_2fhead_5funit_5finteraction_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fhmih_2fhead_5funit_5finteraction_2eproto;
namespace pb {
namespace mf_hmih {
namespace head_unit_interaction {
class HeadUnitInteraction;
class HeadUnitInteractionDefaultTypeInternal;
extern HeadUnitInteractionDefaultTypeInternal _HeadUnitInteraction_default_instance_;
class HeadUnitInteraction_array_port;
class HeadUnitInteraction_array_portDefaultTypeInternal;
extern HeadUnitInteraction_array_portDefaultTypeInternal _HeadUnitInteraction_array_port_default_instance_;
}  // namespace head_unit_interaction
}  // namespace mf_hmih
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction* Arena::CreateMaybeMessage<::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction>(Arena*);
template<> ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction_array_port* Arena::CreateMaybeMessage<::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_hmih {
namespace head_unit_interaction {

// ===================================================================

class HeadUnitInteraction :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.head_unit_interaction.HeadUnitInteraction) */ {
 public:
  HeadUnitInteraction();
  virtual ~HeadUnitInteraction();

  HeadUnitInteraction(const HeadUnitInteraction& from);
  HeadUnitInteraction(HeadUnitInteraction&& from) noexcept
    : HeadUnitInteraction() {
    *this = ::std::move(from);
  }

  inline HeadUnitInteraction& operator=(const HeadUnitInteraction& from) {
    CopyFrom(from);
    return *this;
  }
  inline HeadUnitInteraction& operator=(HeadUnitInteraction&& from) noexcept {
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
  static const HeadUnitInteraction& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const HeadUnitInteraction* internal_default_instance() {
    return reinterpret_cast<const HeadUnitInteraction*>(
               &_HeadUnitInteraction_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(HeadUnitInteraction& a, HeadUnitInteraction& b) {
    a.Swap(&b);
  }
  inline void Swap(HeadUnitInteraction* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline HeadUnitInteraction* New() const final {
    return CreateMaybeMessage<HeadUnitInteraction>(nullptr);
  }

  HeadUnitInteraction* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<HeadUnitInteraction>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const HeadUnitInteraction& from);
  void MergeFrom(const HeadUnitInteraction& from);
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
  void InternalSwap(HeadUnitInteraction* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.head_unit_interaction.HeadUnitInteraction";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fhead_5funit_5finteraction_2eproto);
    return ::descriptor_table_mf_5fhmih_2fhead_5funit_5finteraction_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kApUserActionHeadUnitNuFieldNumber = 2524,
  };
  // optional .pb.mf_hmih.apuser_action_head_unit.APUserActionHeadUnit apUserActionHeadUnit_nu = 2524;
  bool has_apuseractionheadunit_nu() const;
  private:
  bool _internal_has_apuseractionheadunit_nu() const;
  public:
  void clear_apuseractionheadunit_nu();
  ::pb::mf_hmih::apuser_action_head_unit::APUserActionHeadUnit apuseractionheadunit_nu() const;
  void set_apuseractionheadunit_nu(::pb::mf_hmih::apuser_action_head_unit::APUserActionHeadUnit value);
  private:
  ::pb::mf_hmih::apuser_action_head_unit::APUserActionHeadUnit _internal_apuseractionheadunit_nu() const;
  void _internal_set_apuseractionheadunit_nu(::pb::mf_hmih::apuser_action_head_unit::APUserActionHeadUnit value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.head_unit_interaction.HeadUnitInteraction)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int apuseractionheadunit_nu_;
  friend struct ::TableStruct_mf_5fhmih_2fhead_5funit_5finteraction_2eproto;
};
// -------------------------------------------------------------------

class HeadUnitInteraction_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.head_unit_interaction.HeadUnitInteraction_array_port) */ {
 public:
  HeadUnitInteraction_array_port();
  virtual ~HeadUnitInteraction_array_port();

  HeadUnitInteraction_array_port(const HeadUnitInteraction_array_port& from);
  HeadUnitInteraction_array_port(HeadUnitInteraction_array_port&& from) noexcept
    : HeadUnitInteraction_array_port() {
    *this = ::std::move(from);
  }

  inline HeadUnitInteraction_array_port& operator=(const HeadUnitInteraction_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline HeadUnitInteraction_array_port& operator=(HeadUnitInteraction_array_port&& from) noexcept {
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
  static const HeadUnitInteraction_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const HeadUnitInteraction_array_port* internal_default_instance() {
    return reinterpret_cast<const HeadUnitInteraction_array_port*>(
               &_HeadUnitInteraction_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(HeadUnitInteraction_array_port& a, HeadUnitInteraction_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(HeadUnitInteraction_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline HeadUnitInteraction_array_port* New() const final {
    return CreateMaybeMessage<HeadUnitInteraction_array_port>(nullptr);
  }

  HeadUnitInteraction_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<HeadUnitInteraction_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const HeadUnitInteraction_array_port& from);
  void MergeFrom(const HeadUnitInteraction_array_port& from);
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
  void InternalSwap(HeadUnitInteraction_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.head_unit_interaction.HeadUnitInteraction_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fhead_5funit_5finteraction_2eproto);
    return ::descriptor_table_mf_5fhmih_2fhead_5funit_5finteraction_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2019,
  };
  // repeated .pb.mf_hmih.head_unit_interaction.HeadUnitInteraction data = 2019;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction >*
      mutable_data();
  private:
  const ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction& _internal_data(int index) const;
  ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction* _internal_add_data();
  public:
  const ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction& data(int index) const;
  ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.head_unit_interaction.HeadUnitInteraction_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction > data_;
  friend struct ::TableStruct_mf_5fhmih_2fhead_5funit_5finteraction_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// HeadUnitInteraction

// optional .pb.mf_hmih.apuser_action_head_unit.APUserActionHeadUnit apUserActionHeadUnit_nu = 2524;
inline bool HeadUnitInteraction::_internal_has_apuseractionheadunit_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool HeadUnitInteraction::has_apuseractionheadunit_nu() const {
  return _internal_has_apuseractionheadunit_nu();
}
inline void HeadUnitInteraction::clear_apuseractionheadunit_nu() {
  apuseractionheadunit_nu_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::pb::mf_hmih::apuser_action_head_unit::APUserActionHeadUnit HeadUnitInteraction::_internal_apuseractionheadunit_nu() const {
  return static_cast< ::pb::mf_hmih::apuser_action_head_unit::APUserActionHeadUnit >(apuseractionheadunit_nu_);
}
inline ::pb::mf_hmih::apuser_action_head_unit::APUserActionHeadUnit HeadUnitInteraction::apuseractionheadunit_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.head_unit_interaction.HeadUnitInteraction.apUserActionHeadUnit_nu)
  return _internal_apuseractionheadunit_nu();
}
inline void HeadUnitInteraction::_internal_set_apuseractionheadunit_nu(::pb::mf_hmih::apuser_action_head_unit::APUserActionHeadUnit value) {
  assert(::pb::mf_hmih::apuser_action_head_unit::APUserActionHeadUnit_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  apuseractionheadunit_nu_ = value;
}
inline void HeadUnitInteraction::set_apuseractionheadunit_nu(::pb::mf_hmih::apuser_action_head_unit::APUserActionHeadUnit value) {
  _internal_set_apuseractionheadunit_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.head_unit_interaction.HeadUnitInteraction.apUserActionHeadUnit_nu)
}

// -------------------------------------------------------------------

// HeadUnitInteraction_array_port

// repeated .pb.mf_hmih.head_unit_interaction.HeadUnitInteraction data = 2019;
inline int HeadUnitInteraction_array_port::_internal_data_size() const {
  return data_.size();
}
inline int HeadUnitInteraction_array_port::data_size() const {
  return _internal_data_size();
}
inline void HeadUnitInteraction_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction* HeadUnitInteraction_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.head_unit_interaction.HeadUnitInteraction_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction >*
HeadUnitInteraction_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.head_unit_interaction.HeadUnitInteraction_array_port.data)
  return &data_;
}
inline const ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction& HeadUnitInteraction_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction& HeadUnitInteraction_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.head_unit_interaction.HeadUnitInteraction_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction* HeadUnitInteraction_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction* HeadUnitInteraction_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_hmih.head_unit_interaction.HeadUnitInteraction_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::head_unit_interaction::HeadUnitInteraction >&
HeadUnitInteraction_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.head_unit_interaction.HeadUnitInteraction_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace head_unit_interaction
}  // namespace mf_hmih
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fhead_5funit_5finteraction_2eproto
