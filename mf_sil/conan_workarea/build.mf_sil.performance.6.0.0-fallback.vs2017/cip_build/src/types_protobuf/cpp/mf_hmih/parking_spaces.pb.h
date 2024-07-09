// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_hmih/parking_spaces.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fparking_5fspaces_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fparking_5fspaces_2eproto

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
#include "mf_hmih/parking_space.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fhmih_2fparking_5fspaces_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fhmih_2fparking_5fspaces_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fhmih_2fparking_5fspaces_2eproto;
namespace pb {
namespace mf_hmih {
namespace parking_spaces {
class ParkingSpaces;
class ParkingSpacesDefaultTypeInternal;
extern ParkingSpacesDefaultTypeInternal _ParkingSpaces_default_instance_;
class ParkingSpaces_array_port;
class ParkingSpaces_array_portDefaultTypeInternal;
extern ParkingSpaces_array_portDefaultTypeInternal _ParkingSpaces_array_port_default_instance_;
}  // namespace parking_spaces
}  // namespace mf_hmih
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_hmih::parking_spaces::ParkingSpaces* Arena::CreateMaybeMessage<::pb::mf_hmih::parking_spaces::ParkingSpaces>(Arena*);
template<> ::pb::mf_hmih::parking_spaces::ParkingSpaces_array_port* Arena::CreateMaybeMessage<::pb::mf_hmih::parking_spaces::ParkingSpaces_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_hmih {
namespace parking_spaces {

// ===================================================================

class ParkingSpaces :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.parking_spaces.ParkingSpaces) */ {
 public:
  ParkingSpaces();
  virtual ~ParkingSpaces();

  ParkingSpaces(const ParkingSpaces& from);
  ParkingSpaces(ParkingSpaces&& from) noexcept
    : ParkingSpaces() {
    *this = ::std::move(from);
  }

  inline ParkingSpaces& operator=(const ParkingSpaces& from) {
    CopyFrom(from);
    return *this;
  }
  inline ParkingSpaces& operator=(ParkingSpaces&& from) noexcept {
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
  static const ParkingSpaces& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ParkingSpaces* internal_default_instance() {
    return reinterpret_cast<const ParkingSpaces*>(
               &_ParkingSpaces_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ParkingSpaces& a, ParkingSpaces& b) {
    a.Swap(&b);
  }
  inline void Swap(ParkingSpaces* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ParkingSpaces* New() const final {
    return CreateMaybeMessage<ParkingSpaces>(nullptr);
  }

  ParkingSpaces* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ParkingSpaces>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ParkingSpaces& from);
  void MergeFrom(const ParkingSpaces& from);
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
  void InternalSwap(ParkingSpaces* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.parking_spaces.ParkingSpaces";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fparking_5fspaces_2eproto);
    return ::descriptor_table_mf_5fhmih_2fparking_5fspaces_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kRightFieldNumber = 53,
    kLeftFieldNumber = 1090,
    kRearFieldNumber = 1262,
    kFrontFieldNumber = 3030,
  };
  // optional .pb.mf_hmih.parking_space.ParkingSpace right = 53;
  bool has_right() const;
  private:
  bool _internal_has_right() const;
  public:
  void clear_right();
  const ::pb::mf_hmih::parking_space::ParkingSpace& right() const;
  ::pb::mf_hmih::parking_space::ParkingSpace* release_right();
  ::pb::mf_hmih::parking_space::ParkingSpace* mutable_right();
  void set_allocated_right(::pb::mf_hmih::parking_space::ParkingSpace* right);
  private:
  const ::pb::mf_hmih::parking_space::ParkingSpace& _internal_right() const;
  ::pb::mf_hmih::parking_space::ParkingSpace* _internal_mutable_right();
  public:

  // optional .pb.mf_hmih.parking_space.ParkingSpace left = 1090;
  bool has_left() const;
  private:
  bool _internal_has_left() const;
  public:
  void clear_left();
  const ::pb::mf_hmih::parking_space::ParkingSpace& left() const;
  ::pb::mf_hmih::parking_space::ParkingSpace* release_left();
  ::pb::mf_hmih::parking_space::ParkingSpace* mutable_left();
  void set_allocated_left(::pb::mf_hmih::parking_space::ParkingSpace* left);
  private:
  const ::pb::mf_hmih::parking_space::ParkingSpace& _internal_left() const;
  ::pb::mf_hmih::parking_space::ParkingSpace* _internal_mutable_left();
  public:

  // optional .pb.mf_hmih.parking_space.ParkingSpace rear = 1262;
  bool has_rear() const;
  private:
  bool _internal_has_rear() const;
  public:
  void clear_rear();
  const ::pb::mf_hmih::parking_space::ParkingSpace& rear() const;
  ::pb::mf_hmih::parking_space::ParkingSpace* release_rear();
  ::pb::mf_hmih::parking_space::ParkingSpace* mutable_rear();
  void set_allocated_rear(::pb::mf_hmih::parking_space::ParkingSpace* rear);
  private:
  const ::pb::mf_hmih::parking_space::ParkingSpace& _internal_rear() const;
  ::pb::mf_hmih::parking_space::ParkingSpace* _internal_mutable_rear();
  public:

  // optional .pb.mf_hmih.parking_space.ParkingSpace front = 3030;
  bool has_front() const;
  private:
  bool _internal_has_front() const;
  public:
  void clear_front();
  const ::pb::mf_hmih::parking_space::ParkingSpace& front() const;
  ::pb::mf_hmih::parking_space::ParkingSpace* release_front();
  ::pb::mf_hmih::parking_space::ParkingSpace* mutable_front();
  void set_allocated_front(::pb::mf_hmih::parking_space::ParkingSpace* front);
  private:
  const ::pb::mf_hmih::parking_space::ParkingSpace& _internal_front() const;
  ::pb::mf_hmih::parking_space::ParkingSpace* _internal_mutable_front();
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.parking_spaces.ParkingSpaces)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::mf_hmih::parking_space::ParkingSpace* right_;
  ::pb::mf_hmih::parking_space::ParkingSpace* left_;
  ::pb::mf_hmih::parking_space::ParkingSpace* rear_;
  ::pb::mf_hmih::parking_space::ParkingSpace* front_;
  friend struct ::TableStruct_mf_5fhmih_2fparking_5fspaces_2eproto;
};
// -------------------------------------------------------------------

class ParkingSpaces_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.parking_spaces.ParkingSpaces_array_port) */ {
 public:
  ParkingSpaces_array_port();
  virtual ~ParkingSpaces_array_port();

  ParkingSpaces_array_port(const ParkingSpaces_array_port& from);
  ParkingSpaces_array_port(ParkingSpaces_array_port&& from) noexcept
    : ParkingSpaces_array_port() {
    *this = ::std::move(from);
  }

  inline ParkingSpaces_array_port& operator=(const ParkingSpaces_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline ParkingSpaces_array_port& operator=(ParkingSpaces_array_port&& from) noexcept {
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
  static const ParkingSpaces_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ParkingSpaces_array_port* internal_default_instance() {
    return reinterpret_cast<const ParkingSpaces_array_port*>(
               &_ParkingSpaces_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(ParkingSpaces_array_port& a, ParkingSpaces_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(ParkingSpaces_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ParkingSpaces_array_port* New() const final {
    return CreateMaybeMessage<ParkingSpaces_array_port>(nullptr);
  }

  ParkingSpaces_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ParkingSpaces_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ParkingSpaces_array_port& from);
  void MergeFrom(const ParkingSpaces_array_port& from);
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
  void InternalSwap(ParkingSpaces_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.parking_spaces.ParkingSpaces_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fparking_5fspaces_2eproto);
    return ::descriptor_table_mf_5fhmih_2fparking_5fspaces_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 250,
  };
  // repeated .pb.mf_hmih.parking_spaces.ParkingSpaces data = 250;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_hmih::parking_spaces::ParkingSpaces* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::parking_spaces::ParkingSpaces >*
      mutable_data();
  private:
  const ::pb::mf_hmih::parking_spaces::ParkingSpaces& _internal_data(int index) const;
  ::pb::mf_hmih::parking_spaces::ParkingSpaces* _internal_add_data();
  public:
  const ::pb::mf_hmih::parking_spaces::ParkingSpaces& data(int index) const;
  ::pb::mf_hmih::parking_spaces::ParkingSpaces* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::parking_spaces::ParkingSpaces >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.parking_spaces.ParkingSpaces_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::parking_spaces::ParkingSpaces > data_;
  friend struct ::TableStruct_mf_5fhmih_2fparking_5fspaces_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ParkingSpaces

// optional .pb.mf_hmih.parking_space.ParkingSpace left = 1090;
inline bool ParkingSpaces::_internal_has_left() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || left_ != nullptr);
  return value;
}
inline bool ParkingSpaces::has_left() const {
  return _internal_has_left();
}
inline const ::pb::mf_hmih::parking_space::ParkingSpace& ParkingSpaces::_internal_left() const {
  const ::pb::mf_hmih::parking_space::ParkingSpace* p = left_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::mf_hmih::parking_space::ParkingSpace*>(
      &::pb::mf_hmih::parking_space::_ParkingSpace_default_instance_);
}
inline const ::pb::mf_hmih::parking_space::ParkingSpace& ParkingSpaces::left() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.parking_spaces.ParkingSpaces.left)
  return _internal_left();
}
inline ::pb::mf_hmih::parking_space::ParkingSpace* ParkingSpaces::release_left() {
  // @@protoc_insertion_point(field_release:pb.mf_hmih.parking_spaces.ParkingSpaces.left)
  _has_bits_[0] &= ~0x00000002u;
  ::pb::mf_hmih::parking_space::ParkingSpace* temp = left_;
  left_ = nullptr;
  return temp;
}
inline ::pb::mf_hmih::parking_space::ParkingSpace* ParkingSpaces::_internal_mutable_left() {
  _has_bits_[0] |= 0x00000002u;
  if (left_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::mf_hmih::parking_space::ParkingSpace>(GetArenaNoVirtual());
    left_ = p;
  }
  return left_;
}
inline ::pb::mf_hmih::parking_space::ParkingSpace* ParkingSpaces::mutable_left() {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.parking_spaces.ParkingSpaces.left)
  return _internal_mutable_left();
}
inline void ParkingSpaces::set_allocated_left(::pb::mf_hmih::parking_space::ParkingSpace* left) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(left_);
  }
  if (left) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      left = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, left, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  left_ = left;
  // @@protoc_insertion_point(field_set_allocated:pb.mf_hmih.parking_spaces.ParkingSpaces.left)
}

// optional .pb.mf_hmih.parking_space.ParkingSpace right = 53;
inline bool ParkingSpaces::_internal_has_right() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || right_ != nullptr);
  return value;
}
inline bool ParkingSpaces::has_right() const {
  return _internal_has_right();
}
inline const ::pb::mf_hmih::parking_space::ParkingSpace& ParkingSpaces::_internal_right() const {
  const ::pb::mf_hmih::parking_space::ParkingSpace* p = right_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::mf_hmih::parking_space::ParkingSpace*>(
      &::pb::mf_hmih::parking_space::_ParkingSpace_default_instance_);
}
inline const ::pb::mf_hmih::parking_space::ParkingSpace& ParkingSpaces::right() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.parking_spaces.ParkingSpaces.right)
  return _internal_right();
}
inline ::pb::mf_hmih::parking_space::ParkingSpace* ParkingSpaces::release_right() {
  // @@protoc_insertion_point(field_release:pb.mf_hmih.parking_spaces.ParkingSpaces.right)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::mf_hmih::parking_space::ParkingSpace* temp = right_;
  right_ = nullptr;
  return temp;
}
inline ::pb::mf_hmih::parking_space::ParkingSpace* ParkingSpaces::_internal_mutable_right() {
  _has_bits_[0] |= 0x00000001u;
  if (right_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::mf_hmih::parking_space::ParkingSpace>(GetArenaNoVirtual());
    right_ = p;
  }
  return right_;
}
inline ::pb::mf_hmih::parking_space::ParkingSpace* ParkingSpaces::mutable_right() {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.parking_spaces.ParkingSpaces.right)
  return _internal_mutable_right();
}
inline void ParkingSpaces::set_allocated_right(::pb::mf_hmih::parking_space::ParkingSpace* right) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(right_);
  }
  if (right) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      right = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, right, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  right_ = right;
  // @@protoc_insertion_point(field_set_allocated:pb.mf_hmih.parking_spaces.ParkingSpaces.right)
}

// optional .pb.mf_hmih.parking_space.ParkingSpace front = 3030;
inline bool ParkingSpaces::_internal_has_front() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  PROTOBUF_ASSUME(!value || front_ != nullptr);
  return value;
}
inline bool ParkingSpaces::has_front() const {
  return _internal_has_front();
}
inline const ::pb::mf_hmih::parking_space::ParkingSpace& ParkingSpaces::_internal_front() const {
  const ::pb::mf_hmih::parking_space::ParkingSpace* p = front_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::mf_hmih::parking_space::ParkingSpace*>(
      &::pb::mf_hmih::parking_space::_ParkingSpace_default_instance_);
}
inline const ::pb::mf_hmih::parking_space::ParkingSpace& ParkingSpaces::front() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.parking_spaces.ParkingSpaces.front)
  return _internal_front();
}
inline ::pb::mf_hmih::parking_space::ParkingSpace* ParkingSpaces::release_front() {
  // @@protoc_insertion_point(field_release:pb.mf_hmih.parking_spaces.ParkingSpaces.front)
  _has_bits_[0] &= ~0x00000008u;
  ::pb::mf_hmih::parking_space::ParkingSpace* temp = front_;
  front_ = nullptr;
  return temp;
}
inline ::pb::mf_hmih::parking_space::ParkingSpace* ParkingSpaces::_internal_mutable_front() {
  _has_bits_[0] |= 0x00000008u;
  if (front_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::mf_hmih::parking_space::ParkingSpace>(GetArenaNoVirtual());
    front_ = p;
  }
  return front_;
}
inline ::pb::mf_hmih::parking_space::ParkingSpace* ParkingSpaces::mutable_front() {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.parking_spaces.ParkingSpaces.front)
  return _internal_mutable_front();
}
inline void ParkingSpaces::set_allocated_front(::pb::mf_hmih::parking_space::ParkingSpace* front) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(front_);
  }
  if (front) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      front = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, front, submessage_arena);
    }
    _has_bits_[0] |= 0x00000008u;
  } else {
    _has_bits_[0] &= ~0x00000008u;
  }
  front_ = front;
  // @@protoc_insertion_point(field_set_allocated:pb.mf_hmih.parking_spaces.ParkingSpaces.front)
}

// optional .pb.mf_hmih.parking_space.ParkingSpace rear = 1262;
inline bool ParkingSpaces::_internal_has_rear() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  PROTOBUF_ASSUME(!value || rear_ != nullptr);
  return value;
}
inline bool ParkingSpaces::has_rear() const {
  return _internal_has_rear();
}
inline const ::pb::mf_hmih::parking_space::ParkingSpace& ParkingSpaces::_internal_rear() const {
  const ::pb::mf_hmih::parking_space::ParkingSpace* p = rear_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::mf_hmih::parking_space::ParkingSpace*>(
      &::pb::mf_hmih::parking_space::_ParkingSpace_default_instance_);
}
inline const ::pb::mf_hmih::parking_space::ParkingSpace& ParkingSpaces::rear() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.parking_spaces.ParkingSpaces.rear)
  return _internal_rear();
}
inline ::pb::mf_hmih::parking_space::ParkingSpace* ParkingSpaces::release_rear() {
  // @@protoc_insertion_point(field_release:pb.mf_hmih.parking_spaces.ParkingSpaces.rear)
  _has_bits_[0] &= ~0x00000004u;
  ::pb::mf_hmih::parking_space::ParkingSpace* temp = rear_;
  rear_ = nullptr;
  return temp;
}
inline ::pb::mf_hmih::parking_space::ParkingSpace* ParkingSpaces::_internal_mutable_rear() {
  _has_bits_[0] |= 0x00000004u;
  if (rear_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::mf_hmih::parking_space::ParkingSpace>(GetArenaNoVirtual());
    rear_ = p;
  }
  return rear_;
}
inline ::pb::mf_hmih::parking_space::ParkingSpace* ParkingSpaces::mutable_rear() {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.parking_spaces.ParkingSpaces.rear)
  return _internal_mutable_rear();
}
inline void ParkingSpaces::set_allocated_rear(::pb::mf_hmih::parking_space::ParkingSpace* rear) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(rear_);
  }
  if (rear) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      rear = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, rear, submessage_arena);
    }
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  rear_ = rear;
  // @@protoc_insertion_point(field_set_allocated:pb.mf_hmih.parking_spaces.ParkingSpaces.rear)
}

// -------------------------------------------------------------------

// ParkingSpaces_array_port

// repeated .pb.mf_hmih.parking_spaces.ParkingSpaces data = 250;
inline int ParkingSpaces_array_port::_internal_data_size() const {
  return data_.size();
}
inline int ParkingSpaces_array_port::data_size() const {
  return _internal_data_size();
}
inline void ParkingSpaces_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_hmih::parking_spaces::ParkingSpaces* ParkingSpaces_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.parking_spaces.ParkingSpaces_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::parking_spaces::ParkingSpaces >*
ParkingSpaces_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.parking_spaces.ParkingSpaces_array_port.data)
  return &data_;
}
inline const ::pb::mf_hmih::parking_spaces::ParkingSpaces& ParkingSpaces_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_hmih::parking_spaces::ParkingSpaces& ParkingSpaces_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.parking_spaces.ParkingSpaces_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_hmih::parking_spaces::ParkingSpaces* ParkingSpaces_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_hmih::parking_spaces::ParkingSpaces* ParkingSpaces_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_hmih.parking_spaces.ParkingSpaces_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::parking_spaces::ParkingSpaces >&
ParkingSpaces_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.parking_spaces.ParkingSpaces_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace parking_spaces
}  // namespace mf_hmih
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fparking_5fspaces_2eproto
