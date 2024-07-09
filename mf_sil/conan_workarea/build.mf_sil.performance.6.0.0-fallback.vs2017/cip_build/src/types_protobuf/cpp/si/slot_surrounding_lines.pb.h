// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/slot_surrounding_lines.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_si_2fslot_5fsurrounding_5flines_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_si_2fslot_5fsurrounding_5flines_2eproto

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
#include "si/line_segment_serializable.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_si_2fslot_5fsurrounding_5flines_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_si_2fslot_5fsurrounding_5flines_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fslot_5fsurrounding_5flines_2eproto;
namespace pb {
namespace si {
namespace slot_surrounding_lines {
class SlotSurroundingLines;
class SlotSurroundingLinesDefaultTypeInternal;
extern SlotSurroundingLinesDefaultTypeInternal _SlotSurroundingLines_default_instance_;
class SlotSurroundingLines_array_port;
class SlotSurroundingLines_array_portDefaultTypeInternal;
extern SlotSurroundingLines_array_portDefaultTypeInternal _SlotSurroundingLines_array_port_default_instance_;
}  // namespace slot_surrounding_lines
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::si::slot_surrounding_lines::SlotSurroundingLines* Arena::CreateMaybeMessage<::pb::si::slot_surrounding_lines::SlotSurroundingLines>(Arena*);
template<> ::pb::si::slot_surrounding_lines::SlotSurroundingLines_array_port* Arena::CreateMaybeMessage<::pb::si::slot_surrounding_lines::SlotSurroundingLines_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace si {
namespace slot_surrounding_lines {

// ===================================================================

class SlotSurroundingLines :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.slot_surrounding_lines.SlotSurroundingLines) */ {
 public:
  SlotSurroundingLines();
  virtual ~SlotSurroundingLines();

  SlotSurroundingLines(const SlotSurroundingLines& from);
  SlotSurroundingLines(SlotSurroundingLines&& from) noexcept
    : SlotSurroundingLines() {
    *this = ::std::move(from);
  }

  inline SlotSurroundingLines& operator=(const SlotSurroundingLines& from) {
    CopyFrom(from);
    return *this;
  }
  inline SlotSurroundingLines& operator=(SlotSurroundingLines&& from) noexcept {
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
  static const SlotSurroundingLines& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SlotSurroundingLines* internal_default_instance() {
    return reinterpret_cast<const SlotSurroundingLines*>(
               &_SlotSurroundingLines_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(SlotSurroundingLines& a, SlotSurroundingLines& b) {
    a.Swap(&b);
  }
  inline void Swap(SlotSurroundingLines* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SlotSurroundingLines* New() const final {
    return CreateMaybeMessage<SlotSurroundingLines>(nullptr);
  }

  SlotSurroundingLines* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SlotSurroundingLines>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SlotSurroundingLines& from);
  void MergeFrom(const SlotSurroundingLines& from);
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
  void InternalSwap(SlotSurroundingLines* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.slot_surrounding_lines.SlotSurroundingLines";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fslot_5fsurrounding_5flines_2eproto);
    return ::descriptor_table_si_2fslot_5fsurrounding_5flines_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kLeftLineFieldNumber = 1956,
    kRoadLineFieldNumber = 2900,
    kCurbLineFieldNumber = 3303,
    kRightLineFieldNumber = 3663,
  };
  // optional .pb.si.line_segment_serializable.LineSegmentSerializable leftLine = 1956;
  bool has_leftline() const;
  private:
  bool _internal_has_leftline() const;
  public:
  void clear_leftline();
  const ::pb::si::line_segment_serializable::LineSegmentSerializable& leftline() const;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* release_leftline();
  ::pb::si::line_segment_serializable::LineSegmentSerializable* mutable_leftline();
  void set_allocated_leftline(::pb::si::line_segment_serializable::LineSegmentSerializable* leftline);
  private:
  const ::pb::si::line_segment_serializable::LineSegmentSerializable& _internal_leftline() const;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* _internal_mutable_leftline();
  public:

  // optional .pb.si.line_segment_serializable.LineSegmentSerializable roadLine = 2900;
  bool has_roadline() const;
  private:
  bool _internal_has_roadline() const;
  public:
  void clear_roadline();
  const ::pb::si::line_segment_serializable::LineSegmentSerializable& roadline() const;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* release_roadline();
  ::pb::si::line_segment_serializable::LineSegmentSerializable* mutable_roadline();
  void set_allocated_roadline(::pb::si::line_segment_serializable::LineSegmentSerializable* roadline);
  private:
  const ::pb::si::line_segment_serializable::LineSegmentSerializable& _internal_roadline() const;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* _internal_mutable_roadline();
  public:

  // optional .pb.si.line_segment_serializable.LineSegmentSerializable curbLine = 3303;
  bool has_curbline() const;
  private:
  bool _internal_has_curbline() const;
  public:
  void clear_curbline();
  const ::pb::si::line_segment_serializable::LineSegmentSerializable& curbline() const;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* release_curbline();
  ::pb::si::line_segment_serializable::LineSegmentSerializable* mutable_curbline();
  void set_allocated_curbline(::pb::si::line_segment_serializable::LineSegmentSerializable* curbline);
  private:
  const ::pb::si::line_segment_serializable::LineSegmentSerializable& _internal_curbline() const;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* _internal_mutable_curbline();
  public:

  // optional .pb.si.line_segment_serializable.LineSegmentSerializable rightLine = 3663;
  bool has_rightline() const;
  private:
  bool _internal_has_rightline() const;
  public:
  void clear_rightline();
  const ::pb::si::line_segment_serializable::LineSegmentSerializable& rightline() const;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* release_rightline();
  ::pb::si::line_segment_serializable::LineSegmentSerializable* mutable_rightline();
  void set_allocated_rightline(::pb::si::line_segment_serializable::LineSegmentSerializable* rightline);
  private:
  const ::pb::si::line_segment_serializable::LineSegmentSerializable& _internal_rightline() const;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* _internal_mutable_rightline();
  public:

  // @@protoc_insertion_point(class_scope:pb.si.slot_surrounding_lines.SlotSurroundingLines)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* leftline_;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* roadline_;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* curbline_;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* rightline_;
  friend struct ::TableStruct_si_2fslot_5fsurrounding_5flines_2eproto;
};
// -------------------------------------------------------------------

class SlotSurroundingLines_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.slot_surrounding_lines.SlotSurroundingLines_array_port) */ {
 public:
  SlotSurroundingLines_array_port();
  virtual ~SlotSurroundingLines_array_port();

  SlotSurroundingLines_array_port(const SlotSurroundingLines_array_port& from);
  SlotSurroundingLines_array_port(SlotSurroundingLines_array_port&& from) noexcept
    : SlotSurroundingLines_array_port() {
    *this = ::std::move(from);
  }

  inline SlotSurroundingLines_array_port& operator=(const SlotSurroundingLines_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline SlotSurroundingLines_array_port& operator=(SlotSurroundingLines_array_port&& from) noexcept {
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
  static const SlotSurroundingLines_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SlotSurroundingLines_array_port* internal_default_instance() {
    return reinterpret_cast<const SlotSurroundingLines_array_port*>(
               &_SlotSurroundingLines_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(SlotSurroundingLines_array_port& a, SlotSurroundingLines_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(SlotSurroundingLines_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SlotSurroundingLines_array_port* New() const final {
    return CreateMaybeMessage<SlotSurroundingLines_array_port>(nullptr);
  }

  SlotSurroundingLines_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SlotSurroundingLines_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SlotSurroundingLines_array_port& from);
  void MergeFrom(const SlotSurroundingLines_array_port& from);
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
  void InternalSwap(SlotSurroundingLines_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.slot_surrounding_lines.SlotSurroundingLines_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fslot_5fsurrounding_5flines_2eproto);
    return ::descriptor_table_si_2fslot_5fsurrounding_5flines_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2725,
  };
  // repeated .pb.si.slot_surrounding_lines.SlotSurroundingLines data = 2725;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::si::slot_surrounding_lines::SlotSurroundingLines* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::slot_surrounding_lines::SlotSurroundingLines >*
      mutable_data();
  private:
  const ::pb::si::slot_surrounding_lines::SlotSurroundingLines& _internal_data(int index) const;
  ::pb::si::slot_surrounding_lines::SlotSurroundingLines* _internal_add_data();
  public:
  const ::pb::si::slot_surrounding_lines::SlotSurroundingLines& data(int index) const;
  ::pb::si::slot_surrounding_lines::SlotSurroundingLines* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::slot_surrounding_lines::SlotSurroundingLines >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.si.slot_surrounding_lines.SlotSurroundingLines_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::slot_surrounding_lines::SlotSurroundingLines > data_;
  friend struct ::TableStruct_si_2fslot_5fsurrounding_5flines_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SlotSurroundingLines

// optional .pb.si.line_segment_serializable.LineSegmentSerializable leftLine = 1956;
inline bool SlotSurroundingLines::_internal_has_leftline() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || leftline_ != nullptr);
  return value;
}
inline bool SlotSurroundingLines::has_leftline() const {
  return _internal_has_leftline();
}
inline const ::pb::si::line_segment_serializable::LineSegmentSerializable& SlotSurroundingLines::_internal_leftline() const {
  const ::pb::si::line_segment_serializable::LineSegmentSerializable* p = leftline_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::line_segment_serializable::LineSegmentSerializable*>(
      &::pb::si::line_segment_serializable::_LineSegmentSerializable_default_instance_);
}
inline const ::pb::si::line_segment_serializable::LineSegmentSerializable& SlotSurroundingLines::leftline() const {
  // @@protoc_insertion_point(field_get:pb.si.slot_surrounding_lines.SlotSurroundingLines.leftLine)
  return _internal_leftline();
}
inline ::pb::si::line_segment_serializable::LineSegmentSerializable* SlotSurroundingLines::release_leftline() {
  // @@protoc_insertion_point(field_release:pb.si.slot_surrounding_lines.SlotSurroundingLines.leftLine)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* temp = leftline_;
  leftline_ = nullptr;
  return temp;
}
inline ::pb::si::line_segment_serializable::LineSegmentSerializable* SlotSurroundingLines::_internal_mutable_leftline() {
  _has_bits_[0] |= 0x00000001u;
  if (leftline_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::line_segment_serializable::LineSegmentSerializable>(GetArenaNoVirtual());
    leftline_ = p;
  }
  return leftline_;
}
inline ::pb::si::line_segment_serializable::LineSegmentSerializable* SlotSurroundingLines::mutable_leftline() {
  // @@protoc_insertion_point(field_mutable:pb.si.slot_surrounding_lines.SlotSurroundingLines.leftLine)
  return _internal_mutable_leftline();
}
inline void SlotSurroundingLines::set_allocated_leftline(::pb::si::line_segment_serializable::LineSegmentSerializable* leftline) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(leftline_);
  }
  if (leftline) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      leftline = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, leftline, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  leftline_ = leftline;
  // @@protoc_insertion_point(field_set_allocated:pb.si.slot_surrounding_lines.SlotSurroundingLines.leftLine)
}

// optional .pb.si.line_segment_serializable.LineSegmentSerializable rightLine = 3663;
inline bool SlotSurroundingLines::_internal_has_rightline() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  PROTOBUF_ASSUME(!value || rightline_ != nullptr);
  return value;
}
inline bool SlotSurroundingLines::has_rightline() const {
  return _internal_has_rightline();
}
inline const ::pb::si::line_segment_serializable::LineSegmentSerializable& SlotSurroundingLines::_internal_rightline() const {
  const ::pb::si::line_segment_serializable::LineSegmentSerializable* p = rightline_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::line_segment_serializable::LineSegmentSerializable*>(
      &::pb::si::line_segment_serializable::_LineSegmentSerializable_default_instance_);
}
inline const ::pb::si::line_segment_serializable::LineSegmentSerializable& SlotSurroundingLines::rightline() const {
  // @@protoc_insertion_point(field_get:pb.si.slot_surrounding_lines.SlotSurroundingLines.rightLine)
  return _internal_rightline();
}
inline ::pb::si::line_segment_serializable::LineSegmentSerializable* SlotSurroundingLines::release_rightline() {
  // @@protoc_insertion_point(field_release:pb.si.slot_surrounding_lines.SlotSurroundingLines.rightLine)
  _has_bits_[0] &= ~0x00000008u;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* temp = rightline_;
  rightline_ = nullptr;
  return temp;
}
inline ::pb::si::line_segment_serializable::LineSegmentSerializable* SlotSurroundingLines::_internal_mutable_rightline() {
  _has_bits_[0] |= 0x00000008u;
  if (rightline_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::line_segment_serializable::LineSegmentSerializable>(GetArenaNoVirtual());
    rightline_ = p;
  }
  return rightline_;
}
inline ::pb::si::line_segment_serializable::LineSegmentSerializable* SlotSurroundingLines::mutable_rightline() {
  // @@protoc_insertion_point(field_mutable:pb.si.slot_surrounding_lines.SlotSurroundingLines.rightLine)
  return _internal_mutable_rightline();
}
inline void SlotSurroundingLines::set_allocated_rightline(::pb::si::line_segment_serializable::LineSegmentSerializable* rightline) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(rightline_);
  }
  if (rightline) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      rightline = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, rightline, submessage_arena);
    }
    _has_bits_[0] |= 0x00000008u;
  } else {
    _has_bits_[0] &= ~0x00000008u;
  }
  rightline_ = rightline;
  // @@protoc_insertion_point(field_set_allocated:pb.si.slot_surrounding_lines.SlotSurroundingLines.rightLine)
}

// optional .pb.si.line_segment_serializable.LineSegmentSerializable roadLine = 2900;
inline bool SlotSurroundingLines::_internal_has_roadline() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || roadline_ != nullptr);
  return value;
}
inline bool SlotSurroundingLines::has_roadline() const {
  return _internal_has_roadline();
}
inline const ::pb::si::line_segment_serializable::LineSegmentSerializable& SlotSurroundingLines::_internal_roadline() const {
  const ::pb::si::line_segment_serializable::LineSegmentSerializable* p = roadline_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::line_segment_serializable::LineSegmentSerializable*>(
      &::pb::si::line_segment_serializable::_LineSegmentSerializable_default_instance_);
}
inline const ::pb::si::line_segment_serializable::LineSegmentSerializable& SlotSurroundingLines::roadline() const {
  // @@protoc_insertion_point(field_get:pb.si.slot_surrounding_lines.SlotSurroundingLines.roadLine)
  return _internal_roadline();
}
inline ::pb::si::line_segment_serializable::LineSegmentSerializable* SlotSurroundingLines::release_roadline() {
  // @@protoc_insertion_point(field_release:pb.si.slot_surrounding_lines.SlotSurroundingLines.roadLine)
  _has_bits_[0] &= ~0x00000002u;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* temp = roadline_;
  roadline_ = nullptr;
  return temp;
}
inline ::pb::si::line_segment_serializable::LineSegmentSerializable* SlotSurroundingLines::_internal_mutable_roadline() {
  _has_bits_[0] |= 0x00000002u;
  if (roadline_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::line_segment_serializable::LineSegmentSerializable>(GetArenaNoVirtual());
    roadline_ = p;
  }
  return roadline_;
}
inline ::pb::si::line_segment_serializable::LineSegmentSerializable* SlotSurroundingLines::mutable_roadline() {
  // @@protoc_insertion_point(field_mutable:pb.si.slot_surrounding_lines.SlotSurroundingLines.roadLine)
  return _internal_mutable_roadline();
}
inline void SlotSurroundingLines::set_allocated_roadline(::pb::si::line_segment_serializable::LineSegmentSerializable* roadline) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(roadline_);
  }
  if (roadline) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      roadline = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, roadline, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  roadline_ = roadline;
  // @@protoc_insertion_point(field_set_allocated:pb.si.slot_surrounding_lines.SlotSurroundingLines.roadLine)
}

// optional .pb.si.line_segment_serializable.LineSegmentSerializable curbLine = 3303;
inline bool SlotSurroundingLines::_internal_has_curbline() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  PROTOBUF_ASSUME(!value || curbline_ != nullptr);
  return value;
}
inline bool SlotSurroundingLines::has_curbline() const {
  return _internal_has_curbline();
}
inline const ::pb::si::line_segment_serializable::LineSegmentSerializable& SlotSurroundingLines::_internal_curbline() const {
  const ::pb::si::line_segment_serializable::LineSegmentSerializable* p = curbline_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::line_segment_serializable::LineSegmentSerializable*>(
      &::pb::si::line_segment_serializable::_LineSegmentSerializable_default_instance_);
}
inline const ::pb::si::line_segment_serializable::LineSegmentSerializable& SlotSurroundingLines::curbline() const {
  // @@protoc_insertion_point(field_get:pb.si.slot_surrounding_lines.SlotSurroundingLines.curbLine)
  return _internal_curbline();
}
inline ::pb::si::line_segment_serializable::LineSegmentSerializable* SlotSurroundingLines::release_curbline() {
  // @@protoc_insertion_point(field_release:pb.si.slot_surrounding_lines.SlotSurroundingLines.curbLine)
  _has_bits_[0] &= ~0x00000004u;
  ::pb::si::line_segment_serializable::LineSegmentSerializable* temp = curbline_;
  curbline_ = nullptr;
  return temp;
}
inline ::pb::si::line_segment_serializable::LineSegmentSerializable* SlotSurroundingLines::_internal_mutable_curbline() {
  _has_bits_[0] |= 0x00000004u;
  if (curbline_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::line_segment_serializable::LineSegmentSerializable>(GetArenaNoVirtual());
    curbline_ = p;
  }
  return curbline_;
}
inline ::pb::si::line_segment_serializable::LineSegmentSerializable* SlotSurroundingLines::mutable_curbline() {
  // @@protoc_insertion_point(field_mutable:pb.si.slot_surrounding_lines.SlotSurroundingLines.curbLine)
  return _internal_mutable_curbline();
}
inline void SlotSurroundingLines::set_allocated_curbline(::pb::si::line_segment_serializable::LineSegmentSerializable* curbline) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(curbline_);
  }
  if (curbline) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      curbline = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, curbline, submessage_arena);
    }
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  curbline_ = curbline;
  // @@protoc_insertion_point(field_set_allocated:pb.si.slot_surrounding_lines.SlotSurroundingLines.curbLine)
}

// -------------------------------------------------------------------

// SlotSurroundingLines_array_port

// repeated .pb.si.slot_surrounding_lines.SlotSurroundingLines data = 2725;
inline int SlotSurroundingLines_array_port::_internal_data_size() const {
  return data_.size();
}
inline int SlotSurroundingLines_array_port::data_size() const {
  return _internal_data_size();
}
inline void SlotSurroundingLines_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::si::slot_surrounding_lines::SlotSurroundingLines* SlotSurroundingLines_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.si.slot_surrounding_lines.SlotSurroundingLines_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::slot_surrounding_lines::SlotSurroundingLines >*
SlotSurroundingLines_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.si.slot_surrounding_lines.SlotSurroundingLines_array_port.data)
  return &data_;
}
inline const ::pb::si::slot_surrounding_lines::SlotSurroundingLines& SlotSurroundingLines_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::si::slot_surrounding_lines::SlotSurroundingLines& SlotSurroundingLines_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.si.slot_surrounding_lines.SlotSurroundingLines_array_port.data)
  return _internal_data(index);
}
inline ::pb::si::slot_surrounding_lines::SlotSurroundingLines* SlotSurroundingLines_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::si::slot_surrounding_lines::SlotSurroundingLines* SlotSurroundingLines_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.si.slot_surrounding_lines.SlotSurroundingLines_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::slot_surrounding_lines::SlotSurroundingLines >&
SlotSurroundingLines_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.si.slot_surrounding_lines.SlotSurroundingLines_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace slot_surrounding_lines
}  // namespace si
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_si_2fslot_5fsurrounding_5flines_2eproto
