// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/plot_data.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_si_2fplot_5fdata_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_si_2fplot_5fdata_2eproto

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
#include "si/core_plot_data.pb.h"
#include "si/low_plot_data.pb.h"
#include "si/high_plot_data.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_si_2fplot_5fdata_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_si_2fplot_5fdata_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fplot_5fdata_2eproto;
namespace pb {
namespace si {
namespace plot_data {
class PlotData;
class PlotDataDefaultTypeInternal;
extern PlotDataDefaultTypeInternal _PlotData_default_instance_;
class PlotData_array_port;
class PlotData_array_portDefaultTypeInternal;
extern PlotData_array_portDefaultTypeInternal _PlotData_array_port_default_instance_;
}  // namespace plot_data
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::si::plot_data::PlotData* Arena::CreateMaybeMessage<::pb::si::plot_data::PlotData>(Arena*);
template<> ::pb::si::plot_data::PlotData_array_port* Arena::CreateMaybeMessage<::pb::si::plot_data::PlotData_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace si {
namespace plot_data {

// ===================================================================

class PlotData :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.plot_data.PlotData) */ {
 public:
  PlotData();
  virtual ~PlotData();

  PlotData(const PlotData& from);
  PlotData(PlotData&& from) noexcept
    : PlotData() {
    *this = ::std::move(from);
  }

  inline PlotData& operator=(const PlotData& from) {
    CopyFrom(from);
    return *this;
  }
  inline PlotData& operator=(PlotData&& from) noexcept {
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
  static const PlotData& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PlotData* internal_default_instance() {
    return reinterpret_cast<const PlotData*>(
               &_PlotData_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PlotData& a, PlotData& b) {
    a.Swap(&b);
  }
  inline void Swap(PlotData* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PlotData* New() const final {
    return CreateMaybeMessage<PlotData>(nullptr);
  }

  PlotData* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PlotData>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PlotData& from);
  void MergeFrom(const PlotData& from);
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
  void InternalSwap(PlotData* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.plot_data.PlotData";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fplot_5fdata_2eproto);
    return ::descriptor_table_si_2fplot_5fdata_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kCoreFieldNumber = 1842,
    kHighFieldNumber = 2868,
    kLowFieldNumber = 3055,
  };
  // optional .pb.si.core_plot_data.CorePlotData core = 1842;
  bool has_core() const;
  private:
  bool _internal_has_core() const;
  public:
  void clear_core();
  const ::pb::si::core_plot_data::CorePlotData& core() const;
  ::pb::si::core_plot_data::CorePlotData* release_core();
  ::pb::si::core_plot_data::CorePlotData* mutable_core();
  void set_allocated_core(::pb::si::core_plot_data::CorePlotData* core);
  private:
  const ::pb::si::core_plot_data::CorePlotData& _internal_core() const;
  ::pb::si::core_plot_data::CorePlotData* _internal_mutable_core();
  public:

  // optional .pb.si.high_plot_data.HighPlotData high = 2868;
  bool has_high() const;
  private:
  bool _internal_has_high() const;
  public:
  void clear_high();
  const ::pb::si::high_plot_data::HighPlotData& high() const;
  ::pb::si::high_plot_data::HighPlotData* release_high();
  ::pb::si::high_plot_data::HighPlotData* mutable_high();
  void set_allocated_high(::pb::si::high_plot_data::HighPlotData* high);
  private:
  const ::pb::si::high_plot_data::HighPlotData& _internal_high() const;
  ::pb::si::high_plot_data::HighPlotData* _internal_mutable_high();
  public:

  // optional .pb.si.low_plot_data.LowPlotData low = 3055;
  bool has_low() const;
  private:
  bool _internal_has_low() const;
  public:
  void clear_low();
  const ::pb::si::low_plot_data::LowPlotData& low() const;
  ::pb::si::low_plot_data::LowPlotData* release_low();
  ::pb::si::low_plot_data::LowPlotData* mutable_low();
  void set_allocated_low(::pb::si::low_plot_data::LowPlotData* low);
  private:
  const ::pb::si::low_plot_data::LowPlotData& _internal_low() const;
  ::pb::si::low_plot_data::LowPlotData* _internal_mutable_low();
  public:

  // @@protoc_insertion_point(class_scope:pb.si.plot_data.PlotData)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::si::core_plot_data::CorePlotData* core_;
  ::pb::si::high_plot_data::HighPlotData* high_;
  ::pb::si::low_plot_data::LowPlotData* low_;
  friend struct ::TableStruct_si_2fplot_5fdata_2eproto;
};
// -------------------------------------------------------------------

class PlotData_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.plot_data.PlotData_array_port) */ {
 public:
  PlotData_array_port();
  virtual ~PlotData_array_port();

  PlotData_array_port(const PlotData_array_port& from);
  PlotData_array_port(PlotData_array_port&& from) noexcept
    : PlotData_array_port() {
    *this = ::std::move(from);
  }

  inline PlotData_array_port& operator=(const PlotData_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline PlotData_array_port& operator=(PlotData_array_port&& from) noexcept {
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
  static const PlotData_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PlotData_array_port* internal_default_instance() {
    return reinterpret_cast<const PlotData_array_port*>(
               &_PlotData_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(PlotData_array_port& a, PlotData_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(PlotData_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PlotData_array_port* New() const final {
    return CreateMaybeMessage<PlotData_array_port>(nullptr);
  }

  PlotData_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PlotData_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PlotData_array_port& from);
  void MergeFrom(const PlotData_array_port& from);
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
  void InternalSwap(PlotData_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.plot_data.PlotData_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fplot_5fdata_2eproto);
    return ::descriptor_table_si_2fplot_5fdata_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 210,
  };
  // repeated .pb.si.plot_data.PlotData data = 210;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::si::plot_data::PlotData* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::plot_data::PlotData >*
      mutable_data();
  private:
  const ::pb::si::plot_data::PlotData& _internal_data(int index) const;
  ::pb::si::plot_data::PlotData* _internal_add_data();
  public:
  const ::pb::si::plot_data::PlotData& data(int index) const;
  ::pb::si::plot_data::PlotData* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::plot_data::PlotData >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.si.plot_data.PlotData_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::plot_data::PlotData > data_;
  friend struct ::TableStruct_si_2fplot_5fdata_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PlotData

// optional .pb.si.core_plot_data.CorePlotData core = 1842;
inline bool PlotData::_internal_has_core() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || core_ != nullptr);
  return value;
}
inline bool PlotData::has_core() const {
  return _internal_has_core();
}
inline const ::pb::si::core_plot_data::CorePlotData& PlotData::_internal_core() const {
  const ::pb::si::core_plot_data::CorePlotData* p = core_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::core_plot_data::CorePlotData*>(
      &::pb::si::core_plot_data::_CorePlotData_default_instance_);
}
inline const ::pb::si::core_plot_data::CorePlotData& PlotData::core() const {
  // @@protoc_insertion_point(field_get:pb.si.plot_data.PlotData.core)
  return _internal_core();
}
inline ::pb::si::core_plot_data::CorePlotData* PlotData::release_core() {
  // @@protoc_insertion_point(field_release:pb.si.plot_data.PlotData.core)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::si::core_plot_data::CorePlotData* temp = core_;
  core_ = nullptr;
  return temp;
}
inline ::pb::si::core_plot_data::CorePlotData* PlotData::_internal_mutable_core() {
  _has_bits_[0] |= 0x00000001u;
  if (core_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::core_plot_data::CorePlotData>(GetArenaNoVirtual());
    core_ = p;
  }
  return core_;
}
inline ::pb::si::core_plot_data::CorePlotData* PlotData::mutable_core() {
  // @@protoc_insertion_point(field_mutable:pb.si.plot_data.PlotData.core)
  return _internal_mutable_core();
}
inline void PlotData::set_allocated_core(::pb::si::core_plot_data::CorePlotData* core) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(core_);
  }
  if (core) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      core = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, core, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  core_ = core;
  // @@protoc_insertion_point(field_set_allocated:pb.si.plot_data.PlotData.core)
}

// optional .pb.si.low_plot_data.LowPlotData low = 3055;
inline bool PlotData::_internal_has_low() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  PROTOBUF_ASSUME(!value || low_ != nullptr);
  return value;
}
inline bool PlotData::has_low() const {
  return _internal_has_low();
}
inline const ::pb::si::low_plot_data::LowPlotData& PlotData::_internal_low() const {
  const ::pb::si::low_plot_data::LowPlotData* p = low_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::low_plot_data::LowPlotData*>(
      &::pb::si::low_plot_data::_LowPlotData_default_instance_);
}
inline const ::pb::si::low_plot_data::LowPlotData& PlotData::low() const {
  // @@protoc_insertion_point(field_get:pb.si.plot_data.PlotData.low)
  return _internal_low();
}
inline ::pb::si::low_plot_data::LowPlotData* PlotData::release_low() {
  // @@protoc_insertion_point(field_release:pb.si.plot_data.PlotData.low)
  _has_bits_[0] &= ~0x00000004u;
  ::pb::si::low_plot_data::LowPlotData* temp = low_;
  low_ = nullptr;
  return temp;
}
inline ::pb::si::low_plot_data::LowPlotData* PlotData::_internal_mutable_low() {
  _has_bits_[0] |= 0x00000004u;
  if (low_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::low_plot_data::LowPlotData>(GetArenaNoVirtual());
    low_ = p;
  }
  return low_;
}
inline ::pb::si::low_plot_data::LowPlotData* PlotData::mutable_low() {
  // @@protoc_insertion_point(field_mutable:pb.si.plot_data.PlotData.low)
  return _internal_mutable_low();
}
inline void PlotData::set_allocated_low(::pb::si::low_plot_data::LowPlotData* low) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(low_);
  }
  if (low) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      low = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, low, submessage_arena);
    }
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  low_ = low;
  // @@protoc_insertion_point(field_set_allocated:pb.si.plot_data.PlotData.low)
}

// optional .pb.si.high_plot_data.HighPlotData high = 2868;
inline bool PlotData::_internal_has_high() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || high_ != nullptr);
  return value;
}
inline bool PlotData::has_high() const {
  return _internal_has_high();
}
inline const ::pb::si::high_plot_data::HighPlotData& PlotData::_internal_high() const {
  const ::pb::si::high_plot_data::HighPlotData* p = high_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::high_plot_data::HighPlotData*>(
      &::pb::si::high_plot_data::_HighPlotData_default_instance_);
}
inline const ::pb::si::high_plot_data::HighPlotData& PlotData::high() const {
  // @@protoc_insertion_point(field_get:pb.si.plot_data.PlotData.high)
  return _internal_high();
}
inline ::pb::si::high_plot_data::HighPlotData* PlotData::release_high() {
  // @@protoc_insertion_point(field_release:pb.si.plot_data.PlotData.high)
  _has_bits_[0] &= ~0x00000002u;
  ::pb::si::high_plot_data::HighPlotData* temp = high_;
  high_ = nullptr;
  return temp;
}
inline ::pb::si::high_plot_data::HighPlotData* PlotData::_internal_mutable_high() {
  _has_bits_[0] |= 0x00000002u;
  if (high_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::high_plot_data::HighPlotData>(GetArenaNoVirtual());
    high_ = p;
  }
  return high_;
}
inline ::pb::si::high_plot_data::HighPlotData* PlotData::mutable_high() {
  // @@protoc_insertion_point(field_mutable:pb.si.plot_data.PlotData.high)
  return _internal_mutable_high();
}
inline void PlotData::set_allocated_high(::pb::si::high_plot_data::HighPlotData* high) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(high_);
  }
  if (high) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      high = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, high, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  high_ = high;
  // @@protoc_insertion_point(field_set_allocated:pb.si.plot_data.PlotData.high)
}

// -------------------------------------------------------------------

// PlotData_array_port

// repeated .pb.si.plot_data.PlotData data = 210;
inline int PlotData_array_port::_internal_data_size() const {
  return data_.size();
}
inline int PlotData_array_port::data_size() const {
  return _internal_data_size();
}
inline void PlotData_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::si::plot_data::PlotData* PlotData_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.si.plot_data.PlotData_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::plot_data::PlotData >*
PlotData_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.si.plot_data.PlotData_array_port.data)
  return &data_;
}
inline const ::pb::si::plot_data::PlotData& PlotData_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::si::plot_data::PlotData& PlotData_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.si.plot_data.PlotData_array_port.data)
  return _internal_data(index);
}
inline ::pb::si::plot_data::PlotData* PlotData_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::si::plot_data::PlotData* PlotData_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.si.plot_data.PlotData_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::plot_data::PlotData >&
PlotData_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.si.plot_data.PlotData_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace plot_data
}  // namespace si
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_si_2fplot_5fdata_2eproto