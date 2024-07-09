// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: lsm_vedodo/yw_rate_t.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_lsm_5fvedodo_2fyw_5frate_5ft_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_lsm_5fvedodo_2fyw_5frate_5ft_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_lsm_5fvedodo_2fyw_5frate_5ft_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_lsm_5fvedodo_2fyw_5frate_5ft_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_lsm_5fvedodo_2fyw_5frate_5ft_2eproto;
namespace pb {
namespace lsm_vedodo {
namespace yw_rate_t {
class YwRate_t;
class YwRate_tDefaultTypeInternal;
extern YwRate_tDefaultTypeInternal _YwRate_t_default_instance_;
class YwRate_t_array_port;
class YwRate_t_array_portDefaultTypeInternal;
extern YwRate_t_array_portDefaultTypeInternal _YwRate_t_array_port_default_instance_;
}  // namespace yw_rate_t
}  // namespace lsm_vedodo
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::lsm_vedodo::yw_rate_t::YwRate_t* Arena::CreateMaybeMessage<::pb::lsm_vedodo::yw_rate_t::YwRate_t>(Arena*);
template<> ::pb::lsm_vedodo::yw_rate_t::YwRate_t_array_port* Arena::CreateMaybeMessage<::pb::lsm_vedodo::yw_rate_t::YwRate_t_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace lsm_vedodo {
namespace yw_rate_t {

// ===================================================================

class YwRate_t :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.lsm_vedodo.yw_rate_t.YwRate_t) */ {
 public:
  YwRate_t();
  virtual ~YwRate_t();

  YwRate_t(const YwRate_t& from);
  YwRate_t(YwRate_t&& from) noexcept
    : YwRate_t() {
    *this = ::std::move(from);
  }

  inline YwRate_t& operator=(const YwRate_t& from) {
    CopyFrom(from);
    return *this;
  }
  inline YwRate_t& operator=(YwRate_t&& from) noexcept {
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
  static const YwRate_t& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const YwRate_t* internal_default_instance() {
    return reinterpret_cast<const YwRate_t*>(
               &_YwRate_t_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(YwRate_t& a, YwRate_t& b) {
    a.Swap(&b);
  }
  inline void Swap(YwRate_t* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline YwRate_t* New() const final {
    return CreateMaybeMessage<YwRate_t>(nullptr);
  }

  YwRate_t* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<YwRate_t>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const YwRate_t& from);
  void MergeFrom(const YwRate_t& from);
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
  void InternalSwap(YwRate_t* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.lsm_vedodo.yw_rate_t.YwRate_t";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_lsm_5fvedodo_2fyw_5frate_5ft_2eproto);
    return ::descriptor_table_lsm_5fvedodo_2fyw_5frate_5ft_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kCalStatusFieldNumber = 735,
    kZeroRateMinFieldNumber = 793,
    kZeroRateFieldNumber = 1282,
    kZeroRateMaxFieldNumber = 2119,
  };
  // optional uint32 CalStatus = 735;
  bool has_calstatus() const;
  private:
  bool _internal_has_calstatus() const;
  public:
  void clear_calstatus();
  ::PROTOBUF_NAMESPACE_ID::uint32 calstatus() const;
  void set_calstatus(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_calstatus() const;
  void _internal_set_calstatus(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional float ZeroRateMin = 793;
  bool has_zeroratemin() const;
  private:
  bool _internal_has_zeroratemin() const;
  public:
  void clear_zeroratemin();
  float zeroratemin() const;
  void set_zeroratemin(float value);
  private:
  float _internal_zeroratemin() const;
  void _internal_set_zeroratemin(float value);
  public:

  // optional float ZeroRate = 1282;
  bool has_zerorate() const;
  private:
  bool _internal_has_zerorate() const;
  public:
  void clear_zerorate();
  float zerorate() const;
  void set_zerorate(float value);
  private:
  float _internal_zerorate() const;
  void _internal_set_zerorate(float value);
  public:

  // optional float ZeroRateMax = 2119;
  bool has_zeroratemax() const;
  private:
  bool _internal_has_zeroratemax() const;
  public:
  void clear_zeroratemax();
  float zeroratemax() const;
  void set_zeroratemax(float value);
  private:
  float _internal_zeroratemax() const;
  void _internal_set_zeroratemax(float value);
  public:

  // @@protoc_insertion_point(class_scope:pb.lsm_vedodo.yw_rate_t.YwRate_t)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 calstatus_;
  float zeroratemin_;
  float zerorate_;
  float zeroratemax_;
  friend struct ::TableStruct_lsm_5fvedodo_2fyw_5frate_5ft_2eproto;
};
// -------------------------------------------------------------------

class YwRate_t_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.lsm_vedodo.yw_rate_t.YwRate_t_array_port) */ {
 public:
  YwRate_t_array_port();
  virtual ~YwRate_t_array_port();

  YwRate_t_array_port(const YwRate_t_array_port& from);
  YwRate_t_array_port(YwRate_t_array_port&& from) noexcept
    : YwRate_t_array_port() {
    *this = ::std::move(from);
  }

  inline YwRate_t_array_port& operator=(const YwRate_t_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline YwRate_t_array_port& operator=(YwRate_t_array_port&& from) noexcept {
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
  static const YwRate_t_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const YwRate_t_array_port* internal_default_instance() {
    return reinterpret_cast<const YwRate_t_array_port*>(
               &_YwRate_t_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(YwRate_t_array_port& a, YwRate_t_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(YwRate_t_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline YwRate_t_array_port* New() const final {
    return CreateMaybeMessage<YwRate_t_array_port>(nullptr);
  }

  YwRate_t_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<YwRate_t_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const YwRate_t_array_port& from);
  void MergeFrom(const YwRate_t_array_port& from);
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
  void InternalSwap(YwRate_t_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.lsm_vedodo.yw_rate_t.YwRate_t_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_lsm_5fvedodo_2fyw_5frate_5ft_2eproto);
    return ::descriptor_table_lsm_5fvedodo_2fyw_5frate_5ft_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2919,
  };
  // repeated .pb.lsm_vedodo.yw_rate_t.YwRate_t data = 2919;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::lsm_vedodo::yw_rate_t::YwRate_t* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::yw_rate_t::YwRate_t >*
      mutable_data();
  private:
  const ::pb::lsm_vedodo::yw_rate_t::YwRate_t& _internal_data(int index) const;
  ::pb::lsm_vedodo::yw_rate_t::YwRate_t* _internal_add_data();
  public:
  const ::pb::lsm_vedodo::yw_rate_t::YwRate_t& data(int index) const;
  ::pb::lsm_vedodo::yw_rate_t::YwRate_t* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::yw_rate_t::YwRate_t >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.lsm_vedodo.yw_rate_t.YwRate_t_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::yw_rate_t::YwRate_t > data_;
  friend struct ::TableStruct_lsm_5fvedodo_2fyw_5frate_5ft_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// YwRate_t

// optional float ZeroRate = 1282;
inline bool YwRate_t::_internal_has_zerorate() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool YwRate_t::has_zerorate() const {
  return _internal_has_zerorate();
}
inline void YwRate_t::clear_zerorate() {
  zerorate_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float YwRate_t::_internal_zerorate() const {
  return zerorate_;
}
inline float YwRate_t::zerorate() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.yw_rate_t.YwRate_t.ZeroRate)
  return _internal_zerorate();
}
inline void YwRate_t::_internal_set_zerorate(float value) {
  _has_bits_[0] |= 0x00000004u;
  zerorate_ = value;
}
inline void YwRate_t::set_zerorate(float value) {
  _internal_set_zerorate(value);
  // @@protoc_insertion_point(field_set:pb.lsm_vedodo.yw_rate_t.YwRate_t.ZeroRate)
}

// optional float ZeroRateMin = 793;
inline bool YwRate_t::_internal_has_zeroratemin() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool YwRate_t::has_zeroratemin() const {
  return _internal_has_zeroratemin();
}
inline void YwRate_t::clear_zeroratemin() {
  zeroratemin_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline float YwRate_t::_internal_zeroratemin() const {
  return zeroratemin_;
}
inline float YwRate_t::zeroratemin() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.yw_rate_t.YwRate_t.ZeroRateMin)
  return _internal_zeroratemin();
}
inline void YwRate_t::_internal_set_zeroratemin(float value) {
  _has_bits_[0] |= 0x00000002u;
  zeroratemin_ = value;
}
inline void YwRate_t::set_zeroratemin(float value) {
  _internal_set_zeroratemin(value);
  // @@protoc_insertion_point(field_set:pb.lsm_vedodo.yw_rate_t.YwRate_t.ZeroRateMin)
}

// optional float ZeroRateMax = 2119;
inline bool YwRate_t::_internal_has_zeroratemax() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool YwRate_t::has_zeroratemax() const {
  return _internal_has_zeroratemax();
}
inline void YwRate_t::clear_zeroratemax() {
  zeroratemax_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float YwRate_t::_internal_zeroratemax() const {
  return zeroratemax_;
}
inline float YwRate_t::zeroratemax() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.yw_rate_t.YwRate_t.ZeroRateMax)
  return _internal_zeroratemax();
}
inline void YwRate_t::_internal_set_zeroratemax(float value) {
  _has_bits_[0] |= 0x00000008u;
  zeroratemax_ = value;
}
inline void YwRate_t::set_zeroratemax(float value) {
  _internal_set_zeroratemax(value);
  // @@protoc_insertion_point(field_set:pb.lsm_vedodo.yw_rate_t.YwRate_t.ZeroRateMax)
}

// optional uint32 CalStatus = 735;
inline bool YwRate_t::_internal_has_calstatus() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool YwRate_t::has_calstatus() const {
  return _internal_has_calstatus();
}
inline void YwRate_t::clear_calstatus() {
  calstatus_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 YwRate_t::_internal_calstatus() const {
  return calstatus_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 YwRate_t::calstatus() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.yw_rate_t.YwRate_t.CalStatus)
  return _internal_calstatus();
}
inline void YwRate_t::_internal_set_calstatus(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  calstatus_ = value;
}
inline void YwRate_t::set_calstatus(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_calstatus(value);
  // @@protoc_insertion_point(field_set:pb.lsm_vedodo.yw_rate_t.YwRate_t.CalStatus)
}

// -------------------------------------------------------------------

// YwRate_t_array_port

// repeated .pb.lsm_vedodo.yw_rate_t.YwRate_t data = 2919;
inline int YwRate_t_array_port::_internal_data_size() const {
  return data_.size();
}
inline int YwRate_t_array_port::data_size() const {
  return _internal_data_size();
}
inline void YwRate_t_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::lsm_vedodo::yw_rate_t::YwRate_t* YwRate_t_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.lsm_vedodo.yw_rate_t.YwRate_t_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::yw_rate_t::YwRate_t >*
YwRate_t_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.lsm_vedodo.yw_rate_t.YwRate_t_array_port.data)
  return &data_;
}
inline const ::pb::lsm_vedodo::yw_rate_t::YwRate_t& YwRate_t_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::lsm_vedodo::yw_rate_t::YwRate_t& YwRate_t_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.yw_rate_t.YwRate_t_array_port.data)
  return _internal_data(index);
}
inline ::pb::lsm_vedodo::yw_rate_t::YwRate_t* YwRate_t_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::lsm_vedodo::yw_rate_t::YwRate_t* YwRate_t_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.lsm_vedodo.yw_rate_t.YwRate_t_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::yw_rate_t::YwRate_t >&
YwRate_t_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.lsm_vedodo.yw_rate_t.YwRate_t_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace yw_rate_t
}  // namespace lsm_vedodo
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_lsm_5fvedodo_2fyw_5frate_5ft_2eproto