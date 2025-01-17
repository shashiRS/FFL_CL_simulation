// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_drv/us_drv_consts.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fconsts_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fconsts_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fconsts_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fdrv_2fus_5fdrv_5fconsts_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fdrv_2fus_5fdrv_5fconsts_2eproto;
namespace pb {
namespace us_drv {
namespace us_drv_consts {
class US_DRV_Consts;
class US_DRV_ConstsDefaultTypeInternal;
extern US_DRV_ConstsDefaultTypeInternal _US_DRV_Consts_default_instance_;
class US_DRV_Consts_array_port;
class US_DRV_Consts_array_portDefaultTypeInternal;
extern US_DRV_Consts_array_portDefaultTypeInternal _US_DRV_Consts_array_port_default_instance_;
}  // namespace us_drv_consts
}  // namespace us_drv
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_drv::us_drv_consts::US_DRV_Consts* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_consts::US_DRV_Consts>(Arena*);
template<> ::pb::us_drv::us_drv_consts::US_DRV_Consts_array_port* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_consts::US_DRV_Consts_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_drv {
namespace us_drv_consts {

// ===================================================================

class US_DRV_Consts :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_consts.US_DRV_Consts) */ {
 public:
  US_DRV_Consts();
  virtual ~US_DRV_Consts();

  US_DRV_Consts(const US_DRV_Consts& from);
  US_DRV_Consts(US_DRV_Consts&& from) noexcept
    : US_DRV_Consts() {
    *this = ::std::move(from);
  }

  inline US_DRV_Consts& operator=(const US_DRV_Consts& from) {
    CopyFrom(from);
    return *this;
  }
  inline US_DRV_Consts& operator=(US_DRV_Consts&& from) noexcept {
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
  static const US_DRV_Consts& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const US_DRV_Consts* internal_default_instance() {
    return reinterpret_cast<const US_DRV_Consts*>(
               &_US_DRV_Consts_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(US_DRV_Consts& a, US_DRV_Consts& b) {
    a.Swap(&b);
  }
  inline void Swap(US_DRV_Consts* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline US_DRV_Consts* New() const final {
    return CreateMaybeMessage<US_DRV_Consts>(nullptr);
  }

  US_DRV_Consts* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<US_DRV_Consts>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const US_DRV_Consts& from);
  void MergeFrom(const US_DRV_Consts& from);
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
  void InternalSwap(US_DRV_Consts* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_consts.US_DRV_Consts";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fconsts_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fconsts_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kUSDRVMAXNUMASICSFieldNumber = 35,
    kUSDRVMAXNUMSIGNALPATHSFieldNumber = 260,
    kUSDRVMAXNUMSENSORSFieldNumber = 715,
    kUSDRVMAXNUMSAMPLESFieldNumber = 720,
    kUSDRVMAXNUMASICCOMMANDWORDSFieldNumber = 2829,
    kUSDRVMAXNUMDSICHANNELSFieldNumber = 3066,
    kUSDRVMAXNUMDETECTIONSFieldNumber = 3533,
    kUSDRVMAXNUMSTOCHASTICCODESFieldNumber = 4070,
  };
  // optional uint32 US_DRV_MAX_NUM_ASICS = 35;
  bool has_us_drv_max_num_asics() const;
  private:
  bool _internal_has_us_drv_max_num_asics() const;
  public:
  void clear_us_drv_max_num_asics();
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_asics() const;
  void set_us_drv_max_num_asics(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_us_drv_max_num_asics() const;
  void _internal_set_us_drv_max_num_asics(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 US_DRV_MAX_NUM_SIGNAL_PATHS = 260;
  bool has_us_drv_max_num_signal_paths() const;
  private:
  bool _internal_has_us_drv_max_num_signal_paths() const;
  public:
  void clear_us_drv_max_num_signal_paths();
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_signal_paths() const;
  void set_us_drv_max_num_signal_paths(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_us_drv_max_num_signal_paths() const;
  void _internal_set_us_drv_max_num_signal_paths(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 US_DRV_MAX_NUM_SENSORS = 715;
  bool has_us_drv_max_num_sensors() const;
  private:
  bool _internal_has_us_drv_max_num_sensors() const;
  public:
  void clear_us_drv_max_num_sensors();
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_sensors() const;
  void set_us_drv_max_num_sensors(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_us_drv_max_num_sensors() const;
  void _internal_set_us_drv_max_num_sensors(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 US_DRV_MAX_NUM_SAMPLES = 720;
  bool has_us_drv_max_num_samples() const;
  private:
  bool _internal_has_us_drv_max_num_samples() const;
  public:
  void clear_us_drv_max_num_samples();
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_samples() const;
  void set_us_drv_max_num_samples(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_us_drv_max_num_samples() const;
  void _internal_set_us_drv_max_num_samples(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 US_DRV_MAX_NUM_ASIC_COMMAND_WORDS = 2829;
  bool has_us_drv_max_num_asic_command_words() const;
  private:
  bool _internal_has_us_drv_max_num_asic_command_words() const;
  public:
  void clear_us_drv_max_num_asic_command_words();
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_asic_command_words() const;
  void set_us_drv_max_num_asic_command_words(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_us_drv_max_num_asic_command_words() const;
  void _internal_set_us_drv_max_num_asic_command_words(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 US_DRV_MAX_NUM_DSI_CHANNELS = 3066;
  bool has_us_drv_max_num_dsi_channels() const;
  private:
  bool _internal_has_us_drv_max_num_dsi_channels() const;
  public:
  void clear_us_drv_max_num_dsi_channels();
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_dsi_channels() const;
  void set_us_drv_max_num_dsi_channels(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_us_drv_max_num_dsi_channels() const;
  void _internal_set_us_drv_max_num_dsi_channels(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 US_DRV_MAX_NUM_DETECTIONS = 3533;
  bool has_us_drv_max_num_detections() const;
  private:
  bool _internal_has_us_drv_max_num_detections() const;
  public:
  void clear_us_drv_max_num_detections();
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_detections() const;
  void set_us_drv_max_num_detections(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_us_drv_max_num_detections() const;
  void _internal_set_us_drv_max_num_detections(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 US_DRV_MAX_NUM_STOCHASTIC_CODES = 4070;
  bool has_us_drv_max_num_stochastic_codes() const;
  private:
  bool _internal_has_us_drv_max_num_stochastic_codes() const;
  public:
  void clear_us_drv_max_num_stochastic_codes();
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_stochastic_codes() const;
  void set_us_drv_max_num_stochastic_codes(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_us_drv_max_num_stochastic_codes() const;
  void _internal_set_us_drv_max_num_stochastic_codes(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_consts.US_DRV_Consts)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_asics_;
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_signal_paths_;
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_sensors_;
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_samples_;
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_asic_command_words_;
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_dsi_channels_;
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_detections_;
  ::PROTOBUF_NAMESPACE_ID::uint32 us_drv_max_num_stochastic_codes_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fconsts_2eproto;
};
// -------------------------------------------------------------------

class US_DRV_Consts_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_consts.US_DRV_Consts_array_port) */ {
 public:
  US_DRV_Consts_array_port();
  virtual ~US_DRV_Consts_array_port();

  US_DRV_Consts_array_port(const US_DRV_Consts_array_port& from);
  US_DRV_Consts_array_port(US_DRV_Consts_array_port&& from) noexcept
    : US_DRV_Consts_array_port() {
    *this = ::std::move(from);
  }

  inline US_DRV_Consts_array_port& operator=(const US_DRV_Consts_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline US_DRV_Consts_array_port& operator=(US_DRV_Consts_array_port&& from) noexcept {
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
  static const US_DRV_Consts_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const US_DRV_Consts_array_port* internal_default_instance() {
    return reinterpret_cast<const US_DRV_Consts_array_port*>(
               &_US_DRV_Consts_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(US_DRV_Consts_array_port& a, US_DRV_Consts_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(US_DRV_Consts_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline US_DRV_Consts_array_port* New() const final {
    return CreateMaybeMessage<US_DRV_Consts_array_port>(nullptr);
  }

  US_DRV_Consts_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<US_DRV_Consts_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const US_DRV_Consts_array_port& from);
  void MergeFrom(const US_DRV_Consts_array_port& from);
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
  void InternalSwap(US_DRV_Consts_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_consts.US_DRV_Consts_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fconsts_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fconsts_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3426,
  };
  // repeated .pb.us_drv.us_drv_consts.US_DRV_Consts data = 3426;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_drv::us_drv_consts::US_DRV_Consts* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_consts::US_DRV_Consts >*
      mutable_data();
  private:
  const ::pb::us_drv::us_drv_consts::US_DRV_Consts& _internal_data(int index) const;
  ::pb::us_drv::us_drv_consts::US_DRV_Consts* _internal_add_data();
  public:
  const ::pb::us_drv::us_drv_consts::US_DRV_Consts& data(int index) const;
  ::pb::us_drv::us_drv_consts::US_DRV_Consts* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_consts::US_DRV_Consts >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_consts.US_DRV_Consts_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_consts::US_DRV_Consts > data_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fconsts_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// US_DRV_Consts

// optional uint32 US_DRV_MAX_NUM_ASIC_COMMAND_WORDS = 2829;
inline bool US_DRV_Consts::_internal_has_us_drv_max_num_asic_command_words() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool US_DRV_Consts::has_us_drv_max_num_asic_command_words() const {
  return _internal_has_us_drv_max_num_asic_command_words();
}
inline void US_DRV_Consts::clear_us_drv_max_num_asic_command_words() {
  us_drv_max_num_asic_command_words_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::_internal_us_drv_max_num_asic_command_words() const {
  return us_drv_max_num_asic_command_words_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::us_drv_max_num_asic_command_words() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_ASIC_COMMAND_WORDS)
  return _internal_us_drv_max_num_asic_command_words();
}
inline void US_DRV_Consts::_internal_set_us_drv_max_num_asic_command_words(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  us_drv_max_num_asic_command_words_ = value;
}
inline void US_DRV_Consts::set_us_drv_max_num_asic_command_words(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_us_drv_max_num_asic_command_words(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_ASIC_COMMAND_WORDS)
}

// optional uint32 US_DRV_MAX_NUM_SENSORS = 715;
inline bool US_DRV_Consts::_internal_has_us_drv_max_num_sensors() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool US_DRV_Consts::has_us_drv_max_num_sensors() const {
  return _internal_has_us_drv_max_num_sensors();
}
inline void US_DRV_Consts::clear_us_drv_max_num_sensors() {
  us_drv_max_num_sensors_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::_internal_us_drv_max_num_sensors() const {
  return us_drv_max_num_sensors_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::us_drv_max_num_sensors() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_SENSORS)
  return _internal_us_drv_max_num_sensors();
}
inline void US_DRV_Consts::_internal_set_us_drv_max_num_sensors(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  us_drv_max_num_sensors_ = value;
}
inline void US_DRV_Consts::set_us_drv_max_num_sensors(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_us_drv_max_num_sensors(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_SENSORS)
}

// optional uint32 US_DRV_MAX_NUM_STOCHASTIC_CODES = 4070;
inline bool US_DRV_Consts::_internal_has_us_drv_max_num_stochastic_codes() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool US_DRV_Consts::has_us_drv_max_num_stochastic_codes() const {
  return _internal_has_us_drv_max_num_stochastic_codes();
}
inline void US_DRV_Consts::clear_us_drv_max_num_stochastic_codes() {
  us_drv_max_num_stochastic_codes_ = 0u;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::_internal_us_drv_max_num_stochastic_codes() const {
  return us_drv_max_num_stochastic_codes_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::us_drv_max_num_stochastic_codes() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_STOCHASTIC_CODES)
  return _internal_us_drv_max_num_stochastic_codes();
}
inline void US_DRV_Consts::_internal_set_us_drv_max_num_stochastic_codes(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000080u;
  us_drv_max_num_stochastic_codes_ = value;
}
inline void US_DRV_Consts::set_us_drv_max_num_stochastic_codes(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_us_drv_max_num_stochastic_codes(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_STOCHASTIC_CODES)
}

// optional uint32 US_DRV_MAX_NUM_DETECTIONS = 3533;
inline bool US_DRV_Consts::_internal_has_us_drv_max_num_detections() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool US_DRV_Consts::has_us_drv_max_num_detections() const {
  return _internal_has_us_drv_max_num_detections();
}
inline void US_DRV_Consts::clear_us_drv_max_num_detections() {
  us_drv_max_num_detections_ = 0u;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::_internal_us_drv_max_num_detections() const {
  return us_drv_max_num_detections_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::us_drv_max_num_detections() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_DETECTIONS)
  return _internal_us_drv_max_num_detections();
}
inline void US_DRV_Consts::_internal_set_us_drv_max_num_detections(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000040u;
  us_drv_max_num_detections_ = value;
}
inline void US_DRV_Consts::set_us_drv_max_num_detections(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_us_drv_max_num_detections(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_DETECTIONS)
}

// optional uint32 US_DRV_MAX_NUM_SIGNAL_PATHS = 260;
inline bool US_DRV_Consts::_internal_has_us_drv_max_num_signal_paths() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool US_DRV_Consts::has_us_drv_max_num_signal_paths() const {
  return _internal_has_us_drv_max_num_signal_paths();
}
inline void US_DRV_Consts::clear_us_drv_max_num_signal_paths() {
  us_drv_max_num_signal_paths_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::_internal_us_drv_max_num_signal_paths() const {
  return us_drv_max_num_signal_paths_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::us_drv_max_num_signal_paths() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_SIGNAL_PATHS)
  return _internal_us_drv_max_num_signal_paths();
}
inline void US_DRV_Consts::_internal_set_us_drv_max_num_signal_paths(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  us_drv_max_num_signal_paths_ = value;
}
inline void US_DRV_Consts::set_us_drv_max_num_signal_paths(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_us_drv_max_num_signal_paths(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_SIGNAL_PATHS)
}

// optional uint32 US_DRV_MAX_NUM_SAMPLES = 720;
inline bool US_DRV_Consts::_internal_has_us_drv_max_num_samples() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool US_DRV_Consts::has_us_drv_max_num_samples() const {
  return _internal_has_us_drv_max_num_samples();
}
inline void US_DRV_Consts::clear_us_drv_max_num_samples() {
  us_drv_max_num_samples_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::_internal_us_drv_max_num_samples() const {
  return us_drv_max_num_samples_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::us_drv_max_num_samples() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_SAMPLES)
  return _internal_us_drv_max_num_samples();
}
inline void US_DRV_Consts::_internal_set_us_drv_max_num_samples(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  us_drv_max_num_samples_ = value;
}
inline void US_DRV_Consts::set_us_drv_max_num_samples(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_us_drv_max_num_samples(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_SAMPLES)
}

// optional uint32 US_DRV_MAX_NUM_ASICS = 35;
inline bool US_DRV_Consts::_internal_has_us_drv_max_num_asics() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool US_DRV_Consts::has_us_drv_max_num_asics() const {
  return _internal_has_us_drv_max_num_asics();
}
inline void US_DRV_Consts::clear_us_drv_max_num_asics() {
  us_drv_max_num_asics_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::_internal_us_drv_max_num_asics() const {
  return us_drv_max_num_asics_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::us_drv_max_num_asics() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_ASICS)
  return _internal_us_drv_max_num_asics();
}
inline void US_DRV_Consts::_internal_set_us_drv_max_num_asics(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  us_drv_max_num_asics_ = value;
}
inline void US_DRV_Consts::set_us_drv_max_num_asics(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_us_drv_max_num_asics(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_ASICS)
}

// optional uint32 US_DRV_MAX_NUM_DSI_CHANNELS = 3066;
inline bool US_DRV_Consts::_internal_has_us_drv_max_num_dsi_channels() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool US_DRV_Consts::has_us_drv_max_num_dsi_channels() const {
  return _internal_has_us_drv_max_num_dsi_channels();
}
inline void US_DRV_Consts::clear_us_drv_max_num_dsi_channels() {
  us_drv_max_num_dsi_channels_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::_internal_us_drv_max_num_dsi_channels() const {
  return us_drv_max_num_dsi_channels_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 US_DRV_Consts::us_drv_max_num_dsi_channels() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_DSI_CHANNELS)
  return _internal_us_drv_max_num_dsi_channels();
}
inline void US_DRV_Consts::_internal_set_us_drv_max_num_dsi_channels(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  us_drv_max_num_dsi_channels_ = value;
}
inline void US_DRV_Consts::set_us_drv_max_num_dsi_channels(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_us_drv_max_num_dsi_channels(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_consts.US_DRV_Consts.US_DRV_MAX_NUM_DSI_CHANNELS)
}

// -------------------------------------------------------------------

// US_DRV_Consts_array_port

// repeated .pb.us_drv.us_drv_consts.US_DRV_Consts data = 3426;
inline int US_DRV_Consts_array_port::_internal_data_size() const {
  return data_.size();
}
inline int US_DRV_Consts_array_port::data_size() const {
  return _internal_data_size();
}
inline void US_DRV_Consts_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_drv::us_drv_consts::US_DRV_Consts* US_DRV_Consts_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_consts.US_DRV_Consts_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_consts::US_DRV_Consts >*
US_DRV_Consts_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_consts.US_DRV_Consts_array_port.data)
  return &data_;
}
inline const ::pb::us_drv::us_drv_consts::US_DRV_Consts& US_DRV_Consts_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_drv::us_drv_consts::US_DRV_Consts& US_DRV_Consts_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_consts.US_DRV_Consts_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_drv::us_drv_consts::US_DRV_Consts* US_DRV_Consts_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_drv::us_drv_consts::US_DRV_Consts* US_DRV_Consts_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_consts.US_DRV_Consts_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_consts::US_DRV_Consts >&
US_DRV_Consts_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_consts.US_DRV_Consts_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_drv_consts
}  // namespace us_drv
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fconsts_2eproto
