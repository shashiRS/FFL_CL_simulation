// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_psm_app/bcmreq.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_5fapp_2fbcmreq_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_5fapp_2fbcmreq_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ap_5fpsm_5fapp_2fbcmreq_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fpsm_5fapp_2fbcmreq_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fpsm_5fapp_2fbcmreq_2eproto;
namespace pb {
namespace ap_psm_app {
namespace bcmreq {
class BCMReq;
class BCMReqDefaultTypeInternal;
extern BCMReqDefaultTypeInternal _BCMReq_default_instance_;
class BCMReq_array_port;
class BCMReq_array_portDefaultTypeInternal;
extern BCMReq_array_portDefaultTypeInternal _BCMReq_array_port_default_instance_;
}  // namespace bcmreq
}  // namespace ap_psm_app
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_psm_app::bcmreq::BCMReq* Arena::CreateMaybeMessage<::pb::ap_psm_app::bcmreq::BCMReq>(Arena*);
template<> ::pb::ap_psm_app::bcmreq::BCMReq_array_port* Arena::CreateMaybeMessage<::pb::ap_psm_app::bcmreq::BCMReq_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_psm_app {
namespace bcmreq {

// ===================================================================

class BCMReq :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_psm_app.bcmreq.BCMReq) */ {
 public:
  BCMReq();
  virtual ~BCMReq();

  BCMReq(const BCMReq& from);
  BCMReq(BCMReq&& from) noexcept
    : BCMReq() {
    *this = ::std::move(from);
  }

  inline BCMReq& operator=(const BCMReq& from) {
    CopyFrom(from);
    return *this;
  }
  inline BCMReq& operator=(BCMReq&& from) noexcept {
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
  static const BCMReq& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const BCMReq* internal_default_instance() {
    return reinterpret_cast<const BCMReq*>(
               &_BCMReq_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(BCMReq& a, BCMReq& b) {
    a.Swap(&b);
  }
  inline void Swap(BCMReq* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline BCMReq* New() const final {
    return CreateMaybeMessage<BCMReq>(nullptr);
  }

  BCMReq* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<BCMReq>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const BCMReq& from);
  void MergeFrom(const BCMReq& from);
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
  void InternalSwap(BCMReq* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_psm_app.bcmreq.BCMReq";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fpsm_5fapp_2fbcmreq_2eproto);
    return ::descriptor_table_ap_5fpsm_5fapp_2fbcmreq_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kBlockConvTopActivationNuFieldNumber = 2390,
    kLowBeamLightReqNuFieldNumber = 707,
    kBackUpLightReqNuFieldNumber = 775,
    kFoldMirrorsNuFieldNumber = 1850,
    kUnFoldMirrorsNuFieldNumber = 2118,
    kDirIndLeftReqNuFieldNumber = 3851,
    kDirIndRightReqNuFieldNumber = 2133,
    kHazardWarningNuFieldNumber = 3218,
    kEnvironmentLightReqNuFieldNumber = 275,
  };
  // optional bool blockConvTopActivation_nu = 2390;
  bool has_blockconvtopactivation_nu() const;
  private:
  bool _internal_has_blockconvtopactivation_nu() const;
  public:
  void clear_blockconvtopactivation_nu();
  bool blockconvtopactivation_nu() const;
  void set_blockconvtopactivation_nu(bool value);
  private:
  bool _internal_blockconvtopactivation_nu() const;
  void _internal_set_blockconvtopactivation_nu(bool value);
  public:

  // optional bool lowBeamLightReq_nu = 707;
  bool has_lowbeamlightreq_nu() const;
  private:
  bool _internal_has_lowbeamlightreq_nu() const;
  public:
  void clear_lowbeamlightreq_nu();
  bool lowbeamlightreq_nu() const;
  void set_lowbeamlightreq_nu(bool value);
  private:
  bool _internal_lowbeamlightreq_nu() const;
  void _internal_set_lowbeamlightreq_nu(bool value);
  public:

  // optional bool backUpLightReq_nu = 775;
  bool has_backuplightreq_nu() const;
  private:
  bool _internal_has_backuplightreq_nu() const;
  public:
  void clear_backuplightreq_nu();
  bool backuplightreq_nu() const;
  void set_backuplightreq_nu(bool value);
  private:
  bool _internal_backuplightreq_nu() const;
  void _internal_set_backuplightreq_nu(bool value);
  public:

  // optional bool foldMirrors_nu = 1850;
  bool has_foldmirrors_nu() const;
  private:
  bool _internal_has_foldmirrors_nu() const;
  public:
  void clear_foldmirrors_nu();
  bool foldmirrors_nu() const;
  void set_foldmirrors_nu(bool value);
  private:
  bool _internal_foldmirrors_nu() const;
  void _internal_set_foldmirrors_nu(bool value);
  public:

  // optional bool unFoldMirrors_nu = 2118;
  bool has_unfoldmirrors_nu() const;
  private:
  bool _internal_has_unfoldmirrors_nu() const;
  public:
  void clear_unfoldmirrors_nu();
  bool unfoldmirrors_nu() const;
  void set_unfoldmirrors_nu(bool value);
  private:
  bool _internal_unfoldmirrors_nu() const;
  void _internal_set_unfoldmirrors_nu(bool value);
  public:

  // optional bool dirIndLeftReq_nu = 3851;
  bool has_dirindleftreq_nu() const;
  private:
  bool _internal_has_dirindleftreq_nu() const;
  public:
  void clear_dirindleftreq_nu();
  bool dirindleftreq_nu() const;
  void set_dirindleftreq_nu(bool value);
  private:
  bool _internal_dirindleftreq_nu() const;
  void _internal_set_dirindleftreq_nu(bool value);
  public:

  // optional bool dirIndRightReq_nu = 2133;
  bool has_dirindrightreq_nu() const;
  private:
  bool _internal_has_dirindrightreq_nu() const;
  public:
  void clear_dirindrightreq_nu();
  bool dirindrightreq_nu() const;
  void set_dirindrightreq_nu(bool value);
  private:
  bool _internal_dirindrightreq_nu() const;
  void _internal_set_dirindrightreq_nu(bool value);
  public:

  // optional bool hazardWarning_nu = 3218;
  bool has_hazardwarning_nu() const;
  private:
  bool _internal_has_hazardwarning_nu() const;
  public:
  void clear_hazardwarning_nu();
  bool hazardwarning_nu() const;
  void set_hazardwarning_nu(bool value);
  private:
  bool _internal_hazardwarning_nu() const;
  void _internal_set_hazardwarning_nu(bool value);
  public:

  // optional bool environmentLightReq_nu = 275;
  bool has_environmentlightreq_nu() const;
  private:
  bool _internal_has_environmentlightreq_nu() const;
  public:
  void clear_environmentlightreq_nu();
  bool environmentlightreq_nu() const;
  void set_environmentlightreq_nu(bool value);
  private:
  bool _internal_environmentlightreq_nu() const;
  void _internal_set_environmentlightreq_nu(bool value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_psm_app.bcmreq.BCMReq)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  bool blockconvtopactivation_nu_;
  bool lowbeamlightreq_nu_;
  bool backuplightreq_nu_;
  bool foldmirrors_nu_;
  bool unfoldmirrors_nu_;
  bool dirindleftreq_nu_;
  bool dirindrightreq_nu_;
  bool hazardwarning_nu_;
  bool environmentlightreq_nu_;
  friend struct ::TableStruct_ap_5fpsm_5fapp_2fbcmreq_2eproto;
};
// -------------------------------------------------------------------

class BCMReq_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_psm_app.bcmreq.BCMReq_array_port) */ {
 public:
  BCMReq_array_port();
  virtual ~BCMReq_array_port();

  BCMReq_array_port(const BCMReq_array_port& from);
  BCMReq_array_port(BCMReq_array_port&& from) noexcept
    : BCMReq_array_port() {
    *this = ::std::move(from);
  }

  inline BCMReq_array_port& operator=(const BCMReq_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline BCMReq_array_port& operator=(BCMReq_array_port&& from) noexcept {
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
  static const BCMReq_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const BCMReq_array_port* internal_default_instance() {
    return reinterpret_cast<const BCMReq_array_port*>(
               &_BCMReq_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(BCMReq_array_port& a, BCMReq_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(BCMReq_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline BCMReq_array_port* New() const final {
    return CreateMaybeMessage<BCMReq_array_port>(nullptr);
  }

  BCMReq_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<BCMReq_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const BCMReq_array_port& from);
  void MergeFrom(const BCMReq_array_port& from);
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
  void InternalSwap(BCMReq_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_psm_app.bcmreq.BCMReq_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fpsm_5fapp_2fbcmreq_2eproto);
    return ::descriptor_table_ap_5fpsm_5fapp_2fbcmreq_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2276,
  };
  // repeated .pb.ap_psm_app.bcmreq.BCMReq data = 2276;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_psm_app::bcmreq::BCMReq* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm_app::bcmreq::BCMReq >*
      mutable_data();
  private:
  const ::pb::ap_psm_app::bcmreq::BCMReq& _internal_data(int index) const;
  ::pb::ap_psm_app::bcmreq::BCMReq* _internal_add_data();
  public:
  const ::pb::ap_psm_app::bcmreq::BCMReq& data(int index) const;
  ::pb::ap_psm_app::bcmreq::BCMReq* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm_app::bcmreq::BCMReq >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_psm_app.bcmreq.BCMReq_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm_app::bcmreq::BCMReq > data_;
  friend struct ::TableStruct_ap_5fpsm_5fapp_2fbcmreq_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// BCMReq

// optional bool dirIndLeftReq_nu = 3851;
inline bool BCMReq::_internal_has_dirindleftreq_nu() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool BCMReq::has_dirindleftreq_nu() const {
  return _internal_has_dirindleftreq_nu();
}
inline void BCMReq::clear_dirindleftreq_nu() {
  dirindleftreq_nu_ = false;
  _has_bits_[0] &= ~0x00000020u;
}
inline bool BCMReq::_internal_dirindleftreq_nu() const {
  return dirindleftreq_nu_;
}
inline bool BCMReq::dirindleftreq_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm_app.bcmreq.BCMReq.dirIndLeftReq_nu)
  return _internal_dirindleftreq_nu();
}
inline void BCMReq::_internal_set_dirindleftreq_nu(bool value) {
  _has_bits_[0] |= 0x00000020u;
  dirindleftreq_nu_ = value;
}
inline void BCMReq::set_dirindleftreq_nu(bool value) {
  _internal_set_dirindleftreq_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm_app.bcmreq.BCMReq.dirIndLeftReq_nu)
}

// optional bool dirIndRightReq_nu = 2133;
inline bool BCMReq::_internal_has_dirindrightreq_nu() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool BCMReq::has_dirindrightreq_nu() const {
  return _internal_has_dirindrightreq_nu();
}
inline void BCMReq::clear_dirindrightreq_nu() {
  dirindrightreq_nu_ = false;
  _has_bits_[0] &= ~0x00000040u;
}
inline bool BCMReq::_internal_dirindrightreq_nu() const {
  return dirindrightreq_nu_;
}
inline bool BCMReq::dirindrightreq_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm_app.bcmreq.BCMReq.dirIndRightReq_nu)
  return _internal_dirindrightreq_nu();
}
inline void BCMReq::_internal_set_dirindrightreq_nu(bool value) {
  _has_bits_[0] |= 0x00000040u;
  dirindrightreq_nu_ = value;
}
inline void BCMReq::set_dirindrightreq_nu(bool value) {
  _internal_set_dirindrightreq_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm_app.bcmreq.BCMReq.dirIndRightReq_nu)
}

// optional bool hazardWarning_nu = 3218;
inline bool BCMReq::_internal_has_hazardwarning_nu() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool BCMReq::has_hazardwarning_nu() const {
  return _internal_has_hazardwarning_nu();
}
inline void BCMReq::clear_hazardwarning_nu() {
  hazardwarning_nu_ = false;
  _has_bits_[0] &= ~0x00000080u;
}
inline bool BCMReq::_internal_hazardwarning_nu() const {
  return hazardwarning_nu_;
}
inline bool BCMReq::hazardwarning_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm_app.bcmreq.BCMReq.hazardWarning_nu)
  return _internal_hazardwarning_nu();
}
inline void BCMReq::_internal_set_hazardwarning_nu(bool value) {
  _has_bits_[0] |= 0x00000080u;
  hazardwarning_nu_ = value;
}
inline void BCMReq::set_hazardwarning_nu(bool value) {
  _internal_set_hazardwarning_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm_app.bcmreq.BCMReq.hazardWarning_nu)
}

// optional bool environmentLightReq_nu = 275;
inline bool BCMReq::_internal_has_environmentlightreq_nu() const {
  bool value = (_has_bits_[0] & 0x00000100u) != 0;
  return value;
}
inline bool BCMReq::has_environmentlightreq_nu() const {
  return _internal_has_environmentlightreq_nu();
}
inline void BCMReq::clear_environmentlightreq_nu() {
  environmentlightreq_nu_ = false;
  _has_bits_[0] &= ~0x00000100u;
}
inline bool BCMReq::_internal_environmentlightreq_nu() const {
  return environmentlightreq_nu_;
}
inline bool BCMReq::environmentlightreq_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm_app.bcmreq.BCMReq.environmentLightReq_nu)
  return _internal_environmentlightreq_nu();
}
inline void BCMReq::_internal_set_environmentlightreq_nu(bool value) {
  _has_bits_[0] |= 0x00000100u;
  environmentlightreq_nu_ = value;
}
inline void BCMReq::set_environmentlightreq_nu(bool value) {
  _internal_set_environmentlightreq_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm_app.bcmreq.BCMReq.environmentLightReq_nu)
}

// optional bool lowBeamLightReq_nu = 707;
inline bool BCMReq::_internal_has_lowbeamlightreq_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool BCMReq::has_lowbeamlightreq_nu() const {
  return _internal_has_lowbeamlightreq_nu();
}
inline void BCMReq::clear_lowbeamlightreq_nu() {
  lowbeamlightreq_nu_ = false;
  _has_bits_[0] &= ~0x00000002u;
}
inline bool BCMReq::_internal_lowbeamlightreq_nu() const {
  return lowbeamlightreq_nu_;
}
inline bool BCMReq::lowbeamlightreq_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm_app.bcmreq.BCMReq.lowBeamLightReq_nu)
  return _internal_lowbeamlightreq_nu();
}
inline void BCMReq::_internal_set_lowbeamlightreq_nu(bool value) {
  _has_bits_[0] |= 0x00000002u;
  lowbeamlightreq_nu_ = value;
}
inline void BCMReq::set_lowbeamlightreq_nu(bool value) {
  _internal_set_lowbeamlightreq_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm_app.bcmreq.BCMReq.lowBeamLightReq_nu)
}

// optional bool backUpLightReq_nu = 775;
inline bool BCMReq::_internal_has_backuplightreq_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool BCMReq::has_backuplightreq_nu() const {
  return _internal_has_backuplightreq_nu();
}
inline void BCMReq::clear_backuplightreq_nu() {
  backuplightreq_nu_ = false;
  _has_bits_[0] &= ~0x00000004u;
}
inline bool BCMReq::_internal_backuplightreq_nu() const {
  return backuplightreq_nu_;
}
inline bool BCMReq::backuplightreq_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm_app.bcmreq.BCMReq.backUpLightReq_nu)
  return _internal_backuplightreq_nu();
}
inline void BCMReq::_internal_set_backuplightreq_nu(bool value) {
  _has_bits_[0] |= 0x00000004u;
  backuplightreq_nu_ = value;
}
inline void BCMReq::set_backuplightreq_nu(bool value) {
  _internal_set_backuplightreq_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm_app.bcmreq.BCMReq.backUpLightReq_nu)
}

// optional bool foldMirrors_nu = 1850;
inline bool BCMReq::_internal_has_foldmirrors_nu() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool BCMReq::has_foldmirrors_nu() const {
  return _internal_has_foldmirrors_nu();
}
inline void BCMReq::clear_foldmirrors_nu() {
  foldmirrors_nu_ = false;
  _has_bits_[0] &= ~0x00000008u;
}
inline bool BCMReq::_internal_foldmirrors_nu() const {
  return foldmirrors_nu_;
}
inline bool BCMReq::foldmirrors_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm_app.bcmreq.BCMReq.foldMirrors_nu)
  return _internal_foldmirrors_nu();
}
inline void BCMReq::_internal_set_foldmirrors_nu(bool value) {
  _has_bits_[0] |= 0x00000008u;
  foldmirrors_nu_ = value;
}
inline void BCMReq::set_foldmirrors_nu(bool value) {
  _internal_set_foldmirrors_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm_app.bcmreq.BCMReq.foldMirrors_nu)
}

// optional bool unFoldMirrors_nu = 2118;
inline bool BCMReq::_internal_has_unfoldmirrors_nu() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool BCMReq::has_unfoldmirrors_nu() const {
  return _internal_has_unfoldmirrors_nu();
}
inline void BCMReq::clear_unfoldmirrors_nu() {
  unfoldmirrors_nu_ = false;
  _has_bits_[0] &= ~0x00000010u;
}
inline bool BCMReq::_internal_unfoldmirrors_nu() const {
  return unfoldmirrors_nu_;
}
inline bool BCMReq::unfoldmirrors_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm_app.bcmreq.BCMReq.unFoldMirrors_nu)
  return _internal_unfoldmirrors_nu();
}
inline void BCMReq::_internal_set_unfoldmirrors_nu(bool value) {
  _has_bits_[0] |= 0x00000010u;
  unfoldmirrors_nu_ = value;
}
inline void BCMReq::set_unfoldmirrors_nu(bool value) {
  _internal_set_unfoldmirrors_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm_app.bcmreq.BCMReq.unFoldMirrors_nu)
}

// optional bool blockConvTopActivation_nu = 2390;
inline bool BCMReq::_internal_has_blockconvtopactivation_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool BCMReq::has_blockconvtopactivation_nu() const {
  return _internal_has_blockconvtopactivation_nu();
}
inline void BCMReq::clear_blockconvtopactivation_nu() {
  blockconvtopactivation_nu_ = false;
  _has_bits_[0] &= ~0x00000001u;
}
inline bool BCMReq::_internal_blockconvtopactivation_nu() const {
  return blockconvtopactivation_nu_;
}
inline bool BCMReq::blockconvtopactivation_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm_app.bcmreq.BCMReq.blockConvTopActivation_nu)
  return _internal_blockconvtopactivation_nu();
}
inline void BCMReq::_internal_set_blockconvtopactivation_nu(bool value) {
  _has_bits_[0] |= 0x00000001u;
  blockconvtopactivation_nu_ = value;
}
inline void BCMReq::set_blockconvtopactivation_nu(bool value) {
  _internal_set_blockconvtopactivation_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm_app.bcmreq.BCMReq.blockConvTopActivation_nu)
}

// -------------------------------------------------------------------

// BCMReq_array_port

// repeated .pb.ap_psm_app.bcmreq.BCMReq data = 2276;
inline int BCMReq_array_port::_internal_data_size() const {
  return data_.size();
}
inline int BCMReq_array_port::data_size() const {
  return _internal_data_size();
}
inline void BCMReq_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_psm_app::bcmreq::BCMReq* BCMReq_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_psm_app.bcmreq.BCMReq_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm_app::bcmreq::BCMReq >*
BCMReq_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_psm_app.bcmreq.BCMReq_array_port.data)
  return &data_;
}
inline const ::pb::ap_psm_app::bcmreq::BCMReq& BCMReq_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_psm_app::bcmreq::BCMReq& BCMReq_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_psm_app.bcmreq.BCMReq_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_psm_app::bcmreq::BCMReq* BCMReq_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_psm_app::bcmreq::BCMReq* BCMReq_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_psm_app.bcmreq.BCMReq_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm_app::bcmreq::BCMReq >&
BCMReq_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_psm_app.bcmreq.BCMReq_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace bcmreq
}  // namespace ap_psm_app
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_5fapp_2fbcmreq_2eproto