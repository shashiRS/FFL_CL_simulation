// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: com/com_version.proto

#include "com/com_version.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
extern PROTOBUF_INTERNAL_EXPORT_com_2fcom_5fversion_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ComVersion_com_2fcom_5fversion_2eproto;
namespace pb {
namespace com {
namespace com_version {
class ComVersionDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ComVersion> _instance;
} _ComVersion_default_instance_;
class ComVersion_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ComVersion_array_port> _instance;
} _ComVersion_array_port_default_instance_;
}  // namespace com_version
}  // namespace com
}  // namespace pb
static void InitDefaultsscc_info_ComVersion_com_2fcom_5fversion_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::com::com_version::_ComVersion_default_instance_;
    new (ptr) ::pb::com::com_version::ComVersion();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::com::com_version::ComVersion::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ComVersion_com_2fcom_5fversion_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_ComVersion_com_2fcom_5fversion_2eproto}, {}};

static void InitDefaultsscc_info_ComVersion_array_port_com_2fcom_5fversion_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::com::com_version::_ComVersion_array_port_default_instance_;
    new (ptr) ::pb::com::com_version::ComVersion_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::com::com_version::ComVersion_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ComVersion_array_port_com_2fcom_5fversion_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_ComVersion_array_port_com_2fcom_5fversion_2eproto}, {
      &scc_info_ComVersion_com_2fcom_5fversion_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_com_2fcom_5fversion_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_com_2fcom_5fversion_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_com_2fcom_5fversion_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_com_2fcom_5fversion_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::com::com_version::ComVersion, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::com::com_version::ComVersion, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::com::com_version::ComVersion, major_),
  PROTOBUF_FIELD_OFFSET(::pb::com::com_version::ComVersion, minor_),
  PROTOBUF_FIELD_OFFSET(::pb::com::com_version::ComVersion, patch_),
  0,
  1,
  2,
  PROTOBUF_FIELD_OFFSET(::pb::com::com_version::ComVersion_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::com::com_version::ComVersion_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::com::com_version::ComVersion_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::pb::com::com_version::ComVersion)},
  { 11, 17, sizeof(::pb::com::com_version::ComVersion_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::com::com_version::_ComVersion_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::com::com_version::_ComVersion_array_port_default_instance_),
};

const char descriptor_table_protodef_com_2fcom_5fversion_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\025com/com_version.proto\022\022pb.com.com_vers"
  "ion\"<\n\nComVersion\022\016\n\005major\030\257\021 \001(\r\022\016\n\005min"
  "or\030\254\003 \001(\r\022\016\n\005patch\030\373\007 \001(\r\"F\n\025ComVersion_"
  "array_port\022-\n\004data\030\264\027 \003(\0132\036.pb.com.com_v"
  "ersion.ComVersion"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_com_2fcom_5fversion_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_com_2fcom_5fversion_2eproto_sccs[2] = {
  &scc_info_ComVersion_com_2fcom_5fversion_2eproto.base,
  &scc_info_ComVersion_array_port_com_2fcom_5fversion_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_com_2fcom_5fversion_2eproto_once;
static bool descriptor_table_com_2fcom_5fversion_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_com_2fcom_5fversion_2eproto = {
  &descriptor_table_com_2fcom_5fversion_2eproto_initialized, descriptor_table_protodef_com_2fcom_5fversion_2eproto, "com/com_version.proto", 177,
  &descriptor_table_com_2fcom_5fversion_2eproto_once, descriptor_table_com_2fcom_5fversion_2eproto_sccs, descriptor_table_com_2fcom_5fversion_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_com_2fcom_5fversion_2eproto::offsets,
  file_level_metadata_com_2fcom_5fversion_2eproto, 2, file_level_enum_descriptors_com_2fcom_5fversion_2eproto, file_level_service_descriptors_com_2fcom_5fversion_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_com_2fcom_5fversion_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_com_2fcom_5fversion_2eproto), true);
namespace pb {
namespace com {
namespace com_version {

// ===================================================================

void ComVersion::InitAsDefaultInstance() {
}
class ComVersion::_Internal {
 public:
  using HasBits = decltype(std::declval<ComVersion>()._has_bits_);
  static void set_has_major(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_minor(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_patch(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

ComVersion::ComVersion()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.com.com_version.ComVersion)
}
ComVersion::ComVersion(const ComVersion& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&major_, &from.major_,
    static_cast<size_t>(reinterpret_cast<char*>(&patch_) -
    reinterpret_cast<char*>(&major_)) + sizeof(patch_));
  // @@protoc_insertion_point(copy_constructor:pb.com.com_version.ComVersion)
}

void ComVersion::SharedCtor() {
  ::memset(&major_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&patch_) -
      reinterpret_cast<char*>(&major_)) + sizeof(patch_));
}

ComVersion::~ComVersion() {
  // @@protoc_insertion_point(destructor:pb.com.com_version.ComVersion)
  SharedDtor();
}

void ComVersion::SharedDtor() {
}

void ComVersion::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ComVersion& ComVersion::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ComVersion_com_2fcom_5fversion_2eproto.base);
  return *internal_default_instance();
}


void ComVersion::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.com.com_version.ComVersion)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    ::memset(&major_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&patch_) -
        reinterpret_cast<char*>(&major_)) + sizeof(patch_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ComVersion::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional uint32 minor = 428;
      case 428:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 96)) {
          _Internal::set_has_minor(&has_bits);
          minor_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 patch = 1019;
      case 1019:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 216)) {
          _Internal::set_has_patch(&has_bits);
          patch_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 major = 2223;
      case 2223:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 120)) {
          _Internal::set_has_major(&has_bits);
          major_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* ComVersion::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.com.com_version.ComVersion)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional uint32 minor = 428;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(428, this->_internal_minor(), target);
  }

  // optional uint32 patch = 1019;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1019, this->_internal_patch(), target);
  }

  // optional uint32 major = 2223;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2223, this->_internal_major(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.com.com_version.ComVersion)
  return target;
}

size_t ComVersion::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.com.com_version.ComVersion)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional uint32 major = 2223;
    if (cached_has_bits & 0x00000001u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_major());
    }

    // optional uint32 minor = 428;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_minor());
    }

    // optional uint32 patch = 1019;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_patch());
    }

  }
  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ComVersion::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.com.com_version.ComVersion)
  GOOGLE_DCHECK_NE(&from, this);
  const ComVersion* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ComVersion>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.com.com_version.ComVersion)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.com.com_version.ComVersion)
    MergeFrom(*source);
  }
}

void ComVersion::MergeFrom(const ComVersion& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.com.com_version.ComVersion)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      major_ = from.major_;
    }
    if (cached_has_bits & 0x00000002u) {
      minor_ = from.minor_;
    }
    if (cached_has_bits & 0x00000004u) {
      patch_ = from.patch_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ComVersion::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.com.com_version.ComVersion)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ComVersion::CopyFrom(const ComVersion& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.com.com_version.ComVersion)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ComVersion::IsInitialized() const {
  return true;
}

void ComVersion::InternalSwap(ComVersion* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(major_, other->major_);
  swap(minor_, other->minor_);
  swap(patch_, other->patch_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ComVersion::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void ComVersion_array_port::InitAsDefaultInstance() {
}
class ComVersion_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<ComVersion_array_port>()._has_bits_);
};

ComVersion_array_port::ComVersion_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.com.com_version.ComVersion_array_port)
}
ComVersion_array_port::ComVersion_array_port(const ComVersion_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.com.com_version.ComVersion_array_port)
}

void ComVersion_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ComVersion_array_port_com_2fcom_5fversion_2eproto.base);
}

ComVersion_array_port::~ComVersion_array_port() {
  // @@protoc_insertion_point(destructor:pb.com.com_version.ComVersion_array_port)
  SharedDtor();
}

void ComVersion_array_port::SharedDtor() {
}

void ComVersion_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ComVersion_array_port& ComVersion_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ComVersion_array_port_com_2fcom_5fversion_2eproto.base);
  return *internal_default_instance();
}


void ComVersion_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.com.com_version.ComVersion_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ComVersion_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.com.com_version.ComVersion data = 2996;
      case 2996:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 162)) {
          ptr = ctx->ParseMessage(_internal_add_data(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* ComVersion_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.com.com_version.ComVersion_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.com.com_version.ComVersion data = 2996;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2996, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.com.com_version.ComVersion_array_port)
  return target;
}

size_t ComVersion_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.com.com_version.ComVersion_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.com.com_version.ComVersion data = 2996;
  total_size += 3UL * this->_internal_data_size();
  for (const auto& msg : this->data_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ComVersion_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.com.com_version.ComVersion_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const ComVersion_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ComVersion_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.com.com_version.ComVersion_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.com.com_version.ComVersion_array_port)
    MergeFrom(*source);
  }
}

void ComVersion_array_port::MergeFrom(const ComVersion_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.com.com_version.ComVersion_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void ComVersion_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.com.com_version.ComVersion_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ComVersion_array_port::CopyFrom(const ComVersion_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.com.com_version.ComVersion_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ComVersion_array_port::IsInitialized() const {
  return true;
}

void ComVersion_array_port::InternalSwap(ComVersion_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ComVersion_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace com_version
}  // namespace com
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::com::com_version::ComVersion* Arena::CreateMaybeMessage< ::pb::com::com_version::ComVersion >(Arena* arena) {
  return Arena::CreateInternal< ::pb::com::com_version::ComVersion >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::com::com_version::ComVersion_array_port* Arena::CreateMaybeMessage< ::pb::com::com_version::ComVersion_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::com::com_version::ComVersion_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
