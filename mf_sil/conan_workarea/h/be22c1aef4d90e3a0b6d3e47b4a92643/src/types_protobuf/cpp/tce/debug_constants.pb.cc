// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: tce/debug_constants.proto

#include "tce/debug_constants.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_tce_2fdebug_5fconstants_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_DebugConstants_tce_2fdebug_5fconstants_2eproto;
namespace pb {
namespace tce {
namespace debug_constants {
class DebugConstantsDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<DebugConstants> _instance;
} _DebugConstants_default_instance_;
class DebugConstants_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<DebugConstants_array_port> _instance;
} _DebugConstants_array_port_default_instance_;
}  // namespace debug_constants
}  // namespace tce
}  // namespace pb
static void InitDefaultsscc_info_DebugConstants_tce_2fdebug_5fconstants_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::tce::debug_constants::_DebugConstants_default_instance_;
    new (ptr) ::pb::tce::debug_constants::DebugConstants();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::tce::debug_constants::DebugConstants::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_DebugConstants_tce_2fdebug_5fconstants_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_DebugConstants_tce_2fdebug_5fconstants_2eproto}, {}};

static void InitDefaultsscc_info_DebugConstants_array_port_tce_2fdebug_5fconstants_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::tce::debug_constants::_DebugConstants_array_port_default_instance_;
    new (ptr) ::pb::tce::debug_constants::DebugConstants_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::tce::debug_constants::DebugConstants_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_DebugConstants_array_port_tce_2fdebug_5fconstants_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_DebugConstants_array_port_tce_2fdebug_5fconstants_2eproto}, {
      &scc_info_DebugConstants_tce_2fdebug_5fconstants_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_tce_2fdebug_5fconstants_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_tce_2fdebug_5fconstants_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_tce_2fdebug_5fconstants_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_tce_2fdebug_5fconstants_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::tce::debug_constants::DebugConstants, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::tce::debug_constants::DebugConstants, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::tce::debug_constants::DebugConstants, num_mts_debug_freespace_tce_),
  0,
  PROTOBUF_FIELD_OFFSET(::pb::tce::debug_constants::DebugConstants_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::tce::debug_constants::DebugConstants_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::tce::debug_constants::DebugConstants_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 6, sizeof(::pb::tce::debug_constants::DebugConstants)},
  { 7, 13, sizeof(::pb::tce::debug_constants::DebugConstants_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::tce::debug_constants::_DebugConstants_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::tce::debug_constants::_DebugConstants_array_port_default_instance_),
};

const char descriptor_table_protodef_tce_2fdebug_5fconstants_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\031tce/debug_constants.proto\022\026pb.tce.debu"
  "g_constants\"6\n\016DebugConstants\022$\n\033NUM_MTS"
  "_DEBUG_FREESPACE_TCE\030\364\016 \001(\r\"R\n\031DebugCons"
  "tants_array_port\0225\n\004data\030\232\023 \003(\0132&.pb.tce"
  ".debug_constants.DebugConstants"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_tce_2fdebug_5fconstants_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_tce_2fdebug_5fconstants_2eproto_sccs[2] = {
  &scc_info_DebugConstants_tce_2fdebug_5fconstants_2eproto.base,
  &scc_info_DebugConstants_array_port_tce_2fdebug_5fconstants_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_tce_2fdebug_5fconstants_2eproto_once;
static bool descriptor_table_tce_2fdebug_5fconstants_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_tce_2fdebug_5fconstants_2eproto = {
  &descriptor_table_tce_2fdebug_5fconstants_2eproto_initialized, descriptor_table_protodef_tce_2fdebug_5fconstants_2eproto, "tce/debug_constants.proto", 191,
  &descriptor_table_tce_2fdebug_5fconstants_2eproto_once, descriptor_table_tce_2fdebug_5fconstants_2eproto_sccs, descriptor_table_tce_2fdebug_5fconstants_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_tce_2fdebug_5fconstants_2eproto::offsets,
  file_level_metadata_tce_2fdebug_5fconstants_2eproto, 2, file_level_enum_descriptors_tce_2fdebug_5fconstants_2eproto, file_level_service_descriptors_tce_2fdebug_5fconstants_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_tce_2fdebug_5fconstants_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_tce_2fdebug_5fconstants_2eproto), true);
namespace pb {
namespace tce {
namespace debug_constants {

// ===================================================================

void DebugConstants::InitAsDefaultInstance() {
}
class DebugConstants::_Internal {
 public:
  using HasBits = decltype(std::declval<DebugConstants>()._has_bits_);
  static void set_has_num_mts_debug_freespace_tce(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

DebugConstants::DebugConstants()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.tce.debug_constants.DebugConstants)
}
DebugConstants::DebugConstants(const DebugConstants& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  num_mts_debug_freespace_tce_ = from.num_mts_debug_freespace_tce_;
  // @@protoc_insertion_point(copy_constructor:pb.tce.debug_constants.DebugConstants)
}

void DebugConstants::SharedCtor() {
  num_mts_debug_freespace_tce_ = 0u;
}

DebugConstants::~DebugConstants() {
  // @@protoc_insertion_point(destructor:pb.tce.debug_constants.DebugConstants)
  SharedDtor();
}

void DebugConstants::SharedDtor() {
}

void DebugConstants::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const DebugConstants& DebugConstants::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_DebugConstants_tce_2fdebug_5fconstants_2eproto.base);
  return *internal_default_instance();
}


void DebugConstants::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.tce.debug_constants.DebugConstants)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  num_mts_debug_freespace_tce_ = 0u;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* DebugConstants::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional uint32 NUM_MTS_DEBUG_FREESPACE_TCE = 1908;
      case 1908:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 160)) {
          _Internal::set_has_num_mts_debug_freespace_tce(&has_bits);
          num_mts_debug_freespace_tce_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* DebugConstants::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.tce.debug_constants.DebugConstants)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional uint32 NUM_MTS_DEBUG_FREESPACE_TCE = 1908;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1908, this->_internal_num_mts_debug_freespace_tce(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.tce.debug_constants.DebugConstants)
  return target;
}

size_t DebugConstants::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.tce.debug_constants.DebugConstants)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // optional uint32 NUM_MTS_DEBUG_FREESPACE_TCE = 1908;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 2 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_num_mts_debug_freespace_tce());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void DebugConstants::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.tce.debug_constants.DebugConstants)
  GOOGLE_DCHECK_NE(&from, this);
  const DebugConstants* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<DebugConstants>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.tce.debug_constants.DebugConstants)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.tce.debug_constants.DebugConstants)
    MergeFrom(*source);
  }
}

void DebugConstants::MergeFrom(const DebugConstants& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.tce.debug_constants.DebugConstants)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_num_mts_debug_freespace_tce()) {
    _internal_set_num_mts_debug_freespace_tce(from._internal_num_mts_debug_freespace_tce());
  }
}

void DebugConstants::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.tce.debug_constants.DebugConstants)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void DebugConstants::CopyFrom(const DebugConstants& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.tce.debug_constants.DebugConstants)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DebugConstants::IsInitialized() const {
  return true;
}

void DebugConstants::InternalSwap(DebugConstants* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(num_mts_debug_freespace_tce_, other->num_mts_debug_freespace_tce_);
}

::PROTOBUF_NAMESPACE_ID::Metadata DebugConstants::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void DebugConstants_array_port::InitAsDefaultInstance() {
}
class DebugConstants_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<DebugConstants_array_port>()._has_bits_);
};

DebugConstants_array_port::DebugConstants_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.tce.debug_constants.DebugConstants_array_port)
}
DebugConstants_array_port::DebugConstants_array_port(const DebugConstants_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.tce.debug_constants.DebugConstants_array_port)
}

void DebugConstants_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_DebugConstants_array_port_tce_2fdebug_5fconstants_2eproto.base);
}

DebugConstants_array_port::~DebugConstants_array_port() {
  // @@protoc_insertion_point(destructor:pb.tce.debug_constants.DebugConstants_array_port)
  SharedDtor();
}

void DebugConstants_array_port::SharedDtor() {
}

void DebugConstants_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const DebugConstants_array_port& DebugConstants_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_DebugConstants_array_port_tce_2fdebug_5fconstants_2eproto.base);
  return *internal_default_instance();
}


void DebugConstants_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.tce.debug_constants.DebugConstants_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* DebugConstants_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.tce.debug_constants.DebugConstants data = 2458;
      case 2458:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 210)) {
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

::PROTOBUF_NAMESPACE_ID::uint8* DebugConstants_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.tce.debug_constants.DebugConstants_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.tce.debug_constants.DebugConstants data = 2458;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2458, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.tce.debug_constants.DebugConstants_array_port)
  return target;
}

size_t DebugConstants_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.tce.debug_constants.DebugConstants_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.tce.debug_constants.DebugConstants data = 2458;
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

void DebugConstants_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.tce.debug_constants.DebugConstants_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const DebugConstants_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<DebugConstants_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.tce.debug_constants.DebugConstants_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.tce.debug_constants.DebugConstants_array_port)
    MergeFrom(*source);
  }
}

void DebugConstants_array_port::MergeFrom(const DebugConstants_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.tce.debug_constants.DebugConstants_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void DebugConstants_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.tce.debug_constants.DebugConstants_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void DebugConstants_array_port::CopyFrom(const DebugConstants_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.tce.debug_constants.DebugConstants_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DebugConstants_array_port::IsInitialized() const {
  return true;
}

void DebugConstants_array_port::InternalSwap(DebugConstants_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata DebugConstants_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace debug_constants
}  // namespace tce
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::tce::debug_constants::DebugConstants* Arena::CreateMaybeMessage< ::pb::tce::debug_constants::DebugConstants >(Arena* arena) {
  return Arena::CreateInternal< ::pb::tce::debug_constants::DebugConstants >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::tce::debug_constants::DebugConstants_array_port* Arena::CreateMaybeMessage< ::pb::tce::debug_constants::DebugConstants_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::tce::debug_constants::DebugConstants_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>