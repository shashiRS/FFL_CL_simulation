// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_lsca/config_steer_resist_t.proto

#include "mf_lsca/config_steer_resist_t.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_configSteerResist_t_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto;
namespace pb {
namespace mf_lsca {
namespace config_steer_resist_t {
class configSteerResist_tDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<configSteerResist_t> _instance;
} _configSteerResist_t_default_instance_;
class configSteerResist_t_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<configSteerResist_t_array_port> _instance;
} _configSteerResist_t_array_port_default_instance_;
}  // namespace config_steer_resist_t
}  // namespace mf_lsca
}  // namespace pb
static void InitDefaultsscc_info_configSteerResist_t_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_lsca::config_steer_resist_t::_configSteerResist_t_default_instance_;
    new (ptr) ::pb::mf_lsca::config_steer_resist_t::configSteerResist_t();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_lsca::config_steer_resist_t::configSteerResist_t::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_configSteerResist_t_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_configSteerResist_t_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto}, {}};

static void InitDefaultsscc_info_configSteerResist_t_array_port_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_lsca::config_steer_resist_t::_configSteerResist_t_array_port_default_instance_;
    new (ptr) ::pb::mf_lsca::config_steer_resist_t::configSteerResist_t_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_lsca::config_steer_resist_t::configSteerResist_t_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_configSteerResist_t_array_port_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_configSteerResist_t_array_port_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto}, {
      &scc_info_configSteerResist_t_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::mf_lsca::config_steer_resist_t::configSteerResist_t, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_lsca::config_steer_resist_t::configSteerResist_t, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_lsca::config_steer_resist_t::configSteerResist_t, steermargin_deg_),
  0,
  PROTOBUF_FIELD_OFFSET(::pb::mf_lsca::config_steer_resist_t::configSteerResist_t_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_lsca::config_steer_resist_t::configSteerResist_t_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_lsca::config_steer_resist_t::configSteerResist_t_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 6, sizeof(::pb::mf_lsca::config_steer_resist_t::configSteerResist_t)},
  { 7, 13, sizeof(::pb::mf_lsca::config_steer_resist_t::configSteerResist_t_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_lsca::config_steer_resist_t::_configSteerResist_t_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_lsca::config_steer_resist_t::_configSteerResist_t_array_port_default_instance_),
};

const char descriptor_table_protodef_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n#mf_lsca/config_steer_resist_t.proto\022 p"
  "b.mf_lsca.config_steer_resist_t\"/\n\023confi"
  "gSteerResist_t\022\030\n\017steerMargin_deg\030\351\020 \001(\002"
  "\"f\n\036configSteerResist_t_array_port\022D\n\004da"
  "ta\030\202\017 \003(\01325.pb.mf_lsca.config_steer_resi"
  "st_t.configSteerResist_t"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto_sccs[2] = {
  &scc_info_configSteerResist_t_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto.base,
  &scc_info_configSteerResist_t_array_port_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto_once;
static bool descriptor_table_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto = {
  &descriptor_table_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto_initialized, descriptor_table_protodef_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto, "mf_lsca/config_steer_resist_t.proto", 224,
  &descriptor_table_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto_once, descriptor_table_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto_sccs, descriptor_table_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto::offsets,
  file_level_metadata_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto, 2, file_level_enum_descriptors_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto, file_level_service_descriptors_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto), true);
namespace pb {
namespace mf_lsca {
namespace config_steer_resist_t {

// ===================================================================

void configSteerResist_t::InitAsDefaultInstance() {
}
class configSteerResist_t::_Internal {
 public:
  using HasBits = decltype(std::declval<configSteerResist_t>()._has_bits_);
  static void set_has_steermargin_deg(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

configSteerResist_t::configSteerResist_t()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
}
configSteerResist_t::configSteerResist_t(const configSteerResist_t& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  steermargin_deg_ = from.steermargin_deg_;
  // @@protoc_insertion_point(copy_constructor:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
}

void configSteerResist_t::SharedCtor() {
  steermargin_deg_ = 0;
}

configSteerResist_t::~configSteerResist_t() {
  // @@protoc_insertion_point(destructor:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
  SharedDtor();
}

void configSteerResist_t::SharedDtor() {
}

void configSteerResist_t::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const configSteerResist_t& configSteerResist_t::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_configSteerResist_t_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto.base);
  return *internal_default_instance();
}


void configSteerResist_t::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  steermargin_deg_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* configSteerResist_t::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional float steerMargin_deg = 2153;
      case 2153:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 77)) {
          _Internal::set_has_steermargin_deg(&has_bits);
          steermargin_deg_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
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

::PROTOBUF_NAMESPACE_ID::uint8* configSteerResist_t::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional float steerMargin_deg = 2153;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2153, this->_internal_steermargin_deg(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
  return target;
}

size_t configSteerResist_t::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // optional float steerMargin_deg = 2153;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 3 + 4;
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void configSteerResist_t::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
  GOOGLE_DCHECK_NE(&from, this);
  const configSteerResist_t* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<configSteerResist_t>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
    MergeFrom(*source);
  }
}

void configSteerResist_t::MergeFrom(const configSteerResist_t& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_steermargin_deg()) {
    _internal_set_steermargin_deg(from._internal_steermargin_deg());
  }
}

void configSteerResist_t::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void configSteerResist_t::CopyFrom(const configSteerResist_t& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool configSteerResist_t::IsInitialized() const {
  return true;
}

void configSteerResist_t::InternalSwap(configSteerResist_t* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(steermargin_deg_, other->steermargin_deg_);
}

::PROTOBUF_NAMESPACE_ID::Metadata configSteerResist_t::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void configSteerResist_t_array_port::InitAsDefaultInstance() {
}
class configSteerResist_t_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<configSteerResist_t_array_port>()._has_bits_);
};

configSteerResist_t_array_port::configSteerResist_t_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
}
configSteerResist_t_array_port::configSteerResist_t_array_port(const configSteerResist_t_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
}

void configSteerResist_t_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_configSteerResist_t_array_port_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto.base);
}

configSteerResist_t_array_port::~configSteerResist_t_array_port() {
  // @@protoc_insertion_point(destructor:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
  SharedDtor();
}

void configSteerResist_t_array_port::SharedDtor() {
}

void configSteerResist_t_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const configSteerResist_t_array_port& configSteerResist_t_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_configSteerResist_t_array_port_mf_5flsca_2fconfig_5fsteer_5fresist_5ft_2eproto.base);
  return *internal_default_instance();
}


void configSteerResist_t_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* configSteerResist_t_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.mf_lsca.config_steer_resist_t.configSteerResist_t data = 1922;
      case 1922:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<15378>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* configSteerResist_t_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.mf_lsca.config_steer_resist_t.configSteerResist_t data = 1922;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1922, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
  return target;
}

size_t configSteerResist_t_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.mf_lsca.config_steer_resist_t.configSteerResist_t data = 1922;
  total_size += 2UL * this->_internal_data_size();
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

void configSteerResist_t_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const configSteerResist_t_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<configSteerResist_t_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
    MergeFrom(*source);
  }
}

void configSteerResist_t_array_port::MergeFrom(const configSteerResist_t_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void configSteerResist_t_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void configSteerResist_t_array_port::CopyFrom(const configSteerResist_t_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool configSteerResist_t_array_port::IsInitialized() const {
  return true;
}

void configSteerResist_t_array_port::InternalSwap(configSteerResist_t_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata configSteerResist_t_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace config_steer_resist_t
}  // namespace mf_lsca
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::mf_lsca::config_steer_resist_t::configSteerResist_t* Arena::CreateMaybeMessage< ::pb::mf_lsca::config_steer_resist_t::configSteerResist_t >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_lsca::config_steer_resist_t::configSteerResist_t >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::mf_lsca::config_steer_resist_t::configSteerResist_t_array_port* Arena::CreateMaybeMessage< ::pb::mf_lsca::config_steer_resist_t::configSteerResist_t_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_lsca::config_steer_resist_t::configSteerResist_t_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>