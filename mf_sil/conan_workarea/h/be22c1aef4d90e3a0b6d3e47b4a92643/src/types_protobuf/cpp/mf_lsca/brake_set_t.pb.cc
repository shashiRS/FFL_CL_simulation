// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_lsca/brake_set_t.proto

#include "mf_lsca/brake_set_t.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_mf_5flsca_2fbrake_5fmodel_5ft_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_brakeModel_t_mf_5flsca_2fbrake_5fmodel_5ft_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_mf_5flsca_2fbrake_5fset_5ft_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_brakeSet_t_mf_5flsca_2fbrake_5fset_5ft_2eproto;
namespace pb {
namespace mf_lsca {
namespace brake_set_t {
class brakeSet_tDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<brakeSet_t> _instance;
} _brakeSet_t_default_instance_;
class brakeSet_t_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<brakeSet_t_array_port> _instance;
} _brakeSet_t_array_port_default_instance_;
}  // namespace brake_set_t
}  // namespace mf_lsca
}  // namespace pb
static void InitDefaultsscc_info_brakeSet_t_mf_5flsca_2fbrake_5fset_5ft_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_lsca::brake_set_t::_brakeSet_t_default_instance_;
    new (ptr) ::pb::mf_lsca::brake_set_t::brakeSet_t();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_lsca::brake_set_t::brakeSet_t::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_brakeSet_t_mf_5flsca_2fbrake_5fset_5ft_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_brakeSet_t_mf_5flsca_2fbrake_5fset_5ft_2eproto}, {
      &scc_info_brakeModel_t_mf_5flsca_2fbrake_5fmodel_5ft_2eproto.base,}};

static void InitDefaultsscc_info_brakeSet_t_array_port_mf_5flsca_2fbrake_5fset_5ft_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_lsca::brake_set_t::_brakeSet_t_array_port_default_instance_;
    new (ptr) ::pb::mf_lsca::brake_set_t::brakeSet_t_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_lsca::brake_set_t::brakeSet_t_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_brakeSet_t_array_port_mf_5flsca_2fbrake_5fset_5ft_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_brakeSet_t_array_port_mf_5flsca_2fbrake_5fset_5ft_2eproto}, {
      &scc_info_brakeSet_t_mf_5flsca_2fbrake_5fset_5ft_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_mf_5flsca_2fbrake_5fset_5ft_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_mf_5flsca_2fbrake_5fset_5ft_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_mf_5flsca_2fbrake_5fset_5ft_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_mf_5flsca_2fbrake_5fset_5ft_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::mf_lsca::brake_set_t::brakeSet_t, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_lsca::brake_set_t::brakeSet_t, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_lsca::brake_set_t::brakeSet_t, emergencybraking_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_lsca::brake_set_t::brakeSet_t, comfortbraking_),
  0,
  1,
  PROTOBUF_FIELD_OFFSET(::pb::mf_lsca::brake_set_t::brakeSet_t_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_lsca::brake_set_t::brakeSet_t_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_lsca::brake_set_t::brakeSet_t_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::pb::mf_lsca::brake_set_t::brakeSet_t)},
  { 9, 15, sizeof(::pb::mf_lsca::brake_set_t::brakeSet_t_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_lsca::brake_set_t::_brakeSet_t_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_lsca::brake_set_t::_brakeSet_t_array_port_default_instance_),
};

const char descriptor_table_protodef_mf_5flsca_2fbrake_5fset_5ft_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\031mf_lsca/brake_set_t.proto\022\026pb.mf_lsca."
  "brake_set_t\032\033mf_lsca/brake_model_t.proto"
  "\"\220\001\n\nbrakeSet_t\022A\n\020emergencyBraking\030\306\002 \001"
  "(\0132&.pb.mf_lsca.brake_model_t.brakeModel"
  "_t\022\?\n\016comfortBraking\030\301\026 \001(\0132&.pb.mf_lsca"
  ".brake_model_t.brakeModel_t\"J\n\025brakeSet_"
  "t_array_port\0221\n\004data\030\271\005 \003(\0132\".pb.mf_lsca"
  ".brake_set_t.brakeSet_t"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_mf_5flsca_2fbrake_5fset_5ft_2eproto_deps[1] = {
  &::descriptor_table_mf_5flsca_2fbrake_5fmodel_5ft_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_mf_5flsca_2fbrake_5fset_5ft_2eproto_sccs[2] = {
  &scc_info_brakeSet_t_mf_5flsca_2fbrake_5fset_5ft_2eproto.base,
  &scc_info_brakeSet_t_array_port_mf_5flsca_2fbrake_5fset_5ft_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_mf_5flsca_2fbrake_5fset_5ft_2eproto_once;
static bool descriptor_table_mf_5flsca_2fbrake_5fset_5ft_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5flsca_2fbrake_5fset_5ft_2eproto = {
  &descriptor_table_mf_5flsca_2fbrake_5fset_5ft_2eproto_initialized, descriptor_table_protodef_mf_5flsca_2fbrake_5fset_5ft_2eproto, "mf_lsca/brake_set_t.proto", 303,
  &descriptor_table_mf_5flsca_2fbrake_5fset_5ft_2eproto_once, descriptor_table_mf_5flsca_2fbrake_5fset_5ft_2eproto_sccs, descriptor_table_mf_5flsca_2fbrake_5fset_5ft_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_mf_5flsca_2fbrake_5fset_5ft_2eproto::offsets,
  file_level_metadata_mf_5flsca_2fbrake_5fset_5ft_2eproto, 2, file_level_enum_descriptors_mf_5flsca_2fbrake_5fset_5ft_2eproto, file_level_service_descriptors_mf_5flsca_2fbrake_5fset_5ft_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_mf_5flsca_2fbrake_5fset_5ft_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_mf_5flsca_2fbrake_5fset_5ft_2eproto), true);
namespace pb {
namespace mf_lsca {
namespace brake_set_t {

// ===================================================================

void brakeSet_t::InitAsDefaultInstance() {
  ::pb::mf_lsca::brake_set_t::_brakeSet_t_default_instance_._instance.get_mutable()->emergencybraking_ = const_cast< ::pb::mf_lsca::brake_model_t::brakeModel_t*>(
      ::pb::mf_lsca::brake_model_t::brakeModel_t::internal_default_instance());
  ::pb::mf_lsca::brake_set_t::_brakeSet_t_default_instance_._instance.get_mutable()->comfortbraking_ = const_cast< ::pb::mf_lsca::brake_model_t::brakeModel_t*>(
      ::pb::mf_lsca::brake_model_t::brakeModel_t::internal_default_instance());
}
class brakeSet_t::_Internal {
 public:
  using HasBits = decltype(std::declval<brakeSet_t>()._has_bits_);
  static const ::pb::mf_lsca::brake_model_t::brakeModel_t& emergencybraking(const brakeSet_t* msg);
  static void set_has_emergencybraking(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::pb::mf_lsca::brake_model_t::brakeModel_t& comfortbraking(const brakeSet_t* msg);
  static void set_has_comfortbraking(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

const ::pb::mf_lsca::brake_model_t::brakeModel_t&
brakeSet_t::_Internal::emergencybraking(const brakeSet_t* msg) {
  return *msg->emergencybraking_;
}
const ::pb::mf_lsca::brake_model_t::brakeModel_t&
brakeSet_t::_Internal::comfortbraking(const brakeSet_t* msg) {
  return *msg->comfortbraking_;
}
void brakeSet_t::clear_emergencybraking() {
  if (emergencybraking_ != nullptr) emergencybraking_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void brakeSet_t::clear_comfortbraking() {
  if (comfortbraking_ != nullptr) comfortbraking_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
brakeSet_t::brakeSet_t()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_lsca.brake_set_t.brakeSet_t)
}
brakeSet_t::brakeSet_t(const brakeSet_t& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_emergencybraking()) {
    emergencybraking_ = new ::pb::mf_lsca::brake_model_t::brakeModel_t(*from.emergencybraking_);
  } else {
    emergencybraking_ = nullptr;
  }
  if (from._internal_has_comfortbraking()) {
    comfortbraking_ = new ::pb::mf_lsca::brake_model_t::brakeModel_t(*from.comfortbraking_);
  } else {
    comfortbraking_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:pb.mf_lsca.brake_set_t.brakeSet_t)
}

void brakeSet_t::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_brakeSet_t_mf_5flsca_2fbrake_5fset_5ft_2eproto.base);
  ::memset(&emergencybraking_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&comfortbraking_) -
      reinterpret_cast<char*>(&emergencybraking_)) + sizeof(comfortbraking_));
}

brakeSet_t::~brakeSet_t() {
  // @@protoc_insertion_point(destructor:pb.mf_lsca.brake_set_t.brakeSet_t)
  SharedDtor();
}

void brakeSet_t::SharedDtor() {
  if (this != internal_default_instance()) delete emergencybraking_;
  if (this != internal_default_instance()) delete comfortbraking_;
}

void brakeSet_t::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const brakeSet_t& brakeSet_t::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_brakeSet_t_mf_5flsca_2fbrake_5fset_5ft_2eproto.base);
  return *internal_default_instance();
}


void brakeSet_t::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_lsca.brake_set_t.brakeSet_t)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(emergencybraking_ != nullptr);
      emergencybraking_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(comfortbraking_ != nullptr);
      comfortbraking_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* brakeSet_t::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .pb.mf_lsca.brake_model_t.brakeModel_t emergencyBraking = 326;
      case 326:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 50)) {
          ptr = ctx->ParseMessage(_internal_mutable_emergencybraking(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.mf_lsca.brake_model_t.brakeModel_t comfortBraking = 2881;
      case 2881:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_comfortbraking(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* brakeSet_t::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_lsca.brake_set_t.brakeSet_t)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.mf_lsca.brake_model_t.brakeModel_t emergencyBraking = 326;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        326, _Internal::emergencybraking(this), target, stream);
  }

  // optional .pb.mf_lsca.brake_model_t.brakeModel_t comfortBraking = 2881;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2881, _Internal::comfortbraking(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_lsca.brake_set_t.brakeSet_t)
  return target;
}

size_t brakeSet_t::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_lsca.brake_set_t.brakeSet_t)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // optional .pb.mf_lsca.brake_model_t.brakeModel_t emergencyBraking = 326;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *emergencybraking_);
    }

    // optional .pb.mf_lsca.brake_model_t.brakeModel_t comfortBraking = 2881;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *comfortbraking_);
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

void brakeSet_t::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_lsca.brake_set_t.brakeSet_t)
  GOOGLE_DCHECK_NE(&from, this);
  const brakeSet_t* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<brakeSet_t>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_lsca.brake_set_t.brakeSet_t)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_lsca.brake_set_t.brakeSet_t)
    MergeFrom(*source);
  }
}

void brakeSet_t::MergeFrom(const brakeSet_t& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_lsca.brake_set_t.brakeSet_t)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_emergencybraking()->::pb::mf_lsca::brake_model_t::brakeModel_t::MergeFrom(from._internal_emergencybraking());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_comfortbraking()->::pb::mf_lsca::brake_model_t::brakeModel_t::MergeFrom(from._internal_comfortbraking());
    }
  }
}

void brakeSet_t::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_lsca.brake_set_t.brakeSet_t)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void brakeSet_t::CopyFrom(const brakeSet_t& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_lsca.brake_set_t.brakeSet_t)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool brakeSet_t::IsInitialized() const {
  return true;
}

void brakeSet_t::InternalSwap(brakeSet_t* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(emergencybraking_, other->emergencybraking_);
  swap(comfortbraking_, other->comfortbraking_);
}

::PROTOBUF_NAMESPACE_ID::Metadata brakeSet_t::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void brakeSet_t_array_port::InitAsDefaultInstance() {
}
class brakeSet_t_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<brakeSet_t_array_port>()._has_bits_);
};

brakeSet_t_array_port::brakeSet_t_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_lsca.brake_set_t.brakeSet_t_array_port)
}
brakeSet_t_array_port::brakeSet_t_array_port(const brakeSet_t_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.mf_lsca.brake_set_t.brakeSet_t_array_port)
}

void brakeSet_t_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_brakeSet_t_array_port_mf_5flsca_2fbrake_5fset_5ft_2eproto.base);
}

brakeSet_t_array_port::~brakeSet_t_array_port() {
  // @@protoc_insertion_point(destructor:pb.mf_lsca.brake_set_t.brakeSet_t_array_port)
  SharedDtor();
}

void brakeSet_t_array_port::SharedDtor() {
}

void brakeSet_t_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const brakeSet_t_array_port& brakeSet_t_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_brakeSet_t_array_port_mf_5flsca_2fbrake_5fset_5ft_2eproto.base);
  return *internal_default_instance();
}


void brakeSet_t_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_lsca.brake_set_t.brakeSet_t_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* brakeSet_t_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.mf_lsca.brake_set_t.brakeSet_t data = 697;
      case 697:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 202)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<5578>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* brakeSet_t_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_lsca.brake_set_t.brakeSet_t_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.mf_lsca.brake_set_t.brakeSet_t data = 697;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(697, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_lsca.brake_set_t.brakeSet_t_array_port)
  return target;
}

size_t brakeSet_t_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_lsca.brake_set_t.brakeSet_t_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.mf_lsca.brake_set_t.brakeSet_t data = 697;
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

void brakeSet_t_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_lsca.brake_set_t.brakeSet_t_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const brakeSet_t_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<brakeSet_t_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_lsca.brake_set_t.brakeSet_t_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_lsca.brake_set_t.brakeSet_t_array_port)
    MergeFrom(*source);
  }
}

void brakeSet_t_array_port::MergeFrom(const brakeSet_t_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_lsca.brake_set_t.brakeSet_t_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void brakeSet_t_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_lsca.brake_set_t.brakeSet_t_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void brakeSet_t_array_port::CopyFrom(const brakeSet_t_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_lsca.brake_set_t.brakeSet_t_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool brakeSet_t_array_port::IsInitialized() const {
  return true;
}

void brakeSet_t_array_port::InternalSwap(brakeSet_t_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata brakeSet_t_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace brake_set_t
}  // namespace mf_lsca
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::mf_lsca::brake_set_t::brakeSet_t* Arena::CreateMaybeMessage< ::pb::mf_lsca::brake_set_t::brakeSet_t >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_lsca::brake_set_t::brakeSet_t >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::mf_lsca::brake_set_t::brakeSet_t_array_port* Arena::CreateMaybeMessage< ::pb::mf_lsca::brake_set_t::brakeSet_t_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_lsca::brake_set_t::brakeSet_t_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
