// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/uint16_serializable.proto

#include "si/uint16_serializable.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_si_2fuint16_5fserializable_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Uint16Serializable_si_2fuint16_5fserializable_2eproto;
namespace pb {
namespace si {
namespace uint16_serializable {
class Uint16SerializableDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Uint16Serializable> _instance;
} _Uint16Serializable_default_instance_;
class Uint16Serializable_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Uint16Serializable_array_port> _instance;
} _Uint16Serializable_array_port_default_instance_;
}  // namespace uint16_serializable
}  // namespace si
}  // namespace pb
static void InitDefaultsscc_info_Uint16Serializable_si_2fuint16_5fserializable_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::uint16_serializable::_Uint16Serializable_default_instance_;
    new (ptr) ::pb::si::uint16_serializable::Uint16Serializable();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::uint16_serializable::Uint16Serializable::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Uint16Serializable_si_2fuint16_5fserializable_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_Uint16Serializable_si_2fuint16_5fserializable_2eproto}, {}};

static void InitDefaultsscc_info_Uint16Serializable_array_port_si_2fuint16_5fserializable_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::uint16_serializable::_Uint16Serializable_array_port_default_instance_;
    new (ptr) ::pb::si::uint16_serializable::Uint16Serializable_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::uint16_serializable::Uint16Serializable_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_Uint16Serializable_array_port_si_2fuint16_5fserializable_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_Uint16Serializable_array_port_si_2fuint16_5fserializable_2eproto}, {
      &scc_info_Uint16Serializable_si_2fuint16_5fserializable_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_si_2fuint16_5fserializable_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_si_2fuint16_5fserializable_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_si_2fuint16_5fserializable_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_si_2fuint16_5fserializable_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::si::uint16_serializable::Uint16Serializable, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::uint16_serializable::Uint16Serializable, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::uint16_serializable::Uint16Serializable, actualsize_),
  PROTOBUF_FIELD_OFFSET(::pb::si::uint16_serializable::Uint16Serializable, array_),
  0,
  ~0u,
  PROTOBUF_FIELD_OFFSET(::pb::si::uint16_serializable::Uint16Serializable_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::uint16_serializable::Uint16Serializable_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::uint16_serializable::Uint16Serializable_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::pb::si::uint16_serializable::Uint16Serializable)},
  { 9, 15, sizeof(::pb::si::uint16_serializable::Uint16Serializable_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::uint16_serializable::_Uint16Serializable_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::uint16_serializable::_Uint16Serializable_array_port_default_instance_),
};

const char descriptor_table_protodef_si_2fuint16_5fserializable_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\034si/uint16_serializable.proto\022\031pb.si.ui"
  "nt16_serializable\"9\n\022Uint16Serializable\022"
  "\023\n\nactualSize\030\301\027 \001(\r\022\016\n\005array\030\267\034 \003(\r\"]\n\035"
  "Uint16Serializable_array_port\022<\n\004data\030\322\034"
  " \003(\0132-.pb.si.uint16_serializable.Uint16S"
  "erializable"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_si_2fuint16_5fserializable_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_si_2fuint16_5fserializable_2eproto_sccs[2] = {
  &scc_info_Uint16Serializable_si_2fuint16_5fserializable_2eproto.base,
  &scc_info_Uint16Serializable_array_port_si_2fuint16_5fserializable_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_si_2fuint16_5fserializable_2eproto_once;
static bool descriptor_table_si_2fuint16_5fserializable_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fuint16_5fserializable_2eproto = {
  &descriptor_table_si_2fuint16_5fserializable_2eproto_initialized, descriptor_table_protodef_si_2fuint16_5fserializable_2eproto, "si/uint16_serializable.proto", 211,
  &descriptor_table_si_2fuint16_5fserializable_2eproto_once, descriptor_table_si_2fuint16_5fserializable_2eproto_sccs, descriptor_table_si_2fuint16_5fserializable_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_si_2fuint16_5fserializable_2eproto::offsets,
  file_level_metadata_si_2fuint16_5fserializable_2eproto, 2, file_level_enum_descriptors_si_2fuint16_5fserializable_2eproto, file_level_service_descriptors_si_2fuint16_5fserializable_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_si_2fuint16_5fserializable_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_si_2fuint16_5fserializable_2eproto), true);
namespace pb {
namespace si {
namespace uint16_serializable {

// ===================================================================

void Uint16Serializable::InitAsDefaultInstance() {
}
class Uint16Serializable::_Internal {
 public:
  using HasBits = decltype(std::declval<Uint16Serializable>()._has_bits_);
  static void set_has_actualsize(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

Uint16Serializable::Uint16Serializable()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.uint16_serializable.Uint16Serializable)
}
Uint16Serializable::Uint16Serializable(const Uint16Serializable& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      array_(from.array_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  actualsize_ = from.actualsize_;
  // @@protoc_insertion_point(copy_constructor:pb.si.uint16_serializable.Uint16Serializable)
}

void Uint16Serializable::SharedCtor() {
  actualsize_ = 0u;
}

Uint16Serializable::~Uint16Serializable() {
  // @@protoc_insertion_point(destructor:pb.si.uint16_serializable.Uint16Serializable)
  SharedDtor();
}

void Uint16Serializable::SharedDtor() {
}

void Uint16Serializable::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Uint16Serializable& Uint16Serializable::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Uint16Serializable_si_2fuint16_5fserializable_2eproto.base);
  return *internal_default_instance();
}


void Uint16Serializable::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.uint16_serializable.Uint16Serializable)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  array_.Clear();
  actualsize_ = 0u;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* Uint16Serializable::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional uint32 actualSize = 3009;
      case 3009:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_actualsize(&has_bits);
          actualsize_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated uint32 array = 3639;
      case 3639:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 184)) {
          _internal_add_array(::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr));
          CHK_(ptr);
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 186) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedUInt32Parser(_internal_mutable_array(), ptr, ctx);
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

::PROTOBUF_NAMESPACE_ID::uint8* Uint16Serializable::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.uint16_serializable.Uint16Serializable)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional uint32 actualSize = 3009;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(3009, this->_internal_actualsize(), target);
  }

  // repeated uint32 array = 3639;
  for (int i = 0, n = this->_internal_array_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(3639, this->_internal_array(i), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.uint16_serializable.Uint16Serializable)
  return target;
}

size_t Uint16Serializable::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.uint16_serializable.Uint16Serializable)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated uint32 array = 3639;
  {
    size_t data_size = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      UInt32Size(this->array_);
    total_size += 3 *
                  ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(this->_internal_array_size());
    total_size += data_size;
  }

  // optional uint32 actualSize = 3009;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 3 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_actualsize());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Uint16Serializable::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.uint16_serializable.Uint16Serializable)
  GOOGLE_DCHECK_NE(&from, this);
  const Uint16Serializable* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Uint16Serializable>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.uint16_serializable.Uint16Serializable)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.uint16_serializable.Uint16Serializable)
    MergeFrom(*source);
  }
}

void Uint16Serializable::MergeFrom(const Uint16Serializable& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.uint16_serializable.Uint16Serializable)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  array_.MergeFrom(from.array_);
  if (from._internal_has_actualsize()) {
    _internal_set_actualsize(from._internal_actualsize());
  }
}

void Uint16Serializable::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.uint16_serializable.Uint16Serializable)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Uint16Serializable::CopyFrom(const Uint16Serializable& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.uint16_serializable.Uint16Serializable)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Uint16Serializable::IsInitialized() const {
  return true;
}

void Uint16Serializable::InternalSwap(Uint16Serializable* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  array_.InternalSwap(&other->array_);
  swap(actualsize_, other->actualsize_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Uint16Serializable::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void Uint16Serializable_array_port::InitAsDefaultInstance() {
}
class Uint16Serializable_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<Uint16Serializable_array_port>()._has_bits_);
};

Uint16Serializable_array_port::Uint16Serializable_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.uint16_serializable.Uint16Serializable_array_port)
}
Uint16Serializable_array_port::Uint16Serializable_array_port(const Uint16Serializable_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.si.uint16_serializable.Uint16Serializable_array_port)
}

void Uint16Serializable_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_Uint16Serializable_array_port_si_2fuint16_5fserializable_2eproto.base);
}

Uint16Serializable_array_port::~Uint16Serializable_array_port() {
  // @@protoc_insertion_point(destructor:pb.si.uint16_serializable.Uint16Serializable_array_port)
  SharedDtor();
}

void Uint16Serializable_array_port::SharedDtor() {
}

void Uint16Serializable_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Uint16Serializable_array_port& Uint16Serializable_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Uint16Serializable_array_port_si_2fuint16_5fserializable_2eproto.base);
  return *internal_default_instance();
}


void Uint16Serializable_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.uint16_serializable.Uint16Serializable_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* Uint16Serializable_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.si.uint16_serializable.Uint16Serializable data = 3666;
      case 3666:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 146)) {
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

::PROTOBUF_NAMESPACE_ID::uint8* Uint16Serializable_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.uint16_serializable.Uint16Serializable_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.si.uint16_serializable.Uint16Serializable data = 3666;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3666, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.uint16_serializable.Uint16Serializable_array_port)
  return target;
}

size_t Uint16Serializable_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.uint16_serializable.Uint16Serializable_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.si.uint16_serializable.Uint16Serializable data = 3666;
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

void Uint16Serializable_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.uint16_serializable.Uint16Serializable_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const Uint16Serializable_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Uint16Serializable_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.uint16_serializable.Uint16Serializable_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.uint16_serializable.Uint16Serializable_array_port)
    MergeFrom(*source);
  }
}

void Uint16Serializable_array_port::MergeFrom(const Uint16Serializable_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.uint16_serializable.Uint16Serializable_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void Uint16Serializable_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.uint16_serializable.Uint16Serializable_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Uint16Serializable_array_port::CopyFrom(const Uint16Serializable_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.uint16_serializable.Uint16Serializable_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Uint16Serializable_array_port::IsInitialized() const {
  return true;
}

void Uint16Serializable_array_port::InternalSwap(Uint16Serializable_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Uint16Serializable_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace uint16_serializable
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::si::uint16_serializable::Uint16Serializable* Arena::CreateMaybeMessage< ::pb::si::uint16_serializable::Uint16Serializable >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::uint16_serializable::Uint16Serializable >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::si::uint16_serializable::Uint16Serializable_array_port* Arena::CreateMaybeMessage< ::pb::si::uint16_serializable::Uint16Serializable_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::uint16_serializable::Uint16Serializable_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
