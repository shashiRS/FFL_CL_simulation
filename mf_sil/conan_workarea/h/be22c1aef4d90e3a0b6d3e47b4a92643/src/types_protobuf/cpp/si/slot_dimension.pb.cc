// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/slot_dimension.proto

#include "si/slot_dimension.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_si_2fslot_5fdimension_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SlotDimension_si_2fslot_5fdimension_2eproto;
namespace pb {
namespace si {
namespace slot_dimension {
class SlotDimensionDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<SlotDimension> _instance;
} _SlotDimension_default_instance_;
class SlotDimension_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<SlotDimension_array_port> _instance;
} _SlotDimension_array_port_default_instance_;
}  // namespace slot_dimension
}  // namespace si
}  // namespace pb
static void InitDefaultsscc_info_SlotDimension_si_2fslot_5fdimension_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::slot_dimension::_SlotDimension_default_instance_;
    new (ptr) ::pb::si::slot_dimension::SlotDimension();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::slot_dimension::SlotDimension::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SlotDimension_si_2fslot_5fdimension_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_SlotDimension_si_2fslot_5fdimension_2eproto}, {}};

static void InitDefaultsscc_info_SlotDimension_array_port_si_2fslot_5fdimension_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::slot_dimension::_SlotDimension_array_port_default_instance_;
    new (ptr) ::pb::si::slot_dimension::SlotDimension_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::slot_dimension::SlotDimension_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_SlotDimension_array_port_si_2fslot_5fdimension_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_SlotDimension_array_port_si_2fslot_5fdimension_2eproto}, {
      &scc_info_SlotDimension_si_2fslot_5fdimension_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_si_2fslot_5fdimension_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_si_2fslot_5fdimension_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_si_2fslot_5fdimension_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_si_2fslot_5fdimension_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::si::slot_dimension::SlotDimension, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::slot_dimension::SlotDimension, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::slot_dimension::SlotDimension, min_m_),
  PROTOBUF_FIELD_OFFSET(::pb::si::slot_dimension::SlotDimension, typical_m_),
  PROTOBUF_FIELD_OFFSET(::pb::si::slot_dimension::SlotDimension, max_m_),
  1,
  2,
  0,
  PROTOBUF_FIELD_OFFSET(::pb::si::slot_dimension::SlotDimension_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::slot_dimension::SlotDimension_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::slot_dimension::SlotDimension_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::pb::si::slot_dimension::SlotDimension)},
  { 11, 17, sizeof(::pb::si::slot_dimension::SlotDimension_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::slot_dimension::_SlotDimension_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::slot_dimension::_SlotDimension_array_port_default_instance_),
};

const char descriptor_table_protodef_si_2fslot_5fdimension_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\027si/slot_dimension.proto\022\024pb.si.slot_di"
  "mension\"C\n\rSlotDimension\022\016\n\005min_m\030\243\002 \001(\002"
  "\022\022\n\ttypical_m\030\303\t \001(\002\022\016\n\005max_m\030\243\n \001(\002\"N\n\030"
  "SlotDimension_array_port\0222\n\004data\030\372\025 \003(\0132"
  "#.pb.si.slot_dimension.SlotDimension"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_si_2fslot_5fdimension_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_si_2fslot_5fdimension_2eproto_sccs[2] = {
  &scc_info_SlotDimension_si_2fslot_5fdimension_2eproto.base,
  &scc_info_SlotDimension_array_port_si_2fslot_5fdimension_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_si_2fslot_5fdimension_2eproto_once;
static bool descriptor_table_si_2fslot_5fdimension_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fslot_5fdimension_2eproto = {
  &descriptor_table_si_2fslot_5fdimension_2eproto_initialized, descriptor_table_protodef_si_2fslot_5fdimension_2eproto, "si/slot_dimension.proto", 196,
  &descriptor_table_si_2fslot_5fdimension_2eproto_once, descriptor_table_si_2fslot_5fdimension_2eproto_sccs, descriptor_table_si_2fslot_5fdimension_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_si_2fslot_5fdimension_2eproto::offsets,
  file_level_metadata_si_2fslot_5fdimension_2eproto, 2, file_level_enum_descriptors_si_2fslot_5fdimension_2eproto, file_level_service_descriptors_si_2fslot_5fdimension_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_si_2fslot_5fdimension_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_si_2fslot_5fdimension_2eproto), true);
namespace pb {
namespace si {
namespace slot_dimension {

// ===================================================================

void SlotDimension::InitAsDefaultInstance() {
}
class SlotDimension::_Internal {
 public:
  using HasBits = decltype(std::declval<SlotDimension>()._has_bits_);
  static void set_has_min_m(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_typical_m(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_max_m(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

SlotDimension::SlotDimension()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.slot_dimension.SlotDimension)
}
SlotDimension::SlotDimension(const SlotDimension& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&max_m_, &from.max_m_,
    static_cast<size_t>(reinterpret_cast<char*>(&typical_m_) -
    reinterpret_cast<char*>(&max_m_)) + sizeof(typical_m_));
  // @@protoc_insertion_point(copy_constructor:pb.si.slot_dimension.SlotDimension)
}

void SlotDimension::SharedCtor() {
  ::memset(&max_m_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&typical_m_) -
      reinterpret_cast<char*>(&max_m_)) + sizeof(typical_m_));
}

SlotDimension::~SlotDimension() {
  // @@protoc_insertion_point(destructor:pb.si.slot_dimension.SlotDimension)
  SharedDtor();
}

void SlotDimension::SharedDtor() {
}

void SlotDimension::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const SlotDimension& SlotDimension::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_SlotDimension_si_2fslot_5fdimension_2eproto.base);
  return *internal_default_instance();
}


void SlotDimension::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.slot_dimension.SlotDimension)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    ::memset(&max_m_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&typical_m_) -
        reinterpret_cast<char*>(&max_m_)) + sizeof(typical_m_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* SlotDimension::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional float min_m = 291;
      case 291:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 29)) {
          _Internal::set_has_min_m(&has_bits);
          min_m_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional float typical_m = 1219;
      case 1219:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 29)) {
          _Internal::set_has_typical_m(&has_bits);
          typical_m_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional float max_m = 1315;
      case 1315:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 29)) {
          _Internal::set_has_max_m(&has_bits);
          max_m_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* SlotDimension::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.slot_dimension.SlotDimension)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional float min_m = 291;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(291, this->_internal_min_m(), target);
  }

  // optional float typical_m = 1219;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(1219, this->_internal_typical_m(), target);
  }

  // optional float max_m = 1315;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(1315, this->_internal_max_m(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.slot_dimension.SlotDimension)
  return target;
}

size_t SlotDimension::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.slot_dimension.SlotDimension)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional float max_m = 1315;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 + 4;
    }

    // optional float min_m = 291;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 + 4;
    }

    // optional float typical_m = 1219;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 + 4;
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

void SlotDimension::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.slot_dimension.SlotDimension)
  GOOGLE_DCHECK_NE(&from, this);
  const SlotDimension* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<SlotDimension>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.slot_dimension.SlotDimension)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.slot_dimension.SlotDimension)
    MergeFrom(*source);
  }
}

void SlotDimension::MergeFrom(const SlotDimension& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.slot_dimension.SlotDimension)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      max_m_ = from.max_m_;
    }
    if (cached_has_bits & 0x00000002u) {
      min_m_ = from.min_m_;
    }
    if (cached_has_bits & 0x00000004u) {
      typical_m_ = from.typical_m_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void SlotDimension::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.slot_dimension.SlotDimension)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SlotDimension::CopyFrom(const SlotDimension& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.slot_dimension.SlotDimension)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SlotDimension::IsInitialized() const {
  return true;
}

void SlotDimension::InternalSwap(SlotDimension* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(max_m_, other->max_m_);
  swap(min_m_, other->min_m_);
  swap(typical_m_, other->typical_m_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SlotDimension::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void SlotDimension_array_port::InitAsDefaultInstance() {
}
class SlotDimension_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<SlotDimension_array_port>()._has_bits_);
};

SlotDimension_array_port::SlotDimension_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.slot_dimension.SlotDimension_array_port)
}
SlotDimension_array_port::SlotDimension_array_port(const SlotDimension_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.si.slot_dimension.SlotDimension_array_port)
}

void SlotDimension_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_SlotDimension_array_port_si_2fslot_5fdimension_2eproto.base);
}

SlotDimension_array_port::~SlotDimension_array_port() {
  // @@protoc_insertion_point(destructor:pb.si.slot_dimension.SlotDimension_array_port)
  SharedDtor();
}

void SlotDimension_array_port::SharedDtor() {
}

void SlotDimension_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const SlotDimension_array_port& SlotDimension_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_SlotDimension_array_port_si_2fslot_5fdimension_2eproto.base);
  return *internal_default_instance();
}


void SlotDimension_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.slot_dimension.SlotDimension_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* SlotDimension_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.si.slot_dimension.SlotDimension data = 2810;
      case 2810:
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

::PROTOBUF_NAMESPACE_ID::uint8* SlotDimension_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.slot_dimension.SlotDimension_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.si.slot_dimension.SlotDimension data = 2810;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2810, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.slot_dimension.SlotDimension_array_port)
  return target;
}

size_t SlotDimension_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.slot_dimension.SlotDimension_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.si.slot_dimension.SlotDimension data = 2810;
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

void SlotDimension_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.slot_dimension.SlotDimension_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const SlotDimension_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<SlotDimension_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.slot_dimension.SlotDimension_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.slot_dimension.SlotDimension_array_port)
    MergeFrom(*source);
  }
}

void SlotDimension_array_port::MergeFrom(const SlotDimension_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.slot_dimension.SlotDimension_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void SlotDimension_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.slot_dimension.SlotDimension_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SlotDimension_array_port::CopyFrom(const SlotDimension_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.slot_dimension.SlotDimension_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SlotDimension_array_port::IsInitialized() const {
  return true;
}

void SlotDimension_array_port::InternalSwap(SlotDimension_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SlotDimension_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace slot_dimension
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::si::slot_dimension::SlotDimension* Arena::CreateMaybeMessage< ::pb::si::slot_dimension::SlotDimension >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::slot_dimension::SlotDimension >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::si::slot_dimension::SlotDimension_array_port* Arena::CreateMaybeMessage< ::pb::si::slot_dimension::SlotDimension_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::slot_dimension::SlotDimension_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
