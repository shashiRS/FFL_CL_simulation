// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/diagnostic_event_serializable.proto

#include "si/diagnostic_event_serializable.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_si_2fdiagnostic_5fevent_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_DiagnosticEvent_si_2fdiagnostic_5fevent_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_si_2fdiagnostic_5fevent_5fserializable_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_DiagnosticEventSerializable_si_2fdiagnostic_5fevent_5fserializable_2eproto;
namespace pb {
namespace si {
namespace diagnostic_event_serializable {
class DiagnosticEventSerializableDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<DiagnosticEventSerializable> _instance;
} _DiagnosticEventSerializable_default_instance_;
class DiagnosticEventSerializable_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<DiagnosticEventSerializable_array_port> _instance;
} _DiagnosticEventSerializable_array_port_default_instance_;
}  // namespace diagnostic_event_serializable
}  // namespace si
}  // namespace pb
static void InitDefaultsscc_info_DiagnosticEventSerializable_si_2fdiagnostic_5fevent_5fserializable_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::diagnostic_event_serializable::_DiagnosticEventSerializable_default_instance_;
    new (ptr) ::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_DiagnosticEventSerializable_si_2fdiagnostic_5fevent_5fserializable_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_DiagnosticEventSerializable_si_2fdiagnostic_5fevent_5fserializable_2eproto}, {
      &scc_info_DiagnosticEvent_si_2fdiagnostic_5fevent_2eproto.base,}};

static void InitDefaultsscc_info_DiagnosticEventSerializable_array_port_si_2fdiagnostic_5fevent_5fserializable_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::diagnostic_event_serializable::_DiagnosticEventSerializable_array_port_default_instance_;
    new (ptr) ::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_DiagnosticEventSerializable_array_port_si_2fdiagnostic_5fevent_5fserializable_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_DiagnosticEventSerializable_array_port_si_2fdiagnostic_5fevent_5fserializable_2eproto}, {
      &scc_info_DiagnosticEventSerializable_si_2fdiagnostic_5fevent_5fserializable_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_si_2fdiagnostic_5fevent_5fserializable_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_si_2fdiagnostic_5fevent_5fserializable_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_si_2fdiagnostic_5fevent_5fserializable_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_si_2fdiagnostic_5fevent_5fserializable_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable, actualsize_),
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable, array_),
  0,
  ~0u,
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable)},
  { 9, 15, sizeof(::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::diagnostic_event_serializable::_DiagnosticEventSerializable_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::diagnostic_event_serializable::_DiagnosticEventSerializable_array_port_default_instance_),
};

const char descriptor_table_protodef_si_2fdiagnostic_5fevent_5fserializable_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n&si/diagnostic_event_serializable.proto"
  "\022#pb.si.diagnostic_event_serializable\032\031s"
  "i/diagnostic_event.proto\"k\n\033DiagnosticEv"
  "entSerializable\022\023\n\nactualSize\030\301\027 \001(\r\0227\n\005"
  "array\030\304\021 \003(\0132\'.pb.si.diagnostic_event.Di"
  "agnosticEvent\"y\n&DiagnosticEventSerializ"
  "able_array_port\022O\n\004data\030\346\001 \003(\0132@.pb.si.d"
  "iagnostic_event_serializable.DiagnosticE"
  "ventSerializable"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_si_2fdiagnostic_5fevent_5fserializable_2eproto_deps[1] = {
  &::descriptor_table_si_2fdiagnostic_5fevent_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_si_2fdiagnostic_5fevent_5fserializable_2eproto_sccs[2] = {
  &scc_info_DiagnosticEventSerializable_si_2fdiagnostic_5fevent_5fserializable_2eproto.base,
  &scc_info_DiagnosticEventSerializable_array_port_si_2fdiagnostic_5fevent_5fserializable_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_si_2fdiagnostic_5fevent_5fserializable_2eproto_once;
static bool descriptor_table_si_2fdiagnostic_5fevent_5fserializable_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fdiagnostic_5fevent_5fserializable_2eproto = {
  &descriptor_table_si_2fdiagnostic_5fevent_5fserializable_2eproto_initialized, descriptor_table_protodef_si_2fdiagnostic_5fevent_5fserializable_2eproto, "si/diagnostic_event_serializable.proto", 336,
  &descriptor_table_si_2fdiagnostic_5fevent_5fserializable_2eproto_once, descriptor_table_si_2fdiagnostic_5fevent_5fserializable_2eproto_sccs, descriptor_table_si_2fdiagnostic_5fevent_5fserializable_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_si_2fdiagnostic_5fevent_5fserializable_2eproto::offsets,
  file_level_metadata_si_2fdiagnostic_5fevent_5fserializable_2eproto, 2, file_level_enum_descriptors_si_2fdiagnostic_5fevent_5fserializable_2eproto, file_level_service_descriptors_si_2fdiagnostic_5fevent_5fserializable_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_si_2fdiagnostic_5fevent_5fserializable_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_si_2fdiagnostic_5fevent_5fserializable_2eproto), true);
namespace pb {
namespace si {
namespace diagnostic_event_serializable {

// ===================================================================

void DiagnosticEventSerializable::InitAsDefaultInstance() {
}
class DiagnosticEventSerializable::_Internal {
 public:
  using HasBits = decltype(std::declval<DiagnosticEventSerializable>()._has_bits_);
  static void set_has_actualsize(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

void DiagnosticEventSerializable::clear_array() {
  array_.Clear();
}
DiagnosticEventSerializable::DiagnosticEventSerializable()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable)
}
DiagnosticEventSerializable::DiagnosticEventSerializable(const DiagnosticEventSerializable& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      array_(from.array_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  actualsize_ = from.actualsize_;
  // @@protoc_insertion_point(copy_constructor:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable)
}

void DiagnosticEventSerializable::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_DiagnosticEventSerializable_si_2fdiagnostic_5fevent_5fserializable_2eproto.base);
  actualsize_ = 0u;
}

DiagnosticEventSerializable::~DiagnosticEventSerializable() {
  // @@protoc_insertion_point(destructor:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable)
  SharedDtor();
}

void DiagnosticEventSerializable::SharedDtor() {
}

void DiagnosticEventSerializable::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const DiagnosticEventSerializable& DiagnosticEventSerializable::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_DiagnosticEventSerializable_si_2fdiagnostic_5fevent_5fserializable_2eproto.base);
  return *internal_default_instance();
}


void DiagnosticEventSerializable::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  array_.Clear();
  actualsize_ = 0u;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* DiagnosticEventSerializable::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.si.diagnostic_event.DiagnosticEvent array = 2244;
      case 2244:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          ptr = ctx->ParseMessage(_internal_add_array(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 actualSize = 3009;
      case 3009:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_actualsize(&has_bits);
          actualsize_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* DiagnosticEventSerializable::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.si.diagnostic_event.DiagnosticEvent array = 2244;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_array_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2244, this->_internal_array(i), target, stream);
  }

  cached_has_bits = _has_bits_[0];
  // optional uint32 actualSize = 3009;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(3009, this->_internal_actualsize(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable)
  return target;
}

size_t DiagnosticEventSerializable::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.si.diagnostic_event.DiagnosticEvent array = 2244;
  total_size += 3UL * this->_internal_array_size();
  for (const auto& msg : this->array_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
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

void DiagnosticEventSerializable::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable)
  GOOGLE_DCHECK_NE(&from, this);
  const DiagnosticEventSerializable* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<DiagnosticEventSerializable>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable)
    MergeFrom(*source);
  }
}

void DiagnosticEventSerializable::MergeFrom(const DiagnosticEventSerializable& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  array_.MergeFrom(from.array_);
  if (from._internal_has_actualsize()) {
    _internal_set_actualsize(from._internal_actualsize());
  }
}

void DiagnosticEventSerializable::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void DiagnosticEventSerializable::CopyFrom(const DiagnosticEventSerializable& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DiagnosticEventSerializable::IsInitialized() const {
  return true;
}

void DiagnosticEventSerializable::InternalSwap(DiagnosticEventSerializable* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  array_.InternalSwap(&other->array_);
  swap(actualsize_, other->actualsize_);
}

::PROTOBUF_NAMESPACE_ID::Metadata DiagnosticEventSerializable::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void DiagnosticEventSerializable_array_port::InitAsDefaultInstance() {
}
class DiagnosticEventSerializable_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<DiagnosticEventSerializable_array_port>()._has_bits_);
};

DiagnosticEventSerializable_array_port::DiagnosticEventSerializable_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable_array_port)
}
DiagnosticEventSerializable_array_port::DiagnosticEventSerializable_array_port(const DiagnosticEventSerializable_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable_array_port)
}

void DiagnosticEventSerializable_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_DiagnosticEventSerializable_array_port_si_2fdiagnostic_5fevent_5fserializable_2eproto.base);
}

DiagnosticEventSerializable_array_port::~DiagnosticEventSerializable_array_port() {
  // @@protoc_insertion_point(destructor:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable_array_port)
  SharedDtor();
}

void DiagnosticEventSerializable_array_port::SharedDtor() {
}

void DiagnosticEventSerializable_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const DiagnosticEventSerializable_array_port& DiagnosticEventSerializable_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_DiagnosticEventSerializable_array_port_si_2fdiagnostic_5fevent_5fserializable_2eproto.base);
  return *internal_default_instance();
}


void DiagnosticEventSerializable_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* DiagnosticEventSerializable_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.si.diagnostic_event_serializable.DiagnosticEventSerializable data = 230;
      case 230:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 50)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<1842>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* DiagnosticEventSerializable_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.si.diagnostic_event_serializable.DiagnosticEventSerializable data = 230;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(230, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable_array_port)
  return target;
}

size_t DiagnosticEventSerializable_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.si.diagnostic_event_serializable.DiagnosticEventSerializable data = 230;
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

void DiagnosticEventSerializable_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const DiagnosticEventSerializable_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<DiagnosticEventSerializable_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable_array_port)
    MergeFrom(*source);
  }
}

void DiagnosticEventSerializable_array_port::MergeFrom(const DiagnosticEventSerializable_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void DiagnosticEventSerializable_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void DiagnosticEventSerializable_array_port::CopyFrom(const DiagnosticEventSerializable_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.diagnostic_event_serializable.DiagnosticEventSerializable_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DiagnosticEventSerializable_array_port::IsInitialized() const {
  return true;
}

void DiagnosticEventSerializable_array_port::InternalSwap(DiagnosticEventSerializable_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata DiagnosticEventSerializable_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace diagnostic_event_serializable
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable* Arena::CreateMaybeMessage< ::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable_array_port* Arena::CreateMaybeMessage< ::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::diagnostic_event_serializable::DiagnosticEventSerializable_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
