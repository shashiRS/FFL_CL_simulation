// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_em/us_em_point.proto

#include "us_em/us_em_point.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_us_5fem_2fus_5fem_5fpoint_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_UsEmPoint_us_5fem_2fus_5fem_5fpoint_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_us_5fem_2fus_5fem_5fposition_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_UsEmPosition_us_5fem_2fus_5fem_5fposition_2eproto;
namespace pb {
namespace us_em {
namespace us_em_point {
class UsEmPointDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<UsEmPoint> _instance;
} _UsEmPoint_default_instance_;
class UsEmPoint_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<UsEmPoint_array_port> _instance;
} _UsEmPoint_array_port_default_instance_;
}  // namespace us_em_point
}  // namespace us_em
}  // namespace pb
static void InitDefaultsscc_info_UsEmPoint_us_5fem_2fus_5fem_5fpoint_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::us_em::us_em_point::_UsEmPoint_default_instance_;
    new (ptr) ::pb::us_em::us_em_point::UsEmPoint();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::us_em::us_em_point::UsEmPoint::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_UsEmPoint_us_5fem_2fus_5fem_5fpoint_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_UsEmPoint_us_5fem_2fus_5fem_5fpoint_2eproto}, {
      &scc_info_UsEmPosition_us_5fem_2fus_5fem_5fposition_2eproto.base,}};

static void InitDefaultsscc_info_UsEmPoint_array_port_us_5fem_2fus_5fem_5fpoint_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::us_em::us_em_point::_UsEmPoint_array_port_default_instance_;
    new (ptr) ::pb::us_em::us_em_point::UsEmPoint_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::us_em::us_em_point::UsEmPoint_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_UsEmPoint_array_port_us_5fem_2fus_5fem_5fpoint_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_UsEmPoint_array_port_us_5fem_2fus_5fem_5fpoint_2eproto}, {
      &scc_info_UsEmPoint_us_5fem_2fus_5fem_5fpoint_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_us_5fem_2fus_5fem_5fpoint_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_us_5fem_2fus_5fem_5fpoint_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_us_5fem_2fus_5fem_5fpoint_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_us_5fem_2fus_5fem_5fpoint_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::us_em::us_em_point::UsEmPoint, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::us_em::us_em_point::UsEmPoint, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::us_em::us_em_point::UsEmPoint, pointpos_),
  PROTOBUF_FIELD_OFFSET(::pb::us_em::us_em_point::UsEmPoint, probhigh_),
  0,
  1,
  PROTOBUF_FIELD_OFFSET(::pb::us_em::us_em_point::UsEmPoint_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::us_em::us_em_point::UsEmPoint_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::us_em::us_em_point::UsEmPoint_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::pb::us_em::us_em_point::UsEmPoint)},
  { 9, 15, sizeof(::pb::us_em::us_em_point::UsEmPoint_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::us_em::us_em_point::_UsEmPoint_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::us_em::us_em_point::_UsEmPoint_array_port_default_instance_),
};

const char descriptor_table_protodef_us_5fem_2fus_5fem_5fpoint_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\027us_em/us_em_point.proto\022\024pb.us_em.us_e"
  "m_point\032\032us_em/us_em_position.proto\"X\n\tU"
  "sEmPoint\0228\n\010pointPos\030\326\013 \001(\0132%.pb.us_em.u"
  "s_em_position.UsEmPosition\022\021\n\010probHigh\030\264"
  "\023 \001(\002\"F\n\024UsEmPoint_array_port\022.\n\004data\030\306\007"
  " \003(\0132\037.pb.us_em.us_em_point.UsEmPoint"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_us_5fem_2fus_5fem_5fpoint_2eproto_deps[1] = {
  &::descriptor_table_us_5fem_2fus_5fem_5fposition_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_us_5fem_2fus_5fem_5fpoint_2eproto_sccs[2] = {
  &scc_info_UsEmPoint_us_5fem_2fus_5fem_5fpoint_2eproto.base,
  &scc_info_UsEmPoint_array_port_us_5fem_2fus_5fem_5fpoint_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_us_5fem_2fus_5fem_5fpoint_2eproto_once;
static bool descriptor_table_us_5fem_2fus_5fem_5fpoint_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fem_2fus_5fem_5fpoint_2eproto = {
  &descriptor_table_us_5fem_2fus_5fem_5fpoint_2eproto_initialized, descriptor_table_protodef_us_5fem_2fus_5fem_5fpoint_2eproto, "us_em/us_em_point.proto", 237,
  &descriptor_table_us_5fem_2fus_5fem_5fpoint_2eproto_once, descriptor_table_us_5fem_2fus_5fem_5fpoint_2eproto_sccs, descriptor_table_us_5fem_2fus_5fem_5fpoint_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_us_5fem_2fus_5fem_5fpoint_2eproto::offsets,
  file_level_metadata_us_5fem_2fus_5fem_5fpoint_2eproto, 2, file_level_enum_descriptors_us_5fem_2fus_5fem_5fpoint_2eproto, file_level_service_descriptors_us_5fem_2fus_5fem_5fpoint_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_us_5fem_2fus_5fem_5fpoint_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_us_5fem_2fus_5fem_5fpoint_2eproto), true);
namespace pb {
namespace us_em {
namespace us_em_point {

// ===================================================================

void UsEmPoint::InitAsDefaultInstance() {
  ::pb::us_em::us_em_point::_UsEmPoint_default_instance_._instance.get_mutable()->pointpos_ = const_cast< ::pb::us_em::us_em_position::UsEmPosition*>(
      ::pb::us_em::us_em_position::UsEmPosition::internal_default_instance());
}
class UsEmPoint::_Internal {
 public:
  using HasBits = decltype(std::declval<UsEmPoint>()._has_bits_);
  static const ::pb::us_em::us_em_position::UsEmPosition& pointpos(const UsEmPoint* msg);
  static void set_has_pointpos(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_probhigh(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

const ::pb::us_em::us_em_position::UsEmPosition&
UsEmPoint::_Internal::pointpos(const UsEmPoint* msg) {
  return *msg->pointpos_;
}
void UsEmPoint::clear_pointpos() {
  if (pointpos_ != nullptr) pointpos_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
UsEmPoint::UsEmPoint()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.us_em.us_em_point.UsEmPoint)
}
UsEmPoint::UsEmPoint(const UsEmPoint& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_pointpos()) {
    pointpos_ = new ::pb::us_em::us_em_position::UsEmPosition(*from.pointpos_);
  } else {
    pointpos_ = nullptr;
  }
  probhigh_ = from.probhigh_;
  // @@protoc_insertion_point(copy_constructor:pb.us_em.us_em_point.UsEmPoint)
}

void UsEmPoint::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_UsEmPoint_us_5fem_2fus_5fem_5fpoint_2eproto.base);
  ::memset(&pointpos_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&probhigh_) -
      reinterpret_cast<char*>(&pointpos_)) + sizeof(probhigh_));
}

UsEmPoint::~UsEmPoint() {
  // @@protoc_insertion_point(destructor:pb.us_em.us_em_point.UsEmPoint)
  SharedDtor();
}

void UsEmPoint::SharedDtor() {
  if (this != internal_default_instance()) delete pointpos_;
}

void UsEmPoint::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const UsEmPoint& UsEmPoint::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_UsEmPoint_us_5fem_2fus_5fem_5fpoint_2eproto.base);
  return *internal_default_instance();
}


void UsEmPoint::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.us_em.us_em_point.UsEmPoint)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(pointpos_ != nullptr);
    pointpos_->Clear();
  }
  probhigh_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* UsEmPoint::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .pb.us_em.us_em_position.UsEmPosition pointPos = 1494;
      case 1494:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 178)) {
          ptr = ctx->ParseMessage(_internal_mutable_pointpos(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional float probHigh = 2484;
      case 2484:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 165)) {
          _Internal::set_has_probhigh(&has_bits);
          probhigh_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* UsEmPoint::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.us_em.us_em_point.UsEmPoint)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.us_em.us_em_position.UsEmPosition pointPos = 1494;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1494, _Internal::pointpos(this), target, stream);
  }

  // optional float probHigh = 2484;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2484, this->_internal_probhigh(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.us_em.us_em_point.UsEmPoint)
  return target;
}

size_t UsEmPoint::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.us_em.us_em_point.UsEmPoint)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // optional .pb.us_em.us_em_position.UsEmPosition pointPos = 1494;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *pointpos_);
    }

    // optional float probHigh = 2484;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 + 4;
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

void UsEmPoint::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.us_em.us_em_point.UsEmPoint)
  GOOGLE_DCHECK_NE(&from, this);
  const UsEmPoint* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<UsEmPoint>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.us_em.us_em_point.UsEmPoint)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.us_em.us_em_point.UsEmPoint)
    MergeFrom(*source);
  }
}

void UsEmPoint::MergeFrom(const UsEmPoint& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.us_em.us_em_point.UsEmPoint)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_pointpos()->::pb::us_em::us_em_position::UsEmPosition::MergeFrom(from._internal_pointpos());
    }
    if (cached_has_bits & 0x00000002u) {
      probhigh_ = from.probhigh_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void UsEmPoint::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.us_em.us_em_point.UsEmPoint)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void UsEmPoint::CopyFrom(const UsEmPoint& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.us_em.us_em_point.UsEmPoint)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool UsEmPoint::IsInitialized() const {
  return true;
}

void UsEmPoint::InternalSwap(UsEmPoint* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(pointpos_, other->pointpos_);
  swap(probhigh_, other->probhigh_);
}

::PROTOBUF_NAMESPACE_ID::Metadata UsEmPoint::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void UsEmPoint_array_port::InitAsDefaultInstance() {
}
class UsEmPoint_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<UsEmPoint_array_port>()._has_bits_);
};

UsEmPoint_array_port::UsEmPoint_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.us_em.us_em_point.UsEmPoint_array_port)
}
UsEmPoint_array_port::UsEmPoint_array_port(const UsEmPoint_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.us_em.us_em_point.UsEmPoint_array_port)
}

void UsEmPoint_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_UsEmPoint_array_port_us_5fem_2fus_5fem_5fpoint_2eproto.base);
}

UsEmPoint_array_port::~UsEmPoint_array_port() {
  // @@protoc_insertion_point(destructor:pb.us_em.us_em_point.UsEmPoint_array_port)
  SharedDtor();
}

void UsEmPoint_array_port::SharedDtor() {
}

void UsEmPoint_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const UsEmPoint_array_port& UsEmPoint_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_UsEmPoint_array_port_us_5fem_2fus_5fem_5fpoint_2eproto.base);
  return *internal_default_instance();
}


void UsEmPoint_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.us_em.us_em_point.UsEmPoint_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* UsEmPoint_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.us_em.us_em_point.UsEmPoint data = 966;
      case 966:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 50)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<7730>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* UsEmPoint_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.us_em.us_em_point.UsEmPoint_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.us_em.us_em_point.UsEmPoint data = 966;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(966, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.us_em.us_em_point.UsEmPoint_array_port)
  return target;
}

size_t UsEmPoint_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.us_em.us_em_point.UsEmPoint_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.us_em.us_em_point.UsEmPoint data = 966;
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

void UsEmPoint_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.us_em.us_em_point.UsEmPoint_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const UsEmPoint_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<UsEmPoint_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.us_em.us_em_point.UsEmPoint_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.us_em.us_em_point.UsEmPoint_array_port)
    MergeFrom(*source);
  }
}

void UsEmPoint_array_port::MergeFrom(const UsEmPoint_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.us_em.us_em_point.UsEmPoint_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void UsEmPoint_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.us_em.us_em_point.UsEmPoint_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void UsEmPoint_array_port::CopyFrom(const UsEmPoint_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.us_em.us_em_point.UsEmPoint_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool UsEmPoint_array_port::IsInitialized() const {
  return true;
}

void UsEmPoint_array_port::InternalSwap(UsEmPoint_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata UsEmPoint_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace us_em_point
}  // namespace us_em
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::us_em::us_em_point::UsEmPoint* Arena::CreateMaybeMessage< ::pb::us_em::us_em_point::UsEmPoint >(Arena* arena) {
  return Arena::CreateInternal< ::pb::us_em::us_em_point::UsEmPoint >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::us_em::us_em_point::UsEmPoint_array_port* Arena::CreateMaybeMessage< ::pb::us_em::us_em_point::UsEmPoint_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::us_em::us_em_point::UsEmPoint_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
