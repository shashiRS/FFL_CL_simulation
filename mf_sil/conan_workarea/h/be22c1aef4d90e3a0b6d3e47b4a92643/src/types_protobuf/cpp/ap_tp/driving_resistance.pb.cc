// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_tp/driving_resistance.proto

#include "ap_tp/driving_resistance.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_ap_5ftp_2fdriving_5fresistance_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_DrivingResistance_ap_5ftp_2fdriving_5fresistance_2eproto;
namespace pb {
namespace ap_tp {
namespace driving_resistance {
class DrivingResistanceDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<DrivingResistance> _instance;
} _DrivingResistance_default_instance_;
class DrivingResistance_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<DrivingResistance_array_port> _instance;
} _DrivingResistance_array_port_default_instance_;
}  // namespace driving_resistance
}  // namespace ap_tp
}  // namespace pb
static void InitDefaultsscc_info_DrivingResistance_ap_5ftp_2fdriving_5fresistance_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_tp::driving_resistance::_DrivingResistance_default_instance_;
    new (ptr) ::pb::ap_tp::driving_resistance::DrivingResistance();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_tp::driving_resistance::DrivingResistance::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_DrivingResistance_ap_5ftp_2fdriving_5fresistance_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_DrivingResistance_ap_5ftp_2fdriving_5fresistance_2eproto}, {}};

static void InitDefaultsscc_info_DrivingResistance_array_port_ap_5ftp_2fdriving_5fresistance_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_tp::driving_resistance::_DrivingResistance_array_port_default_instance_;
    new (ptr) ::pb::ap_tp::driving_resistance::DrivingResistance_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_tp::driving_resistance::DrivingResistance_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_DrivingResistance_array_port_ap_5ftp_2fdriving_5fresistance_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_DrivingResistance_array_port_ap_5ftp_2fdriving_5fresistance_2eproto}, {
      &scc_info_DrivingResistance_ap_5ftp_2fdriving_5fresistance_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ap_5ftp_2fdriving_5fresistance_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ap_5ftp_2fdriving_5fresistance_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5ftp_2fdriving_5fresistance_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5ftp_2fdriving_5fresistance_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driving_resistance::DrivingResistance, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driving_resistance::DrivingResistance, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driving_resistance::DrivingResistance, distance_m_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driving_resistance::DrivingResistance, type_nu_),
  1,
  0,
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driving_resistance::DrivingResistance_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driving_resistance::DrivingResistance_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driving_resistance::DrivingResistance_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::pb::ap_tp::driving_resistance::DrivingResistance)},
  { 9, 15, sizeof(::pb::ap_tp::driving_resistance::DrivingResistance_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_tp::driving_resistance::_DrivingResistance_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_tp::driving_resistance::_DrivingResistance_array_port_default_instance_),
};

const char descriptor_table_protodef_ap_5ftp_2fdriving_5fresistance_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\036ap_tp/driving_resistance.proto\022\033pb.ap_"
  "tp.driving_resistance\032#ap_tp/driving_res"
  "istance_type.proto\"s\n\021DrivingResistance\022"
  "\023\n\ndistance_m\030\226\n \001(\002\022I\n\007type_nu\030\326\005 \001(\01627"
  ".pb.ap_tp.driving_resistance_type.Drivin"
  "gResistanceType\"]\n\034DrivingResistance_arr"
  "ay_port\022=\n\004data\030\335\001 \003(\0132..pb.ap_tp.drivin"
  "g_resistance.DrivingResistance"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5ftp_2fdriving_5fresistance_2eproto_deps[1] = {
  &::descriptor_table_ap_5ftp_2fdriving_5fresistance_5ftype_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5ftp_2fdriving_5fresistance_2eproto_sccs[2] = {
  &scc_info_DrivingResistance_ap_5ftp_2fdriving_5fresistance_2eproto.base,
  &scc_info_DrivingResistance_array_port_ap_5ftp_2fdriving_5fresistance_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5ftp_2fdriving_5fresistance_2eproto_once;
static bool descriptor_table_ap_5ftp_2fdriving_5fresistance_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftp_2fdriving_5fresistance_2eproto = {
  &descriptor_table_ap_5ftp_2fdriving_5fresistance_2eproto_initialized, descriptor_table_protodef_ap_5ftp_2fdriving_5fresistance_2eproto, "ap_tp/driving_resistance.proto", 310,
  &descriptor_table_ap_5ftp_2fdriving_5fresistance_2eproto_once, descriptor_table_ap_5ftp_2fdriving_5fresistance_2eproto_sccs, descriptor_table_ap_5ftp_2fdriving_5fresistance_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ap_5ftp_2fdriving_5fresistance_2eproto::offsets,
  file_level_metadata_ap_5ftp_2fdriving_5fresistance_2eproto, 2, file_level_enum_descriptors_ap_5ftp_2fdriving_5fresistance_2eproto, file_level_service_descriptors_ap_5ftp_2fdriving_5fresistance_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5ftp_2fdriving_5fresistance_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5ftp_2fdriving_5fresistance_2eproto), true);
namespace pb {
namespace ap_tp {
namespace driving_resistance {

// ===================================================================

void DrivingResistance::InitAsDefaultInstance() {
}
class DrivingResistance::_Internal {
 public:
  using HasBits = decltype(std::declval<DrivingResistance>()._has_bits_);
  static void set_has_distance_m(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_type_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

DrivingResistance::DrivingResistance()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_tp.driving_resistance.DrivingResistance)
}
DrivingResistance::DrivingResistance(const DrivingResistance& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&type_nu_, &from.type_nu_,
    static_cast<size_t>(reinterpret_cast<char*>(&distance_m_) -
    reinterpret_cast<char*>(&type_nu_)) + sizeof(distance_m_));
  // @@protoc_insertion_point(copy_constructor:pb.ap_tp.driving_resistance.DrivingResistance)
}

void DrivingResistance::SharedCtor() {
  ::memset(&type_nu_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&distance_m_) -
      reinterpret_cast<char*>(&type_nu_)) + sizeof(distance_m_));
}

DrivingResistance::~DrivingResistance() {
  // @@protoc_insertion_point(destructor:pb.ap_tp.driving_resistance.DrivingResistance)
  SharedDtor();
}

void DrivingResistance::SharedDtor() {
}

void DrivingResistance::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const DrivingResistance& DrivingResistance::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_DrivingResistance_ap_5ftp_2fdriving_5fresistance_2eproto.base);
  return *internal_default_instance();
}


void DrivingResistance::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_tp.driving_resistance.DrivingResistance)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    ::memset(&type_nu_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&distance_m_) -
        reinterpret_cast<char*>(&type_nu_)) + sizeof(distance_m_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* DrivingResistance::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .pb.ap_tp.driving_resistance_type.DrivingResistanceType type_nu = 726;
      case 726:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 176)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_tp::driving_resistance_type::DrivingResistanceType_IsValid(val))) {
            _internal_set_type_nu(static_cast<::pb::ap_tp::driving_resistance_type::DrivingResistanceType>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(726, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional float distance_m = 1302;
      case 1302:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 181)) {
          _Internal::set_has_distance_m(&has_bits);
          distance_m_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* DrivingResistance::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_tp.driving_resistance.DrivingResistance)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.ap_tp.driving_resistance_type.DrivingResistanceType type_nu = 726;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      726, this->_internal_type_nu(), target);
  }

  // optional float distance_m = 1302;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(1302, this->_internal_distance_m(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_tp.driving_resistance.DrivingResistance)
  return target;
}

size_t DrivingResistance::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_tp.driving_resistance.DrivingResistance)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // optional .pb.ap_tp.driving_resistance_type.DrivingResistanceType type_nu = 726;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_type_nu());
    }

    // optional float distance_m = 1302;
    if (cached_has_bits & 0x00000002u) {
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

void DrivingResistance::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_tp.driving_resistance.DrivingResistance)
  GOOGLE_DCHECK_NE(&from, this);
  const DrivingResistance* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<DrivingResistance>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_tp.driving_resistance.DrivingResistance)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_tp.driving_resistance.DrivingResistance)
    MergeFrom(*source);
  }
}

void DrivingResistance::MergeFrom(const DrivingResistance& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_tp.driving_resistance.DrivingResistance)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      type_nu_ = from.type_nu_;
    }
    if (cached_has_bits & 0x00000002u) {
      distance_m_ = from.distance_m_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void DrivingResistance::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_tp.driving_resistance.DrivingResistance)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void DrivingResistance::CopyFrom(const DrivingResistance& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_tp.driving_resistance.DrivingResistance)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DrivingResistance::IsInitialized() const {
  return true;
}

void DrivingResistance::InternalSwap(DrivingResistance* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(type_nu_, other->type_nu_);
  swap(distance_m_, other->distance_m_);
}

::PROTOBUF_NAMESPACE_ID::Metadata DrivingResistance::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void DrivingResistance_array_port::InitAsDefaultInstance() {
}
class DrivingResistance_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<DrivingResistance_array_port>()._has_bits_);
};

DrivingResistance_array_port::DrivingResistance_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
}
DrivingResistance_array_port::DrivingResistance_array_port(const DrivingResistance_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
}

void DrivingResistance_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_DrivingResistance_array_port_ap_5ftp_2fdriving_5fresistance_2eproto.base);
}

DrivingResistance_array_port::~DrivingResistance_array_port() {
  // @@protoc_insertion_point(destructor:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
  SharedDtor();
}

void DrivingResistance_array_port::SharedDtor() {
}

void DrivingResistance_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const DrivingResistance_array_port& DrivingResistance_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_DrivingResistance_array_port_ap_5ftp_2fdriving_5fresistance_2eproto.base);
  return *internal_default_instance();
}


void DrivingResistance_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* DrivingResistance_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.ap_tp.driving_resistance.DrivingResistance data = 221;
      case 221:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 234)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<1770>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* DrivingResistance_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.ap_tp.driving_resistance.DrivingResistance data = 221;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(221, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
  return target;
}

size_t DrivingResistance_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.ap_tp.driving_resistance.DrivingResistance data = 221;
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

void DrivingResistance_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const DrivingResistance_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<DrivingResistance_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
    MergeFrom(*source);
  }
}

void DrivingResistance_array_port::MergeFrom(const DrivingResistance_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void DrivingResistance_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void DrivingResistance_array_port::CopyFrom(const DrivingResistance_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DrivingResistance_array_port::IsInitialized() const {
  return true;
}

void DrivingResistance_array_port::InternalSwap(DrivingResistance_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata DrivingResistance_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace driving_resistance
}  // namespace ap_tp
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::ap_tp::driving_resistance::DrivingResistance* Arena::CreateMaybeMessage< ::pb::ap_tp::driving_resistance::DrivingResistance >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_tp::driving_resistance::DrivingResistance >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::ap_tp::driving_resistance::DrivingResistance_array_port* Arena::CreateMaybeMessage< ::pb::ap_tp::driving_resistance::DrivingResistance_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_tp::driving_resistance::DrivingResistance_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>