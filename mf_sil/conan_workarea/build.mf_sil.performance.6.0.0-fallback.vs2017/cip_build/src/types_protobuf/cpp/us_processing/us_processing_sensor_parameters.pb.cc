// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_processing/us_processing_sensor_parameters.proto

#include "us_processing/us_processing_sensor_parameters.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameter_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_UsProcessingSensorParameter_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameter_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_UsProcessingSensorParameters_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto;
namespace pb {
namespace us_processing {
namespace us_processing_sensor_parameters {
class UsProcessingSensorParametersDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<UsProcessingSensorParameters> _instance;
} _UsProcessingSensorParameters_default_instance_;
class UsProcessingSensorParameters_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<UsProcessingSensorParameters_array_port> _instance;
} _UsProcessingSensorParameters_array_port_default_instance_;
}  // namespace us_processing_sensor_parameters
}  // namespace us_processing
}  // namespace pb
static void InitDefaultsscc_info_UsProcessingSensorParameters_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::us_processing::us_processing_sensor_parameters::_UsProcessingSensorParameters_default_instance_;
    new (ptr) ::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_UsProcessingSensorParameters_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_UsProcessingSensorParameters_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto}, {
      &scc_info_UsProcessingSensorParameter_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameter_2eproto.base,}};

static void InitDefaultsscc_info_UsProcessingSensorParameters_array_port_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::us_processing::us_processing_sensor_parameters::_UsProcessingSensorParameters_array_port_default_instance_;
    new (ptr) ::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_UsProcessingSensorParameters_array_port_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_UsProcessingSensorParameters_array_port_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto}, {
      &scc_info_UsProcessingSensorParameters_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters, sensorparametercount_),
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters, sensorparameter_),
  0,
  ~0u,
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters)},
  { 9, 15, sizeof(::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::us_processing::us_processing_sensor_parameters::_UsProcessingSensorParameters_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::us_processing::us_processing_sensor_parameters::_UsProcessingSensorParameters_array_port_default_instance_),
};

const char descriptor_table_protodef_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n3us_processing/us_processing_sensor_par"
  "ameters.proto\0220pb.us_processing.us_proce"
  "ssing_sensor_parameters\0322us_processing/u"
  "s_processing_sensor_parameter.proto\"\245\001\n\034"
  "UsProcessingSensorParameters\022\035\n\024sensorPa"
  "rameterCount\030\374\025 \001(\r\022f\n\017sensorParameter\030\302"
  "\n \003(\0132L.pb.us_processing.us_processing_s"
  "ensor_parameter.UsProcessingSensorParame"
  "ter\"\210\001\n\'UsProcessingSensorParameters_arr"
  "ay_port\022]\n\004data\030\332\003 \003(\0132N.pb.us_processin"
  "g.us_processing_sensor_parameters.UsProc"
  "essingSensorParameters"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto_deps[1] = {
  &::descriptor_table_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameter_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto_sccs[2] = {
  &scc_info_UsProcessingSensorParameters_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto.base,
  &scc_info_UsProcessingSensorParameters_array_port_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto_once;
static bool descriptor_table_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto = {
  &descriptor_table_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto_initialized, descriptor_table_protodef_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto, "us_processing/us_processing_sensor_parameters.proto", 462,
  &descriptor_table_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto_once, descriptor_table_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto_sccs, descriptor_table_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto::offsets,
  file_level_metadata_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto, 2, file_level_enum_descriptors_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto, file_level_service_descriptors_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto), true);
namespace pb {
namespace us_processing {
namespace us_processing_sensor_parameters {

// ===================================================================

void UsProcessingSensorParameters::InitAsDefaultInstance() {
}
class UsProcessingSensorParameters::_Internal {
 public:
  using HasBits = decltype(std::declval<UsProcessingSensorParameters>()._has_bits_);
  static void set_has_sensorparametercount(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

void UsProcessingSensorParameters::clear_sensorparameter() {
  sensorparameter_.Clear();
}
UsProcessingSensorParameters::UsProcessingSensorParameters()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
}
UsProcessingSensorParameters::UsProcessingSensorParameters(const UsProcessingSensorParameters& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      sensorparameter_(from.sensorparameter_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  sensorparametercount_ = from.sensorparametercount_;
  // @@protoc_insertion_point(copy_constructor:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
}

void UsProcessingSensorParameters::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_UsProcessingSensorParameters_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto.base);
  sensorparametercount_ = 0u;
}

UsProcessingSensorParameters::~UsProcessingSensorParameters() {
  // @@protoc_insertion_point(destructor:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
  SharedDtor();
}

void UsProcessingSensorParameters::SharedDtor() {
}

void UsProcessingSensorParameters::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const UsProcessingSensorParameters& UsProcessingSensorParameters::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_UsProcessingSensorParameters_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto.base);
  return *internal_default_instance();
}


void UsProcessingSensorParameters::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  sensorparameter_.Clear();
  sensorparametercount_ = 0u;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* UsProcessingSensorParameters::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.us_processing.us_processing_sensor_parameter.UsProcessingSensorParameter sensorParameter = 1346;
      case 1346:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_sensorparameter(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<10770>(ptr));
        } else goto handle_unusual;
        continue;
      // optional uint32 sensorParameterCount = 2812;
      case 2812:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 224)) {
          _Internal::set_has_sensorparametercount(&has_bits);
          sensorparametercount_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* UsProcessingSensorParameters::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.us_processing.us_processing_sensor_parameter.UsProcessingSensorParameter sensorParameter = 1346;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_sensorparameter_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1346, this->_internal_sensorparameter(i), target, stream);
  }

  cached_has_bits = _has_bits_[0];
  // optional uint32 sensorParameterCount = 2812;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2812, this->_internal_sensorparametercount(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
  return target;
}

size_t UsProcessingSensorParameters::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.us_processing.us_processing_sensor_parameter.UsProcessingSensorParameter sensorParameter = 1346;
  total_size += 2UL * this->_internal_sensorparameter_size();
  for (const auto& msg : this->sensorparameter_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // optional uint32 sensorParameterCount = 2812;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 3 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_sensorparametercount());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void UsProcessingSensorParameters::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
  GOOGLE_DCHECK_NE(&from, this);
  const UsProcessingSensorParameters* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<UsProcessingSensorParameters>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
    MergeFrom(*source);
  }
}

void UsProcessingSensorParameters::MergeFrom(const UsProcessingSensorParameters& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  sensorparameter_.MergeFrom(from.sensorparameter_);
  if (from._internal_has_sensorparametercount()) {
    _internal_set_sensorparametercount(from._internal_sensorparametercount());
  }
}

void UsProcessingSensorParameters::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void UsProcessingSensorParameters::CopyFrom(const UsProcessingSensorParameters& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool UsProcessingSensorParameters::IsInitialized() const {
  return true;
}

void UsProcessingSensorParameters::InternalSwap(UsProcessingSensorParameters* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  sensorparameter_.InternalSwap(&other->sensorparameter_);
  swap(sensorparametercount_, other->sensorparametercount_);
}

::PROTOBUF_NAMESPACE_ID::Metadata UsProcessingSensorParameters::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void UsProcessingSensorParameters_array_port::InitAsDefaultInstance() {
}
class UsProcessingSensorParameters_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<UsProcessingSensorParameters_array_port>()._has_bits_);
};

UsProcessingSensorParameters_array_port::UsProcessingSensorParameters_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
}
UsProcessingSensorParameters_array_port::UsProcessingSensorParameters_array_port(const UsProcessingSensorParameters_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
}

void UsProcessingSensorParameters_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_UsProcessingSensorParameters_array_port_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto.base);
}

UsProcessingSensorParameters_array_port::~UsProcessingSensorParameters_array_port() {
  // @@protoc_insertion_point(destructor:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
  SharedDtor();
}

void UsProcessingSensorParameters_array_port::SharedDtor() {
}

void UsProcessingSensorParameters_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const UsProcessingSensorParameters_array_port& UsProcessingSensorParameters_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_UsProcessingSensorParameters_array_port_us_5fprocessing_2fus_5fprocessing_5fsensor_5fparameters_2eproto.base);
  return *internal_default_instance();
}


void UsProcessingSensorParameters_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* UsProcessingSensorParameters_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters data = 474;
      case 474:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 210)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<3794>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* UsProcessingSensorParameters_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters data = 474;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(474, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
  return target;
}

size_t UsProcessingSensorParameters_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters data = 474;
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

void UsProcessingSensorParameters_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const UsProcessingSensorParameters_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<UsProcessingSensorParameters_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
    MergeFrom(*source);
  }
}

void UsProcessingSensorParameters_array_port::MergeFrom(const UsProcessingSensorParameters_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void UsProcessingSensorParameters_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void UsProcessingSensorParameters_array_port::CopyFrom(const UsProcessingSensorParameters_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool UsProcessingSensorParameters_array_port::IsInitialized() const {
  return true;
}

void UsProcessingSensorParameters_array_port::InternalSwap(UsProcessingSensorParameters_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata UsProcessingSensorParameters_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace us_processing_sensor_parameters
}  // namespace us_processing
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters* Arena::CreateMaybeMessage< ::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters >(Arena* arena) {
  return Arena::CreateInternal< ::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters_array_port* Arena::CreateMaybeMessage< ::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::us_processing::us_processing_sensor_parameters::UsProcessingSensorParameters_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
