// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_hmih/general_warnings.proto

#include "mf_hmih/general_warnings.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_mf_5fhmih_2fgeneral_5fwarnings_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_GeneralWarnings_mf_5fhmih_2fgeneral_5fwarnings_2eproto;
namespace pb {
namespace mf_hmih {
namespace general_warnings {
class GeneralWarningsDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<GeneralWarnings> _instance;
} _GeneralWarnings_default_instance_;
class GeneralWarnings_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<GeneralWarnings_array_port> _instance;
} _GeneralWarnings_array_port_default_instance_;
}  // namespace general_warnings
}  // namespace mf_hmih
}  // namespace pb
static void InitDefaultsscc_info_GeneralWarnings_mf_5fhmih_2fgeneral_5fwarnings_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_hmih::general_warnings::_GeneralWarnings_default_instance_;
    new (ptr) ::pb::mf_hmih::general_warnings::GeneralWarnings();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_hmih::general_warnings::GeneralWarnings::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_GeneralWarnings_mf_5fhmih_2fgeneral_5fwarnings_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_GeneralWarnings_mf_5fhmih_2fgeneral_5fwarnings_2eproto}, {}};

static void InitDefaultsscc_info_GeneralWarnings_array_port_mf_5fhmih_2fgeneral_5fwarnings_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_hmih::general_warnings::_GeneralWarnings_array_port_default_instance_;
    new (ptr) ::pb::mf_hmih::general_warnings::GeneralWarnings_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_hmih::general_warnings::GeneralWarnings_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_GeneralWarnings_array_port_mf_5fhmih_2fgeneral_5fwarnings_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_GeneralWarnings_array_port_mf_5fhmih_2fgeneral_5fwarnings_2eproto}, {
      &scc_info_GeneralWarnings_mf_5fhmih_2fgeneral_5fwarnings_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_mf_5fhmih_2fgeneral_5fwarnings_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_mf_5fhmih_2fgeneral_5fwarnings_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_mf_5fhmih_2fgeneral_5fwarnings_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_mf_5fhmih_2fgeneral_5fwarnings_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::mf_hmih::general_warnings::GeneralWarnings, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_hmih::general_warnings::GeneralWarnings, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_hmih::general_warnings::GeneralWarnings, visualdaytimeestimation_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_hmih::general_warnings::GeneralWarnings, narrowparkingspacewarning_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_hmih::general_warnings::GeneralWarnings, parkingspaceobstaclesaremoving_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_hmih::general_warnings::GeneralWarnings, visualsensorsystemwarning_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_hmih::general_warnings::GeneralWarnings, ultrasoundsensorsystemwarning_nu_),
  0,
  1,
  2,
  3,
  4,
  PROTOBUF_FIELD_OFFSET(::pb::mf_hmih::general_warnings::GeneralWarnings_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_hmih::general_warnings::GeneralWarnings_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_hmih::general_warnings::GeneralWarnings_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 10, sizeof(::pb::mf_hmih::general_warnings::GeneralWarnings)},
  { 15, 21, sizeof(::pb::mf_hmih::general_warnings::GeneralWarnings_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_hmih::general_warnings::_GeneralWarnings_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_hmih::general_warnings::_GeneralWarnings_array_port_default_instance_),
};

const char descriptor_table_protodef_mf_5fhmih_2fgeneral_5fwarnings_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\036mf_hmih/general_warnings.proto\022\033pb.mf_"
  "hmih.general_warnings\032\026mf_hmih/day_time."
  "proto\"\371\001\n\017GeneralWarnings\022A\n\032visualDayti"
  "meEstimation_nu\030\273\001 \001(\0162\034.pb.mf_hmih.day_"
  "time.DayTime\022%\n\034narrowParkingSpaceWarnin"
  "g_nu\030\231\034 \001(\010\022*\n!parkingSpaceObstaclesAreM"
  "oving_nu\030\333\014 \001(\010\022%\n\034visualSensorSystemWar"
  "ning_nu\030\303\024 \001(\010\022)\n ultrasoundSensorSystem"
  "Warning_nu\030\200\001 \001(\010\"Y\n\032GeneralWarnings_arr"
  "ay_port\022;\n\004data\030\227\032 \003(\0132,.pb.mf_hmih.gene"
  "ral_warnings.GeneralWarnings"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_mf_5fhmih_2fgeneral_5fwarnings_2eproto_deps[1] = {
  &::descriptor_table_mf_5fhmih_2fday_5ftime_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_mf_5fhmih_2fgeneral_5fwarnings_2eproto_sccs[2] = {
  &scc_info_GeneralWarnings_mf_5fhmih_2fgeneral_5fwarnings_2eproto.base,
  &scc_info_GeneralWarnings_array_port_mf_5fhmih_2fgeneral_5fwarnings_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_mf_5fhmih_2fgeneral_5fwarnings_2eproto_once;
static bool descriptor_table_mf_5fhmih_2fgeneral_5fwarnings_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fhmih_2fgeneral_5fwarnings_2eproto = {
  &descriptor_table_mf_5fhmih_2fgeneral_5fwarnings_2eproto_initialized, descriptor_table_protodef_mf_5fhmih_2fgeneral_5fwarnings_2eproto, "mf_hmih/general_warnings.proto", 428,
  &descriptor_table_mf_5fhmih_2fgeneral_5fwarnings_2eproto_once, descriptor_table_mf_5fhmih_2fgeneral_5fwarnings_2eproto_sccs, descriptor_table_mf_5fhmih_2fgeneral_5fwarnings_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_mf_5fhmih_2fgeneral_5fwarnings_2eproto::offsets,
  file_level_metadata_mf_5fhmih_2fgeneral_5fwarnings_2eproto, 2, file_level_enum_descriptors_mf_5fhmih_2fgeneral_5fwarnings_2eproto, file_level_service_descriptors_mf_5fhmih_2fgeneral_5fwarnings_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_mf_5fhmih_2fgeneral_5fwarnings_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_mf_5fhmih_2fgeneral_5fwarnings_2eproto), true);
namespace pb {
namespace mf_hmih {
namespace general_warnings {

// ===================================================================

void GeneralWarnings::InitAsDefaultInstance() {
}
class GeneralWarnings::_Internal {
 public:
  using HasBits = decltype(std::declval<GeneralWarnings>()._has_bits_);
  static void set_has_visualdaytimeestimation_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_narrowparkingspacewarning_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_parkingspaceobstaclesaremoving_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_visualsensorsystemwarning_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_ultrasoundsensorsystemwarning_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
};

GeneralWarnings::GeneralWarnings()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_hmih.general_warnings.GeneralWarnings)
}
GeneralWarnings::GeneralWarnings(const GeneralWarnings& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&visualdaytimeestimation_nu_, &from.visualdaytimeestimation_nu_,
    static_cast<size_t>(reinterpret_cast<char*>(&ultrasoundsensorsystemwarning_nu_) -
    reinterpret_cast<char*>(&visualdaytimeestimation_nu_)) + sizeof(ultrasoundsensorsystemwarning_nu_));
  // @@protoc_insertion_point(copy_constructor:pb.mf_hmih.general_warnings.GeneralWarnings)
}

void GeneralWarnings::SharedCtor() {
  ::memset(&visualdaytimeestimation_nu_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&ultrasoundsensorsystemwarning_nu_) -
      reinterpret_cast<char*>(&visualdaytimeestimation_nu_)) + sizeof(ultrasoundsensorsystemwarning_nu_));
}

GeneralWarnings::~GeneralWarnings() {
  // @@protoc_insertion_point(destructor:pb.mf_hmih.general_warnings.GeneralWarnings)
  SharedDtor();
}

void GeneralWarnings::SharedDtor() {
}

void GeneralWarnings::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const GeneralWarnings& GeneralWarnings::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_GeneralWarnings_mf_5fhmih_2fgeneral_5fwarnings_2eproto.base);
  return *internal_default_instance();
}


void GeneralWarnings::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_hmih.general_warnings.GeneralWarnings)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    ::memset(&visualdaytimeestimation_nu_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&ultrasoundsensorsystemwarning_nu_) -
        reinterpret_cast<char*>(&visualdaytimeestimation_nu_)) + sizeof(ultrasoundsensorsystemwarning_nu_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* GeneralWarnings::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional bool ultrasoundSensorSystemWarning_nu = 128;
      case 128:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 0)) {
          _Internal::set_has_ultrasoundsensorsystemwarning_nu(&has_bits);
          ultrasoundsensorsystemwarning_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.mf_hmih.day_time.DayTime visualDaytimeEstimation_nu = 187;
      case 187:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 216)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::mf_hmih::day_time::DayTime_IsValid(val))) {
            _internal_set_visualdaytimeestimation_nu(static_cast<::pb::mf_hmih::day_time::DayTime>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(187, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional bool parkingSpaceObstaclesAreMoving_nu = 1627;
      case 1627:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 216)) {
          _Internal::set_has_parkingspaceobstaclesaremoving_nu(&has_bits);
          parkingspaceobstaclesaremoving_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool visualSensorSystemWarning_nu = 2627;
      case 2627:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          _Internal::set_has_visualsensorsystemwarning_nu(&has_bits);
          visualsensorsystemwarning_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool narrowParkingSpaceWarning_nu = 3609;
      case 3609:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 200)) {
          _Internal::set_has_narrowparkingspacewarning_nu(&has_bits);
          narrowparkingspacewarning_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* GeneralWarnings::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_hmih.general_warnings.GeneralWarnings)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional bool ultrasoundSensorSystemWarning_nu = 128;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(128, this->_internal_ultrasoundsensorsystemwarning_nu(), target);
  }

  // optional .pb.mf_hmih.day_time.DayTime visualDaytimeEstimation_nu = 187;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      187, this->_internal_visualdaytimeestimation_nu(), target);
  }

  // optional bool parkingSpaceObstaclesAreMoving_nu = 1627;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1627, this->_internal_parkingspaceobstaclesaremoving_nu(), target);
  }

  // optional bool visualSensorSystemWarning_nu = 2627;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(2627, this->_internal_visualsensorsystemwarning_nu(), target);
  }

  // optional bool narrowParkingSpaceWarning_nu = 3609;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3609, this->_internal_narrowparkingspacewarning_nu(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_hmih.general_warnings.GeneralWarnings)
  return target;
}

size_t GeneralWarnings::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_hmih.general_warnings.GeneralWarnings)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    // optional .pb.mf_hmih.day_time.DayTime visualDaytimeEstimation_nu = 187;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_visualdaytimeestimation_nu());
    }

    // optional bool narrowParkingSpaceWarning_nu = 3609;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 + 1;
    }

    // optional bool parkingSpaceObstaclesAreMoving_nu = 1627;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 + 1;
    }

    // optional bool visualSensorSystemWarning_nu = 2627;
    if (cached_has_bits & 0x00000008u) {
      total_size += 3 + 1;
    }

    // optional bool ultrasoundSensorSystemWarning_nu = 128;
    if (cached_has_bits & 0x00000010u) {
      total_size += 2 + 1;
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

void GeneralWarnings::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_hmih.general_warnings.GeneralWarnings)
  GOOGLE_DCHECK_NE(&from, this);
  const GeneralWarnings* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<GeneralWarnings>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_hmih.general_warnings.GeneralWarnings)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_hmih.general_warnings.GeneralWarnings)
    MergeFrom(*source);
  }
}

void GeneralWarnings::MergeFrom(const GeneralWarnings& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_hmih.general_warnings.GeneralWarnings)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    if (cached_has_bits & 0x00000001u) {
      visualdaytimeestimation_nu_ = from.visualdaytimeestimation_nu_;
    }
    if (cached_has_bits & 0x00000002u) {
      narrowparkingspacewarning_nu_ = from.narrowparkingspacewarning_nu_;
    }
    if (cached_has_bits & 0x00000004u) {
      parkingspaceobstaclesaremoving_nu_ = from.parkingspaceobstaclesaremoving_nu_;
    }
    if (cached_has_bits & 0x00000008u) {
      visualsensorsystemwarning_nu_ = from.visualsensorsystemwarning_nu_;
    }
    if (cached_has_bits & 0x00000010u) {
      ultrasoundsensorsystemwarning_nu_ = from.ultrasoundsensorsystemwarning_nu_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void GeneralWarnings::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_hmih.general_warnings.GeneralWarnings)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void GeneralWarnings::CopyFrom(const GeneralWarnings& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_hmih.general_warnings.GeneralWarnings)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool GeneralWarnings::IsInitialized() const {
  return true;
}

void GeneralWarnings::InternalSwap(GeneralWarnings* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(visualdaytimeestimation_nu_, other->visualdaytimeestimation_nu_);
  swap(narrowparkingspacewarning_nu_, other->narrowparkingspacewarning_nu_);
  swap(parkingspaceobstaclesaremoving_nu_, other->parkingspaceobstaclesaremoving_nu_);
  swap(visualsensorsystemwarning_nu_, other->visualsensorsystemwarning_nu_);
  swap(ultrasoundsensorsystemwarning_nu_, other->ultrasoundsensorsystemwarning_nu_);
}

::PROTOBUF_NAMESPACE_ID::Metadata GeneralWarnings::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void GeneralWarnings_array_port::InitAsDefaultInstance() {
}
class GeneralWarnings_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<GeneralWarnings_array_port>()._has_bits_);
};

GeneralWarnings_array_port::GeneralWarnings_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
}
GeneralWarnings_array_port::GeneralWarnings_array_port(const GeneralWarnings_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
}

void GeneralWarnings_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_GeneralWarnings_array_port_mf_5fhmih_2fgeneral_5fwarnings_2eproto.base);
}

GeneralWarnings_array_port::~GeneralWarnings_array_port() {
  // @@protoc_insertion_point(destructor:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
  SharedDtor();
}

void GeneralWarnings_array_port::SharedDtor() {
}

void GeneralWarnings_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const GeneralWarnings_array_port& GeneralWarnings_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_GeneralWarnings_array_port_mf_5fhmih_2fgeneral_5fwarnings_2eproto.base);
  return *internal_default_instance();
}


void GeneralWarnings_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* GeneralWarnings_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.mf_hmih.general_warnings.GeneralWarnings data = 3351;
      case 3351:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 186)) {
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

::PROTOBUF_NAMESPACE_ID::uint8* GeneralWarnings_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.mf_hmih.general_warnings.GeneralWarnings data = 3351;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3351, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
  return target;
}

size_t GeneralWarnings_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.mf_hmih.general_warnings.GeneralWarnings data = 3351;
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

void GeneralWarnings_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const GeneralWarnings_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<GeneralWarnings_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
    MergeFrom(*source);
  }
}

void GeneralWarnings_array_port::MergeFrom(const GeneralWarnings_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void GeneralWarnings_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void GeneralWarnings_array_port::CopyFrom(const GeneralWarnings_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool GeneralWarnings_array_port::IsInitialized() const {
  return true;
}

void GeneralWarnings_array_port::InternalSwap(GeneralWarnings_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata GeneralWarnings_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace general_warnings
}  // namespace mf_hmih
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::mf_hmih::general_warnings::GeneralWarnings* Arena::CreateMaybeMessage< ::pb::mf_hmih::general_warnings::GeneralWarnings >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_hmih::general_warnings::GeneralWarnings >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::mf_hmih::general_warnings::GeneralWarnings_array_port* Arena::CreateMaybeMessage< ::pb::mf_hmih::general_warnings::GeneralWarnings_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_hmih::general_warnings::GeneralWarnings_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>