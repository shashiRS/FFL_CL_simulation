// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_processing/us_processing_filtered_echo.proto

#include "us_processing/us_processing_filtered_echo.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_UsProcessingFilteredEcho_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto;
namespace pb {
namespace us_processing {
namespace us_processing_filtered_echo {
class UsProcessingFilteredEchoDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<UsProcessingFilteredEcho> _instance;
} _UsProcessingFilteredEcho_default_instance_;
class UsProcessingFilteredEcho_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<UsProcessingFilteredEcho_array_port> _instance;
} _UsProcessingFilteredEcho_array_port_default_instance_;
}  // namespace us_processing_filtered_echo
}  // namespace us_processing
}  // namespace pb
static void InitDefaultsscc_info_UsProcessingFilteredEcho_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::us_processing::us_processing_filtered_echo::_UsProcessingFilteredEcho_default_instance_;
    new (ptr) ::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_UsProcessingFilteredEcho_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_UsProcessingFilteredEcho_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto}, {}};

static void InitDefaultsscc_info_UsProcessingFilteredEcho_array_port_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::us_processing::us_processing_filtered_echo::_UsProcessingFilteredEcho_array_port_default_instance_;
    new (ptr) ::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_UsProcessingFilteredEcho_array_port_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_UsProcessingFilteredEcho_array_port_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto}, {
      &scc_info_UsProcessingFilteredEcho_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho, rxsensorid_),
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho, txsensorid_),
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho, trackid_),
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho, relecutimestamp_us_),
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho, tof_us_),
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho, amplitude_),
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho, phasederivative_),
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho, codingconfidence_),
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho, timeconfidence_),
  5,
  4,
  2,
  7,
  3,
  6,
  0,
  8,
  1,
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 14, sizeof(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho)},
  { 23, 29, sizeof(::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::us_processing::us_processing_filtered_echo::_UsProcessingFilteredEcho_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::us_processing::us_processing_filtered_echo::_UsProcessingFilteredEcho_array_port_default_instance_),
};

const char descriptor_table_protodef_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n/us_processing/us_processing_filtered_e"
  "cho.proto\022,pb.us_processing.us_processin"
  "g_filtered_echo\"\346\001\n\030UsProcessingFiltered"
  "Echo\022\023\n\nrxSensorId\030\330\r \001(\r\022\023\n\ntxSensorId\030"
  "\347\n \001(\r\022\020\n\007trackId\030\221\n \001(\r\022\033\n\022relEcuTimest"
  "amp_us\030\264\024 \001(\021\022\017\n\006tof_us\030\317\n \001(\r\022\022\n\tamplit"
  "ude\030\253\024 \001(\r\022\030\n\017phaseDerivative\030\210\034 \001(\021\022\031\n\020"
  "codingConfidence\030\230\025 \001(\r\022\027\n\016timeConfidenc"
  "e\030\216\002 \001(\r\"|\n#UsProcessingFilteredEcho_arr"
  "ay_port\022U\n\004data\030\261\010 \003(\0132F.pb.us_processin"
  "g.us_processing_filtered_echo.UsProcessi"
  "ngFilteredEcho"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto_sccs[2] = {
  &scc_info_UsProcessingFilteredEcho_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto.base,
  &scc_info_UsProcessingFilteredEcho_array_port_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto_once;
static bool descriptor_table_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto = {
  &descriptor_table_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto_initialized, descriptor_table_protodef_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto, "us_processing/us_processing_filtered_echo.proto", 454,
  &descriptor_table_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto_once, descriptor_table_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto_sccs, descriptor_table_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto::offsets,
  file_level_metadata_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto, 2, file_level_enum_descriptors_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto, file_level_service_descriptors_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto), true);
namespace pb {
namespace us_processing {
namespace us_processing_filtered_echo {

// ===================================================================

void UsProcessingFilteredEcho::InitAsDefaultInstance() {
}
class UsProcessingFilteredEcho::_Internal {
 public:
  using HasBits = decltype(std::declval<UsProcessingFilteredEcho>()._has_bits_);
  static void set_has_rxsensorid(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_txsensorid(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_trackid(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_relecutimestamp_us(HasBits* has_bits) {
    (*has_bits)[0] |= 128u;
  }
  static void set_has_tof_us(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_amplitude(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static void set_has_phasederivative(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_codingconfidence(HasBits* has_bits) {
    (*has_bits)[0] |= 256u;
  }
  static void set_has_timeconfidence(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

UsProcessingFilteredEcho::UsProcessingFilteredEcho()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho)
}
UsProcessingFilteredEcho::UsProcessingFilteredEcho(const UsProcessingFilteredEcho& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&phasederivative_, &from.phasederivative_,
    static_cast<size_t>(reinterpret_cast<char*>(&codingconfidence_) -
    reinterpret_cast<char*>(&phasederivative_)) + sizeof(codingconfidence_));
  // @@protoc_insertion_point(copy_constructor:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho)
}

void UsProcessingFilteredEcho::SharedCtor() {
  ::memset(&phasederivative_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&codingconfidence_) -
      reinterpret_cast<char*>(&phasederivative_)) + sizeof(codingconfidence_));
}

UsProcessingFilteredEcho::~UsProcessingFilteredEcho() {
  // @@protoc_insertion_point(destructor:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho)
  SharedDtor();
}

void UsProcessingFilteredEcho::SharedDtor() {
}

void UsProcessingFilteredEcho::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const UsProcessingFilteredEcho& UsProcessingFilteredEcho::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_UsProcessingFilteredEcho_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto.base);
  return *internal_default_instance();
}


void UsProcessingFilteredEcho::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    ::memset(&phasederivative_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&relecutimestamp_us_) -
        reinterpret_cast<char*>(&phasederivative_)) + sizeof(relecutimestamp_us_));
  }
  codingconfidence_ = 0u;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* UsProcessingFilteredEcho::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional uint32 timeConfidence = 270;
      case 270:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 112)) {
          _Internal::set_has_timeconfidence(&has_bits);
          timeconfidence_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 trackId = 1297;
      case 1297:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 136)) {
          _Internal::set_has_trackid(&has_bits);
          trackid_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 tof_us = 1359;
      case 1359:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 120)) {
          _Internal::set_has_tof_us(&has_bits);
          tof_us_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 txSensorId = 1383;
      case 1383:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          _Internal::set_has_txsensorid(&has_bits);
          txsensorid_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 rxSensorId = 1752;
      case 1752:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 192)) {
          _Internal::set_has_rxsensorid(&has_bits);
          rxsensorid_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 amplitude = 2603;
      case 2603:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 88)) {
          _Internal::set_has_amplitude(&has_bits);
          amplitude_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional sint32 relEcuTimestamp_us = 2612;
      case 2612:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 160)) {
          _Internal::set_has_relecutimestamp_us(&has_bits);
          relecutimestamp_us_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarintZigZag32(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 codingConfidence = 2712;
      case 2712:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 192)) {
          _Internal::set_has_codingconfidence(&has_bits);
          codingconfidence_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional sint32 phaseDerivative = 3592;
      case 3592:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 64)) {
          _Internal::set_has_phasederivative(&has_bits);
          phasederivative_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarintZigZag32(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* UsProcessingFilteredEcho::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional uint32 timeConfidence = 270;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(270, this->_internal_timeconfidence(), target);
  }

  // optional uint32 trackId = 1297;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1297, this->_internal_trackid(), target);
  }

  // optional uint32 tof_us = 1359;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1359, this->_internal_tof_us(), target);
  }

  // optional uint32 txSensorId = 1383;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1383, this->_internal_txsensorid(), target);
  }

  // optional uint32 rxSensorId = 1752;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1752, this->_internal_rxsensorid(), target);
  }

  // optional uint32 amplitude = 2603;
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2603, this->_internal_amplitude(), target);
  }

  // optional sint32 relEcuTimestamp_us = 2612;
  if (cached_has_bits & 0x00000080u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteSInt32ToArray(2612, this->_internal_relecutimestamp_us(), target);
  }

  // optional uint32 codingConfidence = 2712;
  if (cached_has_bits & 0x00000100u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2712, this->_internal_codingconfidence(), target);
  }

  // optional sint32 phaseDerivative = 3592;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteSInt32ToArray(3592, this->_internal_phasederivative(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho)
  return target;
}

size_t UsProcessingFilteredEcho::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    // optional sint32 phaseDerivative = 3592;
    if (cached_has_bits & 0x00000001u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SInt32Size(
          this->_internal_phasederivative());
    }

    // optional uint32 timeConfidence = 270;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_timeconfidence());
    }

    // optional uint32 trackId = 1297;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_trackid());
    }

    // optional uint32 tof_us = 1359;
    if (cached_has_bits & 0x00000008u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_tof_us());
    }

    // optional uint32 txSensorId = 1383;
    if (cached_has_bits & 0x00000010u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_txsensorid());
    }

    // optional uint32 rxSensorId = 1752;
    if (cached_has_bits & 0x00000020u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_rxsensorid());
    }

    // optional uint32 amplitude = 2603;
    if (cached_has_bits & 0x00000040u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_amplitude());
    }

    // optional sint32 relEcuTimestamp_us = 2612;
    if (cached_has_bits & 0x00000080u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SInt32Size(
          this->_internal_relecutimestamp_us());
    }

  }
  // optional uint32 codingConfidence = 2712;
  if (cached_has_bits & 0x00000100u) {
    total_size += 3 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_codingconfidence());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void UsProcessingFilteredEcho::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho)
  GOOGLE_DCHECK_NE(&from, this);
  const UsProcessingFilteredEcho* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<UsProcessingFilteredEcho>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho)
    MergeFrom(*source);
  }
}

void UsProcessingFilteredEcho::MergeFrom(const UsProcessingFilteredEcho& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    if (cached_has_bits & 0x00000001u) {
      phasederivative_ = from.phasederivative_;
    }
    if (cached_has_bits & 0x00000002u) {
      timeconfidence_ = from.timeconfidence_;
    }
    if (cached_has_bits & 0x00000004u) {
      trackid_ = from.trackid_;
    }
    if (cached_has_bits & 0x00000008u) {
      tof_us_ = from.tof_us_;
    }
    if (cached_has_bits & 0x00000010u) {
      txsensorid_ = from.txsensorid_;
    }
    if (cached_has_bits & 0x00000020u) {
      rxsensorid_ = from.rxsensorid_;
    }
    if (cached_has_bits & 0x00000040u) {
      amplitude_ = from.amplitude_;
    }
    if (cached_has_bits & 0x00000080u) {
      relecutimestamp_us_ = from.relecutimestamp_us_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  if (cached_has_bits & 0x00000100u) {
    _internal_set_codingconfidence(from._internal_codingconfidence());
  }
}

void UsProcessingFilteredEcho::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void UsProcessingFilteredEcho::CopyFrom(const UsProcessingFilteredEcho& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool UsProcessingFilteredEcho::IsInitialized() const {
  return true;
}

void UsProcessingFilteredEcho::InternalSwap(UsProcessingFilteredEcho* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(phasederivative_, other->phasederivative_);
  swap(timeconfidence_, other->timeconfidence_);
  swap(trackid_, other->trackid_);
  swap(tof_us_, other->tof_us_);
  swap(txsensorid_, other->txsensorid_);
  swap(rxsensorid_, other->rxsensorid_);
  swap(amplitude_, other->amplitude_);
  swap(relecutimestamp_us_, other->relecutimestamp_us_);
  swap(codingconfidence_, other->codingconfidence_);
}

::PROTOBUF_NAMESPACE_ID::Metadata UsProcessingFilteredEcho::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void UsProcessingFilteredEcho_array_port::InitAsDefaultInstance() {
}
class UsProcessingFilteredEcho_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<UsProcessingFilteredEcho_array_port>()._has_bits_);
};

UsProcessingFilteredEcho_array_port::UsProcessingFilteredEcho_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho_array_port)
}
UsProcessingFilteredEcho_array_port::UsProcessingFilteredEcho_array_port(const UsProcessingFilteredEcho_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho_array_port)
}

void UsProcessingFilteredEcho_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_UsProcessingFilteredEcho_array_port_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto.base);
}

UsProcessingFilteredEcho_array_port::~UsProcessingFilteredEcho_array_port() {
  // @@protoc_insertion_point(destructor:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho_array_port)
  SharedDtor();
}

void UsProcessingFilteredEcho_array_port::SharedDtor() {
}

void UsProcessingFilteredEcho_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const UsProcessingFilteredEcho_array_port& UsProcessingFilteredEcho_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_UsProcessingFilteredEcho_array_port_us_5fprocessing_2fus_5fprocessing_5ffiltered_5fecho_2eproto.base);
  return *internal_default_instance();
}


void UsProcessingFilteredEcho_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* UsProcessingFilteredEcho_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho data = 1073;
      case 1073:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 138)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<8586>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* UsProcessingFilteredEcho_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho data = 1073;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1073, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho_array_port)
  return target;
}

size_t UsProcessingFilteredEcho_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho data = 1073;
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

void UsProcessingFilteredEcho_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const UsProcessingFilteredEcho_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<UsProcessingFilteredEcho_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho_array_port)
    MergeFrom(*source);
  }
}

void UsProcessingFilteredEcho_array_port::MergeFrom(const UsProcessingFilteredEcho_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void UsProcessingFilteredEcho_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void UsProcessingFilteredEcho_array_port::CopyFrom(const UsProcessingFilteredEcho_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool UsProcessingFilteredEcho_array_port::IsInitialized() const {
  return true;
}

void UsProcessingFilteredEcho_array_port::InternalSwap(UsProcessingFilteredEcho_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata UsProcessingFilteredEcho_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace us_processing_filtered_echo
}  // namespace us_processing
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho* Arena::CreateMaybeMessage< ::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho >(Arena* arena) {
  return Arena::CreateInternal< ::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho_array_port* Arena::CreateMaybeMessage< ::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::us_processing::us_processing_filtered_echo::UsProcessingFilteredEcho_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
