// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_drv/us_drv_envelope_data.proto

#include "us_drv/us_drv_envelope_data.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_eco_2fsignal_5fheader_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_UsDrvEnvelopeData_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fenvelope_5fsignal_5fpath_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_UsDrvEnvelopeSignalPath_us_5fdrv_2fus_5fdrv_5fenvelope_5fsignal_5fpath_2eproto;
namespace pb {
namespace us_drv {
namespace us_drv_envelope_data {
class UsDrvEnvelopeDataDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<UsDrvEnvelopeData> _instance;
} _UsDrvEnvelopeData_default_instance_;
class UsDrvEnvelopeData_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<UsDrvEnvelopeData_array_port> _instance;
} _UsDrvEnvelopeData_array_port_default_instance_;
}  // namespace us_drv_envelope_data
}  // namespace us_drv
}  // namespace pb
static void InitDefaultsscc_info_UsDrvEnvelopeData_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::us_drv::us_drv_envelope_data::_UsDrvEnvelopeData_default_instance_;
    new (ptr) ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_UsDrvEnvelopeData_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, 0, InitDefaultsscc_info_UsDrvEnvelopeData_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto}, {
      &scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto.base,
      &scc_info_UsDrvEnvelopeSignalPath_us_5fdrv_2fus_5fdrv_5fenvelope_5fsignal_5fpath_2eproto.base,}};

static void InitDefaultsscc_info_UsDrvEnvelopeData_array_port_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::us_drv::us_drv_envelope_data::_UsDrvEnvelopeData_array_port_default_instance_;
    new (ptr) ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_UsDrvEnvelopeData_array_port_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_UsDrvEnvelopeData_array_port_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto}, {
      &scc_info_UsDrvEnvelopeData_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData, uiversionnumber_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData, ssigheader_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData, sensorstate_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData, numsignalpaths_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData, signalpaths_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData, numsamples_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData, samples_),
  1,
  0,
  ~0u,
  3,
  ~0u,
  2,
  ~0u,
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 12, sizeof(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData)},
  { 19, 25, sizeof(::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::us_drv::us_drv_envelope_data::_UsDrvEnvelopeData_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::us_drv::us_drv_envelope_data::_UsDrvEnvelopeData_array_port_default_instance_),
};

const char descriptor_table_protodef_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n!us_drv/us_drv_envelope_data.proto\022\036pb."
  "us_drv.us_drv_envelope_data\032\027eco/signal_"
  "header.proto\032 us_drv/us_drv_sensor_state"
  ".proto\032(us_drv/us_drv_envelope_signal_pa"
  "th.proto\"\302\002\n\021UsDrvEnvelopeData\022\030\n\017uiVers"
  "ionNumber\030\314\020 \001(\r\0227\n\nsSigHeader\030\211\010 \001(\0132\"."
  "pb.eco.signal_header.SignalHeader\022E\n\013sen"
  "sorState\030\277\030 \003(\0162/.pb.us_drv.us_drv_senso"
  "r_state.UsDrvSensorState\022\027\n\016numSignalPat"
  "hs\030\340\016 \001(\r\022T\n\013signalPaths\030\330\t \003(\0132>.pb.us_"
  "drv.us_drv_envelope_signal_path.UsDrvEnv"
  "elopeSignalPath\022\023\n\nnumSamples\030\341\007 \001(\r\022\017\n\007"
  "samples\030\026 \003(\r\"`\n\034UsDrvEnvelopeData_array"
  "_port\022@\n\004data\030\226\033 \003(\01321.pb.us_drv.us_drv_"
  "envelope_data.UsDrvEnvelopeData"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto_deps[3] = {
  &::descriptor_table_eco_2fsignal_5fheader_2eproto,
  &::descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fsignal_5fpath_2eproto,
  &::descriptor_table_us_5fdrv_2fus_5fdrv_5fsensor_5fstate_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto_sccs[2] = {
  &scc_info_UsDrvEnvelopeData_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto.base,
  &scc_info_UsDrvEnvelopeData_array_port_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto_once;
static bool descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto = {
  &descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto_initialized, descriptor_table_protodef_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto, "us_drv/us_drv_envelope_data.proto", 591,
  &descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto_once, descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto_sccs, descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto_deps, 2, 3,
  schemas, file_default_instances, TableStruct_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto::offsets,
  file_level_metadata_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto, 2, file_level_enum_descriptors_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto, file_level_service_descriptors_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto), true);
namespace pb {
namespace us_drv {
namespace us_drv_envelope_data {

// ===================================================================

void UsDrvEnvelopeData::InitAsDefaultInstance() {
  ::pb::us_drv::us_drv_envelope_data::_UsDrvEnvelopeData_default_instance_._instance.get_mutable()->ssigheader_ = const_cast< ::pb::eco::signal_header::SignalHeader*>(
      ::pb::eco::signal_header::SignalHeader::internal_default_instance());
}
class UsDrvEnvelopeData::_Internal {
 public:
  using HasBits = decltype(std::declval<UsDrvEnvelopeData>()._has_bits_);
  static void set_has_uiversionnumber(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::pb::eco::signal_header::SignalHeader& ssigheader(const UsDrvEnvelopeData* msg);
  static void set_has_ssigheader(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_numsignalpaths(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_numsamples(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::pb::eco::signal_header::SignalHeader&
UsDrvEnvelopeData::_Internal::ssigheader(const UsDrvEnvelopeData* msg) {
  return *msg->ssigheader_;
}
void UsDrvEnvelopeData::clear_ssigheader() {
  if (ssigheader_ != nullptr) ssigheader_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void UsDrvEnvelopeData::clear_signalpaths() {
  signalpaths_.Clear();
}
UsDrvEnvelopeData::UsDrvEnvelopeData()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
}
UsDrvEnvelopeData::UsDrvEnvelopeData(const UsDrvEnvelopeData& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      samples_(from.samples_),
      signalpaths_(from.signalpaths_),
      sensorstate_(from.sensorstate_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_ssigheader()) {
    ssigheader_ = new ::pb::eco::signal_header::SignalHeader(*from.ssigheader_);
  } else {
    ssigheader_ = nullptr;
  }
  ::memcpy(&uiversionnumber_, &from.uiversionnumber_,
    static_cast<size_t>(reinterpret_cast<char*>(&numsignalpaths_) -
    reinterpret_cast<char*>(&uiversionnumber_)) + sizeof(numsignalpaths_));
  // @@protoc_insertion_point(copy_constructor:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
}

void UsDrvEnvelopeData::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_UsDrvEnvelopeData_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto.base);
  ::memset(&ssigheader_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&numsignalpaths_) -
      reinterpret_cast<char*>(&ssigheader_)) + sizeof(numsignalpaths_));
}

UsDrvEnvelopeData::~UsDrvEnvelopeData() {
  // @@protoc_insertion_point(destructor:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
  SharedDtor();
}

void UsDrvEnvelopeData::SharedDtor() {
  if (this != internal_default_instance()) delete ssigheader_;
}

void UsDrvEnvelopeData::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const UsDrvEnvelopeData& UsDrvEnvelopeData::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_UsDrvEnvelopeData_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto.base);
  return *internal_default_instance();
}


void UsDrvEnvelopeData::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  samples_.Clear();
  signalpaths_.Clear();
  sensorstate_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(ssigheader_ != nullptr);
    ssigheader_->Clear();
  }
  if (cached_has_bits & 0x0000000eu) {
    ::memset(&uiversionnumber_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&numsignalpaths_) -
        reinterpret_cast<char*>(&uiversionnumber_)) + sizeof(numsignalpaths_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* UsDrvEnvelopeData::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated uint32 samples = 22;
      case 22:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 176)) {
          ptr -= 2;
          do {
            ptr += 2;
            _internal_add_samples(::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr));
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<176>(ptr));
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 178) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedUInt32Parser(_internal_mutable_samples(), ptr, ctx);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 numSamples = 993;
      case 993:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_numsamples(&has_bits);
          numsamples_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
      case 1033:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 74)) {
          ptr = ctx->ParseMessage(_internal_mutable_ssigheader(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath signalPaths = 1240;
      case 1240:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 194)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_signalpaths(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<9922>(ptr));
        } else goto handle_unusual;
        continue;
      // optional uint32 numSignalPaths = 1888;
      case 1888:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 0)) {
          _Internal::set_has_numsignalpaths(&has_bits);
          numsignalpaths_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 uiVersionNumber = 2124;
      case 2124:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 96)) {
          _Internal::set_has_uiversionnumber(&has_bits);
          uiversionnumber_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .pb.us_drv.us_drv_sensor_state.UsDrvSensorState sensorState = 3135;
      case 3135:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 248)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::us_drv::us_drv_sensor_state::UsDrvSensorState_IsValid(val))) {
            _internal_add_sensorstate(static_cast<::pb::us_drv::us_drv_sensor_state::UsDrvSensorState>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(3135, val, mutable_unknown_fields());
          }
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 250) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedEnumParser(_internal_mutable_sensorstate(), ptr, ctx, ::pb::us_drv::us_drv_sensor_state::UsDrvSensorState_IsValid, &_internal_metadata_, 3135);
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

::PROTOBUF_NAMESPACE_ID::uint8* UsDrvEnvelopeData::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated uint32 samples = 22;
  for (int i = 0, n = this->_internal_samples_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(22, this->_internal_samples(i), target);
  }

  cached_has_bits = _has_bits_[0];
  // optional uint32 numSamples = 993;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(993, this->_internal_numsamples(), target);
  }

  // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1033, _Internal::ssigheader(this), target, stream);
  }

  // repeated .pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath signalPaths = 1240;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_signalpaths_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1240, this->_internal_signalpaths(i), target, stream);
  }

  // optional uint32 numSignalPaths = 1888;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1888, this->_internal_numsignalpaths(), target);
  }

  // optional uint32 uiVersionNumber = 2124;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2124, this->_internal_uiversionnumber(), target);
  }

  // repeated .pb.us_drv.us_drv_sensor_state.UsDrvSensorState sensorState = 3135;
  for (int i = 0, n = this->_internal_sensorstate_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
        3135, this->_internal_sensorstate(i), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
  return target;
}

size_t UsDrvEnvelopeData::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated uint32 samples = 22;
  {
    size_t data_size = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      UInt32Size(this->samples_);
    total_size += 2 *
                  ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(this->_internal_samples_size());
    total_size += data_size;
  }

  // repeated .pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath signalPaths = 1240;
  total_size += 2UL * this->_internal_signalpaths_size();
  for (const auto& msg : this->signalpaths_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .pb.us_drv.us_drv_sensor_state.UsDrvSensorState sensorState = 3135;
  {
    size_t data_size = 0;
    unsigned int count = static_cast<unsigned int>(this->_internal_sensorstate_size());for (unsigned int i = 0; i < count; i++) {
      data_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(
        this->_internal_sensorstate(static_cast<int>(i)));
    }
    total_size += (3UL * count) + data_size;
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *ssigheader_);
    }

    // optional uint32 uiVersionNumber = 2124;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_uiversionnumber());
    }

    // optional uint32 numSamples = 993;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_numsamples());
    }

    // optional uint32 numSignalPaths = 1888;
    if (cached_has_bits & 0x00000008u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_numsignalpaths());
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

void UsDrvEnvelopeData::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
  GOOGLE_DCHECK_NE(&from, this);
  const UsDrvEnvelopeData* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<UsDrvEnvelopeData>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
    MergeFrom(*source);
  }
}

void UsDrvEnvelopeData::MergeFrom(const UsDrvEnvelopeData& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  samples_.MergeFrom(from.samples_);
  signalpaths_.MergeFrom(from.signalpaths_);
  sensorstate_.MergeFrom(from.sensorstate_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_ssigheader()->::pb::eco::signal_header::SignalHeader::MergeFrom(from._internal_ssigheader());
    }
    if (cached_has_bits & 0x00000002u) {
      uiversionnumber_ = from.uiversionnumber_;
    }
    if (cached_has_bits & 0x00000004u) {
      numsamples_ = from.numsamples_;
    }
    if (cached_has_bits & 0x00000008u) {
      numsignalpaths_ = from.numsignalpaths_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void UsDrvEnvelopeData::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void UsDrvEnvelopeData::CopyFrom(const UsDrvEnvelopeData& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool UsDrvEnvelopeData::IsInitialized() const {
  return true;
}

void UsDrvEnvelopeData::InternalSwap(UsDrvEnvelopeData* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  samples_.InternalSwap(&other->samples_);
  signalpaths_.InternalSwap(&other->signalpaths_);
  sensorstate_.InternalSwap(&other->sensorstate_);
  swap(ssigheader_, other->ssigheader_);
  swap(uiversionnumber_, other->uiversionnumber_);
  swap(numsamples_, other->numsamples_);
  swap(numsignalpaths_, other->numsignalpaths_);
}

::PROTOBUF_NAMESPACE_ID::Metadata UsDrvEnvelopeData::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void UsDrvEnvelopeData_array_port::InitAsDefaultInstance() {
}
class UsDrvEnvelopeData_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<UsDrvEnvelopeData_array_port>()._has_bits_);
};

UsDrvEnvelopeData_array_port::UsDrvEnvelopeData_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
}
UsDrvEnvelopeData_array_port::UsDrvEnvelopeData_array_port(const UsDrvEnvelopeData_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
}

void UsDrvEnvelopeData_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_UsDrvEnvelopeData_array_port_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto.base);
}

UsDrvEnvelopeData_array_port::~UsDrvEnvelopeData_array_port() {
  // @@protoc_insertion_point(destructor:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
  SharedDtor();
}

void UsDrvEnvelopeData_array_port::SharedDtor() {
}

void UsDrvEnvelopeData_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const UsDrvEnvelopeData_array_port& UsDrvEnvelopeData_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_UsDrvEnvelopeData_array_port_us_5fdrv_2fus_5fdrv_5fenvelope_5fdata_2eproto.base);
  return *internal_default_instance();
}


void UsDrvEnvelopeData_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* UsDrvEnvelopeData_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData data = 3478;
      case 3478:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 178)) {
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

::PROTOBUF_NAMESPACE_ID::uint8* UsDrvEnvelopeData_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData data = 3478;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3478, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
  return target;
}

size_t UsDrvEnvelopeData_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData data = 3478;
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

void UsDrvEnvelopeData_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const UsDrvEnvelopeData_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<UsDrvEnvelopeData_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
    MergeFrom(*source);
  }
}

void UsDrvEnvelopeData_array_port::MergeFrom(const UsDrvEnvelopeData_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void UsDrvEnvelopeData_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void UsDrvEnvelopeData_array_port::CopyFrom(const UsDrvEnvelopeData_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool UsDrvEnvelopeData_array_port::IsInitialized() const {
  return true;
}

void UsDrvEnvelopeData_array_port::InternalSwap(UsDrvEnvelopeData_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata UsDrvEnvelopeData_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace us_drv_envelope_data
}  // namespace us_drv
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData* Arena::CreateMaybeMessage< ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData >(Arena* arena) {
  return Arena::CreateInternal< ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData_array_port* Arena::CreateMaybeMessage< ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::us_drv::us_drv_envelope_data::UsDrvEnvelopeData_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
