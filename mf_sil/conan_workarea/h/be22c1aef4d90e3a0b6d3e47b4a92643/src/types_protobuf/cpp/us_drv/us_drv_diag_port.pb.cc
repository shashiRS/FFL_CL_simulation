// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_drv/us_drv_diag_port.proto

#include "us_drv/us_drv_diag_port.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fasic_5ferrors_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<6> scc_info_UsDrvAsicErrors_us_5fdrv_2fus_5fdrv_5fasic_5ferrors_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<5> scc_info_UsDrvDiagPort_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fsensor_5ferrors_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_UsDrvSensorErrors_us_5fdrv_2fus_5fdrv_5fsensor_5ferrors_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fsw_5ferrors_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_UsDrvSwErrors_us_5fdrv_2fus_5fdrv_5fsw_5ferrors_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fvariant_5fdata_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_UsDrvVariantData_us_5fdrv_2fus_5fdrv_5fvariant_5fdata_2eproto;
namespace pb {
namespace us_drv {
namespace us_drv_diag_port {
class UsDrvDiagPortDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<UsDrvDiagPort> _instance;
} _UsDrvDiagPort_default_instance_;
class UsDrvDiagPort_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<UsDrvDiagPort_array_port> _instance;
} _UsDrvDiagPort_array_port_default_instance_;
}  // namespace us_drv_diag_port
}  // namespace us_drv
}  // namespace pb
static void InitDefaultsscc_info_UsDrvDiagPort_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::us_drv::us_drv_diag_port::_UsDrvDiagPort_default_instance_;
    new (ptr) ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<5> scc_info_UsDrvDiagPort_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 5, 0, InitDefaultsscc_info_UsDrvDiagPort_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto}, {
      &scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto.base,
      &scc_info_UsDrvVariantData_us_5fdrv_2fus_5fdrv_5fvariant_5fdata_2eproto.base,
      &scc_info_UsDrvSwErrors_us_5fdrv_2fus_5fdrv_5fsw_5ferrors_2eproto.base,
      &scc_info_UsDrvAsicErrors_us_5fdrv_2fus_5fdrv_5fasic_5ferrors_2eproto.base,
      &scc_info_UsDrvSensorErrors_us_5fdrv_2fus_5fdrv_5fsensor_5ferrors_2eproto.base,}};

static void InitDefaultsscc_info_UsDrvDiagPort_array_port_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::us_drv::us_drv_diag_port::_UsDrvDiagPort_array_port_default_instance_;
    new (ptr) ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_UsDrvDiagPort_array_port_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_UsDrvDiagPort_array_port_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto}, {
      &scc_info_UsDrvDiagPort_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_diag_port::UsDrvDiagPort, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_diag_port::UsDrvDiagPort, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_diag_port::UsDrvDiagPort, uiversionnumber_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_diag_port::UsDrvDiagPort, ssigheader_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_diag_port::UsDrvDiagPort, usdrivervariantdata_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_diag_port::UsDrvDiagPort, usdriverswerrors_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_diag_port::UsDrvDiagPort, asicerrors_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_diag_port::UsDrvDiagPort, sensorerrors_),
  3,
  0,
  1,
  2,
  ~0u,
  ~0u,
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_diag_port::UsDrvDiagPort_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_diag_port::UsDrvDiagPort_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::us_drv::us_drv_diag_port::UsDrvDiagPort_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 11, sizeof(::pb::us_drv::us_drv_diag_port::UsDrvDiagPort)},
  { 17, 23, sizeof(::pb::us_drv::us_drv_diag_port::UsDrvDiagPort_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::us_drv::us_drv_diag_port::_UsDrvDiagPort_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::us_drv::us_drv_diag_port::_UsDrvDiagPort_array_port_default_instance_),
};

const char descriptor_table_protodef_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\035us_drv/us_drv_diag_port.proto\022\032pb.us_d"
  "rv.us_drv_diag_port\032\027eco/signal_header.p"
  "roto\032 us_drv/us_drv_variant_data.proto\032\035"
  "us_drv/us_drv_sw_errors.proto\032\037us_drv/us"
  "_drv_asic_errors.proto\032!us_drv/us_drv_se"
  "nsor_errors.proto\"\205\003\n\rUsDrvDiagPort\022\030\n\017u"
  "iVersionNumber\030\314\020 \001(\r\0227\n\nsSigHeader\030\211\010 \001"
  "(\0132\".pb.eco.signal_header.SignalHeader\022M"
  "\n\023usDriverVariantData\030\261\027 \001(\0132/.pb.us_drv"
  ".us_drv_variant_data.UsDrvVariantData\022D\n"
  "\020usDriverSwErrors\030\205\034 \001(\0132).pb.us_drv.us_"
  "drv_sw_errors.UsDrvSwErrors\022B\n\nasicError"
  "s\030\333\034 \003(\0132-.pb.us_drv.us_drv_asic_errors."
  "UsDrvAsicErrors\022H\n\014sensorErrors\030\233\001 \003(\01321"
  ".pb.us_drv.us_drv_sensor_errors.UsDrvSen"
  "sorErrors\"T\n\030UsDrvDiagPort_array_port\0228\n"
  "\004data\030\332\022 \003(\0132).pb.us_drv.us_drv_diag_por"
  "t.UsDrvDiagPort"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto_deps[5] = {
  &::descriptor_table_eco_2fsignal_5fheader_2eproto,
  &::descriptor_table_us_5fdrv_2fus_5fdrv_5fasic_5ferrors_2eproto,
  &::descriptor_table_us_5fdrv_2fus_5fdrv_5fsensor_5ferrors_2eproto,
  &::descriptor_table_us_5fdrv_2fus_5fdrv_5fsw_5ferrors_2eproto,
  &::descriptor_table_us_5fdrv_2fus_5fdrv_5fvariant_5fdata_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto_sccs[2] = {
  &scc_info_UsDrvDiagPort_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto.base,
  &scc_info_UsDrvDiagPort_array_port_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto_once;
static bool descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto = {
  &descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto_initialized, descriptor_table_protodef_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto, "us_drv/us_drv_diag_port.proto", 695,
  &descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto_once, descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto_sccs, descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto_deps, 2, 5,
  schemas, file_default_instances, TableStruct_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto::offsets,
  file_level_metadata_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto, 2, file_level_enum_descriptors_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto, file_level_service_descriptors_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto), true);
namespace pb {
namespace us_drv {
namespace us_drv_diag_port {

// ===================================================================

void UsDrvDiagPort::InitAsDefaultInstance() {
  ::pb::us_drv::us_drv_diag_port::_UsDrvDiagPort_default_instance_._instance.get_mutable()->ssigheader_ = const_cast< ::pb::eco::signal_header::SignalHeader*>(
      ::pb::eco::signal_header::SignalHeader::internal_default_instance());
  ::pb::us_drv::us_drv_diag_port::_UsDrvDiagPort_default_instance_._instance.get_mutable()->usdrivervariantdata_ = const_cast< ::pb::us_drv::us_drv_variant_data::UsDrvVariantData*>(
      ::pb::us_drv::us_drv_variant_data::UsDrvVariantData::internal_default_instance());
  ::pb::us_drv::us_drv_diag_port::_UsDrvDiagPort_default_instance_._instance.get_mutable()->usdriverswerrors_ = const_cast< ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors*>(
      ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors::internal_default_instance());
}
class UsDrvDiagPort::_Internal {
 public:
  using HasBits = decltype(std::declval<UsDrvDiagPort>()._has_bits_);
  static void set_has_uiversionnumber(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static const ::pb::eco::signal_header::SignalHeader& ssigheader(const UsDrvDiagPort* msg);
  static void set_has_ssigheader(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::pb::us_drv::us_drv_variant_data::UsDrvVariantData& usdrivervariantdata(const UsDrvDiagPort* msg);
  static void set_has_usdrivervariantdata(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors& usdriverswerrors(const UsDrvDiagPort* msg);
  static void set_has_usdriverswerrors(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::pb::eco::signal_header::SignalHeader&
UsDrvDiagPort::_Internal::ssigheader(const UsDrvDiagPort* msg) {
  return *msg->ssigheader_;
}
const ::pb::us_drv::us_drv_variant_data::UsDrvVariantData&
UsDrvDiagPort::_Internal::usdrivervariantdata(const UsDrvDiagPort* msg) {
  return *msg->usdrivervariantdata_;
}
const ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors&
UsDrvDiagPort::_Internal::usdriverswerrors(const UsDrvDiagPort* msg) {
  return *msg->usdriverswerrors_;
}
void UsDrvDiagPort::clear_ssigheader() {
  if (ssigheader_ != nullptr) ssigheader_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void UsDrvDiagPort::clear_usdrivervariantdata() {
  if (usdrivervariantdata_ != nullptr) usdrivervariantdata_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
void UsDrvDiagPort::clear_usdriverswerrors() {
  if (usdriverswerrors_ != nullptr) usdriverswerrors_->Clear();
  _has_bits_[0] &= ~0x00000004u;
}
void UsDrvDiagPort::clear_asicerrors() {
  asicerrors_.Clear();
}
void UsDrvDiagPort::clear_sensorerrors() {
  sensorerrors_.Clear();
}
UsDrvDiagPort::UsDrvDiagPort()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
}
UsDrvDiagPort::UsDrvDiagPort(const UsDrvDiagPort& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      sensorerrors_(from.sensorerrors_),
      asicerrors_(from.asicerrors_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_ssigheader()) {
    ssigheader_ = new ::pb::eco::signal_header::SignalHeader(*from.ssigheader_);
  } else {
    ssigheader_ = nullptr;
  }
  if (from._internal_has_usdrivervariantdata()) {
    usdrivervariantdata_ = new ::pb::us_drv::us_drv_variant_data::UsDrvVariantData(*from.usdrivervariantdata_);
  } else {
    usdrivervariantdata_ = nullptr;
  }
  if (from._internal_has_usdriverswerrors()) {
    usdriverswerrors_ = new ::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors(*from.usdriverswerrors_);
  } else {
    usdriverswerrors_ = nullptr;
  }
  uiversionnumber_ = from.uiversionnumber_;
  // @@protoc_insertion_point(copy_constructor:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
}

void UsDrvDiagPort::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_UsDrvDiagPort_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto.base);
  ::memset(&ssigheader_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&uiversionnumber_) -
      reinterpret_cast<char*>(&ssigheader_)) + sizeof(uiversionnumber_));
}

UsDrvDiagPort::~UsDrvDiagPort() {
  // @@protoc_insertion_point(destructor:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
  SharedDtor();
}

void UsDrvDiagPort::SharedDtor() {
  if (this != internal_default_instance()) delete ssigheader_;
  if (this != internal_default_instance()) delete usdrivervariantdata_;
  if (this != internal_default_instance()) delete usdriverswerrors_;
}

void UsDrvDiagPort::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const UsDrvDiagPort& UsDrvDiagPort::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_UsDrvDiagPort_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto.base);
  return *internal_default_instance();
}


void UsDrvDiagPort::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  sensorerrors_.Clear();
  asicerrors_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(ssigheader_ != nullptr);
      ssigheader_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(usdrivervariantdata_ != nullptr);
      usdrivervariantdata_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(usdriverswerrors_ != nullptr);
      usdriverswerrors_->Clear();
    }
  }
  uiversionnumber_ = 0u;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* UsDrvDiagPort::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.us_drv.us_drv_sensor_errors.UsDrvSensorErrors sensorErrors = 155;
      case 155:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 218)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_sensorerrors(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<1242>(ptr));
        } else goto handle_unusual;
        continue;
      // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
      case 1033:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 74)) {
          ptr = ctx->ParseMessage(_internal_mutable_ssigheader(), ptr);
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
      // optional .pb.us_drv.us_drv_variant_data.UsDrvVariantData usDriverVariantData = 2993;
      case 2993:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 138)) {
          ptr = ctx->ParseMessage(_internal_mutable_usdrivervariantdata(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.us_drv.us_drv_sw_errors.UsDrvSwErrors usDriverSwErrors = 3589;
      case 3589:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 42)) {
          ptr = ctx->ParseMessage(_internal_mutable_usdriverswerrors(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .pb.us_drv.us_drv_asic_errors.UsDrvAsicErrors asicErrors = 3675;
      case 3675:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 218)) {
          ptr = ctx->ParseMessage(_internal_add_asicerrors(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* UsDrvDiagPort::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.us_drv.us_drv_sensor_errors.UsDrvSensorErrors sensorErrors = 155;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_sensorerrors_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(155, this->_internal_sensorerrors(i), target, stream);
  }

  cached_has_bits = _has_bits_[0];
  // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1033, _Internal::ssigheader(this), target, stream);
  }

  // optional uint32 uiVersionNumber = 2124;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2124, this->_internal_uiversionnumber(), target);
  }

  // optional .pb.us_drv.us_drv_variant_data.UsDrvVariantData usDriverVariantData = 2993;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2993, _Internal::usdrivervariantdata(this), target, stream);
  }

  // optional .pb.us_drv.us_drv_sw_errors.UsDrvSwErrors usDriverSwErrors = 3589;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3589, _Internal::usdriverswerrors(this), target, stream);
  }

  // repeated .pb.us_drv.us_drv_asic_errors.UsDrvAsicErrors asicErrors = 3675;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_asicerrors_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3675, this->_internal_asicerrors(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
  return target;
}

size_t UsDrvDiagPort::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.us_drv.us_drv_sensor_errors.UsDrvSensorErrors sensorErrors = 155;
  total_size += 2UL * this->_internal_sensorerrors_size();
  for (const auto& msg : this->sensorerrors_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .pb.us_drv.us_drv_asic_errors.UsDrvAsicErrors asicErrors = 3675;
  total_size += 3UL * this->_internal_asicerrors_size();
  for (const auto& msg : this->asicerrors_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *ssigheader_);
    }

    // optional .pb.us_drv.us_drv_variant_data.UsDrvVariantData usDriverVariantData = 2993;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *usdrivervariantdata_);
    }

    // optional .pb.us_drv.us_drv_sw_errors.UsDrvSwErrors usDriverSwErrors = 3589;
    if (cached_has_bits & 0x00000004u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *usdriverswerrors_);
    }

    // optional uint32 uiVersionNumber = 2124;
    if (cached_has_bits & 0x00000008u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_uiversionnumber());
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

void UsDrvDiagPort::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
  GOOGLE_DCHECK_NE(&from, this);
  const UsDrvDiagPort* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<UsDrvDiagPort>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
    MergeFrom(*source);
  }
}

void UsDrvDiagPort::MergeFrom(const UsDrvDiagPort& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  sensorerrors_.MergeFrom(from.sensorerrors_);
  asicerrors_.MergeFrom(from.asicerrors_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_ssigheader()->::pb::eco::signal_header::SignalHeader::MergeFrom(from._internal_ssigheader());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_usdrivervariantdata()->::pb::us_drv::us_drv_variant_data::UsDrvVariantData::MergeFrom(from._internal_usdrivervariantdata());
    }
    if (cached_has_bits & 0x00000004u) {
      _internal_mutable_usdriverswerrors()->::pb::us_drv::us_drv_sw_errors::UsDrvSwErrors::MergeFrom(from._internal_usdriverswerrors());
    }
    if (cached_has_bits & 0x00000008u) {
      uiversionnumber_ = from.uiversionnumber_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void UsDrvDiagPort::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void UsDrvDiagPort::CopyFrom(const UsDrvDiagPort& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool UsDrvDiagPort::IsInitialized() const {
  return true;
}

void UsDrvDiagPort::InternalSwap(UsDrvDiagPort* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  sensorerrors_.InternalSwap(&other->sensorerrors_);
  asicerrors_.InternalSwap(&other->asicerrors_);
  swap(ssigheader_, other->ssigheader_);
  swap(usdrivervariantdata_, other->usdrivervariantdata_);
  swap(usdriverswerrors_, other->usdriverswerrors_);
  swap(uiversionnumber_, other->uiversionnumber_);
}

::PROTOBUF_NAMESPACE_ID::Metadata UsDrvDiagPort::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void UsDrvDiagPort_array_port::InitAsDefaultInstance() {
}
class UsDrvDiagPort_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<UsDrvDiagPort_array_port>()._has_bits_);
};

UsDrvDiagPort_array_port::UsDrvDiagPort_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
}
UsDrvDiagPort_array_port::UsDrvDiagPort_array_port(const UsDrvDiagPort_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
}

void UsDrvDiagPort_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_UsDrvDiagPort_array_port_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto.base);
}

UsDrvDiagPort_array_port::~UsDrvDiagPort_array_port() {
  // @@protoc_insertion_point(destructor:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
  SharedDtor();
}

void UsDrvDiagPort_array_port::SharedDtor() {
}

void UsDrvDiagPort_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const UsDrvDiagPort_array_port& UsDrvDiagPort_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_UsDrvDiagPort_array_port_us_5fdrv_2fus_5fdrv_5fdiag_5fport_2eproto.base);
  return *internal_default_instance();
}


void UsDrvDiagPort_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* UsDrvDiagPort_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.us_drv.us_drv_diag_port.UsDrvDiagPort data = 2394;
      case 2394:
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

::PROTOBUF_NAMESPACE_ID::uint8* UsDrvDiagPort_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.us_drv.us_drv_diag_port.UsDrvDiagPort data = 2394;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2394, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
  return target;
}

size_t UsDrvDiagPort_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.us_drv.us_drv_diag_port.UsDrvDiagPort data = 2394;
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

void UsDrvDiagPort_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const UsDrvDiagPort_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<UsDrvDiagPort_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
    MergeFrom(*source);
  }
}

void UsDrvDiagPort_array_port::MergeFrom(const UsDrvDiagPort_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void UsDrvDiagPort_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void UsDrvDiagPort_array_port::CopyFrom(const UsDrvDiagPort_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool UsDrvDiagPort_array_port::IsInitialized() const {
  return true;
}

void UsDrvDiagPort_array_port::InternalSwap(UsDrvDiagPort_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata UsDrvDiagPort_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace us_drv_diag_port
}  // namespace us_drv
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort* Arena::CreateMaybeMessage< ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort >(Arena* arena) {
  return Arena::CreateInternal< ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort_array_port* Arena::CreateMaybeMessage< ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::us_drv::us_drv_diag_port::UsDrvDiagPort_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
