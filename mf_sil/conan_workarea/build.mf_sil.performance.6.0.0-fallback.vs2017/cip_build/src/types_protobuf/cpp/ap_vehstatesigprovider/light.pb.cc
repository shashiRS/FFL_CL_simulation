// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/light.proto

#include "ap_vehstatesigprovider/light.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2flight_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Light_ap_5fvehstatesigprovider_2flight_2eproto;
namespace pb {
namespace ap_vehstatesigprovider {
namespace light {
class LightDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Light> _instance;
} _Light_default_instance_;
class Light_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Light_array_port> _instance;
} _Light_array_port_default_instance_;
}  // namespace light
}  // namespace ap_vehstatesigprovider
}  // namespace pb
static void InitDefaultsscc_info_Light_ap_5fvehstatesigprovider_2flight_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_vehstatesigprovider::light::_Light_default_instance_;
    new (ptr) ::pb::ap_vehstatesigprovider::light::Light();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_vehstatesigprovider::light::Light::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Light_ap_5fvehstatesigprovider_2flight_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_Light_ap_5fvehstatesigprovider_2flight_2eproto}, {}};

static void InitDefaultsscc_info_Light_array_port_ap_5fvehstatesigprovider_2flight_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_vehstatesigprovider::light::_Light_array_port_default_instance_;
    new (ptr) ::pb::ap_vehstatesigprovider::light::Light_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_vehstatesigprovider::light::Light_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_Light_array_port_ap_5fvehstatesigprovider_2flight_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_Light_array_port_ap_5fvehstatesigprovider_2flight_2eproto}, {
      &scc_info_Light_ap_5fvehstatesigprovider_2flight_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ap_5fvehstatesigprovider_2flight_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ap_5fvehstatesigprovider_2flight_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5fvehstatesigprovider_2flight_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5fvehstatesigprovider_2flight_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light, lowbeamon_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light, highbeamon_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light, indicatorlefton_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light, indicatorrighton_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light, brakelighton_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light, frontfoglighton_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light, rearfoglighton_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light, daytimerunninglightstateon_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light, positionlampon_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light, reverselampon_nu_),
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9,
  0,
  1,
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::light::Light_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 15, sizeof(::pb::ap_vehstatesigprovider::light::Light)},
  { 25, 31, sizeof(::pb::ap_vehstatesigprovider::light::Light_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_vehstatesigprovider::light::_Light_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_vehstatesigprovider::light::_Light_array_port_default_instance_),
};

const char descriptor_table_protodef_ap_5fvehstatesigprovider_2flight_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\"ap_vehstatesigprovider/light.proto\022\037pb"
  ".ap_vehstatesigprovider.light\"\243\002\n\005Light\022"
  "\025\n\014lowBeamOn_nu\030\201\033 \001(\010\022\026\n\rhighBeamOn_nu\030"
  "\310\023 \001(\010\022\033\n\022indicatorLeftOn_nu\030\375\005 \001(\010\022\034\n\023i"
  "ndicatorRightOn_nu\030\362\007 \001(\010\022\030\n\017brakeLightO"
  "n_nu\030\247\020 \001(\010\022\033\n\022frontFogLightOn_nu\030\210\027 \001(\010"
  "\022\032\n\021rearFogLightOn_nu\030\276\014 \001(\010\022&\n\035daytimeR"
  "unningLightStateOn_nu\030\265\014 \001(\010\022\032\n\021position"
  "LampOn_nu\030\252\013 \001(\010\022\031\n\020reverseLampOn_nu\030\314\034 "
  "\001(\010\"I\n\020Light_array_port\0225\n\004data\030\316\031 \003(\0132&"
  ".pb.ap_vehstatesigprovider.light.Light"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5fvehstatesigprovider_2flight_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5fvehstatesigprovider_2flight_2eproto_sccs[2] = {
  &scc_info_Light_ap_5fvehstatesigprovider_2flight_2eproto.base,
  &scc_info_Light_array_port_ap_5fvehstatesigprovider_2flight_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5fvehstatesigprovider_2flight_2eproto_once;
static bool descriptor_table_ap_5fvehstatesigprovider_2flight_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2flight_2eproto = {
  &descriptor_table_ap_5fvehstatesigprovider_2flight_2eproto_initialized, descriptor_table_protodef_ap_5fvehstatesigprovider_2flight_2eproto, "ap_vehstatesigprovider/light.proto", 438,
  &descriptor_table_ap_5fvehstatesigprovider_2flight_2eproto_once, descriptor_table_ap_5fvehstatesigprovider_2flight_2eproto_sccs, descriptor_table_ap_5fvehstatesigprovider_2flight_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_ap_5fvehstatesigprovider_2flight_2eproto::offsets,
  file_level_metadata_ap_5fvehstatesigprovider_2flight_2eproto, 2, file_level_enum_descriptors_ap_5fvehstatesigprovider_2flight_2eproto, file_level_service_descriptors_ap_5fvehstatesigprovider_2flight_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5fvehstatesigprovider_2flight_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5fvehstatesigprovider_2flight_2eproto), true);
namespace pb {
namespace ap_vehstatesigprovider {
namespace light {

// ===================================================================

void Light::InitAsDefaultInstance() {
}
class Light::_Internal {
 public:
  using HasBits = decltype(std::declval<Light>()._has_bits_);
  static void set_has_lowbeamon_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_highbeamon_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_indicatorlefton_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_indicatorrighton_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_brakelighton_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static void set_has_frontfoglighton_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 128u;
  }
  static void set_has_rearfoglighton_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 256u;
  }
  static void set_has_daytimerunninglightstateon_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 512u;
  }
  static void set_has_positionlampon_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_reverselampon_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

Light::Light()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_vehstatesigprovider.light.Light)
}
Light::Light(const Light& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&positionlampon_nu_, &from.positionlampon_nu_,
    static_cast<size_t>(reinterpret_cast<char*>(&daytimerunninglightstateon_nu_) -
    reinterpret_cast<char*>(&positionlampon_nu_)) + sizeof(daytimerunninglightstateon_nu_));
  // @@protoc_insertion_point(copy_constructor:pb.ap_vehstatesigprovider.light.Light)
}

void Light::SharedCtor() {
  ::memset(&positionlampon_nu_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&daytimerunninglightstateon_nu_) -
      reinterpret_cast<char*>(&positionlampon_nu_)) + sizeof(daytimerunninglightstateon_nu_));
}

Light::~Light() {
  // @@protoc_insertion_point(destructor:pb.ap_vehstatesigprovider.light.Light)
  SharedDtor();
}

void Light::SharedDtor() {
}

void Light::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Light& Light::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Light_ap_5fvehstatesigprovider_2flight_2eproto.base);
  return *internal_default_instance();
}


void Light::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_vehstatesigprovider.light.Light)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    ::memset(&positionlampon_nu_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&frontfoglighton_nu_) -
        reinterpret_cast<char*>(&positionlampon_nu_)) + sizeof(frontfoglighton_nu_));
  }
  if (cached_has_bits & 0x00000300u) {
    ::memset(&rearfoglighton_nu_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&daytimerunninglightstateon_nu_) -
        reinterpret_cast<char*>(&rearfoglighton_nu_)) + sizeof(daytimerunninglightstateon_nu_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* Light::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional bool indicatorLeftOn_nu = 765;
      case 765:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 232)) {
          _Internal::set_has_indicatorlefton_nu(&has_bits);
          indicatorlefton_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool indicatorRightOn_nu = 1010;
      case 1010:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 144)) {
          _Internal::set_has_indicatorrighton_nu(&has_bits);
          indicatorrighton_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool positionLampOn_nu = 1450;
      case 1450:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 80)) {
          _Internal::set_has_positionlampon_nu(&has_bits);
          positionlampon_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool daytimeRunningLightStateOn_nu = 1589;
      case 1589:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 168)) {
          _Internal::set_has_daytimerunninglightstateon_nu(&has_bits);
          daytimerunninglightstateon_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool rearFogLightOn_nu = 1598;
      case 1598:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 240)) {
          _Internal::set_has_rearfoglighton_nu(&has_bits);
          rearfoglighton_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool brakeLightOn_nu = 2087;
      case 2087:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          _Internal::set_has_brakelighton_nu(&has_bits);
          brakelighton_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool highBeamOn_nu = 2504;
      case 2504:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 64)) {
          _Internal::set_has_highbeamon_nu(&has_bits);
          highbeamon_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool frontFogLightOn_nu = 2952;
      case 2952:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 64)) {
          _Internal::set_has_frontfoglighton_nu(&has_bits);
          frontfoglighton_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool lowBeamOn_nu = 3457;
      case 3457:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_lowbeamon_nu(&has_bits);
          lowbeamon_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool reverseLampOn_nu = 3660;
      case 3660:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 96)) {
          _Internal::set_has_reverselampon_nu(&has_bits);
          reverselampon_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* Light::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_vehstatesigprovider.light.Light)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional bool indicatorLeftOn_nu = 765;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(765, this->_internal_indicatorlefton_nu(), target);
  }

  // optional bool indicatorRightOn_nu = 1010;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1010, this->_internal_indicatorrighton_nu(), target);
  }

  // optional bool positionLampOn_nu = 1450;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1450, this->_internal_positionlampon_nu(), target);
  }

  // optional bool daytimeRunningLightStateOn_nu = 1589;
  if (cached_has_bits & 0x00000200u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1589, this->_internal_daytimerunninglightstateon_nu(), target);
  }

  // optional bool rearFogLightOn_nu = 1598;
  if (cached_has_bits & 0x00000100u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1598, this->_internal_rearfoglighton_nu(), target);
  }

  // optional bool brakeLightOn_nu = 2087;
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(2087, this->_internal_brakelighton_nu(), target);
  }

  // optional bool highBeamOn_nu = 2504;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(2504, this->_internal_highbeamon_nu(), target);
  }

  // optional bool frontFogLightOn_nu = 2952;
  if (cached_has_bits & 0x00000080u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(2952, this->_internal_frontfoglighton_nu(), target);
  }

  // optional bool lowBeamOn_nu = 3457;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3457, this->_internal_lowbeamon_nu(), target);
  }

  // optional bool reverseLampOn_nu = 3660;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3660, this->_internal_reverselampon_nu(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_vehstatesigprovider.light.Light)
  return target;
}

size_t Light::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_vehstatesigprovider.light.Light)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    // optional bool positionLampOn_nu = 1450;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 + 1;
    }

    // optional bool reverseLampOn_nu = 3660;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 + 1;
    }

    // optional bool lowBeamOn_nu = 3457;
    if (cached_has_bits & 0x00000004u) {
      total_size += 3 + 1;
    }

    // optional bool highBeamOn_nu = 2504;
    if (cached_has_bits & 0x00000008u) {
      total_size += 3 + 1;
    }

    // optional bool indicatorLeftOn_nu = 765;
    if (cached_has_bits & 0x00000010u) {
      total_size += 2 + 1;
    }

    // optional bool indicatorRightOn_nu = 1010;
    if (cached_has_bits & 0x00000020u) {
      total_size += 2 + 1;
    }

    // optional bool brakeLightOn_nu = 2087;
    if (cached_has_bits & 0x00000040u) {
      total_size += 3 + 1;
    }

    // optional bool frontFogLightOn_nu = 2952;
    if (cached_has_bits & 0x00000080u) {
      total_size += 3 + 1;
    }

  }
  if (cached_has_bits & 0x00000300u) {
    // optional bool rearFogLightOn_nu = 1598;
    if (cached_has_bits & 0x00000100u) {
      total_size += 2 + 1;
    }

    // optional bool daytimeRunningLightStateOn_nu = 1589;
    if (cached_has_bits & 0x00000200u) {
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

void Light::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_vehstatesigprovider.light.Light)
  GOOGLE_DCHECK_NE(&from, this);
  const Light* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Light>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_vehstatesigprovider.light.Light)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_vehstatesigprovider.light.Light)
    MergeFrom(*source);
  }
}

void Light::MergeFrom(const Light& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_vehstatesigprovider.light.Light)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    if (cached_has_bits & 0x00000001u) {
      positionlampon_nu_ = from.positionlampon_nu_;
    }
    if (cached_has_bits & 0x00000002u) {
      reverselampon_nu_ = from.reverselampon_nu_;
    }
    if (cached_has_bits & 0x00000004u) {
      lowbeamon_nu_ = from.lowbeamon_nu_;
    }
    if (cached_has_bits & 0x00000008u) {
      highbeamon_nu_ = from.highbeamon_nu_;
    }
    if (cached_has_bits & 0x00000010u) {
      indicatorlefton_nu_ = from.indicatorlefton_nu_;
    }
    if (cached_has_bits & 0x00000020u) {
      indicatorrighton_nu_ = from.indicatorrighton_nu_;
    }
    if (cached_has_bits & 0x00000040u) {
      brakelighton_nu_ = from.brakelighton_nu_;
    }
    if (cached_has_bits & 0x00000080u) {
      frontfoglighton_nu_ = from.frontfoglighton_nu_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  if (cached_has_bits & 0x00000300u) {
    if (cached_has_bits & 0x00000100u) {
      rearfoglighton_nu_ = from.rearfoglighton_nu_;
    }
    if (cached_has_bits & 0x00000200u) {
      daytimerunninglightstateon_nu_ = from.daytimerunninglightstateon_nu_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void Light::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_vehstatesigprovider.light.Light)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Light::CopyFrom(const Light& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_vehstatesigprovider.light.Light)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Light::IsInitialized() const {
  return true;
}

void Light::InternalSwap(Light* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(positionlampon_nu_, other->positionlampon_nu_);
  swap(reverselampon_nu_, other->reverselampon_nu_);
  swap(lowbeamon_nu_, other->lowbeamon_nu_);
  swap(highbeamon_nu_, other->highbeamon_nu_);
  swap(indicatorlefton_nu_, other->indicatorlefton_nu_);
  swap(indicatorrighton_nu_, other->indicatorrighton_nu_);
  swap(brakelighton_nu_, other->brakelighton_nu_);
  swap(frontfoglighton_nu_, other->frontfoglighton_nu_);
  swap(rearfoglighton_nu_, other->rearfoglighton_nu_);
  swap(daytimerunninglightstateon_nu_, other->daytimerunninglightstateon_nu_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Light::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void Light_array_port::InitAsDefaultInstance() {
}
class Light_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<Light_array_port>()._has_bits_);
};

Light_array_port::Light_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_vehstatesigprovider.light.Light_array_port)
}
Light_array_port::Light_array_port(const Light_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.ap_vehstatesigprovider.light.Light_array_port)
}

void Light_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_Light_array_port_ap_5fvehstatesigprovider_2flight_2eproto.base);
}

Light_array_port::~Light_array_port() {
  // @@protoc_insertion_point(destructor:pb.ap_vehstatesigprovider.light.Light_array_port)
  SharedDtor();
}

void Light_array_port::SharedDtor() {
}

void Light_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Light_array_port& Light_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Light_array_port_ap_5fvehstatesigprovider_2flight_2eproto.base);
  return *internal_default_instance();
}


void Light_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_vehstatesigprovider.light.Light_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* Light_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.ap_vehstatesigprovider.light.Light data = 3278;
      case 3278:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 114)) {
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

::PROTOBUF_NAMESPACE_ID::uint8* Light_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_vehstatesigprovider.light.Light_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.ap_vehstatesigprovider.light.Light data = 3278;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3278, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_vehstatesigprovider.light.Light_array_port)
  return target;
}

size_t Light_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_vehstatesigprovider.light.Light_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.ap_vehstatesigprovider.light.Light data = 3278;
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

void Light_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_vehstatesigprovider.light.Light_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const Light_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Light_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_vehstatesigprovider.light.Light_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_vehstatesigprovider.light.Light_array_port)
    MergeFrom(*source);
  }
}

void Light_array_port::MergeFrom(const Light_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_vehstatesigprovider.light.Light_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void Light_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_vehstatesigprovider.light.Light_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Light_array_port::CopyFrom(const Light_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_vehstatesigprovider.light.Light_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Light_array_port::IsInitialized() const {
  return true;
}

void Light_array_port::InternalSwap(Light_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Light_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace light
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::ap_vehstatesigprovider::light::Light* Arena::CreateMaybeMessage< ::pb::ap_vehstatesigprovider::light::Light >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_vehstatesigprovider::light::Light >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::ap_vehstatesigprovider::light::Light_array_port* Arena::CreateMaybeMessage< ::pb::ap_vehstatesigprovider::light::Light_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_vehstatesigprovider::light::Light_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>