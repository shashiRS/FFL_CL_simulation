// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_tonh/speaker_output.proto

#include "mf_tonh/speaker_output.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_mf_5ftonh_2fspeaker_5foutput_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SpeakerOutput_mf_5ftonh_2fspeaker_5foutput_2eproto;
namespace pb {
namespace mf_tonh {
namespace speaker_output {
class SpeakerOutputDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<SpeakerOutput> _instance;
} _SpeakerOutput_default_instance_;
class SpeakerOutput_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<SpeakerOutput_array_port> _instance;
} _SpeakerOutput_array_port_default_instance_;
}  // namespace speaker_output
}  // namespace mf_tonh
}  // namespace pb
static void InitDefaultsscc_info_SpeakerOutput_mf_5ftonh_2fspeaker_5foutput_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_tonh::speaker_output::_SpeakerOutput_default_instance_;
    new (ptr) ::pb::mf_tonh::speaker_output::SpeakerOutput();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_tonh::speaker_output::SpeakerOutput::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SpeakerOutput_mf_5ftonh_2fspeaker_5foutput_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_SpeakerOutput_mf_5ftonh_2fspeaker_5foutput_2eproto}, {}};

static void InitDefaultsscc_info_SpeakerOutput_array_port_mf_5ftonh_2fspeaker_5foutput_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_tonh::speaker_output::_SpeakerOutput_array_port_default_instance_;
    new (ptr) ::pb::mf_tonh::speaker_output::SpeakerOutput_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_tonh::speaker_output::SpeakerOutput_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_SpeakerOutput_array_port_mf_5ftonh_2fspeaker_5foutput_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_SpeakerOutput_array_port_mf_5ftonh_2fspeaker_5foutput_2eproto}, {
      &scc_info_SpeakerOutput_mf_5ftonh_2fspeaker_5foutput_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_mf_5ftonh_2fspeaker_5foutput_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_mf_5ftonh_2fspeaker_5foutput_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_mf_5ftonh_2fspeaker_5foutput_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_mf_5ftonh_2fspeaker_5foutput_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::mf_tonh::speaker_output::SpeakerOutput, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_tonh::speaker_output::SpeakerOutput, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_tonh::speaker_output::SpeakerOutput, pitch_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_tonh::speaker_output::SpeakerOutput, volume_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_tonh::speaker_output::SpeakerOutput, soundon_nu_),
  0,
  2,
  1,
  PROTOBUF_FIELD_OFFSET(::pb::mf_tonh::speaker_output::SpeakerOutput_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_tonh::speaker_output::SpeakerOutput_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_tonh::speaker_output::SpeakerOutput_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::pb::mf_tonh::speaker_output::SpeakerOutput)},
  { 11, 17, sizeof(::pb::mf_tonh::speaker_output::SpeakerOutput_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_tonh::speaker_output::_SpeakerOutput_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_tonh::speaker_output::_SpeakerOutput_array_port_default_instance_),
};

const char descriptor_table_protodef_mf_5ftonh_2fspeaker_5foutput_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\034mf_tonh/speaker_output.proto\022\031pb.mf_to"
  "nh.speaker_output\"K\n\rSpeakerOutput\022\021\n\010pi"
  "tch_nu\030\340\037 \001(\r\022\022\n\tvolume_nu\030\306\020 \001(\r\022\023\n\nsou"
  "ndOn_nu\030\354\006 \001(\010\"S\n\030SpeakerOutput_array_po"
  "rt\0227\n\004data\030\304\017 \003(\0132(.pb.mf_tonh.speaker_o"
  "utput.SpeakerOutput"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_mf_5ftonh_2fspeaker_5foutput_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_mf_5ftonh_2fspeaker_5foutput_2eproto_sccs[2] = {
  &scc_info_SpeakerOutput_mf_5ftonh_2fspeaker_5foutput_2eproto.base,
  &scc_info_SpeakerOutput_array_port_mf_5ftonh_2fspeaker_5foutput_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_mf_5ftonh_2fspeaker_5foutput_2eproto_once;
static bool descriptor_table_mf_5ftonh_2fspeaker_5foutput_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5ftonh_2fspeaker_5foutput_2eproto = {
  &descriptor_table_mf_5ftonh_2fspeaker_5foutput_2eproto_initialized, descriptor_table_protodef_mf_5ftonh_2fspeaker_5foutput_2eproto, "mf_tonh/speaker_output.proto", 219,
  &descriptor_table_mf_5ftonh_2fspeaker_5foutput_2eproto_once, descriptor_table_mf_5ftonh_2fspeaker_5foutput_2eproto_sccs, descriptor_table_mf_5ftonh_2fspeaker_5foutput_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_mf_5ftonh_2fspeaker_5foutput_2eproto::offsets,
  file_level_metadata_mf_5ftonh_2fspeaker_5foutput_2eproto, 2, file_level_enum_descriptors_mf_5ftonh_2fspeaker_5foutput_2eproto, file_level_service_descriptors_mf_5ftonh_2fspeaker_5foutput_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_mf_5ftonh_2fspeaker_5foutput_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_mf_5ftonh_2fspeaker_5foutput_2eproto), true);
namespace pb {
namespace mf_tonh {
namespace speaker_output {

// ===================================================================

void SpeakerOutput::InitAsDefaultInstance() {
}
class SpeakerOutput::_Internal {
 public:
  using HasBits = decltype(std::declval<SpeakerOutput>()._has_bits_);
  static void set_has_pitch_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_volume_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_soundon_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

SpeakerOutput::SpeakerOutput()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_tonh.speaker_output.SpeakerOutput)
}
SpeakerOutput::SpeakerOutput(const SpeakerOutput& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&pitch_nu_, &from.pitch_nu_,
    static_cast<size_t>(reinterpret_cast<char*>(&volume_nu_) -
    reinterpret_cast<char*>(&pitch_nu_)) + sizeof(volume_nu_));
  // @@protoc_insertion_point(copy_constructor:pb.mf_tonh.speaker_output.SpeakerOutput)
}

void SpeakerOutput::SharedCtor() {
  ::memset(&pitch_nu_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&volume_nu_) -
      reinterpret_cast<char*>(&pitch_nu_)) + sizeof(volume_nu_));
}

SpeakerOutput::~SpeakerOutput() {
  // @@protoc_insertion_point(destructor:pb.mf_tonh.speaker_output.SpeakerOutput)
  SharedDtor();
}

void SpeakerOutput::SharedDtor() {
}

void SpeakerOutput::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const SpeakerOutput& SpeakerOutput::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_SpeakerOutput_mf_5ftonh_2fspeaker_5foutput_2eproto.base);
  return *internal_default_instance();
}


void SpeakerOutput::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_tonh.speaker_output.SpeakerOutput)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    ::memset(&pitch_nu_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&volume_nu_) -
        reinterpret_cast<char*>(&pitch_nu_)) + sizeof(volume_nu_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* SpeakerOutput::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional bool soundOn_nu = 876;
      case 876:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 96)) {
          _Internal::set_has_soundon_nu(&has_bits);
          soundon_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 volume_nu = 2118;
      case 2118:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 48)) {
          _Internal::set_has_volume_nu(&has_bits);
          volume_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 pitch_nu = 4064;
      case 4064:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 0)) {
          _Internal::set_has_pitch_nu(&has_bits);
          pitch_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* SpeakerOutput::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_tonh.speaker_output.SpeakerOutput)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional bool soundOn_nu = 876;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(876, this->_internal_soundon_nu(), target);
  }

  // optional uint32 volume_nu = 2118;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2118, this->_internal_volume_nu(), target);
  }

  // optional uint32 pitch_nu = 4064;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(4064, this->_internal_pitch_nu(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_tonh.speaker_output.SpeakerOutput)
  return target;
}

size_t SpeakerOutput::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_tonh.speaker_output.SpeakerOutput)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional uint32 pitch_nu = 4064;
    if (cached_has_bits & 0x00000001u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_pitch_nu());
    }

    // optional bool soundOn_nu = 876;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 + 1;
    }

    // optional uint32 volume_nu = 2118;
    if (cached_has_bits & 0x00000004u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_volume_nu());
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

void SpeakerOutput::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_tonh.speaker_output.SpeakerOutput)
  GOOGLE_DCHECK_NE(&from, this);
  const SpeakerOutput* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<SpeakerOutput>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_tonh.speaker_output.SpeakerOutput)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_tonh.speaker_output.SpeakerOutput)
    MergeFrom(*source);
  }
}

void SpeakerOutput::MergeFrom(const SpeakerOutput& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_tonh.speaker_output.SpeakerOutput)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      pitch_nu_ = from.pitch_nu_;
    }
    if (cached_has_bits & 0x00000002u) {
      soundon_nu_ = from.soundon_nu_;
    }
    if (cached_has_bits & 0x00000004u) {
      volume_nu_ = from.volume_nu_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void SpeakerOutput::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_tonh.speaker_output.SpeakerOutput)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SpeakerOutput::CopyFrom(const SpeakerOutput& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_tonh.speaker_output.SpeakerOutput)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SpeakerOutput::IsInitialized() const {
  return true;
}

void SpeakerOutput::InternalSwap(SpeakerOutput* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(pitch_nu_, other->pitch_nu_);
  swap(soundon_nu_, other->soundon_nu_);
  swap(volume_nu_, other->volume_nu_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SpeakerOutput::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void SpeakerOutput_array_port::InitAsDefaultInstance() {
}
class SpeakerOutput_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<SpeakerOutput_array_port>()._has_bits_);
};

SpeakerOutput_array_port::SpeakerOutput_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_tonh.speaker_output.SpeakerOutput_array_port)
}
SpeakerOutput_array_port::SpeakerOutput_array_port(const SpeakerOutput_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.mf_tonh.speaker_output.SpeakerOutput_array_port)
}

void SpeakerOutput_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_SpeakerOutput_array_port_mf_5ftonh_2fspeaker_5foutput_2eproto.base);
}

SpeakerOutput_array_port::~SpeakerOutput_array_port() {
  // @@protoc_insertion_point(destructor:pb.mf_tonh.speaker_output.SpeakerOutput_array_port)
  SharedDtor();
}

void SpeakerOutput_array_port::SharedDtor() {
}

void SpeakerOutput_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const SpeakerOutput_array_port& SpeakerOutput_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_SpeakerOutput_array_port_mf_5ftonh_2fspeaker_5foutput_2eproto.base);
  return *internal_default_instance();
}


void SpeakerOutput_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_tonh.speaker_output.SpeakerOutput_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* SpeakerOutput_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.mf_tonh.speaker_output.SpeakerOutput data = 1988;
      case 1988:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<15906>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* SpeakerOutput_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_tonh.speaker_output.SpeakerOutput_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.mf_tonh.speaker_output.SpeakerOutput data = 1988;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1988, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_tonh.speaker_output.SpeakerOutput_array_port)
  return target;
}

size_t SpeakerOutput_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_tonh.speaker_output.SpeakerOutput_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.mf_tonh.speaker_output.SpeakerOutput data = 1988;
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

void SpeakerOutput_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_tonh.speaker_output.SpeakerOutput_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const SpeakerOutput_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<SpeakerOutput_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_tonh.speaker_output.SpeakerOutput_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_tonh.speaker_output.SpeakerOutput_array_port)
    MergeFrom(*source);
  }
}

void SpeakerOutput_array_port::MergeFrom(const SpeakerOutput_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_tonh.speaker_output.SpeakerOutput_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void SpeakerOutput_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_tonh.speaker_output.SpeakerOutput_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SpeakerOutput_array_port::CopyFrom(const SpeakerOutput_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_tonh.speaker_output.SpeakerOutput_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SpeakerOutput_array_port::IsInitialized() const {
  return true;
}

void SpeakerOutput_array_port::InternalSwap(SpeakerOutput_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SpeakerOutput_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace speaker_output
}  // namespace mf_tonh
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::mf_tonh::speaker_output::SpeakerOutput* Arena::CreateMaybeMessage< ::pb::mf_tonh::speaker_output::SpeakerOutput >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_tonh::speaker_output::SpeakerOutput >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::mf_tonh::speaker_output::SpeakerOutput_array_port* Arena::CreateMaybeMessage< ::pb::mf_tonh::speaker_output::SpeakerOutput_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_tonh::speaker_output::SpeakerOutput_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>