// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: eco/trace_data.proto

#include "eco/trace_data.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_eco_2ftrace_5fdata_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_TraceData_eco_2ftrace_5fdata_2eproto;
namespace pb {
namespace eco {
namespace trace_data {
class TraceDataDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<TraceData> _instance;
} _TraceData_default_instance_;
class TraceData_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<TraceData_array_port> _instance;
} _TraceData_array_port_default_instance_;
}  // namespace trace_data
}  // namespace eco
}  // namespace pb
static void InitDefaultsscc_info_TraceData_eco_2ftrace_5fdata_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::eco::trace_data::_TraceData_default_instance_;
    new (ptr) ::pb::eco::trace_data::TraceData();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::eco::trace_data::TraceData::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_TraceData_eco_2ftrace_5fdata_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_TraceData_eco_2ftrace_5fdata_2eproto}, {
      &scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto.base,}};

static void InitDefaultsscc_info_TraceData_array_port_eco_2ftrace_5fdata_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::eco::trace_data::_TraceData_array_port_default_instance_;
    new (ptr) ::pb::eco::trace_data::TraceData_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::eco::trace_data::TraceData_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_TraceData_array_port_eco_2ftrace_5fdata_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_TraceData_array_port_eco_2ftrace_5fdata_2eproto}, {
      &scc_info_TraceData_eco_2ftrace_5fdata_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_eco_2ftrace_5fdata_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_eco_2ftrace_5fdata_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_eco_2ftrace_5fdata_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_eco_2ftrace_5fdata_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::eco::trace_data::TraceData, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::eco::trace_data::TraceData, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::eco::trace_data::TraceData, sigheader_),
  PROTOBUF_FIELD_OFFSET(::pb::eco::trace_data::TraceData, rangecheckfailures_),
  0,
  ~0u,
  PROTOBUF_FIELD_OFFSET(::pb::eco::trace_data::TraceData_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::eco::trace_data::TraceData_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::eco::trace_data::TraceData_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::pb::eco::trace_data::TraceData)},
  { 9, 15, sizeof(::pb::eco::trace_data::TraceData_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::eco::trace_data::_TraceData_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::eco::trace_data::_TraceData_array_port_default_instance_),
};

const char descriptor_table_protodef_eco_2ftrace_5fdata_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\024eco/trace_data.proto\022\021pb.eco.trace_dat"
  "a\032\027eco/signal_header.proto\"`\n\tTraceData\022"
  "6\n\tsigHeader\030\252\013 \001(\0132\".pb.eco.signal_head"
  "er.SignalHeader\022\033\n\022rangeCheckFailures\030\347\027"
  " \003(\r\"C\n\024TraceData_array_port\022+\n\004data\030\245\025 "
  "\003(\0132\034.pb.eco.trace_data.TraceData"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_eco_2ftrace_5fdata_2eproto_deps[1] = {
  &::descriptor_table_eco_2fsignal_5fheader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_eco_2ftrace_5fdata_2eproto_sccs[2] = {
  &scc_info_TraceData_eco_2ftrace_5fdata_2eproto.base,
  &scc_info_TraceData_array_port_eco_2ftrace_5fdata_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_eco_2ftrace_5fdata_2eproto_once;
static bool descriptor_table_eco_2ftrace_5fdata_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_eco_2ftrace_5fdata_2eproto = {
  &descriptor_table_eco_2ftrace_5fdata_2eproto_initialized, descriptor_table_protodef_eco_2ftrace_5fdata_2eproto, "eco/trace_data.proto", 233,
  &descriptor_table_eco_2ftrace_5fdata_2eproto_once, descriptor_table_eco_2ftrace_5fdata_2eproto_sccs, descriptor_table_eco_2ftrace_5fdata_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_eco_2ftrace_5fdata_2eproto::offsets,
  file_level_metadata_eco_2ftrace_5fdata_2eproto, 2, file_level_enum_descriptors_eco_2ftrace_5fdata_2eproto, file_level_service_descriptors_eco_2ftrace_5fdata_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_eco_2ftrace_5fdata_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_eco_2ftrace_5fdata_2eproto), true);
namespace pb {
namespace eco {
namespace trace_data {

// ===================================================================

void TraceData::InitAsDefaultInstance() {
  ::pb::eco::trace_data::_TraceData_default_instance_._instance.get_mutable()->sigheader_ = const_cast< ::pb::eco::signal_header::SignalHeader*>(
      ::pb::eco::signal_header::SignalHeader::internal_default_instance());
}
class TraceData::_Internal {
 public:
  using HasBits = decltype(std::declval<TraceData>()._has_bits_);
  static const ::pb::eco::signal_header::SignalHeader& sigheader(const TraceData* msg);
  static void set_has_sigheader(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

const ::pb::eco::signal_header::SignalHeader&
TraceData::_Internal::sigheader(const TraceData* msg) {
  return *msg->sigheader_;
}
void TraceData::clear_sigheader() {
  if (sigheader_ != nullptr) sigheader_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
TraceData::TraceData()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.eco.trace_data.TraceData)
}
TraceData::TraceData(const TraceData& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      rangecheckfailures_(from.rangecheckfailures_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_sigheader()) {
    sigheader_ = new ::pb::eco::signal_header::SignalHeader(*from.sigheader_);
  } else {
    sigheader_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:pb.eco.trace_data.TraceData)
}

void TraceData::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_TraceData_eco_2ftrace_5fdata_2eproto.base);
  sigheader_ = nullptr;
}

TraceData::~TraceData() {
  // @@protoc_insertion_point(destructor:pb.eco.trace_data.TraceData)
  SharedDtor();
}

void TraceData::SharedDtor() {
  if (this != internal_default_instance()) delete sigheader_;
}

void TraceData::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const TraceData& TraceData::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_TraceData_eco_2ftrace_5fdata_2eproto.base);
  return *internal_default_instance();
}


void TraceData::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.eco.trace_data.TraceData)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  rangecheckfailures_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(sigheader_ != nullptr);
    sigheader_->Clear();
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* TraceData::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .pb.eco.signal_header.SignalHeader sigHeader = 1450;
      case 1450:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 82)) {
          ptr = ctx->ParseMessage(_internal_mutable_sigheader(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated uint32 rangeCheckFailures = 3047;
      case 3047:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          _internal_add_rangecheckfailures(::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr));
          CHK_(ptr);
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 58) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedUInt32Parser(_internal_mutable_rangecheckfailures(), ptr, ctx);
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

::PROTOBUF_NAMESPACE_ID::uint8* TraceData::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.eco.trace_data.TraceData)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.eco.signal_header.SignalHeader sigHeader = 1450;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1450, _Internal::sigheader(this), target, stream);
  }

  // repeated uint32 rangeCheckFailures = 3047;
  for (int i = 0, n = this->_internal_rangecheckfailures_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(3047, this->_internal_rangecheckfailures(i), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.eco.trace_data.TraceData)
  return target;
}

size_t TraceData::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.eco.trace_data.TraceData)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated uint32 rangeCheckFailures = 3047;
  {
    size_t data_size = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      UInt32Size(this->rangecheckfailures_);
    total_size += 3 *
                  ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(this->_internal_rangecheckfailures_size());
    total_size += data_size;
  }

  // optional .pb.eco.signal_header.SignalHeader sigHeader = 1450;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 2 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *sigheader_);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void TraceData::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.eco.trace_data.TraceData)
  GOOGLE_DCHECK_NE(&from, this);
  const TraceData* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<TraceData>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.eco.trace_data.TraceData)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.eco.trace_data.TraceData)
    MergeFrom(*source);
  }
}

void TraceData::MergeFrom(const TraceData& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.eco.trace_data.TraceData)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  rangecheckfailures_.MergeFrom(from.rangecheckfailures_);
  if (from._internal_has_sigheader()) {
    _internal_mutable_sigheader()->::pb::eco::signal_header::SignalHeader::MergeFrom(from._internal_sigheader());
  }
}

void TraceData::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.eco.trace_data.TraceData)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void TraceData::CopyFrom(const TraceData& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.eco.trace_data.TraceData)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool TraceData::IsInitialized() const {
  return true;
}

void TraceData::InternalSwap(TraceData* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  rangecheckfailures_.InternalSwap(&other->rangecheckfailures_);
  swap(sigheader_, other->sigheader_);
}

::PROTOBUF_NAMESPACE_ID::Metadata TraceData::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void TraceData_array_port::InitAsDefaultInstance() {
}
class TraceData_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<TraceData_array_port>()._has_bits_);
};

TraceData_array_port::TraceData_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.eco.trace_data.TraceData_array_port)
}
TraceData_array_port::TraceData_array_port(const TraceData_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.eco.trace_data.TraceData_array_port)
}

void TraceData_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_TraceData_array_port_eco_2ftrace_5fdata_2eproto.base);
}

TraceData_array_port::~TraceData_array_port() {
  // @@protoc_insertion_point(destructor:pb.eco.trace_data.TraceData_array_port)
  SharedDtor();
}

void TraceData_array_port::SharedDtor() {
}

void TraceData_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const TraceData_array_port& TraceData_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_TraceData_array_port_eco_2ftrace_5fdata_2eproto.base);
  return *internal_default_instance();
}


void TraceData_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.eco.trace_data.TraceData_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* TraceData_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.eco.trace_data.TraceData data = 2725;
      case 2725:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 42)) {
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

::PROTOBUF_NAMESPACE_ID::uint8* TraceData_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.eco.trace_data.TraceData_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.eco.trace_data.TraceData data = 2725;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2725, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.eco.trace_data.TraceData_array_port)
  return target;
}

size_t TraceData_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.eco.trace_data.TraceData_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.eco.trace_data.TraceData data = 2725;
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

void TraceData_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.eco.trace_data.TraceData_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const TraceData_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<TraceData_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.eco.trace_data.TraceData_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.eco.trace_data.TraceData_array_port)
    MergeFrom(*source);
  }
}

void TraceData_array_port::MergeFrom(const TraceData_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.eco.trace_data.TraceData_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void TraceData_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.eco.trace_data.TraceData_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void TraceData_array_port::CopyFrom(const TraceData_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.eco.trace_data.TraceData_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool TraceData_array_port::IsInitialized() const {
  return true;
}

void TraceData_array_port::InternalSwap(TraceData_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata TraceData_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace trace_data
}  // namespace eco
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::eco::trace_data::TraceData* Arena::CreateMaybeMessage< ::pb::eco::trace_data::TraceData >(Arena* arena) {
  return Arena::CreateInternal< ::pb::eco::trace_data::TraceData >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::eco::trace_data::TraceData_array_port* Arena::CreateMaybeMessage< ::pb::eco::trace_data::TraceData_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::eco::trace_data::TraceData_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
