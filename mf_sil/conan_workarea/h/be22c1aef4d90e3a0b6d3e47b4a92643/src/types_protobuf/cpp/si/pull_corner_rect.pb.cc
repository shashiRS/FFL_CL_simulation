// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/pull_corner_rect.proto

#include "si/pull_corner_rect.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_si_2fpull_5fcorner_5frect_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_PullCornerRect_si_2fpull_5fcorner_5frect_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_si_2fquadrilateral_5fserializable_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_QuadrilateralSerializable_si_2fquadrilateral_5fserializable_2eproto;
namespace pb {
namespace si {
namespace pull_corner_rect {
class PullCornerRectDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<PullCornerRect> _instance;
} _PullCornerRect_default_instance_;
class PullCornerRect_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<PullCornerRect_array_port> _instance;
} _PullCornerRect_array_port_default_instance_;
}  // namespace pull_corner_rect
}  // namespace si
}  // namespace pb
static void InitDefaultsscc_info_PullCornerRect_si_2fpull_5fcorner_5frect_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::pull_corner_rect::_PullCornerRect_default_instance_;
    new (ptr) ::pb::si::pull_corner_rect::PullCornerRect();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::pull_corner_rect::PullCornerRect::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_PullCornerRect_si_2fpull_5fcorner_5frect_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_PullCornerRect_si_2fpull_5fcorner_5frect_2eproto}, {
      &scc_info_QuadrilateralSerializable_si_2fquadrilateral_5fserializable_2eproto.base,}};

static void InitDefaultsscc_info_PullCornerRect_array_port_si_2fpull_5fcorner_5frect_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::pull_corner_rect::_PullCornerRect_array_port_default_instance_;
    new (ptr) ::pb::si::pull_corner_rect::PullCornerRect_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::pull_corner_rect::PullCornerRect_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_PullCornerRect_array_port_si_2fpull_5fcorner_5frect_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_PullCornerRect_array_port_si_2fpull_5fcorner_5frect_2eproto}, {
      &scc_info_PullCornerRect_si_2fpull_5fcorner_5frect_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_si_2fpull_5fcorner_5frect_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_si_2fpull_5fcorner_5frect_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_si_2fpull_5fcorner_5frect_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_si_2fpull_5fcorner_5frect_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::si::pull_corner_rect::PullCornerRect, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::pull_corner_rect::PullCornerRect, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::pull_corner_rect::PullCornerRect, fl_),
  PROTOBUF_FIELD_OFFSET(::pb::si::pull_corner_rect::PullCornerRect, fr_),
  PROTOBUF_FIELD_OFFSET(::pb::si::pull_corner_rect::PullCornerRect, curb_),
  1,
  0,
  2,
  PROTOBUF_FIELD_OFFSET(::pb::si::pull_corner_rect::PullCornerRect_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::pull_corner_rect::PullCornerRect_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::pull_corner_rect::PullCornerRect_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::pb::si::pull_corner_rect::PullCornerRect)},
  { 11, 17, sizeof(::pb::si::pull_corner_rect::PullCornerRect_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::pull_corner_rect::_PullCornerRect_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::pull_corner_rect::_PullCornerRect_array_port_default_instance_),
};

const char descriptor_table_protodef_si_2fpull_5fcorner_5frect_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\031si/pull_corner_rect.proto\022\026pb.si.pull_"
  "corner_rect\032#si/quadrilateral_serializab"
  "le.proto\"\360\001\n\016PullCornerRect\022H\n\002fl\030\267\017 \001(\013"
  "2;.pb.si.quadrilateral_serializable.Quad"
  "rilateralSerializable\022H\n\002fr\030\310\010 \001(\0132;.pb."
  "si.quadrilateral_serializable.Quadrilate"
  "ralSerializable\022J\n\004curb\030\245\026 \001(\0132;.pb.si.q"
  "uadrilateral_serializable.QuadrilateralS"
  "erializable\"R\n\031PullCornerRect_array_port"
  "\0225\n\004data\030\212\002 \003(\0132&.pb.si.pull_corner_rect"
  ".PullCornerRect"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_si_2fpull_5fcorner_5frect_2eproto_deps[1] = {
  &::descriptor_table_si_2fquadrilateral_5fserializable_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_si_2fpull_5fcorner_5frect_2eproto_sccs[2] = {
  &scc_info_PullCornerRect_si_2fpull_5fcorner_5frect_2eproto.base,
  &scc_info_PullCornerRect_array_port_si_2fpull_5fcorner_5frect_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_si_2fpull_5fcorner_5frect_2eproto_once;
static bool descriptor_table_si_2fpull_5fcorner_5frect_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fpull_5fcorner_5frect_2eproto = {
  &descriptor_table_si_2fpull_5fcorner_5frect_2eproto_initialized, descriptor_table_protodef_si_2fpull_5fcorner_5frect_2eproto, "si/pull_corner_rect.proto", 415,
  &descriptor_table_si_2fpull_5fcorner_5frect_2eproto_once, descriptor_table_si_2fpull_5fcorner_5frect_2eproto_sccs, descriptor_table_si_2fpull_5fcorner_5frect_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_si_2fpull_5fcorner_5frect_2eproto::offsets,
  file_level_metadata_si_2fpull_5fcorner_5frect_2eproto, 2, file_level_enum_descriptors_si_2fpull_5fcorner_5frect_2eproto, file_level_service_descriptors_si_2fpull_5fcorner_5frect_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_si_2fpull_5fcorner_5frect_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_si_2fpull_5fcorner_5frect_2eproto), true);
namespace pb {
namespace si {
namespace pull_corner_rect {

// ===================================================================

void PullCornerRect::InitAsDefaultInstance() {
  ::pb::si::pull_corner_rect::_PullCornerRect_default_instance_._instance.get_mutable()->fl_ = const_cast< ::pb::si::quadrilateral_serializable::QuadrilateralSerializable*>(
      ::pb::si::quadrilateral_serializable::QuadrilateralSerializable::internal_default_instance());
  ::pb::si::pull_corner_rect::_PullCornerRect_default_instance_._instance.get_mutable()->fr_ = const_cast< ::pb::si::quadrilateral_serializable::QuadrilateralSerializable*>(
      ::pb::si::quadrilateral_serializable::QuadrilateralSerializable::internal_default_instance());
  ::pb::si::pull_corner_rect::_PullCornerRect_default_instance_._instance.get_mutable()->curb_ = const_cast< ::pb::si::quadrilateral_serializable::QuadrilateralSerializable*>(
      ::pb::si::quadrilateral_serializable::QuadrilateralSerializable::internal_default_instance());
}
class PullCornerRect::_Internal {
 public:
  using HasBits = decltype(std::declval<PullCornerRect>()._has_bits_);
  static const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& fl(const PullCornerRect* msg);
  static void set_has_fl(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& fr(const PullCornerRect* msg);
  static void set_has_fr(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable& curb(const PullCornerRect* msg);
  static void set_has_curb(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable&
PullCornerRect::_Internal::fl(const PullCornerRect* msg) {
  return *msg->fl_;
}
const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable&
PullCornerRect::_Internal::fr(const PullCornerRect* msg) {
  return *msg->fr_;
}
const ::pb::si::quadrilateral_serializable::QuadrilateralSerializable&
PullCornerRect::_Internal::curb(const PullCornerRect* msg) {
  return *msg->curb_;
}
void PullCornerRect::clear_fl() {
  if (fl_ != nullptr) fl_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
void PullCornerRect::clear_fr() {
  if (fr_ != nullptr) fr_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void PullCornerRect::clear_curb() {
  if (curb_ != nullptr) curb_->Clear();
  _has_bits_[0] &= ~0x00000004u;
}
PullCornerRect::PullCornerRect()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.pull_corner_rect.PullCornerRect)
}
PullCornerRect::PullCornerRect(const PullCornerRect& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_fr()) {
    fr_ = new ::pb::si::quadrilateral_serializable::QuadrilateralSerializable(*from.fr_);
  } else {
    fr_ = nullptr;
  }
  if (from._internal_has_fl()) {
    fl_ = new ::pb::si::quadrilateral_serializable::QuadrilateralSerializable(*from.fl_);
  } else {
    fl_ = nullptr;
  }
  if (from._internal_has_curb()) {
    curb_ = new ::pb::si::quadrilateral_serializable::QuadrilateralSerializable(*from.curb_);
  } else {
    curb_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:pb.si.pull_corner_rect.PullCornerRect)
}

void PullCornerRect::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_PullCornerRect_si_2fpull_5fcorner_5frect_2eproto.base);
  ::memset(&fr_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&curb_) -
      reinterpret_cast<char*>(&fr_)) + sizeof(curb_));
}

PullCornerRect::~PullCornerRect() {
  // @@protoc_insertion_point(destructor:pb.si.pull_corner_rect.PullCornerRect)
  SharedDtor();
}

void PullCornerRect::SharedDtor() {
  if (this != internal_default_instance()) delete fr_;
  if (this != internal_default_instance()) delete fl_;
  if (this != internal_default_instance()) delete curb_;
}

void PullCornerRect::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const PullCornerRect& PullCornerRect::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_PullCornerRect_si_2fpull_5fcorner_5frect_2eproto.base);
  return *internal_default_instance();
}


void PullCornerRect::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.pull_corner_rect.PullCornerRect)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(fr_ != nullptr);
      fr_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(fl_ != nullptr);
      fl_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(curb_ != nullptr);
      curb_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* PullCornerRect::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable fr = 1096;
      case 1096:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 66)) {
          ptr = ctx->ParseMessage(_internal_mutable_fr(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable fl = 1975;
      case 1975:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 186)) {
          ptr = ctx->ParseMessage(_internal_mutable_fl(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable curb = 2853;
      case 2853:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 42)) {
          ptr = ctx->ParseMessage(_internal_mutable_curb(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* PullCornerRect::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.pull_corner_rect.PullCornerRect)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable fr = 1096;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1096, _Internal::fr(this), target, stream);
  }

  // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable fl = 1975;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1975, _Internal::fl(this), target, stream);
  }

  // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable curb = 2853;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2853, _Internal::curb(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.pull_corner_rect.PullCornerRect)
  return target;
}

size_t PullCornerRect::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.pull_corner_rect.PullCornerRect)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable fr = 1096;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *fr_);
    }

    // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable fl = 1975;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *fl_);
    }

    // optional .pb.si.quadrilateral_serializable.QuadrilateralSerializable curb = 2853;
    if (cached_has_bits & 0x00000004u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *curb_);
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

void PullCornerRect::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.pull_corner_rect.PullCornerRect)
  GOOGLE_DCHECK_NE(&from, this);
  const PullCornerRect* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<PullCornerRect>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.pull_corner_rect.PullCornerRect)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.pull_corner_rect.PullCornerRect)
    MergeFrom(*source);
  }
}

void PullCornerRect::MergeFrom(const PullCornerRect& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.pull_corner_rect.PullCornerRect)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_fr()->::pb::si::quadrilateral_serializable::QuadrilateralSerializable::MergeFrom(from._internal_fr());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_fl()->::pb::si::quadrilateral_serializable::QuadrilateralSerializable::MergeFrom(from._internal_fl());
    }
    if (cached_has_bits & 0x00000004u) {
      _internal_mutable_curb()->::pb::si::quadrilateral_serializable::QuadrilateralSerializable::MergeFrom(from._internal_curb());
    }
  }
}

void PullCornerRect::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.pull_corner_rect.PullCornerRect)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PullCornerRect::CopyFrom(const PullCornerRect& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.pull_corner_rect.PullCornerRect)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PullCornerRect::IsInitialized() const {
  return true;
}

void PullCornerRect::InternalSwap(PullCornerRect* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(fr_, other->fr_);
  swap(fl_, other->fl_);
  swap(curb_, other->curb_);
}

::PROTOBUF_NAMESPACE_ID::Metadata PullCornerRect::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void PullCornerRect_array_port::InitAsDefaultInstance() {
}
class PullCornerRect_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<PullCornerRect_array_port>()._has_bits_);
};

PullCornerRect_array_port::PullCornerRect_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.pull_corner_rect.PullCornerRect_array_port)
}
PullCornerRect_array_port::PullCornerRect_array_port(const PullCornerRect_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.si.pull_corner_rect.PullCornerRect_array_port)
}

void PullCornerRect_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_PullCornerRect_array_port_si_2fpull_5fcorner_5frect_2eproto.base);
}

PullCornerRect_array_port::~PullCornerRect_array_port() {
  // @@protoc_insertion_point(destructor:pb.si.pull_corner_rect.PullCornerRect_array_port)
  SharedDtor();
}

void PullCornerRect_array_port::SharedDtor() {
}

void PullCornerRect_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const PullCornerRect_array_port& PullCornerRect_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_PullCornerRect_array_port_si_2fpull_5fcorner_5frect_2eproto.base);
  return *internal_default_instance();
}


void PullCornerRect_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.pull_corner_rect.PullCornerRect_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* PullCornerRect_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.si.pull_corner_rect.PullCornerRect data = 266;
      case 266:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 82)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<2130>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* PullCornerRect_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.pull_corner_rect.PullCornerRect_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.si.pull_corner_rect.PullCornerRect data = 266;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(266, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.pull_corner_rect.PullCornerRect_array_port)
  return target;
}

size_t PullCornerRect_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.pull_corner_rect.PullCornerRect_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.si.pull_corner_rect.PullCornerRect data = 266;
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

void PullCornerRect_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.pull_corner_rect.PullCornerRect_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const PullCornerRect_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<PullCornerRect_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.pull_corner_rect.PullCornerRect_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.pull_corner_rect.PullCornerRect_array_port)
    MergeFrom(*source);
  }
}

void PullCornerRect_array_port::MergeFrom(const PullCornerRect_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.pull_corner_rect.PullCornerRect_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void PullCornerRect_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.pull_corner_rect.PullCornerRect_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PullCornerRect_array_port::CopyFrom(const PullCornerRect_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.pull_corner_rect.PullCornerRect_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PullCornerRect_array_port::IsInitialized() const {
  return true;
}

void PullCornerRect_array_port::InternalSwap(PullCornerRect_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata PullCornerRect_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace pull_corner_rect
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::si::pull_corner_rect::PullCornerRect* Arena::CreateMaybeMessage< ::pb::si::pull_corner_rect::PullCornerRect >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::pull_corner_rect::PullCornerRect >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::si::pull_corner_rect::PullCornerRect_array_port* Arena::CreateMaybeMessage< ::pb::si::pull_corner_rect::PullCornerRect_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::pull_corner_rect::PullCornerRect_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
