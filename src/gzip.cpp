#include "gzip.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace rr {

namespace {

constexpr std::size_t kWindow = 32768;
constexpr std::size_t kOutCapacity = (std::size_t{1} << 22) + kWindow;
constexpr std::size_t kMaxMatch = 258;
constexpr std::size_t kCopySlack = 16;
constexpr std::size_t kInChunk = std::size_t{1} << 20;
constexpr int kMaxPadBits = 64;

constexpr std::uint16_t kLengthBase[29] = {3,  4,  5,  6,   7,   8,   9,   10,  11, 13,
                                           15, 17, 19, 23,  27,  31,  35,  43,  51, 59,
                                           67, 83, 99, 115, 131, 163, 195, 227, 258};
constexpr std::uint8_t kLengthExtra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2,
                                           2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
constexpr std::uint16_t kDistBase[30] = {
    1,   2,   3,   4,   5,   7,    9,    13,   17,   25,   33,   49,   65,    97,    129,
    193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
constexpr std::uint8_t kDistExtra[30] = {0, 0, 0, 0, 1, 1, 2, 2,  3,  3,  4,  4,  5,  5,  6,
                                         6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};
constexpr std::uint8_t kCodeLengthOrder[19] = {16, 17, 18, 0, 8,  7, 9,  6, 10, 5,
                                               11, 4,  12, 3, 13, 2, 14, 1, 15};

}

GzipSource::GzipSource(std::unique_ptr<ByteSource> input)
    : input_(std::move(input)), in_(kInChunk), out_(kOutCapacity + kCopySlack) {}

Result<std::size_t> GzipSource::Read(char* out, std::size_t size) {
  while (out_read_ == out_write_) {
    if (state_ == State::kDone) return std::size_t{0};
    if (!Fill()) return Status::Error("gzip: " + error_);
  }
  std::size_t n = std::min(size, out_write_ - out_read_);
  std::memcpy(out, out_.data() + out_read_, n);
  out_read_ += n;
  return n;
}

bool GzipSource::Fail(const char* message) {
  if (error_.empty()) error_ = Overrun() ? "unexpected end of input" : message;
  return false;
}

bool GzipSource::Fill() {
  if (!error_.empty()) return false;
  if (out_write_ + kMaxMatch > kOutCapacity) {
    std::memmove(out_.data(), out_.data() + out_write_ - kWindow, kWindow);
    out_read_ = kWindow;
    out_write_ = kWindow;
  }
  std::size_t start = out_write_;
  bool ok = true;
  while (ok && state_ != State::kDone && out_write_ + kMaxMatch <= kOutCapacity) {
    switch (state_) {
      case State::kMemberHeader:
        ok = ReadMemberHeader();
        break;
      case State::kBlockHeader:
        ok = ReadBlockHeader();
        break;
      case State::kStored:
        ok = DecodeStored();
        break;
      case State::kCompressed:
        ok = DecodeCompressed();
        break;
      case State::kTrailer:
        crc_ = Crc32(crc_, out_.data() + start, out_write_ - start);
        start = out_write_;
        ok = ReadTrailer();
        break;
      case State::kDone:
        break;
    }
  }
  crc_ = Crc32(crc_, out_.data() + start, out_write_ - start);
  return ok;
}

bool GzipSource::Refill() {
  if (in_eof_) return false;
  in_pos_ = 0;
  in_end_ = 0;
  Result<std::size_t> got = input_->Read(reinterpret_cast<char*>(in_.data()), in_.size());
  if (!got.ok()) {
    in_eof_ = true;
    return Fail(got.status().message().c_str());
  }
  if (*got == 0) {
    in_eof_ = true;
    return false;
  }
  in_end_ = *got;
  return true;
}

bool GzipSource::Need(int bits) {
  while (nbits_ < bits) {
    if (in_end_ - in_pos_ >= 8) {
      std::uint64_t v;
      std::memcpy(&v, in_.data() + in_pos_, 8);
      bits_ |= v << nbits_;
      in_pos_ += static_cast<std::size_t>((63 - nbits_) >> 3);
      nbits_ |= 56;
      continue;
    }
    if (in_pos_ == in_end_ && !Refill()) {
      if (!error_.empty()) return false;
      if (pad_bits_ >= kMaxPadBits) return Fail("unexpected end of input");
      nbits_ += 8;
      pad_bits_ += 8;
      continue;
    }
    bits_ |= static_cast<std::uint64_t>(in_[in_pos_++]) << nbits_;
    nbits_ += 8;
  }
  return true;
}

bool GzipSource::Take(int bits, std::uint32_t* value) {
  if (!Need(bits)) return false;
  *value = Peek(bits);
  Drop(bits);
  if (Overrun()) return Fail("unexpected end of input");
  return true;
}

bool GzipSource::HasMoreInput() {
  if (nbits_ - pad_bits_ >= 8) return true;
  if (in_pos_ < in_end_) return true;
  return Refill();
}

bool GzipSource::ReadMemberHeader() {
  std::uint32_t id1 = 0, id2 = 0, method = 0, flags = 0, byte = 0;
  if (!Take(8, &id1) || !Take(8, &id2) || !Take(8, &method) || !Take(8, &flags)) return false;
  if (id1 != 0x1f || id2 != 0x8b) return Fail("not a gzip stream");
  if (method != 8) return Fail("unsupported compression method");
  if ((flags & 0xe0) != 0) return Fail("reserved header flags are set");
  for (int i = 0; i < 6; ++i) {
    if (!Take(8, &byte)) return false;
  }
  if ((flags & 4) != 0) {
    std::uint32_t lo = 0, hi = 0;
    if (!Take(8, &lo) || !Take(8, &hi)) return false;
    for (std::uint32_t i = 0; i < (lo | hi << 8); ++i) {
      if (!Take(8, &byte)) return false;
    }
  }
  for (std::uint32_t mask : {8u, 16u}) {
    if ((flags & mask) == 0) continue;
    do {
      if (!Take(8, &byte)) return false;
    } while (byte != 0);
  }
  if ((flags & 2) != 0 && !Take(16, &byte)) return false;
  crc_ = 0;
  member_size_ = 0;
  last_block_ = false;
  state_ = State::kBlockHeader;
  return true;
}

bool GzipSource::ReadBlockHeader() {
  if (last_block_) {
    state_ = State::kTrailer;
    return true;
  }
  std::uint32_t header = 0;
  if (!Take(3, &header)) return false;
  last_block_ = (header & 1) != 0;
  switch (header >> 1) {
    case 0: {
      Drop(nbits_ % 8);
      std::uint32_t length = 0, complement = 0;
      if (!Take(16, &length) || !Take(16, &complement)) return false;
      if ((length ^ 0xffff) != complement) return Fail("stored block length mismatch");
      stored_left_ = length;
      state_ = State::kStored;
      return true;
    }
    case 1:
      lit_ = &FixedLiterals();
      dist_ = &FixedDistances();
      state_ = State::kCompressed;
      return true;
    case 2:
      if (!ReadDynamicTables()) return false;
      lit_ = &dyn_lit_;
      dist_ = &dyn_dist_;
      state_ = State::kCompressed;
      return true;
    default:
      return Fail("invalid block type");
  }
}

bool GzipSource::ReadDynamicTables() {
  std::uint32_t hlit = 0, hdist = 0, hclen = 0;
  if (!Take(5, &hlit) || !Take(5, &hdist) || !Take(4, &hclen)) return false;
  hlit += 257;
  hdist += 1;
  hclen += 4;
  if (hlit > 286 || hdist > 30) return Fail("too many length or distance codes");
  std::uint8_t lengths[320] = {};
  for (std::uint32_t i = 0; i < hclen; ++i) {
    std::uint32_t value = 0;
    if (!Take(3, &value)) return false;
    lengths[kCodeLengthOrder[i]] = static_cast<std::uint8_t>(value);
  }
  Huffman code_lengths;
  if (!Build(&code_lengths, lengths, 19)) return Fail("invalid code length code");
  std::uint8_t all[320] = {};
  std::uint32_t index = 0;
  const std::uint32_t total = hlit + hdist;
  while (index < total) {
    if (!Need(16)) return false;
    int symbol = Decode(code_lengths);
    if (symbol < 0) return Fail("invalid code length");
    if (symbol < 16) {
      all[index++] = static_cast<std::uint8_t>(symbol);
      continue;
    }
    std::uint32_t repeat = 0;
    std::uint8_t value = 0;
    if (symbol == 16) {
      if (index == 0) return Fail("length repeat with no previous length");
      value = all[index - 1];
      repeat = 3 + Peek(2);
      Drop(2);
    } else if (symbol == 17) {
      repeat = 3 + Peek(3);
      Drop(3);
    } else {
      repeat = 11 + Peek(7);
      Drop(7);
    }
    if (index + repeat > total) return Fail("too many code lengths");
    while (repeat-- > 0) all[index++] = value;
  }
  if (Overrun()) return Fail("unexpected end of input");
  if (all[256] == 0) return Fail("missing end of block code");
  if (!Build(&dyn_lit_, all, static_cast<int>(hlit))) return Fail("invalid literal code");
  if (!Build(&dyn_dist_, all + hlit, static_cast<int>(hdist))) {
    return Fail("invalid distance code");
  }
  return true;
}

bool GzipSource::DecodeStored() {
  while (stored_left_ > 0 && out_write_ < kOutCapacity) {
    if (nbits_ >= 8) {
      std::uint32_t byte = 0;
      if (!Take(8, &byte)) return false;
      out_[out_write_++] = static_cast<std::uint8_t>(byte);
      ++member_size_;
      --stored_left_;
      continue;
    }
    bits_ = 0;
    nbits_ = 0;
    if (in_pos_ == in_end_ && !Refill()) return Fail("unexpected end of input");
    std::size_t n = std::min(
        {static_cast<std::size_t>(stored_left_), in_end_ - in_pos_, kOutCapacity - out_write_});
    std::memcpy(out_.data() + out_write_, in_.data() + in_pos_, n);
    in_pos_ += n;
    out_write_ += n;
    member_size_ += n;
    stored_left_ -= static_cast<std::uint32_t>(n);
  }
  if (stored_left_ == 0) state_ = State::kBlockHeader;
  return true;
}

bool GzipSource::DecodeCompressed() {
  std::uint8_t* out = out_.data();
  std::size_t pos = out_write_;
  const std::size_t limit = kOutCapacity - kMaxMatch;
  bool ok = true;
  while (pos <= limit) {
    if (!Need(48)) {
      ok = false;
      break;
    }
    int symbol = Decode(*lit_);
    if (symbol < 0) {
      ok = Fail("invalid literal or length code");
      break;
    }
    if (symbol < 256) {
      out[pos++] = static_cast<std::uint8_t>(symbol);
      continue;
    }
    if (symbol == 256) {
      state_ = State::kBlockHeader;
      break;
    }
    symbol -= 257;
    if (symbol >= 29) {
      ok = Fail("invalid length symbol");
      break;
    }
    std::uint32_t length = kLengthBase[symbol] + Peek(kLengthExtra[symbol]);
    Drop(kLengthExtra[symbol]);
    int dsym = Decode(*dist_);
    if (dsym < 0 || dsym >= 30) {
      ok = Fail("invalid distance code");
      break;
    }
    std::uint32_t distance = kDistBase[dsym] + Peek(kDistExtra[dsym]);
    Drop(kDistExtra[dsym]);
    if (distance > member_size_ + (pos - out_write_)) {
      ok = Fail("distance points before the start of the stream");
      break;
    }
    const std::uint8_t* src = out + pos - distance;
    if (distance >= 8) {
      for (std::uint32_t i = 0; i < length; i += 8) std::memcpy(out + pos + i, src + i, 8);
    } else {
      for (std::uint32_t i = 0; i < length; ++i) out[pos + i] = src[i];
    }
    pos += length;
  }
  member_size_ += pos - out_write_;
  out_write_ = pos;
  if (ok && Overrun()) ok = Fail("unexpected end of input");
  return ok;
}

bool GzipSource::ReadTrailer() {
  Drop(nbits_ % 8);
  std::uint32_t crc = 0, size = 0, byte = 0;
  for (int i = 0; i < 4; ++i) {
    if (!Take(8, &byte)) return false;
    crc |= byte << (8 * i);
  }
  for (int i = 0; i < 4; ++i) {
    if (!Take(8, &byte)) return false;
    size |= byte << (8 * i);
  }
  if (crc != crc_) return Fail("crc mismatch");
  if (size != static_cast<std::uint32_t>(member_size_)) return Fail("length mismatch");
  if (HasMoreInput()) {
    state_ = State::kMemberHeader;
  } else {
    if (!error_.empty()) return false;
    state_ = State::kDone;
  }
  return true;
}

int GzipSource::Decode(const Huffman& h) {
  std::uint32_t entry = h.fast[bits_ & 1023];
  if (entry != 0) {
    Drop(static_cast<int>(entry & 15));
    return static_cast<int>(entry >> 4);
  }
  int code = 0;
  int first = 0;
  int index = 0;
  std::uint64_t b = bits_;
  for (int len = 1; len <= 15; ++len) {
    code |= static_cast<int>(b & 1);
    b >>= 1;
    int count = h.count[len];
    if (code - first < count) {
      Drop(len);
      return h.symbols[index + (code - first)];
    }
    index += count;
    first += count;
    first <<= 1;
    code <<= 1;
  }
  return -1;
}

bool GzipSource::Build(Huffman* h, const std::uint8_t* lengths, int n) {
  h->fast.fill(0);
  h->count.fill(0);
  for (int s = 0; s < n; ++s) {
    if (lengths[s] > 15) return false;
    h->count[lengths[s]]++;
  }
  h->count[0] = 0;
  int left = 1;
  for (int len = 1; len <= 15; ++len) {
    left <<= 1;
    left -= h->count[len];
    if (left < 0) return false;
  }
  std::array<std::uint16_t, 16> offsets{};
  for (int len = 1; len < 15; ++len) {
    offsets[len + 1] = static_cast<std::uint16_t>(offsets[len] + h->count[len]);
  }
  for (int s = 0; s < n; ++s) {
    if (lengths[s] != 0) h->symbols[offsets[lengths[s]]++] = static_cast<std::uint16_t>(s);
  }
  std::array<std::uint32_t, 16> next{};
  std::uint32_t code = 0;
  for (int len = 1; len <= 15; ++len) {
    code = (code + h->count[len - 1]) << 1;
    next[len] = code;
  }
  for (int s = 0; s < n; ++s) {
    int len = lengths[s];
    if (len == 0) continue;
    std::uint32_t value = next[len]++;
    if (len > 10) continue;
    std::uint32_t reversed = 0;
    for (int i = 0; i < len; ++i) reversed |= ((value >> i) & 1) << (len - 1 - i);
    for (std::uint32_t k = reversed; k < 1024; k += std::uint32_t{1} << len) {
      h->fast[k] = static_cast<std::uint16_t>((s << 4) | len);
    }
  }
  return true;
}

const GzipSource::Huffman& GzipSource::FixedLiterals() {
  static const Huffman table = [] {
    std::uint8_t lengths[288];
    for (int i = 0; i < 288; ++i) lengths[i] = i < 144 ? 8 : i < 256 ? 9 : i < 280 ? 7 : 8;
    Huffman h;
    Build(&h, lengths, 288);
    return h;
  }();
  return table;
}

const GzipSource::Huffman& GzipSource::FixedDistances() {
  static const Huffman table = [] {
    std::uint8_t lengths[30];
    std::fill(std::begin(lengths), std::end(lengths), std::uint8_t{5});
    Huffman h;
    Build(&h, lengths, 30);
    return h;
  }();
  return table;
}

Result<std::unique_ptr<ByteSource>> OpenTextSource(const std::string& path) {
  unsigned char magic[2] = {0, 0};
  std::size_t got = 0;
  if (std::FILE* probe = std::fopen(path.c_str(), "rb")) {
    got = std::fread(magic, 1, 2, probe);
    std::fclose(probe);
  }
  Result<std::unique_ptr<ByteSource>> file = OpenFileSource(path);
  if (!file.ok()) return file;
  if (got == 2 && magic[0] == 0x1f && magic[1] == 0x8b) {
    return std::unique_ptr<ByteSource>(new GzipSource(std::move(*file)));
  }
  return file;
}

}
