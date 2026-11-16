#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "io.h"

namespace rr {

class GzipSource : public ByteSource {
 public:
  explicit GzipSource(std::unique_ptr<ByteSource> input);

  Result<std::size_t> Read(char* out, std::size_t size) override;

 private:
  struct Huffman {
    std::array<std::uint16_t, 1024> fast{};
    std::array<std::uint16_t, 16> count{};
    std::array<std::uint16_t, 320> symbols{};
  };

  enum class State { kMemberHeader, kBlockHeader, kStored, kCompressed, kTrailer, kDone };

  bool Fill();
  bool Refill();
  bool Need(int bits);
  std::uint32_t Peek(int bits) const {
    return static_cast<std::uint32_t>(bits_ & ((std::uint64_t{1} << bits) - 1));
  }
  void Drop(int bits) {
    bits_ >>= bits;
    nbits_ -= bits;
  }
  bool Take(int bits, std::uint32_t* value);
  bool Overrun() const { return nbits_ < pad_bits_; }
  bool HasMoreInput();
  bool ReadMemberHeader();
  bool ReadBlockHeader();
  bool ReadDynamicTables();
  bool DecodeStored();
  bool DecodeCompressed();
  bool ReadTrailer();
  int Decode(const Huffman& h);
  bool Fail(const char* message);
  static bool Build(Huffman* h, const std::uint8_t* lengths, int count);
  static const Huffman& FixedLiterals();
  static const Huffman& FixedDistances();

  std::unique_ptr<ByteSource> input_;
  std::vector<std::uint8_t> in_;
  std::size_t in_pos_ = 0;
  std::size_t in_end_ = 0;
  bool in_eof_ = false;
  std::uint64_t bits_ = 0;
  int nbits_ = 0;
  int pad_bits_ = 0;

  std::vector<std::uint8_t> out_;
  std::size_t out_read_ = 0;
  std::size_t out_write_ = 0;

  State state_ = State::kMemberHeader;
  bool last_block_ = false;
  std::uint32_t stored_left_ = 0;
  Huffman dyn_lit_;
  Huffman dyn_dist_;
  const Huffman* lit_ = nullptr;
  const Huffman* dist_ = nullptr;
  std::uint32_t crc_ = 0;
  std::uint64_t member_size_ = 0;
  std::string error_;
};

Result<std::unique_ptr<ByteSource>> OpenTextSource(const std::string& path);

}
