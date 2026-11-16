#include "io.h"

#include <algorithm>
#include <bit>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <system_error>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace rr {

namespace {

struct CrcTables {
  std::uint32_t t[8][256];

  explicit CrcTables(std::uint32_t poly) {
    for (std::uint32_t i = 0; i < 256; ++i) {
      std::uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = (c & 1) ? (c >> 1) ^ poly : c >> 1;
      t[0][i] = c;
    }
    for (int s = 1; s < 8; ++s) {
      for (std::uint32_t i = 0; i < 256; ++i) {
        t[s][i] = (t[s - 1][i] >> 8) ^ t[0][t[s - 1][i] & 0xff];
      }
    }
  }
};

std::uint32_t CrcUpdate(const CrcTables& tb, std::uint32_t crc, const void* data,
                        std::size_t size) {
  const unsigned char* p = static_cast<const unsigned char*>(data);
  crc = ~crc;
  if (std::endian::native == std::endian::little) {
    while (size >= 8) {
      std::uint64_t v;
      std::memcpy(&v, p, 8);
      v ^= crc;
      crc = tb.t[7][v & 0xff] ^ tb.t[6][(v >> 8) & 0xff] ^ tb.t[5][(v >> 16) & 0xff] ^
            tb.t[4][(v >> 24) & 0xff] ^ tb.t[3][(v >> 32) & 0xff] ^ tb.t[2][(v >> 40) & 0xff] ^
            tb.t[1][(v >> 48) & 0xff] ^ tb.t[0][v >> 56];
      p += 8;
      size -= 8;
    }
  }
  while (size > 0) {
    crc = (crc >> 8) ^ tb.t[0][(crc ^ *p) & 0xff];
    ++p;
    --size;
  }
  return ~crc;
}

std::string ErrnoText() { return std::error_code(errno, std::generic_category()).message(); }

class FileSource : public ByteSource {
 public:
  explicit FileSource(std::FILE* file) : file_(file) {}
  ~FileSource() override { std::fclose(file_); }

  Result<std::size_t> Read(char* out, std::size_t size) override {
    std::size_t got = std::fread(out, 1, size, file_);
    if (got < size && std::ferror(file_)) return Status::Error("read failed: " + ErrnoText());
    return got;
  }

 private:
  std::FILE* file_;
};

void PutU32(std::vector<std::uint8_t>& out, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) out.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
}

void PutU64(std::vector<std::uint8_t>& out, std::uint64_t v) {
  for (int i = 0; i < 8; ++i) out.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
}

std::uint32_t GetU32(const std::uint8_t* p) {
  std::uint32_t v;
  std::memcpy(&v, p, 4);
  return v;
}

std::uint64_t GetU64(const std::uint8_t* p) {
  std::uint64_t v;
  std::memcpy(&v, p, 8);
  return v;
}

std::uint64_t Align8(std::uint64_t v) { return (v + 7) & ~std::uint64_t{7}; }

constexpr std::uint32_t kContainerVersion = 1;
constexpr std::uint64_t kHeaderSize = 32;
constexpr std::uint64_t kEntrySize = 24;
constexpr std::uint32_t kMaxSections = 64;
constexpr std::uint64_t kAlwaysVerifyBytes = 1 << 20;

}

std::uint32_t Crc32c(std::uint32_t crc, const void* data, std::size_t size) {
  static const CrcTables tables(0x82f63b78);
  return CrcUpdate(tables, crc, data, size);
}

std::uint32_t Crc32(std::uint32_t crc, const void* data, std::size_t size) {
  static const CrcTables tables(0xedb88320);
  return CrcUpdate(tables, crc, data, size);
}

std::uint32_t SectionIdentity(std::span<const std::uint32_t> section_crcs) {
  return Crc32c(0, section_crcs.data(), section_crcs.size_bytes());
}

Result<std::unique_ptr<ByteSource>> OpenFileSource(const std::string& path) {
  std::FILE* file = std::fopen(path.c_str(), "rb");
  if (file == nullptr) return Status::Error("cannot open " + path + ": " + ErrnoText());
  return std::unique_ptr<ByteSource>(new FileSource(file));
}

LineReader::LineReader(std::unique_ptr<ByteSource> source)
    : source_(std::move(source)), buffer_(1 << 22) {}

Result<bool> LineReader::Next(std::string_view* line) {
  while (true) {
    const char* start = buffer_.data() + begin_;
    const char* newline = static_cast<const char*>(std::memchr(start, '\n', end_ - begin_));
    if (newline != nullptr || (eof_ && begin_ < end_)) {
      std::size_t length =
          newline != nullptr ? static_cast<std::size_t>(newline - start) : end_ - begin_;
      begin_ += newline != nullptr ? length + 1 : length;
      if (length > 0 && start[length - 1] == '\r') --length;
      *line = std::string_view(start, length);
      ++line_number_;
      return true;
    }
    if (eof_) return false;
    if (begin_ > 0) {
      std::memmove(buffer_.data(), buffer_.data() + begin_, end_ - begin_);
      end_ -= begin_;
      begin_ = 0;
    }
    if (end_ == buffer_.size()) {
      return Status::Error("line " + std::to_string(line_number_ + 1) + " is too long");
    }
    Result<std::size_t> got = source_->Read(buffer_.data() + end_, buffer_.size() - end_);
    if (!got.ok()) return got.status();
    if (*got == 0) eof_ = true;
    end_ += *got;
  }
}

Result<std::shared_ptr<MappedFile>> MappedFile::Open(const std::string& path) {
  std::shared_ptr<MappedFile> mapped(new MappedFile());
#if defined(_WIN32)
  HANDLE file = CreateFileA(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING,
                            FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) return Status::Error("cannot open " + path);
  mapped->file_handle_ = file;
  LARGE_INTEGER size;
  if (!GetFileSizeEx(file, &size)) return Status::Error("cannot stat " + path);
  if (size.QuadPart <= 0) return Status::Error(path + " is empty");
  HANDLE mapping = CreateFileMappingA(file, nullptr, PAGE_READONLY, 0, 0, nullptr);
  if (mapping == nullptr) return Status::Error("cannot map " + path);
  mapped->map_handle_ = mapping;
  void* view = MapViewOfFile(mapping, FILE_MAP_READ, 0, 0, 0);
  if (view == nullptr) return Status::Error("cannot map " + path);
  mapped->data_ = static_cast<const std::uint8_t*>(view);
  mapped->size_ = static_cast<std::uint64_t>(size.QuadPart);
#else
  int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) return Status::Error("cannot open " + path + ": " + ErrnoText());
  mapped->fd_ = fd;
  struct stat st;
  if (::fstat(fd, &st) != 0) return Status::Error("cannot stat " + path + ": " + ErrnoText());
  if (st.st_size <= 0) return Status::Error(path + " is empty");
  void* view = ::mmap(nullptr, static_cast<std::size_t>(st.st_size), PROT_READ, MAP_PRIVATE, fd, 0);
  if (view == MAP_FAILED) return Status::Error("cannot map " + path + ": " + ErrnoText());
  mapped->data_ = static_cast<const std::uint8_t*>(view);
  mapped->size_ = static_cast<std::uint64_t>(st.st_size);
#endif
  return mapped;
}

MappedFile::~MappedFile() {
#if defined(_WIN32)
  if (data_ != nullptr) UnmapViewOfFile(const_cast<void*>(static_cast<const void*>(data_)));
  if (map_handle_ != nullptr) CloseHandle(static_cast<HANDLE>(map_handle_));
  if (file_handle_ != nullptr) CloseHandle(static_cast<HANDLE>(file_handle_));
#else
  if (data_ != nullptr) {
    ::munmap(const_cast<void*>(static_cast<const void*>(data_)), static_cast<std::size_t>(size_));
  }
  if (fd_ >= 0) ::close(fd_);
#endif
}

FileWriter::FileWriter(std::string path, std::string temp_path, std::FILE* file)
    : path_(std::move(path)), temp_path_(std::move(temp_path)), file_(file) {}

Result<std::unique_ptr<FileWriter>> FileWriter::Create(const std::string& path) {
  std::string temp_path = path + ".tmp";
  std::FILE* file = std::fopen(temp_path.c_str(), "wb");
  if (file == nullptr) return Status::Error("cannot create " + temp_path + ": " + ErrnoText());
  std::setvbuf(file, nullptr, _IOFBF, 1 << 20);
  return std::unique_ptr<FileWriter>(new FileWriter(path, std::move(temp_path), file));
}

FileWriter::~FileWriter() {
  if (file_ != nullptr) {
    std::fclose(file_);
    std::error_code ec;
    std::filesystem::remove(temp_path_, ec);
  }
}

Status FileWriter::Write(const void* data, std::size_t size) {
  if (failed_) return Status::Error("write to " + path_ + " failed earlier");
  if (size > 0 && std::fwrite(data, 1, size, file_) != size) {
    failed_ = true;
    return Status::Error("write to " + temp_path_ + " failed: " + ErrnoText());
  }
  offset_ += size;
  return Status();
}

Status FileWriter::PadTo(std::size_t alignment) {
  static const char zeros[64] = {};
  std::size_t pad = static_cast<std::size_t>((alignment - offset_ % alignment) % alignment);
  while (pad > 0) {
    std::size_t chunk = std::min<std::size_t>(pad, sizeof(zeros));
    RR_RETURN_IF_ERROR(Write(zeros, chunk));
    pad -= chunk;
  }
  return Status();
}

Status FileWriter::Finish() {
  if (failed_) return Status::Error("write to " + path_ + " failed");
  bool ok = std::fflush(file_) == 0;
  ok = std::fclose(file_) == 0 && ok;
  file_ = nullptr;
  std::error_code ec;
  if (!ok) {
    std::filesystem::remove(temp_path_, ec);
    return Status::Error("cannot finish " + temp_path_ + ": " + ErrnoText());
  }
  std::filesystem::remove(path_, ec);
  std::filesystem::rename(temp_path_, path_, ec);
  if (ec) {
    std::error_code ignored;
    std::filesystem::remove(temp_path_, ignored);
    return Status::Error("cannot rename " + temp_path_ + ": " + ec.message());
  }
  return Status();
}

ContainerWriter::ContainerWriter(std::string_view magic) : magic_(magic) { magic_.resize(8, '\0'); }

void ContainerWriter::AddBytes(std::uint32_t id, const void* data, std::uint64_t size) {
  sections_.push_back({id, data, size});
}

Status ContainerWriter::Write(const std::string& path) const {
  std::uint64_t offset = Align8(kHeaderSize + kEntrySize * sections_.size());
  std::vector<std::uint8_t> table;
  for (const Section& section : sections_) {
    PutU32(table, section.id);
    PutU32(table, Crc32c(0, section.data, static_cast<std::size_t>(section.size)));
    PutU64(table, offset);
    PutU64(table, section.size);
    offset = Align8(offset + section.size);
  }
  std::vector<std::uint8_t> header(magic_.begin(), magic_.end());
  PutU32(header, kContainerVersion);
  PutU32(header, static_cast<std::uint32_t>(sections_.size()));
  PutU64(header, offset);
  PutU32(header, Crc32c(0, table.data(), table.size()));
  PutU32(header, 0);

  Result<std::unique_ptr<FileWriter>> writer = FileWriter::Create(path);
  if (!writer.ok()) return writer.status();
  FileWriter& out = **writer;
  RR_RETURN_IF_ERROR(out.Write(header.data(), header.size()));
  RR_RETURN_IF_ERROR(out.Write(table.data(), table.size()));
  RR_RETURN_IF_ERROR(out.PadTo(8));
  for (const Section& section : sections_) {
    const char* p = static_cast<const char*>(section.data);
    std::uint64_t left = section.size;
    while (left > 0) {
      std::size_t chunk = static_cast<std::size_t>(std::min<std::uint64_t>(left, 1 << 26));
      RR_RETURN_IF_ERROR(out.Write(p, chunk));
      p += chunk;
      left -= chunk;
    }
    RR_RETURN_IF_ERROR(out.PadTo(8));
  }
  if (out.offset() != offset) return Status::Error("internal layout mismatch in " + path);
  return out.Finish();
}

Result<ContainerReader> ContainerReader::Open(const std::string& path, std::string_view magic,
                                              bool verify_all) {
  Result<std::shared_ptr<MappedFile>> mapped = MappedFile::Open(path);
  if (!mapped.ok()) return mapped.status();
  ContainerReader reader;
  reader.path_ = path;
  reader.file_ = *mapped;
  const std::uint8_t* data = reader.file_->data();
  const std::uint64_t size = reader.file_->size();
  std::string expected(magic);
  expected.resize(8, '\0');
  if (size < kHeaderSize || std::memcmp(data, expected.data(), 8) != 0) {
    return Status::Error(path + " is not a " + std::string(magic) + " file");
  }
  if (GetU32(data + 8) != kContainerVersion) {
    return Status::Error(path + " has an unsupported version");
  }
  std::uint32_t count = GetU32(data + 12);
  if (count > kMaxSections || size < kHeaderSize + kEntrySize * count) {
    return Status::Error(path + " has a corrupt section table");
  }
  if (GetU64(data + 16) != size) {
    return Status::Error(path + " is truncated or has trailing data");
  }
  const std::uint8_t* table = data + kHeaderSize;
  if (Crc32c(0, table, static_cast<std::size_t>(kEntrySize * count)) != GetU32(data + 24) ||
      GetU32(data + 28) != 0) {
    return Status::Error(path + " has a corrupt section table");
  }
  std::uint64_t data_start = kHeaderSize + kEntrySize * count;
  std::vector<std::uint32_t> crcs;
  for (std::uint32_t i = 0; i < count; ++i) {
    const std::uint8_t* p = table + kEntrySize * i;
    Entry entry{GetU32(p), GetU32(p + 4), GetU64(p + 8), GetU64(p + 16)};
    if (entry.offset % 8 != 0 || entry.offset < data_start || entry.offset > size ||
        entry.size > size - entry.offset) {
      return Status::Error(path + ": section " + TagName(entry.id) + " is out of bounds");
    }
    if (reader.Find(entry.id) != nullptr) {
      return Status::Error(path + ": duplicate section " + TagName(entry.id));
    }
    if (verify_all || entry.size <= kAlwaysVerifyBytes) {
      if (Crc32c(0, data + entry.offset, static_cast<std::size_t>(entry.size)) != entry.crc) {
        return Status::Error(path + ": checksum mismatch in section " + TagName(entry.id));
      }
    }
    reader.entries_.push_back(entry);
    crcs.push_back(entry.crc);
  }
  reader.identity_ = SectionIdentity(crcs);
  return reader;
}

Result<std::uint64_t> ContainerReader::SectionSize(std::uint32_t id) const {
  const Entry* entry = Find(id);
  if (entry == nullptr) return Status::Error(path_ + ": missing section " + TagName(id));
  return entry->size;
}

const ContainerReader::Entry* ContainerReader::Find(std::uint32_t id) const {
  for (const Entry& entry : entries_) {
    if (entry.id == id) return &entry;
  }
  return nullptr;
}

std::string ContainerReader::TagName(std::uint32_t id) {
  std::string name;
  for (int i = 0; i < 4; ++i) {
    char c = static_cast<char>((id >> (8 * i)) & 0xff);
    name.push_back(c >= 32 && c < 127 ? c : '?');
  }
  return name;
}

}
