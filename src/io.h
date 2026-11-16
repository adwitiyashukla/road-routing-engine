#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "base.h"

namespace rr {

std::uint32_t Crc32c(std::uint32_t crc, const void* data, std::size_t size);
std::uint32_t Crc32(std::uint32_t crc, const void* data, std::size_t size);

class ByteSource {
 public:
  virtual ~ByteSource() = default;
  virtual Result<std::size_t> Read(char* out, std::size_t size) = 0;
};

Result<std::unique_ptr<ByteSource>> OpenFileSource(const std::string& path);

class LineReader {
 public:
  explicit LineReader(std::unique_ptr<ByteSource> source);

  Result<bool> Next(std::string_view* line);
  std::uint64_t line_number() const { return line_number_; }

 private:
  std::unique_ptr<ByteSource> source_;
  std::vector<char> buffer_;
  std::size_t begin_ = 0;
  std::size_t end_ = 0;
  bool eof_ = false;
  std::uint64_t line_number_ = 0;
};

class MappedFile {
 public:
  static Result<std::shared_ptr<MappedFile>> Open(const std::string& path);

  MappedFile(const MappedFile&) = delete;
  MappedFile& operator=(const MappedFile&) = delete;
  ~MappedFile();

  const std::uint8_t* data() const { return data_; }
  std::uint64_t size() const { return size_; }

 private:
  MappedFile() = default;

  const std::uint8_t* data_ = nullptr;
  std::uint64_t size_ = 0;
#if defined(_WIN32)
  void* file_handle_ = nullptr;
  void* map_handle_ = nullptr;
#else
  int fd_ = -1;
#endif
};

class FileWriter {
 public:
  static Result<std::unique_ptr<FileWriter>> Create(const std::string& path);

  FileWriter(const FileWriter&) = delete;
  FileWriter& operator=(const FileWriter&) = delete;
  ~FileWriter();

  Status Write(const void* data, std::size_t size);
  Status Write(std::string_view text) { return Write(text.data(), text.size()); }
  Status PadTo(std::size_t alignment);
  std::uint64_t offset() const { return offset_; }
  Status Finish();

 private:
  FileWriter(std::string path, std::string temp_path, std::FILE* file);

  std::string path_;
  std::string temp_path_;
  std::FILE* file_ = nullptr;
  std::uint64_t offset_ = 0;
  bool failed_ = false;
};

template <typename T>
class Array {
 public:
  Array() = default;
  explicit Array(std::vector<T> values)
      : owned_(std::move(values)), data_(owned_.data()), size_(owned_.size()) {}
  Array(std::span<const T> view, std::shared_ptr<MappedFile> keep)
      : data_(view.data()), size_(view.size()), keep_(std::move(keep)) {}

  Array(Array&& other) noexcept { *this = std::move(other); }
  Array& operator=(Array&& other) noexcept {
    owned_ = std::move(other.owned_);
    keep_ = std::move(other.keep_);
    data_ = owned_.empty() ? other.data_ : owned_.data();
    size_ = other.size_;
    other.data_ = nullptr;
    other.size_ = 0;
    return *this;
  }
  Array(const Array&) = delete;
  Array& operator=(const Array&) = delete;

  const T* data() const { return data_; }
  std::size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }
  const T& operator[](std::size_t i) const { return data_[i]; }
  std::span<const T> span() const { return {data_, size_}; }

 private:
  std::vector<T> owned_;
  const T* data_ = nullptr;
  std::size_t size_ = 0;
  std::shared_ptr<MappedFile> keep_;
};

constexpr std::uint32_t Tag(const char name[5]) {
  return static_cast<std::uint32_t>(static_cast<unsigned char>(name[0])) |
         static_cast<std::uint32_t>(static_cast<unsigned char>(name[1])) << 8 |
         static_cast<std::uint32_t>(static_cast<unsigned char>(name[2])) << 16 |
         static_cast<std::uint32_t>(static_cast<unsigned char>(name[3])) << 24;
}

class ContainerWriter {
 public:
  explicit ContainerWriter(std::string_view magic);

  template <typename T>
  void Add(std::uint32_t id, std::span<const T> values) {
    AddBytes(id, values.data(), values.size_bytes());
  }
  template <typename T>
  void Add(std::uint32_t id, const std::vector<T>& values) {
    AddBytes(id, values.data(), values.size() * sizeof(T));
  }
  void AddBytes(std::uint32_t id, const void* data, std::uint64_t size);

  Status Write(const std::string& path) const;

 private:
  struct Section {
    std::uint32_t id;
    const void* data;
    std::uint64_t size;
  };
  std::string magic_;
  std::vector<Section> sections_;
};

class ContainerReader {
 public:
  static Result<ContainerReader> Open(const std::string& path, std::string_view magic,
                                      bool verify_all);

  template <typename T>
  Result<Array<T>> Get(std::uint32_t id, std::uint64_t count) const {
    const Entry* entry = Find(id);
    if (entry == nullptr) return Status::Error(path_ + ": missing section " + TagName(id));
    if (entry->size / sizeof(T) != count || entry->size % sizeof(T) != 0) {
      return Status::Error(path_ + ": section " + TagName(id) + " has the wrong size");
    }
    if (entry->offset % alignof(T) != 0) {
      return Status::Error(path_ + ": section " + TagName(id) + " is misaligned");
    }
    const T* data = static_cast<const T*>(static_cast<const void*>(file_->data() + entry->offset));
    return Array<T>(std::span<const T>(data, static_cast<std::size_t>(count)), file_);
  }

  Result<std::uint64_t> SectionSize(std::uint32_t id) const;
  std::uint32_t identity() const { return identity_; }
  const std::string& path() const { return path_; }

 private:
  struct Entry {
    std::uint32_t id;
    std::uint32_t crc;
    std::uint64_t offset;
    std::uint64_t size;
  };

  const Entry* Find(std::uint32_t id) const;
  static std::string TagName(std::uint32_t id);

  std::string path_;
  std::shared_ptr<MappedFile> file_;
  std::vector<Entry> entries_;
  std::uint32_t identity_ = 0;
};

std::uint32_t SectionIdentity(std::span<const std::uint32_t> section_crcs);

}
