#include "io.h"

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "gzip.h"
#include "support.h"
#include "testing.h"

namespace rr {
namespace {

static const unsigned char kDynamicGzip[] = {
    0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x2b, 0x50, 0x28, 0x2e,
    0x50, 0x30, 0x56, 0x30, 0xe2, 0x4a, 0x54, 0x30, 0x54, 0x30, 0x52, 0x30, 0x34, 0x00,
    0x32, 0x8c, 0x40, 0x02, 0x06, 0x5c, 0x05, 0xa3, 0x52, 0xa3, 0x52, 0xa3, 0x52, 0xa3,
    0x52, 0xd8, 0xa4, 0x00, 0x6b, 0x41, 0x44, 0x5c, 0x38, 0x04, 0x00, 0x00};

static const unsigned char kStoredGzip[] = {
    0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x03, 0x01, 0x1b, 0x00,
    0xe4, 0xff, 0x70, 0x20, 0x73, 0x70, 0x20, 0x33, 0x20, 0x32, 0x0a, 0x61, 0x20,
    0x31, 0x20, 0x32, 0x20, 0x31, 0x30, 0x0a, 0x61, 0x20, 0x32, 0x20, 0x33, 0x20,
    0x32, 0x30, 0x0a, 0x24, 0x48, 0xa9, 0xb8, 0x1b, 0x00, 0x00, 0x00};

static const unsigned char kFixedGzip[] = {
    0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x2b, 0x50, 0x28, 0x2e, 0x50,
    0x30, 0x56, 0x30, 0xe2, 0x4a, 0x54, 0x30, 0x54, 0x30, 0x52, 0x30, 0x34, 0x00, 0x32, 0x8c,
    0x40, 0x02, 0x06, 0x5c, 0x00, 0x24, 0x48, 0xa9, 0xb8, 0x1b, 0x00, 0x00, 0x00};

static const unsigned char kMultiGzip[] = {
    0x1f, 0x8b, 0x08, 0x00, 0x79, 0x8a, 0xf1, 0x6a, 0x02, 0xff, 0x4b, 0xcb, 0x2c, 0x2a,
    0x2e, 0xe1, 0x02, 0x00, 0x2a, 0xb3, 0x4a, 0xc7, 0x06, 0x00, 0x00, 0x00, 0x1f, 0x8b,
    0x08, 0x00, 0x79, 0x8a, 0xf1, 0x6a, 0x02, 0xff, 0x2b, 0x4e, 0x4d, 0xce, 0xcf, 0x4b,
    0xe1, 0x02, 0x00, 0x7e, 0xc0, 0x0f, 0x06, 0x07, 0x00, 0x00, 0x00};

std::string Inflate(const unsigned char* data, std::size_t size, std::string* error) {
  testing::TempDir dir;
  std::string path = dir.File("stream.gz");
  Result<std::unique_ptr<FileWriter>> writer = FileWriter::Create(path);
  if (!writer.ok()) return {};
  if (!(*writer)->Write(data, size).ok()) return {};
  if (!(*writer)->Finish().ok()) return {};
  Result<std::unique_ptr<ByteSource>> source = OpenTextSource(path);
  if (!source.ok()) {
    *error = source.status().message();
    return {};
  }
  std::string out;
  char buffer[4096];
  while (true) {
    Result<std::size_t> got = (*source)->Read(buffer, sizeof(buffer));
    if (!got.ok()) {
      *error = got.status().message();
      return out;
    }
    if (*got == 0) return out;
    out.append(buffer, *got);
  }
}

std::string Repeat(const std::string& text, int times) {
  std::string out;
  for (int i = 0; i < times; ++i) out += text;
  return out;
}

RR_TEST(Crc32cMatchesTheCheckValue) {
  RR_CHECK_EQ(Crc32c(0, "123456789", 9), 0xe3069283u);
  RR_CHECK_EQ(Crc32(0, "123456789", 9), 0xcbf43926u);
  RR_CHECK_EQ(Crc32c(0, "", 0), 0u);
  std::string a = "the quick brown fox";
  std::uint32_t whole = Crc32c(0, a.data(), a.size());
  std::uint32_t split = Crc32c(Crc32c(0, a.data(), 7), a.data() + 7, a.size() - 7);
  RR_CHECK_EQ(whole, split);
}

RR_TEST(ContainerSurvivesARoundTrip) {
  testing::TempDir dir;
  std::string path = dir.File("container.bin");
  std::vector<std::uint32_t> numbers;
  for (std::uint32_t i = 0; i < 1000; ++i) numbers.push_back(i * 7 + 1);
  std::vector<std::uint8_t> bytes = {1, 2, 3, 4, 5};
  ContainerWriter writer("RRTEST");
  writer.Add(Tag("NUMS"), numbers);
  writer.Add(Tag("BYTE"), bytes);
  RR_CHECK_STATUS(writer.Write(path));
  Result<ContainerReader> reader = ContainerReader::Open(path, "RRTEST", true);
  RR_CHECK_OK(reader);
  Result<Array<std::uint32_t>> back = reader->Get<std::uint32_t>(Tag("NUMS"), numbers.size());
  RR_CHECK_OK(back);
  for (std::size_t i = 0; i < numbers.size(); ++i) RR_CHECK_EQ((*back)[i], numbers[i]);
  Result<Array<std::uint8_t>> small = reader->Get<std::uint8_t>(Tag("BYTE"), bytes.size());
  RR_CHECK_OK(small);
  RR_CHECK_EQ((*small)[4], 5u);
  RR_CHECK(!reader->Get<std::uint32_t>(Tag("NONE"), 1).ok());
  RR_CHECK(!reader->Get<std::uint32_t>(Tag("NUMS"), 999).ok());
  RR_CHECK(!ContainerReader::Open(path, "OTHER", true).ok());
}

RR_TEST(ContainerRejectsEveryCorruptedByte) {
  testing::TempDir dir;
  std::string path = dir.File("container.bin");
  std::vector<std::uint32_t> numbers;
  for (std::uint32_t i = 0; i < 64; ++i) numbers.push_back(i);
  ContainerWriter writer("RRTEST");
  writer.Add(Tag("NUMS"), numbers);
  RR_CHECK_STATUS(writer.Write(path));
  std::string original;
  {
    Result<std::shared_ptr<MappedFile>> file = MappedFile::Open(path);
    RR_CHECK_OK(file);
    original.assign(reinterpret_cast<const char*>((*file)->data()),
                    static_cast<std::size_t>((*file)->size()));
  }
  int accepted = 0;
  for (std::size_t i = 0; i < original.size(); ++i) {
    std::string broken = original;
    broken[i] = static_cast<char>(broken[i] ^ 0x40);
    std::string broken_path = dir.File("broken.bin");
    Result<std::unique_ptr<FileWriter>> out = FileWriter::Create(broken_path);
    RR_CHECK_OK(out);
    RR_CHECK_STATUS((*out)->Write(broken));
    RR_CHECK_STATUS((*out)->Finish());
    Result<ContainerReader> reader = ContainerReader::Open(broken_path, "RRTEST", true);
    if (reader.ok()) {
      Result<Array<std::uint32_t>> back = reader->Get<std::uint32_t>(Tag("NUMS"), numbers.size());
      if (back.ok()) ++accepted;
    }
  }
  RR_CHECK_EQ(accepted, 0);
  std::string truncated_path = dir.File("short.bin");
  Result<std::unique_ptr<FileWriter>> out = FileWriter::Create(truncated_path);
  RR_CHECK_OK(out);
  RR_CHECK_STATUS((*out)->Write(original.substr(0, original.size() - 8)));
  RR_CHECK_STATUS((*out)->Finish());
  RR_CHECK(!ContainerReader::Open(truncated_path, "RRTEST", true).ok());
}

RR_TEST(FileWriterLeavesNothingBehindOnFailure) {
  testing::TempDir dir;
  std::string path = dir.File("partial.bin");
  {
    Result<std::unique_ptr<FileWriter>> writer = FileWriter::Create(path);
    RR_CHECK_OK(writer);
    RR_CHECK_STATUS((*writer)->Write(std::string("half")));
  }
  RR_CHECK(!MappedFile::Open(path).ok());
  RR_CHECK(!MappedFile::Open(path + ".tmp").ok());
}

RR_TEST(LineReaderHandlesBothLineEndings) {
  testing::TempDir dir;
  std::string path = dir.File("lines.txt");
  Result<std::unique_ptr<FileWriter>> writer = FileWriter::Create(path);
  RR_CHECK_OK(writer);
  RR_CHECK_STATUS((*writer)->Write(std::string("one\r\ntwo\n\nthree")));
  RR_CHECK_STATUS((*writer)->Finish());
  Result<std::unique_ptr<ByteSource>> source = OpenFileSource(path);
  RR_CHECK_OK(source);
  LineReader reader(std::move(*source));
  std::vector<std::string> lines;
  std::string_view line;
  while (true) {
    Result<bool> more = reader.Next(&line);
    RR_CHECK_OK(more);
    if (!*more) break;
    lines.push_back(std::string(line));
  }
  RR_CHECK_EQ(lines.size(), std::size_t{4});
  RR_CHECK_EQ(lines[0], std::string("one"));
  RR_CHECK_EQ(lines[1], std::string("two"));
  RR_CHECK_EQ(lines[2], std::string(""));
  RR_CHECK_EQ(lines[3], std::string("three"));
  RR_CHECK_EQ(reader.line_number(), std::uint64_t{4});
}

RR_TEST(GzipReadsEveryBlockType) {
  std::string error;
  std::string expected = "p sp 3 2\na 1 2 10\na 2 3 20\n";
  RR_CHECK_EQ(Inflate(kStoredGzip, sizeof(kStoredGzip), &error), expected);
  RR_CHECK_EQ(Inflate(kFixedGzip, sizeof(kFixedGzip), &error), expected);
  RR_CHECK_EQ(Inflate(kDynamicGzip, sizeof(kDynamicGzip), &error), Repeat(expected, 40));
  RR_CHECK_EQ(Inflate(kMultiGzip, sizeof(kMultiGzip), &error), std::string("first\nsecond\n"));
  RR_CHECK(error.empty());
}

RR_TEST(GzipRejectsDamagedStreams) {
  std::vector<unsigned char> stream(kDynamicGzip, kDynamicGzip + sizeof(kDynamicGzip));
  int accepted = 0;
  for (std::size_t i = 2; i < stream.size(); ++i) {
    for (int bit = 0; bit < 8; bit += 3) {
      std::vector<unsigned char> broken = stream;
      broken[i] = static_cast<unsigned char>(broken[i] ^ (1 << bit));
      std::string error;
      std::string out = Inflate(broken.data(), broken.size(), &error);
      if (error.empty() && out != Repeat("p sp 3 2\na 1 2 10\na 2 3 20\n", 40)) ++accepted;
    }
  }
  RR_CHECK_EQ(accepted, 0);
  for (std::size_t cut = 2; cut < stream.size(); cut += 3) {
    std::string error;
    Inflate(stream.data(), cut, &error);
    RR_CHECK(!error.empty());
  }
}

RR_TEST(FilesWithoutTheGzipMagicAreReadVerbatim) {
  std::vector<unsigned char> stream(kDynamicGzip, kDynamicGzip + sizeof(kDynamicGzip));
  stream[0] = 'p';
  std::string error;
  std::string out = Inflate(stream.data(), stream.size(), &error);
  RR_CHECK(error.empty());
  RR_CHECK_EQ(out.size(), stream.size());
  RR_CHECK_EQ(static_cast<unsigned char>(out[1]), stream[1]);
}

}
}

int main() { return rr::testing::RunAll(); }
