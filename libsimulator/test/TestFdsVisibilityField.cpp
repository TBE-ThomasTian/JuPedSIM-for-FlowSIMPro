// SPDX-License-Identifier: LGPL-3.0-or-later
#include "FdsVisibilityField.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
class TemporaryDirectory
{
    std::filesystem::path _path;

public:
    TemporaryDirectory()
        : _path(
              std::filesystem::temp_directory_path() /
              ("jps_fds_smoke3d_" + std::to_string(
                                       std::chrono::steady_clock::now()
                                           .time_since_epoch()
                                           .count())))
    {
        std::filesystem::create_directories(_path);
    }

    ~TemporaryDirectory()
    {
        std::error_code error{};
        std::filesystem::remove_all(_path, error);
    }

    const std::filesystem::path& Path() const { return _path; }
};

void WriteU32(std::ostream& output, std::uint32_t value)
{
    const std::array<char, 4> bytes{
        static_cast<char>(value & 0xffU),
        static_cast<char>((value >> 8U) & 0xffU),
        static_cast<char>((value >> 16U) & 0xffU),
        static_cast<char>((value >> 24U) & 0xffU)};
    output.write(bytes.data(), bytes.size());
}

void WriteRecord(std::ostream& output, const std::vector<char>& data)
{
    WriteU32(output, static_cast<std::uint32_t>(data.size()));
    output.write(data.data(), static_cast<std::streamsize>(data.size()));
    WriteU32(output, static_cast<std::uint32_t>(data.size()));
}

std::vector<char> EncodeU32Values(const std::vector<std::uint32_t>& values)
{
    std::vector<char> data{};
    data.reserve(values.size() * 4);
    for(const auto value : values) {
        data.push_back(static_cast<char>(value & 0xffU));
        data.push_back(static_cast<char>((value >> 8U) & 0xffU));
        data.push_back(static_cast<char>((value >> 16U) & 0xffU));
        data.push_back(static_cast<char>((value >> 24U) & 0xffU));
    }
    return data;
}

std::vector<char> EncodeFloat(float value)
{
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(value));
    return EncodeU32Values({bits});
}

void WriteSmokeFile(const std::filesystem::path& path)
{
    std::ofstream output(path, std::ios::binary);
    ASSERT_TRUE(output.good());
    WriteRecord(output, EncodeU32Values({1, 0, 0, 1, 0, 1, 0, 2}));

    const std::vector<char> frame0{
        0, 0, 0, 0,
        static_cast<char>(254), static_cast<char>(254), static_cast<char>(254), static_cast<char>(254),
        0, 0, 0, 0};
    const std::vector<char> frame10{
        0, 0, 0, 0,
        127, 127, 127, 127,
        0, 0, 0, 0};
    for(const auto& [time, values] :
        std::vector<std::pair<float, std::vector<char>>>{{0.0F, frame0}, {10.0F, frame10}}) {
        WriteRecord(output, EncodeFloat(time));
        WriteRecord(
            output,
            EncodeU32Values(
                {static_cast<std::uint32_t>(values.size()),
                 static_cast<std::uint32_t>(values.size())}));
        WriteRecord(output, values);
    }
}

void WriteSizeFile(const std::filesystem::path& path)
{
    std::ofstream output(path);
    ASSERT_TRUE(output.good());
    output << "0\n"
              "0.0 12 12 0.001\n"
              "10.0 12 12 0.001\n";
}

void WriteLegacySizeFile(const std::filesystem::path& path)
{
    std::ofstream output(path);
    ASSERT_TRUE(output.good());
    output << "0\n"
              "0.0 12 99 0 12 0.001\n"
              "10.0 12 99 0 12 0.001\n";
}

void WriteSmvFile(
    const std::filesystem::path& path,
    const std::string& longLabel = "SOOT DENSITY")
{
    std::ofstream output(path);
    ASSERT_TRUE(output.good());
    output << "GRID MESH_0000001\n"
              " 1 1 2 0 0 0 0 0 0\n\n"
              "PDIM\n"
              " 0 1 0 1 0 2 0 0 0\n\n"
              "TRNX\n"
              " 0\n"
              " 0 0.0\n"
              " 1 1.0\n\n"
              "TRNY\n"
              " 0\n"
              " 0 0.0\n"
              " 1 1.0\n\n"
              "TRNZ\n"
              " 0\n"
              " 0 0.0\n"
              " 1 1.0\n"
              " 2 2.0\n\n"
              "SMOKF3D 1 1000.0\n"
              " smoke.s3d\n "
           << longLabel << "\n rho_C\n kg/m3\n";
}
} // namespace

TEST(FdsVisibilityField, DerivesVisibilityFromSmokeDensity)
{
    TemporaryDirectory directory{};
    WriteSmokeFile(directory.Path() / "smoke.s3d");
    WriteSizeFile(directory.Path() / "smoke.s3d.sz");
    WriteSmvFile(directory.Path() / "case.smv");

    auto field =
        FdsVisibilityField::Load(directory.Path() / "case.smv", 1.0, 0.01, 3.0, 100.0);

    ASSERT_EQ(field.SliceCount(), 1);
    EXPECT_DOUBLE_EQ(field.SliceZ(), 1.0);
    EXPECT_DOUBLE_EQ(field.FirstTime(), 0.0);
    EXPECT_DOUBLE_EQ(field.LastTime(), 10.0);
    EXPECT_NEAR(*field.Sample(0.0, Point{0.5, 0.5}), 3.0, 1e-5);
    EXPECT_NEAR(*field.Sample(5.0, Point{0.5, 0.5}), 4.0, 1e-5);
    EXPECT_NEAR(*field.Sample(10.0, Point{0.5, 0.5}), 6.0, 1e-5);
    EXPECT_FALSE(field.Sample(5.0, Point{1.1, 0.5}).has_value());
}

TEST(FdsVisibilityField, RejectsMissingSootDensity)
{
    TemporaryDirectory directory{};
    WriteSmokeFile(directory.Path() / "smoke.s3d");
    WriteSizeFile(directory.Path() / "smoke.s3d.sz");
    WriteSmvFile(directory.Path() / "case.smv", "TEMPERATURE");

    EXPECT_THROW(
        FdsVisibilityField::Load(directory.Path() / "case.smv", 1.0, 0.01, 3.0, 100.0),
        std::runtime_error);
}

TEST(FdsVisibilityField, ReadsLegacyDoublePrecisionSmokeDensity)
{
    TemporaryDirectory directory{};
    WriteSmokeFile(directory.Path() / "smoke.s3dd");
    WriteLegacySizeFile(directory.Path() / "smoke.s3d.sz");
    WriteSmvFile(directory.Path() / "case.smv");

    auto field =
        FdsVisibilityField::Load(directory.Path() / "case.smv", 1.0, 0.01, 3.0, 100.0);

    EXPECT_NEAR(*field.Sample(5.0, Point{0.5, 0.5}), 4.0, 1e-5);
}

TEST(FdsVisibilityField, RejectsMeshOutsideEyeHeightTolerance)
{
    TemporaryDirectory directory{};
    WriteSmokeFile(directory.Path() / "smoke.s3d");
    WriteSizeFile(directory.Path() / "smoke.s3d.sz");
    WriteSmvFile(directory.Path() / "case.smv");

    EXPECT_THROW(
        FdsVisibilityField::Load(directory.Path() / "case.smv", 1.6, 0.1, 3.0, 100.0),
        std::runtime_error);
}
