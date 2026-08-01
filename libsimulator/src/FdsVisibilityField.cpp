// SPDX-License-Identifier: LGPL-3.0-or-later
#include "FdsVisibilityField.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace
{
struct GridDefinition {
    int iMax{};
    int jMax{};
    int kMax{};
    std::vector<double> x{};
    std::vector<double> y{};
    std::vector<double> z{};
};

struct SmokeDefinition {
    int meshIndex{};
    std::filesystem::path file{};
    std::string longLabel{};
    double massExtinctionCoefficient{};
};

struct SizeRow {
    double time{};
    std::size_t uncompressedSize{};
    std::size_t compressedSize{};
    double maximumDensity{};
};

struct DensitySource {
    std::filesystem::path file{};
    std::vector<SizeRow> rows{};
};

std::string Trim(std::string value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if(first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string UpperAscii(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        if(c >= 'a' && c <= 'z') {
            return static_cast<char>(c - 'a' + 'A');
        }
        return static_cast<char>(c);
    });
    return value;
}

bool StartsWith(std::string_view value, std::string_view prefix)
{
    return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
}

std::size_t NextNonEmpty(const std::vector<std::string>& lines, std::size_t index)
{
    while(index < lines.size() && Trim(lines[index]).empty()) {
        ++index;
    }
    if(index >= lines.size()) {
        throw std::runtime_error("Unexpected end of Smokeview file");
    }
    return index;
}

std::vector<double> ParseCoordinates(
    const std::vector<std::string>& lines,
    std::size_t& lineIndex,
    std::size_t count,
    const std::string& axis)
{
    lineIndex = NextNonEmpty(lines, lineIndex + 1); // transformation flag/count
    std::vector<double> coordinates{};
    coordinates.reserve(count);
    for(std::size_t n = 0; n < count; ++n) {
        lineIndex = NextNonEmpty(lines, lineIndex + 1);
        std::istringstream input(Trim(lines[lineIndex]));
        int coordinateIndex = -1;
        double coordinate = 0.0;
        if(!(input >> coordinateIndex >> coordinate) ||
           coordinateIndex != static_cast<int>(n) || !std::isfinite(coordinate)) {
            throw std::runtime_error(
                "Invalid " + axis + " coordinate in Smokeview file at line " +
                std::to_string(lineIndex + 1));
        }
        coordinates.push_back(coordinate);
    }
    if(coordinates.size() < 2 ||
       !std::is_sorted(coordinates.begin(), coordinates.end(), std::less<double>{}) ||
       std::adjacent_find(coordinates.begin(), coordinates.end()) != coordinates.end()) {
        throw std::runtime_error(
            "FDS " + axis + " coordinates must be strictly increasing and contain two points");
    }
    return coordinates;
}

void ReadSmv(
    const std::filesystem::path& smvFile,
    std::vector<GridDefinition>& grids,
    std::vector<SmokeDefinition>& smokeFiles)
{
    std::ifstream input(smvFile);
    if(!input) {
        throw std::runtime_error("Cannot open FDS Smokeview file: " + smvFile.string());
    }

    std::vector<std::string> lines{};
    std::string line{};
    while(std::getline(input, line)) {
        lines.push_back(line);
    }

    GridDefinition* currentGrid = nullptr;
    for(std::size_t i = 0; i < lines.size(); ++i) {
        const auto token = Trim(lines[i]);
        if(StartsWith(token, "GRID")) {
            i = NextNonEmpty(lines, i + 1);
            std::istringstream dimensions(Trim(lines[i]));
            GridDefinition grid{};
            if(!(dimensions >> grid.iMax >> grid.jMax >> grid.kMax) || grid.iMax < 1 ||
               grid.jMax < 1 || grid.kMax < 1) {
                throw std::runtime_error(
                    "Invalid GRID dimensions in Smokeview file at line " +
                    std::to_string(i + 1));
            }
            grids.push_back(std::move(grid));
            currentGrid = &grids.back();
        } else if(token == "TRNX" && currentGrid != nullptr) {
            currentGrid->x = ParseCoordinates(
                lines, i, static_cast<std::size_t>(currentGrid->iMax) + 1, "X");
        } else if(token == "TRNY" && currentGrid != nullptr) {
            currentGrid->y = ParseCoordinates(
                lines, i, static_cast<std::size_t>(currentGrid->jMax) + 1, "Y");
        } else if(token == "TRNZ" && currentGrid != nullptr) {
            currentGrid->z = ParseCoordinates(
                lines, i, static_cast<std::size_t>(currentGrid->kMax) + 1, "Z");
        } else if(StartsWith(token, "SMOKF3D")) {
            std::istringstream header(token.substr(7));
            SmokeDefinition smoke{};
            if(!(header >> smoke.meshIndex >> smoke.massExtinctionCoefficient) ||
               smoke.meshIndex < 1 || !std::isfinite(smoke.massExtinctionCoefficient) ||
               smoke.massExtinctionCoefficient < 0.0) {
                throw std::runtime_error(
                    "Invalid SMOKF3D header in Smokeview file at line " +
                    std::to_string(i + 1));
            }
            i = NextNonEmpty(lines, i + 1);
            smoke.file = Trim(lines[i]);
            i = NextNonEmpty(lines, i + 1);
            smoke.longLabel = Trim(lines[i]);
            i = NextNonEmpty(lines, i + 1); // short label
            i = NextNonEmpty(lines, i + 1); // unit
            if(UpperAscii(smoke.longLabel).find("SOOT DENSITY") != std::string::npos) {
                if(smoke.massExtinctionCoefficient <= 0.0) {
                    throw std::runtime_error(
                        "Smoke3D SOOT DENSITY has no positive mass extinction coefficient");
                }
                smokeFiles.push_back(std::move(smoke));
            }
        }
    }

    if(grids.empty()) {
        throw std::runtime_error("FDS Smokeview file contains no structured GRID");
    }
    if(smokeFiles.empty()) {
        throw std::runtime_error(
            "FDS Smokeview file contains no SMOKF3D SOOT DENSITY data");
    }
    for(std::size_t i = 0; i < grids.size(); ++i) {
        const auto& grid = grids[i];
        if(grid.x.empty() || grid.y.empty() || grid.z.empty()) {
            throw std::runtime_error(
                "FDS GRID " + std::to_string(i + 1) + " is missing TRNX/TRNY/TRNZ data");
        }
    }
}

std::size_t ExactSize(double value, const std::string& context)
{
    if(!std::isfinite(value) || value < 0.0 || std::floor(value) != value ||
       value > static_cast<double>(std::numeric_limits<std::size_t>::max())) {
        throw std::runtime_error("Invalid size in Smoke3D metadata: " + context);
    }
    return static_cast<std::size_t>(value);
}

DensitySource ReadDensitySource(
    const std::filesystem::path& root,
    const std::filesystem::path& configuredS3d)
{
    const auto s3dFile = (root / configuredS3d).lexically_normal();
    auto sizeFile = s3dFile;
    sizeFile += ".sz";
    std::ifstream input(sizeFile);
    if(!input) {
        throw std::runtime_error(
            "Smoke3D size metadata referenced by .smv is missing: " + sizeFile.string());
    }

    std::vector<std::vector<double>> rawRows{};
    std::string line{};
    bool skippedVersion = false;
    while(std::getline(input, line)) {
        line = Trim(line);
        if(line.empty()) {
            continue;
        }
        if(!skippedVersion) {
            skippedVersion = true;
            continue;
        }
        std::istringstream rowInput(line);
        std::vector<double> row{};
        double value = 0.0;
        while(rowInput >> value) {
            row.push_back(value);
        }
        if(row.size() < 4) {
            throw std::runtime_error("Invalid Smoke3D size metadata row in " + sizeFile.string());
        }
        rawRows.push_back(std::move(row));
    }
    if(rawRows.empty()) {
        throw std::runtime_error("Smoke3D size metadata contains no frames: " + sizeFile.string());
    }

    const bool legacyDensityFormat = rawRows.front().size() >= 6;
    DensitySource result{};
    if(legacyDensityFormat) {
        result.file = s3dFile;
        result.file.replace_extension(".s3dd");
        if(!std::filesystem::exists(result.file)) {
            throw std::runtime_error(
                "This FDS Smoke3D format stores physical soot density in a missing .s3dd file: " +
                result.file.string());
        }
    } else {
        result.file = s3dFile;
    }

    result.rows.reserve(rawRows.size());
    for(std::size_t i = 0; i < rawRows.size(); ++i) {
        const auto& row = rawRows[i];
        const std::size_t compressedColumn = legacyDensityFormat ? 4 : 2;
        const std::size_t maximumColumn = legacyDensityFormat ? 5 : 3;
        if(row.size() <= maximumColumn || !std::isfinite(row[0]) || row[0] < 0.0 ||
           !std::isfinite(row[maximumColumn]) || row[maximumColumn] < 0.0) {
            throw std::runtime_error(
                "Invalid Smoke3D density metadata at frame " + std::to_string(i));
        }
        result.rows.push_back(SizeRow{
            row[0],
            ExactSize(row[1], "uncompressed frame"),
            ExactSize(row[compressedColumn], "compressed frame"),
            row[maximumColumn]});
    }
    return result;
}

std::uint32_t ReadU32(std::istream& input, const std::string& context)
{
    std::array<unsigned char, 4> bytes{};
    if(!input.read(reinterpret_cast<char*>(bytes.data()), bytes.size())) {
        throw std::runtime_error("Unexpected end of Smoke3D file while reading " + context);
    }
    return static_cast<std::uint32_t>(bytes[0]) |
           (static_cast<std::uint32_t>(bytes[1]) << 8U) |
           (static_cast<std::uint32_t>(bytes[2]) << 16U) |
           (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

float DecodeF32(const char* data)
{
    const auto* p = reinterpret_cast<const unsigned char*>(data);
    const std::uint32_t bits = static_cast<std::uint32_t>(p[0]) |
                               (static_cast<std::uint32_t>(p[1]) << 8U) |
                               (static_cast<std::uint32_t>(p[2]) << 16U) |
                               (static_cast<std::uint32_t>(p[3]) << 24U);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

std::vector<char> ReadRecord(std::istream& input, const std::string& context)
{
    const auto length = ReadU32(input, context + " record length");
    if(length > 64U * 1024U * 1024U) {
        throw std::runtime_error("Unreasonable record length in Smoke3D file: " + context);
    }
    std::vector<char> data(length);
    if(length > 0 && !input.read(data.data(), static_cast<std::streamsize>(length))) {
        throw std::runtime_error("Unexpected end of Smoke3D record: " + context);
    }
    if(ReadU32(input, context + " trailing record length") != length) {
        throw std::runtime_error("Mismatching Fortran record markers in Smoke3D file: " + context);
    }
    return data;
}

std::array<int, 6> ReadSmokeHeader(std::istream& input, const std::filesystem::path& file)
{
    const auto header = ReadRecord(input, "header");
    if(header.size() != 8 * sizeof(std::uint32_t)) {
        throw std::runtime_error("Unsupported Smoke3D header in " + file.string());
    }
    std::array<std::uint32_t, 8> values{};
    for(std::size_t i = 0; i < values.size(); ++i) {
        const auto* p = reinterpret_cast<const unsigned char*>(header.data()) + i * 4;
        values[i] = static_cast<std::uint32_t>(p[0]) |
                    (static_cast<std::uint32_t>(p[1]) << 8U) |
                    (static_cast<std::uint32_t>(p[2]) << 16U) |
                    (static_cast<std::uint32_t>(p[3]) << 24U);
    }
    if(values[0] != 1U) {
        throw std::runtime_error(
            "Unsupported Smoke3D byte order or header version in " + file.string());
    }
    return {
        static_cast<int>(values[2]),
        static_cast<int>(values[3]),
        static_cast<int>(values[4]),
        static_cast<int>(values[5]),
        static_cast<int>(values[6]),
        static_cast<int>(values[7])};
}

std::size_t CheckedExtent(int minimum, int maximum, const std::string& axis)
{
    if(minimum < 0 || maximum < minimum) {
        throw std::runtime_error("Invalid Smoke3D " + axis + " index bounds");
    }
    return static_cast<std::size_t>(maximum - minimum + 1);
}

std::vector<double> SubCoordinates(
    const std::vector<double>& coordinates,
    int minimum,
    int maximum,
    const std::string& axis)
{
    if(minimum < 0 || maximum < minimum ||
       static_cast<std::size_t>(maximum) >= coordinates.size()) {
        throw std::runtime_error("Smoke3D " + axis + " indices exceed its GRID coordinates");
    }
    return {
        coordinates.begin() + static_cast<std::ptrdiff_t>(minimum),
        coordinates.begin() + static_cast<std::ptrdiff_t>(maximum + 1)};
}

std::size_t NearestCoordinateIndex(const std::vector<double>& coordinates, double value)
{
    const auto upper = std::lower_bound(coordinates.begin(), coordinates.end(), value);
    if(upper == coordinates.begin()) {
        return 0;
    }
    if(upper == coordinates.end()) {
        return coordinates.size() - 1;
    }
    const auto upperIndex = static_cast<std::size_t>(upper - coordinates.begin());
    return value - coordinates[upperIndex - 1] <= coordinates[upperIndex] - value
               ? upperIndex - 1
               : upperIndex;
}

std::pair<std::size_t, double> InterpolationPosition(
    const std::vector<double>& coordinates,
    double value)
{
    if(value <= coordinates.front()) {
        return {0, 0.0};
    }
    if(value >= coordinates.back()) {
        return {coordinates.size() - 2, 1.0};
    }
    const auto upper = std::upper_bound(coordinates.begin(), coordinates.end(), value);
    const auto lowerIndex = static_cast<std::size_t>(upper - coordinates.begin() - 1);
    const double lower = coordinates[lowerIndex];
    const double fraction = (value - lower) / (coordinates[lowerIndex + 1] - lower);
    return {lowerIndex, fraction};
}

std::vector<float> ReadDensityPlane(
    const std::filesystem::path& file,
    std::uint64_t dataOffset,
    std::uint32_t compressedSize,
    double maximumDensity,
    std::size_t fullValueCount,
    std::size_t planeOffset,
    std::size_t planeValueCount)
{
    std::ifstream input(file, std::ios::binary);
    if(!input) {
        throw std::runtime_error("Cannot reopen Smoke3D density file: " + file.string());
    }
    input.seekg(static_cast<std::streamoff>(dataOffset));
    if(!input) {
        throw std::runtime_error("Cannot seek in Smoke3D density file: " + file.string());
    }
    std::vector<unsigned char> compressed(compressedSize);
    if(compressedSize > 0 &&
       !input.read(
           reinterpret_cast<char*>(compressed.data()),
           static_cast<std::streamsize>(compressed.size()))) {
        throw std::runtime_error("Cannot read Smoke3D density frame from: " + file.string());
    }

    std::vector<float> plane(planeValueCount, 0.0F);
    const std::size_t planeEnd = planeOffset + planeValueCount;
    std::size_t inputPosition = 0;
    std::size_t outputPosition = 0;
    while(inputPosition < compressed.size()) {
        const unsigned char markerOrValue = compressed[inputPosition++];
        unsigned char value = markerOrValue;
        std::size_t repeats = 1;
        if(markerOrValue == 255U) {
            if(inputPosition + 1 >= compressed.size()) {
                throw std::runtime_error("Truncated Smoke3D RLE sequence in " + file.string());
            }
            value = compressed[inputPosition++];
            repeats = compressed[inputPosition++];
            if(repeats == 0) {
                throw std::runtime_error("Invalid zero-length Smoke3D RLE sequence");
            }
        }
        if(outputPosition + repeats > fullValueCount) {
            throw std::runtime_error("Smoke3D RLE data expands beyond its declared size");
        }
        const std::size_t runEnd = outputPosition + repeats;
        const std::size_t overlapBegin = std::max(outputPosition, planeOffset);
        const std::size_t overlapEnd = std::min(runEnd, planeEnd);
        if(overlapBegin < overlapEnd) {
            const float density = static_cast<float>(
                (static_cast<double>(value) / 254.0) * maximumDensity);
            std::fill(
                plane.begin() + static_cast<std::ptrdiff_t>(overlapBegin - planeOffset),
                plane.begin() + static_cast<std::ptrdiff_t>(overlapEnd - planeOffset),
                density);
        }
        outputPosition = runEnd;
    }
    if(outputPosition != fullValueCount) {
        throw std::runtime_error("Smoke3D RLE data does not expand to its declared size");
    }
    return plane;
}
} // namespace

FdsVisibilityField FdsVisibilityField::Load(
    const std::filesystem::path& smvFile,
    double targetZ,
    double zTolerance,
    double visibilityFactor,
    double maximumVisibility)
{
    if(!std::isfinite(targetZ)) {
        throw std::runtime_error("FDS target Z must be finite");
    }
    if(!std::isfinite(zTolerance) || zTolerance < 0.0) {
        throw std::runtime_error("FDS Smoke3D Z tolerance must be finite and >= 0");
    }
    if(!std::isfinite(visibilityFactor) || visibilityFactor <= 0.0) {
        throw std::runtime_error("FDS visibility factor must be finite and > 0");
    }
    if(!std::isfinite(maximumVisibility) || maximumVisibility <= 0.0) {
        throw std::runtime_error("FDS maximum visibility must be finite and > 0");
    }

    const auto absoluteSmv = std::filesystem::absolute(smvFile).lexically_normal();
    std::vector<GridDefinition> grids{};
    std::vector<SmokeDefinition> definitions{};
    ReadSmv(absoluteSmv, grids, definitions);

    FdsVisibilityField result{};
    result._smvFile = absoluteSmv;
    result._sliceZ = targetZ;
    result._firstTime = std::numeric_limits<double>::max();
    result._lastTime = std::numeric_limits<double>::lowest();
    result._visibilityFactor = visibilityFactor;
    result._maximumVisibility = maximumVisibility;

    for(const auto& definition : definitions) {
        const auto gridIndex = static_cast<std::size_t>(definition.meshIndex - 1);
        if(gridIndex >= grids.size()) {
            throw std::runtime_error(
                "FDS SMOKF3D refers to missing GRID " + std::to_string(definition.meshIndex));
        }
        const auto& grid = grids[gridIndex];
        const auto nearestK = NearestCoordinateIndex(grid.z, targetZ);
        if(std::abs(grid.z[nearestK] - targetZ) > zTolerance) {
            continue;
        }

        const auto source =
            ReadDensitySource(absoluteSmv.parent_path(), definition.file);
        std::ifstream input(source.file, std::ios::binary);
        if(!input) {
            throw std::runtime_error(
                "Smoke3D soot density file referenced by .smv is missing: " +
                source.file.string());
        }
        const auto bounds = ReadSmokeHeader(input, source.file);
        const auto nx = CheckedExtent(bounds[0], bounds[1], "I");
        const auto ny = CheckedExtent(bounds[2], bounds[3], "J");
        const auto nz = CheckedExtent(bounds[4], bounds[5], "K");
        if(nx < 2 || ny < 2) {
            throw std::runtime_error(
                "Smoke3D mesh needs at least 2x2 horizontal points: " + source.file.string());
        }
        if(nearestK < static_cast<std::size_t>(bounds[4]) ||
           nearestK > static_cast<std::size_t>(bounds[5])) {
            continue;
        }

        Slice slice{};
        slice.file = source.file;
        slice.z = grid.z[nearestK];
        slice.x = SubCoordinates(grid.x, bounds[0], bounds[1], "X");
        slice.y = SubCoordinates(grid.y, bounds[2], bounds[3], "Y");
        slice.fullValueCount = nx * ny * nz;
        slice.planeValueCount = nx * ny;
        slice.planeOffset =
            (nearestK - static_cast<std::size_t>(bounds[4])) * slice.planeValueCount;
        slice.cellArea =
            ((slice.x.back() - slice.x.front()) / static_cast<double>(nx - 1)) *
            ((slice.y.back() - slice.y.front()) / static_cast<double>(ny - 1));
        slice.massExtinctionCoefficient = definition.massExtinctionCoefficient;

        std::size_t frameIndex = 0;
        while(input.peek() != std::char_traits<char>::eof()) {
            if(frameIndex >= source.rows.size()) {
                throw std::runtime_error(
                    "Smoke3D binary file has more frames than its .sz metadata: " +
                    source.file.string());
            }
            const auto timeRecord = ReadRecord(input, "time");
            if(timeRecord.size() != sizeof(float)) {
                throw std::runtime_error("Invalid time record in Smoke3D file: " + source.file.string());
            }
            const double frameTime = DecodeF32(timeRecord.data());
            const auto sizesRecord = ReadRecord(input, "frame sizes");
            if(sizesRecord.size() != 2 * sizeof(std::uint32_t)) {
                throw std::runtime_error("Invalid size record in Smoke3D file: " + source.file.string());
            }
            std::istringstream sizesInput(
                std::string(sizesRecord.data(), sizesRecord.size()),
                std::ios::in | std::ios::binary);
            const auto uncompressedSize = ReadU32(sizesInput, "uncompressed frame size");
            const auto compressedSize = ReadU32(sizesInput, "compressed frame size");
            const auto& metadata = source.rows[frameIndex];
            const double timeTolerance = 1e-4 * std::max(1.0, std::abs(frameTime));
            if(!std::isfinite(frameTime) ||
               (!slice.frames.empty() && frameTime <= slice.frames.back().time) ||
               std::abs(frameTime - metadata.time) > timeTolerance ||
               uncompressedSize != slice.fullValueCount ||
               metadata.uncompressedSize != slice.fullValueCount ||
               compressedSize != metadata.compressedSize) {
                throw std::runtime_error(
                    "Smoke3D binary frames do not match their .sz metadata: " +
                    source.file.string());
            }

            const auto recordLength = ReadU32(input, "compressed data record length");
            if(recordLength != compressedSize) {
                throw std::runtime_error("Smoke3D compressed record length mismatch");
            }
            const auto offset = input.tellg();
            if(offset < 0) {
                throw std::runtime_error("Cannot determine Smoke3D frame offset");
            }
            input.seekg(static_cast<std::streamoff>(compressedSize), std::ios::cur);
            if(!input || ReadU32(input, "trailing compressed record length") != compressedSize) {
                throw std::runtime_error(
                    "Incomplete or corrupt Smoke3D data record: " + source.file.string());
            }
            slice.frames.push_back(FrameReference{
                frameTime,
                static_cast<std::uint64_t>(offset),
                compressedSize,
                metadata.maximumDensity});
            ++frameIndex;
        }
        if(slice.frames.empty()) {
            throw std::runtime_error("Smoke3D density file contains no frames: " + source.file.string());
        }
        result._firstTime = std::min(result._firstTime, slice.frames.front().time);
        result._lastTime = std::max(result._lastTime, slice.frames.back().time);
        result._slices.push_back(std::move(slice));
    }

    if(result._slices.empty()) {
        throw std::runtime_error(
            "No Smoke3D SOOT DENSITY mesh has a grid plane within the requested eye-height "
            "tolerance");
    }
    return result;
}

std::optional<double> FdsVisibilityField::Sample(double time, Point position)
{
    if(!std::isfinite(time) || !std::isfinite(position.x) || !std::isfinite(position.y)) {
        throw std::runtime_error("FDS visibility sample time and position must be finite");
    }

    Slice* selected = nullptr;
    for(auto& slice : _slices) {
        if(position.x < slice.x.front() || position.x > slice.x.back() ||
           position.y < slice.y.front() || position.y > slice.y.back()) {
            continue;
        }
        if(selected == nullptr || slice.cellArea < selected->cellArea) {
            selected = &slice;
        }
    }
    if(selected == nullptr) {
        return std::nullopt;
    }

    const auto upper = std::upper_bound(
        selected->frames.begin(),
        selected->frames.end(),
        time,
        [](double value, const FrameReference& frame) { return value < frame.time; });
    std::size_t lowerIndex = 0;
    std::size_t upperIndex = 0;
    if(upper == selected->frames.begin()) {
        lowerIndex = upperIndex = 0;
    } else if(upper == selected->frames.end()) {
        lowerIndex = upperIndex = selected->frames.size() - 1;
    } else {
        upperIndex = static_cast<std::size_t>(upper - selected->frames.begin());
        lowerIndex = upperIndex - 1;
    }

    if(selected->lowerCacheIndex != lowerIndex) {
        if(selected->upperCacheIndex == lowerIndex) {
            selected->lowerCache = selected->upperCache;
        } else {
            selected->lowerCache = ReadDensityPlane(
                selected->file,
                selected->frames[lowerIndex].dataOffset,
                selected->frames[lowerIndex].compressedSize,
                selected->frames[lowerIndex].maximumDensity,
                selected->fullValueCount,
                selected->planeOffset,
                selected->planeValueCount);
        }
        selected->lowerCacheIndex = lowerIndex;
    }
    if(upperIndex == lowerIndex) {
        selected->upperCache = selected->lowerCache;
        selected->upperCacheIndex = upperIndex;
    } else if(selected->upperCacheIndex != upperIndex) {
        selected->upperCache = ReadDensityPlane(
            selected->file,
            selected->frames[upperIndex].dataOffset,
            selected->frames[upperIndex].compressedSize,
            selected->frames[upperIndex].maximumDensity,
            selected->fullValueCount,
            selected->planeOffset,
            selected->planeValueCount);
        selected->upperCacheIndex = upperIndex;
    }

    const auto [i, fx] = InterpolationPosition(selected->x, position.x);
    const auto [j, fy] = InterpolationPosition(selected->y, position.y);
    const auto nx = selected->x.size();
    const auto interpolateSpatially = [i, j, fx, fy, nx](const std::vector<float>& values) {
        const double v00 = values[i + nx * j];
        const double v10 = values[i + 1 + nx * j];
        const double v01 = values[i + nx * (j + 1)];
        const double v11 = values[i + 1 + nx * (j + 1)];
        const double lower = v00 + (v10 - v00) * fx;
        const double upperValue = v01 + (v11 - v01) * fx;
        return lower + (upperValue - lower) * fy;
    };

    double density = interpolateSpatially(selected->lowerCache);
    if(lowerIndex != upperIndex) {
        const double lowerTime = selected->frames[lowerIndex].time;
        const double upperTime = selected->frames[upperIndex].time;
        const double fraction =
            std::clamp((time - lowerTime) / (upperTime - lowerTime), 0.0, 1.0);
        density +=
            (interpolateSpatially(selected->upperCache) - density) * fraction;
    }

    const double extinction = selected->massExtinctionCoefficient * std::max(0.0, density);
    if(extinction <= std::numeric_limits<double>::epsilon()) {
        return _maximumVisibility;
    }
    return std::min(_maximumVisibility, _visibilityFactor / extinction);
}

std::size_t FdsVisibilityField::SliceCount() const noexcept
{
    return _slices.size();
}

double FdsVisibilityField::SliceZ() const noexcept
{
    return _sliceZ;
}

double FdsVisibilityField::FirstTime() const noexcept
{
    return _firstTime;
}

double FdsVisibilityField::LastTime() const noexcept
{
    return _lastTime;
}

const std::filesystem::path& FdsVisibilityField::SmvFile() const noexcept
{
    return _smvFile;
}
