// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Point.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <vector>

/// Time-dependent horizontal visibility field derived from FDS Smoke3D soot
/// density data referenced by Smokeview metadata.
class FdsVisibilityField
{
    struct FrameReference {
        double time{};
        std::uint64_t dataOffset{};
        std::uint32_t compressedSize{};
        double maximumDensity{};
    };

    struct Slice {
        std::filesystem::path file{};
        double z{};
        std::vector<double> x{};
        std::vector<double> y{};
        std::vector<FrameReference> frames{};
        std::size_t fullValueCount{};
        std::size_t planeOffset{};
        std::size_t planeValueCount{};
        double cellArea{};
        double massExtinctionCoefficient{};

        std::size_t lowerCacheIndex{std::numeric_limits<std::size_t>::max()};
        std::size_t upperCacheIndex{std::numeric_limits<std::size_t>::max()};
        std::vector<float> lowerCache{};
        std::vector<float> upperCache{};
    };

    std::filesystem::path _smvFile{};
    double _sliceZ{};
    double _firstTime{};
    double _lastTime{};
    double _visibilityFactor{};
    double _maximumVisibility{};
    std::vector<Slice> _slices{};

public:
    /// Loads Smoke3D SOOT DENSITY data and selects the grid plane nearest to
    /// targetZ for every FDS mesh.
    static FdsVisibilityField Load(
        const std::filesystem::path& smvFile,
        double targetZ,
        double zTolerance,
        double visibilityFactor,
        double maximumVisibility);

    /// Samples visibility in metres using bilinear spatial and linear temporal
    /// interpolation. Returns no value when the point is outside every loaded
    /// FDS mesh.
    std::optional<double> Sample(double time, Point position);

    std::size_t SliceCount() const noexcept;
    double SliceZ() const noexcept;
    double FirstTime() const noexcept;
    double LastTime() const noexcept;
    const std::filesystem::path& SmvFile() const noexcept;
};
