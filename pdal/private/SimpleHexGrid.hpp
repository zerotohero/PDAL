/******************************************************************************
 * Copyright (c) 2014, Hobu Inc. (howard@hobu.co)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following
 * conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of the Howard Butler or Hobu, Inc.
 *       the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 ****************************************************************************/

#pragma once

#include <cstdint>
#include <vector>

#include "Mathpair.hpp"

namespace pdal
{

static const double SQRT_3 = 1.732050808;

struct HexKey
{
    HexKey(int32_t x, int32_t y)
    {
        m_key.p.m_x = x;
        m_key.p.m_y = y;
    }

    int32_t x() const
        { return m_key.p.m_x; }
    int32_t y() const
        { return m_key.p.m_y; }
    // A key represents an even position if the X coordinate is even.
    bool even() const
        { return (bool)((m_key.p.m_x & 0x1) == 0); }

    bool operator < (const HexKey& other) const
        { return m_key.m_val < other.m_key.m_val; }
    bool operator == (const HexKey& other) const
        { return m_key.m_val == other.m_key.m_val; }

#pragma pack(push)
#pragma pack(1)
    union
    {
        struct
        {
            int32_t m_x;
            int32_t m_y;
        } p;
        uint64_t m_val;
    } m_key;
};
#pragma pack(pop)

}

// Hash function for HexKey.
namespace std
{
    template<>
    struct hash<::pdal::HexKey>
    {
        static std::hash<uint64_t> hasher;

        size_t operator()(const pdal::HexKey key) const
            { return hasher(key.m_key.m_val); }
    };
}

namespace pdal
{

class SimpleHexGrid
{
protected:
    using Point = Mathpair<double>;

public:
    SimpleHexGrid();
    SimpleHexGrid(double height);

    HexKey findHexagon(Point p);
    std::vector<HexKey> neighbors(HexKey key) const;
    HexKey neighbor(HexKey key, unsigned pos) const;
    double width() const
        { return m_width; }
    double height() const
        { return m_height; }
    Point const& offset(int idx) const
        { return m_offsets[idx]; }
    Point centerOffset(int idx) const
        { return (m_offsets[idx] - m_center_offset); }
    Point const& origin() const
        { return m_origin; }

protected:
    void initialize(double height);

private:
    /// Height of the hexagons in the grid (2x apothem)
    double m_height;
    /// Width of the hexagons in the grid
    double m_width;
    /// Origin of the hex grid in point coordinates.
    Point m_origin;
    /// Offsets of vertices of hexagon, going anti-clockwise from upper-left
    Point m_offsets[6];
    /// Offset of the center of the hexagons.
    Point m_center_offset;
};

} // namespace

