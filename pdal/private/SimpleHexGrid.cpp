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

#include <cassert>
#include <cmath>

#include "SimpleHexGrid.hpp"

using namespace std;

namespace pdal
{

SimpleHexGrid::SimpleHexGrid() : m_height(-1.0), m_width(-1.0),
    m_origin(std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max())
{}

SimpleHexGrid::SimpleHexGrid(double height) : 
    m_origin(std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max())
{
    initialize(height);
}


void SimpleHexGrid::initialize(double height)
{
    m_height = height;
    m_width = (3 / (2 * SQRT_3)) * m_height;

    m_offsets[0] = Point(0, 0);
    m_offsets[1] = Point(-m_width / 3, m_height / 2);
    m_offsets[2] = Point(0, m_height);
    m_offsets[3] = Point(2 * m_width / 3, m_height);
    m_offsets[4] = Point(m_width, m_height / 2);
    m_offsets[5] = Point(2 * m_width / 3, 0);
    m_center_offset = Point(m_width / 3, m_height / 2);
}

//  first point (origin) and start of column 0
//   |
//   |
//   |  ------- column 0
//   |  |
//   |  |   end of column 0 - start of column 1
//   |  |   |
//   |  v   |                             _____
//   v____  v                            |\    |
//   /    \                              | \   |  Here's an expansion of what
//  / 0,0  \____                         |  \  |  I'm calling a mini-column,
//  \      /    \                        |   \ |  showing two half rows.
//   \____/ 1,0  \                       |____\|
//   /    \      /                       |    /|  The top rectangle is the
//  / 0,1  \____/                        |   / |  negative slope case and
//  \      /    \                        |  /  |  the lower rectangle is the
//   \____/      \                       | /   |  positive slope case.
//                                       |/____|
//        ** <--  The area above these
//                asterisks are the "mini-column"
//                The mini-column is 1/3 the total width of the column.
//
// We are creating a tesselated plane of hexagons where one side of each
// hexagon is parallel with the X axis.  We think of the columns of
// hexagons as all having the same X value.  Hexagons lower than their
// neighbor have successive Y values.
//
// The hexagon in the second column but just below the hexagon in the first
// column as the same Y value as the hexagon above and to the left.  The third
// column's Y values are the one less than the hexagon below and to the left
// as the second column.
//
// The first point, whatever it's X/Y location, is made the origin, and is
// placed at the top-left edge of hexagon 0,0.
//
HexKey SimpleHexGrid::findHexagon(Point p)
{
    int x, y;

    if (m_origin.m_x == std::numeric_limits<double>::max())
    {
        m_origin = p;
        return HexKey(0, 0);
    }

    // Offset by the origin.
    p -= m_origin;

    double col = p.m_x / m_width;

    // First calculate X and Y as if we had a bunch of offset rectangles.
    // This works for 2/3 of the width of the hexagons.
    x = (int)std::floor(col);
    if (x % 2 == 0)
        y = static_cast<int>(std::floor(p.m_y / m_height));
    else
        y = static_cast<int>(std::floor((p.m_y - (m_height / 2)) / m_height));

    // Compute the column remainder to determine if we are in a strip where
    // the hexagons overlap (the mini-column).
    double xcolOffset = col - std::floor(col);
    if (xcolOffset > 2.0/3.0)
    {
        // Calculate the xvalue as a fraction of the width of the column-piece
        // containing multiple hex columns.  These overlap columns are 1/3
        // the total width of any column.

        // Subtract the 2/3 of the value not relevant to the mini-column.
        xcolOffset -= 2.0/3.0;
        // Scale the value to the width of the mini-column.
        xcolOffset *= 3.0;

        // Each halfrow contains a single sloping edge of a hexagon.
        // The slope of the edge is either sqrt(3) or -sqrt(3).  The edge
        // extends from top left to lower right or from bottom left to top
        // right.  What we do here is compute the horizontal fraction of
        // the box (xcolOffset) and the vertical fraction of the box
        // (yrowOffset) and then compare them.
        double halfrow = p.m_y / (m_height / 2);
        int halfy = (int)halfrow;
        double yrowOffset = halfrow - std::floor(halfrow);

        // Negative slope case.
        if ((halfy % 2 == 0 && x % 2 == 0) || (x % 2 && halfy % 2))
        {
            if (xcolOffset > yrowOffset)
            {
                if (x % 2 == 0)
                    y--;
                x++;
            }
        }
        // Positive slope case.
        else
        {
            if (yrowOffset > xcolOffset)
            {
                if (x % 2)
                    y++;
                x++;
            }
        }
    }
    return HexKey(x, y);
}

std::vector<HexKey> SimpleHexGrid::neighbors(HexKey key) const
{
    std::vector<HexKey> keys;

    int32_t x(key.x());
    int32_t y(key.y());

    keys.emplace_back(x, y - 1);
    keys.emplace_back(x, y + 1);
    if (key.even())
    {
        keys.emplace_back(x - 1, y - 1);
        keys.emplace_back(x - 1, y);
        keys.emplace_back(x + 1, y - 1);
        keys.emplace_back(x + 1, y);
    }
    else
    {
        keys.emplace_back(x - 1, y);
        keys.emplace_back(x - 1, y + 1);
        keys.emplace_back(x + 1, y);
        keys.emplace_back(x + 1, y + 1);
    }
    return keys;
}


/**
// The sides of the hexagon are labeled as shown:
//
//     __0_
//  1 /    \ 5
//   /      \
//   \      /
//  2 \____/ 4
//      3
**/
HexKey SimpleHexGrid::neighbor(HexKey key, unsigned pos) const
{
    assert(pos < 6);

    int32_t x(key.x());
    int32_t y(key.y());
    
    switch (pos)
    {
    case 0:
        return HexKey(x, y - 1);
    case 1:
        return HexKey(x - 1, key.even() ? y - 1 : y);
    case 2:
        return HexKey(x - 1, key.even() ? y : y + 1);
    case 3:
        return HexKey(x, y + 1);
    case 4:
        return HexKey(x + 1, key.even() ? y : y + 1);
    case 5:
        return HexKey(x + 1, key.even() ? y - 1 : y);
    default:
        return HexKey(x, y); // Should never get here - throw?
    }
}

} //namespace hexer
