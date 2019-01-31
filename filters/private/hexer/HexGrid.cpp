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


#include <cmath>
#include <algorithm>

#include "HexGrid.hpp"
#include "HexIter.hpp"
#include "Segment.hpp"

using namespace std;

namespace pdal
{
namespace hexer
{

namespace
{

// Compute hex size based on distance between consecutive points and
// density.  The probably needs some work based on more data.
double computeHexSize(const std::vector<Point>& samples, int density)
{
    auto distance = [](const Point& p1, const Point& p2)
    {
        double xdist = p2.m_x - p1.m_x;
        double ydist = p2.m_y - p1.m_y;
        return std::sqrt(xdist * xdist + ydist * ydist);
    };

    double dist = 0;
    for (std::vector<Point>::size_type i = 0; i < samples.size() - 1; ++i)
    {
        Point p1 = samples[i];
        Point p2 = samples[i + 1];
        dist += distance(p1, p2);
    }
    return ((density * dist) / samples.size());
}

} // unnamed namespace


HexGrid::HexGrid(int dense_limit) : SimpleHexGrid(),
    m_pos_roots(HexCompare()), m_dense_limit(dense_limit), m_miny(1)
{}


HexGrid::HexGrid(double height, int dense_limit) : SimpleHexGrid(height),
    m_pos_roots(HexCompare()), m_dense_limit(dense_limit), m_miny(1)
{}


bool HexGrid::dense(Hexagon *h)
{
    return h->count() >= m_dense_limit;
}

void HexGrid::addPoint(Point p)
{
    if (width() < 0)
    {
        m_sample.push_back(p);
        if (m_sample.size() >= m_maxSample)
            processSample();
        return;
    }

    Hexagon *h = findHexagon(p);
    h->increment();
    if (!h->dense())
    {
        if (dense(h))
        {
            h->setDense();
            m_miny = std::min(m_miny, h->y() - 1);
            if (h->possibleRoot())
                m_pos_roots.insert(h);
            markNeighborBelow(h);
        }
    }
}

void HexGrid::processSample()
{
    if (width() > 0 || m_sample.empty())
        return;

    double height = computeHexSize(m_sample, m_dense_limit);
    initialize(height);
    for (auto pi = m_sample.begin(); pi != m_sample.end(); ++pi)
        addPoint(*pi);
    m_sample.clear();
}

// A debugging function that can be used to make a particular hexagon
// dense.
void HexGrid::addDenseHexagon(HexKey key)
{
    Hexagon *h = getHexagon(key);
    if (!h->dense())
    {
        h->setCount(m_dense_limit);
        h->setDense();
        m_miny = std::min(m_miny, h->y() - 1);
        if (h->possibleRoot())
            m_pos_roots.insert(h);
        markNeighborBelow(h);
    }
}

HexIter HexGrid::hexBegin()
{
    return HexIter(m_hexes.begin(), this);
}

HexIter HexGrid::hexEnd()
{
    return HexIter(m_hexes.end(), this);
}

void HexGrid::markNeighborBelow(Hexagon *h)
{
    HexKey key = neighbor(h->key(), 3);
    Hexagon *below = getHexagon(key);
    below->setDenseNeighbor(0);
    if (below->dense() && !below->possibleRoot())
        m_pos_roots.erase(below);
}

Hexagon *HexGrid::findHexagon(Point p)
{
    HexKey key = SimpleHexGrid::findHexagon(p);
    if (m_hexes.empty())
    {
        // Make a hex at the origin and insert it.  Return a pointer
        // to the hexagon in the map.
        HexMap::value_type hexpair(key, Hexagon(0, 0));
        HexMap::iterator it = m_hexes.insert(hexpair).first;
        return &it->second;
    }
    return getHexagon(key);
}


Hexagon *HexGrid::getHexagon(HexKey key)
{
    // Stick a hexagon into the map if necessary.
    HexMap::value_type hexpair(key, Hexagon(key.x(), key.y()));
    std::pair<HexMap::iterator,bool> retval;
    retval = m_hexes.insert(hexpair);
    HexMap::iterator it = retval.first;

    Hexagon *hex_p = &(it->second);

    // Return a pointer to the located hexagon.
    return hex_p;
}

/**
// Walk the outside of the hexagons to make a path.  Hexagon sides are labeled:
//
//     __0_
//  1 /    \ 5
//   /      \
//   \      /
//  2 \____/ 4
//      3
**/
void HexGrid::findShapes()
{
    if (m_pos_roots.empty())
    {
        throw hexer_error("No areas of sufficient density - no shapes. "
            "Decrease density or area size.");
    }

    while (m_pos_roots.size())
    {
        Hexagon *h = *m_pos_roots.begin();
        findShape(h);
    }
}

void HexGrid::findParentPaths()
{
    std::vector<Path *> roots;
    for (size_t i = 0; i < m_paths.size(); ++i)
    {
        Path *p = m_paths[i];
        findParentPath(p);
        // Either add the path to the root list or the parent's list of
        // children.
        !p->parent() ?  roots.push_back(p) : p->parent()->addChild(p);
    }
    for (size_t i = 0; i < roots.size(); ++i)
       roots[i]->finalize(CLOCKWISE);

    // In the end, the list of paths is just the root paths.  Children can
    // be retrieved from their parents.
    m_paths = roots;
}

void HexGrid::findParentPath(Path *p)
{
    Segment s = p->rootSegment();
    Hexagon *h = s.hex();
    int y = h->y();
    while (y >= m_miny)
    {
        HexPathMap::iterator it = m_hex_paths.find(h);
        if (it != m_hex_paths.end())
        {
            Path *parentPath = it->second;
            if (parentPath == p->parent())
            {
               p->setParent(NULL);
            }
            else if (!p->parent() && parentPath != p)
            {
               p->setParent(parentPath);
            }
        }
        h = getHexagon(HexKey(h->x(), --y));
    }
}

void HexGrid::findShape(Hexagon *hex)
{
    if (!hex)
        throw hexer_error("hexagon was null!");

    Path *p = new Path(this, CLOCKWISE);
    Segment first(hex, 0);
    Segment cur(first);
    do {
        cleanPossibleRoot(cur, p);
        p->push_back(cur);
        Segment next = cur.leftClockwise(this);
        if (!next.hex()->dense())
            next = cur.rightClockwise(this);
        cur = next;
    } while (cur != first);
    m_paths.push_back(p);
}

void HexGrid::cleanPossibleRoot(Segment s, Path *p)
{
    if (s.possibleRoot(this))
        m_pos_roots.erase(s.hex());
    if (s.horizontal())
    {
        s.normalize(this);
        HexPathMap::value_type hexpath(s.hex(), p);
        m_hex_paths.insert(hexpath);
    }
}

void HexGrid::toWKT(std::ostream& output) const
{
    auto outputPath = [this,&output](size_t pathNum)
    {
        Path *p = rootPaths()[pathNum];

        output << "(";
        p->toWKT(output);
        output << ")";
    };

    output << "MULTIPOLYGON (";

    if (rootPaths().size())
        outputPath(0);
    for (size_t pi = 1; pi < rootPaths().size(); ++pi)
    {
        output << ",";
        outputPath(pi);
    }
    output << ")";
}

size_t HexGrid::densePointCount() const
{
    size_t count = 0;
    for (auto it = m_hexes.begin(); it != m_hexes.end(); ++it)
    {
        const Hexagon& h = it->second;
        if (h.dense())
            count += h.count();
    }
    return count;
}

} //namespace hexer
} //namespace pdal
