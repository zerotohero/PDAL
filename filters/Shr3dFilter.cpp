/******************************************************************************
 * Copyright (c) 2017, Bradley J Chambers (brad.chambers@gmail.com)
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
 *     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
 *       names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior
 *       written permission.
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

// PDAL implementation of S. Almes, S. Hagstrom, D. Chilcott, H. Goldberg, M.
// Brown, “Open Source Geospatial Tools to Enable Large Scale 3D Scene
// Modeling,” FOSS4G, 2017.

// Original code released under MIT license at https://github.com/pubgeo/pubgeo.

#include "Shr3dFilter.hpp"

#include <pdal/EigenUtils.hpp>
#include <pdal/KDIndex.hpp>
#include <pdal/Segmentation.hpp>
//#include <pdal/pdal_macros.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "private/DimRange.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace pdal
{

using namespace Dimension;
using namespace Eigen;
using namespace eigen;

static PluginInfo const s_info
{
    "filters.shr3d",
    "Shareable High Resolution 3D (Almes et al., 2017)",
    "http://pdal.io/stages/filters.shr3d.html"
};

CREATE_STATIC_STAGE(Shr3dFilter, s_info)

std::string Shr3dFilter::getName() const
{
    return s_info.name;
}

void Shr3dFilter::addArgs(ProgramArgs& args)
{
    args.add("dh", "Horizontal uncertainty", m_dh, 0.5);
    args.add("dz", "Vertical uncertainty", m_dz, 0.5);
    args.add("agl", "Minimum building height", m_agl, 2.0);
    args.add("area", "Minimum building area", m_area, 50.0);
    args.add("dir", "Optional output directory for debugging", m_dir);
    args.add("ignore", "Ignore values", m_ignored);
    args.add("last", "Consider last returns only?", m_lastOnly, true);
}

void Shr3dFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Id::Classification);
}

void Shr3dFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());

    m_ignored.m_id = layout->findDim(m_ignored.m_name);

    if (m_lastOnly)
    {
        if (!layout->hasDim(Dimension::Id::ReturnNumber) ||
            !layout->hasDim(Dimension::Id::NumberOfReturns))
        {
            log()->get(LogLevel::Warning) << "Could not find ReturnNumber and "
                                             "NumberOfReturns. Skipping "
                                             "segmentation of last returns and "
                                             "proceeding with all returns.\n";
            m_lastOnly = false;
        }
    }
}

void Shr3dFilter::ready(PointTableRef table)
{
    if (m_dir.empty())
        return;

    if (!FileUtils::directoryExists(m_dir))
        throwError("Output directory '" + m_dir + "' does not exist");
}

PointViewSet Shr3dFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;
    if (!view->size())
        return viewSet;

    // Segment input view into ignored/kept views.
    PointViewPtr ignoredView = view->makeNew();
    PointViewPtr keptView = view->makeNew();
    if (m_ignored.m_id == Dimension::Id::Unknown)
        keptView->append(*view);
    else
        Segmentation::ignoreDimRange(m_ignored, view, keptView, ignoredView);

    // Segment kept view into last/other-than-last return views.
    PointViewPtr lastView = keptView->makeNew();
    PointViewPtr nonlastView = keptView->makeNew();
    if (m_lastOnly)
        Segmentation::segmentLastReturns(keptView, lastView, nonlastView);
    else
        lastView->append(*keptView);

    for (PointId i = 0; i < nonlastView->size(); ++i)
        nonlastView->setField(Dimension::Id::Classification, i, 1);

    m_srs = lastView->spatialReference();

    lastView->calculateBounds(m_bounds);
    m_cols = ((m_bounds.maxx - m_bounds.minx) / m_dh) + 1;
    m_rows = ((m_bounds.maxy - m_bounds.miny) / m_dh) + 1;

    // Create raster of minimum Z values per element, median filtered and void
    // filled.
    std::vector<double> ZImin = createZImin(lastView);

    // Create raster of maximum Z values per element, median filtered and void
    // filled.
    std::vector<double> ZImax = createZImax(lastView);

    // Mask trees -> write DSM2
    std::vector<double> ZIdsm2 = createDSM2(ZImin, ZImax);

    // Create label and DTM images (latter initialized to ZImin)
    int dh_bins = 1;
    // need to recalculate dsm_scale
    auto result = std::minmax_element(ZImax.begin(), ZImax.end());
    double scale = (*result.second - *result.first) / (std::pow(2.0, 8) - 1);
    unsigned int dz_short = m_dz / scale;
    unsigned int agl_short = m_agl / scale;

    std::vector<double> ZIlabel(m_rows * m_cols,
                                0.0); // init to zeros or something
    std::vector<double> ZIdtm = ZImin;

    // Classify ground.
    classifyGround(lastView, ZIlabel, ZIdtm, dh_bins, dz_short);

    // Create DTM voids where DSM voids exist
    // Median filter DTM
    // Classify non-ground.
    // Fill voids in DTM.
    // Write DTM.
    // Create classification raster.
    // Fill labels inside building regions
    // Write classification raster
    // Write building raster

    PointViewPtr outView = view->makeNew();
    outView->append(*ignoredView);
    outView->append(*nonlastView);
    // outView->append(*lastView);
    viewSet.insert(outView);

    return viewSet;
}

void Shr3dFilter::labelObjectBoundaries(std::vector<double>& ZIdtm,
                                        std::vector<double>& ZIlabelV,
                                        int edgeResolution,
                                        unsigned int minDistanceShortValue)
{
    // Initialize the labels to LABEL_GROUND.
    for (int c = 0; c < m_cols; ++c)
    {
        for (int r = 0; r < m_rows; ++r)
        {
            ZIlabelV[c * m_rows + r] = 2;
        }
    }

    // Mark the label image with object boundaries.
    int radius = 2;
    float threshold = (float)minDistanceShortValue;
    for (int c = 0; c < m_cols; ++c)
    {
        for (int r = 0; r < m_rows; ++r)
        {
            // Look for Z steps greater than a threshold.
            float value = ZIdtm[c * m_rows + r];

            // Interestingly, this works about as well as checking every step.
            for (int dc = -edgeResolution; dc <= edgeResolution;
                 dc += edgeResolution)
            {
                for (int dr = -edgeResolution; dr <= edgeResolution;
                     dr += edgeResolution)
                {
                    int r2 = std::min(std::max(0, r + dr), m_rows);
                    int c2 = std::min(std::max(0, c + dc), m_cols);
                    if (ZIdtm[c2 * m_rows + r2] != 0)
                    {
                        // Remove local slope to avoid tagging rough terrain.
                        int r3 = std::min(std::max(0, r + dr * 2), m_rows - 1);
                        int c3 = std::min(std::max(0, c + dc * 2), m_cols - 1);
                        float myGradient = (float)ZIdtm[c * m_rows + r] -
                                           (float)ZIdtm[c2 * m_rows + r2];
                        float neighborGradient =
                            (float)ZIdtm[c2 * m_rows + r2] -
                            (float)ZIdtm[c3 * m_rows + r3];
                        float distance = (myGradient - neighborGradient);
                        if (distance > threshold)
                            ZIlabelV[c * m_rows + r] = 1;
                    }
                }
            }
        }
    }

    if (!m_dir.empty())
    {
        std::string fname = FileUtils::toAbsolutePath("zilabel.tif", m_dir);
        MatrixXd ZIlabel = Map<MatrixXd>(ZIlabelV.data(), m_rows, m_cols);
        writeMatrix(ZIlabel, fname, "GTiff", m_dh, m_bounds, m_srs);
    }
}

// Extend object boundaries to capture points missed around the edges.
void Shr3dFilter::extendObjectBoundaries(std::vector<double>& ZIdtm,
                                         std::vector<double>& ZIlabel,
                                         int edgeResolution,
                                         unsigned int minDistanceShortValue)
{
    // Loop enough to capture the edge resolution.
    for (unsigned int k = 0; k < edgeResolution; ++k)
    {
        // First, label any close neighbor LABEL_TEMP.
        for (unsigned int r = 1; r < m_rows - 1; ++r)
        {
            for (unsigned int c = 1; c < m_cols - 1; ++c)
            {
                // For any labeled point, check all neighbors.
                if (ZIlabel[c * m_rows + r] == 1)
                {
                    for (unsigned int rr = r - 1; rr <= r + 1; ++rr)
                    {
                        for (unsigned int cc = c - 1; cc <= c + 1; ++cc)
                        {
                            if (ZIlabel[cc * m_rows + rr] == 1)
                                continue;
                            if (((float)ZIdtm[c * m_rows + r] -
                                 (float)ZIdtm[cc * m_rows + rr]) <
                                minDistanceShortValue / 2.0)
                            {
                                ZIlabel[cc * m_rows + rr] = 9;
                            }
                        }
                    }
                }
            }
        }

        // Then label any high points labeled LABEL_TEMP as an object of
        // interest.
        for (unsigned int r = 0; r < m_rows; ++r)
        {
            for (unsigned int c = 0; c < m_cols; ++c)
            {
                if (ZIlabel[c * m_rows + r] == 9)
                {
                    // Check to make sure this point is also higher than one of
                    // its neighbors.
                    unsigned int r1 = std::max(0u, r - 1u);
                    unsigned int r2 = std::min(r + 1u, (unsigned int)m_rows - 1);
                    unsigned int c1 = std::max(0u, c - 1u);
                    unsigned int c2 = std::min(c + 1u, (unsigned int)m_cols - 1);
                    for (unsigned int rr = r1; rr <= r2; ++rr)
                    {
                        for (unsigned int cc = c1; cc <= c2; ++cc)
                        {
                            if (((float)ZIdtm[c * m_rows + r] -
                                 (float)ZIdtm[cc * m_rows + rr]) >
                                minDistanceShortValue / 2.0)
                            {
                                ZIlabel[c * m_rows + r] = 1;
                            }
                        }
                    }
                }
            }
        }
    }

    // Reset any temporary values.
    for (unsigned int r = 0; r < m_rows; ++r)
    {
        for (unsigned int c = 0; c < m_cols; ++c)
        {
            if (ZIlabel[c * m_rows + r] == 9)
            {
                ZIlabel[c * m_rows + r] = 2;
            }
        }
    }

    if (!m_dir.empty())
    {
        std::string fname = FileUtils::toAbsolutePath("ziextend.tif", m_dir);
        MatrixXd ZIextend = Map<MatrixXd>(ZIlabel.data(), m_rows, m_cols);
        writeMatrix(ZIextend, fname, "GTiff", m_dh, m_bounds, m_srs);
    }
}

void Shr3dFilter::classifyGround(PointViewPtr view,
                                 std::vector<double>& ZIlabel,
                                 std::vector<double>& ZIdtm, int dh_bins,
                                 unsigned int dz_short)
{
    // Fill DTM voids (shouldn't they already be filled?)
    // Allocate void image

    int maxCount = 10000 / (m_dh * m_dh);
    // for each of five iterations
    labelObjectBoundaries(ZIdtm, ZIlabel, dh_bins, dz_short);
    extendObjectBoundaries(ZIdtm, ZIlabel, dh_bins, dz_short);

    // // "While many authors use a single value for the elevation threshold, we
    // // suggest that a second parameter be used to increase the threshold on
    // // steep slopes, transforming the threshold to a slope-dependent value.
    // The
    // // total permissible distance is then equal to a fixed elevation
    // threshold
    // // plus the scaling value multiplied by the slope of the DEM at each
    // LIDAR
    // // point. The rationale behind this approach is that small horizontal and
    // // vertical displacements yield larger errors on steep slopes, and as a
    // // result the BE/OBJ threshold distance should be more permissive at
    // these
    // // points."
    // MatrixXd gsurfs(m_rows, m_cols);
    // MatrixXd thresh(m_rows, m_cols);
    // {
    //     MatrixXd ZIproM = Map<MatrixXd>(ZIpro.data(), m_rows, m_cols);
    //     MatrixXd scaled = ZIproM / m_dh;
    //
    //     MatrixXd gx = gradX(scaled);
    //     MatrixXd gy = gradY(scaled);
    //     gsurfs = (gx.cwiseProduct(gx) + gy.cwiseProduct(gy)).cwiseSqrt();
    //     std::vector<double> gsurfsV(gsurfs.data(),
    //                                 gsurfs.data() + gsurfs.size());
    //     std::vector<double> gsurfs_fillV = knnfill(view, gsurfsV);
    //     gsurfs = Map<MatrixXd>(gsurfs_fillV.data(), m_rows, m_cols);
    //     thresh = (m_threshold + m_scalar * gsurfs.array()).matrix();
    //
    //     if (!m_dir.empty())
    //     {
    //         std::string fname = FileUtils::toAbsolutePath("gx.tif", m_dir);
    //         writeMatrix(gx, fname, "GTiff", m_dh, m_bounds, m_srs);
    //
    //         fname = FileUtils::toAbsolutePath("gy.tif", m_dir);
    //         writeMatrix(gy, fname, "GTiff", m_dh, m_bounds, m_srs);
    //
    //         fname = FileUtils::toAbsolutePath("gsurfs.tif", m_dir);
    //         writeMatrix(gsurfs, fname, "GTiff", m_dh, m_bounds, m_srs);
    //
    //         fname = FileUtils::toAbsolutePath("gsurfs_fill.tif", m_dir);
    //         MatrixXd gsurfs_fill =
    //             Map<MatrixXd>(gsurfs_fillV.data(), m_rows, m_cols);
    //         writeMatrix(gsurfs_fill, fname, "GTiff", m_dh, m_bounds, m_srs);
    //
    //         fname = FileUtils::toAbsolutePath("thresh.tif", m_dir);
    //         writeMatrix(thresh, fname, "GTiff", m_dh, m_bounds, m_srs);
    //     }
    // }
    //
    // for (PointId i = 0; i < view->size(); ++i)
    // {
    //     double x = view->getFieldAs<double>(Id::X, i);
    //     double y = view->getFieldAs<double>(Id::Y, i);
    //     double z = view->getFieldAs<double>(Id::Z, i);
    //
    //     size_t c = static_cast<size_t>(std::floor(x - m_bounds.minx) / m_dh);
    //     size_t r = static_cast<size_t>(std::floor(y - m_bounds.miny) / m_dh);
    //
    //     // TODO(chambbj): We don't quite do this by the book and yet it seems
    //     to
    //     // work reasonably well:
    //     // "The calculation requires that both elevation and slope are
    //     // interpolated from the provisional DEM. There are any number of
    //     // interpolation techniques that might be used, and even nearest
    //     // neighbor approaches work quite well, so long as the cell size of
    //     the
    //     // DEM nearly corresponds to the resolution of the LIDAR data. Based
    //     on
    //     // these results, we find that a splined cubic interpolation provides
    //     // the best results."
    //     if (std::isnan(ZIpro[c * m_rows + r]))
    //         continue;
    //
    //     if (std::isnan(gsurfs(r, c)))
    //         continue;
    //
    //     // "The final step of the algorithm is the identification of
    //     // ground/object LIDAR points. This is accomplished by measuring the
    //     // vertical distance between each LIDAR point and the provisional
    //     // DEM, and applying a threshold calculation."
    //     if (std::fabs(ZIpro[c * m_rows + r] - z) > thresh(r, c))
    //         view->setField(Id::Classification, i, 1);
    //     else
    //         view->setField(Id::Classification, i, 2);
    // }
}

std::vector<double> Shr3dFilter::createZImax(PointViewPtr view)
{
    using namespace Dimension;

    std::vector<double> ZImaxV(m_rows * m_cols,
                               std::numeric_limits<double>::quiet_NaN());

    for (PointId i = 0; i < view->size(); ++i)
    {
        double x = view->getFieldAs<double>(Id::X, i);
        double y = view->getFieldAs<double>(Id::Y, i);
        double z = view->getFieldAs<double>(Id::Z, i);

        int c = static_cast<int>(floor(x - m_bounds.minx) / m_dh);
        int r = static_cast<int>(floor(y - m_bounds.miny) / m_dh);

        if (z > ZImaxV[c * m_rows + r] || std::isnan(ZImaxV[c * m_rows + r]))
            ZImaxV[c * m_rows + r] = z;
    }

    std::vector<double> ZImax_medV = medfilt(ZImaxV);

    std::vector<double> ZImax_fillV = knnfill(view, ZImax_medV);

    if (!m_dir.empty())
    {
        std::string fname = FileUtils::toAbsolutePath("zimax.tif", m_dir);
        MatrixXd ZImax = Map<MatrixXd>(ZImaxV.data(), m_rows, m_cols);
        writeMatrix(ZImax, fname, "GTiff", m_dh, m_bounds, m_srs);

        fname = FileUtils::toAbsolutePath("zimax_fill.tif", m_dir);
        MatrixXd ZImax_fill = Map<MatrixXd>(ZImax_fillV.data(), m_rows, m_cols);
        writeMatrix(ZImax_fill, fname, "GTiff", m_dh, m_bounds, m_srs);
    }

    return ZImax_fillV;
}

std::vector<double> Shr3dFilter::createZImin(PointViewPtr view)
{
    using namespace Dimension;

    std::vector<double> ZIminV(m_rows * m_cols,
                               std::numeric_limits<double>::quiet_NaN());

    for (PointId i = 0; i < view->size(); ++i)
    {
        double x = view->getFieldAs<double>(Id::X, i);
        double y = view->getFieldAs<double>(Id::Y, i);
        double z = view->getFieldAs<double>(Id::Z, i);

        int c = static_cast<int>(floor(x - m_bounds.minx) / m_dh);
        int r = static_cast<int>(floor(y - m_bounds.miny) / m_dh);

        if (z < ZIminV[c * m_rows + r] || std::isnan(ZIminV[c * m_rows + r]))
            ZIminV[c * m_rows + r] = z;
    }

    std::vector<double> ZImin_medV = medfilt(ZIminV);

    std::vector<double> ZImin_fillV = knnfill(view, ZImin_medV);

    if (!m_dir.empty())
    {
        std::string fname = FileUtils::toAbsolutePath("zimin.tif", m_dir);
        MatrixXd ZImin = Map<MatrixXd>(ZIminV.data(), m_rows, m_cols);
        writeMatrix(ZImin, fname, "GTiff", m_dh, m_bounds, m_srs);

        fname = FileUtils::toAbsolutePath("zimin_fill.tif", m_dir);
        MatrixXd ZImin_fill = Map<MatrixXd>(ZImin_fillV.data(), m_rows, m_cols);
        writeMatrix(ZImin_fill, fname, "GTiff", m_dh, m_bounds, m_srs);
    }

    return ZImin_fillV;
}

// Fill voids with the average of eight nearest neighbors.
// This isn't APL's filling algorithm, but may do the job for now.
std::vector<double> Shr3dFilter::knnfill(PointViewPtr view,
                                         std::vector<double> const& cz)
{
    // Create a temporary PointView that encodes our raster values so that we
    // can construct a 2D KDIndex and perform nearest neighbor searches.
    PointViewPtr temp = view->makeNew();
    PointId i(0);
    for (int c = 0; c < m_cols; ++c)
    {
        for (int r = 0; r < m_rows; ++r)
        {
            if (std::isnan(cz[c * m_rows + r]))
                continue;

            temp->setField(Id::X, i, m_bounds.minx + (c + 0.5) * m_dh);
            temp->setField(Id::Y, i, m_bounds.miny + (r + 0.5) * m_dh);
            temp->setField(Id::Z, i, cz[c * m_rows + r]);
            i++;
        }
    }

    KD2Index kdi(*temp);
    kdi.build();

    // Where the raster has voids (i.e., NaN), we search for that cell's eight
    // nearest neighbors, and fill the void with the average value of the
    // neighbors.
    std::vector<double> out = cz;
    for (int c = 0; c < m_cols; ++c)
    {
        for (int r = 0; r < m_rows; ++r)
        {
            if (!std::isnan(out[c * m_rows + r]))
                continue;

            double x = m_bounds.minx + (c + 0.5) * m_dh;
            double y = m_bounds.miny + (r + 0.5) * m_dh;
            int k = 8;
            std::vector<PointId> neighbors(k);
            std::vector<double> sqr_dists(k);
            kdi.knnSearch(x, y, k, &neighbors, &sqr_dists);

            double M1(0.0);
            size_t j(0);
            for (auto const& n : neighbors)
            {
                j++;
                double delta = temp->getFieldAs<double>(Id::Z, n) - M1;
                M1 += (delta / j);
            }

            out[c * m_rows + r] = M1;
        }
    }

    return out;
};

// Median filter.
std::vector<double> Shr3dFilter::medfilt(std::vector<double> const& cz)
{
    auto result = std::minmax_element(cz.begin(), cz.end());
    double scale = (*result.second - *result.first) / (std::pow(2.0, 8) - 1);

    auto estimate_median = [](std::vector<double> vals) {
        std::nth_element(vals.begin(), vals.begin() + vals.size() / 2,
                         vals.end());
        return *(vals.begin() + vals.size() / 2);
    };

    std::vector<double> out = cz;
    for (int c = 0; c < m_cols; ++c)
    {
        for (int r = 0; r < m_rows; ++r)
        {
            std::vector<double> block;
            for (int cc = c - 1; cc <= c + 1; ++cc)
            {
                if ((cc < 0) || (cc >= m_cols))
                    continue;
                for (int rr = r - 1; rr <= r + 1; ++rr)
                {
                    if ((rr < 0) || (rr >= m_rows))
                        continue;

                    block.push_back(out[cc * m_rows + rr]);
                }
            }
            std::cerr << block.size() << std::endl;
            // grab the 3x3 window
            double med = estimate_median(block);
            // std::cerr << med - out[c * m_rows + r] << ", " << m_agl / scale
            // << std::endl;
            if ((med - out[c * m_rows + r]) > (m_agl / scale))
            {
                std::cerr << out[c * m_rows + r] << " -> " << med << std::endl;
                out[c * m_rows + r] = med;
            }
        }
    }

    return out;
};

// Median filter.
std::vector<double> Shr3dFilter::createDSM2(std::vector<double> const& minz,
                                            std::vector<double> const& maxz)
{
    auto minresult = std::minmax_element(minz.begin(), minz.end());
    double minscale =
        (*minresult.second - *minresult.first) / (std::pow(2.0, 8) - 1);
    double t1 = 40.0 / minscale;

    std::cerr << *minresult.first << "\t" << *minresult.second << "\t" << t1
              << std::endl;

    auto maxresult = std::minmax_element(maxz.begin(), maxz.end());
    double maxscale =
        (*maxresult.second - *maxresult.first) / (std::pow(2.0, 8) - 1);
    double t2 = m_dz / maxscale;

    std::cerr << *maxresult.first << "\t" << *maxresult.second << "\t" << t2
              << std::endl;

    std::vector<double> out = maxz;
    for (int c = 0; c < m_cols; ++c)
    {
        for (int r = 0; r < m_rows; ++r)
        {
            double minval = minz[c * m_rows + r];
            if ((maxz[c * m_rows + r] - minval) < t1)
            {
                std::cerr << maxz[c * m_rows + r] - minval << " < " << t1
                          << std::endl;
                bool found = false;
                for (int cc = c - 1; cc <= c + 1; ++cc)
                {
                    if ((cc < 0) || (cc >= m_cols))
                        continue;
                    for (int rr = r - 1; rr <= r + 1; ++rr)
                    {
                        if ((rr < 0) || (rr >= m_rows))
                            continue;
                        double diff = out[cc * m_rows + rr] - minval;
                        if (diff < t2)
                        {
                            std::cerr << diff << " < " << t2 << std::endl;
                            found = true;
                        }
                    }
                }

                if (!found)
                    out[c * m_rows + r] = -9999.0;
            }
        }
    }

    if (!m_dir.empty())
    {
        std::string fname = FileUtils::toAbsolutePath("dsm2.tif", m_dir);
        MatrixXd ZIdsm2 = Map<MatrixXd>(out.data(), m_rows, m_cols);
        writeMatrix(ZIdsm2, fname, "GTiff", m_dh, m_bounds, m_srs);
    }

    return out;
};

} // namespace pdal
