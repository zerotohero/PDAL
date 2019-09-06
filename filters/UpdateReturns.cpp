/******************************************************************************
 * Copyright (c) 2019, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "UpdateReturns.hpp"

#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static StaticPluginInfo const s_info{
    "filters.updatereturns", "Update return information in presence of noise",
    "http://pdal.io/stages/filters.updatereturns.html"};

CREATE_STATIC_STAGE(UpdateReturnsFilter, s_info)

std::string UpdateReturnsFilter::getName() const
{
    return s_info.name;
}

void UpdateReturnsFilter::addArgs(ProgramArgs& args)
{
    // Probably want to eventually allow some flexibility in determining what
    // DimRange to base adjustment on. For now, stick with Classification 7.
}

void UpdateReturnsFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    if (!layout->hasDim(Dimension::Id::ReturnNumber) ||
        !layout->hasDim(Dimension::Id::NumberOfReturns))
    {
        log()->get(LogLevel::Error) << "Could not find ReturnNumber or "
                                       "NumberOfReturns. Nothing to update!\n";
    }
}

PointViewSet UpdateReturnsFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;
    if (!inView->size())
        return viewSet;

    // Scan point cloud looking for all points marked as noise. If there are
    // none, we can return. Where there are noise returns, note the GPS Time,
    // and find all other points associated with the same pulse(s).
    std::set<double> tset;
    for (PointId idx = 0; idx < inView->size(); idx++)
    {
        PointRef p = inView->point(idx);
        uint8_t c = p.getFieldAs<uint8_t>(Dimension::Id::Classification);
        if (c == 7)
        {
            double t = p.getFieldAs<double>(Dimension::Id::GpsTime);
            tset.insert(t);
        }
    }

    std::map<double, std::vector<PointId>> tidxmap;
    for (PointId idx = 0; idx < inView->size(); idx++)
    {
        // If GpsTime in tset, then stash indices in map.
        PointRef p = inView->point(idx);
        double t = p.getFieldAs<double>(Dimension::Id::GpsTime);
        auto pi = tset.find(t);
        if (pi == tset.end())
            continue;
        tidxmap[t].push_back(idx);
    }

    // For each pulse, update the return information.
    for (auto const& t : tidxmap)
    {
        if (t.second.size() == 1)
        {
            PointRef p = inView->point(t.second[0]);
            uint8_t rn = p.getFieldAs<uint8_t>(Dimension::Id::ReturnNumber);
            uint8_t nr = p.getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns);
            uint8_t c = p.getFieldAs<uint8_t>(Dimension::Id::Classification);
            if (rn != 1 || nr != 1)
                // perhaps becuase the other returns in the pulse were
                // previously cropped/culled? should still update, ignore for
                // now.
                continue;
        }
        else
        {
            std::vector<PointId> returns(t.second.size());
            for (auto const& idx : t.second)
            {
                PointRef p = inView->point(idx);
                uint8_t rn = p.getFieldAs<uint8_t>(Dimension::Id::ReturnNumber);
                returns[rn - 1] = idx;
            }

            for (size_t i = 0; i < returns.size(); ++i)
            {
                PointId idx = returns[i];
                PointRef p = inView->point(idx);
                uint8_t rn = p.getFieldAs<uint8_t>(Dimension::Id::ReturnNumber);
                uint8_t nr =
                    p.getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns);
                uint8_t c =
                    p.getFieldAs<uint8_t>(Dimension::Id::Classification);
                if (c == 7 && rn == 1)
                {
                    // decrement rn and nr for all subsequent returns
                    for (size_t j = i + 1; j < returns.size(); ++j)
                    {
                        PointId jdx = returns[j];
                        PointRef q = inView->point(jdx);
                        uint8_t rn =
                            q.getFieldAs<uint8_t>(Dimension::Id::ReturnNumber);
                        uint8_t nr = q.getFieldAs<uint8_t>(
                            Dimension::Id::NumberOfReturns);
                        q.setField<uint8_t>(Dimension::Id::ReturnNumber,
                                            rn - 1);
                        q.setField<uint8_t>(Dimension::Id::NumberOfReturns,
                                            nr - 1);
                    }
                }
                else if (c == 7 && rn == nr)
                {
                    // decrement nr for all preceding returns
                    for (size_t j = 0; j < i; ++j)
                    {
                        PointId jdx = returns[j];
                        PointRef q = inView->point(jdx);
                        uint8_t nr = q.getFieldAs<uint8_t>(
                            Dimension::Id::NumberOfReturns);
                        q.setField<uint8_t>(Dimension::Id::NumberOfReturns,
                                            nr - 1);
                    }
                }
                else
                {
                    // decrement rn for all subsequent and nr for all returns
                    for (size_t j = i + 1; j < returns.size(); ++j)
                    {
                        PointId jdx = returns[j];
                        PointRef q = inView->point(jdx);
                        uint8_t rn =
                            q.getFieldAs<uint8_t>(Dimension::Id::ReturnNumber);
                        q.setField<uint8_t>(Dimension::Id::ReturnNumber,
                                            rn - 1);
                    }
                    for (size_t j = 0; j < returns.size(); ++j)
                    {
                        PointId jdx = returns[j];
                        PointRef q = inView->point(jdx);
                        uint8_t nr = q.getFieldAs<uint8_t>(
                            Dimension::Id::NumberOfReturns);
                        q.setField<uint8_t>(Dimension::Id::NumberOfReturns,
                                            nr - 1);
                    }
                }
            }
        }
    }

    viewSet.insert(inView);
    return viewSet;
}

} // namespace pdal
