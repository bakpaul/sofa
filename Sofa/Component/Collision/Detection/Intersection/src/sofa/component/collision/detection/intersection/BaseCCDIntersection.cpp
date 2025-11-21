/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/component/collision/detection/intersection/BaseCCDIntersection.h>

namespace sofa::component::collision::detection::intersection
{

using namespace sofa::component::collision::geometry;

BaseCCDIntersection::BaseCCDIntersection()
    : d_contactDistance(initData(&d_contactDistance, 0.5_sreal, "contactDistance", "Distance below which a contact is created"))
{
    d_contactDistance.setRequired(true);
}


bool BaseCCDIntersection::testIntersection(Cube& cube1, Cube& cube2, const core::collision::Intersection* currentIntersection)
{
    const std::array<type::Vec3, 8> points{cube1.minVect(),           cube2.minVect(),           cube1.maxVect(),           cube2.maxVect(),
                                           cube1.continuousMinVect(), cube2.continuousMinVect(), cube1.continuousMaxVect(), cube2.continuousMaxVect()};

    const auto alarmDist = currentIntersection->getAlarmDistance() + cube1.getContactDistance() + cube2.getContactDistance();

    const type::Vec3 Cube1MotionEdgeA = (points[2] - points[0]) /2;
    const type::Vec3 Cube1MotionEdge = (points[6] - points[4])/2 - Cube1MotionEdgeA;
    const SReal Cube1MotionEdgeNorm = Cube1MotionEdge.norm();
    const type::Vec3 Cube1MotionVec = Cube1MotionEdge.normalized();

    SReal maxNormal1DistToNormal = 0;
    SReal maxTangent1DistToNormal = 0;

    //Lighter version would be to use max of dist from 4 points to the corresponding bary, the cylinder will be overestimated but the computation will be way faster

    for(const auto id : {0, 2, 4, 6})
    {
        SReal currTangentDistToNormal = dot(points[id] - Cube1MotionEdgeA,Cube1MotionVec);
        SReal currNormalDistToNormal = (points[id] - Cube1MotionEdgeA - Cube1MotionVec * currTangentDistToNormal).norm();
        currTangentDistToNormal = (currTangentDistToNormal < Cube1MotionEdgeNorm && currTangentDistToNormal > 0) ? 0 : currTangentDistToNormal - Cube1MotionEdgeNorm;
        maxNormal1DistToNormal = currNormalDistToNormal > maxNormal1DistToNormal ? currNormalDistToNormal : maxNormal1DistToNormal;
        maxTangent1DistToNormal = currTangentDistToNormal > maxTangent1DistToNormal ? currTangentDistToNormal : maxTangent1DistToNormal;
    }
    const auto CorrectedCube1MotionEdge = Cube1MotionEdge + Cube1MotionVec * (maxTangent1DistToNormal + alarmDist);
    const auto CorrectedCube1MotionEdgeA = Cube1MotionEdgeA - Cube1MotionVec * (maxTangent1DistToNormal + alarmDist);

    const type::Vec3 Cube2MotionEdgeA = (points[2] - points[0]) /2;
    const type::Vec3 Cube2MotionEdge = (points[6] - points[4])/2 - Cube2MotionEdgeA;
    const SReal Cube2MotionEdgeNorm = Cube2MotionEdge.norm();
    const type::Vec3 Cube2MotionVec = Cube2MotionEdge.normalized();
    
    SReal maxNormal2DistToNormal = 0;
    SReal maxTangent2DistToNormal = 0;

    for(const auto id : {0, 2, 4, 6})
    {
        SReal currTangentDistToNormal = dot(points[id] - Cube2MotionEdgeA,Cube2MotionVec);
        SReal currNormalDistToNormal = (points[id] - Cube2MotionEdgeA - Cube2MotionVec * currTangentDistToNormal).norm();
        currTangentDistToNormal = (currTangentDistToNormal < Cube2MotionEdgeNorm && currTangentDistToNormal > 0) ? 0 : currTangentDistToNormal - Cube2MotionEdgeNorm;
        maxNormal2DistToNormal = currNormalDistToNormal > maxNormal2DistToNormal ? currNormalDistToNormal : maxNormal2DistToNormal;
        maxTangent2DistToNormal = currTangentDistToNormal > maxTangent2DistToNormal ? currTangentDistToNormal : maxTangent2DistToNormal;
    }
    const auto CorrectedCube2MotionEdge = Cube2MotionEdge + Cube2MotionVec * (maxTangent2DistToNormal + alarmDist);
    const auto CorrectedCube2MotionEdgeA = Cube2MotionEdgeA - Cube2MotionVec * (maxTangent2DistToNormal + alarmDist);

    const SReal EdgeDist = maxNormal1DistToNormal + maxNormal2DistToNormal + alarmDist;

    type::Vec2 baryCoords(type::NOINIT);
    sofa::geometry::Edge::closestPointWithEdge(CorrectedCube1MotionEdgeA ,
                                               CorrectedCube1MotionEdgeA + CorrectedCube1MotionEdge,
                                               CorrectedCube2MotionEdgeA ,
                                               CorrectedCube2MotionEdgeA + CorrectedCube2MotionEdge, baryCoords);

    return (   ((1-baryCoords[0]) * CorrectedCube1MotionEdgeA + CorrectedCube1MotionEdgeA + CorrectedCube1MotionEdge)
             - ((1-baryCoords[1]) * CorrectedCube2MotionEdgeA + CorrectedCube2MotionEdgeA + CorrectedCube2MotionEdge)).norm() < EdgeDist;

}

int BaseCCDIntersection::computeIntersection(Cube& cube1, Cube& cube2, OutputVector* contacts, const core::collision::Intersection* currentIntersection)
{
    SOFA_UNUSED(cube1);
    SOFA_UNUSED(cube2);
    SOFA_UNUSED(contacts);
    SOFA_UNUSED(currentIntersection);

    return 0;
}

} // namespace sofa::component::collision::detection::intersection
