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

#include "sofa/type/vector_algebra.h"

namespace sofa::component::collision::detection::intersection
{

using namespace sofa::component::collision::geometry;

double projectPointToEdge(const type::Vec3& point, const type::Vec3& edgeA, const type::Vec3& edgeB )
{
    const auto v = edgeB - edgeA;
    const double vnorm = v.norm();
    return type::dot(v,point - edgeA)/(vnorm * vnorm);
}


BaseCCDIntersection::BaseCCDIntersection()
    : d_contactDistance(initData(&d_contactDistance, 0.5_sreal, "contactDistance", "Distance below which a contact is created"))
{
    d_contactDistance.setRequired(true);
}


bool BaseCCDIntersection::testIntersection(Cube& cube1, Cube& cube2, const core::collision::Intersection* currentIntersection)
{
    const auto alarmDist = currentIntersection->getAlarmDistance() + cube1.getContactDistance() + cube2.getContactDistance();

    const type::Vec3 Cube1MotionEdgeA = (cube1.maxVect()           + cube1.minVect())/2;
    const type::Vec3 Cube1MotionEdgeB = (cube1.continuousMaxVect() + cube1.continuousMinVect())/2;
    const type::Vec3 Cube1MotionEdge  = Cube1MotionEdgeB - Cube1MotionEdgeA;
    const SReal Cube1MaxDist          = fmax((cube1.maxVect() - cube1.minVect()).norm()/2,(cube1.continuousMaxVect() - cube1.continuousMinVect()).norm()/2 );

    const type::Vec3 Cube2MotionEdgeA = (cube2.maxVect()           + cube2.minVect())/2;
    const type::Vec3 Cube2MotionEdgeB = (cube2.continuousMaxVect() + cube2.continuousMinVect())/2;
    const type::Vec3 Cube2MotionEdge  = Cube2MotionEdgeB - Cube2MotionEdgeA;
    const SReal Cube2MaxDist          = fmax((cube2.maxVect() - cube2.minVect()).norm()/2,(cube2.continuousMaxVect() - cube2.continuousMinVect()).norm()/2 );


    const SReal EdgeDist = Cube1MaxDist + Cube2MaxDist + alarmDist;

    type::Vec2 baryCoords(type::NOINIT);

    double v1v2 = dot(Cube2MotionEdge, Cube1MotionEdge);
    type::Vec3 A1A2 = Cube1MotionEdgeA - Cube2MotionEdgeA;
    double det =  1 - v1v2 * v1v2;
    std::array<double,2> b = {-dot(Cube1MotionEdge, A1A2), dot(Cube2MotionEdge, A1A2) };
    baryCoords[0] = (dot(Cube1MotionEdge,Cube1MotionEdge) * b[0] + v1v2 * b[1])/det;
    baryCoords[1] = (v1v2 * b[0]                                 + dot(Cube2MotionEdge,Cube2MotionEdge) * b[1])/det;

    bool projectFirst = (baryCoords[0] < 0) || (baryCoords[0] > 1);
    bool projectSecond = (baryCoords[1] < 0) || (baryCoords[1] > 1);

    if (baryCoords[0] < 0) baryCoords[0] = 0;
    if (baryCoords[0] > 1) baryCoords[0] = 1;
    if (baryCoords[1] < 0) baryCoords[1] = 0;
    if (baryCoords[1] > 1) baryCoords[1] = 1;

    if (projectFirst || projectSecond)
    {

        const auto oldFirstPoint = Cube1MotionEdgeA + Cube1MotionEdge * baryCoords[0];
        const auto oldSecondPoint = Cube2MotionEdgeA + Cube2MotionEdge * baryCoords[1];
        if (projectFirst && projectSecond)
        {
            double newFirstBary = projectPointToEdge(oldSecondPoint, Cube1MotionEdgeA, Cube1MotionEdgeB);
            if ( newFirstBary > 1 ) newFirstBary = 1;
            if ( newFirstBary < 1 ) newFirstBary = 0;

            double newSecondBary = projectPointToEdge(oldFirstPoint, Cube2MotionEdgeA, Cube2MotionEdgeB);
            if ( newSecondBary > 1 ) newSecondBary = 1;
            if ( newSecondBary < 1 ) newSecondBary = 0;

            if (  ((1-newSecondBary) * Cube2MotionEdgeA + newSecondBary * Cube2MotionEdgeB - oldFirstPoint ).norm()
                < ((1-newFirstBary ) * Cube1MotionEdgeA + newFirstBary  * Cube1MotionEdgeB - oldSecondPoint ).norm())
            {
                baryCoords[1] = newSecondBary;
            }
            else
            {
                baryCoords[0] = newFirstBary;
            }
        }
        else if (projectFirst)
        {
            double newSecondBary = projectPointToEdge(oldFirstPoint, Cube2MotionEdgeA, Cube2MotionEdgeB);
            if ( newSecondBary > 1 ) newSecondBary = 1;
            if ( newSecondBary < 1 ) newSecondBary = 0;
            if (  ((1-newSecondBary) * Cube2MotionEdgeA + newSecondBary * Cube2MotionEdgeB - oldFirstPoint ).norm()
                < (oldFirstPoint - oldSecondPoint).norm())
            {
                baryCoords[1] = newSecondBary;
            }
        }
        else if (projectSecond)
        {
            double newFirstBary = projectPointToEdge(oldSecondPoint, Cube1MotionEdgeA, Cube1MotionEdgeB);
            if ( newFirstBary > 1 ) newFirstBary = 1;
            if ( newFirstBary < 1 ) newFirstBary = 0;
            if (  ((1-newFirstBary) * Cube1MotionEdgeA + newFirstBary * Cube1MotionEdgeB - oldSecondPoint ).norm()
                < (oldFirstPoint - oldSecondPoint).norm())
            {
                baryCoords[0] = newFirstBary;
            }
        }
    }

    return (   (Cube1MotionEdgeA + baryCoords[0] * Cube1MotionEdge )
             - (Cube2MotionEdgeA + baryCoords[1] * Cube2MotionEdge )).norm() < EdgeDist;

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
