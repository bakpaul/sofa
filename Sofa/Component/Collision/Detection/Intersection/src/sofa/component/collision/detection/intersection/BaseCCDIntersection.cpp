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
    sofa::geometry::Edge::closestPointWithEdge(Cube1MotionEdgeA,
                                               Cube1MotionEdgeB,
                                               Cube2MotionEdgeA,
                                               Cube2MotionEdgeB, baryCoords);

    std::cout<<"Cube1 pose diag : "<<(cube1.maxVect() - cube1.minVect()).norm()/2<<std::endl;
    std::cout<<"Cube1 free diag : "<<(cube1.continuousMaxVect() - cube1.continuousMinVect()).norm()/2<<std::endl;
    std::cout<<"Cube1MaxDist : "<<Cube1MaxDist<<std::endl;
    std::cout<<"Cube2MaxDist : "<<Cube2MaxDist<<std::endl;
    std::cout<<"CurrentDist : "<<(   (Cube1MotionEdgeA + baryCoords[0] * Cube1MotionEdge )
                                      - (Cube2MotionEdgeA + baryCoords[1] * Cube2MotionEdge )).norm()<<std::endl;


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
