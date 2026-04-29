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
#include <sofa/simulation/TimeIntegrator.h>

#include <sofa/core/ConstraintParams.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/BehaviorUpdatePositionVisitor.h>
#include <sofa/simulation/CollisionBeginEvent.h>
#include <sofa/simulation/CollisionEndEvent.h>
#include <sofa/simulation/CollisionVisitor.h>
#include <sofa/simulation/IntegrateBeginEvent.h>
#include <sofa/simulation/IntegrateEndEvent.h>
#include <sofa/simulation/PropagateEventVisitor.h>
#include <sofa/simulation/UpdateInternalDataVisitor.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalAccumulateMatrixDeriv.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalBeginIntegrationVisitor.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalEndIntegrationVisitor.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalProjectPositionAndVelocityVisitor.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalPropagateOnlyPositionAndVelocityVisitor.h>

namespace sofa::simulation
{

TimeIntegrator::TimeIntegrator()
    : l_node(initLink("targetNode", "Link to the scene's node that will be processed by the integrator"))
{}

TimeIntegrator::~TimeIntegrator()
{}

void TimeIntegrator::init()
{
    Inherit1::init();
    if (!l_node)
        l_node.set(dynamic_cast<Node*>(this->getContext()));
}

void TimeIntegrator::behaviorUpdatePosition(const core::ExecParams* params, const SReal dt) const
{
    SCOPED_TIMER("BehaviorUpdatePositionVisitor");
    BehaviorUpdatePositionVisitor beh(params, dt);
    l_node->execute(beh);
}

void TimeIntegrator::updateInternalData(const core::ExecParams* params) const
{
    SCOPED_TIMER("UpdateInternalDataVisitor");
    l_node->execute<UpdateInternalDataVisitor>(params);
}

void TimeIntegrator::beginIntegration(const core::ExecParams* params, SReal dt) const
{
    propagateIntegrateBeginEvent(params);

    SCOPED_TIMER("beginIntegration");
    mechanicalvisitor::MechanicalBeginIntegrationVisitor beginVisitor(params, dt);
    l_node->execute(&beginVisitor);
}

void TimeIntegrator::propagateIntegrateBeginEvent(const core::ExecParams* params) const
{
    SCOPED_TIMER("propagateIntegrateBeginEvent");
    IntegrateBeginEvent evBegin;
    PropagateEventVisitor eventPropagation(params, &evBegin);
    eventPropagation.execute(l_node);
}

void TimeIntegrator::accumulateMatrixDeriv(const core::ConstraintParams cparams) const
{
    SCOPED_TIMER("accumulateMatrixDeriv");
    mechanicalvisitor::MechanicalAccumulateMatrixDeriv accumulateMatrixDeriv(&cparams, core::vec_id::write_access::constraintJacobian);
    accumulateMatrixDeriv.execute(l_node);
}

void TimeIntegrator::propagateIntegrateEndEvent(const core::ExecParams* params) const
{
    SCOPED_TIMER("propagateIntegrateEndEvent");
    IntegrateEndEvent evEnd;
    PropagateEventVisitor eventPropagation(params, &evEnd);
    eventPropagation.execute(l_node);
}

void TimeIntegrator::endIntegration(const core::ExecParams* params, const SReal dt) const
{
    {
        SCOPED_TIMER("endIntegration");
        mechanicalvisitor::MechanicalEndIntegrationVisitor endVisitor(params, dt);
        l_node->execute(&endVisitor);
    }

    propagateIntegrateEndEvent(params);
}

void TimeIntegrator::projectPositionAndVelocity(const SReal nextTime, const sofa::core::MechanicalParams& mparams) const
{
    SCOPED_TIMER("projectPositionAndVelocity");
    mechanicalvisitor::MechanicalProjectPositionAndVelocityVisitor(&mparams, nextTime,
        sofa::core::vec_id::write_access::position, sofa::core::vec_id::write_access::velocity
    ).execute(l_node);
}

void TimeIntegrator::propagateOnlyPositionAndVelocity(const SReal nextTime, const sofa::core::MechanicalParams& mparams) const
{
    SCOPED_TIMER("propagateOnlyPositionAndVelocity");
    mechanicalvisitor::MechanicalPropagateOnlyPositionAndVelocityVisitor(&mparams, nextTime,
        core::vec_id::write_access::position,
        core::vec_id::write_access::velocity).execute(l_node);
}

void TimeIntegrator::propagateCollisionBeginEvent(const core::ExecParams* params) const
{
    SCOPED_TIMER("CollisionBeginEvent");
    CollisionBeginEvent evBegin;
    PropagateEventVisitor eventPropagation(params, &evBegin);
    eventPropagation.execute(l_node);
}

void TimeIntegrator::propagateCollisionEndEvent(const core::ExecParams* params) const
{
    SCOPED_TIMER("CollisionEndEvent");
    CollisionEndEvent evEnd;
    PropagateEventVisitor eventPropagation(params, &evEnd);
    eventPropagation.execute(l_node);
}

void TimeIntegrator::collisionDetection(const core::ExecParams* params) const
{
    propagateCollisionBeginEvent(params);

    {
        SCOPED_TIMER("collision");
        CollisionVisitor act(params);
        l_node->execute(&act);
    }

    propagateCollisionEndEvent(params);
}

} // namespace sofa::simulation
