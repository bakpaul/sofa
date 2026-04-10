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
#include <sofa/core/MechanicalParams.h>
#include <sofa/core/behavior/BaseInteractionForceField.h>
#include <sofa/core/behavior/OdeSolver.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/SetupIntegrationStepVisitor.h>
#include <sofa/simulation/task/MainTaskSchedulerFactory.h>
#include <sofa/simulation/task/TaskScheduler.h>

#include "sofa/core/behavior/IntegrationScheme.h"

namespace sofa::simulation
{

void SetupIntegrationStepVisitor::processSolver(simulation::Node* node, sofa::core::behavior::IntegrationScheme* s)
{
    helper::ScopedAdvancedTimer timer("Mechanical",node);
    s->setupIntegrationStep(params, dt, x, v);
}


Visitor::Result SetupIntegrationStepVisitor::processNodeTopDown(simulation::Node* node)
{
    if (! node->integrationScheme.empty())
    {
        for_each(this, node, node->integrationScheme, &SetupIntegrationStepVisitor::processSolver);

        return RESULT_PRUNE;
    }
    return RESULT_CONTINUE;
}

void SetupIntegrationStepVisitor::processNodeBottomUp(simulation::Node*)
{

}

void SetupIntegrationStepVisitor::setDt(SReal _dt)
{
    dt = _dt;
}

SReal SetupIntegrationStepVisitor::getDt() const
{
    return dt;
}


SetupIntegrationStepVisitor::SetupIntegrationStepVisitor(const sofa::core::ExecParams* params, SReal _dt,
                                                         sofa::core::MultiVecCoordId X, sofa::core::MultiVecDerivId V)
: Visitor(params)
, dt(_dt)
, x(X)
, v(V)
{}

SetupIntegrationStepVisitor::SetupIntegrationStepVisitor(const sofa::core::ExecParams* params, SReal _dt, bool free)
: Visitor(params), dt(_dt)
{
    if(free)
    {
        x = sofa::core::vec_id::write_access::freePosition;
        v = sofa::core::vec_id::write_access::freeVelocity;
    }
    else
    {
        x = sofa::core::vec_id::write_access::position;
        v = sofa::core::vec_id::write_access::velocity;
    }
}
} // namespace sofa::simulation

