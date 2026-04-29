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
#include <sofa/simulation/ComputeRHSVisitor.h>
#include <sofa/core/behavior/BaseInteractionForceField.h>
#include <sofa/core/behavior/IntegrationScheme.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/Node.h>

namespace sofa::simulation
{

ComputeRHSVisitor::ComputeRHSVisitor(const sofa::core::ExecParams* params,
                     bool parallelSolve,
                     SReal dt,
                     unsigned iteration)
    : IntegrationSchemeBaseVisitor(params, parallelSolve)
    , m_dt(dt)
    , m_iteration(iteration)
{}

void ComputeRHSVisitor::processSolver(simulation::Node* node, sofa::core::behavior::IntegrationScheme* s)
{
    helper::ScopedAdvancedTimer timer("Mechanical", node);
    s->computeRHS(m_iteration);
}

void ComputeRHSVisitor::fwdInteractionForceField(Node* node, core::behavior::BaseInteractionForceField* forceField)
{
    SOFA_UNUSED(node);

    const core::MultiVecDerivId ffId = core::vec_id::write_access::externalForce;
    core::MechanicalParams mparams;
    mparams.setDt(m_dt);
    forceField->addForce(&mparams, ffId);
}

Visitor::Result ComputeRHSVisitor::processNodeTopDown(simulation::Node* node)
{
    if (!node->integrationScheme.empty())
    {
        if (m_parallelSolve)
        {
            parallelSolve(node);
        }
        else
        {
            sequentialSolve(node);
        }
        return RESULT_PRUNE;
    }

    if (m_computeForceIsolatedInteractionForceFields)
    {
        for_each(this, node, node->interactionForceField, &ComputeRHSVisitor::fwdInteractionForceField);
    }
    return RESULT_CONTINUE;
}

} // namespace sofa::simulation
