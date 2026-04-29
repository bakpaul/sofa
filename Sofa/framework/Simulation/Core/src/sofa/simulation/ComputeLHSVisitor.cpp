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
#include <sofa/simulation/ComputeLHSVisitor.h>
#include <sofa/core/behavior/IntegrationScheme.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/Node.h>

namespace sofa::simulation
{

ComputeLHSVisitor::ComputeLHSVisitor(const sofa::core::ExecParams* params,
                     bool parallelSolve,
                     unsigned iteration)
    : IntegrationSchemeBaseVisitor(params, parallelSolve)
    , m_iteration(iteration)
{}

void ComputeLHSVisitor::processSolver(simulation::Node* node, sofa::core::behavior::IntegrationScheme* s)
{
    helper::ScopedAdvancedTimer timer("Mechanical", node);
    s->computeLHS(m_iteration);
}

} // namespace sofa::simulation
