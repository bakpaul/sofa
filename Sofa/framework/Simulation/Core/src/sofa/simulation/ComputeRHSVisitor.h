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
#pragma once

#include <sofa/simulation/IntegrationSchemeBaseVisitor.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/simulation/task/CpuTask.h>

#include <list>

namespace sofa::simulation
{

class ComputeRHSVisitorTask;

/** Used by the animation loop: send the solve signal to the others solvers
This visitor is able to run the solvers sequentially or concurrently.
 */
class SOFA_SIMULATION_CORE_API ComputeRHSVisitor : public IntegrationSchemeBaseVisitor
{
public:

    ComputeRHSVisitor(const sofa::core::ExecParams* params,
                     bool parallelSolve,
                     SReal dt,
                     unsigned iteration);

    void processSolver(simulation::Node* node, sofa::core::behavior::IntegrationScheme* b) override;
    Result processNodeTopDown(simulation::Node* node) override;
    void fwdInteractionForceField(Node* node, core::behavior::BaseInteractionForceField* forceField);



protected:
    SReal m_dt;
    unsigned m_iteration;
};

} // namespace sofa
