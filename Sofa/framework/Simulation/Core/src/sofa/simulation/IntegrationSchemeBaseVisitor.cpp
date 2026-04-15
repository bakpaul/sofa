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
#include <sofa/simulation/IntegrationSchemeBaseVisitor.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/simulation/Node.h>
#include <sofa/core/behavior/OdeSolver.h>
#include <sofa/simulation/task/TaskScheduler.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/task/MainTaskSchedulerFactory.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/core/behavior/BaseInteractionForceField.h>

namespace sofa::simulation
{

Visitor::Result IntegrationSchemeBaseVisitor::processNodeTopDown(simulation::Node* node)
{
    if (! node->solver.empty())
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
    return RESULT_CONTINUE;
}

void IntegrationSchemeBaseVisitor::processNodeBottomUp(simulation::Node*)
{
    // only in case of parallel solving:
    // processNodeBottomUp is called after all processNodeTopDown calls are done,
    // i.e when all parallel tasks have been created and started.
    // It is time to wait them to finish

    if (!m_tasks.empty())
    {
        auto* taskScheduler = sofa::simulation::MainTaskSchedulerFactory::createInRegistry();
        assert(taskScheduler != nullptr);
        SCOPED_TIMER_VARNAME(parallelSolveTimer, "waitParallelTasks");
        taskScheduler->workUntilDone(&m_status);
    }
    m_tasks.clear();
}


IntegrationSchemeBaseVisitor::IntegrationSchemeBaseVisitor(const sofa::core::ExecParams* params, bool _parallelSolve)

        : Visitor(params)
        , m_parallelSolve(_parallelSolve)
{
    if (m_parallelSolve)
    {
        initializeTaskScheduler();
    }
}


void IntegrationSchemeBaseVisitor::sequentialSolve(simulation::Node* node)
{
    for_each(this, node, node->integrationScheme, &IntegrationSchemeBaseVisitor::processSolver);
}

void IntegrationSchemeBaseVisitor::parallelSolve(simulation::Node* node)
{
    auto* taskScheduler = sofa::simulation::MainTaskSchedulerFactory::createInRegistry();
    assert(taskScheduler != nullptr);

    const auto task = std::bind(&IntegrationSchemeBaseVisitor::processSolver, this, node, std::placeholders::_1);

    for (auto* solver : node->integrationScheme)
    {
        m_tasks.emplace_back(&m_status, solver, task);
        taskScheduler->addTask(&m_tasks.back());
    }
}

void IntegrationSchemeBaseVisitor::initializeTaskScheduler()
{
    auto* taskScheduler = sofa::simulation::MainTaskSchedulerFactory::createInRegistry();
    assert(taskScheduler != nullptr);
    if (taskScheduler->getThreadCount() < 1)
    {
        taskScheduler->init(0);
    }
}

sofa::simulation::Task::MemoryAlloc IntegrationSchemeBaseVisitorTask::run()
{
    m_task(m_solver);
    return Task::Stack;
}

} // namespace sofa::simulation

