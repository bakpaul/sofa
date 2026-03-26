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
#include <sofa/simulation/LinearTimeIntegrator.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ConstraintParams.h>

#include <sofa/simulation/task/MainTaskSchedulerFactory.h>
#include <sofa/simulation/task/TaskScheduler.h>

namespace sofa::simulation
{

LinearTimeIntegrator::LinearTimeIntegrator()
: d_parallelODESolving(initData(&d_parallelODESolving, false, "parallelODESolving", "If true, solves all the ODEs in parallel"))
{
    this->addUpdateCallback("parallelODESolving", {&d_parallelODESolving},
  [this](const core::DataTracker& tracker) -> sofa::core::objectmodel::ComponentState
  {
      SOFA_UNUSED(tracker);
      if (d_parallelODESolving.getValue())
      {
          simulation::TaskScheduler* taskScheduler = simulation::MainTaskSchedulerFactory::createInRegistry();
          assert(taskScheduler);

          if (taskScheduler->getThreadCount() < 1)
          {
              taskScheduler->init(0);
              msg_info() << "Task scheduler initialized on " << taskScheduler->getThreadCount() << " threads";
          }
          else
          {
              msg_info() << "Task scheduler already initialized on " << taskScheduler->getThreadCount() << " threads";
          }
      }
      return d_componentState.getValue();
  },
{});
}

 void LinearTimeIntegrator::integrate(const core::ExecParams* params, SReal dt)
 {

     const SReal startTime = this->getContext()->getTime();
     const SReal nextTime = startTime + dt;

     sofa::core::MechanicalParams mparams(*params);
     mparams.setDt(dt);

     behaviorUpdatePosition(params, dt);
     updateInternalData(params);

     collisionDetection(params);

     beginIntegration(params, dt);
     {
         const core::ConstraintParams cparams;
         accumulateMatrixDeriv(cparams);

         solve(params, dt, d_parallelODESolving.getValue());

         projectPositionAndVelocity(nextTime, mparams);
         propagateOnlyPositionAndVelocity(nextTime, mparams);
     }
     endIntegration(params, dt);
 }






}
