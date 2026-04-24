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
#include <sofa/simulation/NonLinearTimeIntegrator.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ConstraintParams.h>

#include <sofa/simulation/task/MainTaskSchedulerFactory.h>
#include <sofa/simulation/task/TaskScheduler.h>



#include <sofa/simulation//ComputeLHSVisitor.h>
#include <sofa/simulation//ComputeRHSVisitor.h>
#include <sofa/simulation//SetupIntegrationStepVisitor.h>
#include <sofa/simulation//SolveLinearSystemVisitor.h>
#include <sofa/simulation//SquaredNormRHSVisitor.h>
#include <sofa/simulation//UpdateVelAndPosVisitor.h>

#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::simulation
{

NonLinearTimeIntegrator::NonLinearTimeIntegrator()
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

 void NonLinearTimeIntegrator::integrate(const core::ExecParams* params, SReal dt)
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



void NonLinearTimeIntegrator::solve(const sofa::core::ExecParams* params, SReal dt, bool parallelODESolving) const
{
    constexpr bool usefreeVecIds = false;
    constexpr bool computeForceIsolatedInteractionForceFields = true;
    SCOPED_TIMER("solve");

    simulation::SetupIntegrationStepVisitor setupIntegration(params, dt, usefreeVecIds);
    setupIntegration.execute(l_node);

    simulation::ComputeLHSVisitor ComputeLHSVisitor(params, parallelODESolving, 0);
    ComputeLHSVisitor.execute(l_node);

    simulation::ComputeRHSVisitor ComputeRHSVisitor(params, parallelODESolving, dt, 0);
    ComputeRHSVisitor.execute(l_node);

    simulation::SolveLinearSystemVisitor SolveLinearSystemVisitor(params, parallelODESolving);
    SolveLinearSystemVisitor.execute(l_node);

    simulation::UpdateVelAndPosVisitor UpdateVelAndPosVisitor(params, parallelODESolving, 0, 1.0);
    UpdateVelAndPosVisitor.execute(l_node);
}


void registerNonLinearTimeIntegrator(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Time integrator, created by default when the user does not define one in the scene. This loop first computes the collision detection and then solves the physics by linearizing it once.")
        .add<NonLinearTimeIntegrator>());
}


}
