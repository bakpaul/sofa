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

#include <sofa/simulation/config.h>
#include <sofa/core/behavior/BaseTimeIntegrator.h>
#include <sofa/simulation/Node.h>

namespace sofa::simulation
{

/**
 *  \brief Component responsible for time integration, i.e. advancing the simulation
 *  from time t to t+dt including collision detection and mechanical solving.
 *
 *  Derived classes implement the specific integration strategy (e.g. linear, nonlinear).
 */
class SOFA_SIMULATION_CORE_API TimeIntegrator : public sofa::core::behavior::BaseTimeIntegrator
{

public:
    SOFA_ABSTRACT_CLASS(TimeIntegrator, sofa::core::behavior::BaseTimeIntegrator);

protected:
    TimeIntegrator();
    ~TimeIntegrator() override;

public:
    void init() override;

    /// Link to the scene node this integrator will process (defaults to the node it is in).
    SingleLink<TimeIntegrator, simulation::Node, BaseLink::FLAG_STOREPATH> l_node;

    /// Perform the mechanical solve step (without collision and bookkeeping).
    virtual void solve(const sofa::core::ExecParams* params, SReal dt, bool parallelODESolving) const = 0;

    void behaviorUpdatePosition(const sofa::core::ExecParams* params, SReal dt) const;
    void updateInternalData(const sofa::core::ExecParams* params) const;
    void beginIntegration(const sofa::core::ExecParams* params, SReal dt) const;
    void propagateIntegrateBeginEvent(const sofa::core::ExecParams* params) const;
    void accumulateMatrixDeriv(sofa::core::ConstraintParams cparams) const;
    void propagateIntegrateEndEvent(const sofa::core::ExecParams* params) const;
    void endIntegration(const sofa::core::ExecParams* params, SReal dt) const;
    void projectPositionAndVelocity(SReal nextTime, const sofa::core::MechanicalParams& mparams) const;
    void propagateOnlyPositionAndVelocity(SReal nextTime, const sofa::core::MechanicalParams& mparams) const;
    void propagateCollisionBeginEvent(const sofa::core::ExecParams* params) const;
    void propagateCollisionEndEvent(const sofa::core::ExecParams* params) const;
    void collisionDetection(const sofa::core::ExecParams* params) const;
};

} // namespace sofa::simulation
