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

#include <sofa/component/constraint/lagrangian/solver/BuiltConstraintSolver.h>
#include <sofa/core/behavior/ConstraintResolution.h>
#include <thread>
#include <future>
#include <atomic>

namespace sofa::component::constraint::lagrangian::solver
{
class SOFA_COMPONENT_CONSTRAINT_LAGRANGIAN_SOLVER_API ImprovedJacobiConstraintSolver : public BuiltConstraintSolver
{
public:
    SOFA_CLASS(ImprovedJacobiConstraintSolver, BuiltConstraintSolver);
    Data<int> d_numberOfThread;

    ImprovedJacobiConstraintSolver();

    class AsynchSubSolver
    {
    public:
        AsynchSubSolver(unsigned idBegin, unsigned idEnd,
                 unsigned dimension, SReal rho, SReal tol,
                 SReal *d, SReal *correctedD, SReal *dfree, SReal** w, SReal* force, SReal* deltaF, SReal* lastF,
                 std::vector<core::behavior::ConstraintResolution*>* constraintCorr, ImprovedJacobiConstraintSolver * solver);

        AsynchSubSolver(const AsynchSubSolver& from);
        ~AsynchSubSolver();

        AsynchSubSolver& operator =(const AsynchSubSolver& from);


        void mainLoop(std::shared_future<void> startFuture);

        void setContinueFuture(std::shared_future<bool> continueFuture);

        void startThread(std::shared_future<void> startFuture);

       private:

        unsigned m_idBegin;
        unsigned m_idEnd;
        unsigned m_dimension;
        unsigned m_bufferId;
        SReal    m_rho;
        SReal    m_tol;
        SReal*   m_d;
        SReal*   m_correctedD;
        SReal*   m_dfree;
        SReal**  m_w;
        SReal*   m_force;
        SReal*  m_deltaF;
        SReal*  m_lastF;
        std::vector<core::behavior::ConstraintResolution*>* m_constraintCorr;
        ImprovedJacobiConstraintSolver * m_solver;

        std::thread * m_thread;

        std::shared_future< std::tuple<bool, unsigned> > m_continueFuture;


    };

    /**
     * Based on paper
     * Francu, Mihai & Moldoveanu, Florica. An Improved Jacobi Solver for Particle Simulation.
     * VRPHYS 2014
     **/
    virtual void doSolve( SReal timeout = 0.0) override;

    static std::tuple<bool, SReal> iterate(unsigned idBegin, unsigned idEnd,
                 unsigned dimension, SReal rho, SReal tol, SReal beta,
                 SReal *d, SReal* correctedD, SReal* dfree, SReal** w, SReal* force, SReal* newDeltaF, SReal* newLastF,
                 const SReal* deltaF, const SReal* lastF,
                 std::vector<core::behavior::ConstraintResolution*>& constraintCorr);

    std::atomic_int m_workerCounter;
};
}