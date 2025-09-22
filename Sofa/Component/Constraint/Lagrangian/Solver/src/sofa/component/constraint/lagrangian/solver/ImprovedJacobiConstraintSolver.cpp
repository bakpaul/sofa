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

#include <sofa/component/constraint/lagrangian/solver/ImprovedJacobiConstraintSolver.h>
#include <sofa/component/constraint/lagrangian/solver/GenericConstraintSolver.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/core/ObjectFactory.h>
#include <Eigen/Eigenvalues>

namespace sofa::component::constraint::lagrangian::solver
{

ImprovedJacobiConstraintSolver::AsynchSubSolver::AsynchSubSolver(unsigned idBegin, unsigned idEnd,
                 unsigned dimension, SReal rho, SReal tol,
                 SReal *d, SReal *correctedD, SReal *dfree, SReal** w, SReal* force, SReal* deltaF, SReal* lastF,
                 std::vector<core::behavior::ConstraintResolution*>* constraintCorr, ImprovedJacobiConstraintSolver * solver)
: m_idBegin(idBegin)
, m_idEnd(idEnd)
, m_dimension(dimension)
, m_rho(rho)
, m_tol(tol)
, m_d(d)
, m_correctedD(correctedD)
, m_dfree(dfree)
, m_w(w)
, m_force(force)
, m_deltaF(deltaF)
, m_lastF(lastF)
, m_constraintCorr(constraintCorr)
, m_solver(solver)
, m_thread(nullptr)
, m_allVerified(false)
, m_currError(0.0)
{

}

ImprovedJacobiConstraintSolver::AsynchSubSolver::AsynchSubSolver(const AsynchSubSolver & from)
: m_idBegin(from.m_idBegin)
, m_idEnd(from.m_idEnd)
, m_dimension(from.m_dimension)
, m_rho(from.m_rho)
, m_tol(from.m_tol)
, m_d(from.m_d)
, m_correctedD(from.m_correctedD)
, m_dfree(from.m_dfree)
, m_w(from.m_w)
, m_force(from.m_force)
, m_deltaF(from.m_deltaF)
, m_lastF(from.m_lastF)
, m_constraintCorr(from.m_constraintCorr)
, m_solver(from.m_solver)
, m_thread(nullptr)
, m_allVerified(from.m_allVerified)
, m_currError(from.m_currError)
{}

ImprovedJacobiConstraintSolver::AsynchSubSolver & ImprovedJacobiConstraintSolver::AsynchSubSolver::operator=(const AsynchSubSolver & from)
{
    m_idBegin = from.m_idBegin;
    m_idEnd = from.m_idEnd;
    m_dimension = from.m_dimension;
    m_rho = from.m_rho;
    m_tol = from.m_tol;
    m_d = from.m_d;
    m_correctedD = from.m_correctedD;
    m_dfree = from.m_dfree;
    m_w = from.m_w;
    m_force = from.m_force;
    m_deltaF = from.m_deltaF;
    m_lastF = from.m_lastF;
    m_constraintCorr = from.m_constraintCorr;
    m_solver = from.m_solver;
    m_thread = from.m_thread;
    m_allVerified = from.m_allVerified;
    m_currError = from.m_currError;
}

ImprovedJacobiConstraintSolver::AsynchSubSolver::~AsynchSubSolver()
{
    if(!m_thread)
        return;

    if (m_thread->joinable())
        m_thread->detach();

}

void ImprovedJacobiConstraintSolver::AsynchSubSolver::startThread()
{
    this->m_thread = new std::thread(std::bind(&ImprovedJacobiConstraintSolver::AsynchSubSolver::mainLoop, this));
}



void ImprovedJacobiConstraintSolver::AsynchSubSolver::mainLoop()
{
    bool iterate;
    SReal beta;
    std::atomic_fetch_add(&m_solver->m_workerCounter,1 );

    while(true)
    {
        //Wait for synchronization point for main thread, this allows for error gathering and future change
        std::tie( iterate, beta) = m_futures[m_solver->m_bufferNumber.load()].get();
        if( ! iterate)
            break;

        //The two following ID are either 1 or 0
        const int currBuffer = m_solver->m_bufferNumber.load();
        const int nextBuffer = (currBuffer+1)%2;

        //This allows for synchronization with main thread when starting to accumulate the error,
        std::lock_guard<std::mutex> lock(m_accessMutex);
        std::tie(m_allVerified, m_currError) = ImprovedJacobiConstraintSolver::iterate(m_idBegin, m_idEnd, m_dimension,
                                                m_rho, m_tol, beta, m_d, m_correctedD, m_dfree, m_w, m_force, m_lastF,
                                                m_deltaF + m_dimension*(currBuffer), m_deltaF + m_dimension*(nextBuffer),
                                                *m_constraintCorr);

    }

}


ImprovedJacobiConstraintSolver::ImprovedJacobiConstraintSolver()
    : BuiltConstraintSolver()
    , d_maximumNumberOfThread(initData(&d_maximumNumberOfThread, 1, "maximumNumberOfThread", "Number of thread to execute solver concurrently"))
    , d_minimumNumberOfLinePerThread(initData(&d_minimumNumberOfLinePerThread, 1, "minimumNumberOfLinePerThread", "Number of thread to execute solver concurrently"))
    , d_useSpectralCorrection(initData(&d_useSpectralCorrection,false,"useSpectralCorrection","If set to true, the solution found after each iteration will be multiplied by spectralCorrectionFactor*2/spr(W), with spr() denoting the spectral radius."))
    , d_spectralCorrectionFactor(initData(&d_spectralCorrectionFactor,1.0,"spectralCorrectionFactor","Factor used to modulate the spectral correction"))
    , d_useConjugateResidue(initData(&d_useConjugateResidue,false,"useConjugateResidue","If set to true, the solution found after each iteration will be corrected along the solution direction using `\\lambda^{i+1} -= beta^{i} * (\\lambda^{i} - \\lambda^{i-1})` with beta following the formula beta^{i} = min(1, (i/maxIterations)^{conjugateResidueSpeedFactor}) "))
    , d_conjugateResidueSpeedFactor(initData(&d_conjugateResidueSpeedFactor,10.0,"conjugateResidueSpeedFactor","FActor used to modulate the speed in which beta used in the conjugate residue part reaches 1.0. The higher the value, the slower the reach. "))
{

}

void ImprovedJacobiConstraintSolver::doSolve( SReal timeout)
{
    SCOPED_TIMER_VARNAME(gaussSeidelTimer, "ConstraintsGaussSeidel");


    const int dimension = current_cp->getDimension();

    if(!dimension)
    {
        current_cp->currentError = 0.0;
        current_cp->currentIterations = 0;
        return;
    }

    SReal *dfree = current_cp->getDfree();
    SReal *force = current_cp->getF();
    SReal **w = current_cp->getW();
    SReal tol = current_cp->tolerance;
    SReal *d = current_cp->_d.ptr();

    std::copy_n(dfree, dimension, d);

    for(unsigned i=0; i< dimension; ++i)
    {
        force[i] = 0.0;
    }

    // The size of this buffer depend on the number of thread that are used.
    // If only one thread is used then the vector is of size dimension
    // Else it is of twice the dimensions, to allow the use of a switching buffer mechanism
    std::vector<SReal> deltaF;

    std::vector<SReal> lastF;
    lastF.resize(current_cp->getDimension(), 0.0);

    std::vector<SReal> correctedD;
    correctedD.resize(current_cp->getDimension(), 0.0);


    SReal error=0.0;
    bool convergence = false;
    if(current_cp->scaleTolerance && !current_cp->allVerified)
    {
        tol *= dimension;
    }

    for(int i=0; i<dimension; )
    {
        if(!current_cp->constraintsResolutions[i])
        {
            msg_error()<< "Bad size of constraintsResolutions in GenericConstraintSolver" ;
            break;
        }
        current_cp->constraintsResolutions[i]->init(i, w, force);
        i += current_cp->constraintsResolutions[i]->getNbLines();
    }

    bool showGraph = d_computeGraphs.getValue();
    sofa::type::vector<SReal>* graph_residuals = nullptr;
    if (showGraph)
    {
        graph_residuals = &(*d_graphErrors.beginEdit())["Error"];
        graph_residuals->clear();
    }

    int iterCount = 0;

    //Spectral correction computation. The overhead might be big if W is large.
    SReal rho = 1.0;
    if (d_useSpectralCorrection.getValue())
    {
        Eigen::Map<Eigen::MatrixX<SReal>> EigenW(w[0],dimension, dimension) ;
        SReal eigenRadius = 0;
        for(auto s : EigenW.eigenvalues())
        {
            eigenRadius=std::max(eigenRadius,norm(s));
        }
        rho = d_spectralCorrectionFactor.getValue()*std::min(2.0, 0.2 * 2/eigenRadius);
    }

    std::vector<AsynchSubSolver> asynchSolvers;
    //Setup and distribute constraints ids among threads.
    unsigned usedthreads = std::min(d_maximumNumberOfThread.getValue(),
                                   std::max(1,(int) (current_cp->constraintsResolutions.size()/d_minimumNumberOfLinePerThread.getValue()))) ;
    if (usedthreads == 1)
    {
        deltaF.resize(current_cp->getDimension(),0.0);
    }
    else
    {
        asynchSolvers.reserve(usedthreads);
        //Allocate twice the number of dimension to enable buffer changing mechanism to avoid concurrency
        deltaF.resize(2*current_cp->getDimension(),0.0);

        unsigned beginId = 0;
        unsigned endId = 0;

        //Define how many thread to use
        for (unsigned j=0; j<usedthreads && endId < current_cp->constraintsResolutions.size(); ++j)
        {
            if (j == usedthreads - 1)
            {
                endId = dimension;
            }
            else
            {
                while ((endId - beginId)<(dimension/usedthreads) && (endId < current_cp->constraintsResolutions.size()) )
                {
                    endId += current_cp->constraintsResolutions[endId]->getNbLines();
                }
            }
            asynchSolvers.emplace_back(beginId, endId, dimension, rho, tol, d, correctedD.data(), dfree, w, force, deltaF.data(), lastF.data(), &(current_cp->constraintsResolutions), this);
            beginId = endId;
            //If endId == current_cp->constraintsResolutions.size() we go out, so the number of actual thread might be smaller than expected. This can happen for instance in a case where dimension = 4 but we only have 3 constraint resolutions
            // Or worse, when whe have nbThread=3, dimension = 3 but only one constraint resolution exists.
            // For now on, the number of thread is actually asynchSolvers.size().
        }
        usedthreads = asynchSolvers.size();

        //If we have only one thread, let's stay on the main one
        if (usedthreads == 1)
            asynchSolvers.clear();
        else
        {
            //Setup futures and promises used for synchronization
            m_bufferNumber.store(0);
            m_promises[0] = std::promise<std::tuple<bool, SReal>>();
            m_promises[1] = std::promise<std::tuple<bool, SReal>>();
            std::shared_future<std::tuple<bool, SReal>> sharedFuture0(m_promises[0].get_future());
            std::shared_future<std::tuple<bool, SReal>> sharedFuture1(m_promises[1].get_future());

            //This counter is only used for the first synchronization point
            m_workerCounter.store(0);
            //Launch threads but not the first one given that it'll be the main thread
            for(unsigned i=1; i< usedthreads; ++i) //Start at 1 because the main thread is in charge of the first set of constraints
            {
                asynchSolvers[i].m_futures[0] = sharedFuture0;
                asynchSolvers[i].m_futures[1] = sharedFuture1;
                asynchSolvers[i].startThread();
            }
        }
    }

    for(int i=0; i<current_cp->maxIterations; i++)
    {
        iterCount ++;
        //Compute conjugate residue factor
        const SReal beta = d_useConjugateResidue.getValue() * std::min(1.0, pow( ((float)i)/current_cp->maxIterations,d_conjugateResidueSpeedFactor.getValue()));
        bool constraintsAreVerified;

        //If monothreaded simplify everything.
        if (usedthreads == 1)
        {
            std::tie(constraintsAreVerified, error) = iterate(0, dimension,
                                          dimension, rho, tol, beta,
                                          d, correctedD.data(), dfree, w, force, lastF.data(), deltaF.data(),  deltaF.data(),
                                          current_cp->constraintsResolutions );
            if (current_cp->allVerified)
            {
                if (constraintsAreVerified)
                {
                    convergence = true;
                    break;
                }
            }
            else if(error < tol && i > 0) // do not stop at the first iteration (that is used for initial guess computation)
            {
                convergence = true;
                break;
            }
        }
        else
        {

            //Current value of bufferNumber is actually the old one because threads are waiting on it right now. So it was the old synchronization point.
            const int oldNextBufferNumber = m_bufferNumber.load();
            const int currBufferNumber = (currBufferNumber+ 1)%2;

            //If first iteration, then wait for all threads to synchronize then launch iteration
            if(i==0) //First run
            {
                while(m_workerCounter.load() < usedthreads-1)
                    std::this_thread::sleep_for(std::chrono::microseconds(10)); //TODO is there a better way ?
                m_workerCounter.store(0);
            }

            //Synchronization point. We know all threads are waiting on oldNextBufferNumber future here, we can switch buffers.
            m_bufferNumber.store(currBufferNumber);
            m_promises[oldNextBufferNumber].set_value({true,beta });


            //Compute new value of lambda on our set of constraints
            std::tie(constraintsAreVerified, error) = iterate(asynchSolvers[0].m_idBegin, asynchSolvers[0].m_idEnd, dimension,
                                                    rho, tol, beta, d, correctedD.data(), dfree, w, force, lastF.data(),
                                                    deltaF.data() + dimension*(currBufferNumber), deltaF.data() + dimension*(oldNextBufferNumber),
                                                    current_cp->constraintsResolutions);


            m_promises[oldNextBufferNumber] = std::promise<std::tuple<bool, SReal>>();
            std::shared_future<std::tuple<bool, SReal>> sharedFuture(m_promises[oldNextBufferNumber].get_future());

            for(unsigned sId = 1 ; sId<usedthreads; ++sId) //Start at 1 because the main thread is in charge of the first set of constraints
            {
                std::lock_guard<std::mutex> lock(asynchSolvers[sId].m_accessMutex);
                error += asynchSolvers[sId].m_currError;
                constraintsAreVerified &= asynchSolvers[sId].m_allVerified;
                asynchSolvers[sId].m_futures[oldNextBufferNumber] = sharedFuture;
            }


            if (current_cp->allVerified)
            {
                if (constraintsAreVerified)
                {
                    convergence = true;
                    m_promises[currBufferNumber].set_value({false, 0.0});
                    break;
                }
            }
            else if(error < tol && i > 0) // do not stop at the first iteration (that is used for initial guess computation)
            {
                convergence = true;
                m_promises[currBufferNumber].set_value({false, 0.0});
                break;
            }

            if (i == current_cp->maxIterations - 1 )
                m_promises[currBufferNumber].set_value({false, 0.0});
        }
        if (showGraph)
        {
            graph_residuals->push_back(error);
        }

    }

    sofa::helper::AdvancedTimer::valSet("GS iterations", current_cp->currentIterations);

    current_cp->result_output(this, force, error, iterCount, convergence);


}

std::tuple<bool, SReal>  ImprovedJacobiConstraintSolver::iterate(unsigned idBegin, unsigned idEnd,
                 unsigned dimension, SReal rho, SReal tol, SReal beta,
                 SReal *d, SReal* correctedD, SReal* dfree, SReal** w, SReal* force,  SReal* lastF, SReal* newDeltaF,
                 const SReal* deltaF,
                 std::vector<core::behavior::ConstraintResolution*>& constraintCorr)
{

    bool constraintsAreVerified = true;
    SReal error=0.0;
    for(unsigned j=idBegin; j<idEnd; ) // increment of j realized at the end of the loop
    {
        // 1. nbLines provide the dimension of the constraint
        const unsigned int nb = constraintCorr[j]->getNbLines();

        for(unsigned l=j; l<j+nb; ++l )
        {
            for(unsigned k=0; k<dimension; ++k)
            {
                d[l] +=  w[l][k] * deltaF[k];
            }
            correctedD[l] = rho * d[l]  ;
        }
        j += nb;
    }

    for(unsigned j=idBegin; j<idEnd; ) // increment of j realized at the end of the loop
    {
        // 1. nbLines provide the dimension of the constraint
        const unsigned int nb = constraintCorr[j]->getNbLines();

        constraintCorr[j]->resolution(j,w,correctedD, force, dfree);
        for(unsigned l=j; l<j+nb; ++l )
        {
            force[l] -= beta * deltaF[l] ;
            newDeltaF[l] = force[l] - lastF[l];
            lastF[l] = force[l];
        }

        double cstError = 0.0;
        for(unsigned l=j; l<j+nb; ++l )
        {
            for(unsigned k=0; k<dimension; ++k)
            {
                cstError += pow(w[l][k] * deltaF[k],2);
            }
            constraintsAreVerified = constraintsAreVerified && cstError < pow(tol,2);
        }
        error += sqrt(cstError);
        j+= nb;

    }
    return {constraintsAreVerified, error};
}

void registerImprovedJacobiConstraintSolver(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("A Constraint Solver using the Linear Complementarity Problem formulation to solve Constraint based components using a Projected Gauss-Seidel iterative method")
        .add< ImprovedJacobiConstraintSolver >());
}


}