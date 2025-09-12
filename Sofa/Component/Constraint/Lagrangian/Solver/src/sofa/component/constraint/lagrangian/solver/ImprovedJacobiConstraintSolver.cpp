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
                 std::vector<core::behavior::ConstraintResolution*>* constraintCorr)
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
}

ImprovedJacobiConstraintSolver::AsynchSubSolver::~AsynchSubSolver()
{
    if (m_thread.joinable())
        m_thread.join();
}


ImprovedJacobiConstraintSolver::ImprovedJacobiConstraintSolver()
    : BuiltConstraintSolver()
    , d_numberOfThread(initData(&d_numberOfThread, 1, "numberOfThread", "Number of thread to execute solver concurrently"))
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
        force[i] = 0;
    }

    // The size of those buffer depend on the number of thread that are used.
    // If only one thread is used then the vector is of size dimension
    // Else it is of twice the dimensions, to allow the use of a switching buffer mechanism
    std::vector<SReal> lastF;
    std::vector<SReal> deltaF;

    std::vector<SReal> correctedD;
    correctedD.resize(current_cp->getDimension(), 0.0);

//    std::cout<<"Initialized vectors"<<std::endl;

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

    sofa::type::vector<SReal> tabErrors(dimension);

    int iterCount = 0;

    Eigen::Map<Eigen::MatrixX<SReal>> EigenW(w[0],dimension, dimension) ;
    SReal eigenRadius = 0;
    for(auto s : EigenW.eigenvalues())
    {
        eigenRadius=std::max(eigenRadius,norm(s));
    }
    const SReal rho = std::min(1.0, 0.9 * 2/eigenRadius);

    std::vector<AsynchSubSolver> asynchSolvers;
    //Setup and distribute constraints ids among threads
    unsigned usedthreads = std::min(d_numberOfThread.getValue(),(int) current_cp->constraintsResolutions.size()) ;
    if (usedthreads == 1)
    {
        lastF.resize(current_cp->getDimension(),0.0);
        deltaF.resize(current_cp->getDimension(),0.0);
    }
    else
    {
        //Allocate twice the number of dimension to enable buffer changing mechanism to avoid concurrency
        lastF.resize(2*current_cp->getDimension(),0.0);
        deltaF.resize(2*current_cp->getDimension(),0.0);

        unsigned beginId = 0;
        unsigned endId = 0;
        unsigned cstId = 0;
        //Define how many thread to use
        for (unsigned j=0; j<usedthreads && cstId < current_cp->constraintsResolutions.size(); ++j)
        {
            if (j == usedthreads - 1)
            {
                endId = dimension;
            }
            else
            {
                while ((endId - beginId)<(dimension/usedthreads) && (cstId < current_cp->constraintsResolutions.size()) )
                {
                    endId += current_cp->constraintsResolutions[cstId]->getNbLines();
                    ++cstId;
                }
            }

            asynchSolvers.emplace_back(beginId, endId, dimension, rho, tol, d, correctedD.data(), dfree, w, force, deltaF.data(), lastF.data(), &(current_cp->constraintsResolutions));
            beginId = endId;
            //If endId == current_cp->constraintsResolutions.size() we go out, so the number of actual thread might be smaller than expected. This can happen for instance in a case where dimension = 4 but we only have 3 constraint resolutions
            // Or worse, when whe have nbThread=3, dimension = 3 but only one constraint resolution exists.
            // For now on, the number of thread is actually asynchSolvers.size().
        }
        usedthreads = asynchSolvers.size();
        if (usedthreads == 1)
            asynchSolvers.clear();
    }

    //Thread initialization
    if (usedthreads > 1)
    {

    }

    for(int i=0; i<current_cp->maxIterations; i++)
    {
        iterCount ++;
        SReal beta = std::min(1.0, pow( ((float)i)/current_cp->maxIterations,0.6));


        if (usedthreads == 1)
        {
            bool constraintsAreVerified;
            std::tie(constraintsAreVerified, error) = ImprovedJacobiConstraintSolver::iterate(0, dimension,
                                          dimension, rho, tol, beta,
                                          d, correctedD.data(), dfree, w, force, deltaF.data(), lastF.data(), deltaF.data(), lastF.data(),
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

        }
    }

    sofa::helper::AdvancedTimer::valSet("GS iterations", current_cp->currentIterations);

    current_cp->result_output(this, force, error, iterCount, convergence);


}

std::tuple<bool, SReal>  ImprovedJacobiConstraintSolver::iterate(unsigned idBegin, unsigned idEnd,
                 unsigned dimension, SReal rho, SReal tol, SReal beta,
                 SReal *d, SReal* correctedD, SReal* dfree, SReal** w, SReal* force, SReal* newDeltaF, SReal* newLastF,
                 const SReal* deltaF, const SReal* lastF,
                 std::vector<core::behavior::ConstraintResolution*>& constraintCorr)
{

    bool constraintsAreVerified = true;
    SReal error=0.0;

    for(int j=idBegin; j<idEnd; ) // increment of j realized at the end of the loop
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
        constraintCorr[j]->resolution(j,w,correctedD, force, dfree);
        for(unsigned l=j; l<j+nb; ++l )
        {
            force[l] += beta * deltaF[l] ;
            newDeltaF[l] = force[l] - lastF[l];
            newLastF[l] = force[l];
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