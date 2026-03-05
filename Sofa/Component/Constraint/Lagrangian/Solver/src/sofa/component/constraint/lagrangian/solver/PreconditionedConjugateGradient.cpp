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

#include <float.h>
#include <sofa/component/constraint/lagrangian/solver/GenericConstraintSolver.h>
#include <sofa/component/constraint/lagrangian/solver/PreconditionedConjugateGradient.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/linearalgebra/FullMatrix.h>
#include <sofa/linearalgebra/FullVector.h>

#include <Eigen/Eigenvalues>

namespace sofa::component::constraint::lagrangian::solver
{

PreconditionedConjugateGradient::PreconditionedConjugateGradient()
    : BuiltConstraintSolver()
{}

    void PreconditionedConjugateGradient::doSolve(GenericConstraintProblem * problem , SReal timeout)
{
    SCOPED_TIMER_VARNAME(gaussSeidelTimer, "PreconditionedConjugateGradient");


    const int dimension = problem->getDimension();

    if(!dimension)
    {
        problem->currentError = 0.0;
        problem->currentIterations = 0;
        return;
    }

    const auto & dfree = problem->dFree;
    auto & force = problem->f;
    const auto & W = problem->W;
    SReal tol = problem->tolerance;

    //The use of std::vector-based matrix and vectors is motivated by the destruction
    linearalgebra::FullVector<SReal> r(dimension);
    linearalgebra::FullVector<SReal> p(dimension);
    linearalgebra::FullVector<SReal> Wp(dimension);
    linearalgebra::FullMatrix<SReal> MW(dimension, dimension);

    // ===== BEGIN Initialization =====
    // Initialize state (matrices/vectors) and apply Jacobi preconditioner on left and right to keep symetry
    // This imply that the solution will be P^{-1}*x and thus should be corrected after solving.
    for(unsigned j=0; j< dimension; ++j)
    {
        const SReal invWjj = 1.0/sqrt(W[j][j]);
        for(unsigned k=0; k< dimension; ++k)
        {
            const SReal invWkk = 1.0/sqrt(W[k][k]);

            MW[j][k] = W[j][k] * invWjj * invWkk;
        }
    }

    for(unsigned j=0; j< dimension; ++j)
    {
        const SReal invWjj = 1.0/sqrt(W[j][j]);
        r[j] = -dfree[j] * invWjj;
    }

    memset(force.ptr(), 0, dimension*sizeof(SReal));
    p = r;
    SReal previousRSqrdNorm = r.dot(r);
    // ===== END Initialization =====


    SReal error=0.0;
    bool convergence = false;
    if(problem->scaleTolerance && !problem->allVerified)
    {
        tol *= dimension;
    }

    const unsigned maxIt = std::min(problem->maxIterations, problem->getDimension());
    unsigned iterCount = 0;
    for(unsigned i=0; i<maxIt; i++)
    {
        iterCount ++;
        bool constraintsAreVerified = true;

        Wp = MW * p;
        const SReal alpha = previousRSqrdNorm /(p.dot(Wp));
        force.eq(force, p, alpha);
        r.eq(r, Wp, -alpha);

        error=0.0;

        for(int j=0; j<dimension; ) // increment of j realized at the end of the loop
        {
            // 1. nbLines provide the dimension of the constraint
            const unsigned int nb = problem->constraintsResolutions[j]->getNbLines();
            SReal cstError = 0.0;
            for(unsigned l=j; l<j+nb; ++l )
            {
                cstError += pow(r[i],2);
                constraintsAreVerified = constraintsAreVerified && cstError < pow(tol,2);
            }
            error += sqrt(cstError);
            j+= nb;
        }

        if (problem->allVerified)
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

        const SReal rSqrdNorm = r.dot(r);
        const SReal beta = rSqrdNorm/previousRSqrdNorm;
        previousRSqrdNorm = rSqrdNorm;

        p.eq(r, p, beta);

    }

    //Apply preconditionner to unknown to get real force
    for(int j=0; j<dimension; ++j)
    {
        force[j] /= sqrt(W[j][j]);
    }

    sofa::helper::AdvancedTimer::valSet("PCG iterations", problem->currentIterations);

    problem->result_output(this, force.ptr(), error, iterCount, convergence);

}



void registerPreconditionedConjugateGradient(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("A Constraint Solver using the Linear Complementarity Problem formulation to solve Constraint based components using a Projected Jacobi iterative method")
        .add< PreconditionedConjugateGradient >());
}


}