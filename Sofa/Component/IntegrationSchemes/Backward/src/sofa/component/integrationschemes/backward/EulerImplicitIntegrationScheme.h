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
#include <sofa/component/integrationschemes/backward/config.h>
#include <sofa/core/behavior/LinearSolverAccessor.h>

#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/VectorOperations.h>
#include <sofa/core/behavior/IntegrationScheme.h>

namespace sofa::simulation::common
{
class VectorOperations;
}
namespace sofa::component::integrationschemes::backward
{

/** Semi-implicit time integrator using backward Euler scheme for first and second degree ODEs. (default: second)
 *
 *** 2nd Order ***
 *
 * This is based on [Baraff and Witkin, Large Steps in Cloth Simulation, SIGGRAPH 1998]
 * The integration scheme is based on the following equations:
 *
 *   \f$x_{t+h} = x_t + h v_{t+h}\f$
 *   \f$v_{t+h} = v_t + h a_{t+h}\f$
 *
 *   The unknown is
 *   \f$v_{t+h} - v_t = dv\f$
 *
 *   Newton's law is
 *   \f$ M dv = h f(t+h) \f$
 *   \f$ M dv = h ( f(t) + K dx     + (B - r_M M + r_K K) (v+dv) )\f$
 *   \f$ M dv = h ( f(t) + K h (v+dv) + (B - r_M M + r_K K) (v+dv) )\f$
 *
 *   \f$ M \f$ is the mass matrix.
 *   \f$ K = df/dx \f$ is the stiffness implemented (or not) by the force fields.
 *   \f$ B = df/dv \f$ is the damping implemented (or not) by the force fields.
 *   An additional, uniform Rayleigh damping  \f$- r_M M + r_K K\f$ is imposed by the solver.
 *
 * This corresponds to the following equation system:
 *
 *   \f$ ( (1+h r_M) M - h B - h(h + r_K) K ) dv = h ( f(t) + (h+r_K) K v + B v - r_M M v )\f$
 *
 * Moreover, the projective constraints filter out the forbidden motions.
 * This is equivalent with multiplying vectors with a projection matrix \f$P\f$.
 * Finally, the equation system set by this ode solver is:
 *
 *   \f$ P ( (1+h r_M) M - h B - h(h + r_K) K ) P dv = P h ( f(t) + (h + r_K) K v + B v - r_M M v )\f$
 *
 *** 1st Order ***
 *
 * This integration scheme is based on the following equation:
 *
 *   \f$x_{t+h} = x_t + h v_{t+h}\f$
 *
 * Applied to this mechanical system:
 *
 *   \f$ M v_t = f_{ext} \f$
 *
 *   \f$ M v_{t+h} = f_{ext_{t+h}} \f$
 *   \f$           = f_{ext_{t}} + h (df_{ext}/dt)_{t+h} \f$
 *   \f$           = f_{ext_{t}} + h (df_{ext}/dx)_{t+h} v_{t+h} \f$
 *   \f$           = f_{ext_{t}} - h K v_{t+h} \f$
 *
 *   \f$ ( M + h K ) v_{t+h} = f_{ext} \f$
 *
 *
 *** Trapezoidal Rule ***
 *
 * The trapezoidal scheme is based on
 *
 *   \f$v_{t+h} = h/2 ( f(t+h) + f(t) )\f$
 *
 * With this and the same techniques as for the implicit Euler scheme we receive for *** 2nd Order *** equations
 *
 *   \f$ P ( (1+h/2 r_M) M - h/2 B - h/2 (h + r_K) K ) P dv = P h/2 ( 2 f(t) + (h + r_K) K v + B v - r_M M v )\f$
 *
 * and for *** 1st Order ***
 *
 *   \f$ ( M + h/2 K ) v_{t+h} = f_{ext} \f$
 *
 */
class SOFA_COMPONENT_INTEGRATIONSCHEMES_BACKWARD_API EulerImplicitIntegrationScheme :
    public sofa::core::behavior::IntegrationScheme,
    public sofa::core::behavior::LinearSolverAccessor
{
public:
    SOFA_CLASS2(EulerImplicitIntegrationScheme, sofa::core::behavior::IntegrationScheme, sofa::core::behavior::LinearSolverAccessor);

    Data<SReal> d_rayleighStiffness; ///< Rayleigh damping coefficient related to stiffness, > 0
    Data<SReal> d_rayleighMass; ///< Rayleigh damping coefficient related to mass, > 0
    Data<SReal> d_velocityDamping; ///< Velocity decay coefficient (no decay if null)
    Data<bool> d_firstOrder; ///< Use backward Euler scheme for first order ODE system, which means that only the first derivative of the DOFs (state) appears in the equation. Higher derivatives are absent
    Data<bool> d_trapezoidalScheme; ///< Boolean to use the trapezoidal scheme instead of the implicit Euler scheme and get second order accuracy in time (false by default)
    Data<bool> d_solveConstraint; ///< Apply ConstraintSolver (requires a ConstraintSolver in the same node as this solver, disabled by by default for now)
    Data<bool> d_threadSafeVisitor; ///< If true, do not use realloc and free visitors in fwdInteractionForceField.

    Data<bool> d_computeResidual; ///< If true, the residual is computed at the end of the solving
    Data<SReal> d_residual; ///< Residual norm at the end of the free-motion solving

protected:
    EulerImplicitIntegrationScheme();
public:

    virtual void setupIntegrationStep(const core::ExecParams* params, SReal dt, sofa::core::MultiVecCoordId xResult, sofa::core::MultiVecDerivId vResult) override;

    /**
     * Compute the system matrix.
     */
    virtual void computeLHS() override;

    /**
    * compute the current RHS.
    */
    virtual void computeRHS() override;

    /**
     * Returns the squared norm of the last evaluation of the RHS
     */
    virtual SReal squaredNormRHS() override;


    /**
     * Solve the linear equation from a Newton iteration, i.e. it computes (x^{i+1}-x^i).
     */
    virtual void solveLinearEquation() override;

    /**
     * Once (x^{i+1}-x^i) has been computed, the result is used internally to update the current
     * guess. It computes x^{i+1} += alpha * dx, where dx is the result of the linear system. It is
     * not necessary to share the result with the Newton-Raphson method.
     */
    virtual void updateVelocityAndPositionFromLinearSolution( SReal alpha) override;

    /**
     * Compute ||x^{i+1}-x^i||^2
     */
    virtual SReal squaredNormDSolution() override;

    /**
     * Compute ||x^{i+1}||^2
     */
    virtual SReal squaredLastSolution() override;

    virtual void init() override;
    virtual void cleanup() override;

protected:

    sofa::core::MultiVecCoordId m_newPosID;
    sofa::core::MultiVecDerivId m_newVelID;

    /// the solution vector is stored for warm-start
    core::behavior::MultiVecDeriv x;

    /// Right-hand side vector
    core::behavior::MultiVecDeriv b;

    /// Residual vector (optionally computed)
    core::behavior::MultiVecDeriv m_residual;

    void reallocSolutionVector(sofa::simulation::common::VectorOperations* vop);
    void reallocRightHandSideVector(sofa::simulation::common::VectorOperations* vop);
    void reallocResidualVector(sofa::simulation::common::VectorOperations* vop);

    SReal m_trapezoidFactor;
};

} // namespace sofa::component::integrationschemes::backward
