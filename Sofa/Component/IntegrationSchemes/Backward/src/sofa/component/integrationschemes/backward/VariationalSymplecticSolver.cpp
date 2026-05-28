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
#include <sofa/component/integrationschemes/backward/VariationalSymplecticSolver.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/behavior/BaseMass.h>
#include <sofa/core/behavior/LinearSolver.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/MappingGraph.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/VectorOperations.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalGetNonDiagonalMassesCountVisitor.h>

using sofa::simulation::mechanicalvisitor::MechanicalGetNonDiagonalMassesCountVisitor;


namespace sofa::component::integrationschemes::backward
{

VariationalSymplecticSolver::VariationalSymplecticSolver()
:d_alpha(initData(&d_alpha, 0.5_sreal ,"alpha","Alpha parameter chosen in ]0; 1], if = 0.5 the IS precision is quadratic"))
,d_rayleighStiffness(initData(&d_rayleighStiffness, 0.0_sreal ,"rayleighStiffness","Rayleigh damping coefficient related to stiffness, > 0"))
,d_rayleighMass(initData(&d_rayleighMass, 0.0_sreal ,"rayleighMass","Rayleigh damping coefficient related to mass, > 0"))
,d_computeHamiltonian(initData(&d_computeHamiltonian, false,"computeHamiltonian","Compute hamiltonian, if true, the hamiltonian will be used in the line search"))
,d_hamiltonianEnergy(initData(&d_hamiltonianEnergy, 0.0_sreal ,"hamiltonianEnergy","Current value of the hamiltonian energy"))
,d_useIncrementalPotentialEnergy(initData(&d_useIncrementalPotentialEnergy, true,"useIncrementalPotentialEnergy","use real potential energy, if false use approximate potential energy"))
{
}


void VariationalSymplecticSolver::doSetupIntegrationStep(const core::ExecParams* params, SReal dt, sofa::core::MultiVecCoordId xResult, sofa::core::MultiVecDerivId vResult)
{

    simulation::common::VectorOperations::realloc(*m_vop, m_unknown, "dv", this, true);
    simulation::common::VectorOperations::realloc(*m_vop, m_momentum, "momentum", this, true);
    simulation::common::VectorOperations::realloc(*m_vop, m_r0, "r0", this, true);

    m_x0.resize(1);
    simulation::common::VectorOperations::realloc(*m_vop, m_x0[0], "x0", this, true);
    sofa::core::behavior::MultiVecCoord x0(m_vop.get(), m_x0[0]);
    x0.eq(core::vec_id::write_access::position);

    if (this->getTime() < std::numeric_limits<SReal>::epsilon())
    {
        computeMomentum(m_momentum, m_xResult, m_vResult);
    }

}

/**
 * Compute the system matrix.
 */
void VariationalSymplecticSolver::computeLHS(bool firstIteration)
{
    SOFA_UNUSED(firstIteration);

    SCOPED_TIMER("setSystemMBKMatrix");
    const core::MatricesFactors::M mFact( 1.0/m_dt + d_rayleighMass.getValue());
    const core::MatricesFactors::B bFact( 1.0 );
    const core::MatricesFactors::K kFact( m_dt * d_alpha.getValue() * (1 - d_alpha.getValue()) - d_rayleighStiffness.getValue());

    m_mop->setSystemMBKMatrix(mFact, bFact, kFact, l_linearSolver.get());

}

/**
* compute the current RHS.
*/
void VariationalSymplecticSolver::computeRHS(bool firstIteration)
{

    sofa::core::behavior::MultiVecDeriv f(m_vop.get(), core::vec_id::write_access::force );
    f.clear();

    {
        //TODO deal with that.
        SCOPED_TIMER("ComputeForce");
        m_mop->mparams.setImplicit(true); // this solver is implicit
        // compute the net forces at the beginning of the time step
        m_mop->computeForce(f);                                                               //f = Kx + Bv

    }
    {
        SCOPED_TIMER("ComputeRHTerm");

        m_vop->v_eq(m_r0, f, 1-d_alpha.getValue());
        auto backV = m_mop->mparams.v();
        m_mop->mparams.setV(m_vResult);

        m_mop->addMBKv(m_r0, core::MatricesFactors::M(1.0/m_dt + 1.0/m_dt * d_rayleighMass.getValue()),
                                core::MatricesFactors::B(0.0),
                                core::MatricesFactors::K(- 1.0/m_dt * d_rayleighStiffness.getValue()));


        m_mop->mparams.setV(backV);
        m_vop->v_eq(m_r0, m_momentum, -1/m_dt);

        // Set the factor of the left hand side taking into account the rayleigh damping
        // Apply projective constraints to the full residue
        m_mop->projectResponse(m_r0);

    }


}


/**
 * Returns the evaluation of the current residue
 */
SReal VariationalSymplecticSolver::evaluateResidue()
{
    core::behavior::MultiVecDeriv r0(m_vop.get(), m_r0);

    return r0.dot(r0);
}


/**
 * Solve the linear equation from a Newton iteration, i.e. it computes (x^{i+1}-x^i).
 */
void VariationalSymplecticSolver::solveLinearEquation()
{
    SCOPED_TIMER("MBKSolve");

    l_linearSolver->getLinearSystem()->setSystemSolution(m_unknown);
    l_linearSolver->getLinearSystem()->setRHS(m_r0);
    l_linearSolver->solveSystem();
    l_linearSolver->getLinearSystem()->dispatchSystemSolution(m_unknown);
}

/**
 * Once (x^{i+1}-x^i) has been computed, the result is used internally to update the current
 * guess. It computes x^{i+1} += alpha * dx, where dx is the result of the linear system. It is
 * not necessary to share the result with the Newton-Raphson method.
 */
void VariationalSymplecticSolver::updateStatesFromLinearSolution(SReal alpha, bool firstIteration)
{
    SOFA_UNUSED(firstIteration);

    sofa::core::behavior::MultiVecCoord pos(m_vop.get(), m_xResult);
    sofa::core::behavior::MultiVecDeriv vel(m_vop.get(), m_vResult);

    vel.peq(m_unknown, alpha);
    pos.eq(m_x0[0],vel.id(), m_dt * d_alpha.getValue() );
}

void VariationalSymplecticSolver::computeMomentum(sofa::core::MultiVecDerivId momentum, sofa::core::MultiVecCoordId position, sofa::core::MultiVecDerivId velocity)
{
    auto backX = m_mop->mparams.x();
    auto backV = m_mop->mparams.v();
    m_mop->mparams.setV(velocity);
    m_mop->mparams.setX(position);

    m_vop->v_clear(momentum);
    m_mop->computeForce(momentum);
    m_vop->v_teq(momentum, d_alpha.getValue() * m_dt);
    m_mop->addMBKv(momentum, core::MatricesFactors::M(1.0),
                                core::MatricesFactors::B(0),
                                core::MatricesFactors::K(0) );

    m_mop->mparams.setV(backV);
    m_mop->mparams.setX(backX);

}

void VariationalSymplecticSolver::postSolve()
{
    sofa::core::behavior::MultiVecCoord pos(m_vop.get(), m_xResult);
    sofa::core::behavior::MultiVecDeriv vel(m_vop.get(), m_vResult);

    computeMomentum(m_momentum,m_xResult,m_vResult);

    //Only need to add (1-alpha)*vel because alpha has already been accumulated during the last
    //call to updateStatesFromLinearSolution
    pos.peq(vel.id(), 1-d_alpha.getValue());
}

SReal VariationalSymplecticSolver::getVelocityIntegrationFactor() const
{
    return 0.0;
}


SReal VariationalSymplecticSolver::getPositionIntegrationFactor() const
{
    return 1.0;
}

sofa::Size  VariationalSymplecticSolver::getIntegrationSchemeTimeOrder() const
{
    return 1;
}


void registerVariationalSymplecticSolver(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Time integrator using implicit backward Euler scheme.")
        .add< VariationalSymplecticSolver >());
}

} // namespace sofa::component::integrationschemes::forward
