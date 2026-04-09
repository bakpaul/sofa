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
#include <sofa/simulation/integrationschemes/AccelerationBasedIntegrationScheme.h>
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

//#define SOFA_NO_VMULTIOP

namespace sofa::simulation::integrationschemes
{
AccelerationBasedIntegrationScheme::AccelerationBasedIntegrationScheme()
{
    
}


void AccelerationBasedIntegrationScheme::doSetupIntegrationStep(const core::ExecParams* params, SReal dt, sofa::core::MultiVecCoordId xResult, sofa::core::MultiVecDerivId vResult)
{

    sofa::simulation::common::VectorOperations vop( m_params, this->getContext() );
    sofa::simulation::common::MechanicalOperations mop( m_params, this->getContext() );
    simulation::common::VectorOperations::realloc(vop, m_r0, "r0", this);
    simulation::common::VectorOperations::realloc(vop, m_r1, "r1", this);
    simulation::common::VectorOperations::realloc(vop, m_r2, "r2", this);
    simulation::common::VectorOperations::realloc(vop, m_x0, "x0", this);
    simulation::common::VectorOperations::realloc(vop, m_v0, "v0", this);
    simulation::common::VectorOperations::realloc(vop, m_a0, "a0", this);
    simulation::common::VectorOperations::realloc(vop, m_unknown, "da", this);

    computeCurrentAccelerationFromVelocity(m_a0, core::vec_id::write_access::velocity);
}

/**
 * Compute the system matrix.
 */
void AccelerationBasedIntegrationScheme::computeLHS(unsigned iteration)
{
    SOFA_UNUSED(iteration);

    sofa::simulation::common::VectorOperations vop( m_params, this->getContext() );
    sofa::simulation::common::MechanicalOperations mop( m_params, this->getContext() );

    {
        SReal DGx = getPositionUpdateDerivedFromVelocity()*getVelocityUpdateDerivedFromAcceleration() + getPositionUpdateDerivedFromAcceleration();
        SReal DGv = getVelocityUpdateDerivedFromAcceleration();

        SCOPED_TIMER("setSystemMBKMatrix");
        const core::MatricesFactors::M mFact( 1 - DGv * d_rayleighMass.getValue() );
        const core::MatricesFactors::B bFact( -DGv );
        const core::MatricesFactors::K kFact( - DGx - DGv * d_rayleighMass.getValue() );

        mop.setSystemMBKMatrix(mFact, bFact, kFact, l_linearSolver.get());

#ifdef SOFA_DUMP_VISITOR_INFO
        simulation::Visitor::printNode("SystemSolution");
#endif
    }

}

/**
* compute the current RHS.
*/
void AccelerationBasedIntegrationScheme::computeRHS(unsigned iteration)
{
    sofa::simulation::common::VectorOperations vop( m_params, this->getContext() );
    sofa::simulation::common::MechanicalOperations mop( m_params, this->getContext() );
    sofa::core::behavior::MultiVecCoord pos(&vop, core::vec_id::write_access::position );
    sofa::core::behavior::MultiVecDeriv vel(&vop, core::vec_id::write_access::velocity );
    sofa::core::behavior::MultiVecDeriv f(&vop, core::vec_id::write_access::force );


    sofa::core::behavior::MultiVecDeriv b(&vop, m_r0 );
    b.clear();



    {
        SCOPED_TIMER("ComputeForce");
        mop->setImplicit(true); // this solver is implicit
        // compute the net forces at the beginning of the time step
        mop.computeForce(f);                                                               //f = Kx + Bv

        msg_info() << "initial f = " << f;
    }

    {
        SCOPED_TIMER("ComputeRHTerm");
        // new more powerful visitors

        // force in the current configuration
        b.eq(f, 1.0);  // b = f

        msg_info() << "f = " << f;

        if (   fabs(d_rayleighMass.getValue()) > std::numeric_limits<SReal>::epsilon()
            || fabs(d_rayleighMass.getValue()) > std::numeric_limits<SReal>::epsilon())
        {
            //TODO verify if we need to modulate by the timestep
            mop.addMBKv(b, core::MatricesFactors::M(d_rayleighMass.getValue()),
            core::MatricesFactors::B(0),
            core::MatricesFactors::K(d_rayleighStiffness.getValue()));
        }


        if (iteration == 0) [[unlikely]]
        {
            sofa::core::behavior::MultiVecDeriv r1(&vop, m_r1 );
            r1.clear();
            sofa::core::behavior::MultiVecDeriv r2(&vop, m_r2 );
            r2.clear();

            computePositionUpdateFromVelocityAndAcceleration(m_r1, core::vec_id::write_access::velocity, m_a0);
            r1.teq(-1);
            r1.peq(core::vec_id::write_access::position);

            auto backV = mop->v();
            mop->setV(m_r1);
            // add the change of force due to stiffness + Rayleigh damping
            mop.addMBKv(b, core::MatricesFactors::M(0.0),
                        core::MatricesFactors::B(0),
                        core::MatricesFactors::K(1.0));
            b.peq(m_r1, -1.0);


            //TODO
            //computeVelocityUpdateFromAcceleration(core::vec_id::read_access::velocity, m_a0);
            r2.teq(-1);
            r2.peq(core::vec_id::write_access::velocity);

            mop->setV(m_r2);
            // add the change of force due to stiffness + Rayleigh damping
            mop.addMBKv(b, core::MatricesFactors::M(0.0),
                        core::MatricesFactors::B(1.0),
                        core::MatricesFactors::K(getPositionUpdateDerivedFromVelocity()));
            b.peq(m_r1, -1.0);
            mop->setV(backV);
        }

        if (iteration) [[likely]]
        {
            auto backV = mop->v();
            mop->setV(m_a0);
            // add the change of force due to stiffness + Rayleigh damping
            mop.addMBKv(b, core::MatricesFactors::M(1.0),
                        core::MatricesFactors::B(0),
                        core::MatricesFactors::K(0));
            mop->setV(backV);
        }

        msg_info() << "b = " << b;

        mop.projectResponse(b);                                   // b is projected to the constrained space

        msg_info() << "projected b = " << b;
    }

}


/**
 * Returns the squared norm of the last evaluation of the RHS
 */
SReal AccelerationBasedIntegrationScheme::squaredNormRHS()
{
    sofa::simulation::common::VectorOperations vop( m_params, this->getContext() );

    core::behavior::MultiVecDeriv r0(&vop, m_r0);
    core::behavior::MultiVecDeriv r1(&vop, m_r1);

    return r0.dot(r0) + r1.dot(r1);
}


/**
 * Solve the linear equation from a Newton iteration, i.e. it computes (x^{i+1}-x^i).
 */
void AccelerationBasedIntegrationScheme::solveLinearEquation()
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
void AccelerationBasedIntegrationScheme::updateVelocityAndPositionFromLinearSolution(SReal alpha, unsigned iteration)
{
    sofa::simulation::common::VectorOperations vop( m_params, this->getContext() );

    sofa::core::behavior::MultiVecCoord pos(&vop, core::vec_id::write_access::position );
    sofa::core::behavior::MultiVecDeriv vel(&vop, core::vec_id::write_access::velocity );

    const SReal DGx = getPositionUpdateDerivedFromVelocity()*getVelocityUpdateDerivedFromAcceleration() + getPositionUpdateDerivedFromAcceleration();
    const SReal DGv = getVelocityUpdateDerivedFromAcceleration();


    vel.peq(m_unknown, DGv);
    pos.peq(m_unknown, DGx);
    if (iteration == 0) [[unlikely]]
    {
        vel.peq(m_r2, -1.0);

        pos.peq(m_r1, -1.0);
        pos.peq(m_r2, -getPositionUpdateDerivedFromVelocity());
    }
}

/**
 * Compute ||x^{i+1}-x^i||^2
 */
SReal AccelerationBasedIntegrationScheme::squaredNormDSolution()
{
    sofa::simulation::common::VectorOperations vop( m_params, this->getContext() );

    core::behavior::MultiVecDeriv dv(&vop, m_unknown);

    return dv.dot(dv);
}

void AccelerationBasedIntegrationScheme::computeCurrentAccelerationFromVelocity(sofa::core::MultiVecDerivId& result, const sofa::core::MultiVecDerivId& velocity)
{

};


} // namespace sofa::component::integrationschemes::forward
