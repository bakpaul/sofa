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
#include <sofa/component/integrationschemes/backward/EulerImplicitIntegrationScheme.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/ScopedAdvancedTimer.h>

#include <iomanip>


namespace sofa::component::integrationschemes::backward
{

using core::VecId;
using namespace sofa::defaulttype;
using namespace core::behavior;

EulerImplicitIntegrationScheme::EulerImplicitIntegrationScheme()
    : d_rayleighStiffness(initData(&d_rayleighStiffness, (SReal)0.0, "rayleighStiffness", "Rayleigh damping coefficient related to stiffness, > 0") )
    , d_rayleighMass(initData(&d_rayleighMass, (SReal)0.0, "rayleighMass", "Rayleigh damping coefficient related to mass, > 0"))
    , d_velocityDamping(initData(&d_velocityDamping, (SReal)0.0, "vdamping", "Velocity decay coefficient (no decay if null)") )
    , d_firstOrder (initData(&d_firstOrder, false, "firstOrder", "Use backward Euler scheme for first order ODE system, which means that only the first derivative of the DOFs (state) appears in the equation. Higher derivatives are absent"))
    , d_trapezoidalScheme( initData(&d_trapezoidalScheme,false,"trapezoidalScheme","Boolean to use the trapezoidal scheme instead of the implicit Euler scheme and get second order accuracy in time (false by default)") )
    , d_solveConstraint(initData(&d_solveConstraint, false, "solveConstraint", "Apply ConstraintSolver (requires a ConstraintSolver in the same node as this solver, disabled by by default for now)") )
    , d_threadSafeVisitor(initData(&d_threadSafeVisitor, false, "threadSafeVisitor", "If true, do not use realloc and free visitors in fwdInteractionForceField."))
    , d_computeResidual(initData(&d_computeResidual, false, "computeResidual", "If true, the residual is computed at the end of the solving"))
    , d_residual(initData(&d_residual, std::numeric_limits<SReal>::max(), "residual", "Residual norm at the end of the free-motion solving"))
{
}





void EulerImplicitIntegrationScheme::init()
{
    sofa::core::behavior::IntegrationScheme::init();
    sofa::core::behavior::LinearSolverAccessor::init();

    if (!this->getTags().empty())
    {
        msg_info() << "Responsible for the following objects with tags " << this->getTags() << " :";
        type::vector<core::objectmodel::BaseComponent*> objs;
        this->getContext()->get<core::objectmodel::BaseComponent>(&objs,this->getTags(),sofa::core::objectmodel::BaseContext::SearchDown);
        for (const auto* obj : objs)
        {
            msg_info() << "  " << obj->getClassName() << ' ' << obj->getName();
        }
    }

    simulation::common::VectorOperations vop(sofa::core::execparams::defaultInstance(), this->getContext());
    reallocSolutionVector(&vop);
    reallocRightHandSideVector(&vop);


}


void EulerImplicitIntegrationScheme::setupIntegrationStep(const core::ExecParams* params, SReal dt, sofa::core::MultiVecCoordId xResult, sofa::core::MultiVecDerivId vResult)
{
    m_params = params;

    sofa::simulation::common::VectorOperations vop( m_params, this->getContext() );
    sofa::simulation::common::MechanicalOperations mop( m_params, this->getContext() );

    m_newPosID = xResult;
    m_newVelID = vResult;

    // the only difference for the trapezoidal rule is the factor tr = 0.5 for some usages of h
    const bool optTrapezoidal = d_trapezoidalScheme.getValue();
    if (optTrapezoidal)
        m_trapezoidFactor = 0.5;
    else
        m_trapezoidFactor = 1.0;

    MultiVecDeriv dx(&vop, core::vec_id::write_access::dx);
    dx.realloc(&vop, !d_threadSafeVisitor.getValue(), true);

    reallocSolutionVector(&vop);
    reallocRightHandSideVector(&vop);

    simulation::common::VectorOperations::realloc(vop, m_x0, "v_t", this);
    simulation::common::VectorOperations::realloc(vop, m_r0, "rhs", this);
    simulation::common::VectorOperations::realloc(vop, m_r1, "r1", this);

    vop.v_eq(m_x0, mop.mparams.v());

}

void EulerImplicitIntegrationScheme::cleanup()
{
    // free the locally created vectors (including eventual external mechanical states linked by an InteractionForceField)
    sofa::simulation::common::VectorOperations vop( core::execparams::defaultInstance(), this->getContext() );
    vop.v_free(x.id(), !d_threadSafeVisitor.getValue(), true);
    vop.v_free(b.id(), !d_threadSafeVisitor.getValue(), true);
    vop.v_free(m_residual.id(), !d_threadSafeVisitor.getValue(), true);
}


void EulerImplicitIntegrationScheme::computeLHS()
{
    sofa::simulation::common::VectorOperations vop( m_params, this->getContext() );
    sofa::simulation::common::MechanicalOperations mop( m_params, this->getContext() );

    const bool firstOrder = d_firstOrder.getValue();

    // the only difference for the trapezoidal rule is the factor tr = 0.5 for some usages of h
    const bool optTrapezoidal = d_trapezoidalScheme.getValue();
    {
        SCOPED_TIMER("setSystemMBKMatrix");
        const core::MatricesFactors::M mFact (firstOrder ? 1 : 1 + m_trapezoidFactor * m_dt * d_rayleighMass.getValue());
        const core::MatricesFactors::B bFact (firstOrder ? 0 : -m_trapezoidFactor * m_dt);
        const core::MatricesFactors::K kFact (firstOrder ? -m_dt * m_trapezoidFactor : -m_trapezoidFactor * m_dt * (m_trapezoidFactor * m_dt + d_rayleighStiffness.getValue()));

        mop.setSystemMBKMatrix(mFact, bFact, kFact, l_linearSolver.get());

#ifdef SOFA_DUMP_VISITOR_INFO
        simulation::Visitor::printNode("SystemSolution");
#endif
    }
}


void EulerImplicitIntegrationScheme::computeRHS()
{
    sofa::simulation::common::VectorOperations vop( m_params, this->getContext() );
    sofa::simulation::common::MechanicalOperations mop( m_params, this->getContext() );
    MultiVecCoord pos(&vop, core::vec_id::write_access::position );
    MultiVecDeriv vel(&vop, core::vec_id::write_access::velocity );
    MultiVecDeriv f(&vop, core::vec_id::write_access::force );

    const bool firstOrder = d_firstOrder.getValue();


    {
        SCOPED_TIMER("ComputeForce");
        mop->setImplicit(true); // this solver is implicit
        // compute the net forces at the beginning of the time step
        mop.computeForce(f);                                                               //f = Kx + Bv

        msg_info() << "initial f = " << f;
    }

    {
        SCOPED_TIMER("ComputeRHTerm");
        if (firstOrder)
        {
            b.eq(f);  // b = f
        }
        else
        {
            // new more powerful visitors

            // force in the current configuration
            b.eq(f);  // b = f

            msg_info() << "f = " << f;

            // add the change of force due to stiffness + Rayleigh damping
            mop.addMBKv(b, core::MatricesFactors::M(-d_rayleighMass.getValue()),
                        core::MatricesFactors::B(0),
                        core::MatricesFactors::K(m_dt * m_trapezoidFactor + d_rayleighStiffness.getValue())); // b =  f + ( rm M + (h+rs) K ) v

            // integration over a time step
            b.teq(m_dt);                                             // b = h(f + ( rm M + (h+rs) K ) v )
        }

        msg_info() << "b = " << b;

        mop.projectResponse(b);                                   // b is projected to the constrained space

        msg_info() << "projected b = " << b;
    }
}


SReal EulerImplicitIntegrationScheme::squaredNormRHS()
{

}


void EulerImplicitIntegrationScheme::solveLinearEquation()
{
    SCOPED_TIMER("MBKSolve");

    l_linearSolver->getLinearSystem()->setSystemSolution(x);
    l_linearSolver->getLinearSystem()->setRHS(b);
    l_linearSolver->solveSystem();
    l_linearSolver->getLinearSystem()->dispatchSystemSolution(x);
}


void EulerImplicitIntegrationScheme::updateVelocityAndPositionFromLinearSolution( SReal alpha)
{
    //TODO use alpha

    sofa::simulation::common::VectorOperations vop( m_params, this->getContext() );
    sofa::simulation::common::MechanicalOperations mop( m_params, this->getContext() );

    MultiVecCoord newPos(&vop, m_newPosID );
    MultiVecDeriv newVel(&vop, m_newVelID );
    MultiVecCoord pos(&vop, core::vec_id::write_access::position );
    MultiVecDeriv vel(&vop, core::vec_id::write_access::velocity );


    typedef core::behavior::BaseMechanicalState::VMultiOp VMultiOp;
    VMultiOp ops;
    if (d_firstOrder.getValue())
    {
        ops.resize(2);
        ops[0].first = newVel;
        ops[0].second.push_back(std::make_pair(x.id(),1.0));
        ops[1].first = newPos;
        ops[1].second.push_back(std::make_pair(pos.id(),1.0));
        ops[1].second.push_back(std::make_pair(newVel.id(),m_dt));
    }
    else
    {
        ops.resize(2);
        ops[0].first = newVel;
        ops[0].second.push_back(std::make_pair(vel.id(),1.0));
        ops[0].second.push_back(std::make_pair(x.id(),1.0));
        ops[1].first = newPos;
        ops[1].second.push_back(std::make_pair(pos.id(),1.0));
        ops[1].second.push_back(std::make_pair(newVel.id(),m_dt));
    }

    SCOPED_TIMER_VARNAME(updateVAndXTimer, "UpdateVAndX");
    vop.v_multiop(ops);

    mop.addSeparateGravity(m_dt, newVel);	// v += dt*g . Used if mass wants to add G separately from the other forces to v

    if (d_velocityDamping.getValue() != 0.0)
        newVel *= exp(-m_dt * d_velocityDamping.getValue());

}


SReal EulerImplicitIntegrationScheme::squaredNormDSolution()
{

}


SReal EulerImplicitIntegrationScheme::squaredLastSolution()
{

}



void EulerImplicitIntegrationScheme::reallocSolutionVector(sofa::simulation::common::VectorOperations* vop)
{
    x.realloc(vop, !d_threadSafeVisitor.getValue(), true,
        core::VecIdProperties{.label = "solution", .group = GetClass()->className});
}
void EulerImplicitIntegrationScheme::reallocRightHandSideVector(sofa::simulation::common::VectorOperations* vop)
{
    b.realloc(vop, !d_threadSafeVisitor.getValue(), true,
        core::VecIdProperties{.label = "RHS", .group = GetClass()->className});
}
void EulerImplicitIntegrationScheme::reallocResidualVector(sofa::simulation::common::VectorOperations* vop)
{
    m_residual.realloc(vop, !d_threadSafeVisitor.getValue(), true,
       core::VecIdProperties{.label = "residual", .group = GetClass()->className});
}

void registerEulerImplicitIntegrationScheme(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Time integrator using implicit backward Euler scheme.")
        .add< EulerImplicitIntegrationScheme >());
}

} // namespace sofa::component::odesolver::backward
