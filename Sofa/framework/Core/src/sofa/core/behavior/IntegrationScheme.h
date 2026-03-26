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

#include <sofa/core/behavior/MultiVec.h>

namespace sofa::core::behavior
{

/**
 *  \brief Component responsible for timestep integration, i.e. advancing the state from time t to t+dt.
 *
 *  This class currently control both the integration scheme (explicit,
 *  implicit, static, etc).
 *
 *  While all computations required to do the integration step are handled by
 *  this object, they should not be implemented directly in it, but instead
 *  the solver propagates orders (or Visitor) to the other components in the
 *  scenegraph that will locally execute them. This allows for greater
 *  flexibility (the solver can just ask for the forces to be computed without
 *  knowing what type of forces are present), as well as performances
 *  (some computations can be executed in parallel).
 *
 */
class SOFA_CORE_API IntegrationScheme : public virtual objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(IntegrationScheme, objectmodel::BaseObject);
    SOFA_BASE_CAST_IMPLEMENTATION(IntegrationScheme)

protected:
    IntegrationScheme();
    ~IntegrationScheme() override;

public:

    virtual void setupIntegrationStep(const core::ExecParams* params, SReal dt, sofa::core::MultiVecCoordId xResult, sofa::core::MultiVecDerivId vResult) = 0;

    /**
     * Compute the system matrix.
     */
    virtual void computeLHS() = 0;

     /**
     * compute the current RHS.
     */
    virtual void computeRHS() = 0;


    /**
     * Returns the squared norm of the last evaluation of the RHS
     */
    virtual SReal squaredNormRHS() = 0;


    /**
     * Solve the linear equation from a Newton iteration, i.e. it computes (x^{i+1}-x^i).
     */
    virtual void solveLinearEquation() = 0;

    /**
     * Once (x^{i+1}-x^i) has been computed, the result is used internally to update the current
     * guess. It computes x^{i+1} += alpha * dx, where dx is the result of the linear system. It is
     * not necessary to share the result with the Newton-Raphson method.
     */
    virtual void updateVelocityAndPositionFromLinearSolution(SReal alpha) = 0;

    /**
     * Compute ||x^{i+1}-x^i||^2
     */
    virtual SReal squaredNormDSolution() = 0;

    /**
     * Compute ||x^{i+1}||^2
     */
    virtual SReal squaredLastSolution() = 0;


    /// By default the integration scheme are meant to solve the equations in velocity
    virtual SReal getVelocityIntegrationFactor() const
    {
        return 1.0;
    }

    virtual SReal getPositionIntegrationFactor() const
    {
        return this->getContext()->getDt();
    }


    bool insertInNode( objectmodel::BaseNode* node ) override;
    bool removeInNode( objectmodel::BaseNode* node ) override;

protected:

    const core::ExecParams* m_params;
    SReal m_dt;
    sofa::core::MultiVecCoordId m_xResult;
    sofa::core::MultiVecDerivId m_vResult;

};

} // namespace sofa::core::behavior
