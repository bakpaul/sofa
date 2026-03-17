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

#include <sofa/core/objectmodel/BaseComponent.h>
#include <sofa/core/objectmodel/BaseNode.h>

namespace sofa::core::behavior
{

/**
 *  \brief Component responsible for main animation algorithms, managing how
 *  and when mechanical computation happens in one animation step.
 *
 *  This class can optionally replace the default computation scheme of computing
 *  collisions then doing an integration step.
 *
 *  Note that it is in a preliminary stage, hence its functionalities and API will
 *  certainly change soon.
 *
 */
class SOFA_CORE_API BaseTimeIntegrator : public virtual objectmodel::BaseObject
{

public:
    SOFA_ABSTRACT_CLASS(BaseTimeIntegrator, objectmodel::BaseObject);
    SOFA_BASE_CAST_IMPLEMENTATION(BaseTimeIntegrator)

    // the node where the loop will start processing.
    SingleLink<BaseTimeIntegrator, core::objectmodel::BaseNode, BaseLink::FLAG_STOREPATH> l_node;

protected:
    BaseTimeIntegrator();
    ~BaseTimeIntegrator() override;

public:
    void init() override;

    /// Main computation method.
    ///
    /// Specify and execute all computations for integrating a timestep, such
    /// as one or more collisions and integrations stages.
    virtual void integrate(const core::ExecParams* params, SReal dt) = 0;

    bool insertInNode( objectmodel::BaseNode* node ) override;
    bool removeInNode( objectmodel::BaseNode* node ) override;
};

} // namespace sofa::core::behavior
