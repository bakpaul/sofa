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
#include <sofa/core/ConstraintParams.h>
#include <sofa/simulation/DefaultAnimationLoop.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/Node.h>
#include <sofa/simulation/UpdateContextVisitor.h>
#include <sofa/simulation/UpdateMappingVisitor.h>
#include <sofa/simulation/PropagateEventVisitor.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/UpdateMappingEndEvent.h>
#include <sofa/simulation/UpdateBoundingBoxVisitor.h>

#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/helper/AdvancedTimer.h>

#include <sofa/core/visual/VisualParams.h>

#include <sofa/simulation/task/TaskScheduler.h>


namespace sofa::simulation
{

void registerDefaultAnimationLoop(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Simulation loop, created by default when the user does not define one in the scene. This loop first computes the collision detection and then solves the physics.")
        .add<DefaultAnimationLoop>()
        .addDocumentationURL(std::string(sofa::SOFA_DOCUMENTATION_URL) + std::string("components/animationloops/defaultanimationloop/"))
        .addDescription(R"(
This loop triggers the following steps:
- build and solve all linear systems in the scene : collision and time integration to compute the new values of the dofs
- update the context (dt++)
- update the mappings
- update the bounding box (volume covering all objects of the scene))"));
}

DefaultAnimationLoop::DefaultAnimationLoop(simulation::Node* _m_node)
    : Inherit()
{
    SOFA_UNUSED(_m_node);
}

DefaultAnimationLoop::~DefaultAnimationLoop() = default;

using DefaultTimeIntegrator = sofa::simulation::LinearTimeIntegrator;

void DefaultAnimationLoop::init()
{
    Inherit::init();
    if (!l_node)
    {
        this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
    }
    else
    {
        this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
    }


    if (!l_baseTimeIntegrator)
    {
        l_baseTimeIntegrator.set(this->getContext()->get<sofa::core::behavior::BaseTimeIntegrator>(core::objectmodel::BaseContext::SearchDown));
        if (!l_baseTimeIntegrator)
        {
            if (const auto timeIntegrator = sofa::core::objectmodel::New<DefaultTimeIntegrator>())
            {
                getContext()->addObject(timeIntegrator);
                timeIntegrator->setName( this->getContext()->getNameHelper().resolveName(timeIntegrator->getClassName(), {}));
                timeIntegrator->f_printLog.setValue(this->f_printLog.getValue());
                l_baseTimeIntegrator.set(timeIntegrator.get());

                msg_warning() << "A ConstraintSolver is required by " << this->getClassName() << " but has not been found:"
                    " a default " << timeIntegrator->getClassName() << " is automatically added in the scene for you. To remove this warning, add"
                    " a ConstraintSolver in the scene. The list of available constraint solvers is: "
                    << core::ObjectFactory::getInstance()->listClassesDerivedFrom<sofa::core::behavior::BaseTimeIntegrator>();
            }
            else
            {
                msg_fatal() << "A ConstraintSolver is required by " << this->getClassName() << " but has not been found:"
                    " a default " << DefaultTimeIntegrator::GetClass()->className << " could not be automatically added in the scene. To remove this error, add"
                    " a ConstraintSolver in the scene. The list of available constraint solvers is: "
                    << core::ObjectFactory::getInstance()->listClassesDerivedFrom<sofa::core::behavior::BaseTimeIntegrator>();
                this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
                return;
            }
        }
    }
}

void DefaultAnimationLoop::setNode(simulation::Node* n)
{
    l_node.set(n);
}


void DefaultAnimationLoop::updateSimulationContext(const core::ExecParams* params, const SReal dt, const SReal startTime) const
{
    SCOPED_TIMER("UpdateSimulationContextVisitor");
    m_node->setTime(startTime + dt);
    m_node->execute<UpdateSimulationContextVisitor>(params);
}

void DefaultAnimationLoop::propagateAnimateEndEvent(const core::ExecParams* params, const SReal dt) const
{
    AnimateEndEvent ev(dt);
    PropagateEventVisitor propagateEventVisitor(params, &ev);
    m_node->execute(propagateEventVisitor);
}

void DefaultAnimationLoop::updateMapping(const core::ExecParams* params, const SReal dt) const
{
    SCOPED_TIMER("UpdateMapping");
    //Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
    m_node->execute<UpdateMappingVisitor>(params);
    {
        UpdateMappingEndEvent ev(dt);
        PropagateEventVisitor propagateEventVisitor(params, &ev);
        m_node->execute(propagateEventVisitor);
    }
}

void DefaultAnimationLoop::computeBoundingBox(const core::ExecParams* params) const
{
    if (d_computeBoundingBox.getValue())
    {
        SCOPED_TIMER("UpdateBBox");
        m_node->execute<UpdateBoundingBoxVisitor>(params);
    }
}

void DefaultAnimationLoop::propagateAnimateBeginEvent(const core::ExecParams* params, const SReal dt) const
{
    AnimateBeginEvent ev(dt);
    PropagateEventVisitor act(params, &ev);
    m_node->execute(act);
}

void DefaultAnimationLoop::step(const core::ExecParams* params, SReal dt)
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
    {
        return;
    }

    m_node = dynamic_cast<sofa::simulation::Node*>(this->l_node.get());
    assert(m_node);

    if (dt == 0_sreal)
    {
        dt = m_node->getDt();
    }

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("Step");
#endif

    propagateAnimateBeginEvent(params, dt);
    l_baseTimeIntegrator->integrate(params, dt);
    updateSimulationContext(params, dt, m_node->getTime());
    propagateAnimateEndEvent(params, dt);

    updateMapping(params, dt);
    computeBoundingBox(params);

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("Step");
#endif
}


} // namespace sofa
