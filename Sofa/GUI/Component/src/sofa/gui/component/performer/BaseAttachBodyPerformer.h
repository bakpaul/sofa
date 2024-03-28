#pragma once

#include <sofa/core/objectmodel/BaseObject.h>


namespace sofa::gui::component::performer
{
struct BodyPicked;

class BaseAttachBodyPerformer
{
public:
    virtual sofa::core::objectmodel::BaseObject* getInteractionObject() = 0;
    virtual void clear() = 0;
    virtual bool start_partial(const BodyPicked& picked) = 0;
};
}