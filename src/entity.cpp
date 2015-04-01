#include "ed/entity.h"

#include <geolib/Shape.h>
#include <geolib/Mesh.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Entity::Entity(const UUID& id, const TYPE& type) :
    id_(id),
    revision_(0),
    type_(type),
    shape_revision_(0),
    pose_(geo::Pose3D::identity())
{
}

// ----------------------------------------------------------------------------------------------------

Entity::~Entity()
{
//    std::cout << "Removing entity with ID: " << id_ << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void Entity::setShape(const geo::ShapeConstPtr& shape)
{
    if (shape_ != shape)
    {
        ++shape_revision_;
        shape_ = shape;
    }
}

// ----------------------------------------------------------------------------------------------------

UUID Entity::generateID() {
    static const char alphanum[] =
        "0123456789"
        "abcdef";

    std::string s;
    for (int i = 0; i < 32; ++i) {
        int n = rand() / (RAND_MAX / (sizeof(alphanum) - 1) + 1);
        s += alphanum[n];
    }

    return UUID(s);
}

}
