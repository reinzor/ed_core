#ifndef entity_h_
#define entity_h_

#include "ed/types.h"
#include "ed/uuid.h"

#include <tue/config/data.h>
#include <geolib/datatypes.h>

#include <boost/circular_buffer.hpp>
#include <ros/time.h>

#include "ed/property.h"
#include "ed/property_key.h"

#include "ed/logging.h"

namespace ed
{

class Entity
{

public:
    Entity(const UUID& id = generateID(), const TYPE& type = "");
    ~Entity();

    // ID
    static UUID generateID();
    inline const UUID& id() const { return id_; }

    // Type
    inline const TYPE& type() const { return type_; }
    inline void setType(const TYPE& type) { type_ = type; }

    // Shape
    inline geo::ShapeConstPtr shape() const { return shape_; }
    void setShape(const geo::ShapeConstPtr& shape);
    inline int shapeRevision() const{ return shape_ ? shape_revision_ : 0; }

    // Pose
    inline const geo::Pose3D& pose() const { return pose_; }
    inline void setPose(const geo::Pose3D& pose) { pose_ = pose; }

    // Data loaded from model
    inline const tue::config::DataConstPointer& data() const { return data_; }
    inline void setData(const tue::config::DataConstPointer& data) { data_ = data; }

    // Relations (what does this do?)
    inline void setRelationTo(Idx child_idx, Idx r_idx) { relations_to_[child_idx] = r_idx; }

    inline void setRelationFrom(Idx parent_idx, Idx r_idx) { relations_from_[parent_idx] = r_idx; }

    inline Idx relationTo(Idx child_idx) const
    {
        std::map<Idx, Idx>::const_iterator it = relations_to_.find(child_idx);
        if (it == relations_to_.end())
            return INVALID_IDX;
        return it->second;
    }

    inline Idx relationFrom(Idx parent_idx) const
    {
        std::map<Idx, Idx>::const_iterator it = relations_from_.find(parent_idx);
        if (it == relations_from_.end())
            return INVALID_IDX;
        return it->second;
    }

    const std::map<Idx, Idx>& relationsFrom() const { return relations_from_; }

    const std::map<Idx, Idx>& relationsTo() const { return relations_to_; }

    template<typename T>
    const T* property(const PropertyKey<T>& key) const
    {
        std::map<Idx, Property>::const_iterator it = properties_.find(key.idx);
        if (it == properties_.end())
            return 0;

        const Property& p = it->second;

        try
        {
            return &p.value.getValue<T>();
        }
        catch (std::bad_cast& e)
        {
            return 0;
        }
    }

    void setProperty(Idx idx, const Property& p)
    {
        std::map<Idx, Property>::iterator it = properties_.find(idx);
        if (it == properties_.end())
        {
            Property& p_new = properties_[idx];
            p_new.entry = p.entry;
            p_new.revision = p.revision;
            p_new.value = p.value;
        }
        else
        {
            Property& p_new = it->second;
            p_new.value = p.value;
            p_new.revision = p.revision;
        }

        if (revision_ < p.revision)
            revision_ = p.revision;
    }

    const std::map<Idx, Property>& properties() const { return properties_; }

    unsigned long revision() const { return revision_; }

    void setRevision(unsigned long revision) { revision_ = revision; }

private:

    // ID
    UUID id_;

    // Revision
    unsigned long revision_;

    // type
    TYPE type_;

    // Shape
    geo::ShapeConstPtr shape_;
    int shape_revision_;

    // Absolute pose
    geo::Pose3D pose_;

    // Data loaded from the model
    tue::config::DataConstPointer data_;

    // Relations
    std::map<Idx, Idx> relations_from_;
    std::map<Idx, Idx> relations_to_;

    // Generic property map
    std::map<Idx, Property> properties_;

};

}

#endif
