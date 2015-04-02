#ifndef ED_UPDATE_REQUEST_H_
#define ED_UPDATE_REQUEST_H_

#include "ed/types.h"
#include "ed/uuid.h"
#include "ed/property.h"
#include "ed/property_key.h"
#include "ed/property_key_db.h"

#include <tue/config/data_pointer.h>

#include <map>
#include <vector>
#include <geolib/datatypes.h>

namespace ed
{

class UpdateRequest
{

public:

    UpdateRequest() {}

    // SHAPES

    std::map<UUID, geo::ShapeConstPtr> shapes;
    void setShape(const UUID& id, const geo::ShapeConstPtr& shape) { shapes[id] = shape; flagUpdated(id); }

    // TYPES

    std::map<UUID, std::string> types;
    void setType(const UUID& id, const std::string& type) { types[id] = type; flagUpdated(id); }


    // POSES

    std::map<UUID, geo::Pose3D> poses;
    void setPose(const UUID& id, const geo::Pose3D& pose) { poses[id] = pose; flagUpdated(id); }


    // RELATIONS

    std::map<UUID, std::map<UUID, RelationConstPtr> > relations;
    void setRelation(const UUID& id1, const UUID& id2, const RelationConstPtr& r) { relations[id1][id2] = r; flagUpdated(id1); flagUpdated(id2);}

    // DATA

    std::map<UUID, tue::config::DataConstPointer> datas;

    void addData(const UUID& id, const tue::config::DataConstPointer& data)
    {
        std::map<UUID, tue::config::DataConstPointer>::iterator it = datas.find(id);
        if (it == datas.end())
        {
            datas[id] = data;
        }
        else
        {
            tue::config::DataPointer data_total;
            data_total.add(it->second);
            data_total.add(data);

            it->second = data_total;
        }

        flagUpdated(id);
    }

    std::map<UUID, std::map<Idx, Property> > properties;

    template<typename T>
    void setProperty(const UUID& id, const PropertyKey<T>& key, const T& value)
    {
        if (!key.valid())
            return;

        Property& p = properties[id][key.idx];
        p.entry = key.entry;
        p.value = value;
        flagUpdated(id);
    }

    void setProperty(const UUID& id, const PropertyKeyDBEntry* entry, const ed::Variant& v)
    {
        Property& p = properties[id][entry->idx];
        p.entry = entry;
        p.value = v;
        flagUpdated(id);
    }


    // REMOVED ENTITIES

    std::set<UUID> removed_entities;

    void removeEntity(const UUID& id) { removed_entities.insert(id); flagUpdated(id); }



    // UPDATED (AND REMOVED) ENTITIES

    std::set<UUID> updated_entities;

    bool empty() const { return updated_entities.empty(); }


private:

    void flagUpdated(const ed::UUID& id) { updated_entities.insert(id); }

};

}

#endif
