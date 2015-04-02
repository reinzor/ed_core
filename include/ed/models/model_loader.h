#ifndef ED_MODEL_LOADER_H_
#define ED_MODEL_LOADER_H_

#include "ed/world_model/uuid.h"

#include <map>
#include <geolib/datatypes.h>
#include <tue/config/data_pointer.h>

namespace ed
{

namespace models
{

class ModelLoader
{

public:

    ModelLoader();

    ~ModelLoader();

    void addModelPath(const std::string& path) { model_paths_.push_back(path); }

    bool create(const UUID& id, const std::string& type, UpdateRequest& req, std::stringstream& error);

    bool create(const tue::config::DataConstPointer& data, UpdateRequest& req, std::stringstream& error);

    bool create(const tue::config::DataConstPointer& data, const UUID& id_opt, const UUID& parent_id,
                UpdateRequest& req, std::stringstream& error, const std::string& model_path = "");

    bool exists(const std::string& type) const;

private:

    // Model paths
    std::vector<std::string> model_paths_;

    // Model name to model data
    std::map<std::string, tue::config::DataConstPointer> model_cache_;

    // Shape filename to shape
    std::map<std::string, geo::ShapePtr> shape_cache_;

    tue::config::DataConstPointer loadModelData(const std::string& type, std::string& model_path_str, std::stringstream& error);

};

} // end namespace models

} // end namespace ed

#endif
