#ifndef ED_SERVER_H_
#define ED_SERVER_H_

#include "ed/types.h"

#include <tue/profiling/profiler.h>
#include <tue/profiling/ros/profile_publisher.h>

#include <ed/models/model_loader.h>

#include "ed/property_key_db.h"

#include <queue>

#include <tue/config/configuration.h>

namespace ed
{

class Server
{

public:
    Server();
    virtual ~Server();    

    void configure(tue::Configuration& config, bool reconfigure = false);

    void initialize();

    void reset();

    void update(const ed::UpdateRequest& req);

    void update(const std::string& update_str, std::string& error);

    WorldModelConstPtr world_model() const { return world_model_; }

    void addPluginPath(const std::string& path) { plugin_paths_.push_back(path); }

    PluginContainerPtr loadPlugin(const std::string& plugin_name, const std::string& lib_file, tue::Configuration config);

    void stepPlugins();

    void publishStatistics() const;

    const PropertyKeyDBEntry* getPropertyKeyDBEntry(const std::string& name) const
    {
        return property_key_db_.getPropertyKeyDBEntry(name);
    }

private:

    // World model datastructure
    WorldModelConstPtr world_model_;

    //! World name
    std::string world_name_;

    std::queue<UpdateRequest> update_requests_;

    void initializeWorld();

    //! Model loading
    models::ModelLoader model_loader_;

    //! Property Key DB
    PropertyKeyDB property_key_db_;

    //! Plugins
    std::vector<std::string> plugin_paths_;
    std::vector<PluginContainerPtr> plugin_containers_;

    //! Profiling
    tue::Profiler profiler_;

    std::string getFullLibraryPath(const std::string& lib);
};

}

#endif
