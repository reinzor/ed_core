#ifndef ED_SERVER_H_
#define ED_SERVER_H_

#include "ed/types.h"

#include <tue/profiling/profiler.h>

#include <ed/models/model_loader.h>

#include <queue>

#include <tue/config/configuration.h>

namespace ed
{

class Server
{

public:
    Server();
    virtual ~Server();    

    void configure(tue::Configuration& config);

    void initialize();

    void update(const ed::UpdateRequest& req);

    WorldModelConstPtr world_model() const { return world_model_; }

    void addPluginPath(const std::string& path) { plugin_paths_.push_back(path); }

    PluginContainerPtr loadPlugin(const std::string& plugin_name, const std::string& lib_file, tue::Configuration config);

    void stepPlugins();

    void publishStatistics() const;

private:

    //! World model datastructure
    WorldModelConstPtr world_model_;

    std::queue<UpdateRequest> update_requests_;

    //! Plugins
    std::vector<std::string> plugin_paths_;
    std::vector<PluginContainerPtr> plugin_containers_;

    //! Profiling
    tue::Profiler profiler_;

    std::string getFullLibraryPath(const std::string& lib);
};

}

#endif
