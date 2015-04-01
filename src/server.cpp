#include "ed/server.h"

#include "ed/entity.h"

#include <tue/filesystem/path.h>

#include "ed/plugin.h"
#include "ed/plugin_container.h"
#include "ed/world_model.h"

#include <tue/config/loaders/yaml.h>

#include <boost/make_shared.hpp>

#include "ed/error_context.h"

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Server::Server() : world_model_(new WorldModel(&property_key_db_))
{
}

// ----------------------------------------------------------------------------------------------------

Server::~Server()
{
}

// ----------------------------------------------------------------------------------------------------

std::string Server::getFullLibraryPath(const std::string& lib)
{
    for(std::vector<std::string>::const_iterator it = plugin_paths_.begin(); it != plugin_paths_.end(); ++it)
    {
        std::string lib_file_test = *it + "/" + lib;
        if (tue::filesystem::Path(lib_file_test).exists())
        {
            return lib_file_test;
        }
    }

    return "";
}

// ----------------------------------------------------------------------------------------------------

void Server::configure(tue::Configuration& config)
{
    ErrorContext errc("Server", "configure");

    // Unload all previously loaded plugins
    plugin_containers_.clear();

    if (config.readArray("plugins"))
    {
        while(config.nextArrayItem())
        {
            std::string name;
            if (!config.value("name", name))
                continue;

            std::string lib;
            if (!config.value("lib", lib))
                continue;

            loadPlugin(name, lib, config);

        } // end iterate plugins

        config.endArray();
    }

    // Initialize profiler
    profiler_.setName("ed");
}

// ----------------------------------------------------------------------------------------------------

void Server::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

PluginContainerPtr Server::loadPlugin(const std::string& plugin_name, const std::string& lib_file, tue::Configuration config)
{    
    ErrorContext errc("Server", "loadPlugin");

    config.setErrorContext("While loading plugin '" + plugin_name + "': ");

    if (lib_file.empty())
    {
        config.addError("Empty library file given.");
        return PluginContainerPtr();
    }

    std::string full_lib_file = lib_file;
    if (lib_file[0] != '/')
    {
        // library file is relative
        full_lib_file = getFullLibraryPath(lib_file);
        if (full_lib_file.empty())
        {
            config.addError("Could not find plugin '" + lib_file + "'.");
            return PluginContainerPtr();
        }
    }

    if (!tue::filesystem::Path(full_lib_file).exists())
    {
        config.addError("Could not find plugin '" + full_lib_file + "'.");
        return PluginContainerPtr();
    }

    // Create a plugin container
    PluginContainerPtr container(new PluginContainer(world_model_));

    InitData init(property_key_db_, config);

    // Load the plugin
    if (!container->loadPlugin(plugin_name, full_lib_file, init))
        return PluginContainerPtr();

    // Add the plugin container
    plugin_containers_.push_back(container);

    // Start the plugin
    container->runThreaded();

    return container;
}

// ----------------------------------------------------------------------------------------------------

void Server::stepPlugins()
{   
    ErrorContext errc("Server", "stepPlugins");

    WorldModelPtr new_world_model;

    // collect and apply all update requests
    std::vector<PluginContainerPtr> plugins_with_requests;
    for(std::vector<PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = *it;

        if (c->updateRequest())
        {
            if (!new_world_model)
            {
                // Create world model copy (shallow)
                new_world_model = boost::make_shared<WorldModel>(*world_model_);
            }

            new_world_model->update(*c->updateRequest());
            plugins_with_requests.push_back(c);

            // Temporarily for Javier
            for(std::vector<PluginContainerPtr>::iterator it2 = plugin_containers_.begin(); it2 != plugin_containers_.end(); ++it2)
            {
                PluginContainerPtr c2 = *it2;
                c2->addDelta(c->updateRequest());
            }
        }
    }

    if (new_world_model)
    {
        // Set the new (updated) world
        for(std::vector<PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
        {
            const PluginContainerPtr& c = *it;
            c->setWorld(new_world_model);
        }

        world_model_ = new_world_model;

        // Clear the requests of all plugins that had requests (which flags them to continue processing)
        for(std::vector<PluginContainerPtr>::iterator it = plugins_with_requests.begin(); it != plugins_with_requests.end(); ++it)
        {
            PluginContainerPtr c = *it;
            c->clearUpdateRequest();
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void Server::update(const ed::UpdateRequest& req)
{
    // Create world model copy (shallow)
    WorldModelPtr new_world_model = boost::make_shared<WorldModel>(*world_model_);

    // Update the world model
    new_world_model->update(req);

    // Notify all plugins of the updated world model
    for(std::vector<PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = *it;
        c->setWorld(new_world_model);
    }

    // Set the new (updated) world
    world_model_ = new_world_model;
}

// ----------------------------------------------------------------------------------------------------


// ----------------------------------------------------------------------------------------------------

void Server::publishStatistics() const
{
    std::stringstream s;

    s << "[plugins]" << std::endl;
    for(std::vector<PluginContainerPtr>::const_iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        const PluginContainerPtr& p = *it;

        // Calculate CPU usage percentage
        double cpu_perc = p->totalProcessingTime() * 100 / p->totalRunningTime();

        s << "    " << p->name() << ": " << cpu_perc << " % (" << p->loopFrequency() << " hz)" << std::endl;
    }

    std::cout << s.str() << std::endl;
}

}
