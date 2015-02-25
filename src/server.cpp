#include "ed/server.h"

#include "ed/sensor_modules/kinect.h"
#include "ed/entity.h"
#include "ed/measurement.h"
#include "ed/helpers/visualization.h"
#include "ed/helpers/depth_data_processing.h"

#include <geolib/Box.h>

// Storing measurements to disk
#include "ed/io/filesystem/write.h"

#include <tue/profiling/scoped_timer.h>

#include <tue/filesystem/path.h>

#include "ed/plugin.h"
#include "ed/plugin_container.h"
#include "ed/world_model.h"

#include <tue/config/loaders/yaml.h>

#include <boost/make_shared.hpp>

#include <std_msgs/String.h>

#include "ed/serialization/serialization.h"
#include <tue/config/writer.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Server::Server() : world_model_(new WorldModel)
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

void Server::configure(tue::Configuration& config, bool reconfigure)
{
    // For now, do not reconfigure perception
    if (!reconfigure && config.readGroup("perception"))
    {
        perception_.configure(config.limitScope());
        config.endGroup();
    }

    if (config.readArray("sensors"))
    {
        while(config.nextArrayItem())
        {
            std::string name;
            if (config.value("name", name))
            {
                std::map<std::string, SensorModulePtr>::iterator it_sensor = sensors_.find(name);

                if (it_sensor == sensors_.end())
                {
                    // Sensor module does not yet exist. Determine the type and create a sensor
                    // module accordingly.

                    std::string type;
                    if (config.value("type", type))
                    {
                        if (type == "kinect")
                        {
                            SensorModulePtr sensor_mod(new Kinect(tf_listener_));
                            sensor_mod->configure(config);
                            sensors_[name] = sensor_mod;
                        }
                    }
                }
                else
                {
                    // Sensor module exists, so reconfigure
                    it_sensor->second->configure(config, true);
                }
            }
        }

        config.endArray();
    }

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

    config.value("visualize", visualize_);

    // Initialize profiler
    profiler_.setName("ed");
    pub_profile_.initialize(profiler_);

    if (config.value("world_name", world_name_))
        initializeWorld();
    else
        std::cout << "No world specified in parameter file, cannot initialize world" << std::endl;

    if (pub_stats_.getTopic() == "")
    {
        ros::NodeHandle nh;
        pub_stats_ = nh.advertise<std_msgs::String>("/ed/stats", 10);
    }

}

// ----------------------------------------------------------------------------------------------------

void Server::initialize()
{
    // Initialize visualization
    if (visualize_) {
        ros::NodeHandle nh;
        vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("world_model",0,false);
    }
}

// ----------------------------------------------------------------------------------------------------

void Server::reset()
{
    world_model_.reset(new WorldModel);

    initializeWorld();
}

// ----------------------------------------------------------------------------------------------------

PluginContainerPtr Server::loadPlugin(const std::string& plugin_name, const std::string& lib_file, tue::Configuration config)
{    
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
    PluginContainerPtr container(new PluginContainer());

    // Load the plugin
    if (!container->loadPlugin(plugin_name, full_lib_file, config))
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
            for(std::vector<PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
            {
                PluginContainerPtr c = *it;
                c->plugin()->updateRequestCallback(*c->updateRequest());
            }
        }
    }

    if (new_world_model)
    {
        // Set the new (updated) world
        for(std::vector<PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
        {
            PluginContainerPtr c = *it;
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

void Server::update()
{
    tue::ScopedTimer t(profiler_, "ed");

    // Create world model copy (shallow)
    WorldModelPtr new_world_model = boost::make_shared<WorldModel>(*world_model_);

    // Sensor Measurements integration (Loop over all sensors and integrate the measurements)
    {
        tue::ScopedTimer t(profiler_, "sensor integration");
        for (std::map<std::string, SensorModulePtr>::const_iterator it = sensors_.begin(); it != sensors_.end(); ++it) {
            UpdateRequest req;
            it->second->update(new_world_model, req);
            new_world_model->update(req);
        }
    }

    // Visualize the world model
    if (visualize_)
    {
        tue::ScopedTimer t(profiler_, "visualization");
        helpers::visualization::publishWorldModelVisualizationMarkerArray(*world_model_, vis_pub_);
    }

    // Perception update (make soup of the entity measurements)
    {
        tue::ScopedTimer t(profiler_, "perception");

        UpdateRequest req;
        perception_.update(new_world_model, req);
        new_world_model->update(req);
    }

    // Look if we can merge some not updates entities
    {
        tue::ScopedTimer t(profiler_, "merge entities");
        mergeEntities(*new_world_model, 5.0, 0.5);
    }

    // Notify all plugins of the updated world model
    for(std::vector<PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = *it;
        c->setWorld(new_world_model);
    }

    // Set the new (updated) world
    world_model_ = new_world_model;

    pub_profile_.publish();
}

// ----------------------------------------------------------------------------------------------------

void Server::update(const std::string& update_str, std::string& error)
{
    tue::ScopedTimer t(profiler_, "ed");


    // convert update string to update request
    tue::Configuration cfg;

    tue::config::loadFromYAMLString(update_str, cfg);

    if (cfg.hasError())
    {
        error = cfg.error();
        return;
    }

    // - - - - - - - - - Create update request from cfg - - - - - - - - -

    UpdateRequest req;

    if (cfg.readArray("entities"))
    {
        while(cfg.nextArrayItem())
        {
            std::string id;
            if (!cfg.value("id", id))
                continue;

            if (cfg.readGroup("pose"))
            {
                geo::Pose3D pose;

                if (!cfg.value("x", pose.t.x) || !cfg.value("y", pose.t.y) || !cfg.value("z", pose.t.z))
                    continue;

                double rx = 0, ry = 0, rz = 0;
                cfg.value("rx", rx, tue::OPTIONAL);
                cfg.value("ry", ry, tue::OPTIONAL);
                cfg.value("rz", rz, tue::OPTIONAL);

                pose.R.setRPY(rx, ry, rz);

                req.setPose(id, pose);

                cfg.endGroup();
            }
        }

        cfg.endArray();
    }

    // Check for errors
    if (cfg.hasError())
    {
        error = cfg.error();
        return;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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

void Server::initializeWorld()
{
    ed::UpdateRequest req;
    if (!model_loader_.create(world_name_, world_name_, req))
        return;

    // Create world model copy (shallow)
    WorldModelPtr new_world_model = boost::make_shared<WorldModel>(*world_model_);

    new_world_model->update(req);

    // Temporarily for Javier
    for(std::vector<PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = *it;
        c->plugin()->updateRequestCallback(req);
    }

    world_model_ = new_world_model;
}

// ----------------------------------------------------------------------------------------------------

void Server::storeEntityMeasurements(const std::string& path) const
{
    for(WorldModel::const_iterator it = world_model_->begin(); it != world_model_->end(); ++it)
    {
        const EntityConstPtr& e = *it;
        MeasurementConstPtr msr = e->lastMeasurement();
        if (!msr)
            continue;

        std::string filename = path + "/" + e->id().str();
        if (!write(filename, *msr))
        {
            std::cout << "Saving measurement failed." << std::endl;
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void Server::mergeEntities(WorldModel& world, double not_updated_time, double overlap_fraction)
{
    std::set<UUID> merge_target_ids;

    UpdateRequest req;

    // Iter over all entities and check if the current_time - last_update_time > not_updated_time
    for (WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const EntityConstPtr& e = *it;

        // skip if e is null, theres some bug somewhere
        if (e == NULL) continue;

        if (!e->lastMeasurement())
            continue;

        if (e->shape() || merge_target_ids.find(e->id()) != merge_target_ids.end() )
            continue;

        if ( ros::Time::now().toSec() - e->lastMeasurement()->timestamp() > not_updated_time )
        {
            // Try to merge with other polygons (except for itself)
            for (WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it)
            {
                const EntityConstPtr& e_target = *e_it;
                const UUID& id2 = e_target->id();

                // Skip self
                if (id2 == e->id())
                    continue;

                MeasurementConstPtr last_m = e_target->lastMeasurement();

                if (!last_m)
                    continue;

                if (ros::Time::now().toSec() - last_m->timestamp() < not_updated_time)
                    continue;

                double overlap_factor;
                bool collision = helpers::ddp::polygonCollisionCheck(e_target->convexHull(),
                                                                     e->convexHull(),
                                                                     overlap_factor);

                if (collision && overlap_factor > 0.5)
                { //! TODO: NEEDS REVISION
                    req.removeEntity(e->id());

                    ConvexHull2D convex_hull_target = e_target->convexHull();
                    helpers::ddp::add2DConvexHull(e->convexHull(), convex_hull_target);

                    // Update the convex hull
                    req.setConvexHull(e_target->id(), convex_hull_target);

                    // Update the best measurement
                    MeasurementConstPtr best_measurement = e->bestMeasurement();
                    if (best_measurement)
                        req.addMeasurement(e_target->id(), best_measurement);

                    merge_target_ids.insert(e_target->id());
                    break;
                }
            }
        }
    }

    world.update(req);
}


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


    std_msgs::String msg;
    msg.data = s.str();

    pub_stats_.publish(msg);

//    // TEMP
//    tue::config::DataPointer data;
//    tue::config::Writer w(data);
//    ed::serialize(*world_model_, w);

//    std::cout << data << std::endl;
}

}
