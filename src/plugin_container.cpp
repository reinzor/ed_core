#include "ed/plugin_container.h"

#include "ed/plugin/plugin.h"
#include "ed/time/rate.h"

#include <ed/logging/error_context.h>

namespace ed
{

// --------------------------------------------------------------------------------

PluginContainer::PluginContainer(WorldModelConstPtr world_model)
    : class_loader_(0), stop_(false), cycle_duration_(0.1), loop_frequency_(10), step_finished_(true), t_last_update_(0),
      world_current_(world_model), total_process_time_sec_(0)
{
    timer_.start();
}

// --------------------------------------------------------------------------------

PluginContainer::~PluginContainer()
{
    stop_ = true;

//    if (thread_)
//        thread_->join();

    plugin_.reset();
    delete class_loader_;
}

// --------------------------------------------------------------------------------

PluginPtr PluginContainer::loadPlugin(const std::string plugin_name, const std::string& lib_filename, tue::Configuration& config)
{
    // Load the library
    delete class_loader_;
    class_loader_ = new class_loader::ClassLoader(lib_filename);

    // Create plugin
    class_loader_->loadLibrary();
    std::vector<std::string> classes = class_loader_->getAvailableClasses<ed::Plugin>();

    if (classes.empty())
    {
        config.addError("Could not find any plugins in '" + class_loader_->getLibraryPath() + "'.");
    } else if (classes.size() > 1)
    {
        config.addError("Multiple plugins registered in '" + class_loader_->getLibraryPath() + "'.");
    } else
    {
        plugin_ = class_loader_->createInstance<Plugin>(classes.front());
        if (plugin_)
        {
            double freq = 10; // default

            name_ = plugin_name;
            plugin_->name_ = plugin_name;

            // Read optional frequency
            config.value("frequency", freq, tue::OPTIONAL);

            if (config.readGroup("parameters"))
            {
                tue::Configuration scoped_config = config.limitScope();

                plugin_->configure(scoped_config);

                // Read optional frequency (inside parameters is obsolete)
                if (config.value("frequency", freq, tue::OPTIONAL))
                {
                    std::cout << "[ED]: Warning while loading plugin '" << name_ << "': please specify parameter 'frequency' outside 'parameters'." << std::endl;
                }

                config.endGroup();
            }
            else
            {
                // No parameter available
                tue::Configuration scoped_config;

                plugin_->configure(scoped_config);

                if (scoped_config.hasError())
                    config.addError(scoped_config.error());
            }

            // If there was an error during configuration, do not start plugin
            if (config.hasError())
                return PluginPtr();

            // Initialize the plugin
            plugin_->initialize();

            // Set plugin loop frequency
            setLoopFrequency(freq);

            return plugin_;
        }
    }

    return PluginPtr();
}

// --------------------------------------------------------------------------------

void PluginContainer::runThreaded()
{
    thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&PluginContainer::run, this)));
    pthread_setname_np(thread_->native_handle(), name_.c_str());
}

// --------------------------------------------------------------------------------

void PluginContainer::run()
{
    total_timer_.start();

    Rate r(loop_frequency_);
    while(!stop_)
    {
        step();
        r.sleep();
    }
}

// --------------------------------------------------------------------------------

void PluginContainer::step()
{
    // If we still have an update_request, it means the request is not yet handled,
    // so we have to skip this cycle (and wait until the world model has handled it)
    {
        boost::lock_guard<boost::mutex> lg(mutex_update_request_);
        if (update_request_)
            return;
    }

    // Check if there is a new world. If so replace the current one with the new one
    {
        boost::lock_guard<boost::mutex> lg(mutex_world_);
        if (world_new_)
        {
            world_current_ = world_new_;
            world_new_.reset();
        }
    }

    if (world_current_)
    {
        UpdateRequestPtr update_request(new UpdateRequest);

        tue::Timer timer;
        timer.start();

        // Old
        {
            ed::log::ErrorContext errc("Plugin:", name().c_str());

            plugin_->process(*world_current_, *update_request);
        }

        timer.stop();
        total_process_time_sec_ += timer.getElapsedTimeInSec();

        // If the received update_request was not empty, set it
        if (!update_request->empty())
            update_request_ = update_request;
    }
}

// --------------------------------------------------------------------------------

void PluginContainer::stop()
{
    stop_ = true;
    thread_->join();
    plugin_.reset();
}

// --------------------------------------------------------------------------------

}


