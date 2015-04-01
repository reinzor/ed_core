#include "ed/plugin_container.h"

#include "ed/plugin.h"
#include "ed/rate.h"

#include <ed/error_context.h>

namespace ed
{

// --------------------------------------------------------------------------------

PluginContainer::PluginContainer(WorldModelConstPtr world_model)
    : class_loader_(0), stop_(false), cycle_duration_(0.1), loop_frequency_(10), step_finished_(true), t_last_update_(0),
      total_process_time_sec_(0), world_current_(world_model)
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

PluginPtr PluginContainer::loadPlugin(const std::string plugin_name, const std::string& lib_filename, InitData& init)
{
    // Load the library
    delete class_loader_;
    class_loader_ = new class_loader::ClassLoader(lib_filename);

    // Create plugin
    class_loader_->loadLibrary();
    std::vector<std::string> classes = class_loader_->getAvailableClasses<ed::Plugin>();

    if (classes.empty())
    {
        init.config.addError("Could not find any plugins in '" + class_loader_->getLibraryPath() + "'.");
    } else if (classes.size() > 1)
    {
        init.config.addError("Multiple plugins registered in '" + class_loader_->getLibraryPath() + "'.");
    } else
    {
        plugin_ = class_loader_->createInstance<Plugin>(classes.front());
        if (plugin_)
        {
            double freq = 10; // default

            name_ = plugin_name;
            plugin_->name_ = plugin_name;

            // Read optional frequency
            init.config.value("frequency", freq, tue::OPTIONAL);

            if (init.config.readGroup("parameters"))
            {
                tue::Configuration scoped_config = init.config.limitScope();
                InitData scoped_init(init.properties, scoped_config);

                plugin_->configure(scoped_config);  // This call will become obsolete (TODO)
                plugin_->initialize(scoped_init);

                // Read optional frequency (inside parameters is obsolete)
                if (init.config.value("frequency", freq, tue::OPTIONAL))
                {
                    std::cout << "[ED]: Warning while loading plugin '" << name_ << "': please specify parameter 'frequency' outside 'parameters'." << std::endl;
                }

                init.config.endGroup();
            }
            else
            {
                // No parameter available
                tue::Configuration scoped_config;
                InitData scoped_init(init.properties, scoped_config);

                plugin_->configure(scoped_config);  // This call will become obsolete (TODO)
                plugin_->initialize(scoped_init);

                if (scoped_config.hasError())
                    init.config.addError(scoped_config.error());
            }

            // If there was an error during configuration, do not start plugin
            if (init.config.hasError())
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

    std::vector<UpdateRequestConstPtr> world_deltas;

    // Check if there is a new world. If so replace the current one with the new one
    {
        boost::lock_guard<boost::mutex> lg(mutex_world_);
        if (world_new_)
        {
            world_current_ = world_new_;
            world_deltas = world_deltas_;

            world_deltas_.clear();
            world_new_.reset();
        }
    }

    if (world_current_)
    {
        PluginInput data(*world_current_, world_deltas);

        UpdateRequestPtr update_request(new UpdateRequest);

        tue::Timer timer;
        timer.start();

        // Old
        {
            ed::ErrorContext errc("Plugin:", name().c_str());

            plugin_->process(*world_current_, *update_request);

            // New
            plugin_->process(data, *update_request);
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


