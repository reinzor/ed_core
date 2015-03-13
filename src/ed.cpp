#include "ed/server.h"

#include <ed/world_model.h>

// Query
#include <ed/entity.h>
#include <tue/config/yaml_emitter.h>

// Reset
#include <std_srvs/Empty.h>

// Loop
#include <ed/event_clock.h>

// Plugin loading
#include <ed/plugin.h>
#include <tue/config/loaders/yaml.h>

#include <signal.h>
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <boost/thread.hpp>
#include "ed/error_context.h"
boost::thread::id main_thread_id;

ed::Server* ed_wm;

// ----------------------------------------------------------------------------------------------------

bool getEnvironmentVariable(const std::string& var, std::string& value)
{
     const char * val = ::getenv(var.c_str());
     if ( val == 0 )
         return false;

     value = val;
     return true;
}

// ----------------------------------------------------------------------------------------------------

void signalHandler( int sig )
{
    // Make sure to remove all signal handlers
    signal(SIGSEGV, SIG_DFL);
    signal(SIGABRT, SIG_DFL);

    std::cout << "\033[38;5;1m";
    std::cout << "[ED] ED Crashed!" << std::endl << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Print signal

    std::cout << "    Signal: ";
    if (sig == SIGSEGV)
        std::cout << "segmentation fault";
    else if (sig == SIGABRT)
        std::cout << "abort";
    else
        std::cout << "unknown";
    std::cout << std::endl << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Print thread name

    std::cout << "    Thread: ";

    char name[1000];
    size_t name_size = 1000;
    if (pthread_getname_np(pthread_self(), name, name_size) == 0)
    {
        if (std::string(name) == "ed_main")
        {
            if (boost::this_thread::get_id() == main_thread_id)
                std::cout << "main";
            else
                std::cout << "name unknown (id = " << boost::this_thread::get_id() << ")";
        }
        else
            std::cout << name;
    }
    else
        std::cout << "name unknown (id = " << boost::this_thread::get_id() << ")";

    std::cout << std::endl << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Print error context

    std::cout << "    Error context: ";

    ed::ErrorContextData* edata = ed::ErrorContext::data();
    if (edata && !edata->stack.empty())
    {
        std::cout << std::endl << std::endl;

        for(unsigned int i = edata->stack.size(); i > 0; --i)
        {
            const char* message = edata->stack[i-1].first;
            const char* value = edata->stack[i-1].second;

            if (message)
            {
                std::cout << "        " << message;
                if (value)
                    std::cout << " " << value;
                std::cout << std::endl;
            }
        }
    }
    else
        std::cout << "unknown";

    std::cout << std::endl << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Print backtrace

    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "Backtrace: " << std::endl << std::endl;

    void *array[10];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 10);

    // print out all the frames to stderr
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    std::cout << "\033[0m" << std::endl;
    exit(1);
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    // Set the name of the main thread
    pthread_setname_np(pthread_self(), "ed_main");

    // Remember the main thread id
    main_thread_id = boost::this_thread::get_id();

    // register signal SIGINT and signal handler
    signal(SIGSEGV, signalHandler);
    signal(SIGABRT, signalHandler);

    ed::ErrorContext errc("Start ED server", "init");

    // Create the ED server
    ed::Server server;
    ed_wm = &server;

    // - - - - - - - - - - - - - - - configure - - - - - - - - - - - - - - -

    errc.change("Start ED server", "configure");

    // Get plugin paths
    std::string ed_plugin_path;
    if (getEnvironmentVariable("ED_PLUGIN_PATH", ed_plugin_path))
    {
        std::stringstream ss(ed_plugin_path);
        std::string item;
        while (std::getline(ss, item, ':'))
            ed_wm->addPluginPath(item);
    }
    else
    {
        std::cout << "Error: Environment variable ED_PLUGIN_PATH not set." << std::endl;
        return 1;
    }

    tue::Configuration config;

    // Check if a config file was provided. If so, load it. If not, load the default AMIGO config.
    if (argc >= 2)
    {
        std::string yaml_filename = argv[1];
        config.loadFromYAMLFile(yaml_filename);
    }

    // Configure ED
    ed_wm->configure(config);

    if (config.hasError())
    {
        std::cout << std::endl << "Error during configuration:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    // - - - - - - - - - - - - service initialization - - - - - - - - - - - -

    errc.change("Start ED server", "service init");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    errc.change("Start ED server", "init");

    // Init ED
    ed_wm->initialize();

    ed::EventClock trigger_config(10);
    ed::EventClock trigger_ed(10);
    ed::EventClock trigger_stats(2);
    ed::EventClock trigger_plugins(1000);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    errc.change("ED server", "main loop");

    for (;;)
    {
        // Check if configuration has changed. If so, call reconfigure
        if (trigger_config.triggers() && config.sync())
            ed_wm->configure(config, true);

        if (trigger_ed.triggers())
            ed_wm->update();

        if (trigger_plugins.triggers())
            ed_wm->stepPlugins();

        if (trigger_stats.triggers())
            ed_wm->publishStatistics();

        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }

    return 0;
}
