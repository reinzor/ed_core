#include "hello_world_plugin.h"

#include <iostream>

// ----------------------------------------------------------------------------------------------------

HelloWorld::HelloWorld()
{
}

// ----------------------------------------------------------------------------------------------------

HelloWorld::~HelloWorld()
{
}

// ----------------------------------------------------------------------------------------------------

void HelloWorld::configure(tue::Configuration config)
{
    config.value("text", text_);
}

// ----------------------------------------------------------------------------------------------------

void HelloWorld::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    std::cout << text_ << std::endl;
}

ED_REGISTER_PLUGIN(HelloWorld)
