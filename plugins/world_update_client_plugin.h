#ifndef WORLD_UPDATE_CLIENT_PLUGIN_H
#define WORLD_UPDATE_CLIENT_PLUGIN_H

#include <ros/callback_queue.h>
#include <ed/plugin.h>
#include <ed/WorldModelDelta.h>

class WorldUpdateClient : public ed::Plugin
{
public:
    WorldUpdateClient();

    virtual ~WorldUpdateClient();

    void configure(tue::Configuration config);

    void initialize();

    void updateWithDelta(ed::WorldModelDelta& a, const ed::WorldModel &world, ed::UpdateRequest &req);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    int current_rev_number;
};

#endif // WORLD_UPDATE_CLIENT_PLUGIN_H
