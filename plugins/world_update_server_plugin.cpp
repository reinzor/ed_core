#include "world_update_server_plugin.h"

WorldUpdateServer::WorldUpdateServer()
{
    current_rev_number = 0;
}


WorldUpdateServer::~WorldUpdateServer()
{

}

bool WorldUpdateServer::GetWorldModel(ed::GetWorldModel::Request &req, ed::GetWorldModel::Response &res)
{
    res.number_revisions = 0;

    if (current_rev_number >= req.rev_number) {
        for (int i = req.rev_number; i < current_rev_number; i ++) {
            res.world.push_back(this->deltaModels[i]);
            res.number_revisions++;
        }
    } else {
        return false;
    }
}

void WorldUpdateServer::configure(tue::Configuration config)
{

}

void WorldUpdateServer::initialize()
{
    has_new_delta = false;
    n.setCallbackQueue(&cb_queue_);
    current_rev_number = 0;

    ROS_INFO("Advetising new service");
    if (!n.advertiseService("get_world_model", &WorldUpdateServer::GetWorldModel, this)) {
        ROS_ERROR("Did not work");
    }
}

void WorldUpdateServer::updateRequestCallback(const ed::UpdateRequest &req)
{

    for ( std::map<ed::UUID, geo::ShapeConstPtr>::const_iterator it = req.shapes.begin();
          it != req.shapes.end(); it ++) {
        modified_entities_current_delta.insert(it->first);
        shapes_current_delta[it->first] = it->second;
    }

    for ( std::map<ed::UUID, std::string>::const_iterator it = req.types.begin();
          it != req.types.end(); it ++) {
        modified_entities_current_delta.insert(it->first);
        types_current_delta[it->first] = it->second;
    }

    for ( std::map<ed::UUID, geo::Pose3D>::const_iterator it = req.poses.begin();
          it != req.poses.end(); it ++) {
        modified_entities_current_delta.insert(it->first);
        poses_current_delta[it->first] = it->second;
    }

    removed_entities_current_delta.insert(req.removed_entities.begin(), req.removed_entities.end());
    has_new_delta = true;

}

void WorldUpdateServer::createNewDelta()
{
    current_rev_number++;

    ed::WorldModelDelta new_delta;

    for (std::set<ed::UUID>::const_iterator it = modified_entities_current_delta.begin();
         it != modified_entities_current_delta.end(); it++) {
        ed::EntityUpdateInfo new_info;

        new_info.id = it->str();

        // Pose

        if (poses_current_delta.find(*it) != poses_current_delta.end()) {
            new_info.new_pose = true;
            geo::convert(poses_current_delta[*it], new_info.pose);
        }

        // Type

        if (types_current_delta.find(*it) != types_current_delta.end()) {
            new_info.new_type = true;
            new_info.type = types_current_delta[*it];
        }

        // Shape

        if (shapes_current_delta.find(*it) != shapes_current_delta.end()) {
            new_info.new_shape_or_convex = true;
            new_info.is_convex_hull = false;

            const geo::Mesh mesh = shapes_current_delta[*it]->getMesh();

            for (std::vector<geo::Vector3>::const_iterator it2 = mesh.getPoints().begin();
                 it2 != mesh.getPoints().end(); it2++) {
                 new_info.mesh.vertices.push_back(it2->getX());
                 new_info.mesh.vertices.push_back(it2->getY());
                 new_info.mesh.vertices.push_back(it2->getZ());
            }

            for (std::vector<geo::TriangleI>::const_iterator it2 = mesh.getTriangleIs().begin();
                 it2 != mesh.getTriangleIs().end(); it2++) {
                 new_info.mesh.vertices.push_back(it2->i1_);
                 new_info.mesh.vertices.push_back(it2->i2_);
                 new_info.mesh.vertices.push_back(it2->i3_);
            }

        }

        new_delta.update_entities.push_back(new_info);

    }

    for (std::set<ed::UUID>::iterator it = removed_entities_current_delta.begin();
         it != removed_entities_current_delta.end(); it++) {
        new_delta.remove_entities.push_back(it->str());
    }

    deltaModels.push_back(new_delta);
    has_new_delta = false;
    ROS_INFO("New Delta Created");

}

void WorldUpdateServer::process(const ed::WorldModel &world, ed::UpdateRequest &req)
{
    if (has_new_delta) {
        this->createNewDelta();
    }

    this->cb_queue_.callAvailable();

}



ED_REGISTER_PLUGIN(WorldUpdateServer)
