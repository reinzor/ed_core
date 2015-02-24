#include "world_update_server_plugin.h"

WorldUpdateServer::WorldUpdateServer()
{
    current_rev_number = 0;
    min_rev_number_stored = 0;
}


WorldUpdateServer::~WorldUpdateServer()
{

}

bool WorldUpdateServer::GetWorldModel(ed::GetWorldModel::Request &req, ed::GetWorldModel::Response &res)
{

    bool success = true;

    ROS_INFO_STREAM("Queried revision " << req.rev_number
                    << " , " << "Current rev number = "
                    << current_rev_number << std::endl);

    if (req.rev_number < 0 || req.rev_number > current_rev_number) {
        success = false;
    } else if (current_rev_number > req.rev_number) {
        res.world = combineDeltas(req.rev_number);
        res.rev_number = current_rev_number;
    }

    return success;
}

void WorldUpdateServer::configure(tue::Configuration config)
{

}

void WorldUpdateServer::initialize()
{
    ros::NodeHandle nh;

    has_new_delta = false;
    current_rev_number = 0;
    min_rev_number_stored = 0;

    ROS_INFO("Advetising new service");

    ros::AdvertiseServiceOptions get_world_model =
            ros::AdvertiseServiceOptions::create<ed::GetWorldModel>(
                "/ed/get_world", boost::bind(&WorldUpdateServer::GetWorldModel, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);
    srv_get_world_ = nh.advertiseService(get_world_model);

}

void WorldUpdateServer::updateRequestCallback(const ed::UpdateRequest &req)
{

    for ( std::map<ed::UUID, geo::ShapeConstPtr>::const_iterator it = req.shapes.begin();
          it != req.shapes.end(); it ++) {
        modified_entities_current_delta.insert(it->first);
        shapes_current_delta[it->first.str()] = it->second;
    }

    for ( std::map<ed::UUID, std::string>::const_iterator it = req.types.begin();
          it != req.types.end(); it ++) {
        modified_entities_current_delta.insert(it->first);
        types_current_delta[it->first.str()] = it->second;
    }

    for ( std::map<ed::UUID, geo::Pose3D>::const_iterator it = req.poses.begin();
          it != req.poses.end(); it ++) {
        modified_entities_current_delta.insert(it->first);
        poses_current_delta[it->first.str()] = it->second;
    }


    for ( std::map<ed::UUID, ed::ConvexHull2D>::const_iterator it = req.convex_hulls.begin();
          it != req.convex_hulls.end(); it ++) {
        modified_entities_current_delta.insert(it->first);
        convex_hulls_current_delta[it->first.str()] = it->second;
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

        if (poses_current_delta.find(it->str()) != poses_current_delta.end()) {
            new_info.new_pose = true;
            geo::convert(poses_current_delta[it->str()], new_info.pose);
        }

        // Type

        if (types_current_delta.find(it->str()) != types_current_delta.end()) {
            new_info.new_type = true;
            new_info.type = types_current_delta[it->str()];
        }

        // Shape

        if (shapes_current_delta.find(it->str()) != shapes_current_delta.end()) {
            new_info.new_shape_or_convex = true;
            new_info.is_convex_hull = false;

            const geo::Mesh mesh = shapes_current_delta[it->str()]->getMesh();

            for (std::vector<geo::Vector3>::const_iterator it2 = mesh.getPoints().begin();
                 it2 != mesh.getPoints().end(); it2++) {
                 new_info.mesh.vertices.push_back(it2->getX());
                 new_info.mesh.vertices.push_back(it2->getY());
                 new_info.mesh.vertices.push_back(it2->getZ());
            }

            for (std::vector<geo::TriangleI>::const_iterator it2 = mesh.getTriangleIs().begin();
                 it2 != mesh.getTriangleIs().end(); it2++) {
                 new_info.mesh.triangles.push_back(it2->i1_);
                 new_info.mesh.triangles.push_back(it2->i2_);
                 new_info.mesh.triangles.push_back(it2->i3_);
            }

        }

        // Convex Hulls

        if (convex_hulls_current_delta.find(it->str()) != convex_hulls_current_delta.end()) {

            new_info.new_shape_or_convex = true;
            new_info.is_convex_hull = true;

            for (pcl::PointCloud<pcl::PointXYZ>::iterator it2 = convex_hulls_current_delta[it->str()].chull.begin();
                 it2 != convex_hulls_current_delta[it->str()].chull.end(); it++) {

                new_info.polygon.xs.push_back(it2->x);
                new_info.polygon.ys.push_back(it2->y);

            }

            new_info.polygon.z_min = convex_hulls_current_delta[it->str()].min_z;
            new_info.polygon.z_max = convex_hulls_current_delta[it->str()].max_z;
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

ed::WorldModelDelta WorldUpdateServer::combineDeltas(int rev_number)
{
    ed::WorldModelDelta res_delta;
    std::map<std::string, ed::EntityUpdateInfo> entity_update_res_delta;
    std::set<std::string> removed_entities_res_delta;
    int starting_delta = rev_number + 1;

    // Merge information

    if (starting_delta >= this->min_rev_number_stored) {
        for (int i = starting_delta; i <= this->current_rev_number; i ++) {
            for (std::vector<ed::EntityUpdateInfo>::const_iterator it = deltaModels[i - min_rev_number_stored - 1].update_entities.begin();
                 it != deltaModels[i - min_rev_number_stored - 1].update_entities.end(); it++) {
                    if (entity_update_res_delta.find(it->id) == entity_update_res_delta.end()) {
                        entity_update_res_delta[it->id] = *it;
                    } else {
                        if (it->new_pose) {
                            entity_update_res_delta[it->id].new_pose = true;
                            entity_update_res_delta[it->id].pose = it->pose;
                        }

                        if (it->new_shape_or_convex) {
                            entity_update_res_delta[it->id].new_shape_or_convex = true;
                            entity_update_res_delta[it->id].is_convex_hull = it->is_convex_hull;
                            entity_update_res_delta[it->id].center = it->center;
                            entity_update_res_delta[it->id].polygon = it->polygon;
                            entity_update_res_delta[it->id].mesh = it->mesh;
                        }

                        if (it->new_type) {
                            entity_update_res_delta[it->id].type = it->type;
                            entity_update_res_delta[it->id].new_type = it->new_type;
                        }

                        entity_update_res_delta[it->id].id = it->id;

                    }
              }

            for (std::vector<std::string>::iterator it = deltaModels[i - min_rev_number_stored - 1].remove_entities.begin();
                 it != deltaModels[i - min_rev_number_stored - 1].remove_entities.end(); it ++) {
                std::map<std::string, ed::EntityUpdateInfo>::iterator pos = entity_update_res_delta.find(*it);

                if (pos != entity_update_res_delta.end()) {
                    entity_update_res_delta.erase(pos);
                }

                removed_entities_res_delta.insert(*it);
            }
        }
    }

    // Create final delta

    res_delta.rev_number = this->current_rev_number;

    for (std::map<std::string, ed::EntityUpdateInfo>::iterator it = entity_update_res_delta.begin();
         it != entity_update_res_delta.end(); it ++) {
        res_delta.update_entities.push_back(it->second);
    }


    for (std::set<std::string>::const_iterator it = removed_entities_res_delta.begin();
         it != removed_entities_res_delta.end(); it ++) {
        res_delta.remove_entities.push_back(*it);
    }

    return res_delta;
}



ED_REGISTER_PLUGIN(WorldUpdateServer)
