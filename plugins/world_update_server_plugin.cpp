#include "world_update_server_plugin.h"

#include <ed/entity.h>

// ----------------------------------------------------------------------------------------------------

void shapeToMsg(const geo::Shape& shape, ed::Mesh& msg)
{
    const geo::Mesh& mesh = shape.getMesh();

    for (std::vector<geo::Vector3>::const_iterator it2 = mesh.getPoints().begin();
         it2 != mesh.getPoints().end(); it2++)
    {
         msg.vertices.push_back(it2->getX());
         msg.vertices.push_back(it2->getY());
         msg.vertices.push_back(it2->getZ());
    }

    for (std::vector<geo::TriangleI>::const_iterator it2 = mesh.getTriangleIs().begin();
         it2 != mesh.getTriangleIs().end(); it2++)
    {
         msg.triangles.push_back(it2->i1_);
         msg.triangles.push_back(it2->i2_);
         msg.triangles.push_back(it2->i3_);
    }
}

// ----------------------------------------------------------------------------------------------------

void polygonToMsg(const ed::ConvexHull2D& ch, ed::Polygon& msg)
{
    msg.z_min = ch.min_z;
    msg.z_max = ch.max_z;

    msg.xs.resize(ch.chull.size());
    msg.ys.resize(ch.chull.size());

    for(unsigned int i = 0; i <  ch.chull.size(); ++i)
    {
        msg.xs[i] = ch.chull[i].x;
        msg.ys[i] = ch.chull[i].y;
    }
}

// ----------------------------------------------------------------------------------------------------

void entityToMsg(const ed::Entity& e, ed::EntityUpdateInfo& msg)
{
    // id
    msg.id = e.id().str();

    // type
    msg.type = e.type();
    msg.new_type = true;

    // pose
    geo::convert(e.pose(), msg.pose);
    msg.new_pose = true;

    // shape
    if (e.shape())
    {
        // Mesh
        shapeToMsg(*e.shape(), msg.mesh);
        msg.new_shape_or_convex = true;
    }
    else if (!e.convexHull().chull.empty())
    {
        // Polygon
        polygonToMsg(e.convexHull(), msg.polygon);
        msg.new_shape_or_convex = true;
    }
}

// ----------------------------------------------------------------------------------------------------

WorldUpdateServer::WorldUpdateServer()
{
    current_rev_number = 0;
    i_delta_models_start_ = 0;
    i_latest_delta_ = 0;

    using_delta_ = false;

    max_num_delta_models_ = 10;
}

// ----------------------------------------------------------------------------------------------------

WorldUpdateServer::~WorldUpdateServer()
{
}

// ----------------------------------------------------------------------------------------------------

bool WorldUpdateServer::GetWorldModel(ed::GetWorldModel::Request &req, ed::GetWorldModel::Response &res)
{
    ROS_INFO_STREAM("Queried revision " << req.rev_number
                    << " , " << "Current rev number = "
                    << current_rev_number << std::endl);

    if (req.rev_number > current_rev_number)
    {
        std::stringstream ss;
        ss << "Requested revision is in the future: client asks revision '" << req.rev_number << "' but current revision is '" << current_rev_number << "'.";
        res.error = ss.str();
    }
    else if (current_rev_number > req.rev_number)
    {
        if (req.rev_number + deltaModels.size() < current_rev_number)
        {
            // The revision is too old for the number of deltas stored. Therefore, send full entity info, but only
            // for the entities that changed since the requested revision number

            ROS_INFO_STREAM("Requested revision too old for sending small delta revision; will send full entity info instead.");

            for(unsigned int i = 0; i < world_->entities().size(); ++i)
            {
                const ed::EntityConstPtr& e = world_->entities()[i];
                if (req.rev_number < entity_server_revisions_[i])
                {
                    res.world.update_entities.push_back(ed::EntityUpdateInfo());
                    ed::EntityUpdateInfo& info = res.world.update_entities.back();
                    info.index = i;

                    if (e)
                        entityToMsg(*e, info);
                }
            }
        }
        else
        {
            res.world = combineDeltas(req.rev_number);
        }

        res.rev_number = current_rev_number;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

void WorldUpdateServer::configure(tue::Configuration config)
{
}

// ----------------------------------------------------------------------------------------------------

void WorldUpdateServer::initialize()
{
    ros::NodeHandle nh;

    has_new_delta[0] = false;
    has_new_delta[1] = false;

    current_rev_number = 0;

    ROS_INFO("Advetising new service");

    ros::AdvertiseServiceOptions get_world_model =
            ros::AdvertiseServiceOptions::create<ed::GetWorldModel>(
                "/ed/get_world", boost::bind(&WorldUpdateServer::GetWorldModel, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);
    srv_get_world_ = nh.advertiseService(get_world_model);
}

// ----------------------------------------------------------------------------------------------------

void WorldUpdateServer::updateRequestCallback(const ed::UpdateRequest &req)
{
    // This function is called from another thread than all the other functions. Therefore, we need
    // a double buffer system for the delta update

    if (using_delta_)
    {
        // The other thread is currently using the latest delta, so switch to a new one
        i_latest_delta_ = 1 - i_latest_delta_;

        // Make sure the delta is empty
        latest_delta_[i_latest_delta_] = ed::UpdateRequest();
    }

    ed::UpdateRequest& delta = latest_delta_[i_latest_delta_];

    // Notify that the delta is not ready
    has_new_delta[i_latest_delta_] = false;

    delta.shapes.insert(req.shapes.begin(), req.shapes.end());
    delta.types.insert(req.types.begin(), req.types.end());
    delta.poses.insert(req.poses.begin(), req.poses.end());
    delta.convex_hulls.insert(req.convex_hulls.begin(), req.convex_hulls.end());
    delta.removed_entities.insert(req.removed_entities.begin(), req.removed_entities.end());

    // Notify that the delta is ready
    has_new_delta[i_latest_delta_] = true;
}

// ----------------------------------------------------------------------------------------------------

ed::EntityUpdateInfo& WorldUpdateServer::addOrGetEntityUpdate(
        const ed::UUID& id, std::map<std::string, unsigned int>& ids, ed::WorldModelDelta& delta)
{
    std::map<std::string, unsigned int>::const_iterator it = ids.find(id.str());
    if (it != ids.end())
        return delta.update_entities[it->second];

    // Remember the entity index for next time
    ids[id.str()] = delta.update_entities.size();

    // Add a fresh new entity update
    delta.update_entities.push_back(ed::EntityUpdateInfo());

    // Get a reference to the new entity update
    ed::EntityUpdateInfo& new_info = delta.update_entities.back();

    // Set id
    new_info.id = id.str();

    // Find entity index
    world_->findEntityIdx(new_info.id, new_info.index);

    // Update entity server revision list
    for(unsigned int i = entity_server_revisions_.size(); i < new_info.index + 1; ++i)
        entity_server_revisions_.push_back(0);
    entity_server_revisions_[new_info.index] = current_rev_number;

    return new_info;
}

// ----------------------------------------------------------------------------------------------------

void WorldUpdateServer::createNewDelta()
{
    current_rev_number++;

    ed::WorldModelDelta new_delta;
    std::map<std::string, unsigned int> updated_ids;

    using_delta_ = true;
    ed::UpdateRequest& delta = latest_delta_[i_latest_delta_];

    // Types
    for (std::map<ed::UUID, std::string>::const_iterator it = delta.types.begin(); it != delta.types.end(); it ++)
    {
        ed::EntityUpdateInfo& info = addOrGetEntityUpdate(it->first, updated_ids, new_delta);
        info.new_type = true;
        info.type = it->second;
    }

    // Poses
    for (std::map<ed::UUID, geo::Pose3D>::const_iterator it = delta.poses.begin(); it != delta.poses.end(); it ++)
    {
        ed::EntityUpdateInfo& info = addOrGetEntityUpdate(it->first, updated_ids, new_delta);
        info.new_pose = true;
        geo::convert(it->second, info.pose);
    }

    // Shapes
    for (std::map<ed::UUID, geo::ShapeConstPtr>::const_iterator it = delta.shapes.begin(); it != delta.shapes.end(); it ++)
    {
        ed::EntityUpdateInfo& info = addOrGetEntityUpdate(it->first, updated_ids, new_delta);
        info.new_shape_or_convex = true;
        info.is_convex_hull = false;
        shapeToMsg(*it->second, info.mesh);
    }

    // Convex hulls
    for (std::map<ed::UUID, ed::ConvexHull2D>::const_iterator it = delta.convex_hulls.begin(); it != delta.convex_hulls.end(); it ++)
    {
        ed::EntityUpdateInfo& info = addOrGetEntityUpdate(it->first, updated_ids, new_delta);
        info.new_shape_or_convex = true;
        info.is_convex_hull = true;
        polygonToMsg(it->second, info.polygon);
    }

    // Removed entities
    for (std::set<ed::UUID>::iterator it = delta.removed_entities.begin(); it != delta.removed_entities.end(); it++)
    {
        new_delta.remove_entities.push_back(it->str());

        // Find entity index
        unsigned int entity_idx;
        world_->findEntityIdx(*it, entity_idx);

        // Update entity server revision list
        for(unsigned int i = entity_server_revisions_.size(); i < entity_idx + 1; ++i)
            entity_server_revisions_.push_back(0);
        entity_server_revisions_[entity_idx] = current_rev_number;
    }

    using_delta_ = false;

    if (deltaModels.size() < max_num_delta_models_)
    {
        // If we have not yet reached the max number of delta models, simply push the delta
        deltaModels.push_back(new_delta);
    }
    else
    {
        // Otherwise, calculate the new index (deltaModels is a circular buffer)
        deltaModels[i_delta_models_start_] = new_delta;
        i_delta_models_start_ = (i_delta_models_start_ + 1) % max_num_delta_models_;
    }

    has_new_delta[i_latest_delta_] = false;

    ROS_INFO("New Delta Created");
}

// ----------------------------------------------------------------------------------------------------

void WorldUpdateServer::process(const ed::WorldModel &world, ed::UpdateRequest &req)
{
    world_ = &world;

    if (has_new_delta[i_latest_delta_] && max_num_delta_models_ > 0) {
        this->createNewDelta();
    }

    this->cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

ed::WorldModelDelta WorldUpdateServer::combineDeltas(int rev_number)
{
    ed::WorldModelDelta res_delta;
    std::map<std::string, ed::EntityUpdateInfo> entity_update_res_delta;
    std::set<std::string> removed_entities_res_delta;

    // Merge information

    for (int i = rev_number; i < this->current_rev_number; i++)
    {
        const ed::WorldModelDelta& delta = deltaModels[(i + deltaModels.size() + i_delta_models_start_ - current_rev_number) % max_num_delta_models_];

        for (std::vector<ed::EntityUpdateInfo>::const_iterator it = delta.update_entities.begin(); it != delta.update_entities.end(); it++)
        {
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

        for (std::vector<std::string>::const_iterator it = delta.remove_entities.begin();
             it != delta.remove_entities.end(); it ++)
        {
            std::map<std::string, ed::EntityUpdateInfo>::iterator pos = entity_update_res_delta.find(*it);

            if (pos != entity_update_res_delta.end()) {
                entity_update_res_delta.erase(pos);
            }

            removed_entities_res_delta.insert(*it);
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
