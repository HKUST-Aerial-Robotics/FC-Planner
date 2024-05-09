/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
                     Mingjie Zhang <zagerzhang at gmail dot com>, STAR Lab, SYSU.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main algorithm of basic solver for planning
 *                   in FC-Planner.
 * License      :    GNU General Public License <http://www.gnu.org/licenses/>.
 * Project      :    FC-Planner is free software: you can redistribute it and/or
 *                   modify it under the terms of the GNU Lesser General Public
 *                   License as published by the Free Software Foundation,
 *                   either version 3 of the License, or (at your option) any
 *                   later version.
 *                   FC-Planner is distributed in the hope that it will be useful,
 *                   but WITHOUT ANY WARRANTY; without even the implied warranty
 *                   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *                   See the GNU General Public License for more details.
 * Website      :    https://hkust-aerial-robotics.github.io/FC-Planner/
 *⭐⭐⭐*****************************************************************⭐⭐⭐*/
#include <viewpoint_manager/viewpoint_manager_node.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "viewpoint_manager_node");
    ros::NodeHandle nh("~");

    nh.param("viewpoint_manager_node/cloud_path", model_path_, string("null"));
    nh.param("viewpoint_manager_node/free_cloud_path", free_cloud_path_, string("null"));
    nh.param("viewpoint_manager_node/downsampled_size", downsampled_size_, 0.1);
    nh.param("viewpoint_manager_node/downsampled_size_for_viewpoint", downsampled_size_for_viewpoint_, 0.2);
    nh.param("viewpoint_manager_node/min_box_z", min_box_z_, -1.0);
    nh.param("viewpoint_manager_node/max_box_z", max_box_z_, -1.0);
    nh.param("viewpoint_manager/viewpoints_distance", dist_vp_, -1.0);

    model_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    free_model_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    downsampled_model_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    downsampled_normals_.reset(new pcl::PointCloud<pcl::Normal>);
    vis_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    initUtils(nh);
    modelProcessing();

    /* Filter high-quality surface point clouds */
    pcl::PointCloud<pcl::Normal>::Ptr tmp_normals;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_;
    tmp_normals.reset(new pcl::PointCloud<pcl::Normal>);
    tmp_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < (int)downsampled_normals_->points.size(); i++)
    {
        if (downsampled_model_->points[i].z < min_box_z_ || downsampled_model_->points[i].z > max_box_z_)
            continue;
        Vector3d pt(downsampled_model_->points[i].x, downsampled_model_->points[i].y, downsampled_model_->points[i].z);
        Vector3d dir(downsampled_normals_->points[i].normal_x, downsampled_normals_->points[i].normal_y, 0.0);
        Vector3d tmp_vp1 = pt + dist_vp_ * dir;
        Vector3d tmp_vp2 = pt - dist_vp_ * dir;
        pcl::Normal tmp_normal;

        if (viewpointSafetyCheck(tmp_vp1) && viewpointUsefulCheck(pt, tmp_vp1))
        {
            tmp_normal.normal_x = downsampled_normals_->points[i].normal_x;
            tmp_normal.normal_y = downsampled_normals_->points[i].normal_y;
            tmp_normal.normal_z = 0.0;
            tmp_normals->points.push_back(tmp_normal);
            tmp_cloud_->points.push_back(downsampled_model_->points[i]);
        }
        else if (viewpointSafetyCheck(tmp_vp2) && viewpointUsefulCheck(pt, tmp_vp2))
        {
            tmp_normal.normal_x = -downsampled_normals_->points[i].normal_x;
            tmp_normal.normal_y = -downsampled_normals_->points[i].normal_y;
            tmp_normal.normal_z = 0.0;
            tmp_normals->points.push_back(tmp_normal);
            tmp_cloud_->points.push_back(downsampled_model_->points[i]);
        }
    }
    downsampled_normals_ = tmp_normals;
    downsampled_model_ = tmp_cloud_;

    map<Eigen::Vector3d, Eigen::Vector3d, Vector3dCompare0> tmp_pt_normal_pairs;
    for (int i = 0; i < (int)downsampled_model_->points.size(); i++)
    {
        Vector3d pt_vec(downsampled_model_->points[i].x, downsampled_model_->points[i].y, downsampled_model_->points[i].z);
        Vector3d normal_vec(downsampled_normals_->points[i].normal_x, downsampled_normals_->points[i].normal_y, downsampled_normals_->points[i].normal_z);
        tmp_pt_normal_pairs[pt_vec] = normal_vec;
    }

    /* Origin viewpoint generation */
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_model_for_vp;
    downsampled_model_for_vp.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(downsampled_model_);
    sor.setLeafSize(downsampled_size_for_viewpoint_, downsampled_size_for_viewpoint_, downsampled_size_for_viewpoint_);
    sor.filter(*downsampled_model_for_vp);

    pcl::PointCloud<pcl::PointNormal>::Ptr vps(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin_vps(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < (int)downsampled_model_for_vp->points.size(); i++)
    {
        pcl::PointXYZ pt, vp;
        pt = downsampled_model_for_vp->points[i];
        Vector3d pt_vec = Vector3d(pt.x, pt.y, pt.z);
        auto normal_dir = tmp_pt_normal_pairs.find(pt_vec)->second;

        // filter out wrong normal vectors
        if (normal_dir.norm() < 1e-6)
            continue;
        pcl::PointNormal vp_fov_normal;
        vp.x = pt.x + dist_vp_ * normal_dir(0);
        vp.y = pt.y + dist_vp_ * normal_dir(1);
        vp.z = pt.z + dist_vp_ * normal_dir(2);
        vp_fov_normal.x = vp.x;
        vp_fov_normal.y = vp.y;
        vp_fov_normal.z = vp.z;
        vp_fov_normal.normal_x = -normal_dir(0);
        vp_fov_normal.normal_y = -normal_dir(1);
        vp_fov_normal.normal_z = -normal_dir(2);
        vps->points.push_back(vp_fov_normal);

        Vector3d tmp_vp1 = pt_vec + 0.85 * dist_vp_ * normal_dir;
        pcl::PointXYZ vpp;
        vpp.x = tmp_vp1(0);
        vpp.y = tmp_vp1(1);
        vpp.z = tmp_vp1(2);
        origin_vps->points.push_back(vpp);
    }

    /* Main API for viewpoint update */
    vector<SingleViewpoint> updated_vps;
    viewpoint_manager_->reset();
    viewpoint_manager_->setMapPointCloud(model_);
    viewpoint_manager_->setModel(downsampled_model_);
    viewpoint_manager_->setNormals(tmp_pt_normal_pairs);
    viewpoint_manager_->setInitViewpoints(vps);
    viewpoint_manager_->updateViewpoints();
    viewpoint_manager_->getUpdatedViewpoints(updated_vps);

    /* For visualization */
    pcl::PointCloud<pcl::PointNormal>::Ptr up_vps(new pcl::PointCloud<pcl::PointNormal>);
    vector<vector<Eigen::Vector3d>> updated_views1, updated_views2;
    for (int i = 0; i < (int)updated_vps.size(); i++)
    {
        pcl::PointNormal vp_fov_normal;
        Eigen::VectorXd vp_pose = updated_vps[i].pose;
        vp_fov_normal.x = vp_pose(0);
        vp_fov_normal.y = vp_pose(1);
        vp_fov_normal.z = vp_pose(2);
        up_vps->points.push_back(vp_fov_normal);
        percep_utils_->setPose_PY(vp_pose.head(3), vp_pose(3), vp_pose(4));
        std::vector<Eigen::Vector3d> v1, v2;
        percep_utils_->getFOV_PY(v1, v2);
        updated_views1.push_back(v1);
        updated_views2.push_back(v2);
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr nor_vps(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr origin_normals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < (int)vps->points.size(); i++)
    {
        pcl::PointNormal vp_fov_normal;
        vp_fov_normal.x = origin_vps->points[i].x;
        vp_fov_normal.y = origin_vps->points[i].y;
        vp_fov_normal.z = origin_vps->points[i].z;
        nor_vps->points.push_back(vp_fov_normal);

        pcl::Normal nor;
        nor.normal_x = vps->points[i].normal_x;
        nor.normal_y = vps->points[i].normal_y;
        nor.normal_z = vps->points[i].normal_z;
        origin_normals->points.push_back(nor);
    }

    ros::Rate loop_rate(0.5);
    while (ros::ok())
    {
        /* Map related visualization */
        publishCloudMap(occ_pub_, model_);
        publishNormals(model_normals_pub_, downsampled_model_, downsampled_normals_);
        // HCMap_->publishMap();

        /* Updated viewpoints visualization */
        publishViewpoints(updated_vps_pub_, up_vps, 0.4, Eigen::Vector4d(1, 0, 0, 1));
        publishFOV(updated_vps_fov_pub_, updated_views1, updated_views2);

        /* Origin viewpoints visualization */
        publishViewpoints(origin_vps_pub_, nor_vps, 0.35, Eigen::Vector4d(0.3, 0.2, 0.9, 1));
        publishNormals(origin_vps_normals_pub_, origin_vps, origin_normals);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void inputModel()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_model;
    double factor = 1.75;
    tmp_model.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(model_path_, *tmp_model);
    for (int i = 0; i < (int)tmp_model->points.size(); i++)
    {
        tmp_model->points[i].x *= factor;
        tmp_model->points[i].y *= factor;
        tmp_model->points[i].z *= 1.4;
        if (tmp_model->points[i].z < 0.55)
            continue;
        model_->points.push_back(tmp_model->points[i]);
    }

    tmp_model.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(free_cloud_path_, *tmp_model);
    for (int i = 0; i < (int)tmp_model->points.size(); i++)
    {
        tmp_model->points[i].x *= factor;
        tmp_model->points[i].y *= factor;
        tmp_model->points[i].z *= 1.4;
        if (tmp_model->points[i].z < 0.55)
            continue;
        free_model_->points.push_back(tmp_model->points[i]);
    }
}

void initUtils(ros::NodeHandle nh)
{
    occ_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/occ_point_cloud", 1);
    free_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/free_point_cloud", 1);
    model_normals_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/model_normals", 1);
    updated_vps_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/updated_viewpoints", 1);
    updated_vps_fov_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/updated_viewpoints_fov", 1);
    origin_vps_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/origin_viewpoints", 1);
    origin_vps_normals_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/origin_vps_normals", 1);

    inputModel();
    viewpoint_manager_.reset(new ViewpointManager);
    viewpoint_manager_->init(nh);

    percep_utils_.reset(new PerceptionUtils);
    percep_utils_->init(nh);

    HCMap_.reset(new SDFMap);
    HCMap_->initHCMap(nh, model_);
    HCMap_->inputFreePointCloud(free_model_);

    raycast_.reset(new RayCaster);
    raycast_->setParams(HCMap_->hcmp_->resolution_, HCMap_->hcmp_->map_origin_);
}

void modelProcessing()
{
    // model downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_model(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(model_);
    sor.setLeafSize(downsampled_size_, downsampled_size_, downsampled_size_);
    sor.filter(*tmp_model);
    for (auto pt : tmp_model->points)
    {
        Eigen::Vector3d pt_vec;
        pt_vec << pt.x, pt.y, pt.z;
        Eigen::Vector3i pt_index;
        HCMap_->posToIndex_hc(pt_vec, pt_index);
        if (HCMap_->hcmd_->occupancy_buffer_hc_[HCMap_->toAddress_hc(pt_index)] == 1)
            downsampled_model_->points.push_back(pt);
    }

    // get downsampled model's normal vectors
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(downsampled_model_);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(1.0);
    ne.compute(*downsampled_normals_);
}

bool viewpointSafetyCheck(Vector3d vp)
{
    Eigen::Vector3i vp_idx;
    HCMap_->posToIndex_hc(vp, vp_idx);
    if (!HCMap_->freeCheck(vp_idx))
        return false;
    return true;
}

bool viewpointSafetyCheck(pcl::PointXYZ vp)
{
    Vector3d vp_vec(vp.x, vp.y, vp.z);
    if (!viewpointSafetyCheck(vp_vec))
        return false;
    return true;
}

bool viewpointUsefulCheck(Vector3d pt, Vector3d vp)
{
    Vector3d pos1 = pt;
    Vector3d pos2 = vp;
    Eigen::Vector3i idx, pt_idx;
    HCMap_->posToIndex_hc(pt, pt_idx);

    raycast_->input(pos2, pos1);
    while (raycast_->nextId(idx))
    {
        Vector3d pos;
        HCMap_->indexToPos_hc(idx, pos);
        pcl::PointXYZ point;
        point.x = pos(0);
        point.y = pos(1);
        point.z = pos(2);
        if (point.z > 1.0 && point.z < 1.5)
            vis_cloud_->points.push_back(point);
        if (!HCMap_->occCheck(idx))
        {
            break;
        }
    }
    if ((pt_idx - idx).norm() <= 1)
        return true;
    return false;
}

void publishCloudMap(ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // Convert the PCL point cloud to a ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = frame_id_; // Set the frame ID

    // Publish the point cloud
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}

void publishNormals(ros::Publisher &pub,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                    const pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    visualization_msgs::MarkerArray pcloud_normals;
    int counter = 0;
    double scale = 1.0;
    for (int i = 0; i < (int)input_cloud->points.size(); ++i)
    {
        visualization_msgs::Marker nm;
        nm.header.frame_id = frame_id_;
        nm.header.stamp = ros::Time::now();
        nm.id = counter;
        nm.type = visualization_msgs::Marker::ARROW;
        nm.action = visualization_msgs::Marker::ADD;

        nm.pose.orientation.w = 1.0;
        nm.scale.x = 0.1;
        nm.scale.y = 0.05;
        nm.scale.z = 0.05;

        geometry_msgs::Point pt_;
        pt_.x = input_cloud->points[i].x;
        pt_.y = input_cloud->points[i].y;
        pt_.z = input_cloud->points[i].z;
        nm.points.push_back(pt_);

        pt_.x = input_cloud->points[i].x + scale * normals->points[i].normal_x;
        pt_.y = input_cloud->points[i].y + scale * normals->points[i].normal_y;
        pt_.z = input_cloud->points[i].z + scale * normals->points[i].normal_z;
        nm.points.push_back(pt_);

        nm.color.r = 0.8;
        nm.color.g = 0.2;
        nm.color.b = 0.2;
        nm.color.a = 1.0;

        pcloud_normals.markers.push_back(nm);
        counter++;
    }

    pub.publish(pcloud_normals);
}

void publishViewpoints(ros::Publisher &pub,
                       const pcl::PointCloud<pcl::PointNormal>::Ptr &viewpoints,
                       const double &scale, const Eigen::Vector4d &color)
{
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < viewpoints->size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "viewpoints";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        pcl::PointNormal vp = viewpoints->points[i];

        marker.pose.position.x = vp.x;
        marker.pose.position.y = vp.y;
        marker.pose.position.z = vp.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        marker.color.r = color(0);
        marker.color.g = color(1);
        marker.color.b = color(2);
        marker.color.a = color(3);

        marker_array.markers.push_back(marker);
    }

    pub.publish(marker_array);
}

void publishFOV(ros::Publisher &pub,
                const vector<vector<Eigen::Vector3d>> &list1, const vector<vector<Eigen::Vector3d>> &list2)
{
    visualization_msgs::MarkerArray vp_set;
    int counter = 0;
    for (int j = 0; j < (int)list1.size(); ++j)
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id_;
        mk.header.stamp = ros::Time::now();
        mk.id = counter;
        mk.ns = "FOV";
        mk.type = visualization_msgs::Marker::LINE_LIST;
        mk.color.r = 0.0;
        mk.color.g = 0.0;
        mk.color.b = 0.0;
        mk.color.a = 1.0;
        mk.scale.x = 0.08;
        mk.scale.y = 0.08;
        mk.scale.z = 0.08;
        mk.pose.orientation.w = 1.0;

        geometry_msgs::Point pt;
        for (int i = 0; i < int(list1[j].size()); ++i)
        {
            pt.x = list1[j][i](0);
            pt.y = list1[j][i](1);
            pt.z = list1[j][i](2);
            mk.points.push_back(pt);

            pt.x = list2[j][i](0);
            pt.y = list2[j][i](1);
            pt.z = list2[j][i](2);
            mk.points.push_back(pt);
        }

        vp_set.markers.push_back(mk);
        counter++;
    }

    pub.publish(vp_set);
}
