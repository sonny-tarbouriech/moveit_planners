/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/ompl_interface/detail/safe_state_validity_checker.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/profiler/profiler.h>
#include <ros/ros.h>

//STa
#include "geometric_shapes/shape_operations.h"
#include <moveit/collision_detection_fcl/collision_world_fcl.h>

//STa temp
#include <fstream>

static const double SAFETY_DISTANCE = 2;


ompl_interface::SafeStateValidityChecker::SafeStateValidityChecker(const ModelBasedPlanningContext *pc)
: ompl::base::SafeStateValidityChecker(pc->getOMPLSimpleSetup()->getSpaceInformation())
, planning_context_(pc)
, group_name_(pc->getGroupName())
, tsss_(pc->getCompleteInitialRobotState())
, tsss1_(pc->getCompleteInitialRobotState())
, tsss2_(pc->getCompleteInitialRobotState())
{
	//STa
	safe_collision_robot_fcl_unpadded_ = new collision_detection::SafeCollisionRobotFCL(*(static_cast<const collision_detection::CollisionRobotFCL*> (planning_context_->getPlanningScene()->getCollisionRobotUnpadded().get())));
	safe_collision_robot_fcl_padded_ = new collision_detection::SafeCollisionRobotFCL(planning_context_->getRobotModel(), 0.02, 1);

	safe_collision_world_fcl_ = static_cast<const collision_detection::SafeCollisionWorldFCL*> (planning_context_->getPlanningScene()->getCollisionWorld().get());

	collision_request_with_distance_.distance = true;

	collision_request_with_contacts_.contacts = true;

	collision_request_simple_.group_name = planning_context_->getGroupName();
	collision_request_with_distance_.group_name = planning_context_->getGroupName();
	collision_request_with_contacts_.group_name = planning_context_->getGroupName();

	fcl_collision_obj_ = safe_collision_world_fcl_->getCollisionObjects();

	obj_danger_factor_ =  getCollisionObjectsFactors();


	nb_objects_ = fcl_collision_obj_.size();

	active_joints_ = planning_context_->getJointModelGroup()->getActiveJointModels();

	nb_safety_links_ = NB_SAFETY_LINKS;
	safety_links_name_.resize(NB_SAFETY_LINKS);

	safety_links_name_cc_.resize(NB_SAFETY_LINKS);

	safety_links_name_with_collision_ = planning_context_->getRobotModel()->getLinkModelNamesWithCollisionGeometry();

	//	safety_links_name_with_collision_.push_back("collision_head_link_1");
	//	safety_links_name_with_collision_.push_back("collision_head_link_2");
	//	safety_links_name_with_collision_.push_back("left_hand");
	//	safety_links_name_with_collision_.push_back("left_lower_elbow");
	//	safety_links_name_with_collision_.push_back("left_lower_forearm");
	//	safety_links_name_with_collision_.push_back("left_lower_shoulder");
	//	safety_links_name_with_collision_.push_back("left_upper_elbow");
	//	safety_links_name_with_collision_.push_back("left_upper_elbow_visual");
	//	safety_links_name_with_collision_.push_back("left_upper_forearm");
	//	safety_links_name_with_collision_.push_back("left_upper_forearm_visual");
	//	safety_links_name_with_collision_.push_back("left_upper_shoulder");
	//	safety_links_name_with_collision_.push_back("left_wrist");
	//	safety_links_name_with_collision_.push_back("pedestal");
	//	safety_links_name_with_collision_.push_back("right_hand");
	//	safety_links_name_with_collision_.push_back("right_lower_elbow");
	//	safety_links_name_with_collision_.push_back("right_lower_forearm");
	//	safety_links_name_with_collision_.push_back("right_lower_shoulder");
	//	safety_links_name_with_collision_.push_back("right_upper_elbow");
	//	safety_links_name_with_collision_.push_back("right_upper_elbow_visual");
	//	safety_links_name_with_collision_.push_back("right_upper_forearm");
	//	safety_links_name_with_collision_.push_back("right_upper_forearm_visual");
	//	safety_links_name_with_collision_.push_back("right_upper_shoulder");
	//	safety_links_name_with_collision_.push_back("right_wrist");
	//	safety_links_name_with_collision_.push_back("torso");

	if (planning_context_->getGroupName().compare("right_arm") == 0 )
	{
		safety_links_name_[0] = "right_lower_shoulder";
		safety_links_name_[1] = "right_lower_elbow";
		safety_links_name_[2] = "right_lower_forearm";

		specific_collision_links_.push_back("right_hand_accelerometer");
		specific_collision_links_.push_back("right_hand_camera");
		specific_collision_links_.push_back("right_hand_range");


		safety_links_name_cc_[0].push_back("right_upper_shoulder");
		safety_links_name_cc_[0].push_back("right_lower_shoulder");
		safety_links_name_cc_[0].push_back("right_upper_elbow");
		safety_links_name_cc_[0].push_back("right_upper_elbow_visual");

		safety_links_name_cc_[1].push_back("right_lower_elbow");
		safety_links_name_cc_[1].push_back("right_upper_forearm");
		safety_links_name_cc_[1].push_back("right_upper_forearm_visual");

		safety_links_name_cc_[2].push_back("right_lower_forearm");
		safety_links_name_cc_[2].push_back("right_wrist");
		safety_links_name_cc_[2].push_back("right_hand");
		safety_links_name_cc_[2].push_back("right_gripper_base");
		safety_links_name_cc_[2].push_back("right_gripper");
		safety_links_name_cc_[2].push_back("right_hand_accelerometer");
		safety_links_name_cc_[2].push_back("right_hand_camera");
		safety_links_name_cc_[2].push_back("right_hand_range");

	}
	else if (planning_context_->getGroupName().compare("left_arm") == 0 )
	{
		safety_links_name_[0] = "left_lower_shoulder";
		safety_links_name_[1] = "left_lower_elbow";
		safety_links_name_[2] = "left_lower_forearm";

		specific_collision_links_.push_back("left_hand_accelerometer");
		specific_collision_links_.push_back("left_hand_camera");
		specific_collision_links_.push_back("left_hand_range");

	}

	L_max_.resize(NB_SAFETY_LINKS);
	angle_max_.resize(NB_SAFETY_LINKS);

	//	//Baxter hardware specifications
	//	L_max_[0] = 0.37082 + 0.069 + 0.069;
	//	L_max_[1] = 0.37429;
	//	L_max_[2] = 0.229525 + 0.015355; // + camera

//	//Collision links
//	//lower_shoulder radius,upper elbow length, upper elbow visual length, lower_elbow radius (twice)
//	L_max_[0] = std::sqrt(std::pow(0.06+0.107+0.273+0.06,2)+std::pow(0.06,2)); //0.5035
//	L_max_[1] = 0.37429;
//	L_max_[2] = 0.2331 + 0.1; // + hand range added + gripper with fingers

//    //Collision links using URDF model of collision + 0.01 margin
//    L_max_[0] = 0.370895668 + 0.069 + 0.01;
//    L_max_[1] = 0.37442 + 0.01;
//    L_max_[2] = 0.343 + 0.01;

    //Collision links where lower_elbow belongs to link 1 instead of 0 and lower_forearm belongs to link 2 instead of 1
    L_max_[0] = 0.069+0.107+0.273;
    L_max_[1] = 0.069+0.088+0.272;
    L_max_[2] = 0.01 + 0.343;

	angle_max_[0] = 0;
	angle_max_[1] = M_PI/2;
	angle_max_[2] = M_PI/2;

	std::vector<std::string> joint_name_singularity;
	//	joint_name_singularity.push_back("right_s1");
	joint_name_singularity.push_back("right_e0");
	joint_name_singularity.push_back("right_e1");
	joint_name_singularity.push_back("right_w0");
	joint_name_singularity.push_back("right_w1");

	std::string homepath = getenv("HOME");
	std::fstream file((homepath + "/ros/hyperspheres.txt").c_str(), std::ios_base::in);


	if (file.is_open())
	{

		std::string line;

		while ( getline (file,line) )
		{
			Hypersphere sphere(joint_name_singularity);

			std::stringstream stream(line);
			std::vector<double> vec;
			double d1,d2;
			while (stream >> d1 && stream >> d2)
			{
				sphere.center_.push_back(d1);
				sphere.radius_.push_back(d2);

			}
			hypersphere_.push_back(sphere);


		}
		file.close();

	}

	std::vector<size_t> user_ids = getUserIDs();
	std::vector<std::string> co_names = safe_collision_world_fcl_->getCollisionObjectNames();
	for(size_t i=0; i < co_names.size(); ++i)
	{
		if (co_names[i].find("human_eye_gaze") != std::string::npos)
		{
			std::string delimiter = "_";
			std::string s = co_names[i];
			size_t pos = 0;
			while ((pos = s.find(delimiter)) != std::string::npos) {
				s.erase(0, pos + delimiter.length());
			}

			std::stringstream ss;
			ss << s;
			int user_id;
			ss >> user_id;


			if (std::find(user_ids.begin(), user_ids.end(), user_id)!=user_ids.end())
			{
				fcl::Quaternion3f fcl_q = fcl_collision_obj_[i]->getTransform().getQuatRotation();
				fcl::Vec3f fcl_pos = fcl_collision_obj_[i]->getTransform().getTranslation();
				Eigen::Quaterniond q(fcl_q.getW(), fcl_q.getX(), fcl_q.getY(), fcl_q.getZ());
				Eigen::Vector3d pos(fcl_pos.data[0], fcl_pos.data[1], fcl_pos.data[2]);
				Eigen::Vector3d scale(1,1,1);
				Eigen::Affine3d aff;
				aff.fromPositionOrientationScale(pos,q,scale);

				human_eye_gaze_.push_back(aff);
			}

		}

	}

	if (human_eye_gaze_.size() > 0)
		human_presence_ = true;
	else
		human_presence_ = false;

//	moveit::core::FixedTransformsMap transform = planning_context_->getPlanningScene()->getTransforms().getAllTransforms();
//	for (moveit::core::FixedTransformsMap::iterator it = transform.begin(); it != transform.end(); ++it)
//	{
//		if (it->first.find("human_eye_gaze") != std::string::npos)
//		{
//			std::string delimiter = "_";
//			std::string s = it->first;
//			size_t pos = 0;
//			while ((pos = s.find(delimiter)) != std::string::npos) {
//				s.erase(0, pos + delimiter.length());
//			}
//
//			std::stringstream ss;
//			ss << s;
//			int user_id;
//			ss >> user_id;
//
//
//			if (std::find(user_ids.begin(), user_ids.end(), user_id)!=user_ids.end())
//				human_eye_gaze_.push_back(it->second);
//
//
//		}
//	}
}

std::vector<double> ompl_interface::SafeStateValidityChecker::getCollisionObjectsFactors()
{
	std::vector<std::string> co_names = safe_collision_world_fcl_->getCollisionObjectNames();
	std::vector<double> co_factor(co_names.size());
	for(size_t i=0; i < co_names.size(); ++i)
	{
		if (co_names[i].find("head") != std::string::npos)
			co_factor[i] = 0.5;
		else if (co_names[i].find("torso") != std::string::npos)
			co_factor[i] = 0.6;
		else if (co_names[i].find("human") != std::string::npos)
			co_factor[i] = 0.8;
		else
			co_factor[i] = 1;
	}

	return co_factor;
}

std::vector<size_t> ompl_interface::SafeStateValidityChecker::getUserIDs()
{
	std::vector<size_t> user_ids;

	std::vector<std::string> co_names = safe_collision_world_fcl_->getCollisionObjectNames();
	for(size_t i=0; i < co_names.size(); ++i)
	{
		if (co_names[i].find("head") != std::string::npos)
		{
			std::string delimiter = "_";

			size_t pos = 0;
			while ((pos = co_names[i].find(delimiter)) != std::string::npos) {
				co_names[i].erase(0, pos + delimiter.length());
			}

			std::stringstream ss;
			ss << co_names[i];
			int user_id;
			ss >> user_id;

			user_ids.push_back(user_id);

		}

	}

	return user_ids;
}

bool ompl_interface::SafeStateValidityChecker::isValidSelf(const ompl::base::State *state) const
{
	if (state->as<ModelBasedStateSpace::StateType>()->isValidityKnown())
		return state->as<ModelBasedStateSpace::StateType>()->isMarkedValid();

	if (!si_->satisfiesBounds(state))
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}

	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	// check path constraints
	const kinematic_constraints::KinematicConstraintSetPtr &kset = planning_context_->getPathConstraints();
	if (kset && !kset->decide(*kstate).satisfied)
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}

	// check feasibility
	if (!planning_context_->getPlanningScene()->isStateFeasible(*kstate))
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}

	// check self collision avoidance
	collision_detection::CollisionResult res;
	//STa
	//	planning_context_->getPlanningScene()->checkSelfCollision(collision_request_simple_, res, *kstate);



	safe_collision_robot_fcl_padded_->checkSelfCollision(collision_request_simple_, res, *kstate, planning_context_->getPlanningScene()->getAllowedCollisionMatrix());

	//	for(collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin();
	//	    it != res.contacts.end();
	//	    ++it)
	//	{
	//	  ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
	//	}

	if (res.collision == false)
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markValid();
		return true;
	}
	else
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}
}

bool ompl_interface::SafeStateValidityChecker::isValidApprox(const ompl::base::State *state) const
{
	if (state->as<ModelBasedStateSpace::StateType>()->isValidityKnown())
		return state->as<ModelBasedStateSpace::StateType>()->isMarkedValid();

	if (!si_->satisfiesBounds(state))
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}

	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	// check path constraints
	const kinematic_constraints::KinematicConstraintSetPtr &kset = planning_context_->getPathConstraints();
	if (kset && !kset->decide(*kstate).satisfied)
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}

	// check feasibility
	if (!planning_context_->getPlanningScene()->isStateFeasible(*kstate))
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}

	collision_detection::CollisionResult res;
	planning_context_->getPlanningScene()->checkSelfCollision(collision_request_simple_, res, *kstate);
	if (res.collision)
		return false;

	double factor;
	if (computeRobotMinObstacleDistIndividualLinks(kstate, true, factor) > COLLISION_DIST)
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markValid();
		return true;
	}

	else
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}

	return false;
}

bool ompl_interface::SafeStateValidityChecker::isValid(const ompl::base::State *state) const
{
	if (state->as<ModelBasedStateSpace::StateType>()->isValidityKnown())
		return state->as<ModelBasedStateSpace::StateType>()->isMarkedValid();

	if (!si_->satisfiesBounds(state))
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}

	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	// check path constraints
	const kinematic_constraints::KinematicConstraintSetPtr &kset = planning_context_->getPathConstraints();
	if (kset && !kset->decide(*kstate).satisfied)
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}

	// check feasibility
	if (!planning_context_->getPlanningScene()->isStateFeasible(*kstate))
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}

	// check collision avoidance
	collision_detection::CollisionResult res;
	planning_context_->getPlanningScene()->checkCollision(collision_request_simple_, res, *kstate);
	if (res.collision == false)
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markValid();
		return true;
	}
	else
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}
}

bool ompl_interface::SafeStateValidityChecker::isValid(const ompl::base::State *state, double &dist) const
{
	if (state->as<ModelBasedStateSpace::StateType>()->isValidityKnown())
		return state->as<ModelBasedStateSpace::StateType>()->isMarkedValid();

	if (!si_->satisfiesBounds(state))
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}

	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	// check path constraints
	const kinematic_constraints::KinematicConstraintSetPtr &kset = planning_context_->getPathConstraints();
	if (kset && !kset->decide(*kstate).satisfied)
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}

	// check feasibility
	if (!planning_context_->getPlanningScene()->isStateFeasible(*kstate))
	{
		const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
		return false;
	}


	// check collision avoidance
	collision_detection::CollisionResult res;
	planning_context_->getPlanningScene()->checkCollision( collision_request_with_distance_, res, *kstate);
	dist = res.distance;
	return res.collision == false;


}



void ompl_interface::SafeStateValidityChecker::getCollidingLinksFCL(std::vector<std::string> &links, std::vector<double>& colliding_joint_values, const ompl::base::State *state) const
{
	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	colliding_joint_values.resize(active_joints_.size());
	for (size_t i = 0; i < colliding_joint_values.size(); ++i)
	{
		colliding_joint_values[i] = kstate->getVariablePosition(active_joints_[i]->getName());
	}

	planning_context_->getPlanningScene()->getCollidingLinks(links, *kstate);
}


void ompl_interface::SafeStateValidityChecker::computeInitialDistDataObstacle(const ompl::base::State *s1, const ompl::base::State *s2,  std::vector<std::vector<double> >& dist_s1_obs, std::vector<std::vector<double> >& dist_s2_obs, bool fast_dist)
{
	//	//STa temp
	//	ROS_WARN("Enter computeInitialApproxDistDataObstacle");

	robot_state::RobotState *ks1 = tsss1_.getStateStorage();
	robot_state::RobotState *ks2 = tsss2_.getStateStorage();

	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks1, s1);
	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks2, s2);

	dist_s1_obs.resize(nb_safety_links_);
	dist_s2_obs.resize(nb_safety_links_);


	std::vector<double> joints_diff = computeJointsDiff(ks1, ks2);

	for(size_t i=0; i < nb_safety_links_; ++i)
	{
		dist_s1_obs[i].resize(nb_objects_);
		dist_s2_obs[i].resize(nb_objects_);

		for(size_t j=0; j < nb_objects_; ++j)
		{
			if (fast_dist)
			{
				dist_s1_obs[i][j] = computeLinkApproxMinObstacleDist(ks1, i, j);
				dist_s2_obs[i][j] = computeLinkApproxMinObstacleDist(ks2, i, j);
			}
			else
			{
				dist_s1_obs[i][j] = computeLinkExactMinObstacleDist(ks1, i, j);
				dist_s2_obs[i][j] = computeLinkExactMinObstacleDist(ks2, i, j);
			}


		}
	}
}


void ompl_interface::SafeStateValidityChecker::computeInitialDistDataSelf(const ompl::base::State *s1, const ompl::base::State *s2,  std::vector<double>& dist_s1_self, std::vector<double>& dist_s2_self)
{
	robot_state::RobotState *ks1 = tsss1_.getStateStorage();
	robot_state::RobotState *ks2 = tsss2_.getStateStorage();

	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks1, s1);
	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks2, s2);


	dist_s1_self.resize(nb_safety_links_);
	dist_s2_self.resize(nb_safety_links_);

	for(size_t i=0; i < nb_safety_links_; ++i)
	{
		std::vector<std::string> other_link_names;
		for (size_t j=0; j < safety_links_name_with_collision_.size(); ++j)
		{
			if(std::find(safety_links_name_cc_[i].begin(), safety_links_name_cc_[i].end(), safety_links_name_with_collision_[j]) == safety_links_name_cc_[i].end())
			{
				other_link_names.push_back(safety_links_name_with_collision_[j]);
			}
		}

		dist_s1_self[i] = safe_collision_robot_fcl_unpadded_->distanceSelf(*ks1, &planning_context_->getPlanningScene()->getAllowedCollisionMatrix(), safety_links_name_cc_[i], other_link_names);
		dist_s2_self[i] = safe_collision_robot_fcl_unpadded_->distanceSelf(*ks2, &planning_context_->getPlanningScene()->getAllowedCollisionMatrix(), safety_links_name_cc_[i], other_link_names);

		//STa temp
		//		ROS_WARN_STREAM("dist_s1_self = " << dist_s1_self[i] << "; dist_s2_self = " << dist_s2_self[i]);
	}
}

void ompl_interface::SafeStateValidityChecker::computeInitialDistDataTravel(const ompl::base::State *s1, const ompl::base::State *s2, std::vector<double>& dist_travel)
{
	robot_state::RobotState *ks1 = tsss1_.getStateStorage();
	robot_state::RobotState *ks2 = tsss2_.getStateStorage();

	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks1, s1);
	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks2, s2);

	dist_travel.resize(nb_safety_links_);

	std::vector<double> joints_diff = computeJointsDiff(ks1, ks2);

	for(size_t i=0; i < nb_safety_links_; ++i)
	{
		dist_travel[i] = computeTravelDist(ks1, ks2, i, joints_diff);
	}
}

void ompl_interface::SafeStateValidityChecker::computeInitialDistDataTravelModulation(const ompl::base::State *s1, const ompl::base::State *s2,  std::vector<double>& dist_travel)
{
	robot_state::RobotState *ks1 = tsss1_.getStateStorage();
	robot_state::RobotState *ks2 = tsss2_.getStateStorage();

	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks1, s1);
	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks2, s2);

	std::vector<double> joints_diff = computeJointsDiff(ks1, ks2);
	std::vector<double> joints_mod = computeJointsModulation(ks1, ks2);

	dist_travel.resize(nb_safety_links_);

	for(size_t i=0; i < nb_safety_links_; ++i)
	{
		dist_travel[i] = computeTravelDist(ks1, ks2, i, joints_diff, joints_mod);;
	}

//	//STa temp
//	std::string homepath = getenv("HOME");
//	std::ofstream output_file_((homepath + "/computeInitialDistDataTravelModulation.txt").c_str(), std::ios::out | std::ios::app);
//	if (output_file_)
//	{
//	    output_file_ << "state 1 : \n";
//	    si_->printState(s1, output_file_);
//	    output_file_ << "state 2 : \n";
//	    si_->printState(s2, output_file_);
//	    for (int i=0; i< joints_diff.size(); ++i)
//	    {
//	        output_file_ << "joints_diff " << i << " = " << joints_diff[i] <<  "\n";
//
//	    }
//        for (int i=0; i< joints_mod.size(); ++i)
//        {
//            output_file_ << "joints_mod " << i << " = " << joints_mod[i] <<  "\n";
//
//        }
//        for (int i=0; i< dist_travel.size(); ++i)
//        {
//            output_file_ << "dist_travel " << i << " = " << dist_travel[i] <<  "\n";
//
//        }
//	    output_file_ <<  "\n \n";
//	    output_file_.close();
//	}
}

double ompl_interface::SafeStateValidityChecker::computeDistTravelModulation(const ompl::base::State *s1, const ompl::base::State *s2, size_t link_index)
{
	robot_state::RobotState *ks1 = tsss1_.getStateStorage();
	robot_state::RobotState *ks2 = tsss2_.getStateStorage();

	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks1, s1);
	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks2, s2);

	//TODO: Compute only for useful joints
	std::vector<double> joints_diff = computeJointsDiff(ks1, ks2);
	std::vector<double> joints_mod = computeJointsModulation(ks1, ks2);


//		//STa temp
//		double d_temp = computeTravelDist(ks1, ks2, link_index, joints_diff, joints_mod);
//		std::string homepath = getenv("HOME");
//		std::ofstream output_file_((homepath + "/computeDistTravelModulation.txt").c_str(), std::ios::out | std::ios::app);
//		if (output_file_)
//		{
//		    output_file_ << "link_index = " << link_index << "\n";
//		    output_file_ << "state 1 : \n";
//		    si_->printState(s1, output_file_);
//		    output_file_ << "state 2 : \n";
//		    si_->printState(s2, output_file_);
//			for (int i=0; i< link_index; ++i)
//			{
//				output_file_
//				<< "joints_diff " << 2*i << " = " << joints_diff[2*i] <<  "\n"
//				<< "joints_diff " << 2*i+1 << " = " << joints_diff[2*i+1] <<  "\n"
//				<< "joints_mod " << i << " = " << joints_mod[i] <<  "\n";
//			}
//			output_file_
//			<< "TravelDist " << d_temp <<  "\n \n";
//			output_file_.close();
//		}
//		return d_temp;

	return computeTravelDist(ks1, ks2, link_index, joints_diff, joints_mod);
}

double ompl_interface::SafeStateValidityChecker::computeMinDistFromSphere(robot_state::RobotState *ks1, robot_state::RobotState *ks2, Hypersphere sphere) const
{
	size_t sphere_dim = sphere.joint_name_.size();

	Eigen::VectorXd v1(sphere_dim),v2(sphere_dim), s(sphere_dim);

	for (size_t i=0; i < sphere_dim; ++i)
	{
		v1[i] = ks1->getVariablePosition(sphere.joint_name_[i]);
		v2[i] = ks2->getVariablePosition(sphere.joint_name_[i]);
		s[i] = sphere.center_[i];
	}

	Eigen::VectorXd v12 = v2 - v1;
	Eigen::VectorXd v1_sphere = s - v1;
	Eigen::VectorXd v2_sphere = s - v2;

	double d = 0;
	double d_temp;

	double c1 =  v1_sphere.dot(v12);
	if ( c1 <= 0 )
	{
		for (size_t i=0; i < sphere_dim; ++i)
		{
			if (std::abs(double(v1_sphere[i]))  > sphere.radius_[i])
			{
				d += std::pow(std::abs(double(v1_sphere[i])) - sphere.radius_[i], 2);
			}
		}
		d = std::sqrt(d);

		return d;
	}

	double c2 = v12.dot(v12);
	if ( c2 <= c1 )
	{
		for (size_t i=0; i < sphere_dim; ++i)
		{
			if (std::abs(double(v2_sphere[i])) > sphere.radius_[i])
			{
				d += std::pow(std::abs(double(v2_sphere[i])) - sphere.radius_[i], 2);
			}
		}
		d = std::sqrt(d);

		return d;
	}


	double t = c1 / c2 ;
	Eigen::VectorXd nearest_point_sphere = s - (v1 + t * v12) ;

	for (size_t i=0; i < sphere_dim; ++i)
	{
		if (std::abs(double(nearest_point_sphere[i])) > sphere.radius_[i])
		{
			d += std::pow(std::abs(double(nearest_point_sphere[i])) - sphere.radius_[i], 2);
		}
	}
	d = std::sqrt(d);

	return d;
}



double ompl_interface::SafeStateValidityChecker::computeMinDistFromSphere(robot_state::RobotState *kstate, Hypersphere sphere) const
{
	//STa temp
	//	std::string homepath = getenv("HOME");
	//	std::ofstream output_file((homepath + "/computeMinDistFromSphere.txt").c_str(), std::ios::out | std::ios::app);

	size_t sphere_dim = sphere.joint_name_.size();
	Eigen::VectorXd v(sphere_dim), s(sphere_dim);

	for (size_t i=0; i < sphere_dim; ++i)
	{
		v[i] = kstate->getVariablePosition(sphere.joint_name_[i]);
		s[i] = sphere.center_[i];
		//		output_file << "v[" << i <<"] = " << v[i] << "; s[" << i <<"] = " << s[i] << "; r = " << sphere.radius_[i] <<"\n";
	}

	Eigen::VectorXd v_sphere = s - v;

	double d = 0;

	for (size_t i=0; i < sphere_dim; ++i)
	{
		if (std::abs(double(v_sphere[i]))  > sphere.radius_[i])
		{
			d += std::pow(std::abs(double(v_sphere[i])) - sphere.radius_[i], 2);
		}
	}

	d = std::sqrt(d);

	//	output_file << "distance = " << d << "\n \n";

	return d;
}


double ompl_interface::SafeStateValidityChecker::computeRobotMinObstacleDistIndividualLinks(const ompl::base::State *state,  bool fast_dist, double& object_danger_factor) const
{
	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	return computeRobotMinObstacleDistIndividualLinks(kstate, fast_dist, object_danger_factor);
}


double ompl_interface::SafeStateValidityChecker::computeRobotMinObstacleDistIndividualLinks(const robot_state::RobotState *kstate, bool fast_dist, double& object_danger_factor) const
{
	float min_dist = std::numeric_limits<float>::infinity();
	float temp_dist;
	for (size_t i = 0; i < NB_SAFETY_LINKS; ++i)
	{
		temp_dist = computeLinkMinObstacleDist(kstate, i, fast_dist);
		if (temp_dist < min_dist)
		{
			min_dist = temp_dist;
			object_danger_factor = obj_danger_factor_[i];
		}
		if (min_dist <= 0)
		{
			return  0;
		}
	}

	return min_dist;
}

double ompl_interface::SafeStateValidityChecker::computeLinkMinObstacleDist(const robot_state::RobotState *kstate, int link_index, bool fast_dist) const
{
	float min_dist = std::numeric_limits<float>::infinity();
	double temp_dist;

	for (size_t object_index = 0; object_index < fcl_collision_obj_.size(); ++object_index)
	{
		if (fast_dist)
			temp_dist = computeLinkApproxMinObstacleDist(kstate, link_index, object_index);
		else
			temp_dist = computeLinkExactMinObstacleDist(kstate, link_index, object_index);

//		//STa temp
//		std::string homepath = getenv("HOME");
//		std::ofstream output_file((homepath + "/computeLinkMinObstacleDist.txt").c_str(), std::ios::out | std::ios::app);
//		ros::Time t1 = ros::Time::now();
//		double d1;
//		for (size_t i = 0 ; i < 1000 ; ++i)
//			d1 = computeLinkApproxMinObstacleDist(kstate, link_index, object_index);
//		ros::Time t2 = ros::Time::now();
//		double d2;
//		for (size_t i = 0 ; i < 1000 ; ++i)
//			d2 = computeLinkExactMinObstacleDist(kstate, link_index, object_index);
//		ros::Time t3 = ros::Time::now();
//		output_file << fcl_collision_obj_[object_index]->getNodeType() << "\n";
//		output_file << d1 << "  " << (t2-t1).toNSec() << "\n";
//		output_file << d2 << "  " << (t3-t2).toNSec() << "\n \n";
//		output_file.close();



		if (temp_dist < min_dist)
		{
			min_dist = temp_dist;
		}

		if (min_dist <= 0)
		{
			return  0;
		}
	}
	return min_dist;
}



double ompl_interface::SafeStateValidityChecker::computeLinkApproxMinObstacleDist(const robot_state::RobotState *kstate, int link_index, int object_index) const
{
	//TODO : Get point on the link that minimizes distance to obstacle
	//       Currently, only origin of the link is considered

	//	ROS_WARN_STREAM("Enter computeLinkMinObstacleDist");

	//STa : Box object or mesh object bounding box
//	if (fcl_collision_obj_[object_index]->getObjectType() == fcl::OBJECT_TYPE::OT_BVH || fcl_collision_obj_[object_index]->getNodeType() == fcl::NODE_TYPE::GEOM_BOX)
//	{
		return computeLinkApproxMinBoxDist(kstate, link_index, object_index);
//	}
//	else if (fcl_collision_obj_[object_index]->getNodeType() == fcl::NODE_TYPE::GEOM_SPHERE)
//	{
//		return computeLinkApproxMinSphereDist(kstate, link_index, object_index);
//	}
//	else if (fcl_collision_obj_[object_index]->getNodeType() == fcl::NODE_TYPE::GEOM_CYLINDER)
//	{
//		return computeLinkApproxMinCylinderDist(kstate, link_index, object_index);
//	}
//	else return -1;


}


double ompl_interface::SafeStateValidityChecker::computeLinkApproxMinBoxDist(const robot_state::RobotState *kstate, int link_index, int object_index) const
{
	float min_dist = std::numeric_limits<float>::infinity();
	double temp_dist;
	double sample_length;
	int nb_sample;

	Eigen::Vector3d box_position(fcl_collision_obj_[object_index]->getTranslation().data.vs);

	for(size_t i = 0; i < safety_links_name_cc_[link_index].size(); ++i)
	{
		for(size_t i2 = 0; i2 < kstate->getLinkModel(safety_links_name_cc_[link_index][i])->getShapes().size(); ++i2)
		{

			shapes::ShapeConstPtr link_shape = kstate->getLinkModel(safety_links_name_cc_[link_index][i])->getShapes()[i2];
			const EigenSTL::vector_Affine3d link_offset_STL = kstate->getLinkModel(safety_links_name_cc_[link_index][i])->getCollisionOriginTransforms();

			Eigen::Vector3d link_offset = link_offset_STL[i2].translation().transpose() * link_offset_STL[i2].rotation();
			Eigen::Vector3d link_position = kstate->getGlobalLinkTransform(safety_links_name_cc_[link_index][i]).translation();
			Eigen::Matrix3d link_rotation = kstate->getGlobalLinkTransform(safety_links_name_cc_[link_index][i]).rotation()* link_offset_STL[i2].rotation();

			if (link_shape->type == shapes::CYLINDER)
			{
				//TODO : Compute this data once
				const shapes::Cylinder* cylinder = static_cast<const shapes::Cylinder*>(link_shape.get());
				sample_length = 2*cylinder->radius;
				nb_sample = std::ceil((cylinder->length) / sample_length);
				double adjusted_sample_length = (cylinder->length)/(nb_sample);
				Eigen::Vector3d cylinder_offset, sample_step;
				cylinder_offset << 0, 0, cylinder->length / 2;
				sample_step << 0, 0, adjusted_sample_length;

				double sample_radius = std::sqrt(std::pow(cylinder->radius,2) + std::pow(adjusted_sample_length/2,2));

				for (size_t j = 0; j < nb_sample; ++j)
				{

					Eigen::Vector3d sample_offset = -cylinder_offset + link_offset + j * sample_step + (sample_step/2);
					Eigen::Vector3d sample_position = link_position + link_rotation * sample_offset;

					//If obstacle is too far, we can iterate.
					double max_center_dist = std::sqrt(std::pow(fcl_collision_obj_[object_index]->getAABB().depth(), 2) + std::pow(fcl_collision_obj_[object_index]->getAABB().height(), 2)+ std::pow(fcl_collision_obj_[object_index]->getAABB().width(),2));
					if((box_position - sample_position).norm() - max_center_dist - sample_radius < min_dist)
					{
						fcl::Transform3f Id;
						Id.setIdentity();
						fcl::Transform3f inverse_transform = fcl_collision_obj_[object_index]->getTransform().inverseTimes(Id);

						fcl::Vec3f sample_position_fcl(sample_position[0], sample_position[1], sample_position[2]);
						fcl::Vec3f sample_position_in_box_frame = inverse_transform.transform(sample_position_fcl);


						fcl::Vec3f max_in_box_frame = fcl_collision_obj_[object_index]->collisionGeometry()->aabb_local.max_;
						fcl::Vec3f min_in_box_frame = fcl_collision_obj_[object_index]->collisionGeometry()->aabb_local.min_;


						fcl::Vec3f box_nearest_point_in_box_frame;

						for(size_t k=0; k < 3; ++k)
						{
							if (sample_position_in_box_frame[k] < min_in_box_frame[k])
								box_nearest_point_in_box_frame[k] = min_in_box_frame[k];
							else if (sample_position_in_box_frame[k] > max_in_box_frame[k])
								box_nearest_point_in_box_frame[k] = max_in_box_frame[k];
							else
								box_nearest_point_in_box_frame[k] = sample_position_in_box_frame[k];
						}

						fcl::Vec3f box_nearest_point_fcl = fcl_collision_obj_[object_index]->getTransform().transform(box_nearest_point_in_box_frame);
						Eigen::Vector3d box_nearest_point(box_nearest_point_fcl.data.vs);
						temp_dist = (box_nearest_point - sample_position).norm();

						temp_dist -= sample_radius;

						if (temp_dist < min_dist)
						{
							min_dist = temp_dist;
						}

						if (min_dist < 0)
						{
							//				ROS_WARN_STREAM("Exit computeLinkMinObstacleDist");
							return  -1;
						}
					}
				}
			}
			else if (link_shape->type == shapes::BOX)
			{
				//TODO : sample_step always equals 0

				const shapes::Box* box = static_cast<const shapes::Box*>(link_shape.get());

				std::vector<std::pair<size_t, double> > box_size;
				box_size.push_back(std::pair<size_t, double> (0, box->size[0]));
				box_size.push_back(std::pair<size_t, double> (1, box->size[1]));
				box_size.push_back(std::pair<size_t, double> (2, box->size[2]));
				std::sort(box_size.begin(), box_size.end(),
						boost::bind(&std::pair<size_t, double>::second, _1) >
				boost::bind(&std::pair<size_t, double>::second, _2));

				sample_length = 2*box_size[1].second;
				nb_sample = std::ceil(box_size[0].second / sample_length);
				double adjusted_sample_length = (box_size[0].second)/(nb_sample);
				Eigen::Vector3d box_offset, sample_step;
				box_offset << ((box_size[0].first == 0) ? box_size[0].second/2 : 0), ((box_size[0].first == 1) ? box_size[0].second/2 : 0), ((box_size[0].first == 2) ? box_size[0].second/2 : 0);
				sample_step << ((box_size[0].first == 0) ? adjusted_sample_length : 0), ((box_size[0].first == 1) ? adjusted_sample_length : 0), ((box_size[0].first == 2) ? adjusted_sample_length : 0);

				double sample_radius;
				if (box->size[0] == box_size[0].second)
					sample_radius = std::sqrt(std::pow(adjusted_sample_length/2,2) + std::pow(box->size[1],2)  + std::pow(box->size[2],2));
				else if (box->size[1] == box_size[0].second)
					sample_radius = std::sqrt(std::pow(adjusted_sample_length/2,2) + std::pow(box->size[0],2)  + std::pow(box->size[2],2));
				else
					sample_radius = std::sqrt(std::pow(adjusted_sample_length/2,2) + std::pow(box->size[0],2)  + std::pow(box->size[1],2));

				for (size_t j = 0; j < nb_sample; ++j)
				{

					Eigen::Vector3d sample_offset = -box_offset + link_offset + j * sample_step + (sample_step/2);
					Eigen::Vector3d sample_position = link_position + link_rotation * sample_offset;

					//If obstacle is too far, we can iterate.
					double max_center_dist = std::sqrt(std::pow(fcl_collision_obj_[object_index]->getAABB().depth(), 2) + std::pow(fcl_collision_obj_[object_index]->getAABB().height(), 2)+ std::pow(fcl_collision_obj_[object_index]->getAABB().width(),2));
					if((box_position - sample_position).norm() - max_center_dist - sample_radius < min_dist)
					{
						fcl::Transform3f Id;
						Id.setIdentity();
						fcl::Transform3f inverse_transform = fcl_collision_obj_[object_index]->getTransform().inverseTimes(Id);

						fcl::Vec3f sample_position_fcl(sample_position[0], sample_position[1], sample_position[2]);
						fcl::Vec3f sample_position_in_box_frame = inverse_transform.transform(sample_position_fcl);


						fcl::Vec3f max_in_box_frame = fcl_collision_obj_[object_index]->collisionGeometry()->aabb_local.max_;
						fcl::Vec3f min_in_box_frame = fcl_collision_obj_[object_index]->collisionGeometry()->aabb_local.min_;


						fcl::Vec3f box_nearest_point_in_box_frame;

						for(size_t k=0; k < 3; ++k)
						{
							if (sample_position_in_box_frame[k] < min_in_box_frame[k])
								box_nearest_point_in_box_frame[k] = min_in_box_frame[k];
							else if (sample_position_in_box_frame[k] > max_in_box_frame[k])
								box_nearest_point_in_box_frame[k] = max_in_box_frame[k];
							else
								box_nearest_point_in_box_frame[k] = sample_position_in_box_frame[k];
						}

						fcl::Vec3f box_nearest_point_fcl = fcl_collision_obj_[object_index]->getTransform().transform(box_nearest_point_in_box_frame);
						Eigen::Vector3d box_nearest_point(box_nearest_point_fcl.data.vs);
						temp_dist = (box_nearest_point - sample_position).norm();

						temp_dist -= sample_radius;

						if (temp_dist < min_dist)
						{
							min_dist = temp_dist;
						}

						if (min_dist < 0)
						{
							//				ROS_WARN_STREAM("Exit computeLinkMinObstacleDist");
							return  -1;
						}
					}
				}
			}
			//TODO: If shape == sphere
		}
	}
	//	ROS_WARN_STREAM("Exit computeLinkMinObstacleDist");
	return min_dist;
}

double ompl_interface::SafeStateValidityChecker::computeLinkApproxMinSphereDist(const robot_state::RobotState *kstate, int link_index, int object_index) const
{
	float min_dist = std::numeric_limits<float>::infinity();
		double temp_dist;
		double sample_length;
		int nb_sample;

		Eigen::Vector3d sphere_position(fcl_collision_obj_[object_index]->getTranslation().data.vs);

		for(size_t i = 0; i < safety_links_name_cc_[link_index].size(); ++i)
		{
			for(size_t i2 = 0; i2 < kstate->getLinkModel(safety_links_name_cc_[link_index][i])->getShapes().size(); ++i2)
			{

				shapes::ShapeConstPtr link_shape = kstate->getLinkModel(safety_links_name_cc_[link_index][i])->getShapes()[i2];
				const EigenSTL::vector_Affine3d link_offset_STL = kstate->getLinkModel(safety_links_name_cc_[link_index][i])->getCollisionOriginTransforms();

				Eigen::Vector3d link_offset = link_offset_STL[i2].translation().transpose() * link_offset_STL[i2].rotation();
				Eigen::Vector3d link_position = kstate->getGlobalLinkTransform(safety_links_name_cc_[link_index][i]).translation();
				Eigen::Matrix3d link_rotation = kstate->getGlobalLinkTransform(safety_links_name_cc_[link_index][i]).rotation();

				if (link_shape->type == shapes::CYLINDER)
				{
					//TODO : Compute this data once
					const shapes::Cylinder* cylinder = static_cast<const shapes::Cylinder*>(link_shape.get());


					sample_length = 2*cylinder->radius;
					nb_sample = std::ceil((cylinder->length) / sample_length);
					double adjusted_sample_length = (cylinder->length)/(nb_sample);
					Eigen::Vector3d cylinder_offset, sample_step;
					cylinder_offset << 0, 0, cylinder->length / 2;
					sample_step << 0, 0, adjusted_sample_length;

					double sample_radius = std::sqrt(std::pow(cylinder->radius,2) + std::pow(adjusted_sample_length/2,2));

					for (size_t j = 0; j < nb_sample; ++j)
					{

						Eigen::Vector3d sample_offset = -cylinder_offset + link_offset + j * sample_step + (sample_step/2);
						Eigen::Vector3d sample_position = link_position + link_rotation * sample_offset;

						//If obstacle is too far, we can iterate.
						double max_center_dist = std::sqrt(std::pow(fcl_collision_obj_[object_index]->getAABB().depth(), 2) + std::pow(fcl_collision_obj_[object_index]->getAABB().height(), 2)+ std::pow(fcl_collision_obj_[object_index]->getAABB().width(),2));
						if((sphere_position - sample_position).norm() - max_center_dist - sample_radius < min_dist)
						{
							fcl::Transform3f Id;
							Id.setIdentity();
							fcl::Transform3f inverse_transform = fcl_collision_obj_[object_index]->getTransform().inverseTimes(Id);

							fcl::Vec3f sample_position_fcl(sample_position[0], sample_position[1], sample_position[2]);
							fcl::Vec3f sample_position_in_box_frame = inverse_transform.transform(sample_position_fcl);


							fcl::Vec3f max_in_box_frame = fcl_collision_obj_[object_index]->collisionGeometry()->aabb_local.max_;
							fcl::Vec3f min_in_box_frame = fcl_collision_obj_[object_index]->collisionGeometry()->aabb_local.min_;


							fcl::Vec3f box_nearest_point_in_box_frame;

							for(size_t k=0; k < 3; ++k)
							{
								if (sample_position_in_box_frame[k] < min_in_box_frame[k])
									box_nearest_point_in_box_frame[k] = min_in_box_frame[k];
								else if (sample_position_in_box_frame[k] > max_in_box_frame[k])
									box_nearest_point_in_box_frame[k] = max_in_box_frame[k];
								else
									box_nearest_point_in_box_frame[k] = sample_position_in_box_frame[k];
							}

							fcl::Vec3f box_nearest_point_fcl = fcl_collision_obj_[object_index]->getTransform().transform(box_nearest_point_in_box_frame);
							Eigen::Vector3d box_nearest_point(box_nearest_point_fcl.data.vs);
							temp_dist = (box_nearest_point - sample_position).norm();

							temp_dist -= sample_radius;

							if (temp_dist < min_dist)
							{
								min_dist = temp_dist;
							}

							if (min_dist < 0)
							{
								//				ROS_WARN_STREAM("Exit computeLinkMinObstacleDist");
								return  -1;
							}
						}
					}
				}
				else if (link_shape->type == shapes::BOX)
				{
					//TODO : sample_step always equals 0

					const shapes::Box* box = static_cast<const shapes::Box*>(link_shape.get());

					std::vector<std::pair<size_t, double> > box_size;
					box_size.push_back(std::pair<size_t, double> (0, box->size[0]));
					box_size.push_back(std::pair<size_t, double> (1, box->size[1]));
					box_size.push_back(std::pair<size_t, double> (2, box->size[2]));
					std::sort(box_size.begin(), box_size.end(),
							boost::bind(&std::pair<size_t, double>::second, _1) >
					boost::bind(&std::pair<size_t, double>::second, _2));

					sample_length = 2*box_size[1].second;
					nb_sample = std::ceil(box_size[0].second / sample_length);
					double adjusted_sample_length = (box_size[0].second)/(nb_sample);
					Eigen::Vector3d box_offset, sample_step;
					box_offset << ((box->size[0] == box_size[0].first) ? box_size[0].second/2 : 0), ((box->size[1] == box_size[0].first) ? box_size[0].second/2 : 0), ((box->size[2] == box_size[0].first) ? box_size[0].second/2 : 0);
					sample_step << ((box->size[0] == box_size[0].first) ? adjusted_sample_length : 0), ((box->size[1] == box_size[0].first) ? adjusted_sample_length : 0), ((box->size[2] == box_size[0].first) ? adjusted_sample_length : 0);

					double sample_radius;
					if (box->size[0] == box_size[0].second)
						sample_radius = std::sqrt(std::pow(adjusted_sample_length/2,2) + std::pow(box->size[1],2)  + std::pow(box->size[2],2));
					else if (box->size[1] == box_size[0].second)
						sample_radius = std::sqrt(std::pow(adjusted_sample_length/2,2) + std::pow(box->size[0],2)  + std::pow(box->size[2],2));
					else
						sample_radius = std::sqrt(std::pow(adjusted_sample_length/2,2) + std::pow(box->size[0],2)  + std::pow(box->size[1],2));

					for (size_t j = 0; j < nb_sample; ++j)
					{

						Eigen::Vector3d sample_offset = -box_offset + link_offset + j * sample_step + (sample_step/2);
						Eigen::Vector3d sample_position = link_position + link_rotation * sample_offset;

						//If obstacle is too far, we can iterate.
						double max_center_dist = std::sqrt(std::pow(fcl_collision_obj_[object_index]->getAABB().depth(), 2) + std::pow(fcl_collision_obj_[object_index]->getAABB().height(), 2)+ std::pow(fcl_collision_obj_[object_index]->getAABB().width(),2));
						if((sphere_position - sample_position).norm() - max_center_dist - sample_radius < min_dist)
						{
							fcl::Transform3f Id;
							Id.setIdentity();
							fcl::Transform3f inverse_transform = fcl_collision_obj_[object_index]->getTransform().inverseTimes(Id);

							fcl::Vec3f sample_position_fcl(sample_position[0], sample_position[1], sample_position[2]);
							fcl::Vec3f sample_position_in_box_frame = inverse_transform.transform(sample_position_fcl);


							fcl::Vec3f max_in_box_frame = fcl_collision_obj_[object_index]->collisionGeometry()->aabb_local.max_;
							fcl::Vec3f min_in_box_frame = fcl_collision_obj_[object_index]->collisionGeometry()->aabb_local.min_;


							fcl::Vec3f box_nearest_point_in_box_frame;

							for(size_t k=0; k < 3; ++k)
							{
								if (sample_position_in_box_frame[k] < min_in_box_frame[k])
									box_nearest_point_in_box_frame[k] = min_in_box_frame[k];
								else if (sample_position_in_box_frame[k] > max_in_box_frame[k])
									box_nearest_point_in_box_frame[k] = max_in_box_frame[k];
								else
									box_nearest_point_in_box_frame[k] = sample_position_in_box_frame[k];
							}

							fcl::Vec3f box_nearest_point_fcl = fcl_collision_obj_[object_index]->getTransform().transform(box_nearest_point_in_box_frame);
							Eigen::Vector3d box_nearest_point(box_nearest_point_fcl.data.vs);
							temp_dist = (box_nearest_point - sample_position).norm();

							temp_dist -= sample_radius;

							if (temp_dist < min_dist)
							{
								min_dist = temp_dist;
							}

							if (min_dist < 0)
							{
								//				ROS_WARN_STREAM("Exit computeLinkMinObstacleDist");
								return  -1;
							}
						}
					}
				}
				//TODO: If shape == sphere
			}
		}
		//	ROS_WARN_STREAM("Exit computeLinkMinObstacleDist");
		return min_dist;
}
double ompl_interface::SafeStateValidityChecker::computeLinkApproxMinCylinderDist(const robot_state::RobotState *kstate, int link_index, int object_index) const
{
	return 0;
}

double ompl_interface::SafeStateValidityChecker::computeLinkExactMinObstacleDist(const robot_state::RobotState *kstate, int link_index, int object_index) const
{
	return safe_collision_world_fcl_->distanceRobot(safe_collision_robot_fcl_unpadded_, *kstate,  &planning_context_->getPlanningScene()->getAllowedCollisionMatrix(), safety_links_name_cc_[link_index], object_index);
}

double ompl_interface::SafeStateValidityChecker::computeLinkMinObstacleDist(const ompl::base::State *state, int link_index, int object_index, bool fast_dist)
{
	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

//		//STa test
//		std::string homepath = getenv("HOME");
////		std::ofstream output_file((homepath + "/min_obs_dist_time.txt").c_str(), std::ios::out | std::ios::app);
//		std::ofstream output_file_2((homepath + "/min_obs_dist_result.txt").c_str(), std::ios::out | std::ios::app);
////		ompl::time::point init = ompl::time::now();
//		double approx = computeLinkApproxMinObstacleDist(kstate, link_index, object_index);
//		if (approx < 0)
//			approx = 0;
////		ompl::time::duration dur1 = ompl::time::now() - init;
//		double exact = computeLinkExactMinObstacleDist(kstate, link_index, object_index);
//		if (exact < 0)
//			exact = 0;
////		ompl::time::duration dur2 = ompl::time::now() - init - dur1;
////		output_file << ompl::time::seconds(dur1) << "  " << ompl::time::seconds(dur2) << "\n";
//		if (approx > exact + 0.0001)
//		    output_file_2 << approx << "  " << exact << "  " << link_index << "\n";
//		return approx;



	if (fast_dist)
		return computeLinkApproxMinObstacleDist(kstate, link_index, object_index);
	else
		return computeLinkExactMinObstacleDist(kstate, link_index, object_index);

}


double ompl_interface::SafeStateValidityChecker::computeLinkMinSelfDist(const ompl::base::State *state, int link_index)
{
	//	ROS_WARN("Enter computeLinkMinSelfDist");

	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	std::vector<std::string> other_link_names;
	for (size_t i=0; i < safety_links_name_with_collision_.size(); ++i)
	{
		if(std::find(safety_links_name_cc_[link_index].begin(), safety_links_name_cc_[link_index].end(), safety_links_name_with_collision_[i]) == safety_links_name_cc_[link_index].end())
		{
			other_link_names.push_back(safety_links_name_with_collision_[i]);
		}
	}

	//STa temp
	//	ROS_WARN("Flag1");
	double d = safe_collision_robot_fcl_unpadded_->distanceSelf(*kstate, &planning_context_->getPlanningScene()->getAllowedCollisionMatrix(), safety_links_name_cc_[link_index], other_link_names);
	//	ROS_WARN("Exit computeLinkMinSelfDist");
	return d;

	//	return safe_collision_robot_fcl_unpadded_->distanceSelf(*kstate, &planning_context_->getPlanningScene()->getAllowedCollisionMatrix(), safety_links_name_cc_[link_index], other_link_names);
}

std::vector<double> ompl_interface::SafeStateValidityChecker::computeJointsDiff(const robot_state::RobotState *ks1, const robot_state::RobotState *ks2) const
{
	std::vector<double> joints_diff(2*NB_SAFETY_LINKS);
	for (size_t i = 0; i < joints_diff.size(); ++i)
	{
		joints_diff[i] = computeJointsDiff(ks1, ks2, i);
	}
	return joints_diff;

}

double ompl_interface::SafeStateValidityChecker::computeJointsDiff(const robot_state::RobotState *ks1, const robot_state::RobotState *ks2, size_t joint_index) const
{

	return std::abs(ks1->getVariablePosition(active_joints_[joint_index]->getName()) - ks2->getVariablePosition(active_joints_[joint_index]->getName()));
}

std::vector<double> ompl_interface::SafeStateValidityChecker::computeJointsModulation(const robot_state::RobotState *ks1, const robot_state::RobotState *ks2) const
{
	std::vector<double> joints_modulation(NB_SAFETY_LINKS);

	for (size_t i = 0; i < joints_modulation.size(); ++i)
	{
		joints_modulation[i] = computeJointsModulation(ks1, ks2, i);
	}

	return joints_modulation;
}

double ompl_interface::SafeStateValidityChecker::computeJointsModulation(const robot_state::RobotState *ks1, const robot_state::RobotState *ks2, size_t link_index) const
{
	double ang_min, ang_max, dist, center;

	ang_min = std::min(ks1->getVariablePosition(active_joints_[2*link_index+1]->getName()) - angle_max_[link_index], ks2->getVariablePosition(active_joints_[2*link_index+1]->getName())) - angle_max_[link_index];
	ang_max = std::max(ks1->getVariablePosition(active_joints_[2*link_index+1]->getName()) - angle_max_[link_index], ks2->getVariablePosition(active_joints_[2*link_index+1]->getName())) - angle_max_[link_index];

	dist = ang_max - ang_min;

	if (dist > M_PI)
		return 1;

	center = (ang_max + ang_min)/2;

	//center expressed between 0 and PI as the modulation is modulo PI
	if (center >= 0)
	{
		while (center > M_PI)
			center -= M_PI;
	}
	else
	{
		while (center < 0)
			center += M_PI;
	}

//    //STa temp
//    std::string homepath = getenv("HOME");
//    std::ofstream output_file_((homepath + "/computeJointsModulation.txt").c_str(), std::ios::out | std::ios::app);
//    double result;
//    if ( (center + dist/2 >=0  && center - dist/2 <= 0) || (center + dist/2 >= M_PI  && center - dist/2 <= M_PI ))
//        result= 1;
//    else if (center > M_PI/2)
//        result= cos(M_PI - (center + dist/2));
//    else
//        result= cos((center - dist/2));
//    if (output_file_)
//    {
//        output_file_
//        << "angle_max_ = " << angle_max_[link_index] <<  "\n"
//        << "ang_min = " << ang_min <<  "\n"
//        << "ang_max = " << ang_max <<  "\n"
//        << "dist = " << dist <<  "\n"
//        << "center = " << center <<  "\n"
//        << "joints_modulation = " << result <<  "\n \n";
//        output_file_.close();
//    }
//    return result;

	if ( (center + dist/2 >=0  && center - dist/2 <= 0) || (center + dist/2 >= M_PI  && center - dist/2 <= M_PI ))
		return 1;
	else if (center > M_PI/2)
		return cos(M_PI - (center + dist/2));
	else
		return cos((center - dist/2));



}


double ompl_interface::SafeStateValidityChecker::computeTravelDist(const robot_state::RobotState *ks1, const robot_state::RobotState *ks2, size_t link_index, std::vector<double> joints_diff, std::vector<double> joints_modulation) const
{
	double travel_dist = 0;
	std::vector<double> L_max = L_max_;

	//	//STa temp
	//	double travel_dist_2 = 0;
	//	double lenght_2 = 0;

	//Worst case method
	//	for (int i = link_index; i >= 0; i--)
	//	{
	//		lenght += L_max_[i];
	//		travel_dist += lenght * (joints_diff[i*2]*joints_modulation[i] + joints_diff[i*2+1] );
	//	}

//	if (link_index < 2)
//	    lenght += 0.0781; //We consider the farthest point of the link -> we add the distance between the center of the cylinder and the farthest point in it.

	if (link_index < 2)
	    L_max[link_index] = std::sqrt(std::pow(L_max[link_index],2)+std::pow(0.06,2));

//	//Interpolation method
//	for (int i = link_index; i >= 0; i--)
//	{
//		travel_dist += lenght * (std::sqrt(std::pow(joints_diff[i*2]*joints_modulation[i],2) + std::pow(joints_diff[i*2+1],2)));
//		if (i > 0)
//		    lenght += L_max_[i-1];
//	}

	//Try
    for (int i = 0; i <= link_index; i++)
    {
        travel_dist += L_max[i]* (std::sqrt(std::pow(joints_diff[i*2]*joints_modulation[i],2) + std::pow(joints_diff[i*2+1],2)));

        for (int j=i+1; j <= link_index; ++j)
        {
            travel_dist += L_max[j]* (std::sqrt(std::pow(joints_diff[i*2],2) + std::pow(joints_diff[i*2+1],2)));
        }
    }


	//STa test temp
	//		std::string homepath = getenv("HOME");
	//		std::ofstream output_file_((homepath + "/travel_compare.txt").c_str(), std::ios::out | std::ios::app);
	//		if (output_file_)
	//		{
	//			output_file_
	//			<< "travel_dist_1 " << travel_dist <<  "\n"
	//			<< "travel_dist_2 " << travel_dist_2 <<  "\n \n";
	//			output_file_.close();
	//		}

	return travel_dist;
}

double ompl_interface::SafeStateValidityChecker::computeTravelDist(const robot_state::RobotState *ks1, const robot_state::RobotState *ks2, size_t link_index, std::vector<double> joints_diff) const
{
	double travel_dist = 0;
	double lenght = 0;

	if (link_index < 2)
	        lenght += 0.0781; //We consider the farthest point of the link -> we add the distance between the center of the cylinder and the farthest point in it.

	//Interpolation method
	for (int i = link_index; i >= 0; i--)
	{
		lenght += L_max_[i];
		travel_dist += lenght * (std::sqrt(std::pow(joints_diff[i*2],2) + std::pow(joints_diff[i*2+1],2)));
	}

	return travel_dist;
}

double ompl_interface::SafeStateValidityChecker::computeRobotExactMinObstacleDist(const ompl::base::State *state) const
{
	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	//	return computeRobotExactMinObstacleDist(kstate);

	double temp_dist, min_dist = 10000000;
	for (size_t i=0; i<3; ++i)
	{
		for (size_t j = 0; j < fcl_collision_obj_.size(); ++j)
		{
			temp_dist = computeLinkExactMinObstacleDist(kstate,i,j);
			if (temp_dist < min_dist)
				min_dist = temp_dist;
		}
	}
	return min_dist;
}

double ompl_interface::SafeStateValidityChecker::computeRobotExactMinObstacleDist(const robot_state::RobotState *kstate) const
{
	//	return planning_context_->getPlanningScene()->getCollisionWorld()->distanceRobot(*(planning_context_->getPlanningScene()->getCollisionRobot()), *kstate, planning_context_->getPlanningScene()->getAllowedCollisionMatrix(), planning_context_->getGroupName() );
	//	return planning_context_->getPlanningScene()->getCollisionWorld()->distanceRobot(*(planning_context_->getPlanningScene()->getCollisionRobot()), *kstate, planning_context_->getPlanningScene()->getAllowedCollisionMatrix());

	return safe_collision_world_fcl_->distanceRobot(*safe_collision_robot_fcl_unpadded_, *kstate,  planning_context_->getPlanningScene()->getAllowedCollisionMatrix(), planning_context_->getGroupName());

}


double ompl_interface::SafeStateValidityChecker::manipulability(const ompl::base::State *state) const
{
//	//STa temp
//	std::string homepath = getenv("HOME");
//	std::ofstream output_file((homepath + "/manipulability.txt").c_str(), std::ios::out | std::ios::app);

	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	Eigen::MatrixXd jacobian = kstate->getJacobian(planning_context_->getPlanningScene()->getRobotModel()->getJointModelGroup(planning_context_->getGroupName()));

	//Method 1
	Eigen::MatrixXd product = jacobian*jacobian.transpose();
	double mom1 = std::sqrt(product.determinant());

//	output_file << mom1 << "  ";

	//Method 2
	//	double mom2 = jacobian.jacobiSvd(0).singularValues().prod();

//	//STa temp
//	double min_dist = std::numeric_limits<double>::infinity();
//	double temp_dist;
//
//	for(size_t i=0; i < hypersphere_.size(); ++i)
//	{
//		temp_dist = computeMinDistFromSphere(kstate, hypersphere_[i]);
//		if (temp_dist < min_dist)
//			min_dist = temp_dist;
//	}
//	output_file << min_dist << "\n";
//	output_file.close();

	return mom1;
}

double ompl_interface::SafeStateValidityChecker::manipulability(const ompl::base::State* s1, const ompl::base::State* s2)
{
	robot_state::RobotState *ks1 = tsss1_.getStateStorage();
	robot_state::RobotState *ks2 = tsss2_.getStateStorage();

	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks1, s1);
	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks2, s2);

	double min_dist = std::numeric_limits<double>::infinity();
	double temp_dist;

	for(size_t i=0; i < hypersphere_.size(); ++i)
	{
		temp_dist = computeMinDistFromSphere(ks1, ks2, hypersphere_[i]);
		if (temp_dist < min_dist)
			min_dist = temp_dist;
	}
	return min_dist;
}

double ompl_interface::SafeStateValidityChecker::humanAwareness(const ompl::base::State *state) const
{
	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	Eigen::Affine3d end_effector_transform;

	if (planning_context_->getGroupName().compare("right_arm") == 0 )
		end_effector_transform = kstate->getGlobalLinkTransform("right_gripper");
	else if (planning_context_->getGroupName().compare("left_arm") == 0 )
		end_effector_transform = kstate->getGlobalLinkTransform("left_gripper");
	else
		return 0; //STa TODO : Add the case when the two arms are moving

	double worst_value = 0;

	for(size_t i=0; i < human_eye_gaze_.size(); ++i)
	{
		Eigen::Vector3d hum_to_eef = end_effector_transform.translation() - human_eye_gaze_[i].translation();
		if(hum_to_eef.norm() < SAFETY_DISTANCE)
		{

			Eigen::Vector3d hum_direction = human_eye_gaze_[i] * Eigen::Vector3d(1, 0, 0)- human_eye_gaze_[i].translation();
			double value = Eigen::Quaterniond::FromTwoVectors(hum_to_eef,hum_direction).angularDistance( Eigen::Quaterniond::Identity());

			//The human gaze eye direction (defined as the angular distance) is weighted by the the distance to eef
//			value /= hum_to_eef.norm();

			if (value > worst_value)
				worst_value = value;
		}
	}

	return worst_value;
}


std::vector<double> ompl_interface::SafeStateValidityChecker::getJointValues(const ompl::base::State *state) const
{
	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	std::vector<double> q(active_joints_.size());
	for (size_t i=0; i < active_joints_.size(); ++i)
	{
		q[i] = kstate->getVariablePosition(active_joints_[i]->getName());
	}
	return q;
}


void ompl_interface::SafeStateValidityChecker::getJointLimits(std::vector<double>& q_min, std::vector<double>& q_max) const
{
	q_min.resize(active_joints_.size());
	q_max.resize(active_joints_.size());
	for (size_t i=0; i < active_joints_.size(); ++i)
	{
		q_min[i] = planning_context_->getRobotModel()->getVariableBounds(active_joints_[i]->getName()).min_position_;
		q_max[i] = planning_context_->getRobotModel()->getVariableBounds(active_joints_[i]->getName()).max_position_;
	}
}

void ompl_interface::SafeStateValidityChecker::printStatePositions(const ompl::base::State *state,std::ostream &out) const
{

	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	kstate->printStatePositions(out);
}

