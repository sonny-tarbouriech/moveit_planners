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

/* Author: Sonny Tarbouriech */

#include <moveit/ompl_interface/detail/safe_state_validity_checker.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/profiler/profiler.h>
#include <ros/ros.h>

//STa
#include "geometric_shapes/shape_operations.h"
#include <moveit/collision_detection_fcl/collision_world_fcl.h>

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
	safe_collision_robot_fcl_padded_ = new collision_detection::SafeCollisionRobotFCL(planning_context_->getRobotModel(), 0.03, 1);

	human_eye_gaze_.clear();

	safe_collision_world_fcl_ = static_cast<const collision_detection::SafeCollisionWorldFCL*> (planning_context_->getPlanningScene()->getCollisionWorld().get());

	collision_request_with_distance_.distance = true;

	collision_request_simple_.group_name = planning_context_->getGroupName();
	collision_request_with_distance_.group_name = planning_context_->getGroupName();

	std::vector<boost::shared_ptr<fcl::CollisionObject> > fcl_collision_obj = safe_collision_world_fcl_->getCollisionObjects();
	std::vector<std::string> obj_names = safe_collision_world_fcl_->getCollisionObjectNames();
	std::vector<double> obj_danger_factor =  getCollisionObjectsFactors(obj_names);
	std::vector<size_t> user_ids = getUserIDs(obj_names);

	fcl_obj_.resize(fcl_collision_obj.size());

	for(size_t i=0; i < fcl_collision_obj.size(); ++i)
	{
		fcl_obj_[i].fcl_collision_obj_= fcl_collision_obj[i];
		fcl_obj_[i].obj_names_= obj_names[i];
		fcl_obj_[i].obj_danger_factor_= obj_danger_factor[i];
	}


	for(size_t i=0; i < fcl_obj_.size(); ++i)
	{
		if (fcl_obj_[i].obj_names_.find("human_eye_gaze") != std::string::npos)
		{
			std::string delimiter = "_";
			std::string s = fcl_obj_[i].obj_names_;
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
				fcl::Quaternion3f fcl_q = fcl_obj_[i].fcl_collision_obj_->getTransform().getQuatRotation();
				fcl::Vec3f fcl_pos = fcl_obj_[i].fcl_collision_obj_->getTransform().getTranslation();
				Eigen::Quaterniond q(fcl_q.getW(), fcl_q.getX(), fcl_q.getY(), fcl_q.getZ());
				Eigen::Vector3d pos(fcl_pos.data[0], fcl_pos.data[1], fcl_pos.data[2]);
				Eigen::Vector3d scale(1,1,1);
				Eigen::Affine3d aff;
				aff.fromPositionOrientationScale(pos,q,scale);

				human_eye_gaze_.push_back(aff);
			}

			fcl_obj_.erase(fcl_obj_.begin() + i);

		}

	}

	for (size_t i=0; i < fcl_obj_.size(); ++i)
	{
		if(fcl_obj_[i].fcl_collision_obj_->getObjectType() == 3)
		{
			const fcl::OcTree* tree = static_cast<const fcl::OcTree*>(fcl_obj_[i].fcl_collision_obj_->collisionGeometry().get());
			bool null;
			std::vector<std::pair<fcl::Box*, fcl::Transform3f> > boxes;
			generateBoxesFromOctomapRecurse(boxes, tree, tree->getRoot(), tree->getRootBV(), 0.15, null);
			for (size_t j=0; j < boxes.size(); ++j)
			{
				boost::shared_ptr<fcl::CollisionGeometry> cg(boxes[j].first);
				fcl::CollisionObject* obj = new fcl::CollisionObject(cg, boxes[j].second);
				boost::shared_ptr<fcl::CollisionObject> co(obj);
				ocTree_boxes_.push_back(co);
			}

			OMPL_INFORM("SafeStateValidityChecker: Created %d collision boxes from octomap", ocTree_boxes_.size());

			fcl_obj_.erase(fcl_obj_.begin() + i);

			for (size_t j=0; j < ocTree_boxes_.size(); ++j)
				fcl_obj_.insert(fcl_obj_.begin() + i + j, FCLObject(ocTree_boxes_[j],"ocTree_box" + j,1));

			i += ocTree_boxes_.size() - 1;

		}
	}

	active_joints_ = planning_context_->getJointModelGroup()->getActiveJointModels();

	safety_links_name_cc_.resize(NB_SAFETY_LINKS);
	safety_links_name_with_collision_ = planning_context_->getRobotModel()->getLinkModelNamesWithCollisionGeometry();


	if (planning_context_->getGroupName().compare("right_arm") == 0 )
	{
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
	    safety_links_name_cc_[2].push_back("r_gripper_l_finger");
	    safety_links_name_cc_[2].push_back("r_gripper_l_finger_tip");
	    safety_links_name_cc_[2].push_back("r_gripper_r_finger");
	    safety_links_name_cc_[2].push_back("r_gripper_r_finger_tip");
		safety_links_name_cc_[2].push_back("right_hand_accelerometer");
		safety_links_name_cc_[2].push_back("right_hand_camera");
		safety_links_name_cc_[2].push_back("right_hand_range");
	}
	else if (planning_context_->getGroupName().compare("left_arm") == 0 )
	{
		safety_links_name_cc_[0].push_back("left_upper_shoulder");
		safety_links_name_cc_[0].push_back("left_lower_shoulder");
		safety_links_name_cc_[0].push_back("left_upper_elbow");
		safety_links_name_cc_[0].push_back("left_upper_elbow_visual");

		safety_links_name_cc_[1].push_back("left_lower_elbow");
		safety_links_name_cc_[1].push_back("left_upper_forearm");
		safety_links_name_cc_[1].push_back("left_upper_forearm_visual");

		safety_links_name_cc_[2].push_back("left_lower_forearm");
		safety_links_name_cc_[2].push_back("left_wrist");
		safety_links_name_cc_[2].push_back("left_hand");
		safety_links_name_cc_[2].push_back("left_gripper_base");
		safety_links_name_cc_[2].push_back("left_gripper");
	    safety_links_name_cc_[2].push_back("l_gripper_l_finger");
	    safety_links_name_cc_[2].push_back("l_gripper_l_finger_tip");
	    safety_links_name_cc_[2].push_back("l_gripper_r_finger");
	    safety_links_name_cc_[2].push_back("l_gripper_r_finger_tip");
		safety_links_name_cc_[2].push_back("left_hand_accelerometer");
		safety_links_name_cc_[2].push_back("left_hand_camera");
		safety_links_name_cc_[2].push_back("left_hand_range");
	}

	L_max_.resize(NB_SAFETY_LINKS);
	angle_max_.resize(NB_SAFETY_LINKS);

    //Collision links where lower_elbow belongs to link 1 instead of 0 and lower_forearm belongs to link 2 instead of 1
	Eigen::Vector3d r_l_shoulder_pos = planning_context_->getCompleteInitialRobotState().getGlobalLinkTransform("right_lower_shoulder").translation();
	Eigen::Vector3d r_l_elbow_pos = planning_context_->getCompleteInitialRobotState().getGlobalLinkTransform("right_lower_elbow").translation();
	Eigen::Vector3d r_l_forearm_pos = planning_context_->getCompleteInitialRobotState().getGlobalLinkTransform("right_lower_forearm").translation();
	Eigen::Vector3d r_gripper_r_finger_tip_pos = planning_context_->getCompleteInitialRobotState().getGlobalLinkTransform("r_gripper_r_finger_tip").translation();

	L_max_[0] = planning_context_->getRobotModel()->getLinkModel("right_lower_shoulder")->getJointOriginTransform ().translation()[0];
	L_max_[0] += (r_l_shoulder_pos-r_l_elbow_pos).norm(); //~0.440
	L_max_[1] = (r_l_elbow_pos-r_l_forearm_pos).norm(); //~0.374
	L_max_[2] = (r_l_forearm_pos-r_gripper_r_finger_tip_pos).norm(); //~0.389

	angle_max_[0] = 0;
	angle_max_[1] = M_PI/2;
	angle_max_[2] = M_PI/2;
}

std::vector<double> ompl_interface::SafeStateValidityChecker::getCollisionObjectsFactors(std::vector<std::string> co_names)
{
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

std::vector<size_t> ompl_interface::SafeStateValidityChecker::getUserIDs(std::vector<std::string> co_names)
{
	std::vector<size_t> user_ids;

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

	safe_collision_robot_fcl_padded_->checkSelfCollision(collision_request_simple_, res, *kstate, planning_context_->getPlanningScene()->getAllowedCollisionMatrix());

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
	robot_state::RobotState *ks1 = tsss1_.getStateStorage();
	robot_state::RobotState *ks2 = tsss2_.getStateStorage();

	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks1, s1);
	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks2, s2);

	dist_s1_obs.resize(NB_SAFETY_LINKS);
	dist_s2_obs.resize(NB_SAFETY_LINKS);


	std::vector<double> joints_diff = computeJointsDiff(ks1, ks2);

	for(size_t i=0; i < NB_SAFETY_LINKS; ++i)
	{
		dist_s1_obs[i].resize(fcl_obj_.size());
		dist_s2_obs[i].resize(fcl_obj_.size());

		for(size_t j=0; j < fcl_obj_.size(); ++j)
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


	dist_s1_self.resize(NB_SAFETY_LINKS);
	dist_s2_self.resize(NB_SAFETY_LINKS);

	for(size_t i=0; i < NB_SAFETY_LINKS; ++i)
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
	}
}

void ompl_interface::SafeStateValidityChecker::computeInitialDistDataTravel(const ompl::base::State *s1, const ompl::base::State *s2, std::vector<double>& dist_travel)
{
	robot_state::RobotState *ks1 = tsss1_.getStateStorage();
	robot_state::RobotState *ks2 = tsss2_.getStateStorage();

	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks1, s1);
	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks2, s2);

	dist_travel.resize(NB_SAFETY_LINKS);

	std::vector<double> joints_diff = computeJointsDiff(ks1, ks2);

	for(size_t i=0; i < NB_SAFETY_LINKS; ++i)
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

	dist_travel.resize(NB_SAFETY_LINKS);

	for(size_t i=0; i < NB_SAFETY_LINKS; ++i)
	{
		dist_travel[i] = computeTravelDist(ks1, ks2, i, joints_diff, joints_mod);;
	}
}

double ompl_interface::SafeStateValidityChecker::computeDistTravelModulation(const ompl::base::State *s1, const ompl::base::State *s2, size_t link_index)
{
	robot_state::RobotState *ks1 = tsss1_.getStateStorage();
	robot_state::RobotState *ks2 = tsss2_.getStateStorage();

	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks1, s1);
	planning_context_->getOMPLStateSpace()->copyToRobotState(*ks2, s2);

	std::vector<double> joints_diff = computeJointsDiff(ks1, ks2);
	std::vector<double> joints_mod = computeJointsModulation(ks1, ks2);

	return computeTravelDist(ks1, ks2, link_index, joints_diff, joints_mod);
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
	double object_danger_factor_temp;
	for (size_t i = 0; i < NB_SAFETY_LINKS; ++i)
	{
		temp_dist = computeLinkMinObstacleDist(kstate, i, fast_dist, object_danger_factor_temp);
		if (temp_dist < min_dist)
		{
			min_dist = temp_dist;
			object_danger_factor = object_danger_factor_temp;
		}
		if (min_dist <= 0)
		{
			return  0;
		}
	}

	return min_dist;
}

double ompl_interface::SafeStateValidityChecker::computeLinkMinObstacleDist(const robot_state::RobotState *kstate, int link_index, bool fast_dist, double& object_danger_factor) const
{
	float min_dist = std::numeric_limits<float>::infinity();
	double temp_dist;

	for (size_t object_index = 0; object_index < fcl_obj_.size(); ++object_index)
	{
		if (fast_dist)
			temp_dist = computeLinkApproxMinObstacleDist(kstate, link_index, object_index);
		else
			temp_dist = computeLinkExactMinObstacleDist(kstate, link_index, object_index);

		if (temp_dist < min_dist)
		{
			min_dist = temp_dist;
			object_danger_factor = getObjectDangerFactor(object_index);
		}

		if (min_dist <= 0)
		{
			return  0;
		}
	}
	return min_dist;
}

void ompl_interface::SafeStateValidityChecker::generateBoxesFromOctomap(std::vector<fcl::CollisionObject*>& boxes, const fcl::OcTree* tree) const
{
  std::vector<boost::array<fcl::FCL_REAL, 6> > boxes_ = tree->toBoxes();

  for(std::size_t i = 0; i < boxes_.size(); ++i)
  {
	  fcl::FCL_REAL x = boxes_[i][0];
	  fcl::FCL_REAL y = boxes_[i][1];
	  fcl::FCL_REAL z = boxes_[i][2];
	  fcl::FCL_REAL size = boxes_[i][3];
	  fcl::FCL_REAL cost = boxes_[i][4];
	  fcl::FCL_REAL threshold = boxes_[i][5];

	  //Keep only dangerous boxes
	  if (z < 1.5 && z > -0.5 && x < 1.5 && x > -1.5 && y < 1.5 && y > -1.5 )
	  {
		  fcl::Box* box = new fcl::Box(size, size, size);
		  box->cost_density = cost;
		  box->threshold_occupied = threshold;
		  fcl::CollisionObject* obj = new fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(box), fcl::Transform3f(fcl::Vec3f(x, y, z)));
		  boxes.push_back(obj);
	  }
  }

  std::cout << "boxes size: " << boxes.size() << std::endl;

}

double ompl_interface::SafeStateValidityChecker::generateBoxesFromOctomapRecurse(std::vector<std::pair<fcl::Box*, fcl::Transform3f> >& boxes, const fcl::OcTree* tree, const fcl::OcTree::OcTreeNode* node, const fcl::AABB& node_bv, double precision, bool& is_occupied_inside_ws) const
{

	//Keep only dangerous boxes
	bool inside_ws = std::min(node_bv.min_[2],node_bv.max_[2]) < 1.5 && std::max(node_bv.min_[2],node_bv.max_[2]) > -0.5 && std::min(node_bv.min_[1],node_bv.max_[1]) < 0.5 && std::max(node_bv.min_[1],node_bv.max_[1]) > -1.5 && std::min(node_bv.min_[0],node_bv.max_[0]) < 1.5 && std::max(node_bv.min_[0],node_bv.max_[0]) > -1.5;

	double rate;
	std::vector<std::pair<fcl::Box*, fcl::Transform3f> > boxes_child;
	if(node->hasChildren())
	{
		is_occupied_inside_ws = false;
		rate=0;
		double nb_child=0;
		for(unsigned int i = 0; i < 8; ++i)
		{
			if(node->childExists(i))
			{
				fcl::AABB child_bv;
				fcl::computeChildBV(node_bv, i, child_bv);
				bool is_child_occupied_inside_ws;
				rate += generateBoxesFromOctomapRecurse(boxes_child, tree, node->getChild(i), child_bv, precision, is_child_occupied_inside_ws);
				if (is_child_occupied_inside_ws)
					is_occupied_inside_ws = true;
			}
		}
		rate /= 8;
	}
	else
	{
		rate = node->getOccupancy();
		is_occupied_inside_ws = inside_ws && tree->isNodeOccupied(node);
	}

	if (is_occupied_inside_ws)
	{
		if (rate >= precision)
		{
			fcl::Box* box = new fcl::Box;
			fcl::Transform3f box_tf, tf(fcl::Vec3f(0, 0, 0));
			constructBox(node_bv, tf, *box, box_tf);
			boxes.push_back(std::pair<fcl::Box*, fcl::Transform3f>(box, box_tf));

			for(size_t i=0; i < boxes_child.size(); ++i)
				delete boxes_child[i].first;

		}
		else
		{
			for(size_t i=0; i < boxes_child.size(); ++i)
				boxes.push_back(boxes_child[i]);
		}
	}

	return rate;
}

double ompl_interface::SafeStateValidityChecker::getOctomapOccupancyVolume(const fcl::OcTree* tree)
{

	double volume = 0;
	std::vector<boost::array<fcl::FCL_REAL, 6> > boxes_ = tree->toBoxes();
	for(std::size_t i = 0; i < boxes_.size(); ++i)
		volume += std::pow(boxes_[i][3], 3);

	return volume;
}


double ompl_interface::SafeStateValidityChecker::computeLinkApproxMinObstacleDist(const robot_state::RobotState *kstate, int link_index, int object_index) const
{
		return computeLinkApproxMinBoxDist(kstate, link_index, fcl_obj_[object_index].fcl_collision_obj_.get(), std::numeric_limits<float>::infinity());
}


double ompl_interface::SafeStateValidityChecker::computeLinkApproxMinBoxDist(const robot_state::RobotState *kstate, int link_index, fcl::CollisionObject* object, float min_dist ) const
{
	double temp_dist;
	double sample_length;
	int nb_sample;

	Eigen::Vector3d box_position(object->getTranslation().data.vs);
	double object_max_center_dist = std::sqrt(std::pow(object->getAABB().depth(), 2) + std::pow(object->getAABB().height(), 2)+ std::pow(object->getAABB().width(),2));

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
				const shapes::Cylinder* cylinder = static_cast<const shapes::Cylinder*>(link_shape.get());

				//If obstacle is too far from the link, we can iterate.
				double cylinder_max_center_dist = std::sqrt(std::pow(cylinder->length, 2) + std::pow(cylinder->radius, 2));
				if((box_position - (link_position + link_offset)).norm() - object_max_center_dist - cylinder_max_center_dist >= min_dist)
					continue;

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

					//If obstacle is too far from the sample, we can iterate.
					if((box_position - sample_position).norm() - object_max_center_dist - sample_radius >= min_dist)
						continue;

					fcl::Transform3f Id;
					Id.setIdentity();
					fcl::Transform3f inverse_transform = object->getTransform().inverseTimes(Id);

					fcl::Vec3f sample_position_fcl(sample_position[0], sample_position[1], sample_position[2]);
					fcl::Vec3f sample_position_in_box_frame = inverse_transform.transform(sample_position_fcl);


					fcl::Vec3f max_in_box_frame = object->collisionGeometry()->aabb_local.max_;
					fcl::Vec3f min_in_box_frame = object->collisionGeometry()->aabb_local.min_;


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

					fcl::Vec3f box_nearest_point_fcl = object->getTransform().transform(box_nearest_point_in_box_frame);
					Eigen::Vector3d box_nearest_point(box_nearest_point_fcl.data.vs);
					temp_dist = (box_nearest_point - sample_position).norm();

					temp_dist -= sample_radius;

					if (temp_dist < min_dist)
						min_dist = temp_dist;

					if (min_dist < 0)
						return  -1;
				}
			}
			else if (link_shape->type == shapes::BOX)
			{
				const shapes::Box* box = static_cast<const shapes::Box*>(link_shape.get());

				//If obstacle is too far from the link, we can iterate.
				double box_max_center_dist = std::sqrt(std::pow(box->size[0], 2) + std::pow(box->size[1], 2) + std::pow(box->size[2], 2));
				if((box_position - (link_position + link_offset)).norm() - object_max_center_dist - box_max_center_dist >= min_dist)
					continue;

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
					if((box_position - sample_position).norm() - object_max_center_dist - sample_radius >= min_dist)
						continue;

					fcl::Transform3f Id;
					Id.setIdentity();
					fcl::Transform3f inverse_transform = object->getTransform().inverseTimes(Id);

					fcl::Vec3f sample_position_fcl(sample_position[0], sample_position[1], sample_position[2]);
					fcl::Vec3f sample_position_in_box_frame = inverse_transform.transform(sample_position_fcl);


					fcl::Vec3f max_in_box_frame = object->collisionGeometry()->aabb_local.max_;
					fcl::Vec3f min_in_box_frame = object->collisionGeometry()->aabb_local.min_;


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

					fcl::Vec3f box_nearest_point_fcl = object->getTransform().transform(box_nearest_point_in_box_frame);
					Eigen::Vector3d box_nearest_point(box_nearest_point_fcl.data.vs);
					temp_dist = (box_nearest_point - sample_position).norm();

					temp_dist -= sample_radius;


					if (temp_dist < min_dist)
						min_dist = temp_dist;

					if (min_dist < 0)
						return  -1;
				}
			}
		}
	}
	return min_dist;
}

double ompl_interface::SafeStateValidityChecker::computeLinkExactMinObstacleDist(const robot_state::RobotState *kstate, int link_index, int object_index) const
{
	return safe_collision_world_fcl_->distanceRobot(safe_collision_robot_fcl_unpadded_, *kstate,  &planning_context_->getPlanningScene()->getAllowedCollisionMatrix(), safety_links_name_cc_[link_index], object_index);
}

double ompl_interface::SafeStateValidityChecker::computeLinkMinObstacleDist(const ompl::base::State *state, int link_index, int object_index, bool fast_dist)
{
	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	if (fast_dist)
		return computeLinkApproxMinObstacleDist(kstate, link_index, object_index);
	else
		return computeLinkExactMinObstacleDist(kstate, link_index, object_index);
}


double ompl_interface::SafeStateValidityChecker::computeLinkMinSelfDist(const ompl::base::State *state, int link_index)
{
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

	return safe_collision_robot_fcl_unpadded_->distanceSelf(*kstate, &planning_context_->getPlanningScene()->getAllowedCollisionMatrix(), safety_links_name_cc_[link_index], other_link_names);
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

	if (link_index == 1) //We consider the farthest point of the link -> the distance is increased for this link
	    L_max[link_index] = std::sqrt(std::pow(L_max[link_index],2)+std::pow(0.06,2));

    for (int i = 0; i <= link_index; i++)
    {
        travel_dist += L_max[i]* (std::sqrt(std::pow(joints_diff[i*2]*joints_modulation[i],2) + std::pow(joints_diff[i*2+1],2)));

        for (int j=i+1; j <= link_index; ++j)
        {
            travel_dist += L_max[j]* (std::sqrt(std::pow(joints_diff[i*2],2) + std::pow(joints_diff[i*2+1],2)));
        }
    }
	return travel_dist;
}

double ompl_interface::SafeStateValidityChecker::computeTravelDist(const robot_state::RobotState *ks1, const robot_state::RobotState *ks2, size_t link_index, std::vector<double> joints_diff) const
{
	double travel_dist = 0;
	std::vector<double> L_max = L_max_;

	if (link_index == 1) //We consider the farthest point of the link -> the distance is increased for this link
	    L_max[link_index] = std::sqrt(std::pow(L_max[link_index],2)+std::pow(0.06,2));

    for (int i = 0; i <= link_index; i++)
    {
        travel_dist += L_max[i]* (std::sqrt(std::pow(joints_diff[i*2],2) + std::pow(joints_diff[i*2+1],2)));

        for (int j=i+1; j <= link_index; ++j)
        {
            travel_dist += L_max[j]* (std::sqrt(std::pow(joints_diff[i*2],2) + std::pow(joints_diff[i*2+1],2)));
        }
    }
	return travel_dist;
}

double ompl_interface::SafeStateValidityChecker::computeRobotExactMinObstacleDist(const ompl::base::State *state) const
{
	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	double temp_dist, min_dist = std::numeric_limits<double>::infinity();
	for (size_t i=0; i<3; ++i)
	{
		for (size_t j = 0; j < fcl_obj_.size(); ++j)
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
	return safe_collision_world_fcl_->distanceRobot(*safe_collision_robot_fcl_unpadded_, *kstate,  planning_context_->getPlanningScene()->getAllowedCollisionMatrix(), planning_context_->getGroupName());
}

//Manipulability objective using discrete states
double ompl_interface::SafeStateValidityChecker::manipulability(const ompl::base::State *state) const
{
	robot_state::RobotState *kstate = tsss_.getStateStorage();
	planning_context_->getOMPLStateSpace()->copyToRobotState(*kstate, state);

	Eigen::MatrixXd jacobian = kstate->getJacobian(planning_context_->getPlanningScene()->getRobotModel()->getJointModelGroup(planning_context_->getGroupName()));

	Eigen::MatrixXd product = jacobian*jacobian.transpose();
	double mom = std::sqrt(product.determinant());

	return mom;
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
