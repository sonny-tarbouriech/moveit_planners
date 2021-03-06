//STa

#ifndef MOVEIT_OMPL_INTERFACE_DETAIL_SAFE_STATE_VALIDITY_CHECKER_
#define MOVEIT_OMPL_INTERFACE_DETAIL_SAFE_STATE_VALIDITY_CHECKER_

#include <ompl/base/SafeStateValidityChecker.h>
#include <moveit/ompl_interface/detail/threadsafe_state_storage.h>

//STa
#include <moveit/collision_detection_fcl/collision_common.h>
#include <moveit/collision_detection_fcl/safe_collision_robot_fcl.h>
#include <moveit/collision_detection_fcl/safe_collision_world_fcl.h>

namespace ompl_interface
{

static const int NB_SAFETY_LINKS = 3;
static const double COLLISION_DIST = 0.001;

class ModelBasedPlanningContext;

/** @class StateValidityChecker
    @brief An interface for a OMPL state validity checker*/
class SafeStateValidityChecker : public ompl::base::SafeStateValidityChecker
{
public:


	struct FCLObject
	{
		FCLObject(): obj_names_(""), obj_danger_factor_(1)
		{}

		FCLObject(boost::shared_ptr<fcl::CollisionObject> fcl_collision_obj, std::string obj_names, double obj_danger_factor):
			fcl_collision_obj_(fcl_collision_obj),
			obj_names_(obj_names),
			obj_danger_factor_(obj_danger_factor)
		{
		}

		boost::shared_ptr<fcl::CollisionObject> fcl_collision_obj_;
		std::string obj_names_;
		double obj_danger_factor_;
	};

	SafeStateValidityChecker(const ModelBasedPlanningContext *planning_context);

  virtual bool isValid(const ompl::base::State *state) const;
  virtual bool isValid(const ompl::base::State *state, double &dist) const;
  virtual bool isValidSelf(const ompl::base::State *state) const;

  virtual void getCollidingLinksFCL(std::vector<std::string> &links, std::vector<double>& colliding_joint_values, const ompl::base::State *state) const;

  virtual void computeInitialDistDataObstacle(const ompl::base::State *s1, const ompl::base::State *s2, std::vector<std::vector<double> >& dist_s1_obs, std::vector<std::vector<double> >& dist_s2_obs, bool fast_dist);
  virtual void computeInitialDistDataSelf(const ompl::base::State *s1, const ompl::base::State *s2,  std::vector<double>& dist_s1_self, std::vector<double>& dist_s2_self);
  virtual void computeInitialDistDataTravel(const ompl::base::State *s1, const ompl::base::State *s2, std::vector<double>& dist_travel);
  virtual void computeInitialDistDataTravelModulation(const ompl::base::State *s1, const ompl::base::State *s2,  std::vector<double>& dist_travel);
  virtual double computeDistTravelModulation(const ompl::base::State *s1, const ompl::base::State *s2, size_t link_index);

  virtual double computeLinkMinObstacleDist(const ompl::base::State *state, int link_index, int object_index, bool fast_dist) ;

  virtual double computeLinkMinSelfDist(const ompl::base::State *state, int link_index);


  virtual double computeRobotMinObstacleDistIndividualLinks(const ompl::base::State *state, bool fast_dist, double& object_danger_factor) const;
  virtual double computeRobotExactMinObstacleDist(const ompl::base::State *state) const;

  virtual std::vector<double> getJointValues(const ompl::base::State *state) const;
  virtual void getJointLimits(std::vector<double>& q_min, std::vector<double>& q_max) const;

  virtual double manipulability(const ompl::base::State *state) const;
  virtual double humanAwareness(const ompl::base::State *state) const;

  virtual void printStatePositions(const ompl::base::State *state, std::ostream &out = std::cout) const;

  virtual bool humanPresence() const
  {
	  return human_eye_gaze_.size() > 0;
  }
  virtual size_t getNbSafetyLinks() const
  {
	  return NB_SAFETY_LINKS;
  }
  virtual size_t getNbObjects() const
  {
	  return fcl_obj_.size();
  }
  virtual double getObjectDangerFactor(size_t index) const
  {
	  return fcl_obj_[index].obj_danger_factor_;
  }

protected:

  std::vector<double> getCollisionObjectsFactors(std::vector<std::string> co_names);
  std::vector<size_t> getUserIDs(std::vector<std::string> co_names);

  void generateBoxesFromOctomap(std::vector<fcl::CollisionObject*>& boxes, const fcl::OcTree* tree) const;
  double generateBoxesFromOctomapRecurse(std::vector<std::pair<fcl::Box*, fcl::Transform3f> >& boxes, const fcl::OcTree* tree, const fcl::OcTree::OcTreeNode* node, const fcl::AABB& node_bv, double precision, bool& is_occupied_inside_ws) const;

  double getOctomapOccupancyVolume(const fcl::OcTree* tree);

  bool isValidApprox(const ompl::base::State *state) const;

  double computeRobotMinObstacleDistIndividualLinks(const robot_state::RobotState *kstate, bool fast_dist, double& object_danger_factor) const;
  double computeRobotExactMinObstacleDist(const robot_state::RobotState *kstate) const;

  double computeLinkMinObstacleDist(const robot_state::RobotState *kstate, int link_index, bool fast_dist, double& object_danger_factor) const;
  double computeLinkApproxMinObstacleDist(const robot_state::RobotState *kstate, int link_index, int object_index) const;
  double computeLinkExactMinObstacleDist(const robot_state::RobotState *kstate, int link_index, int object_index) const;
  double computeLinkApproxMinBoxDist(const robot_state::RobotState *kstate, int link_index, fcl::CollisionObject* object, float min_dist ) const;

  double computeTravelDist(const robot_state::RobotState *ks1, const robot_state::RobotState *ks2, size_t link_index, std::vector<double> joints_diff, std::vector<double> joints_modulation) const;
  double computeTravelDist(const robot_state::RobotState *ks1, const robot_state::RobotState *ks2, size_t link_index, std::vector<double> joints_diff) const;
  std::vector<double> computeJointsDiff(const robot_state::RobotState *ks1, const robot_state::RobotState *ks2) const;
  double computeJointsDiff(const robot_state::RobotState *ks1, const robot_state::RobotState *ks2, size_t joint_index) const;
  std::vector<double> computeJointsModulation(const robot_state::RobotState *ks1, const robot_state::RobotState *ks2) const;
  double computeJointsModulation(const robot_state::RobotState *ks1, const robot_state::RobotState *ks2, size_t link_index) const;


  std::vector<FCLObject> fcl_obj_;

  const ModelBasedPlanningContext      *planning_context_;
  std::string                           group_name_;

  TSStateStorage						tsss_;
  //STa
  TSStateStorage						tsss1_, tsss2_;

  collision_detection::CollisionRequest collision_request_simple_;
  collision_detection::CollisionRequest collision_request_with_distance_;


  std::vector<const moveit::core::JointModel*> active_joints_;


  std::vector< std::string> safety_links_name_with_collision_;
  std::vector<std::vector< std::string > > safety_links_name_cc_;

  std::vector<double> L_max_;
  std::vector<double> angle_max_;

  const collision_detection::SafeCollisionRobotFCL* safe_collision_robot_fcl_unpadded_;
  const collision_detection::SafeCollisionRobotFCL* safe_collision_robot_fcl_padded_;
  const collision_detection::SafeCollisionWorldFCL* safe_collision_world_fcl_;

  std::vector<Eigen::Affine3d> human_eye_gaze_;

  std::vector<boost::shared_ptr<fcl::CollisionObject> > ocTree_boxes_;



};

}

#endif
