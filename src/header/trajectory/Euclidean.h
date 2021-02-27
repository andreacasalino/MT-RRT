///**
// * Author:    Andrea Casalino
// * Created:   16.05.2019
// *
// * report any bug to andrecasa91@gmail.com.
// **/
//
//#ifndef MT_RRT_TRAJECTORY_EUCLIDEAN_H
//#define MT_RRT_TRAJECTORY_EUCLIDEAN_H
//
//#include <trajectory/Manager.h>
//
//namespace mt::traj {
//    class Euclidean : public Manager {
//    public:
//        float cost2Go(const NodeState& start, const NodeState& ending_node, const bool& ignoreConstraints) override;
//
//    protected:
//        TrajectoryPtr getTrajectory(const NodeState& start, const NodeState& trg) override;
//
//        ProblemEuclidean(SamplerPtr sampler, CheckerPtr checker, const std::size_t& stateSpaceSize, const float& gamma, const float& steerDegree);
//
//    private:
//        float steerDegree;
//    };
//}
//
//#endif