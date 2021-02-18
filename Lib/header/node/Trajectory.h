/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/
 
#ifndef MT_RRT_TRAJECTORY_H
#define MT_RRT_TRAJECTORY_H

#include <node/Node.h>

namespace mt::node {
    /** \brief This oject is used to represent an optimal trajectory \tau (Section 1.2 of the documentation), going from two states \in \mathcal{X}
    \details It used to compute the distance from that states and to move along the trajectory when doing steering operations.
    */
    class Trajectory{
    public:
        virtual ~Trajectory();

        /** \brief Returns the Cost to go of the configurations connected by this trajectory.
        \details In case a trajecctory connecting the peers of this trajectory does not exist, a FLT_MAX is returned.
        */
        virtual float cost2Go() const = 0;

        /** \brief Move along the trajectory.
        \details An internal state (gettable using Get_state_current) is set equal to the starting configuration after building this object.
        Then, by calling this method, the state is modified, advancing along the trajectory of a certain quantity.
        The old state is internally stored and is gettable using Get_state_previous.
        * @param[out] return false when the end of the trajectory is reached after calling this method (after that you cannot call again Advance).
        */
        virtual bool advance() = 0;

        /** \brief See I_trajectory::Advance.
        */
        inline const std::vector<float>& getStateCursor() {  return this->stateCursor; };

        /** \brief See I_trajectory::Advance. Returns the cost to go from the beginning of the trajectory till the state internally reached at present.
        */
        inline const float& cost2GoCumulated() { return this->Cumulated_cost; };

    protected:
        Trajectory(const Node& start, const Node& end);

    // data
        const Node&        start;
        const Node&        end;
        std::vector<float> stateCursor;
        float		       Cumulated_cost = 0.f;
    };
}

#endif