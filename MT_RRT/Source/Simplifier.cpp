#include "../Header/Simplifier.h"
#include <float.h>
using namespace std;
using namespace MT_RTT;



class Waypoint : public Node{
public:
    ~Waypoint(){ this->State = nullptr; };

    Waypoint(float* state_buffer, const size_t& pos) : Node(state_buffer), position(pos) {};

    const size_t& get_pos() const { return this->position; };
private:
    size_t position;
};


class Waypoint_factory{
public:
    Waypoint* create(float* state_buffer, const size_t& pos){
        this->created.emplace_back(state_buffer, pos);
        return &this->created.back();
    };
private:
    list<Waypoint>  created;
};


class CostToGo_table{
public:
    CostToGo_table(const size_t& N_waypoints , Node::I_Node_factory* problem);
    virtual  ~CostToGo_table();

    const float& at(const Waypoint* from, const Waypoint* to);
private:
    Node::I_Node_factory*             Problem;
    std::vector<std::vector<float*>>  Table;
};

CostToGo_table::CostToGo_table(const size_t& N_waypoints, Node::I_Node_factory* problem) : Problem(problem) {

    if(N_waypoints < 3) throw 0;

    size_t S2;
    for(size_t k=1; k<N_waypoints; ++k){
        S2 = N_waypoints - k;
        this->Table.emplace_back();
        for(size_t kk = 0; kk<S2; ++kk) this->Table.back().emplace_back(nullptr);
    }
    
}

CostToGo_table::~CostToGo_table(){

    for(size_t k=0; k<this->Table.size(); ++k){
        for(size_t kk=0;kk<this->Table[k].size(); ++kk) delete this->Table[k][kk];
    }

}

const float& CostToGo_table::at(const Waypoint* from, const Waypoint* to) { 

    size_t F = from->get_pos();
    size_t T = to->get_pos();
    size_t k = T - (1 + F);
    if(this->Table[F][k] == nullptr){
        this->Table[F][k] = new float();
        if((F+1) == T) this->Problem->Cost_to_go(this->Table[F][k], from, to);
        else           this->Problem->Cost_to_go_constraints( this->Table[F][k], from, to);
    } 
    return *this->Table[F][k];

}



void simplify(std::list<Array>* wayps , Waypoint* Path){

    size_t pos_prev;
    Waypoint* cursor = Path;
//consistency checks
    if(cursor->get_pos() != (wayps->size() - 1)) throw 0;
    while (cursor->Get_Father() != nullptr) {
        pos_prev = cursor->get_pos();
        cursor = static_cast<Waypoint*>(cursor->Get_Father());
        if(cursor->get_pos() >= pos_prev) throw 1;
    }
    if(cursor->get_pos() != 0) throw 2;
    
// get the simplified incidences
    list<size_t> simple_pos;
    cursor = Path;
    while (cursor != nullptr) {
        simple_pos.push_front(cursor->get_pos()); 
        cursor = static_cast<Waypoint*>(cursor->Get_Father());
    }

    // string to_print = "echo pos: ";
    // for(auto it=simple_pos.begin(); it!=simple_pos.end(); ++it) to_print += " " + to_string(*it);
    // system(to_print.c_str());

//do the simplification
    auto it_w = wayps->begin();
    auto it_pos = simple_pos.begin();
    auto it_pos_prev = it_pos;
    ++it_pos;
    size_t k, K;
    for (it_pos; it_pos != simple_pos.end(); ++it_pos) {
        K = *it_pos - *it_pos_prev;
        ++it_w;
        for (k = 1; k < K; ++k) it_w = wayps->erase(it_w);
        ++it_pos_prev;
    }

}



void Brute_force_Simplifier::operator()(std::list<Array>* wayps, Node::I_Node_factory* problem) const{

    //system("echo got in simplifier");

    if(wayps->size() < 3) return;

    CostToGo_table table(wayps->size(), problem);
    Waypoint_factory factory;

    list<Waypoint*> explor_queue;

// find best solutions
    explor_queue.push_back(factory.create(&wayps->front()[0] , 0));
    size_t k, k_start, K = wayps->size();
    Waypoint* Best = nullptr;
    float Best_cost = FLT_MAX;
    float cost;
    while(!explor_queue.empty()){
        k_start=explor_queue.front()->get_pos();
        if(k_start == (K-1)){
            explor_queue.front()->Cost_to_root(&cost);
            if(cost < Best_cost) {
                Best_cost = cost;
                Best = explor_queue.front();
            }
        } 
        else{
            auto it_w = wayps->begin();
            advance(it_w, k_start);
            for(k = k_start + 1; k<K; ++k){
                ++it_w;
                Waypoint* temp = factory.create(&(*it_w)[0], k);
                cost = table.at(explor_queue.front() , temp);
                if (cost != FLT_MAX) {
                    temp->Set_Father(explor_queue.front() , cost);
                    explor_queue.push_back(temp);
                }
            }
        }
        explor_queue.pop_front();
    }

    simplify(wayps , Best);

}
