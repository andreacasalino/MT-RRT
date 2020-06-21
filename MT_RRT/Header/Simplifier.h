/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#pragma once
#ifndef __MT_RRT_SIMPLIFIER_H__
#define __MT_RRT_SIMPLIFIER_H__

#include "Tree.h"
#include <vector>

namespace MT_RTT
{

    class I_Simplifier{
    public:
        virtual ~I_Simplifier(){};

        virtual void operator()(std::list<Array>* wayps, Node::I_Node_factory* problem) const = 0;
    protected:
        I_Simplifier(){};
    };


    class Brute_force_Simplifier : public I_Simplifier {
    public:
        Brute_force_Simplifier() {};

        virtual void operator()(std::list<Array>* wayps, Node::I_Node_factory* problem) const;
    };

}

#endif