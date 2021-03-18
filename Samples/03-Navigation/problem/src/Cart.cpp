/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Cart.h>
#include <math.h>

namespace mt::sample {
    Cart::Cart(const Positive<float>& width, const Positive<float>& length) {
        const float halfW = 0.5f * width.get();
        const float halfH = 0.5f * length.get();
        this->vertices[0] = geometry::Point(halfW , -halfH);
        this->vertices[1] = geometry::Point(halfW , halfH);
        this->vertices[2] = geometry::Point(-halfW , halfH);
        this->vertices[3] = geometry::Point(-halfW , -halfH);

        this->segments[0] = std::make_unique<geometry::Segment>(this->vertices[0], this->vertices[1]);
        this->segments[1] = std::make_unique<geometry::Segment>(this->vertices[1], this->vertices[2]);
        this->segments[2] = std::make_unique<geometry::Segment>(this->vertices[2], this->vertices[3]);
        this->segments[3] = std::make_unique<geometry::Segment>(this->vertices[3], this->vertices[0]);

        this->relativePos.z() = 0.0;
    }

    const float Cart::getWidth() const {
        return 2.0 * this->vertices[0].x();
    }


    const float Cart::getLength() const {
        return 2.0 * this->vertices[0].y();
    }

    bool Cart::isColliding(const float* pose, const geometry::Sphere& obstacle) const {
        float cosAngle = cosf(pose[2]);
        float sinAngle = sinf(pose[2]);
        float delta[2] = {pose[0] - obstacle.getCenter().x() , pose[1] - obstacle.getCenter().y()};
        this->relativePos.x() = cosAngle * delta[0] + sinAngle * delta[1];
        this->relativePos.y() = -sinAngle * delta[0] + cosAngle * delta[1];
        
        bool positiveNess[2] = {this->relativePos.x() >= 0.0 , this->relativePos.y() >= 0.0};
        if(positiveNess[0] && positiveNess[1]) {
            this->checker0.check(*this->segments[0] , this->relativePos);
            this->checker1.check(*this->segments[1] , this->relativePos);            
        }
        else if(!positiveNess[0] && positiveNess[1]) {
            this->checker0.check(*this->segments[1] , this->relativePos);
            this->checker1.check(*this->segments[2] , this->relativePos);
        }
        else if(positiveNess[0] && !positiveNess[1]) {
            this->checker0.check(*this->segments[0] , this->relativePos);
            this->checker1.check(*this->segments[3] , this->relativePos);
        }
        else {
            this->checker0.check(*this->segments[2] , this->relativePos);
            this->checker1.check(*this->segments[3] , this->relativePos);
        }
        if (this->checker0.getDistance() <= obstacle.getRay()) return true;
        if (this->checker1.getDistance() <= obstacle.getRay()) return true;
        return false;
    }
}