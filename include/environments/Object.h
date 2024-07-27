//
// Created by nermin on 09.02.24.
//

#ifndef RPMPL_OBJECT_H
#define RPMPL_OBJECT_H

#include <vector>
#include <string> 
#include <memory>
#include <ostream>
#include <fcl/fcl.h>

namespace env
{	
	class Object
	{
	public:
        Object() {}
        virtual ~Object() = 0;

        inline const std::string &getLabel() const { return label; }
		inline std::shared_ptr<fcl::CollisionObject<float>> getCollObject() const { return coll_object; }
        inline const fcl::Vector3f &getPosition() const { return position; }
        inline const fcl::Vector3f &getVelocity() const { return velocity; }
        inline const fcl::Vector3f &getAcceleration() const { return acceleration; }
        inline float getMaxVel() const { return max_vel; }
        inline float getMaxAcc() const { return max_acc; }
		inline float getMinDistTol() const { return min_dist_tol; }
        
        inline void setLabel(const std::string &label_) { label = label_; } 
		inline void setCollObject(const std::shared_ptr<fcl::CollisionObject<float>> coll_object_) { coll_object = coll_object_; }
        void setPosition(const fcl::Vector3f &position_);
		inline void setVelocity(const fcl::Vector3f &velocity_) { velocity = velocity_; }
		inline void setAcceleration(const fcl::Vector3f &acceleration_) { acceleration = acceleration_; }
		inline void setMaxVel(float max_vel_) { max_vel = max_vel_; }
		inline void setMaxAcc(float max_acc_) { max_acc = max_acc_; }
		inline void setMinDistTol(float min_dist_tol_) { min_dist_tol = min_dist_tol_; }

        friend std::ostream &operator<<(std::ostream &os, const std::shared_ptr<env::Object> obj);

	protected:
        std::string label;
		std::shared_ptr<fcl::CollisionObject<float>> coll_object;
		fcl::Vector3f position; 									// Position vector in [m]
		fcl::Vector3f velocity; 									// Velocity vector in [m/s]
		fcl::Vector3f acceleration; 						   		// Acceleration vector in [m/s²]
		float max_vel; 										    	// Maximal velocity in [m/s]
		float max_acc; 									        	// Maximal acceleration in [m/s²]
		float min_dist_tol; 										// Minimal distance tolerance for a static obstacle to not be included into a dynamic scene	
	};
}

#endif //RPMPL_OBJECT_H