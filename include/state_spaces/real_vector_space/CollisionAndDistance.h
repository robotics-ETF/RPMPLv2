//
// Created by nermin on 07.03.22.
//

#ifndef RPMPL_COLLISIONANDDISTANCE_H
#define RPMPL_COLLISIONANDDISTANCE_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
// #include <QuadProg++.hh>
// #include <glog/log_severity.h>
// #include <glog/logging.h>

namespace base
{
    class CollisionAndDistance
    {
    public:
        CollisionAndDistance() {}

		static bool collisionCapsuleToBox(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs);
		static bool collisionCapsuleToRectangle(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs, int coord);
		static bool collisionLineSegToLineSeg(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector3f &C, Eigen::Vector3f &D);
		static bool collisionCapsuleToSphere(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs);

        static std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> distanceCapsuleToBox
			(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs);
		std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> distanceCapsuleToBoxQP
			(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs);
		static std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> distanceLineSegToLineSeg
			(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C, const Eigen::Vector3f &D);
		static std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> distanceLineSegToPoint
			(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C);
		static std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> distanceCapsuleToSphere
			(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs);

    private:
		static float checkCases(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector4f &rec, Eigen::Vector2f &point, 
                                float obs_coord, int coord);
		static const Eigen::Vector3f get3DPoint(const Eigen::Vector2f &point, float coord_value, int coord);
        
        class CapsuleToBox
        {
        private:
            std::shared_ptr<Eigen::MatrixXf> nearest_pts;
            Eigen::Vector3f A, B, P1, P2;
            Eigen::MatrixXi projections;					// Determines whether projections on obs exist. First column is for point 'A', and second is for point 'B'
            Eigen::Vector2f dist_AB_obs;					// Distances of 'A' and 'B' to 'obs' (if projections exist)
            Eigen::MatrixXf AB;								// Contains points 'A' and 'B'
            Eigen::VectorXf obs;
            float d_c;
            float radius;

            void projectionLineSegOnSide(int min1, int min2, int min3, int max1, int max2, int max3);
            void checkEdges(Eigen::Vector3f &point, int k);
            std::shared_ptr<Eigen::MatrixXf> getLineSegments(const Eigen::Vector2f &point, float min1, float min2, float max1, float max2, 
                                                            float coord_value, int coord);
            void distanceToMoreLineSegments(const Eigen::MatrixXf &line_segments);
            void checkOtherCases();

        public:
            CapsuleToBox(const Eigen::Vector3f &A_, const Eigen::Vector3f &B_, float radius_, Eigen::VectorXf &obs_);

            void compute();
            float getDistance() { return d_c; }
            std::shared_ptr<Eigen::MatrixXf> getNearestPoints() { return nearest_pts; }
        };
    };
}
#endif //RPMPL_COLLISIONANDDISTANCE_H