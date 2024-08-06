#include "CollisionAndDistance.h"

// Check collision between capsule (determined with line segment AB and 'radius') and box (determined with 'obs = (x_min, y_min, z_min, x_max, y_max, z_max)')
bool base::CollisionAndDistance::collisionCapsuleToBox(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs)
{
    bool collision { false };
    float r_new = radius * sqrt(3) / 3;

	if ((A(0) > obs(0) - r_new && A(1) > obs(1) - r_new && A(2) > obs(2) - r_new &&
		 A(0) < obs(3) + r_new && A(1) < obs(4) + r_new && A(2) < obs(5) + r_new) ||
		(B(0) > obs(0) - r_new && B(1) > obs(1) - r_new && B(2) > obs(2) - r_new &&
		 B(0) < obs(3) + r_new && B(1) < obs(4) + r_new && B(2) < obs(5) + r_new))
		return true;
    else if ((A(0) < obs(0) - radius && B(0) < obs(0) - radius) || (A(0) > obs(3) + radius && B(0) > obs(3) + radius) ||
           	 (A(1) < obs(1) - radius && B(1) < obs(1) - radius) || (A(1) > obs(4) + radius && B(1) > obs(4) + radius) ||
           	 (A(2) < obs(2) - radius && B(2) < obs(2) - radius) || (A(2) > obs(5) + radius && B(2) > obs(5) + radius))
		return false;
    
    if (A(0) < obs(0)) {    				// < x_min
        collision = collisionCapsuleToRectangle(A, B, radius, obs, 0);
	}
    else if (A(0) > obs(3)) {  				// > x_max
        collision = collisionCapsuleToRectangle(A, B, radius, obs, 3);
	}
    
	if (!collision)
	{
		if (A(1) < obs(1))     				// < y_min
			collision = collisionCapsuleToRectangle(A, B, radius, obs, 1);
		else if (A(1) > obs(4)) 			// > y_max
			collision = collisionCapsuleToRectangle(A, B, radius, obs, 4);
		
		if (!collision)
		{
			if (A(2) < obs(2))     			// < z_min
				collision = collisionCapsuleToRectangle(A, B, radius, obs, 2);
			else if (A(2) > obs(5)) 		// > z_max
				collision = collisionCapsuleToRectangle(A, B, radius, obs, 5);
		}
	}
    return collision;
}

// Check collision between capsule (determined with line segment AB and 'radius') and rectangle (determined with 'obs',
// where 'coord' determines which coordinate is constant: {0,1,2,3,4,5} = {x_min, y_min, z_min, x_max, y_max, z_max}
bool base::CollisionAndDistance::collisionCapsuleToRectangle(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, 
															 Eigen::VectorXf &obs, size_t coord)
{
	float obs_coord { obs(coord) };
    if (coord > 2) {
        coord -= 3;
	}
    
	Eigen::Vector4f rec {}; rec << obs.head(coord), obs.segment(coord+1, 2-coord), obs.segment(3, coord), obs.tail(2-coord);
	Eigen::Vector2f A_rec {}; A_rec << A.head(coord), A.tail(2-coord);
	Eigen::Vector2f B_rec {}; B_rec << B.head(coord), B.tail(2-coord);
    float t { (B(coord) - A(coord)) != 0 ? (obs_coord - A(coord)) / (B(coord) - A(coord)) : INFINITY };
	Eigen::Vector2f M { A_rec + t * (B_rec - A_rec) };
	Eigen::Vector3f A_proj { get3DPoint(A_rec, obs_coord, coord) };
	Eigen::Vector3f B_proj { get3DPoint(B_rec, obs_coord, coord) };

    if (t > 0 && t < 1)		// Line segment AB intersects the plane 'obs(coord)'
	{
		if (M(0) > rec(0) - radius && M(0) < rec(2) + radius && 
			M(1) > rec(1) - radius && M(1) < rec(3) + radius) 	// Whether point lies on the oversized rectangle
		{
			if ((M(0) > rec(0) - radius && M(0) < rec(2) + radius && M(1) > rec(1) && M(1) < rec(3)) || 
				(M(1) > rec(1) - radius && M(1) < rec(3) + radius && M(0) > rec(0) && M(0) < rec(2)) ||
				(M(0) < rec(0) && M(1) < rec(2) && (M - Eigen::Vector2f(rec(0), rec(2))).norm() < radius) ||
				(M(0) < rec(0) && M(1) > rec(3) && (M - Eigen::Vector2f(rec(0), rec(3))).norm() < radius) ||
				(M(0) > rec(1) && M(1) < rec(2) && (M - Eigen::Vector2f(rec(1), rec(2))).norm() < radius) ||
				(M(0) > rec(1) && M(1) > rec(3) && (M - Eigen::Vector2f(rec(1), rec(3))).norm() < radius))
				return true;
		}
	}
	else if (std::min((A - A_proj).norm(), (B - B_proj).norm()) > radius)
		return false;

    // Considering collision between capsule and rectangle
	if (radius > 0)
	{
		if (A_rec(0) > rec(0) && A_rec(0) < rec(2) && A_rec(1) > rec(1) && A_rec(1) < rec(3))
		{
			if (B_rec(0) > rec(0) && B_rec(0) < rec(2) && B_rec(1) > rec(1) && B_rec(1) < rec(3)) 	// Both projections
			{
				if (std::min((A - A_proj).norm(), (B - B_proj).norm()) < radius)
					return true;
			}
			else																					// Only projection of point A
			{
				if (std::min(checkCases(A, B, rec, B_rec, obs_coord, coord), (A - A_proj).norm()) < radius)
					return true;
			}
		}
		else if (B_rec(0) > rec(0) && B_rec(0) < rec(2) && B_rec(1) > rec(1) && B_rec(1) < rec(3))	// Only projection of point B
		{
			if (std::min(checkCases(A, B, rec, A_rec, obs_coord, coord), (B- B_proj).norm()) < radius)
				return true;
		}
		else																						// No projections						
		{
			if (checkCases(A, B, rec, A_rec, obs_coord, coord) < radius)
				return true;
				
			if (checkCases(A, B, rec, B_rec, obs_coord, coord) < radius)
				return true;
		}
	}
	
	return false;
}

float base::CollisionAndDistance::checkCases(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector4f &rec, 
											 Eigen::Vector2f &point, float obs_coord, size_t coord)
{
	float d_c1 { INFINITY };
	float d_c2 { INFINITY };
	Eigen::Vector3f C {};
	Eigen::Vector3f D {};
	
	if (point(0) < rec(0))
	{
		C = get3DPoint(Eigen::Vector2f(rec(0), rec(1)), obs_coord, coord);
		D = get3DPoint(Eigen::Vector2f(rec(0), rec(3)), obs_coord, coord);
		d_c1 = std::get<0>(distanceLineSegToLineSeg(A, B, C, D));
	}
	else if (point(0) > rec(2))
	{
		C = get3DPoint(Eigen::Vector2f(rec(2), rec(1)), obs_coord, coord);
		D = get3DPoint(Eigen::Vector2f(rec(2), rec(3)), obs_coord, coord);
		d_c1 = std::get<0>(distanceLineSegToLineSeg(A, B, C, D));
	}
	
	if (d_c1 > 0 && point(1) < rec(1))
	{
		C = get3DPoint(Eigen::Vector2f(rec(0), rec(1)), obs_coord, coord);
		D = get3DPoint(Eigen::Vector2f(rec(2), rec(1)), obs_coord, coord);
		d_c2 = std::get<0>(distanceLineSegToLineSeg(A, B, C, D));
	}
	else if (d_c1 > 0 && point(1) > rec(3))
	{
		C = get3DPoint(Eigen::Vector2f(rec(0), rec(3)), obs_coord, coord);
		D = get3DPoint(Eigen::Vector2f(rec(2), rec(3)), obs_coord, coord);
		d_c2 = std::get<0>(distanceLineSegToLineSeg(A, B, C, D));
	}

	return std::min(d_c1, d_c2);
}

const Eigen::Vector3f base::CollisionAndDistance::get3DPoint(const Eigen::Vector2f &point, float coord_value, size_t coord)
{
	Eigen::Vector3f point_new {};
	point_new << point.head(coord), coord_value, point.tail(2-coord);
	return point_new;
}

// Check collision between two line segments, AB and CD
bool base::CollisionAndDistance::collisionLineSegToLineSeg(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector3f &C, Eigen::Vector3f &D)
{
    float alpha1 { (B - A).squaredNorm() };
    float alpha2 { (B - A).dot(D - C) };
    float beta1  { (C - D).dot(B - A) };
    float beta2  { (C - D).dot(D - C) };
    float gamma1 { (A - C).dot(A - B) };
    float gamma2 { (A - C).dot(C - D) };
    float s { (alpha1 * gamma2 - alpha2 * gamma1) / (alpha1 * beta2 - alpha2 * beta1) };
    float t { (gamma1 - beta1 * s) / alpha1 };
    Eigen::Vector3f P1 {};
    Eigen::Vector3f P2 {};

    if (t > 0 && t < 1 && s > 0 && s < 1)
	{
        P1 = A + t * (B - A);
        P2 = C + s * (D - C);
        if ((P2 - P1).norm() < RealVectorSpaceConfig::EQUALITY_THRESHOLD) 	// The collision occurs
            return true;
    }
    return false;
}

// Check collision between capsule (determined with line segment AB and 'radius') and sphere (determined with 'obs = (x_c, y_c, z_c, r)')
bool base::CollisionAndDistance::collisionCapsuleToSphere(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs)
{
	radius += obs(3);
    if ((A - obs.head(3)).norm() < radius || (B - obs.head(3)).norm() < radius)
        return true;    // The collision occurs

    else
	{
        float a { (B - A).squaredNorm() };
        float b { 2 * (A.dot(B) + (A - B).dot(obs.head(3)) - A.squaredNorm()) };
        float c { A.squaredNorm() + obs.head(3).squaredNorm() - 2 * A.dot(obs.head(3)) - radius * radius };
        float D { b * b - 4 * a * c };

        if (D >= 0)
		{
            float t1 = (-b + sqrt(D)) / (2 * a);
            float t2 = (-b - sqrt(D)) / (2 * a);
            if ((t1 > 0 && t1 < 1) || (t2 > 0 && t2 < 1))
                return true;
        }
    }
    return false;
}

// Get distance (and nearest points) between capsule (determined with line segment AB and 'radius') 
// and box (determined with 'obs = (x_min, y_min, z_min, x_max, y_max, z_max)')
std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> base::CollisionAndDistance::distanceCapsuleToBox
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs)
{
	CapsuleToBox capsule_box(A, B, radius, obs);
	capsule_box.compute();
	return {capsule_box.getDistance(), capsule_box.getNearestPoints()};
}

// Get distance (and nearest points) between two line segments, AB and CD
std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> base::CollisionAndDistance::distanceLineSegToLineSeg
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C, const Eigen::Vector3f &D)
{
    float d_c { INFINITY };
    std::shared_ptr<Eigen::MatrixXf> nearest_pts { std::make_shared<Eigen::MatrixXf>(3, 2) };
    std::shared_ptr<Eigen::MatrixXf> nearest_pts_temp { std::make_shared<Eigen::MatrixXf>(3, 2) };
    float alpha1 { (B - A).squaredNorm() };
    float alpha2 { (B - A).dot(D - C) };
    float beta1  { (C - D).dot(B - A) };
    float beta2  { (C - D).dot(D - C) };
    float gamma1 { (A - C).dot(A - B) };
    float gamma2 { (A - C).dot(C - D) };
    float s { (alpha1 * gamma2 - alpha2 * gamma1) / (alpha1 * beta2 - alpha2 * beta1) };
    float t { (gamma1 - beta1 * s) / alpha1 };
	
	if (t > 0 && t < 1 && s > 0 && s < 1)
	{
        nearest_pts->col(0) = A + t * (B - A);
        nearest_pts->col(1) = C + s * (D - C);
		d_c = (nearest_pts->col(1) - nearest_pts->col(0)).norm();
        if (d_c < RealVectorSpaceConfig::EQUALITY_THRESHOLD) 	// The collision occurs
            return {0, nullptr};
    }
    else
	{
		float d_c_temp { 0 };
        float alpha3 { (C - D).squaredNorm() };
        Eigen::Vector4f opt((A - C).dot(A - B) / alpha1,	// s = 0
							(A - D).dot(A - B) / alpha1,	// s = 1
							(A - C).dot(D - C) / alpha3,	// t = 0
							(B - C).dot(D - C) / alpha3);	// t = 1

        for (size_t i = 0; i < 4; i++)
		{
            if (opt(i) < 0)
			{
				if (i == 0 || i == 2)     	// s = 0, t = 0
				{
					nearest_pts_temp->col(0) = A;
					nearest_pts_temp->col(1) = C; 
				}
                else if (i == 1)      		// s = 1, t = 0
				{
					nearest_pts_temp->col(0) = A;
                    nearest_pts_temp->col(1) = D; 
				}
                else                      	// t = 1, s = 0
				{
					nearest_pts_temp->col(0) = B;
                    nearest_pts_temp->col(1) = C; 
				}
			}
            else if (opt(i) > 1)
			{
				if (i == 1 || i == 3)    	// s = 1, t = 1
				{
					nearest_pts_temp->col(0) = B;
					nearest_pts_temp->col(1) = D; 
				}
                else if (i == 0)        	// s = 0, t = 1
				{
					nearest_pts_temp->col(0) = B;
					nearest_pts_temp->col(1) = C; 
				}                    
                else                    	// t = 0, s = 1
				{
					nearest_pts_temp->col(0) = A;
					nearest_pts_temp->col(1) = D; 
				}
			}
            else
			{
				if (i == 0)                	// s = 0, t € [0, 1]
				{
					nearest_pts_temp->col(0) = A + opt(i) * (B - A);
					nearest_pts_temp->col(1) = C; 
				}                    
                else if (i == 1)       		// s = 1, t € [0, 1]
				{
					nearest_pts_temp->col(0) = A + opt(i) * (B - A);
                    nearest_pts_temp->col(1) = D; 
				}
                else if (i == 2)           	// t = 0, s € [0, 1]
				{
					nearest_pts_temp->col(0) = A;
                    nearest_pts_temp->col(1) = C + opt(i) * (D - C); 
				}
                else                       	// t = 1, s € [0, 1]
				{
					nearest_pts_temp->col(0) = B;
                    nearest_pts_temp->col(1) = C + opt(i) * (D - C); 
				}
			}
            
            d_c_temp = (nearest_pts_temp->col(1) - nearest_pts_temp->col(0)).norm();
            if (d_c_temp < d_c)
			{
                d_c = d_c_temp;
				*nearest_pts = *nearest_pts_temp;
			}
        }
    }
	return {d_c, nearest_pts};
}

// Get distance (and nearest points) between line segment AB and point C
std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> base::CollisionAndDistance::distanceLineSegToPoint
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C)
{
    std::shared_ptr<Eigen::MatrixXf> nearest_pts { std::make_shared<Eigen::MatrixXf>(3, 2) };
    nearest_pts->col(1) = C;
    float t_opt { (C - A).dot(B - A) / (B - A).squaredNorm() };

    if (t_opt < 0)
		nearest_pts->col(0) = A;
    else if (t_opt > 1)
        nearest_pts->col(0) = B;
    else
		nearest_pts->col(0) = A + t_opt * (B - A);
	
	float d_c { (nearest_pts->col(1) - nearest_pts->col(0)).norm() };
	if (d_c < RealVectorSpaceConfig::EQUALITY_THRESHOLD)
		return {0, nullptr};

	return {d_c, nearest_pts};
}

// Get distance (and nearest points) between capsule (determined with line segment AB and 'radius') 
// and sphere (determined with 'obs = (x_c, y_c, z_c, r)')
std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> base::CollisionAndDistance::distanceCapsuleToSphere
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs)
{
    std::shared_ptr<Eigen::MatrixXf> nearest_pts { std::make_shared<Eigen::MatrixXf>(3, 2) };
	float AO { (A - obs.head(3)).norm() };
    float d_c { AO - obs(3) };
    if (d_c < radius)	// The collision occurs
        return {0, nullptr};
    
    float BO { (B - obs.head(3)).norm() };
    float d_c_temp { BO - obs(3) };
    if (d_c_temp < radius) 	// The collision occurs
        return {0, nullptr};
    
    if (d_c_temp < d_c)
        d_c = d_c_temp; 

    float AB { (A - B).norm() };
    float s { (AB + AO + BO) / 2 };
    float alpha = acos((AO * AO + AB * AB - BO * BO) / (2 * AO * AB));
    d_c_temp = 2 * sqrt(s * (s - AB) * (s - AO) * (s - BO)) / AB - obs(3);     // h = 2 * P / AB; d_c_temp = h - obs(3);

    if (alpha < M_PI / 2)
	{
        float beta = acos((BO * BO + AB * AB - AO * AO) / (2 * BO * AB));
        if (beta < M_PI / 2) 	// Acute triangle
		{    
            d_c = d_c_temp;
            if (d_c_temp < radius)	// The collision occurs
			    return {0, nullptr};    
            
            nearest_pts->col(0) = A + AO * cos(alpha) / AB * (B - A);
            nearest_pts->col(1) = nearest_pts->col(0) + d_c / (obs.head(3) - nearest_pts->col(0)).norm() * (obs.head(3) - nearest_pts->col(0));
        }
        else
		{
            nearest_pts->col(1) = B + d_c / BO * (obs.head(3) - B);  
            nearest_pts->col(0) = B;
        }
    }
    else
	{
        nearest_pts->col(1) = A + d_c / AO * (obs.head(3) - A);  
        nearest_pts->col(0) = A;
    }
    return {d_c, nearest_pts};
}

// ------------------------------------------------ Class CapsuleToBox -------------------------------------------------------//
base::CollisionAndDistance::CapsuleToBox::CapsuleToBox(const Eigen::Vector3f &A_, const Eigen::Vector3f &B_, float radius_, Eigen::VectorXf &obs_)
{
	A = A_;
	B = B_;
	AB = Eigen::MatrixXf(3, 2);
	AB << A_, B_;
	radius = radius_;
	obs = obs_;
	d_c = INFINITY;
    nearest_pts = std::make_shared<Eigen::MatrixXf>(3, 2);
	projections = Eigen::MatrixXi::Zero(6, 2);
	dist_AB_obs = Eigen::Vector2f(INFINITY, INFINITY);
}

void base::CollisionAndDistance::CapsuleToBox::compute()
{
	projectionLineSegOnSide(1, 2, 0, 4, 5, 3);   // Projection on x_min or x_max
	projectionLineSegOnSide(0, 2, 1, 3, 5, 4);   // Projection on y_min or y_max
	projectionLineSegOnSide(0, 1, 2, 3, 4, 5);   // Projection on z_min or z_max     
	if (d_c == 0)
	{
		nearest_pts = nullptr;
		return;
	}

	size_t num_proj = (projections.col(0) + projections.col(1)).maxCoeff();
	if (num_proj > 0) 					// Projection of one or two points exists
	{
		size_t idx_point = (dist_AB_obs(0) < dist_AB_obs(1)) ? 0 : 1;
		d_c = dist_AB_obs.minCoeff();
		nearest_pts->col(0) = AB.col(idx_point);
		Eigen::Index idx_coord {};
		projections.col(idx_point).maxCoeff(&idx_coord);

		if (idx_coord == 0 || idx_coord == 3)
			nearest_pts->col(1) << obs(idx_coord), AB.col(idx_point).tail(2);
		else if (idx_coord == 1 || idx_coord == 4)
			nearest_pts->col(1) << AB(0, idx_point), obs(idx_coord), AB(2, idx_point);
		else if (idx_coord == 2 || idx_coord == 5)
			nearest_pts->col(1) << AB.col(idx_point).head(2), obs(idx_coord);
		
		if (num_proj == 1)
		{
			if (idx_point == 0)   		// Projection of 'A' exists, but projection of 'B' does not exist)
				checkEdges(B, idx_point);
			else 						// Projection of 'B' exists, but projection of 'A' does not exist
				checkEdges(A, idx_point);
		}		
	}
	else								// There is no projection of any point
		checkOtherCases();

	d_c -= radius;
}

void base::CollisionAndDistance::CapsuleToBox::projectionLineSegOnSide(size_t min1, size_t min2, size_t min3, size_t max1, size_t max2, size_t max3)
{
	// 'min3' and 'max3' corresponds to the coordinate which is constant
	for (size_t i = 0; i < 2; i++)
	{
		if (AB(min1, i) >= obs(min1) && AB(min1, i) <= obs(max1) && AB(min2, i) >= obs(min2) && AB(min2, i) <= obs(max2))
		{
			if (AB(min3,i) > obs(min3) && AB(min3,i) < obs(max3))
			{
				d_c = 0;
				return;
			}
			else if (AB(min3, i) <= obs(min3))
			{
				projections(min3, i) = 1;
				dist_AB_obs(i) = obs(min3) - AB(min3, i);
			}
			else if (AB(min3, i) >= obs(max3))
			{
				projections(max3, i) = 1;
				dist_AB_obs(i) = AB(min3, i) - obs(max3);
			}
		}
	}
}

void base::CollisionAndDistance::CapsuleToBox::checkEdges(Eigen::Vector3f &point, size_t idx)
{
	std::shared_ptr<Eigen::MatrixXf> line_segments { nullptr };
	if (projections(0, idx))  		// Projection on x_min
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 0))
			line_segments = getLineSegments(Eigen::Vector2f(point(1), point(2)), obs(1), obs(2), obs(4), obs(5), obs(0), 0);
		else
		{
			d_c = 0;
			return;
		}
	}				
	else if (projections(3, idx))  	// Projection on x_max
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 3))           
			line_segments = getLineSegments(Eigen::Vector2f(point(1), point(2)), obs(1), obs(2), obs(4), obs(5), obs(3), 0);
		else
		{
			d_c = 0;
			return;
		}
	}				
	else if (projections(1, idx))  	// Projection on y_min
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 1))
			line_segments = getLineSegments(Eigen::Vector2f(point(0), point(2)), obs(0), obs(2), obs(3), obs(5), obs(1), 1);
		else
		{
			d_c = 0;
			return;
		}
	}
	else if (projections(4, idx))  	// Projection on y_max
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 4))           
			line_segments = getLineSegments(Eigen::Vector2f(point(0), point(2)), obs(0), obs(2), obs(3), obs(5), obs(4), 1);
		else
		{
			d_c = 0;
			return;
		}
	}
	else if (projections(2, idx))  	// Projection on z_min
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 2))
			line_segments = getLineSegments(Eigen::Vector2f(point(0), point(1)), obs(0), obs(1), obs(3), obs(4), obs(2), 2);
		else
		{
			d_c = 0;
			return;
		}
	}
	else if (projections(5, idx))  	// Projection on z_max 
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 5))            
			line_segments = getLineSegments(Eigen::Vector2f(point(0), point(1)), obs(0), obs(1), obs(3), obs(4), obs(5), 2);
		else
		{
			d_c = 0;
			return;
		}
	}
	distanceToMoreLineSegments(*line_segments);
}

std::shared_ptr<Eigen::MatrixXf> base::CollisionAndDistance::CapsuleToBox::getLineSegments
	(const Eigen::Vector2f &point, float min1, float min2, float max1, float max2, float coord_value, size_t coord)
{
	std::shared_ptr<Eigen::MatrixXf> line_segments { std::make_shared<Eigen::MatrixXf>(3, 2) };
	size_t num { 0 };

	if (point(0) < min1)
	{
		line_segments->col(0) = get3DPoint(Eigen::Vector2f(min1, min2), coord_value, coord);
		line_segments->col(1) = get3DPoint(Eigen::Vector2f(min1, max2), coord_value, coord);
		num += 2;
	}
	else if (point(0) > max1)
	{
		line_segments->col(0) = get3DPoint(Eigen::Vector2f(max1, min2), coord_value, coord);
		line_segments->col(1) = get3DPoint(Eigen::Vector2f(max1, max2), coord_value, coord);
		num += 2;
	}
				
	if (point(1) < min2)
	{
		line_segments->conservativeResize(3, 2 + num);
		line_segments->col(num) 	= get3DPoint(Eigen::Vector2f(min1, min2), coord_value, coord);
		line_segments->col(num + 1) = get3DPoint(Eigen::Vector2f(max1, min2), coord_value, coord);
	}
	else if (point(1) > max2)
	{
		line_segments->conservativeResize(3, 2 + num);
		line_segments->col(num) 	= get3DPoint(Eigen::Vector2f(min1, max2), coord_value, coord);
		line_segments->col(num + 1) = get3DPoint(Eigen::Vector2f(max1, max2), coord_value, coord);
	}
	return line_segments;	
}
	
void base::CollisionAndDistance::CapsuleToBox::distanceToMoreLineSegments(const Eigen::MatrixXf &line_segments)
{
	float d_c_temp { 0 };
	std::shared_ptr<Eigen::MatrixXf> nearest_pts_temp { nullptr };
	
	for (int k = 0; k < line_segments.cols(); k += 2)
	{
		tie(d_c_temp, nearest_pts_temp) = distanceLineSegToLineSeg(A, B, line_segments.col(k), line_segments.col(k+1));
		if (d_c_temp <= 0)
		{
			d_c = 0;
			return;
		}
		else if (d_c_temp < d_c)
		{
			d_c = d_c_temp;
			*nearest_pts = *nearest_pts_temp;
		}
	}
}

void base::CollisionAndDistance::CapsuleToBox::checkOtherCases()
{
	if (A(0) < obs(0) && B(0) < obs(0))
	{
		if (A(1) < obs(1) && B(1) < obs(1))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < x_min, < y_min, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(0), obs(1), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// < x_min, < y_min, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(0), obs(1), obs(5)));
			else    									// < x_min, < y_min
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(2)), 
																 	   Eigen::Vector3f(obs(0), obs(1), obs(5)));
		}
		else if (A(1) > obs(4) && B(1) > obs(4))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < x_min, > y_max, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(0), obs(4), obs(2)));                        
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// < x_min, > y_max, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(0), obs(4), obs(5)));
			else    									// < x_min, > y_max
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(4), obs(2)), 
																 	   Eigen::Vector3f(obs(0), obs(4), obs(5)));
		}
		else
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < x_min, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(2)), 
																 	   Eigen::Vector3f(obs(0), obs(4), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// < x_min, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(5)), 
																 	   Eigen::Vector3f(obs(0), obs(4), obs(5)));
			else    									// < x_min
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(0), obs(0), obs(0), obs(0), obs(0), obs(0), obs(0), 
								 obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), obs(1), obs(1), 
								 obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(2);
				distanceToMoreLineSegments(line_segments);
			}
		}
	}
	////////////////////////////////////////
	else if (A(0) > obs(3) && B(0) > obs(3))
	{
		if (A(1) < obs(1) && B(1) < obs(1))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// > x_max, < y_min, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(3), obs(1), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > x_max, < y_min, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(3), obs(1), obs(5)));
			else    									// > x_max, < y_min
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(3), obs(1), obs(2)),
																 	   Eigen::Vector3f(obs(3), obs(1), obs(5)));                    
		}
		else if (A(1) > obs(4) && B(1) > obs(4))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// > x_max, > y_max, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(3), obs(4), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > x_max, > y_max, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(3), obs(4), obs(5)));
			else    									// > x_max, > y_max
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(3), obs(4), obs(2)), 
																 	   Eigen::Vector3f(obs(3), obs(4), obs(5)));                     
		}
		else
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// > x_max, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(3), obs(1), obs(2)),
																 	   Eigen::Vector3f(obs(3), obs(4), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > x_max, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(3), obs(1), obs(5)), 
																 	   Eigen::Vector3f(obs(3), obs(4), obs(5)));
			else    									// > x_max
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(3), obs(3), obs(3), obs(3), obs(3), obs(3), obs(3), obs(3), 
								 obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), obs(1), obs(1), 
								 obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(2);
				distanceToMoreLineSegments(line_segments);
			}
		}
	}
	///////////////////////////////////////
	else
	{
		if (A(1) < obs(1) && B(1) < obs(1))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < y_min, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(2)), 
																 	   Eigen::Vector3f(obs(3), obs(1), obs(2))); 
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// < y_min, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(5)), 
																 	   Eigen::Vector3f(obs(3), obs(1), obs(5))); 
			else    									// < y_min
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0),
								 obs(1), obs(1), obs(1), obs(1), obs(1), obs(1), obs(1), obs(1),
								 obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(2);
				distanceToMoreLineSegments(line_segments);                        
			}
		}
		else if (A(1) > obs(4) && B(1) > obs(4))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// > y_max, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(4), obs(2)), 
																 	   Eigen::Vector3f(obs(3), obs(4), obs(2)));                        
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > y_max, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(4), obs(5)),
																 	   Eigen::Vector3f(obs(3), obs(4), obs(5)));                             
			else    									// > y_max
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0),
								 obs(4), obs(4), obs(4), obs(4), obs(4), obs(4), obs(4), obs(4),
								 obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(2);
				distanceToMoreLineSegments(line_segments);  
			}
		}					                    
		else
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < z_min
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0), 
								 obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), 
								 obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(2);
				distanceToMoreLineSegments(line_segments);
			}
											
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > z_max
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0), 
								 obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), 
								 obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5);
				distanceToMoreLineSegments(line_segments);  
			}
					
			else
			{
				for (size_t kk = 0; kk < 6; kk++)    		// Check collision with all sides
				{
					if (collisionCapsuleToRectangle(A, B, 0, obs, kk))
					{
						d_c = 0;
						return;
					}
				}
				Eigen::MatrixXf line_segments(3, 24);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0), obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0), obs(0), obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0),
								 obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), obs(1), obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4),
								 obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(2), obs(5), obs(2), obs(5), obs(2), obs(5), obs(2), obs(5);
				distanceToMoreLineSegments(line_segments);
			}      
		}
	}
}
