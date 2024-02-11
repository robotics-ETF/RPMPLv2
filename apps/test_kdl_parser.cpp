//
// Created by dinko on 6.2.22.
// simple kdl_parser test
//

#include <nanoflann.hpp>

#include <ctime>
#include <cstdlib>
#include <ostream>
#include <string>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <glog/logging.h>

KDL::Vector getCartForSegment(KDL::Tree& robot_tree, KDL::Segment& segment)
{
	KDL::Chain robot_chain;
	//robot_tree.getChain("base_link", segment.getName(), robot_chain);
	KDL::TreeFkSolverPos_recursive treefksolver = KDL::TreeFkSolverPos_recursive(robot_tree);

	KDL::JntArray jointpositions = KDL::JntArray(2);
	jointpositions(0) = 0.1;
	jointpositions(1) = -0.1;
	KDL::Frame cartpos; 

	bool kinematics_status = treefksolver.JntToCart(jointpositions, cartpos, segment.getName());
	if(kinematics_status >= 0)
	{
        LOG(INFO) << "x: " << cartpos.p.x() << ";" << "y: " << cartpos.p.y() << ";" << "z: " << cartpos.p.z() ;
		return cartpos.p;
    }
	else
	{
        LOG(INFO) << "Error: could not calculate forward kinematics!";
    }

}

bool parse_file(std::string filename)
{
	KDL::Tree robot_tree;
	if (!kdl_parser::treeFromFile(filename, robot_tree))
	{
		LOG(ERROR) << "Failed to construct kdl tree";
		return false;
	}
	LOG(INFO) << "Number of joints: " << robot_tree.getNrOfJoints();
	LOG(INFO) << "Number of segments: " << robot_tree.getNrOfSegments();
	KDL::Chain robot_chain;
	robot_tree.getChain("base_link", "tool", robot_chain);
	
	for (int i = 0; i < robot_tree.getNrOfSegments(); i++)
	{
		KDL::Vector pos = getCartForSegment(robot_tree, robot_chain.getSegment(i));
		LOG(INFO) << robot_chain.getSegment(i).getName() << " pos is: " << pos.x() << ";" << pos.y() << ";" << pos.z();
	}
	return true;
}

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr = true;

	bool parsed = parse_file("data/planar_2dof/planar_2dof.urdf");

	return 0;
}
