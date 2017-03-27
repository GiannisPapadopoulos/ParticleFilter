/**
 ******************************************************
 * A simple localization utility for simulated bots,
 * based on a Particle Filter
 ******************************************************
 **/
#include "ParticleFilter.hpp"

#include "occupancy_grid_utils/ray_tracer.h" // Used for simulateRangeScan
#include <iostream>

/* TODO Any extra includes go here */
#include <random_numbers/random_numbers.h>


/* ********************************************************** */

/**  
 * Initialize particles, based on the given initialpose.
 * So far it just places all particles on a x - y = 0 line.
 **/
void MyLocalizer::initialisePF( const geometry_msgs::PoseWithCovarianceStamped& initialpose )
{
	/* TODO Replace this with something more sensible.
	 * E.g. draw particles from a Gaussian distribution
	 * with large variance centered on the supplied initial pose,
	 * or just place them in a regular grid across the map. */

    int width = map.info.width;
    int height = map.info.height;

    ROS_INFO("width %d", width);
    ROS_INFO("height %d", height);

    random_numbers::RandomNumberGenerator rng;
    double g = rng.gaussian(1, 5);

    ROS_INFO("g %f", g);

    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
	{
		particleCloud.poses[i].position.x = i;
		particleCloud.poses[i].position.y = i;
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(i*0.1);
		particleCloud.poses[i].orientation = odom_quat;
	}
	
	/* ********************************************************** */	
}
/**
 * Apply motion model based on the given changes in pose.
 * So far we just increment the poses by the change values.
 **/
void MyLocalizer::applyMotionModel( double deltaX, double deltaY, double deltaT )
{
	/* TODO Implement a motion model here.
	 * In your implementation sample from a random distribution,
	 * to allow for odometry noise. Make sure that you also update
	 * the angle of the particles correctly. */
	
	if (deltaX > 0 or deltaY > 0)
		ROS_DEBUG( "applying odometry: %f %f %f", deltaX, deltaY, deltaT );
		
	for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
	{
		particleCloud.poses[i].position.x += deltaX;
		particleCloud.poses[i].position.y += deltaY;      
	}
	
	/* ********************************************************** */
}
/**
 * Apply the sensor model to the given scan reading.
 * After the motion model moves the particles around, approximately
 * according to the odometry readings, the sensor model is used to
 * weight each particle according to how likely it is to get the
 * sensor reading that we have from that pose.
 **/
void MyLocalizer::applySensorModel( const sensor_msgs::LaserScan& scan )
{
	/* TODO Implement a sensor model here.
	 * Below is the beginning of an implementation of a beam
	 * sensor model. Use it as a base for your implementation. */
	
	for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
	{
		geometry_msgs::Pose sensor_pose;      
		sensor_pose =  particleCloud.poses[i];
		// If the laser and centre of the robot weren't at the same
		// position, we would first apply the tf from /base_footprint
		// to /base_laser here.
		sensor_msgs::LaserScan::Ptr simulatedScan;
		
		try
		{
			simulatedScan = occupancy_grid_utils::simulateRangeScan( this->map, sensor_pose, scan, true );
		}
		catch (occupancy_grid_utils::PointOutOfBoundsException)
		{
			continue;
		}

		// Now we have the actual scan, and a simulated version ---
		// i.e., how a scan would look if the robot had the pose
		// that particle i says it has. So now we should evaluate how
		// likely this pose is; i.e., the actual sensor model.

		//if (i == 0)
		//{
			//for (unsigned int k = 0; k < simulatedScan->ranges.size(); ++k)
				//ROS_INFO_STREAM(simulatedScan->ranges[k]);
				
			//std::cerr << "\n\n";
		//}
	}
	
	/* ********************************************************** */
}
/**
 * Resample particles, after the motion and sensor models were applied.
 **/
geometry_msgs::PoseArray MyLocalizer::updateParticleCloud
( const sensor_msgs::LaserScan& scan,
  const nav_msgs::OccupancyGrid& map,
  const geometry_msgs::PoseArray& particleCloud )
{
	/* TODO Implement a resampling procedure here */
	
	/* ********************************************************** */
	
	return this->particleCloud;
}
/**
 * Update and return the most likely pose. 
 **/
geometry_msgs::PoseWithCovariance MyLocalizer::updatePose()
{
	/* TODO Update the most likely pose and its covariance here */
	
	this->estimatedPose.pose.pose = particleCloud.poses[1];
	
	/* ********************************************************** */
	
	return this->estimatedPose.pose;
}

/* TODO Any extra methods go here */

/* ********************************************************** */
