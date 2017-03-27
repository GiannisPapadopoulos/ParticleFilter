#ifndef MMN_PARTICLEFILTER_HPP_1403
#define MMN_PARTICLEFILTER_HPP_1403

#include "MCLocalizer.hpp"

/**
 * This class implements the pure virtual methods of PFLocalizer. It
 * is an "empty" class, in that the methods don't actually update
 * anything. The assignment consists in implementing a class just like
 * this one, but with actual content in the methods.
 **/
class MyLocalizer: public MCLocalizer
{
	public:
		/**  
		 * Initialize particles, based on the given initialpose.
		 * So far it just places all particles on a x - y = 0 line.
		 **/
		virtual void initialisePF( const geometry_msgs::PoseWithCovarianceStamped& initialpose );
	protected:
		/**
		 * Apply motion model based on the given changes in pose.
		 * So far we just increment the poses by the change values.
		 **/
		virtual void applyMotionModel( double deltaX, double deltaY, double deltaT );
		/**
		 * Apply the sensor model to the given scan reading.
		 * After the motion model moves the particles around, approximately
		 * according to the odometry readings, the sensor model is used to
		 * weight each particle according to how likely it is to get the
		 * sensor reading that we have from that pose.
		 **/
		virtual void applySensorModel( const sensor_msgs::LaserScan& scan );
		/**
		 * Resample particles, after the motion and sensor models were applied.
		 **/
		virtual geometry_msgs::PoseArray updateParticleCloud
		( const sensor_msgs::LaserScan& scan,
		  const nav_msgs::OccupancyGrid& map,
		  const geometry_msgs::PoseArray& particleCloud );
		/**
		 * Update and return the most likely pose. 
		 */
		virtual geometry_msgs::PoseWithCovariance updatePose();
  
		/* TODO Any extra variables go here */

		/* ********************************************************** */


		/* TODO Any extra methods go here */

		/* ********************************************************** */
  
};

#endif
