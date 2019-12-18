#ifndef __VELODYNE_DRIVER_DRIVER_H
#define __VELODYNE_DRIVER_DRIVER_H


#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "velodyne_driver/input.h"


namespace velodyne_driver
{
	class VelodyneDriver
	{
		public:
			VelodyneDriver(ros::NodeHandle &node);
			~VelodyneDriver() {}

			bool poll(void);


		private:
			void diagTimerCallback(const ros::TimerEvent &event);


			struct
			{
				std::string frame_id;
				std::string model;
				int npackets;
				double rpm;
				int cut_angle;
				double time_offset = 0.0;
			} config_;


			boost::shared_ptr<Input> input_;
			ros::Publisher output_;


			ros::Timer diag_timer_;
			diagnostic_updater::Updater diagnostics_;
			double diag_min_freq_;
			double diag_max_freq_;
			boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
	};
}

#endif
