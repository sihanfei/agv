#include "utils.h"


namespace pnc
{

// 速度区间
SpeedInterval::SpeedInterval(uint8_t index,double smin,double smax,double v)
	:index_(index),s_min_(smin),s_max_(smax),v_limit_(v)
{
}


void SpeedInterval::setIndex(uint8_t	index)
{
	index_ = index;
}

void SpeedInterval::setSmin(double s)
{
	s_min_ = s;
}

void SpeedInterval::setSmax(double s)
{
	s_max_ = s;
}

void SpeedInterval::setVLimit(double v)
{
	v_limit_ = v;
}

uint8_t SpeedInterval::getIndex()
{
	return index_;
}

double SpeedInterval::getSmin()
{
	return s_min_;
}

double SpeedInterval::getSmax()
{
	return s_max_;
}

double SpeedInterval::getVLimit()
{
	return v_limit_;
}



// 速度地图
SpeedMap::SpeedMap(ReferenceLine ref_line)
{
	// 通过参考线构造速度地图

	// 获取参考线上的参考的参考点
	std::vector<ReferenceLinePoint> ref_points = ref_line.getReferenceLinePoints();

	// 变量初始化
	double smin = ref_points[0].getS();
	double smax = ref_points[0].getS();
	double vlimt = ref_points[0].getMaxSpeed();
	uint8_t index = 0;

	SpeedInterval sp_int;

	// 依次遍历参考点
	for (uint32_t i = 1; i < ref_points.size(); i++)
	{
		double max_speed = ref_points[i].getMaxSpeed();	// 获取参考点允许的最大速度

		// 出现新的速度区间
		if(vlimt != max_speed)
		{
			smax = ref_points[i-1].getS();			
			// 写入速度区间
			sp_int.setIndex(index);
			sp_int.setSmin(smin);
			sp_int.setSmax(smax);
			sp_int.setVLimit(vlimt);

			speed_interval_vec_.push_back(sp_int);	
	
			// 更新下一个速度区间
			index = index + 1;	
			smin = ref_points[i].getS();
			vlimt = max_speed;
		}
		else if(i == ref_points.size() - 1)	// 最后一端没有速度变化，需要再保存为一段速度区间
		{
			smax = ref_points[i].getS();
			// 写入速度区间
			sp_int.setIndex(index);
			sp_int.setSmin(smin);
			sp_int.setSmax(smax);
			sp_int.setVLimit(max_speed);
			speed_interval_vec_.push_back(sp_int);		
		}
	}

	// 输出速度地图信息
	ROS_INFO("SpeedMap:speed_interval size = %d",speed_interval_vec_.size());
	for(uint32_t i = 0;i<speed_interval_vec_.size();i++)
	{
		ROS_INFO("SpeedMap:smin = %f;smax = %f ;vlimit = %f",
							speed_interval_vec_[i].getSmin(),speed_interval_vec_[i].getSmax(),speed_interval_vec_[i].getVLimit());
	}

}


void SpeedMap::clearSpeedInterval()
{
	// 清空速度地图
	speed_interval_vec_.clear();
}


void SpeedMap::addSpeedInterval(SpeedInterval speed_interval)
{
	// 增加一个速度区间
  speed_interval_vec_.push_back(speed_interval);
}	


std::vector<SpeedInterval> SpeedMap::getSpeedMap()
{
	// 获取速度地图
	return speed_interval_vec_;
}		

uint8_t SpeedMap::getMaxIndex()
{
	// 获取速度区间最大的索引号
	return speed_interval_vec_.size() - 1;
}





uint8_t SpeedMap::getIntervalIndexByS(double s)
{
	// 通过s获取对应的速度区间索引号
	uint8_t search_flag = 0;

	for (uint8_t i = 0; i < speed_interval_vec_.size(); ++i)
	{
		SpeedInterval sp_int = speed_interval_vec_[i];

		if((s >= sp_int.getSmin())&&(s <= sp_int.getSmax()))
		{
			search_flag = 1;
			return sp_int.getIndex(); 
		}
	}

	if(search_flag == 0)
	{
		ROS_ERROR("SpeedInterval:Cannot find the index.");
		return 255;
	}
}	

double SpeedMap::getSpeedLimitByIndex(uint8_t index)
{
	// 通过index获取对应的速度限制	
	if(index >= speed_interval_vec_.size())
	{
		ROS_ERROR("SpeedInterval:index invalid.");
		return 0;
	}

	return speed_interval_vec_[index].getVLimit();

}	

double SpeedMap::getSminByIndex(uint8_t index)
{
	// 通过index获取对应的速度区间最小s值	
	if(index >= speed_interval_vec_.size())
	{
		ROS_ERROR("SpeedInterval:index invalid.");
		return 0;
	}
	return speed_interval_vec_[index].getSmin();
}	

double SpeedMap::getSmaxByIndex(uint8_t index)
{
	// 通过index获取对应的速度区间最大s值		
	if(index >= speed_interval_vec_.size())
	{
		ROS_ERROR("SpeedInterval:index invalid.");
		return 0;
	}
	return speed_interval_vec_[index].getSmax();
}	


}

