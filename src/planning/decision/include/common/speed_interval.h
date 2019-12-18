#ifndef SPEEDINTERVAL_H
#define SPEEDINTERVAL_H


namespace pnc 
{

// 速度区间类
class SpeedInterval 
{

public:

    SpeedInterval() = default;
    ~SpeedInterval() = default;

		SpeedInterval(uint8_t	index,double smin,double smax,double v); 

    void setIndex(uint8_t	index);	// 设置速度区间的索引号
    void setSmin(double s);				// 设置速度区间开始的s值
    void setSmax(double s);				// 设置速度区间结束的s值
    void setVLimit(double v);			// 设置允许的最大速度

    uint8_t getIndex();	// 获取速度区间的索引号
    double getSmin();		// 获取速度区间开始的s值
    double getSmax();		// 获取速度区间结束的s值
    double getVLimit();	// 获取允许的最大速度

private:

	uint8_t	index_;			// 该速度区间在整个参考线中的索引号
	double s_min_;			// 速度区间开始的s值
	double s_max_;			// 速度区间结束的s值
	double v_limit_;		// 允许最大速度
};


class SpeedMap
{

public:

	SpeedMap() = default;
	~SpeedMap() = default;

	SpeedMap(ReferenceLine ref_line);	// 通过参考线构造速度地图

	void clearSpeedInterval();									// 清空速度地图
	void addSpeedInterval(SpeedInterval speed_interval);	// 设置速度地图
	std::vector<SpeedInterval> getSpeedMap();		// 获取速度地图

	uint8_t getMaxIndex();	// 获取速度区间最大的索引号
	uint8_t getIntervalIndexByS(double s);			// 通过s获取对应的速度区间索引号
	double getSpeedLimitByIndex(uint8_t index);	// 通过index获取对应的速度限制
	double getSminByIndex(uint8_t index);				// 通过index获取对应的速度区间的最小s值
	double getSmaxByIndex(uint8_t index);				// 通过index获取对应的速度区间的最大s值

private:

	std::vector<SpeedInterval> speed_interval_vec_;		// 参考线上的速度地图

};


}//end namespace

#endif // SPEEDINTERVAL_H

