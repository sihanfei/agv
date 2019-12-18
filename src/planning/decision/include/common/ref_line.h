#ifndef REFERENCELINE_H
#define REFERENCELINE_H


namespace pnc 
{

// 车道宽结构体
struct LaneRange
{
    double left_boundary_;		// 左边界宽
    double right_boundary_;		// 右边界宽
};

// 参考点类
class ReferenceLinePoint
{

public:
    ReferenceLinePoint() = default;
    ~ReferenceLinePoint() = default;

    ReferenceLinePoint(double s,double x,double y,double theta,double kappa,double dkappa);		
    ReferenceLinePoint(double s,double x,double y,double theta,double kappa,double dkappa,double max_speed,std::vector<LaneRange> lane_ranges);


    void setS(double s);
    void setX(double x);
    void setY(double y);
    void setTheta(double theta);
    void setKappa(double kappa);
    void setdKappa(double dkappa);
    void setMaxSpeed(double max_speed);
    void setdWidthLeft(double width_left);
    void setdWidthRight(double width_right);
    void setLaneRanges(std::vector<LaneRange> lane_ranges);


    double getS() const;
    double getX() const;
    double getY() const;
    double getTheta() const;
    double getKappa() const;
    double getdKappa() const;
    double getMaxSpeed() const;
    double getdWidthLeft() const;
    double getdWidthRight() const;
    std::vector<LaneRange> getLaneRanges() const;



private:
    double s_;
    double x_;
    double y_;
    double theta_;
    double kappa_;
    double dkappa_;
    double max_speed_;
    double width_left_;
    double width_right_;

    std::vector<LaneRange> lane_ranges_;
};



// 参考线类
class ReferenceLine
{

public:

    ReferenceLine() = default;
    ~ReferenceLine() = default;

    void clearReferencePoints();									//清除参考线上所有的参考点信息
    double getReferenceLineMaxS() const;					//获取参考线最大的S值																							
    uint32_t getReferenceLinePointsSize() const;	// 获取参考线上的点个数
    std::vector<ReferenceLinePoint> getReferenceLinePoints() const;											// 获取参考线上的点
    void setReferenceLinePoints(const std::vector<ReferenceLinePoint> ref_line_points);	// 设置参考线上的参考点
    void addReferencePoint(ReferenceLinePoint ref_point);																// 在参考线上增加一个参考点


    ReferenceLinePoint getReferenceLinePointByIndex(uint32_t index) const;	// 通过索引号获取参考线上对应的参考点信息
    ReferenceLinePoint getReferenceLinePointByS(double s) const;						// 通过s值获取参考线上相应的参考点信息
    ReferenceLinePoint getNearestRefLinePoint(double x,double y);						// 根据XY坐标获取参考上相应的参考点信息

private:

    std::vector<ReferenceLinePoint> ref_line_points_;				// 参考线上的参考点信息

};
}//end namespace

#endif // REFERENCELINE_H

