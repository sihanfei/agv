## 20190910  Author(LZC)
完成了decision程序的编写   

## 20190911
1、因算法的坐标系和其他模块的坐标系不一致，将所有消息中输入的角度进行坐标系转换  
2、仿真时发现是先有目标指令再有参考线信息，所以，不能在收到指令或者其他位置信息时的那一瞬间就直接将进行frenet和cartesian坐标系之间的转换  
3、frenetToCartesian():修改frenet到cartesian坐标系转换算法，用s值距离替代直线距离对插值区间进行查找  
4、LatticePlanner::calTrajectoryCost_Lat():用1/S来代替S进行评分,表示S越大越好  

## 20190916
1、Decision::isObstacleSafe()：针对障碍物时有时无的情况，添加障碍物消失计数器，当计数器达到5个周期时，则认为障碍物安全  
2、Decision::task_management(): 若连续5个周期轨迹规划失败，则给车辆下发急停信号，若车辆规划失败次数小于5次，则将上一时刻的轨迹下发给车辆  
3、computeCoefficients():修改了多项式系数的求解方法，代替了原来的矩阵库SVD解法  
4、SpeedMap::SpeedMap()：解决了速度地图生成错误的问题，将if语句中的“==”写成了“=”  
5、Decision::locationCallback():增加了对位置的校验，若车辆中心点位置会导致车头或车尾超出参考线范围，则认为该位置信息无效  
6、LatticePlanner::TrajectoryGenerate_Longitude(): 针对停车规划的st轨迹，将撒点轨迹的终点由目标点修改为分段目标点，以避免直接撒向终点的轨迹全部不合理的情况  

## 20190917
1、在Curve类和继承类（QuarticPolynomial、QuinticPolynomial、TrajectoryCurve）中，增加了2个成员函数getOrder()和getCoef()，用来获取轨迹的阶数和曲线的多项式系数  
2、LatticePlanner::TrajectoryGenerate_Latitude()：将横向轨迹的撒点起点从s0改为0，便于曲线方程求解  
3、LatticePlanner::Trajectory_Combine():计算l时，需要在参数s的基础上减去s0  
4、utils.h 中修改曲率限制，KAPPA_LIMIT = 0.216  
5、LatticePlanner::TrajectoryGenerate_Latitude()	：若计算出的dmin大于dmax，则将dmin赋值为0，以解决规划失败的问题  
6、LatticePlanner::LatticePlanne()：若车辆已经到达目标点，则直接认为规划成功  
7、修改车辆的停车逻辑，将原来的通过cartesian坐标系判断改为通过frenet坐标系判断  
8、取消车辆位置有效性尾部超过车道参考线开始位置的判断，这是因为地图发出来的参考线不是一条完整的全局路径参考线，有可能参考线的起点就刚好超过车体了  

## 20190918 
1、Decision::locationCallback():将车辆角度偏差从0~2pi圆整到-pi~pi，再进行偏差超限判断  
2、Decision::isObstacleSafe():障碍物不在车辆范围内的变量判断，将s修正为l；修正障碍物最小s值计算if语句  
3、Decision::isObstacleSafe():修改车道判断逻辑，将原来在哪一条车道上的判断逻辑修改为车辆占据那些车道范围的判断逻辑，若查询失败，则采用车道宽进行默认补充计算  
4、LatticePlanner::calTrajectoryCost_Lon_Cruise()：将评价参数中的T改为1/T，表示巡航规划时T越大越好  
5、utils.h 增加无穷大数变量MAX_NUM。给变量初始化是赋最小值不能用std::numeric_limits<double>::min()，这是因为这个值最小是0，而不是负无穷大  
6、Decision::VMSCmdHandle():收到新指令后，将指令完成标志位清0，以避免在主循环中将收到的指令立马清除;删除历史轨迹信息,以避免新指令的起点选用老轨迹上的点进行规划  
7、Decision::refLineCallback():增加变量锁，防止在使用参考线的时候，参考线突然更新导致内存溢出；收到参考线时候，先将参考线ready信号清0，防止参考线突然更新导致内存溢出  
8、增加Trajectory::clearTrajectoryPoint()函数，清除轨迹上所有点  
9、Decision::locationCallback():增加变量锁，防止再使用坐标的时候，位置发生变化  
10、Decision::task_management():连续5个周期规划失败下发急停后，规划失败计数器清0；增加对位置有效性判断，若位置未准备就行，则不跳出当前主循环  
11、Decision::VMSCmdHandle():收到指令后，将参考线的ready信号清0，防止新指令在老参考线上进行规划  
12、Decision::PublishEstopMsg():车辆触发急停后，删除原指令信息  
13、Decision::task_management():将和指令相关的处理逻辑单独拿出来，放在新增加Decision::getValidCMD()函数中，提高程序的可读性  

## 20190919(新增换道功能)
1、新建一个decision文件夹，将原decision.cpp放入其中，并新建lane_change.cpp，用来支持车辆的换道逻辑  
2、LaneChange::ClassifyObstacles():对障碍物进行分类，将障碍物分为五类：左侧，左中，中间，右中，右侧  
3、Decision::task_management():增加换道逻辑的输入接口和调用函数  
4、utils.h 中增加换道触发逻辑的变量DISTANCE_LANE_CHANGE，设计为距离前方障碍物25m就开始触发换道逻辑判断  

## 20190920
1、trajectory_pair.h中增加7个私有变量，用来表示每条轨迹的评价值  
2、LatticePlanner::Trajectory_Cost():修改轨迹对评分规则，将原来的单条轨迹评分，改为归一化后的评分  
3、取消了calTrajectoryCost_Pair()，calTrajectoryCost_Lat(),calTrajectoryCost_Lon_StopingMerging(),calTrajectoryCost_Lon_Cruise()函数的实现方式  
4、LatticePlanner::Trajectory_Cost():测试中发现，会产生单项的最大值和最小值相等的情况，导致计算出的cost为无穷大，为了防止这种情况出现，对得到的最大最小值进行进一步处理  
5、LatticePlanner::calJerkInt_LS(),LatticePlanner::calJerkInt_ST()：用原始的jerk积分值代替使用除以系数后的值  
6、LatticePlanner::Trajectory_Cost()：根据预测时间来计算轨迹最后的偏移量d1而不是直接根据轨迹的终点  
7、LaneChange::isObstacleOccupation():修改阈值，认为车辆前方20m～25m内无障碍物，触发换道命令  

## 20190926
1、LatticePlanner::LatticePlanner():构造函数中加入换道命令接口  
2、LatticePlanner::TrajectoryGenerate_Latitude():修改横向撒点规则，将横向撒点从"-0.2~0~0.2"修改为"-0.2+l_offset,l_offset,0.2+l_offset"，以支持换道算法逻辑  
3、LaneChange::CalLaneRangeCarIn():增加左车道，中间车道，右车道的判断逻辑  
4、LaneChange::ClassifyObstacles():忽略不在车道范围内的障碍物  
5、LaneChange::LaneChange():修改换道逻辑，在换道逻辑中加入车道信息，防止出现向左换道但左车道不存在的情况  

## 20190927
1、LaneChange::LaneChange():增加换道指令的初始化操作，防止程序一开始时出现不合理的规划  
2、Decision::task_management():在障碍物停车逻辑中增加换道的条件判断，防止换道时，进入障碍物停车逻辑  
3、LatticePlanner::Trajectory_Check():修改碰撞算法，原来的碰撞检测中，只是检测当前车辆和障碍物的距离而不是轨迹上所有点与障碍物的碰撞情况  
4、LaneChange::LaneChanlge():修改换道后的判断逻辑，当车辆向左换道时，若lmax<=lmax_leftlane,则认为左换道结束；向右换道时，若lmin>=lmin_rightlane,则认为右换道结束  
5、LaneChange::LaneChange():当车辆换道时，若换道方向上没有车道时，则默认返回原车道  
6、LatticePlanner::Trajectory_Cost():对d1进行修正，若车辆处在换道过程中，则d1越大越好，否则越小越好  
7、LatticePlanner::Trajectory_Pair():增加车辆超出边界信息判断；先分别判断横向轨迹和纵向轨迹的有效性的，然后再对有效的轨迹进行配对，以减少算法的执行时间  

## 20190929
1、utils.h 将车辆的横向安全距离1.0m改为0.5m，避免车辆在换道时，触发障碍物急停操作  
2、LaneChange::LaneChange():在车辆处于非换道模式是，增加对cmd的赋值，避免当前位置是非换道模式时，换道模块生成的换道指令为系统默认值，造成算法错误  
3、cartesianToFrenet():修改在参考线上找到距离车辆中心最近的参考点，将“通过距离最近”的判断方法改为“距离最近且航向角符合车体航向角范围”  
4、cartesianToFrenet():修改插值区间的计算方法，将“起点往回推或者终点往前推”的方法改为“起点和终点一起往回推或者往前推”  
5、calcRatio():当插值端的起点和终点非常接近时，将比例系数改为0而不是-1  
6、Decision::task_management():增加调试用的index计数器，便于分析log  

## 20190930
1、LatticePlanner::Trajectory_Pair():修改SL轨迹校验方法，将“逐点校验车辆在轨迹上是否超出车道”直接改为“校验车辆中心点在参考线上的偏移小于1.5倍的车道宽”  
2、LatticePlanner::Trajectory_Cost():修改评分项中s的计算方法，将"直接求sl轨迹的终点"改为“求st轨迹终点和sl轨迹终点中的最小值”  
3、utils.h：将MAX_NUM的定义从10^10改为10^20，以解决归一化操作时出现负数的情况  
4、LatticePlanner::Trajectory_Cost():修改归一化计算时，求normal_cost_lat_s最大最小值的bug  
5、Decision::isObstacleSafe():当车辆处在换道模式时，不更新障碍物的最小距离，且把和障碍物的最小距离成默认最大值  
6、Decision::task_management():取消“在障碍物停车逻辑中增加换道的条件判断，防止换道时，进入障碍物停车逻辑”  

## 20191008(新增斜行换道功能)
1、LaneChange::LaneChange():换道完成的逻辑从车辆边界的l_max修改为中心点的l值，已解决“障碍物允许换道，但换道条件不满足”的情况  
2、LatticePlanner::Trajectory_Cost()：原来的写法误将“S = S_sl > S_st ? S_st : S_sl”写成了“S = S_sl > S_st ? S_st : S_st”，会导致每次轨迹选择都无法选到长轨迹  
3、utils.h：增加轨迹角度修改标志位theta_modify_flag_  
4、LatticePlanner::LatticePlanner():增加角度修改接口  
5、Decision::task_management():增加对轨迹角度修改标志位的判断逻辑  
6、LatticePlanner::Trajectory_Combine():根据轨迹角度标志位，修改轨迹角度，使其和参考线保持一致，以便支持斜行功能  

## 20191009(新增数据自动保存功能)
1、utils.h：将纵向规划的时间权重值“WEIGHT_LON_T”从5改为1，主要考虑到停车时时间权重不能占比太重，容易导致车辆以极限减速度停车，造成后续轨迹可能失败的情况  
2、Decision.h:增加文件保存计数器，当计数器达到200时，生成trace文件  
3、Decision.cpp：增加TraceData()函数，用来保存决策信息，轨迹信息和障碍物信息  
4、Decision::TraceData()：修改文件保存逻辑，将原来的“20s保存一次”修改为“60s保存一次或者完成任务或者急停时”  
5、Decision::getValidCMD()：增加逻辑“当指令完成时，生成trace文件”  
6、Decision::Init()：增加规划起点和终点的初始化操作  

## 20191010
1、Decision::TraceData()：修改保存轨迹数据和感知数据只能保存最后一条的bug  
2、Decision::TraceData()：数据保存开始，先清空容器，防止下一周期的数据含有当前周期的数据  
3、Decision::getValidCMD()	：修改指令完成的判断逻辑，增加车辆距离目标点小于5cm时，判定车辆完成当前任务  

## 20191011
1、Decision::Init():新建Trace文件保存的文件夹  
2、Decision::TraceData():修改trace文件保存的位置，将文件保存在初始化中生成的特定文件夹中  

## 20191012
1、Trajectory_Cost():修改停车规划时间权重的权重的计算方法，用1/T代替T来表示st轨迹的cost值，表示停车时间越长越好  
2、LatticePlanner.cpp:增加getStopFlag()和getPlanningEndPoint(),主要是为了方便trace分析数据用  
3、Decision::TraceData():增加两个trace数据，停车规划标志位和规划终点  
4、Decision::TraceData():新增trace文件个数控制保存功能，将文件夹中的文件个数控制在60个  
