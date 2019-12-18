# coding : utf-8
import numpy as np
from enum import Enum, unique

@unique
class DirectionOfTrafficEnum(Enum):
    """
    道路通行方向
    """
    STRAIGHT = 0b0001
    LEFT = 0b0010
    RIGHT = 0b0100
    BIDIRECTION = 0b1000

# class GPSPointXYZ(Structure):
#     _fields_ = [("x", c_float), ("y", c_float), ("z", c_float)]

# class GPSPointList(list):
#     pass

# class GPSPointUnion(Union):
#     _fields_ = [('xyz', POINTER(GPSPointXYZ)), ('list', POINTER(GPSPointList))]
#     pass

class GPSPointDef(list):
    def __init__(self, x=0, y=0, z=0):
        list.__init__([])
        self.extend([x, y, z])

    def fromArray(self, point):
        pass

    def toStr(self, sep=','):
        style_list = []
        for value in self:
            style_list.extend([str(value)])
        style_str = sep.join(style_list)
        return style_str

    def fromStr(self, style_str, sep=','):
        style_list = style_str.split(sep)
        # self = list([])
        for i, value in enumerate(style_list):
            self[i] = float(value)

    @staticmethod
    def calcDistance(p0, p1):
        distance = np.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2 +
                           (p1[2] - p0[2])**2)
        return distance

    @staticmethod
    def getDiAngle(Ps, Pe):
        """
        计算有向线段的与x轴的夹角:
            实际可以认为是Ps-Pe构成的有向线段与x轴单位向量的夹角
        """
        angle = 0
        start = np.array(Ps)
        end = np.array(Pe)
        pline = end - start
        l = GPSPoint.calcTwoPointsDistance(Ps, Pe)
        pline = pline / l
        angle = np.arccos(np.dot([1, 0], pline))
        if angle >= 0 and pline[1] < 0:
            angle = -1 * angle
        return angle


class RefPointDef:
    """
    车道参考线上的点
    1. point
    2. width
    3. cuv
    4. gcuv
    5. s
    6. theta
    """

    def __init__(self,
                 gps=GPSPointDef(0.0, 0.0, 0.0),
                 lanes=1,
                 width_list=[],
                 cuv=0.0,
                 gcuv=0.0,
                 slength=0.0,
                 theta=0.0):
        self.gps_ = gps
        self.lanes_ = lanes
        self.width_list_ = width_list
        self.cuv_ = cuv
        self.gcuv_ = gcuv
        self.slength_ = slength
        self.theta_ = theta
        pass

    def toStr(self, sep=','):
        style_list = []
        style_list.append('{}'.format(self.gps_[0]))
        style_list.append('{}'.format(self.gps_[1]))
        style_list.append('{}'.format(self.gps_[2]))
        style_list.append('{}'.format(self.lanes_))
        for width in self.width_list_:
            style_list.append('{}'.format(width[0]))
            style_list.append('{}'.format(width[1]))
            # # for test
            # if width[0]<0 or width[1] < 0:
            #     print('elememtn: got a negative', style_list)
        style_list.append('{}'.format(self.cuv_))
        style_list.append('{}'.format(self.gcuv_))
        style_list.append('{}'.format(self.slength_))
        style_list.append('{}'.format(self.theta_))
        style_str = sep.join(style_list)
        return style_str

    def  fromStr(self, style_str, sep=','):
        style_list = style_str.split(sep)
        self.gps_[0] = float(style_list[0])
        self.gps_[1] = float(style_list[1])
        self.gps_[2] = float(style_list[2])
        index = 3
        self.lanes_ = int(style_list[3])
        
        self.width_list_ = []
        for i in range(self.lanes_):
            width = [0,0]
            index += 1
            width[0] = float(style_list[index])
            index += 1
            width[1] = float(style_list[index])
            self.width_list_.append(width)
        index += 1
        self.cuv_ = float(style_list[index])
        index += 1
        self.gcuv_ = float(style_list[index])
        index += 1
        self.slength_ = float(style_list[index])
        index += 1
        self.theta_ = float(style_list[index])


    def setGPS(self, gps):
        self.gps_ = gps
        pass

    def setLanesNumber(self, lanes):
        self.lanes_ = lanes
        pass

    def setWidth(self, width_list):
        self.width_list_ = width_list
        pass

    def setCuv(self, cuv):
        self.cuv_ = cuv
        pass

    def setGCuv(self, gcuv):
        self.gcuv_ = gcuv
        pass

    def setSlength(self, slength):
        self.slength_ = slength
        pass

    def setTheta(self, theta):
        self.theta_ = theta
        pass

@unique
class RoadMaterial(Enum):
    UNDEFINED = 0
    PITCH = 1
    CEMENT = 2
    GRASS = 3

@unique
class AreaType(Enum):
    """
    道路附属物类型
    """
    UNDEFINED = 0
    TRAFFICLIGHT = 1
    CROSSAREA = 2
    VLOCALIZATIONSIGN = 3
    LLOCALIZATIONSIGN = 4
    WORKSHOP = 5
    STATICOBJECT = 6
    PARKING = 7 # 停车位
    BERTH = 8 # 泊位
    BRIDGE = 9 # 桥梁
    CLEANING = 10 # 清洗区
    CHARGING = 11 # 充电区
    YARD = 12 # 堆场
    ROAD = 13 # 普通道路


class RefLineDef:
    """
    车道参考线
    1.id
    2.限速
    3.曲率（1/转弯半径）
    4.限高
    5.车道数量
    6.行车方向
    7.坡度
    8.参考线长度
    9.参考线起点
    10.参考线终点
    11.材质
    12.区域属性
    """

    def __init__(
            self,
            index=0,
            speed_limited=[0, 0],  # m/s
            height_limited = 0.0,
            curv = 0.0, # 曲率 
            lanes_number=1,
            ratio=0.0,
            direction=DirectionOfTrafficEnum.STRAIGHT,
            slength=0,
            startpoint=GPSPointDef(0,0,0),
            endpoint=GPSPointDef(0,0,0),
            material=RoadMaterial.UNDEFINED,
            area=AreaType.UNDEFINED):
        self.index = index
        self.speed_limited = speed_limited
        self.height_limited = height_limited
        self.curv = curv
        self.lanes_number = lanes_number
        self.ratio = ratio
        self.direction = direction
        self.slength = slength
        self.startpoint = startpoint
        self.endpoint = endpoint
        self.material = material
        self.area = area
        pass

    def copyFrom(self, ref_line):
        self.index = ref_line.index
        self.speed_limited = ref_line.speed_limited
        self.height_limited = ref_line.height_limited
        self.curv = ref_line.curv
        self.lanes_number = ref_line.lanes_number
        self.ratio = ref_line.ratio
        self.direction = ref_line.direction
        self.slength = ref_line.slength
        self.startpoint = ref_line.startpoint
        self.endpoint = ref_line.endpoint
        self.material = ref_line.material
        self.area = ref_line.area

    def toStr(self):
        # write title
        # file.write('index, speed_low, speed_up, lanes, ratio, direction, slength, startpoint, endpoint, material\n')
        style_list = []
        style_list.append('{}'.format(int(self.index)))
        style_list.append('{}'.format(self.speed_limited[0]))
        style_list.append('{}'.format(self.speed_limited[1]))
        style_list.append('{}'.format(self.height_limited))
        style_list.append('{}'.format(self.curv))
        style_list.append('{}'.format(int(self.lanes_number)))
        style_list.append('{}'.format(self.ratio))
        style_list.append('{}'.format(int(self.direction)))
        style_list.append('{}'.format(self.slength))
        style_list.append('{}'.format(self.startpoint[0]))
        style_list.append('{}'.format(self.startpoint[1]))
        style_list.append('{}'.format(self.startpoint[2]))
        style_list.append('{}'.format(self.endpoint[0]))
        style_list.append('{}'.format(self.endpoint[1]))
        style_list.append('{}'.format(self.endpoint[2]))
        style_list.append('{}'.format(int(self.material)))
        style_list.append('{}'.format(int(self.area)))
        style_str = ','.join(style_list)
        return style_str

    def fromStr(self, style_str):
        str_list = style_str.split(',')
        self.index = int(str_list[0])
        self.speed_limited[0] = float(str_list[1])
        self.speed_limited[1] = float(str_list[2])
        self.height_limited = float(str_list[3])
        self.curv = float(str_list[4])
        self.lanes_number = int(str_list[5])
        self.ratio = float(str_list[6])
        self.direction = int(str_list[7])
        self.slength = float(str_list[8])
        self.startpoint = GPSPointDef(float(str_list[9]), float(str_list[10]), float(str_list[11]))
        self.endpoint = GPSPointDef(float(str_list[12]), float(str_list[13]), float(str_list[14]))
        self.material = int(str_list[15])
        self.area = int(str_list[16])
        return

    def setIndex(self, index):
        self.index = index
        pass

    def setSpeedLimited(self, speed_limited):
        self.speed_limited = speed_limited

    def setLanesNumber(self, lanes_number):
        self.lanes_number = lanes_number
        pass

    def setMaterial(self, material):
        self.material = material
        pass

    def setDirection(self, direction):
        self.direction = direction
    
    def setRatio(self, ratio):
        self.ratio = ratio

    def setAreaType(self, area_type):
        self.area = area_type


class AppendixObjectDef:
    """
    附属物定义
    1.id
    2.类型
    3.位置(n个GPS坐标)
    4.归属于哪条ref_line
    """

    def __init__(self,
                 index=1,
                 object_type=AreaType.UNDEFINED,
                 position=[],
                 belongingID=1):
        self.index_ = index
        self.type_ = object_type
        self.position_ = position
        self.belongingID_ = belongingID  # ref_line_id
        pass

    def setIndex(self, index):
        self.index_ = index
        pass

    def setType(self, object_type):
        self.type_ = object_type
        pass

    def setPosition(self, position):
        self.position_ = position

    def setBelonging(self, belongingID):
        self.belongingID_ = belongingID

    def toStr(self):
        style_list = []
        style_list.append('{}'.format(self.index_))
        style_list.append('{}'.format(self.type_))
        for point in self.position_: # 每个point是一个list
            style_list.append('{}'.format(point[0]))
            style_list.append('{}'.format(point[1]))
            style_list.append('{}'.format(point[2]))
        style_list.append('{}'.format(self.belongingID_))
        style_str = ','.join(style_list)
        return style_str

    def fromStr(self, style_str):
        str_list = style_str.split(',')
        self.index_ = int(str_list[0])
        self.type_ = int(str_list[1])
        self.belongingID_ = int(str_list[-1])
        self.position_ = []
        for i in range(2, len(str_list)-1, 3):
            point = GPSPointDef(float(str_list[i+0]), float(str_list[i+1]), float(str_list[i+2]))
            self.position_.append(point)

    def copyFrom(self, orig):
        self.index_ = orig.index_
        self.type_ = orig.type_
        self.belongingID_ = orig.belongingID_
        self.position_ = []
        for point in orig.position_:
            self.position_.append(point)


class SpiralLine:
    """
    在缓行曲线上，dL/dt = v, dbeta/dt = omiga(转角速度,前轮偏角)
    则有dL/dbeta = v/omiga
    对于AGV，有omiga_max = 20度/s ~= pi/10,
    则dL/dbeta = 10v/pi，因此，有 L = 10v/pi*beta

    对于AGV，假定轴距为L，前轮转角为alpha，后轮转角为beta(从车头/尾指向车身中间为正方向，以逆时针旋转为正)
    则有：
    前轮转弯半径：
    R1 = L/(cos(alpha)(tan(alpha)+tan(beta)))
    后轮转弯半径：
    R2 = L/(cos(beta)(tan(alpha)+tan(beta)))
    
    假定缓行曲线上的最高速度为v，最快转角速度为omiga，前后轮采用同样的转角策略
    那么前后轮的迟滞时间为t = L/v

    当前后轮转角完全一致时，转弯半径达到最小，为Rmin
    此时，有R1=R2=Rmin, alpha=beta=
    ------------------------------
    row(任意一点的半径) = R(连接的圆半径)*ls(缓和曲线总长)/l(当前点的缓和曲线长度)
    beta(任意一点半径与起点半径的夹角) = l**2/(2*R*ls)
    缓行线上任一点坐标
        x = l - l**5/(40*R**2*ls**2)
        y = l**3/(6*R*ls) - l**7/(336*R**3*ls**3)
    圆曲线终点坐标:
        x0 = ls - ls**3/(40*R**2)
        y0 = ls**2/(6*R)
    内移距p
        p = ls**2 / (24*R)
    切线增长q
        q = ls/2 - ls**3/(240*R**2)
    切线长：
        TH(两条直线交点到直线起点的长度) = (R+p)*tan(alpha(缓行直线夹角)/2)+q
    曲线长：
        LH = R*(alpha-2*beta0)*pi/180 + 2*ls,
        其中：R*(alpha-2*beta0)*pi/180 为圆曲长
    外距：
        EH = (R+p)/cos(alpha/2) - R
    切曲差：
        DH = 2*TH - LH
    """
    def __init__(self):
        return

    def getPoints(self):
        points_list = []
        return points_list

    def calcFromCircleEP(self, x0, y0, radius):
        """
        根据圆曲线终点坐标计算缓和曲线ls
        根据圆参数计算内移距p(缓行圆与直线的最小距离)和切线增长q(缓行圆心相对直线垂线的偏移距离)
        """
        self.ls = np.sqrt(6 * y0 * radius)
        self.p = self.ls ** 2 / (24 * radius)
        self.q = self.ls / 2 - self.ls ** 3 / (240 * radius ** 2)
        return