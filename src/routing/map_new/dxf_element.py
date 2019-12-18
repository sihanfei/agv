# coding = utf-8
from ctypes import *

import numpy as np

from enum import Enum, unique

import matplotlib.pyplot as plt
import matplotlib.lines as ln

import dxfgrabber

from scipy import spatial

from map_element import (GPSPointDef, RefLineDef, AppendixObjectDef, RefPointDef)

class BaseEntityMethod:
    """
    根据dxf_entity来计算line2D
    todo 参考gps_point.py中getWidthinP 和 getDiAngle
    """
    # @staticmethod
    # def getRefPoints(dxf_entity, step, lanes, boards):
    #     ref_points = []
    #     gps_points = BaseEntityMethod.scatterGPSPoints(dxf_entity, step)
    #     for point in gps_points:
    #         ref_point = RefPointDef()
    #         ref_point.gps_ = point
    #         ref_point.lanes_ = lanes
    #         if dxf_entity.dxftype == 'LINE':                
    #             ref_point.cuv_ = 0
    #             ref_point.gcuv_ = 0
    #             ref_point.slength_ = GPSPointDef.calcDistance(point, gps_points[0])
    #             ref_point.theta_ = []
    #             for board in boards:
    #                 ref_point.width_list_ = []
    #                 pass
    #             pass
    #         elif dxf_entity.dxftype == 'ARC':
                
    #             pass
    #         else:
                
    #             pass
    #         ref_points.extend(ref_point)
    #         return ref_points
        
    @staticmethod
    def calcEntityLength(dxf_entity):
        length = 0
        if dxf_entity.dxftype == 'LINE':
            sp, ep = dxf_entity.start, dxf_entity.end
            length = np.sqrt((ep[0] - sp[0])**2 + (ep[1] - sp[1])**2)
            pass
        elif dxf_entity.dxftype == 'ARC':
            end_angle_ = dxf_entity.end_angle * np.pi / 180
            if dxf_entity.end_angle < dxf_entity.start_angle:
                start_angle_ = dxf_entity.start_angle * np.pi / 180 - 2 * np.pi
            else:
                start_angle_ = dxf_entity.start_angle * np.pi / 180
            length = dxf_entity.radius * (end_angle_ - start_angle_)
            pass
        else:
            pass
        return length

    @staticmethod
    def getEndPoints(dxf_entity):
        pass
        startp = []
        endp = []
        if dxf_entity.dxftype == 'LINE':
            startp = dxf_entity.start
            endp = dxf_entity.end
        elif dxf_entity.dxftype == 'ARC':
            startp = tuple(
                np.array(dxf_entity.center) + np.array([
                    np.cos(dxf_entity.start_angle * np.pi / 180),
                    np.sin(dxf_entity.start_angle * np.pi / 180)
                ]) * dxf_entity.radius)
            endp = tuple(
                np.array(dxf_entity.center) + np.array([
                    np.cos(dxf_entity.end_angle * np.pi / 180),
                    np.sin(dxf_entity.end_angle * np.pi / 180)
                ]) * dxf_entity.radius)
        return startp, endp

    @staticmethod
    def scatterGPSPoints(dxf_entity, step):
        """
        step: 希望离散成多长距离的点, 例如 0.5表示0.5米
        """
        length = BaseEntityMethod.calcEntityLength(dxf_entity)
        if dxf_entity.dxftype == 'LINE':
            sp, ep = dxf_entity.start, dxf_entity.end
            delta_x = (ep[0] - sp[0]) / length * step
            delta_y = (ep[1] - sp[1]) / length * step
            if delta_y == 0:
                xlist = np.arange(sp[0], ep[0], delta_x)
                ylist = np.ones(xlist.shape) * sp[1]
            elif delta_x == 0:
                ylist = np.arange(sp[1], ep[1], delta_y)
                xlist = np.ones(ylist.shape) * sp[0]
            else:
                xlist = np.arange(sp[0], ep[0], delta_x)
                ylist = np.arange(sp[1], ep[1], delta_y)
            xlist = np.append(xlist, ep[0])
            ylist = np.append(ylist, ep[1])
            points = tuple(zip(xlist, ylist))
            # print('LineEntity: scatter: line points={}'.format(
            #     (self.start, self.end, self.length)))
        elif dxf_entity.dxftype == 'ARC':
            if dxf_entity.start_angle > dxf_entity.end_angle:
                start_angle_ = dxf_entity.start_angle * np.pi / 180 - 2 * np.pi
            else:
                start_angle_ = dxf_entity.start_angle * np.pi / 180
            end_angle_ = dxf_entity.end_angle * np.pi / 180
            angle_step_ = step / dxf_entity.radius
            angle_list_ = np.arange(start_angle_, end_angle_,
                                    angle_step_)
            angle_list_ = np.append(angle_list_, end_angle_)
            xlist = np.cos(
                angle_list_) * dxf_entity.radius + dxf_entity.center[0]
            ylist = np.sin(
                angle_list_) * dxf_entity.radius + dxf_entity.center[1]
            points = tuple(zip(xlist, ylist))
            # print('arc points', points)
            # print('LineEntity: scatter: arc points={}'.format(self.angles))
        else:
            points = []
            pass
        return points

    @staticmethod
    def toPlotLine(dxf_entity, step, **kwds):
        points = BaseEntityMethod.scatterGPSPoints(dxf_entity, step)
        [x_, y_] = np.array(tuple((zip(*points))))
        line_ = ln.Line2D(x_, y_, **kwds)
        # to RefPointDef
        return line_


if __name__ == "__main__":

    # class dxf_entity:
    #     def __init__(self, dxftype):
    #         self.dxftype = dxftype

    #     def setStart(self, start):
    #         self.start = start

    #     def setEnd(self, end):
    #         self.end = end

    #     def setCenter(self, center):
    #         self.center = center

    #     def setStartAngle(self, start_angle):
    #         self.start_angle = start_angle

    #     def setEndAngle(self, end_angle):
    #         self.end_angle = end_angle

    #     def setRadius(self, radius):
    #         self.radius = radius

    # entity0_ = dxf_entity('LINE')
    # entity0_.setStart([0, 0, 0])
    # entity0_.setEnd([10, 10, 0])

    # line0_ = BaseEntityMethod.toPlotLine(entity0_, 0.5)

    # entity1_ = dxf_entity('ARC')
    # entity1_.setStartAngle(3 / 4 * 2*np.pi)
    # entity1_.setEndAngle(0)
    # entity1_.setCenter([10, 10, 0])
    # entity1_.setRadius(5)
    # line1_ = BaseEntityMethod.toPlotLine(entity1_, 0.5)
    # fig = plt.figure()
    # ax = fig.add_subplot(111)

    # file = dxfgrabber.readfile('d:/my_code/map_tools/data/base_map.dxf')

    # for entity in file.entities:
    #     line0_ = BaseEntityMethod.toPlotLine(entity, 0.5)
    #     ax.add_line(line0_)
    # ax.margins(0)
    # ax.grid(True)

    # print(GPSPointDef.calcDistance(GPSPointDef(0, 0, 0), GPSPointDef(1, 1, 0)))

    # plt.show()
    # pass
    p0 = GPSPointDef(0.1, 1.1, 2.1)
    print(p0.toStr())
    p0.fromStr('1,2,3')
    print(p0)
    p0=[1,2]

        #     style_list.append('{}'.format(self.gps_[0]))
        # style_list.append('{}'.format(self.gps_[1]))
        # style_list.append('{}'.format(self.gps_[2]))
        # style_list.append('{}'.format(self.lanes_))
        # for width in self.width_list_:
        #     style_list.append('{}'.format(width[0]))
        #     style_list.append('{}'.format(width[1]))
        # style_list.append('{}'.format(self.cuv_))
        # style_list.append('{}'.format(self.gcuv_))
        # style_list.append('{}'.format(self.slength_))
        # style_list.append('{}'.format(self.theta_))
        # style_str = ','.join(style_list)
    
    data = '1,2,3,2,3,1,1,-1,1.1,2.1,10.0,3.14'
    p = RefPointDef()
    p.fromStr(data)
    print('lanes', p.lanes_)
    for width in p.width_list_:
        print('width', width)
