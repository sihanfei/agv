# coding : utf-8
import tkinter
from tkinter import ttk
from tkinter import filedialog
from tkinter import messagebox

from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure
from matplotlib import lines

import numpy as np

from scipy import spatial

import dxfgrabber

import dxf_element
from dxf_element import BaseEntityMethod
from attribute_def import (APPENDIX_ATTRIBUTE_VALUE, ROAD_AREA_VALUE)

from enum import Enum, unique

title_text = ['select ref_line', 'select the start point', 'Config the attribute', 'select the board', 'config the board', 'select the appendix', 'config the appendix']

def checklineInFile(file_dir, index):
    with open(file_dir, 'r') as file:
        lines = file.readlines()
        for line in lines:
            ref_line = dxf_element.RefLineDef()
            ref_line.fromStr(line)
            if ref_line.index == index:
                return True
        return False

def rewriteRefPointsToFile(file, index):
    pass

def calcPointsDistance(p0, p1):
    value = 0
    for i in range(len(p0)):
        value += (p1[i] - p0[i])**2
    return(np.sqrt(value))


def calcAngle(p0, p1, p2):
    """
    计算有向线段p0-p1与p0-p2的夹角(p01向p02旋转，-pi~pi, 逆时针为正）, 不考虑z轴位移
    """
    angle01_ = np.arctan2(p1[1]-p0[1], p1[0]-p0[0])
    if angle01_ < 0:
        angle01_ += 2*np.pi
    angle02_ = np.arctan2(p2[1]-p0[1], p2[0]-p0[0])
    if angle02_ < 0:
        angle02_ += 2*np.pi
    # print('angle01={}, angle02={}'.format(angle01_, angle02_))
    angle = angle02_ - angle01_
    if angle > np.pi:
        angle -= 2*np.pi
    elif angle < -np.pi:
        angle += 2*np.pi
    return angle

@unique
class DataProcSchedule(Enum):
    """
    处理步骤枚举定义
    """
    START = 0
    REF_PICKED = 1
    EPS_PICKED = 2
    REF_CONFIGED = 3
    BOARD_PICKED = 4
    BOARD_COMBINED = 5
    POINTS_DONE = 6

@unique
class AppendixSchedule(Enum):
    """
    附属物处理步骤
    """
    START = 0
    GET_BELONG = 1
    CONFIG = 2
    SAVE = 3

class DataProcess:
    """
    基于matplotlib的数据处理方法
    """
    def __init__(self):
        self.schedule = DataProcSchedule.START
        self.dxfentity_dict = {}
        self.ref_line_dict = {} # 保存好，处理好的refLine
        self.handle_ref_flag = True
        self.tips_enable = False
        self.initALineVarials()
        self.initAppendixVarials()

        self.appendix_dict = {}

        self.ref_line_style = '-.'
        self.appendix_line_style = '-'
        self.board_line_style = '--'

        self.total_road_cnt = 0
        self.saved_road_cnt = 0

        self.margins = 0
        pass

    def initAppendixVarials(self):
        self.current_appendix_id = 0
        self.current_appendix_to_id = 0
        self.current_appendix_line_list = []
        self.appendix_schedule = AppendixSchedule.START # 'PICK_BELONGS'/'CONFIG'
        # self.appendix_id = 0

    def resetBoardLinesDisplay(self):
        for line in self.total_board_lines_list:
            line.set_picker(3)
            line.set_color('brown')
        
    def setRefLinePicker(self, radius):
        """
        设置全部refline的picker属性
        """
        ax = self.figure.gca()
        for line in ax.lines:
            if line.get_linestyle() == self.ref_line_style:
                line.set_picker(radius)
        pass

    def setBoardLinePicker(self, radius):
        """
        设置全部boardline的picker属性
        """
        ax = self.figure.gca()
        for line in ax.lines:
            if line.get_url() == 'board':
                line.set_picker(radius)

    def setAppendixLinePicker(self, radius):
        """
        设置全部appendixline的picker属性
        """
        ax = self.figure.gca()
        for line in ax.lines:
            if line.get_linestyle() == self.appendix_line_style:
                line.set_picker(radius)
        pass

    def combineBoardLines(self):
        """
        根据当前ref_line的起点、board_line的数据，将board_line组合为需要的board
        """
        board = []
        # 计算tree与start_point的距离
        for line in self.current_board_lines_list:
            tree = spatial.KDTree(line)
            dist, index = tree.query(self.current_line_startpoint)
        self.current_boards_list.append(board)

    def removeLineEPs(self, figure):
        """
        关闭线段端点的显示
        """
        ax = figure.gca()
        ax.lines.remove(self.line2d_end_id) # 将端点直接从ax的line2d中移除
        ax.lines.remove(self.line2d_start_id)
        figure.canvas.draw()

    def resetRefLineDisplay(self, color):
        self.current_line.set_color(color)

    def saveRefLineBaseAttribute(self, user_info):
        """
        保存参考线的基本属性到self.ref_line_dict中
        """
        print('ref_line_config_done!')
        attribute = dxf_element.RefLineDef()
        attribute.index = self.current_line_id
        self.speed_lower = user_info[0]
        self.speed_upper = user_info[1]
        attribute.speed_limited = [user_info[0], user_info[1]]
        attribute.height_limited = user_info[2]
        dxf_entity = self.dxfentity_dict[self.current_line_id]
        if dxf_entity.dxftype == 'LINE':
            attribute.curv = 0.0
        elif dxf_entity.dxftype == 'ARC':
            attribute.curv = 1/dxf_entity.radius
        self.lanes_number = int(user_info[3])
        attribute.lanes_number = user_info[3]
        self.ratio = float(user_info[4])
        attribute.ratio = user_info[4] 
        self.traffic_direct = user_info[5]
        direction = 0
        for i, value in enumerate(user_info[5]): # 
            direction += 2**(i)*int(value)
        attribute.direction = direction
        self.road_material = user_info[6]
        attribute.material = user_info[6]
        attribute.area = user_info[7]
        attribute.slength = BaseEntityMethod.calcEntityLength(self.dxfentity_dict[self.current_line_id])
        attribute.startpoint = dxf_element.GPSPointDef(self.current_line_startpoint[0], self.current_line_startpoint[1])
        attribute.endpoint = dxf_element.GPSPointDef(self.current_line_endpoint[0], self.current_line_endpoint[1])
        # save data
        self.ref_line_dict[self.current_line_id] = attribute
        # 如果是双向车道，那么把id置成负数
        if self.traffic_direct[3] == 1:
            Nattribute = dxf_element.RefLineDef()
            print('bidirect get!')
            Nattribute.copyFrom(attribute)
            Nattribute.index = -Nattribute.index
            Nattribute.startpoint, Nattribute.endpoint = Nattribute.endpoint, Nattribute.startpoint
            self.ref_line_dict[-self.current_line_id] = Nattribute
            pass
        if self.tips_enable:
            messagebox.showinfo(title='下一步', message='请选择参考线对应的某侧道路边界(由左到右，由远到近)')
        pass

    def toggleTipsEnable(self):
        """
        提示信息使能
        """
        self.tips_enable = not self.tips_enable

    def loadRefLineData(self):
        """
        将已经存在的ref_line数据载入到self.ref_line_dict中
        """
        try:
            ref_line_file_dir = self.fold_dir + '/ref_line.json'
            ref_line_file = open(ref_line_file_dir, 'r') 
            lines = ref_line_file.readlines()
            self.saved_road_cnt = 0
            self.saved_road_length = 0
            self.bidirection_road_cnt = 0
            for line in lines:
                ref_line = dxf_element.RefLineDef()
                ref_line.fromStr(line)
                if ref_line.index > 0:
                    self.saved_road_cnt += 1
                    self.saved_road_length += ref_line.slength
                else:
                    self.bidirection_road_cnt += 1
                self.ref_line_dict[ref_line.index] = ref_line
                print('key:gps= {}:{}'.format(ref_line.index, ref_line.startpoint))
            ref_line_file.close()
            messagebox.showinfo(title='完成', message='参考线数据导入成功')
            pass
        except FileNotFoundError as identifier:
            messagebox.showinfo(title='完成', message='无参考线信息')
            return
        finally:
            pass    
        pass

    def loadAppendixData(self):
        """
        从appendix_atrribute.json中载入数据到self.appendix_dict
        """
        try:
            file_name = self.fold_dir + '/appendix_attribute.json'
            file = open(file_name, 'r')
            lines = file.readlines()
            for line in lines:
                print('run load', line)
                appendix = dxf_element.AppendixObjectDef()
                appendix.fromStr(line)
                self.appendix_dict[appendix.index_] = appendix
                print('run load', appendix.position_)
            file.close()
        except FileNotFoundError as identifier:
            return
        finally:
            pass
        return

    def loadData(self, dir, figure, ref_layer=['ref_layer'], board_layer=['board_layer'], appendix_layer=['appendix_layer']):
        """
        从dxf文件中读入数据
        """
        self.figure = figure
        self.fold_dir = dir
        # read exist file
        self.loadRefLineData()
        self.loadAppendixData()

        file_dir = str(dir)+('/base_map.dxf')
        total_lines = []
        self.scatter_step = 0.5
        self.ref_line2D_dict = {}
        try:
            file = dxfgrabber.readfile(file_dir)
            # file = dxfgrabber.readfile('d:/my_code/map_tools/data/base_map.dxf')
            road_id = 0
            appendix_id = 0
            for entity in file.entities:
                if entity.layer in ref_layer:
                    road_id += 1
                    line0 = BaseEntityMethod.toPlotLine(entity, self.scatter_step, picker=0, url=road_id, color='g', linestyle=self.ref_line_style, linewidth=2)
                    if line0.get_url() in self.ref_line_dict.keys():
                        line0.set_color('k')
                        # line0.set_linewidth(1)
                        # line0.set_alpha(0.5)
                    total_lines.append(line0)
                    self.dxfentity_dict[road_id] = entity
                    self.ref_line2D_dict[road_id] = line0
                elif entity.layer in board_layer:
                    line0 = BaseEntityMethod.toPlotLine(entity, self.scatter_step, picker=0, url='board', color='brown', linestyle=self.board_line_style)
                    total_lines.append(line0)
                elif entity.layer in appendix_layer:
                    appendix_id += 1
                    line0 = BaseEntityMethod.toPlotLine(entity, self.scatter_step, picker=0, url=appendix_id, color='orange', linestyle=self.appendix_line_style, linewidth=2)
                    if line0.get_url() in self.appendix_dict.keys():
                        line0.set_color('k')
                        # line0.set_linewidth(1)
                        # line0.set_alpha(0.5)
                    total_lines.append(line0)
        except FileNotFoundError or IOError:
            messagebox.showerror(title='错误', message='文件不存在或被占用')
            return total_lines
        finally:
            if self.tips_enable:
                messagebox.showinfo(title='第一步', message='请选择一条道路参考线')
            self.total_road_cnt = road_id
            return total_lines 

    def addNewLine(self, line):
        return

    def drawLines(self, ax, lines):
        """
        显示多条line2d数据
        """
        for line in lines:
            ax.add_line(line)
        ax.margins(0.05)
        ax.set_title('Select Ref_line')
        self.xmin, self.xmax = ax.get_xlim()
        self.ymin, self.ymax = ax.get_ylim()
        self.xscale = (self.xmax - self.xmin) / 10
        self.yscale = (self.ymax - self.ymin) / 10
    
    def onButtonClick(self, event):
        pass

    def getLine(self, picked_line):
        """
        获取被pick线段的line2d入口和id
        """
        self.current_line = picked_line
        self.current_line_id = picked_line.get_url()
        pass
    
    def getEndPoints(self, m_point):
        """
        根据m_point，得到被pick线段的两个端点中离m_point较近的那个点
        """
        line_data = self.current_line.get_xydata()
        # self.ref_lines[-1].set_color('k')
        l0 = calcPointsDistance(m_point, line_data[0])
        l1 = calcPointsDistance(m_point, line_data[-1])
        if l0 < l1:
            self.current_ep_changed = False
            self.current_line_startpoint = line_data[0] # [0], line_data[0][1], 0]
            self.current_line_endpoint = line_data[-1] # [0], line_data[-1][1], 0]
        else:
            self.current_ep_changed = True
            print('orig point:', line_data[0], line_data[-1])
            for xy in line_data:
                print('out_line:', xy)
            line_data = np.flip(line_data, 0) # 翻转数据
            self.current_line.set_data(line_data[:,0], line_data[:,1])
            line_data = self.current_line.get_xydata()
            print('flip point:', line_data[0], line_data[-1])            
            self.current_line_startpoint = line_data[0] # [0], line_data[0][1], 0]
            self.current_line_endpoint = line_data[-1] # [0], line_data[-1][1], 0]
        pass

    def sortBoards(self):
        """
        对board进行重排序，保证按照由左到右（按照行车方向）排列
        1.计算board中与start最近点
        2.计算该最近点与行车方向的夹角，当为正时，在左侧；反之在右侧
        """
        dist_list_ = []
        data_list_ = []
        for tree in self.current_boards_tree_list: # 获取各条道路边界上的最近点
            dist_, index_ = tree.query(self.current_line_startpoint, 1)
            dist_list_.append(dist_)
            data_list_.append(tree.data[index_])
        # 计算角度
        xy_data_ = self.current_line.get_xydata()
        p0_ = xy_data_[0]
        p1_ = xy_data_[2]
        angle_list_ = []
        angle_index_over0_ = []
        angle_index_below0_ = []
        for i_, p2_ in enumerate(data_list_): # 计算道路边界与车道线的夹角
            angle_ = calcAngle(p0_, p1_, p2_)
            angle_list_.append(angle_)
            print(angle_)
            if angle_ > 0:
                angle_index_over0_.append(i_)
            else:
                angle_index_below0_.append(i_)

        if len(angle_index_over0_) == 0 or len(angle_index_below0_) == 0:
            print('over list:{}'.format(angle_index_over0_))
            print('below list:{}'.format(angle_index_below0_))
            return False
        else:
            # 在angle>0值与<0值中，分别对dist_list_进行排序
            dist_over0_ = list(dist_list_[i] for i in angle_index_over0_)
            dist_over0_.sort(reverse = True)
            self.current_board_sign_list = list(1 for _ in dist_over0_)
            dist_below0_ = list(dist_list_[i] for i in angle_index_below0_)
            dist_below0_.sort(reverse = True)
            self.current_board_sign_list.extend(list(-1 for _ in dist_below0_))
            dist_over0_.extend(dist_below0_) # 把距离排序拼接
            # 获取距离的原序号，并对boards_list 和 tree_list进行重排
            orig_index_ = list(dist_over0_.index(value) for value in dist_list_)

            self.current_boards_list = list(self.current_boards_list[i] for i in orig_index_)
            self.current_boards_tree_list = list(self.current_boards_tree_list[i] for i in orig_index_)
            # self.current_boards_tree_list[dist_index_over0_]
            return True

    def saveRefLineDict(self, file_dir): # todo change name
        """
        因为self.ref_line_dict中保存了所有数据，因此每次都是覆盖写
        """
        self.saved_road_cnt = 0
        self.bidirection_road_cnt = 0
        with open(file_dir, 'w+') as w:
            for value in self.ref_line_dict.values():
                if value.index > 0:
                    self.saved_road_cnt += 1
                    self.saved_road_length += value.slength
                else:
                    self.bidirection_road_cnt += 1
                data = value.toStr()
                # print('saveRefLineDict:', data)
                w.write(data)
                w.write('\n')

    def writeCurrentRefPoints(self, file_dir):
        """
        追加写
        因为ref_points仅保存了当前的车道参考线信息，所以写之前，必须先调用
        deleteCurrentRefPointsInFile
        然后再写入，不然就会出现两个同样ID的数据
        """
        with open(file_dir, 'a+') as file:
            for ref_point in self.ref_line_points:
                file.write(str(self.current_line_id))
                file.write(',')
                file.write(ref_point.toStr())
                file.write('\n')
            if -self.current_line_id in self.ref_line_dict.keys(): # 双向
                slength = self.ref_line_points[-1].slength_
                self.ref_line_points.reverse()
                for ref_point in self.ref_line_points:
                    file.write(str(-self.current_line_id))
                    file.write(',')
                    ref_point.slength_ = slength - ref_point.slength_
                    if self.dxfentity_dict[self.current_line_id].dxftype == 'LINE':
                        ref_point.theta_ = np.pi * 2 - ref_point.theta_
                    file.write(ref_point.toStr())
                    file.write('\n')
                    


    def deleteCurrentRefPointsInFile(self, file_dir):
        """
        删除原文件中，与self.current_line_id具有相同id的ref_points数据
        """
        # 先判断self.current_line_id和-self.current_line_id是否在self.dict中存在
        keys = self.ref_line_dict.keys()
        if self.current_line_id not in keys and -self.current_line_id not in keys:
            return        
        try:
            r = open(file_dir, 'r')
            lines = r.readlines()
            with open(file_dir, 'w+') as w:
                for line in lines:
                    value = line.split(',', 1)
                    key = int(value[0])
                    key = abs(key)
                    if key != self.current_line_id: # 正负转换为同一个
                        w.write(line)
            r.close()
        except FileNotFoundError as identifier:
            return
        
    def saveCurrentAppendixData(self):
        """
        保存当前选择并配置的appendix数据
        """
        file_name = self.fold_dir + '/appendix_attribute.json'
        with open(file_name, 'w+') as w:
            for key in self.appendix_dict.keys():
                w.write(self.appendix_dict[key].toStr())
                w.write('\n')
                # print('save appendix run here')
        # 保存
        file_name = self.fold_dir + '/ref_line_appendix.json'
        ref_line_appendix = {}
        for key in self.appendix_dict.keys():
            value = self.appendix_dict[key]
            if value.belongingID_ in ref_line_appendix.keys():
                ref_line_appendix[value.belongingID_].extend([key])
            else:
                ref_line_appendix[value.belongingID_] = list([key])
        with open(file_name, 'w+') as w:
            for key in ref_line_appendix.keys():
                value = ref_line_appendix[key]
                w.write(str(key))
                for data in value:
                    w.write(',')
                    w.write(str(data))
                w.write('\n')
        return

    def saveCurrentRefLineData(self):
        """
        1.保存全部ref_line_dict（w+）方式
        2.保存当前全部refPoints（a+）
        2.1 先判断这条道路之间是否已经保存过，如果是，删除
        2.2 追加写入
        2.2.1 需要判断是否存在bidirect
        """
        # save ref_line_dict
        dict_name = self.fold_dir + '/ref_line.json'
        self.saveRefLineDict(dict_name)
        # check self.current_line_id in refpoint.json
        points_name = self.fold_dir + '/ref_points.json'
        self.deleteCurrentRefPointsInFile(points_name)
        self.writeCurrentRefPoints(points_name)

    def onWheel(self, event):
        """
        以鼠标位置为中心进行缩放
        """
        direction = event.button
        x_center = event.xdata
        y_center = event.ydata
        ax = event.inaxes
        x_min, x_max = ax.get_xlim()
        x_len = x_max - x_min
        y_min, y_max = ax.get_ylim()
        y_len = y_max - y_min
        if direction == "down":
            x_l = max(x_center - x_len, self.xmin)
            x_r = min(x_center + x_len, self.xmax)
            x_lim = (min(x_l, x_r), max(x_l, x_r))
            y_l = max(y_center - y_len, self.ymin)
            y_r = min(y_center + y_len, self.ymax)
            y_lim = (min(y_l, y_r), max(y_l, y_r))
        elif direction == 'up':
            x_l = max(x_center - x_len/4, self.xmin)
            x_r = min(x_center + x_len/4, self.xmax)
            y_l = max(y_center - y_len/4, self.ymin)
            y_r = min(y_center + y_len/4, self.ymax)
            x_lim = (min(x_l, x_r), max(x_l, x_r))
            y_lim = (min(y_l, y_r), max(y_l, y_r))
        ax.set_xlim(x_lim)
        ax.set_ylim(y_lim)
        self.figure.canvas.draw()
        return

    def makeConnectMap(self):
        """
        理论上所有数据已经存在self.ref_line_dict中
        所以每次都是直接重写
        """
        file_name = self.fold_dir + '/connect_map.json'
        self.connect_dict = {}
        start_point_list = []
        end_point_list = []
        key_list = []
        # 根据ref_line_dict，得到connect_dict
        for key in self.ref_line_dict.keys():
            ref_line = self.ref_line_dict[key]
            start_point_list.append(ref_line.startpoint)
            end_point_list.append(ref_line.endpoint)
            key_list.append(key)
            self.connect_dict[key] = list([])
        print('end=', end_point_list)
        print('start=', start_point_list)

        for i, point0 in enumerate(end_point_list):
            for j, point1 in enumerate(start_point_list):
                """
                当终点和起点之间的距离小于0.5m(理论可以更小)，则认为是相接的两条车道。需要排除双向车道
                """
                if calcPointsDistance(point0, point1) < 0.5 and abs(key_list[i]) != abs(key_list[j]):
                    self.connect_dict[key_list[i]].extend([key_list[j]])
        with open(file_name, 'w+') as w:
            for key in self.connect_dict.keys():
                w.write(str(key))
                for value in self.connect_dict[key]:
                    w.write(',')
                    w.write(str(value))
                w.write('\n')
        return

    def getRefLinePoints(self):
        """
        直接取ref_line上各点与board_line的最近点作为距离
        """
        if self.sortBoards(): # 对boardline进行排序，按照行车方向，以左侧为正，由远到近
            dxf_entity = self.dxfentity_dict[self.current_line_id]
            self.ref_line_points = [] # 建立一个列表，存储参考线点数据
            xy_data = self.current_line.get_xydata() # 获取当前gps坐标
            slength = 0
            theta = 0
            # self.slength_ = 0
            if dxf_entity.dxftype == 'LINE':
                for i, xy in enumerate(xy_data):
                    ref_point = dxf_element.RefPointDef() # 单个点
                    ref_point.gps_ = dxf_element.GPSPointDef(xy[0], xy[1])
                    ref_point.lanes_ = self.lanes_number
                    dist_list = []
                    for i, tree in enumerate(self.current_boards_tree_list): # 获取车道宽度
                        dist, _ = tree.query(xy, 1)
                        dist_list.append(dist)
                    ref_point.width_list_ = list([dist_list[i]*self.current_board_sign_list[i], \
                                                  dist_list[i+1]*self.current_board_sign_list[i+1]] \
                                                  for i in range(len(dist_list)-1))
                    ref_point.cuv_ = 0
                    ref_point.gcuv_ = 0
                    ref_point.theta_ = calcAngle(self.current_line_startpoint, \
                                           [self.current_line_startpoint[0], self.current_line_startpoint[1]+1], \
                                            self.current_line_endpoint) # 因为是直线，可以用终点与起点形成的线段与y轴的夹角
                    if ref_point.theta_ < 0:
                        ref_point.theta_ += np.pi * 2
                    slength = calcPointsDistance(xy, self.current_line_startpoint)
                    # slength += self.scatter_step
                    # print('Line Slength=', slength)
                    ref_point.slength_ = slength # 实际就是step
                    self.ref_line_points.extend([ref_point])
            elif dxf_entity.dxftype == 'ARC':
                # add 
                if dxf_entity.start_angle > dxf_entity.end_angle:
                    start_angle_ = dxf_entity.start_angle * np.pi / 180 - 2 * np.pi
                else:
                    start_angle_ = dxf_entity.start_angle * np.pi / 180
                start_angle_ -= np.pi/2
                end_angle_ = dxf_entity.end_angle * np.pi / 180
                end_angle_ -= np.pi/2
                angle_step_ = self.scatter_step / dxf_entity.radius
                angle_list_ = np.arange(start_angle_, end_angle_,
                                        angle_step_)
                angle_list_ = np.append(angle_list_, end_angle_)
                if self.current_ep_changed: # 逆序
                    angle_list_ = angle_list_[-1::-1]
                for i, xy in enumerate(xy_data):
                    ref_point = dxf_element.RefPointDef() # 单个点
                    ref_point.gps_ = dxf_element.GPSPointDef(xy[0], xy[1])
                    ref_point.lanes_ = self.lanes_number
                    dist_list = []
                    for j, tree in enumerate(self.current_boards_tree_list): # 获取车道宽度
                        dist, _ = tree.query(xy, 1)
                        dist_list.append(dist)
                    ref_point.width_list_ = list([dist_list[k]*self.current_board_sign_list[k], dist_list[k+1]*self.current_board_sign_list[k+1]] for k in range(len(dist_list)-1))
                    ref_point.cuv_ = 1/dxf_entity.radius
                    ref_point.gcuv_ = 0
                    print('current i=', i)
                    if angle_list_[i] >= 0:
                        ref_point.theta_ = angle_list_[i]
                    else:
                        ref_point.theta_ = angle_list_[i] + 2*np.pi
                    slength = dxf_entity.radius * abs(angle_list_[i] - angle_list_[0])
                    # print('Arc Slength=', slength)
                    ref_point.slength_ = slength # 实际就是step
                    self.ref_line_points.extend([ref_point])
            return True
        else:
            return False

    def deleteALine(self):
        pass


    def reflineHandler(self, event):
        """
        参考线处理主流程
        """
        if self.schedule == DataProcSchedule.START and event.artist.get_linestyle() == self.ref_line_style:
            self.getLine(event.artist)
            self.current_line.set_color('r')
            self.schedule = DataProcSchedule.REF_PICKED
            if self.tips_enable:
                messagebox.showinfo(title='下一步', message='请选择道路参考线起点')
        # 选择起点
        elif self.schedule == DataProcSchedule.REF_PICKED and event.artist.get_linestyle() == self.ref_line_style:
            m_point = [event.mouseevent.xdata, event.mouseevent.ydata]
            self.getEndPoints(m_point)
            # 为方便观察，将起点和终点显示出来
            self.line2d_start = lines.Line2D([self.current_line_startpoint[0]], [self.current_line_startpoint[1]], marker = 'o', color='r')
            self.line2d_end = lines.Line2D([self.current_line_endpoint[0]], [self.current_line_endpoint[1]], marker = 'o', color='k')
            ax = event.artist.figure.gca()
            self.line2d_start_id = ax.add_line(self.line2d_start)
            self.line2d_end_id = ax.add_line(self.line2d_end)
            event.canvas.draw()
            
            self.schedule = DataProcSchedule.EPS_PICKED
            self.setRefLinePicker(0) # 关闭参考线选择
            if self.tips_enable:
                messagebox.showinfo(title='下一步', message='请设置道路参考线属性')
        # 选择车道边界
        elif self.schedule == DataProcSchedule.REF_CONFIGED and self.lanes_number == 0:
            # print('请先对参考线进行配置！')
            messagebox.showerror(title='Error Occur', message='请先对参考线进行设定！')
            pass
        elif self.schedule == DataProcSchedule.REF_CONFIGED and event.artist.get_url() != 'board' and not self.lanes_number == 0:
            messagebox.showwarning(title='Warning Occur', message='请选择车道边界线！')
        elif self.schedule == DataProcSchedule.REF_CONFIGED and event.artist.get_url() == 'board' and not self.lanes_number == 0:
            if event.artist.get_color() == 'r': # 通过再次点选，更改车道边界已经选择的对象，并将之移除list中
                event.artist.set_color('grey')
                if self.current_board_line_cnt > 0:
                    self.current_board_line_cnt -= 1
                    self.current_board_lines_list.remove(event.artist)
            else: 
                event.artist.set_color('r')
                self.current_board_lines_list.append(event.artist) # 
                self.total_board_lines_list.append(event.artist)
                self.current_board_line_cnt += 1
            event.canvas.draw()

    def appendixHandler(self, event):
        """
        附属物主处理函数
        """
        artist = event.artist
        if artist.get_linestyle() == self.appendix_line_style and self.appendix_schedule == AppendixSchedule.START:
            artist.set_color('r')
            xy_data = artist.get_xydata()
            self.current_appendix_id = artist.get_url()
            current_appendix_object = dxf_element.AppendixObjectDef(self.current_appendix_id)
            current_appendix_object.position_ = [dxf_element.GPSPointDef(xy_data[0][0], xy_data[0][1]), dxf_element.GPSPointDef(xy_data[-1][0], xy_data[-1][1])]
            self.appendix_dict[self.current_appendix_id] = current_appendix_object
            self.current_appendix_line_list.append(artist)
            return
        elif artist.get_linestyle() == self.ref_line_style and self.appendix_schedule == AppendixSchedule.GET_BELONG:
            artist.set_color('r')
            event.canvas.draw()
            self.current_appendix_line_list.append(artist)
            self.current_appendix_to_id = artist.get_url()
            # self.current_appendix_object.belongingID_ = self.current_appendix_to_id
            self.appendix_dict[self.current_appendix_id].belongingID_ = self.current_appendix_to_id
            messagebox.showinfo(title='', message='道路参考线ID为{}'.format(self.current_appendix_to_id))
            self.setRefLinePicker(0)
            self.setAppendixLinePicker(3)
            self.appendix_schedule = AppendixSchedule.CONFIG
        else:
            messagebox.showwarning(title='错误', message='当前为附属物处理流程，请选择道路附属物')
            return

    def resetAppendixLineListColor(self):
        for line in self.current_appendix_line_list:
            print('run here')
            if line.get_linestyle() == self.appendix_line_style:
                line.set_color('k')
            else:
                line.set_color('g')
        return

    def onPick(self, event):
        """
        pick event回调函数
        """
        mouse_event = event.mouseevent
        if mouse_event.button == 1:
            # print('flag=', self.handle_ref_flag)
            if self.handle_ref_flag:
                self.reflineHandler(event)
            elif not self.handle_ref_flag:
                self.appendixHandler(event)
                pass 
        elif mouse_event.button == 3:
            if self.handle_ref_flag:
                if self.schedule == DataProcSchedule.REF_PICKED:
                    self.current_line.set_color('g')
                    self.initALineVarials()
                    self.setBoardLinePicker(0)
                    if self.handle_ref_flag:
                        self.setRefLinePicker(3)
                        self.setAppendixLinePicker(0)
                    else:
                        self.setRefLinePicker(0)
                        self.setAppendixLinePicker(3)
        event.canvas.draw()

    def checkBoardLines(self):
        """
        检查boardlines是否是有效的
        依据：是否首尾相连
        """
        eps = []
        for line in self.current_board_lines_list:
            xy_data = line.get_xydata()
            eps.append(xy_data[0])
            eps.append(xy_data[-1])
        
        ep_cnt = 0
        for i in range(len(eps)):
            for j in range(len(eps)):
                ep0 = eps[i]
                ep1 = eps[j]
                if i!=j and calcPointsDistance(ep0, ep1) < 1.5:
                    break
            else:
                ep_cnt += 1
                if ep_cnt > 2:
                    return False
        return True
        # for ep0 in eps:
        #     for ep1 in eps:
        #         if (ep0 == ep1).all():
        #             pass
        #         else:
        #             if calcPointsDistance(ep0, ep1) < 1.5: # 找到了一个临近点
        #                 break
        #     else: # 整个循环都没有找到临近点
        #         ep_cnt += 1
        #         if ep_cnt > 2: # 正常只有2个端点找不到临近点
        #             return False
        # return True # 首位不相连



    def initALineVarials(self):
        """
        初始化 一次 ref_line 处理所需要的变量
        """
        self.schedule = DataProcSchedule.START
        self.speed_lower = None
        self.speed_upper = None
        self.lanes_number = 0
        self.traffic_direct = None
        self.road_material = None
        self.ratio = 0

        self.current_line = None # line2D
        self.current_line_id = None # line2D_url
        self.current_line_startpoint = None # start_point
        self.current_line_endpoint = None # end_point
        self.current_boards_number = 0 # boards_number = lanes_number + 1
        self.current_boards_list = [] # 完整车道线（每条车道线由多条Board_line组成）列表
        self.current_boards_tree_list = [] # 完整车道线KDtree列表
        self.current_board_lines_list = [] # 单条完整车道线由多条board_line组成
        self.total_board_lines_list = []
        self.current_board_line_cnt = 0 # 单挑完整车道线的board_line数量

    def callbackConnect(self, canvas):
        canvas.mpl_connect('pick_event', self.onPick)
        canvas.mpl_connect('scroll_event', self.onWheel)

class AppendixSaveWidget(tkinter.Toplevel):
    """
    input：appendix_id, ref_line_id
    1.选择appendix
    2.选择ref_line
    3.配置
    """
    def __init__(self, appendix_id, ref_line_id):
        super().__init__()
        self.title('设置附属物信息')
        # self.area_type = tkinter.StringVar()
        self.setupUI(appendix_id, ref_line_id)
        self.userinfo = None

    def setupUI(self, appendix_id, ref_line_id):
        row1 = tkinter.Frame(self)
        row1.pack(fill="x")
        tkinter.Label(row1, text='附属物ID：', width=12).pack(side=tkinter.LEFT)
        tkinter.Label(row1, text='{}'.format(appendix_id), width=6).pack(side=tkinter.LEFT)        
        #
        row2 = tkinter.Frame(self)
        row2.pack(fill="x")
        tkinter.Label(row2, text='车道参考系ID：', width=12).pack(side=tkinter.LEFT)
        tkinter.Label(row2, text='{}'.format(ref_line_id), width=6).pack(side=tkinter.LEFT)        
        # 第n行
        row7 = tkinter.Frame(self)
        row7.pack(fill="x")
        tkinter.Button(row7, text="取消", command=self.cancel).pack(side=tkinter.RIGHT, padx = 6, ipadx = 4)
        ok_bt = tkinter.Button(row7, text="确定", command=self.ok)
        ok_bt.pack(side=tkinter.RIGHT, ipadx = 4)
        ok_bt.focus_set()
        return

    def ok(self):
        # print(self.box['value'].index(self.box.get()))
        self.userinfo = 1
        self.destroy() # 销毁窗口
    
    def cancel(self):
        self.userinfo = None # 空！
        self.destroy()

class AppendixAttributeWidget(tkinter.Toplevel):
    """
    input：appendix_id, ref_line_id
    1.选择appendix
    2.选择ref_line
    3.配置
    """
    def __init__(self, appendix_id):
        super().__init__()
        self.title('设置附属物信息')
        self.area_type = tkinter.StringVar()
        self.setupUI(appendix_id)
    
    def setupUI(self, appendix_id):
        row1 = tkinter.Frame(self)
        row1.pack(fill="x")
        tkinter.Label(row1, text='附属物ID：', width=12).pack(side=tkinter.LEFT)
        tkinter.Label(row1, text='{}'.format(appendix_id), width=6).pack(side=tkinter.LEFT)        
        #
        # row2 = tkinter.Frame(self)
        # row2.pack(fill="x")
        # tkinter.Label(row2, text='附属物ID：', width=12).pack(side=tkinter.LEFT)
        # tkinter.Label(row2, text='{}'.format(ref_line_id), width=6).pack(side=tkinter.LEFT)        
        #
        row_area_type = tkinter.Frame(self)
        row_area_type.pack(fill="x", ipadx=1, ipady=1)
        tkinter.Label(row_area_type, text='附属物属性', width=12).pack(side=tkinter.LEFT)
        box_area_type = ttk.Combobox(row_area_type, textvariable=self.area_type, width=12)
        box_area_type.pack(side=tkinter.LEFT, padx = 6)
        box_area_type['value'] = APPENDIX_ATTRIBUTE_VALUE
        box_area_type.current(1)
        self.box_area_type = box_area_type
        # 第n行
        row7 = tkinter.Frame(self)
        row7.pack(fill="x")
        tkinter.Button(row7, text="取消", command=self.cancel).pack(side=tkinter.RIGHT, padx = 6, ipadx = 4)
        ok_bt = tkinter.Button(row7, text="确定", command=self.ok)
        ok_bt.pack(side=tkinter.RIGHT, ipadx = 4)
        ok_bt.focus_set()

    def ok(self):
        # print(self.box['value'].index(self.box.get()))
        self.userinfo = [self.box_area_type['value'].index(self.box_area_type.get())] # 设置数据
        # print(self.userinfo)
        self.destroy() # 销毁窗口
    
    def cancel(self):
        self.userinfo = None # 空！
        self.destroy()

class RefLineAttributeWidget(tkinter.Toplevel):
    """
    参考线属性窗口类
    ToDo：配置一次后，记录配置好的数据，下次使用这些数据
    """
    def __init__(self, current_line_id, curv):
        super().__init__()
        self.title('设置参考线信息')        
        self.current_line_id = current_line_id
        self.speed_lower = tkinter.StringVar()
        self.speed_upper = tkinter.StringVar()
        self.height_limited = tkinter.StringVar()
        self.lanes_number = tkinter.StringVar()
        # self.traffic_direct = tkinter.StringVar()
        self.road_material = tkinter.StringVar()
        self.area_type = tkinter.StringVar()
        self.ratio = tkinter.StringVar()
        # self.userinfo = [self.speed_lower, self.speed_upper, self.lanes_number, self.traffic_direct, self.road_material]
        # self.label_text = ['速度下限', '速度上限', '车道数量', '坡度', '行车方向', '道路材料']
        # self.current_line_id = 0
        # self.speed_lower = 0
        # self.speed_upper = 0
        # self.lanes_number = 0
        # self.traffic_direct = 0
        # self.road_material = 0
       # 弹窗界面
        self.userinfo = None
        self.setupUI(curv)

    def setupRow(self, row, text, var):
        row.pack(fill="x", ipadx=1, ipady=1)
        tkinter.Label(row, text=text, width=12).pack(side=tkinter.LEFT)
        tkinter.Entry(row, textvariable=var, width=12).pack(side=tkinter.LEFT, padx = 6)

    def setupComboRow(self, row, text, var, value_list):
        pass
        row.pack(fill="x", ipadx=1, ipady=1)
        tkinter.Label(row, text=text, width=12).pack(side=tkinter.LEFT)
        box = ttk.Combobox(row, textvariable=var, width=12).pack(side=tkinter.LEFT, padx = 6)
        box['value'] = value_list

    def setupUI(self, curv):
        # 1.id
        # 2.限速
        # 3.车道数量
        # 4.行车方向
        # 8.材质
        # 第一行（两列）
        row1 = tkinter.Frame(self)
        row1.pack(fill="x")
        tkinter.Label(row1, text='参考线ID：', width=12).pack(side=tkinter.LEFT)
        self.name = tkinter.StringVar()
        tkinter.Label(row1, text='{}'.format(self.current_line_id), width=12).pack(side=tkinter.LEFT)        
        # 第二行
        row2 = tkinter.Frame(self)
        self.speed_lower.set(1)
        self.setupRow(row2, '车速下限：', self.speed_lower)
        tkinter.Label(row2, text='m/s', width=12).pack(side=tkinter.LEFT)        
        # 第三行
        row3 = tkinter.Frame(self)
        self.speed_upper.set(10)
        self.setupRow(row3, '车速上限：', self.speed_upper)
        tkinter.Label(row3, text='m/s', width=12).pack(side=tkinter.LEFT)        
        #
        row_height = tkinter.Frame(self)
        self.height_limited.set(20)
        self.setupRow(row_height, '高度限制：', self.height_limited)
        tkinter.Label(row_height, text='m', width=12).pack(side=tkinter.LEFT)        
        #
        row_curv = tkinter.Frame(self)
        row_curv.pack(fill="x")
        tkinter.Label(row_curv, text='曲率(1/m)：', width=12).pack(side=tkinter.LEFT)
        tkinter.Label(row_curv, text='{:.2f}'.format(curv), width=12).pack(side=tkinter.LEFT)        
        # 第四行
        row4 = tkinter.Frame(self)
        self.lanes_number.set(1)
        self.setupRow(row4, '车道数量：', self.lanes_number)        
        # 第四行
        row_ratio = tkinter.Frame(self)
        self.ratio.set(0)
        self.setupRow(row_ratio, '道路坡度：', self.ratio)        
        # 第五行
        row5 = tkinter.Frame(self)
        row5.pack(fill="x", ipadx=1, ipady=1)
        tkinter.Label(row5, text='行车方向', width=12).pack(side=tkinter.LEFT)
        # checkbutton0
        self.straight_value = tkinter.IntVar()
        straight_ck = tkinter.Checkbutton(row5, text='直行', variable=self.straight_value)
        straight_ck.select()
        straight_ck.pack(side=tkinter.LEFT)
        # checkbutton1
        self.left_value = tkinter.IntVar()
        left_ck = tkinter.Checkbutton(row5, text='左转', variable=self.left_value)
        left_ck.pack(side=tkinter.LEFT)
        # checkbutton2
        self.right_value = tkinter.IntVar()
        right_ck = tkinter.Checkbutton(row5, text='右转', variable=self.right_value)
        right_ck.pack(side=tkinter.LEFT)
        # checkbutton3
        self.bidirect_value = tkinter.IntVar()
        bidirect_ck = tkinter.Checkbutton(row5, text='双向', variable=self.bidirect_value)
        bidirect_ck.pack(side=tkinter.LEFT)
        # 第六行
        row6 = tkinter.Frame(self)
        row6.pack(fill="x", ipadx=1, ipady=1)
        tkinter.Label(row6, text='道路材料', width=12).pack(side=tkinter.LEFT)
        box = ttk.Combobox(row6, textvariable=self.road_material, width=12)
        box.pack(side=tkinter.LEFT, padx = 6)
        box['value'] = ['未知', '沥青', '水泥']
        box.current(1)
        self.box = box
        # 第七行
        row_area_type = tkinter.Frame(self)
        row_area_type.pack(fill="x", ipadx=1, ipady=1)
        tkinter.Label(row_area_type, text='区域属性', width=12).pack(side=tkinter.LEFT)
        box_area_type = ttk.Combobox(row_area_type, textvariable=self.area_type, width=12)
        box_area_type.pack(side=tkinter.LEFT, padx = 6)
        box_area_type['value'] = ROAD_AREA_VALUE
        box_area_type.current(1)
        self.box_area_type = box_area_type
        # 第n行
        row7 = tkinter.Frame(self)
        row7.pack(fill="x")
        tkinter.Button(row7, text="取消", command=self.cancel).pack(side=tkinter.RIGHT, padx = 6, ipadx = 4)
        ok_bt = tkinter.Button(row7, text="确定", command=self.ok)
        ok_bt.pack(side=tkinter.RIGHT, ipadx = 4)
        ok_bt.focus_set()
        pass
    
    def ok(self):
        # print(self.box['value'].index(self.box.get()))
        traffic_direct_value = [self.straight_value.get(), self.left_value.get(), self.right_value.get(), self.bidirect_value.get()]
        self.userinfo = [self.speed_lower.get(), self.speed_upper.get(), self.height_limited.get(), self.lanes_number.get(), self.ratio.get(), traffic_direct_value, self.box['value'].index(self.box.get()), self.box_area_type['value'].index(self.box_area_type.get())] # 设置数据
        self.destroy() # 销毁窗口
    
    def cancel(self):
        self.userinfo = None # 空！
        self.destroy()

class SelectLineTypeWidget(tkinter.Toplevel):
    """
    将一条线设置为参考线/边界/附属物边线 
    暂时无用
    """
    def __init__(self):
        super().__init__()        
        self.setupUI()
        self.userinfo = None


    def setupUI(self):
        self.line_type = tkinter.IntVar()
        self.line_type.set(0)
        row_select_type = tkinter.Frame(self)
        row_select_type.pack(fill="x")
        tkinter.Radiobutton(row_select_type, lable='设为参考线', variable = self.line_type, value = 0)
        tkinter.Radiobutton(row_select_type, label = '设为附属物', variable = self.line_type, value = 1)
        # 第n行
        row7 = tkinter.Frame(self)
        row7.pack(fill="x")
        tkinter.Button(row7, text="取消", command=self.cancel).pack(side=tkinter.RIGHT, padx = 6, ipadx = 4)
        ok_bt = tkinter.Button(row7, text="确定", command=self.ok)
        ok_bt.pack(side=tkinter.RIGHT, ipadx = 4)
        ok_bt.focus_set()
        pass
    
    def ok(self):
        # print(self.box['value'].index(self.box.get()))
        self.userinfo = [self.line_type.get()]
        self.destroy() # 销毁窗口
    
    def cancel(self):
        self.userinfo = None # 空！
        self.destroy()

class RefLinePointsWidget(tkinter.Toplevel):
    """
    生成车道参考点
    """
    def __init__(self, current_line_id, speed_lower, speed_upper, lanes_number, ratio):
        super().__init__()
        self.title('生成车道参考线离散点')
        self.setupUI(current_line_id, speed_lower, speed_upper, lanes_number, ratio)
        self.userinfo = None

    def setupRow(self, row, text0, width0, text1, width1):
        row.pack(fill="x", ipadx=1, ipady=1)
        tkinter.Label(row, text=text0, width=width0).pack(side=tkinter.LEFT)
        tkinter.Label(row, text=text1, width=width1).pack(side=tkinter.LEFT, padx=2)

    def setupUI(self, current_line_id, speed_lower, speed_upper, lanes_number, ratio):
        # 1.id
        # 2.限速
        # 3.车道数量
        # 4.行车方向
        # 8.材质
        # 第一行（两列）
        row1 = tkinter.Frame(self)
        row1.pack(fill="x")
        tkinter.Label(row1, text='参考线ID：', width=12).pack(side=tkinter.LEFT)
        self.name = tkinter.StringVar()
        tkinter.Label(row1, text='{}'.format(current_line_id), width=4).pack(side=tkinter.LEFT)        
        # 第二行
        row2 = tkinter.Frame(self)
        self.setupRow(row2, '车速限制：', 12, '{} - '.format(speed_lower), 2)
        self.setupRow(row2, '{}'.format(speed_upper), 2, 'm/s', 2)       
        # 第四行
        row4 = tkinter.Frame(self)
        self.setupRow(row4, '车道数量：', 12, '{}'.format(lanes_number), 2)        
        # 第四行
        row_ratio = tkinter.Frame(self)
        self.setupRow(row_ratio, '道路坡度：', 12, '{}'.format(ratio), 2)        
        # 第n行
        row7 = tkinter.Frame(self)
        row7.pack(fill="x")
        tkinter.Button(row7, text="取消", command=self.cancel).pack(side=tkinter.RIGHT, padx = 6, ipadx = 4)
        ok_bt = tkinter.Button(row7, text="确定", command=self.ok)
        ok_bt.pack(side=tkinter.RIGHT, ipadx = 4)
        ok_bt.focus_set()
        pass
    
    def ok(self):
        # print(self.box['value'].index(self.box.get()))
        self.userinfo = 1 
        self.destroy() # 销毁窗口
    
    def cancel(self):
        self.userinfo = None # 空！
        self.destroy()

class BoardAttributeWidget(tkinter.Toplevel):
    """
    车道边界线属性窗口类
    """
    def __init__(self, lanes_number, current_lanes, total_boards):
        super().__init__()
        self.title('组合车道边界线')
        self.lanes_number = lanes_number
        self.current_lanes = current_lanes
        self.total_boards = total_boards        
        # 弹窗界面
        self.setupUI()
        self.userinfo = None

    def setupUI(self):
        row1 = tkinter.Frame(self)
        row1.pack(fill="x")
        tkinter.Label(row1, text='车道数量：', width=16).pack(side=tkinter.LEFT)
        self.name = tkinter.StringVar()
        tkinter.Label(row1, text='{}'.format(self.lanes_number), width=12).pack(side=tkinter.LEFT)        

        row2 = tkinter.Frame(self)
        row2.pack(fill="x")
        tkinter.Label(row2, text='已配置边界数：', width=16).pack(side=tkinter.LEFT)
        self.name = tkinter.StringVar()
        tkinter.Label(row2, text='{}'.format(self.current_lanes), width=12).pack(side=tkinter.LEFT)        

        row3 = tkinter.Frame(self)
        row3.pack(fill="x")
        tkinter.Label(row3, text='已选择边线数：', width=16).pack(side=tkinter.LEFT)
        self.name = tkinter.StringVar()
        tkinter.Label(row3, text='{}'.format(self.total_boards), width=12).pack(side=tkinter.LEFT)        

        # 第n行
        row7 = tkinter.Frame(self)
        row7.pack(fill="x")
        tkinter.Button(row7, text="取消", command=self.cancel).pack(side=tkinter.RIGHT, padx = 6, ipadx = 4)
        ok_bt = tkinter.Button(row7, text="确定", command=self.ok)
        ok_bt.pack(side=tkinter.RIGHT, ipadx = 4)
        ok_bt.focus_set()
        pass
    
    def ok(self):
        # print(self.box['value'].index(self.box.get()))
        self.userinfo = 1 
        self.destroy() # 销毁窗口
    
    def cancel(self):
        self.userinfo = None # 空！
        self.destroy()

class ShowDataWidget(tkinter.Toplevel):
    def __init__(self, saved_line=0, bidirect_line=0, saved_length=0, total_line=0, saved_appendix=0):
        super().__init__()
        self.title('数据统计')
        # 弹窗界面
        self.setupUI(saved_line, bidirect_line, saved_length, total_line, saved_appendix)



    def setupUI(self, saved_line, bidirect_line, saved_length, total_line, saved_appendix):
        row1 = tkinter.Frame(self)
        row1.pack(fill="x")
        tkinter.Label(row1, text='已完成参考线数量：', width=16).pack(side=tkinter.LEFT)
        tkinter.Label(row1, text='{}'.format(saved_line), width=12, bg='green').pack(side=tkinter.LEFT)        

        row2 = tkinter.Frame(self)
        row2.pack(fill="x")
        tkinter.Label(row2, text='双向车道数量：', width=16).pack(side=tkinter.LEFT)
        tkinter.Label(row2, text='{}'.format(bidirect_line), width=12).pack(side=tkinter.LEFT)        

        row2 = tkinter.Frame(self)
        row2.pack(fill="x")
        tkinter.Label(row2, text='已完成参考线总长：', width=16).pack(side=tkinter.LEFT)
        tkinter.Label(row2, text='{:.2f}'.format(saved_length), width=12).pack(side=tkinter.LEFT)        

        row_nosaved = tkinter.Frame(self)
        row_nosaved.pack(fill="x")
        tkinter.Label(row_nosaved, text='未完成参考线：', width=16).pack(side=tkinter.LEFT)
        tkinter.Label(row_nosaved, text='{}'.format(total_line-saved_line), width=12).pack(side=tkinter.LEFT)        

        row3 = tkinter.Frame(self)
        row3.pack(fill="x")
        tkinter.Label(row3, text='参考线总数：', width=16).pack(side=tkinter.LEFT)
        tkinter.Label(row3, text='{}'.format(total_line), width=12).pack(side=tkinter.LEFT)        

        row4 = tkinter.Frame(self)
        row4.pack(fill="x")
        tkinter.Label(row4, text='已完成附属物：', width=16).pack(side=tkinter.LEFT)
        tkinter.Label(row4, text='{}'.format(saved_appendix), width=12).pack(side=tkinter.LEFT)        

        # 第n行
        row_button = tkinter.Frame(self)
        row_button.pack(fill="x")
        ok_bt = tkinter.Button(row_button, text="确定", command=self.ok)
        ok_bt.pack(side=tkinter.RIGHT, ipadx = 4)
        ok_bt.focus_set()
        pass
    
    def ok(self):
        self.destroy() # 销毁窗口

class checkConnectWidget(tkinter.Toplevel):
    def __init__(self, processor):
        super().__init__()
        self.current_line2d = None
        self.next_line2d_list = []
        self.title('连通图检查')
        # 弹窗界面
        self.processor = processor
        self.saved_line_list = sorted(list(self.processor.connect_dict.keys())) # 
        self.saved_line_index = 0
        self.unsaved_line_list = []
        self.unsaved_line_index = 0
        for key in self.processor.dxfentity_dict.keys():
            if key not in self.processor.connect_dict.keys():
                self.unsaved_line_list.append(key)
        self.setupUI()

    def showConnect(self, event):
        if self.show_saved.get() == 1:
            if len(self.saved_line_list) != 0:
                self.saved_line_index = self.box_current_line['value'].index(self.box_current_line.get()) # 获取到了值，值是connect_dict的key
                print(self.processor.connect_dict[self.saved_line_list[self.saved_line_index]])
                self.lable_next_lines["text"] = '{}'.format(self.processor.connect_dict[self.saved_line_list[self.saved_line_index]])
        else:
            if len(self.unsaved_line_list) != 0:
                self.unsaved_line_index = self.box_current_line['value'].index(self.box_current_line.get())
                self.lable_next_lines["text"] = 'Unknown'
            # else:
            #     pass
        self.setLinesColor()
        return

    def setLinesColor(self):
        """
        设置line2D颜色并显示
        """
        if self.current_line2d != None:
            self.current_line2d.set_color('k')
        self.current_line2d = None

        if len(self.next_line2d_list) != 0:
            for line in self.next_line2d_list:
                line.set_color('k')
        self.next_line2d_list = []

        if self.show_saved.get() == 1 and len(self.saved_line_list)>0:
            saved_line_ID = self.saved_line_list[self.saved_line_index]
            self.current_line2d = self.processor.ref_line2D_dict[saved_line_ID]
            self.current_line2d.set_color('g')
            next_line_list = self.processor.connect_dict[saved_line_ID]
            if len(next_line_list) > 0:
                for id in next_line_list:
                    self.next_line2d_list.append(self.processor.ref_line2D_dict[id])
                    self.processor.ref_line2D_dict[id].set_color('r')

        elif self.show_saved.get() != 1 and len(self.unsaved_line_list)>0:
            unsaved_line_ID = self.unsaved_line_list[self.unsaved_line_index]
            self.current_line2d = self.processor.ref_line2D_dict[unsaved_line_ID]
            self.current_line2d.set_color('g')

        self.processor.figure.canvas.draw()
        return

    def rpick(self):
        if self.show_saved.get() == 1:
            self.box_current_line['value'] = self.saved_line_list
            if len(self.saved_line_list) != 0:
                self.saved_line_index = 0
                self.box_current_line.current(self.saved_line_index)
                self.lable_next_lines['text'] = '{}'.format(self.processor.connect_dict[self.saved_line_list[self.saved_line_index]])
                self.box_current_line['state'] = 'enabled'
            else:
                self.box_current_line.set('None')
                self.box_current_line['state'] = 'disabled'
                self.lable_next_lines['text'] = 'Unknown'             
        else:
            self.box_current_line['value'] = self.unsaved_line_list
            if len(self.unsaved_line_list) != 0:
                self.box_current_line.current(0)
                self.lable_next_lines['text'] = 'Unknown'
                self.box_current_line['state'] = 'enabled'
            else:
                self.box_current_line.set('None')
                self.box_current_line['state'] = 'disabled'
                self.lable_next_lines['text'] = 'Unknown'
        self.setLinesColor()
        return

    def getValue(self):
        """
        返回self.box_current_line中value在整个value_list中的index和具体的value
        value实际是dict的key值
        """
        key = 0
        key = self.box_current_line.get() # 获取current_line中的value，也就是dict中的key
        index = self.box_current_line['value'].index(key)
        return index, int(key)

    def setupUI(self):
        row_saved_flag = tkinter.Frame(self)
        row_saved_flag.pack(fill="x", ipadx=1, ipady=1, pady=4)
        self.show_saved = tkinter.IntVar()
        self.show_saved.set(1)
        self.rbt_saved = tkinter.Radiobutton(row_saved_flag, text = '未完成车道', variable=self.show_saved, value = 0, command=self.rpick)
        self.rbt_saved.pack(side=tkinter.LEFT)
        rbt_unsaved = tkinter.Radiobutton(row_saved_flag, text = '已完成车道', variable=self.show_saved, value = 1, command=self.rpick)
        rbt_unsaved.pack(side=tkinter.RIGHT)

        row_current_line = tkinter.Frame(self)
        row_current_line.pack(fill="x", ipadx=1, ipady=1, pady=4)
        tkinter.Label(row_current_line, text='当前车道序号:', width=16).pack(side=tkinter.LEFT)
        self.box_current_line = ttk.Combobox(row_current_line, width=12)
        self.box_current_line.pack(side=tkinter.LEFT, padx = 6)
        self.box_current_line['value'] = self.saved_line_list # 默认情况为saved_line_list，key的列表
        self.box_current_line.current(self.saved_line_index)
        self.box_current_line.bind('<<ComboboxSelected>>', self.showConnect) # 下拉列表选中事件

        row_next_lines = tkinter.Frame(self)
        row_next_lines.pack(fill="x", pady=4)
        tkinter.Label(row_next_lines, text='后续车道序号:', width=16).pack(side=tkinter.LEFT)
        self.lable_next_lines = tkinter.Label(row_next_lines, text='{}'.format(self.processor.connect_dict[self.saved_line_list[self.saved_line_index]]), width=12)
        self.lable_next_lines.pack(side=tkinter.LEFT)        

        # 第n行
        row_button = tkinter.Frame(self)
        row_button.pack(fill="x", pady=4)
        pre_bt = tkinter.Button(row_button, text="<<前一条车道", command=self.pre_line)
        pre_bt.pack(side=tkinter.LEFT, ipadx = 4, padx = 4)

        next_bt = tkinter.Button(row_button, text="后一条车道>>", command=self.next_line)
        next_bt.pack(side=tkinter.RIGHT, ipadx = 4, padx = 4)
        next_bt.focus_set()
        pass
    
    def pre_line(self):
        if self.show_saved.get() == 1:
            if self.saved_line_index > 0  and len(self.saved_line_list) > 0:
                self.saved_line_index -= 1
                self.box_current_line.current(self.saved_line_index)
                self.lable_next_lines['text'] = '{}'.format(self.processor.connect_dict[self.saved_line_list[self.saved_line_index]])
        else:
            if self.unsaved_line_index > 0 and len(self.unsaved_line_list) > 0:
                self.unsaved_line_index -= 1
                self.box_current_line.current(self.unsaved_line_index)
        self.setLinesColor()

    def next_line(self):
        if self.show_saved.get() == 1:
            if self.saved_line_index < (len(self.saved_line_list)-1)  and len(self.saved_line_list) > 0:
                self.saved_line_index += 1
                self.box_current_line.current(self.saved_line_index)
                self.lable_next_lines['text'] = '{}'.format(self.processor.connect_dict[self.saved_line_list[self.saved_line_index]])
        else:
            if self.unsaved_line_index < (len(self.unsaved_line_list)-1) and len(self.unsaved_line_list) > 0:
                self.unsaved_line_index += 1
                self.box_current_line.current(self.unsaved_line_index)
        self.setLinesColor()

class MapToolsApp:
    """
    建立一个窗口，显示plot
    """
    def __init__(self, master, data_processor):
        self.processor = data_processor
        self.master = master
        self.initMenu()
        self.figure = Figure()
        self.initFigureFrame()
        self.master.bind('<Button-3>',self.popup)
        # self.initPopupMenu()
        pass

    # 右键事件响应
    def popup(self, event):
        # self.popup_menu.post(event.x_root, event.y_root)
        if self.processor.schedule == DataProcSchedule.EPS_PICKED:
            self.configRefLine()
        elif self.processor.schedule == DataProcSchedule.REF_CONFIGED:
            self.configBoard()
        elif self.processor.schedule == DataProcSchedule.BOARD_PICKED:
            self.makeRefLinePoints()
        elif self.processor.appendix_schedule == AppendixSchedule.START and not self.processor.handle_ref_flag:
            self.configAppendix()
        elif self.processor.appendix_schedule == AppendixSchedule.CONFIG and not self.processor.handle_ref_flag:
            # print('pop run save')
            self.saveAppendix()
        # elif self.processor.schedule == DataProcSchedule.REF_PICKED:
        #     self.selectLineType()
            pass
        print('right bt clicked')

    def selectLineType(self):
        info = self.getConfigure('select_line_type')
        if info is None:
            return
        else:
            # todo 完全修改模式
            pass

    # menu菜单初始化
    def initMenu(self):
        # 创建位于self.master中的menubar
        menubar = tkinter.Menu(self.master)
        # 将menubar作为master的'menu'属性
        self.master['menu'] = menubar 

        # 创建file_menu一级菜单，位于menubar
        file_menu = tkinter.Menu(menubar, tearoff = 0)
        # 利用add_cascade将file_manu添加到menubar界面
        menubar.add_cascade(label = '文件', menu = file_menu)
        # 利用add_command为file_menu添加菜单
        file_menu.add_command(label='选择路径', command=self.openFold)
        file_menu.add_separator()
        file_menu.add_command(label='生成连通图', command=self.makeConnectMap)


        # 创建config_attribute一级菜单
        config_attribute_menu = tkinter.Menu(menubar, tearoff = 0)
        menubar.add_cascade(label = '设定', menu = config_attribute_menu)
        # 添加下拉菜单
        self.ref_line_flag = tkinter.IntVar()
        self.ref_line_flag.set(0)
        config_attribute_menu.add_radiobutton(label = '处理参考线', variable = self.ref_line_flag, command=self.radioBtHandler, value = 0)
        config_attribute_menu.add_radiobutton(label = '处理附属物', variable = self.ref_line_flag, command=self.radioBtHandler, value = 1)
        # config_attribute_menu.add_separator()

        # 创建校验
        check_menu = tkinter.Menu(menubar, tearoff = 0)
        menubar.add_cascade(label = '校验', menu = check_menu)
        # 利用add_command为file_menu添加菜单
        check_menu.add_command(label='统计数据', command=self.showData)
        check_menu.add_separator()
        check_menu.add_command(label='检查连通图', command=self.checkConnectMap)


        # 创建关于一级菜单
        about_menu = tkinter.Menu(menubar, tearoff = 0)
        menubar.add_cascade(label = '关于', menu = about_menu)
        # 添加单选按键，是否开启提示
        self.tips_enable = tkinter.IntVar()
        self.tips_enable.set(0) # 默认关闭
        about_menu.add_checkbutton(label = '开启提示', variable = self.tips_enable, command = self.setTipsEnable)

    def showData(self): # todo
        # 已经配置的参考线/参考线总数
        # 已经配置的参考线长度/总长
        # 已经配置的附属物数量
        # 
        ShowDataWidget(self.processor.saved_road_cnt, self.processor.bidirection_road_cnt, self.processor.saved_road_length, self.processor.total_road_cnt)
        return

    def checkConnectMap(self): # todo
        # 显示当前车道ID，相邻的车道ID
        # 允许手动输入ID
        # 允许显示未连接ID
        checkConnectWidget(self.processor)
        return

    def radioBtHandler(self):
        self.processor.handle_ref_flag = (self.ref_line_flag.get() == 0)
        if self.processor.handle_ref_flag:
            self.processor.setRefLinePicker(3)
            self.processor.setAppendixLinePicker(0)
        else:
            self.processor.setRefLinePicker(0)
            self.processor.setAppendixLinePicker(3)
        # print(self.processor.handle_ref_flag)

    def setTipsEnable(self):
        self.processor.toggleTipsEnable()
        pass

    def scatterRefLine(self):
        if self.processor.getRefLinePoints(): # 参考线点生成完成
            # 开始保存数据                
            #
            self.processor.saveCurrentRefLineData()
            messagebox.showinfo(title='成功', message='参考线生成完成')
            self.processor.resetRefLineDisplay('k')
        else:
            messagebox.showerror(title='失败', message='请确认道路边界是否正确')
            self.processor.resetRefLineDisplay('g')
        self.processor.resetBoardLinesDisplay()
        self.processor.initALineVarials()
        self.processor.setBoardLinePicker(0)
        if self.processor.handle_ref_flag:
            self.processor.setRefLinePicker(3)
            self.processor.setAppendixLinePicker(0)
        else:
            self.processor.setRefLinePicker(0)
            self.processor.setAppendixLinePicker(3)
        self.canvas.draw()
        pass

    def configRefLine(self):
        if not self.processor.handle_ref_flag:
            messagebox.showerror(title='Error Occur', message='请确认处于参考线处理模式！')
            return None
        if self.processor.schedule == DataProcSchedule.START:
            messagebox.showerror(title='Error Occur', message='请先选择一条参考线！')
            return None
        elif not self.processor.schedule == DataProcSchedule.EPS_PICKED:
            messagebox.showerror(title='Error Occur', message='请先选择参考线起点！')
            return None
        else:
            info = self.getConfigure('ref_line')
            if info is None:
                return
            else:
                self.processor.saveRefLineBaseAttribute(info)
                self.processor.schedule = DataProcSchedule.REF_CONFIGED
                self.processor.removeLineEPs(self.figure)
                self.processor.setBoardLinePicker(3)
                pass

    def getConfigure(self, widget):
        if widget == 'ref_line':
            dxf_entity = self.processor.dxfentity_dict[self.processor.current_line_id]
            if dxf_entity.dxftype == 'LINE':
                curv = 0.0
            elif dxf_entity.dxftype == 'ARC':
                curv = 1/dxf_entity.radius
            inputDialog = RefLineAttributeWidget(self.processor.current_line_id, curv)
        elif widget == 'board':
            inputDialog = BoardAttributeWidget(self.processor.lanes_number, self.processor.current_boards_number, self.processor.current_board_line_cnt)
        elif widget == 'appendix':
            inputDialog = AppendixAttributeWidget(self.processor.current_appendix_id)
        elif widget == 'save_appendix':
            inputDialog = AppendixSaveWidget(self.processor.current_appendix_id, self.processor.current_appendix_to_id)
        elif widget == 'make_ref_points':
            inputDialog = RefLinePointsWidget(self.processor.current_line_id, self.processor.speed_lower, self.processor.speed_upper, self.processor.lanes_number, self.processor.ratio)
        elif widget == 'select_line_type':
            inputDialog = SelectLineTypeWidget()
        self.master.wait_window(inputDialog) # 这一句很重要！！！        
        return inputDialog.userinfo
        
    def makeRefLinePoints(self):
        info = self.getConfigure('make_ref_points')
        if info is None:
            return
        else:
            self.scatterRefLine()
        pass

    def configBoard(self):
        if self.processor.current_board_line_cnt == 0: 
            messagebox.showerror(title='Error Occur', message='请先选择足够的道路边界！')
            return
        elif not self.processor.checkBoardLines():
            messagebox.showerror(title='Error Occur', message='每次选择一个道路边界')
            return
        else: # 每组合一次，就将boards数量加1
            info = self.getConfigure('board')
            if info is None:
                return
            else:
                # 调用processor中的方法
                board_data = []
                # 合成一个data
                for artist in self.processor.current_board_lines_list:
                    board_data.extend(artist.get_xydata())
                # 加入board list
                self.processor.current_boards_list.append(board_data)
                self.processor.current_boards_tree_list.append(spatial.KDTree(board_data))
                self.processor.current_boards_number += 1
                if self.processor.current_boards_number ==  self.processor.lanes_number + 1: # 车道边界是车道数量+1       
                    self.processor.schedule = DataProcSchedule.BOARD_PICKED
                # 清空本条车道边界
                self.processor.current_board_lines_list = []
                self.processor.current_board_line_cnt = 0
                self.canvas.draw()
                return

    def configAppendix(self):
        if self.processor.current_appendix_id == 0: 
            messagebox.showerror(title='Error Occur', message='请先选择附属物！')
            return None
        else:
            info = self.getConfigure('appendix')
            if info is None:
                return
            else:
                self.processor.appendix_dict[self.processor.current_appendix_id].type_ = info[0]
                self.processor.setAppendixLinePicker(0)
                self.processor.setRefLinePicker(3)
                self.processor.appendix_schedule = AppendixSchedule.GET_BELONG
                
    def saveAppendix(self):
        info = self.getConfigure('save_appendix')
        if info is None:
            return
        else:
            # print('save run here')
            self.processor.saveCurrentAppendixData()
            self.processor.resetAppendixLineListColor()
            self.processor.initAppendixVarials()
            self.processor.setBoardLinePicker(0)
            if self.processor.handle_ref_flag:
                self.processor.setRefLinePicker(3)
                self.processor.setAppendixLinePicker(0)
            else:
                self.processor.setRefLinePicker(0)
                self.processor.setAppendixLinePicker(3)        
            self.processor.appendix_schedule = AppendixSchedule.START
            self.canvas.draw()
        return

    # 保存现有数据
    def makeConnectMap(self):
        # todo now
        self.processor.makeConnectMap()
        messagebox.showinfo(title='完成', message='连通图保存成功')
        return

    # 打开文件夹事件
    def openFold(self):
        file_fold_ = filedialog.askdirectory()
        if file_fold_ != '':
            self.processor.__init__()
            self.figure.clf()
            self.ax = self.figure.add_subplot(111)
            self.ax.margins(0)
            self.ax.grid(True)
            totla_lines = self.processor.loadData(file_fold_, self.figure, ['ref_line'], ['board_white', 'board_yellow'])
            self.processor.drawLines(self.ax, totla_lines)
            self.processor.setBoardLinePicker(0)
            if self.processor.handle_ref_flag:
                self.processor.setRefLinePicker(3)
                self.processor.setAppendixLinePicker(0)
            else:
                self.processor.setRefLinePicker(0)
                self.processor.setAppendixLinePicker(3)
            self.canvas.draw()
        pass

    # matplot显示窗口
    def initFigureFrame(self):
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.master)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(
            side=tkinter.TOP, fill=tkinter.BOTH, expand=1)
        self.processor.callbackConnect(self.canvas)


if __name__ == "__main__":
    root = tkinter.Tk()
    root.title('HD Map Tools')
    data_processor = DataProcess()
    MapToolsApp(root, data_processor)
    root.mainloop()

    