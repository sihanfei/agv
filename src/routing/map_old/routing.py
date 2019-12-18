import csv as csv
import networkx as nx
import matplotlib.pyplot as plt
import PIL.Image as PIImg
import numpy as np

import socket

from enum import Enum

from socketserver import BaseRequestHandler, ThreadingTCPServer
import threading

import time


def get_host_ip():
    """
    利用UDP封包提取ip
    查询本机ip地址
    :return: ip
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip


BUF_SIZE = 1024
DiGraphic = []


def doPathSearch(data, DiGraphic):
    str_data = data.decode()
    num = 0
    num_list = []
    for s in str_data:
        if s == ',':
            print('num is {}'.format(num))
            num_list.append(num)
            num = 0
        else:
            num = int(s) + num * 10

    carID = num_list[0]
    src_ID = num_list[1]
    end_ID = num

    if not src_ID in DiGraphic.nodes:
        status = str(0xAA) + str(0xAA) + ',' + str(carID) + ',' + str(
            11) + ',' + str(0xCC) + str(0xCC)
        return status

    if not end_ID in DiGraphic.nodes:
        status = str(0xAA) + str(0xAA) + ',' + str(carID) + ',' + str(
            12) + ',' + str(0xCC) + str(0xCC)
        return status

    if src_ID == end_ID:
        status = str(0xAA) + str(0xAA) + ',' + str(carID) + ',' + str(
            1) + ',' + str(2) + ',' + str(src_ID) + ',' + str(
                end_ID) + ',' + str(0xCC) + str(0xCC)
        return status

    try:
        length = nx.shortest_path_length(
            DiGraphic, src_ID, end_ID, weight='length')
        path = nx.shortest_path(DiGraphic, src_ID, end_ID, weight='length')
        print('path:{}, length:{}'.format(path, length))
        status = str(0xAA) + str(0xAA) + ',' + str(carID) + ',' + str(
            1) + ',' + str(len(path))
        for IDn in path:
            status = status + ',' + str(IDn)
        status = status + ',' + str(0xCC) + str(0xCC)
    except nx.NetworkXNoPath:
        status = str(0xAA) + str(0xAA) + str(carID) + ',' + str(
            13) + ',' + str(0xCC) + str(0xCC)
    return status


class PathErrorCode(Enum):
    Success = 1
    NoStartID = 11
    NoEndID = 12
    NoPath = 13


class Handler(BaseRequestHandler):
    def handle(self):
        while True:
            data = self.request.recv(BUF_SIZE)
            if len(data) > 0:
                print('receive={}'.format(data))
                # cur_thread = threading.current_thread()
                # response = '{}:{}'.format(cur_thread.ident, data)
                status = doPathSearch(data, DiGraphic)
                self.request.sendall(status.encode())
                t = time.time()
                while time.time() - t < 1:
                    pass
                self.request.sendall(status.encode())
                print('send:{}'.format(status))
            else:
                # self.request.close()
                print('close')
                break


def getConnectMapFromJson(filename):
    connect_map_dict = {}
    try:
        fp = open(filename, 'r')
        connect_map = csv.reader(fp)
        for item in connect_map:
            value = []
            for i in range(2, len(item)):
                value.append(int(item[i]))
            connect_map_dict[int(item[0])] = value
    except FileExistsError:
        return False
    finally:
        fp.close()
    return connect_map_dict


def getRefLinePointMapFromJson(filename):
    try:
        fp = open(filename, 'r')
        ref_line_map = csv.reader(fp)
        ref_line_point_map_dict = {}
        ref_line_ID = 0
        # ref_line_speed_limited = []
        for item in ref_line_map:  # 每一行就是一个点，需要通过每行前的id号判断是否是同一条参考线
            point = tuple((float(item[1]), float(item[2])))
            if ref_line_ID == 0:  # 第一个数据
                ref_line_ID = int(item[0])
                point_list = []
                point_list.append(point)
                ref_line_point_map_dict[ref_line_ID] = point_list
            else:
                if ref_line_ID != int(item[0]):  # 新的ref line
                    ref_line_ID = int(item[0])
                    point_list = []
                    point_list.append(point)
                    ref_line_point_map_dict[ref_line_ID] = point_list
                else:
                    point_list.append(point)
                    ref_line_point_map_dict[ref_line_ID] = point_list

            point = tuple((float(item[1]), float(item[2])))
            pass
    except FileExistsError:
        return False
    finally:
        fp.close()
    return ref_line_point_map_dict


def getRefLineLengthMapFromJson(filename):
    try:
        fp = open(filename, 'r')
        ref_line_map = csv.reader(fp)
        ref_line_length_map_dict = {}
        ref_line_ID = []
        ref_line_length = []
        # ref_line_speed_limited = []
        for item in ref_line_map:  # 每一行就是一个点，需要通过每行前的id号判断是否是同一条参考线
            ref_line_ID = int(item[0])
            ref_line_length = float(item[3])
            ref_line_length_map_dict[
                ref_line_ID] = ref_line_length  # 利用了两个特性：字典的key唯一，ref_line的长度是递增的
    except FileExistsError:
        return False
    finally:
        fp.close()
    return ref_line_length_map_dict


class routeMap:
    def __init__(self):
        return

    def buildDirectGraphic(self, connect_map_dict, ref_line_point_map_dict):
        DG = nx.DiGraph()
        for _, key in enumerate(connect_map_dict):
            value = connect_map_dict[key]
            for nextID in value:
                print('main: for nextID:{}:{}'.format(key, nextID))
                # 需要走过的是车辆所在的ID，所以用所在车道的长度来度量经过的路程
                DG.add_weighted_edges_from(
                    [(key, nextID, ref_line_length_map_dict[key])],
                    weight='length')
        return DG

    def drawRouteMap(self, direct_graphic, connect_map_dict,
                     ref_line_point_map_dict):
        pos_dict = {}
        label_dict = {}
        for _, key in ref_line_point_map_dict:
            pos_dict[key] = ref_line_point_map_dict[key][0]
            label_dict[key] = str(key)
        nx.draw_networkx_labels(direct_graphic, pos_dict, labels=label_dict)

    def getrouteMapIDList(self, direct_graphic):
        ID_list = 0
        direct_graphic
        return ID_list

    def getPath(self, direct_graphic, start_id, end_id):
        error_code = PathErrorCode()
        try:
            length = nx.shortest_path_length(
                direct_graphic, start_id, start_id, weight='length')
            path = nx.shortest_path(
                direct_graphic, start_id, start_id, weight='length')
        except nx.NetworkXNoPath:
            return error_code.NoPath
        return length, path


if __name__ == "__main__":
    file_dir = "./map/zhenjiang"
    connect_map_file = file_dir + "/connect_map.json"
    connect_map_dict = getConnectMapFromJson(connect_map_file)
    for _, key in enumerate(connect_map_dict):
        print('main:{}:{}'.format(key, connect_map_dict[key]))

    ref_line_map_file = file_dir + "/ref_line_map.json"
    ref_line_length_map_dict = getRefLineLengthMapFromJson(ref_line_map_file)
    for _, key in enumerate(ref_line_length_map_dict):
        print('main:ref_line->{}:{}'.format(key,
                                            ref_line_length_map_dict[key]))

    # 通过networkx画连通图
    DiGraphic = nx.DiGraph()
    for _, key in enumerate(connect_map_dict):
        value = connect_map_dict[key]
        for nextID in value:
            print('main: for nextID:{}:{}'.format(key, nextID))
            # 需要走过的是车辆所在的ID，所以用所在车道的长度来度量经过的路程
            DiGraphic.add_weighted_edges_from(
                [(key, nextID, ref_line_length_map_dict[key])],
                weight='length')

    # img1 = PIImg.open("../data/map/zhenjiang/zhenjiang.bmp")
    # npimg1 = np.array(img1)
    # npimg1 = npimg1[-1:0:-1, :, :]
    # # 准备绘图
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.imshow(npimg1, origin='lower')
    # ax.autoscale(False)

    HOST = get_host_ip()
    print(HOST)
    PORT = 9001
    ADDR = (HOST, PORT)
    server = ThreadingTCPServer(ADDR, Handler)  #参数为监听地址和已建立连接的处理类
    print('listening')
    server.serve_forever()  #监听，建立好TCP连接后，为该连接创建新的socket和线程，并由处理类中的handle方法处理

    # # show map
    # ref_line_point_map_dict = getRefLinePointMapFromJson(
    #     ref_line_map_file)
    # x = []
    # y = []
    # for id in path:
    #     points = ref_line_point_map_dict[id]
    #     for p in points:
    #         x.append(p[0])
    #         y.append(p[1])
    # plt.plot(x, y, 'r.')
    # # plt.plot()
    # plt.show()
    # break
