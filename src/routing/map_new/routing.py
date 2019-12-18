import networkx as nx
import matplotlib.pyplot as plt
import numpy as np

import socket

from enum import Enum

from socketserver import BaseRequestHandler, ThreadingTCPServer
import threading

import time

import map_element

from tkinter import filedialog


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


def get_local_ip(ifname='eth0'):
    import socket
    import fcntl
    import struct
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    inet = fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', ifname[:15]))
    ret = socket.inet_ntoa(inet[20:24])
    return ret


BUF_SIZE = 1024
DiGraphic = []
connect_map_dict = {}
ref_line_dict = {}
ref_line_time_map_dict = {}


def doPathSearch(data, DiGraphic):
    """
    carid, height, radius, startid, endid
    """
    str_list = data.decode().split(',')
    carID = int(str_list[0])
    height = float(str_list[1])
    radius = float(str_list[2])
    src_ID = int(str_list[3])
    end_ID = int(str_list[4])

    if not src_ID in DiGraphic.nodes:
        status = str(0xAA) + str(0xAA) + ',' + str(carID) + ',' + str(
            11) + ',' + str(0xCC) + str(0xCC)
        return status

    if not end_ID in DiGraphic.nodes:
        status = str(0xAA) + str(0xAA) + ',' + str(carID) + ',' + str(
            12) + ',' + str(0xCC) + str(0xCC)
        return status

    if src_ID == end_ID:  # 最小回环
        if not bool(DiGraphic[src_ID]):  # 后续无连接,直接返回自身
            status = str(0xAA) + str(0xAA) + ',' + str(carID) + ',' + str(
                1) + ',' + str(2) + ',' + str(src_ID) + ',' + str(
                    end_ID) + ',' + str(0xCC) + str(0xCC)
            return status

        length = 0
        path = []
        for next_ID in DiGraphic[src_ID].keys():  # 枚举
            try:
                length0 = nx.shortest_path_length(DiGraphic,
                                                  next_ID,
                                                  end_ID,
                                                  weight='length')
                path0 = nx.shortest_path(DiGraphic,
                                         next_ID,
                                         end_ID,
                                         weight='length')

                print('path:{}, length:{}'.format(path0, length0))
                if length == 0:  # 第一次计算
                    length = length0
                    path = path0
                elif length > length0:  # 新的更短
                    length = length0
                    path = path0
                status = str(0xAA) + str(0xAA) + ',' + str(carID) + ',' + str(
                    1) + ',' + str(len(path) + 1) + ',' + str(
                        src_ID)  # 把src_ID先加入进去
                for IDn in path:
                    status = status + ',' + str(IDn)
                status = status + ',' + str(0xCC) + str(0xCC)
            except nx.NetworkXNoPath:  # 没有回环路径,直接返回自身
                status = str(0xAA) + str(0xAA) + ',' + str(carID) + ',' + str(
                    1) + ',' + str(2) + ',' + str(src_ID) + ',' + str(
                        end_ID) + ',' + str(0xCC) + str(0xCC)
        return status

    try:
        length = nx.shortest_path_length(DiGraphic,
                                         src_ID,
                                         end_ID,
                                         weight='length')
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


def getConnectMapFromJson(filename):
    """
    读入连通图数据
    """
    connect_map_dict = {}
    with open(filename, 'r') as r:
        lines = r.readlines()
        for line in lines:
            str_list = line.split(',')
            connect_map_dict[int(str_list[0])] = list(
                int(value) for value in str_list[1:])
    return connect_map_dict


def getRefLineTimeMapFromJson(filename):
    """
    将slength/time作为参数
    """
    ref_line_dict = {}
    ref_line_time_map_dict = {}
    with open(filename, 'r') as r:
        lines = r.readlines()
        for line in lines:
            ref_line = map_element.RefLineDef()
            ref_line.fromStr(line)
            ref_line_dict[ref_line.index] = ref_line
            ref_line_time_map_dict[
                ref_line.index] = ref_line.slength / ref_line.speed_limited[1]
    return ref_line_dict, ref_line_time_map_dict


def makeCurrentGraphic(connect_map_dict, ref_line_dict, ref_line_time_map_dict,
                       current_height, current_radius):
    Graphic = nx.DiGraph()
    for key in connect_map_dict.keys():
        value = connect_map_dict[key]
        for nextID in value:
            ref_line = ref_line_dict[nextID]
            # 实际高度小于限高，实际转弯半径小于车道曲率半径
            if current_height < ref_line.height_limited and (
                    ref_line.curv == 0 or current_radius < 1 / ref_line.curv):
                Graphic.add_weighted_edges_from(
                    [(key, nextID, ref_line_time_map_dict[key])],
                    weight='length')
    return Graphic


class Handler(BaseRequestHandler):
    def handle(self):
        while True:
            data = self.request.recv(BUF_SIZE)
            if len(data) > 0:
                print('receive={}'.format(data))
                str_list = data.decode().split(',')
                current_heigth = float(str_list[1])
                current_radius = float(str_list[2])
                DiGraphic = makeCurrentGraphic(connect_map_dict, ref_line_dict,
                                               ref_line_time_map_dict,
                                               current_heigth, current_radius)
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


####################################################################################################
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        file_dir = sys.argv[1]
        connect_map_file = file_dir + "_connect_map.json"
        ref_line_map_file = file_dir + "_ref_line.json"
    else:
        file_dir = "./data"
        connect_map_file = file_dir + "/connect_map.json"
        ref_line_map_file = file_dir + "/ref_line.json"
    connect_map_dict = getConnectMapFromJson(connect_map_file)
    for _, key in enumerate(connect_map_dict):
        print('main:{}:{}'.format(key, connect_map_dict[key]))

    ref_line_dict, ref_line_time_map_dict = getRefLineTimeMapFromJson(
        ref_line_map_file)
    for _, key in enumerate(ref_line_time_map_dict):
        print('main:ref_line->{}:{}'.format(key, ref_line_time_map_dict[key]))

    # 通过networkx画连通图
    DiGraphic = nx.DiGraph()
    for _, key in enumerate(connect_map_dict):
        value = connect_map_dict[key]
        print('main: for nextID:{}:{}'.format(key, value))
        for nextID in value:
            # print('main: for nextID:{}:{}'.format(key, nextID))
            # 需要走过的是车辆所在的ID，所以用所在车道的长度/限速来度量经过的路程
            DiGraphic.add_weighted_edges_from(
                [(key, nextID, ref_line_time_map_dict[key])], weight='length')

    # print(list(DiGraphic[21].keys())[0])

    HOST = get_host_ip()
    print(HOST)
    PORT = 19001
    ADDR = (HOST, PORT)
    server = ThreadingTCPServer(ADDR, Handler)  # 参数为监听地址和已建立连接的处理类
    print('listening')
    server.serve_forever()  # 监听，建立好TCP连接后，为该连接创建新的socket和线程，并由处理类中的handle方法处理
