# 用来检查连通图的正确性

import csv as csv
import networkx as nx
import matplotlib.pyplot as plt
import PIL.Image as PIImg
import numpy as np

import socket
from enum import Enum


class PathErrorCode(Enum):
    Success = 1
    NoStartID = 11
    NoEndID = 12
    NoPath = 13


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
        return {}
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
        return {}
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
                     ref_line_point_map_dict, plot_ax):
        pos_dict = {}
        label_dict = {}
        for _, key in enumerate(ref_line_point_map_dict):
            # pos_dict[key] = (np.array(ref_line_point_map_dict[key][0]) +
            #                  np.array(ref_line_point_map_dict[key][-1])) / 2
            pos_dict[key] = (ref_line_point_map_dict[key][int(
                len(ref_line_point_map_dict[key]) / 2)])
            print('pos = {}:{}'.format(key, pos_dict[key]))
            label_dict[key] = str(key)

        nx.draw(direct_graphic, pos_dict, ax=plot_ax)
        nx.draw_networkx_labels(direct_graphic, pos_dict, label_dict)
        # nx.draw_networkx_nodes(direct_graphic, pos_dict)

    def getrouteMapIDList(self, direct_graphic):
        ID_list = 0
        direct_graphic
        return ID_list

    def getPath(self, direct_graphic, start_id, end_id):
        error_code = PathErrorCode

        try:
            length = nx.shortest_path_length(
                G, start_id, end_id, weight='length')
            path = nx.shortest_path(G, start_id, end_id, weight='length')
        except nx.NetworkXNoPath:
            return error_code.NoPath
        return length, path


if __name__ == "__main__":
    connect_map_file = "../data/map/shenzhen/shenzhen_connect_map.json"
    connect_map_dict = getConnectMapFromJson(connect_map_file)
    for _, key in enumerate(connect_map_dict):
        print('main:{}:{}'.format(key, connect_map_dict[key]))

    ref_line_map_file = "../data/map/shenzhen/shenzhen_ref_line_map.json"
    ref_line_length_map_dict = getRefLineLengthMapFromJson(ref_line_map_file)
    for _, key in enumerate(ref_line_length_map_dict):
        print('main:ref_line->{}:{}'.format(key,
                                            ref_line_length_map_dict[key]))

    ref_line_point_map_dict = getRefLinePointMapFromJson(ref_line_map_file)

    # 通过networkx画连通图
    G = nx.DiGraph()
    for _, key in enumerate(connect_map_dict):
        value = connect_map_dict[key]
        for nextID in value:
            # 需要走过的是车辆所在的ID，所以用所在车道的长度来度量经过的路程
            G.add_weighted_edges_from(
                [(key, nextID, ref_line_length_map_dict[key])],
                weight='length')
        print('road ID:{}'.format(key))

    img1 = PIImg.open("../data/map/shenzhen/shenzhen.bmp")
    npimg1 = np.array(img1)
    npimg1 = npimg1[-1:0:-1, :, :]
    # 准备绘图
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(npimg1, origin='lower')
    ax.autoscale(False)

    route_map = routeMap()
    route_map.drawRouteMap(G, connect_map_dict, ref_line_point_map_dict, ax)

    # do path search
    src = int(input('input start id:'))
    end = int(input('input end id:'))

    x = []
    y = []
    print(G.nodes)
    if (src in G.nodes) and (end in G.nodes):
        try:
            length = nx.shortest_path_length(G, src, end, weight='length')
            path = nx.shortest_path(G, src, end, weight='length')
            print('path:{}, length:{}'.format(path, length))
            # show map

            for id in path:
                points = ref_line_point_map_dict[id]
                for p in points:
                    x.append(p[0])
                    y.append(p[1])

            plt.plot(x, y, 'r.')
            plt.show()
        except nx.NetworkXNoPath:
            print('NO path')
    else:
        print('node not in map')

    plt.show()
