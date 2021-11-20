import numpy as np, copy
import copy



def iou_2d(x, y):
    area_x = abs(x[0][0] - x[1][0]) * abs(x[1][1] - x[2][1])

    area_y = abs(y[0][0] - y[1][0]) * abs(y[1][1] - y[2][1])
    whole_area = area_x + area_y
    x = x.tolist()
    y = y.tolist()
    list = polygon_clip(x, y)
    if list == None:
        return 0
    else:
        arra = np.array(list)
        xx = arra[:, 0]
        yy = arra[:, 1]
        intersection = poly_area(xx, yy)

        iou_d = intersection / (whole_area - intersection)
        return iou_d


def poly_area(x, y):
    """ Ref: http://stackoverflow.com/questions/24467972/calculate-area-of-polygon-given-x-y-coordinates """  ###计算凸多边形的面积
    return 0.5 * np.abs(
        np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))  ###np.abs()返回绝对值，dot函数返回向量内积 roll函数沿着给定数轴滚动


def convert_2dbox_to_4conrner(bbox2d_input):
    bbox = copy.copy(bbox2d_input)
    x1 = bbox[0]
    y1 = bbox[1]
    x2 = bbox[2]
    y2 = bbox[3]
    # bbox2d=[(x1,x2),(x3,x2),(x3,x4),(x1,x4)]
    xx = [x1, x2, x2, x1]
    yy = [y1, y1, y2, y2]
    corners = np.vstack([xx, yy])
    # bbox2d=np.array(bbox2d)
    # return bbox2d
    return np.transpose(corners)


def polygon_clip(subjectPolygon, clipPolygon):
    """ Clip a polygon with another polygon.
	Ref: https://rosettacode.org/wiki/Sutherland-Hodgman_polygon_clipping#Python

	Args:
		subjectPolygon: a list of (x,y) 2d points, any polygon.
		clipPolygon: a list of (x,y) 2d points, has to be *convex*
	Note:
		**points have to be counter-clockwise ordered**        ###逆时针方向

	Return:
		a list of (x,y) vertex point for the intersection polygon.     ###相交多边形的(x,y)顶点的列表
	"""

    def inside(p):
        return (cp2[0] - cp1[0]) * (p[1] - cp1[1]) > (cp2[1] - cp1[1]) * (p[0] - cp1[0])

    def computeIntersection():
        dc = [cp1[0] - cp2[0], cp1[1] - cp2[1]]
        dp = [s[0] - e[0], s[1] - e[1]]
        n1 = cp1[0] * cp2[1] - cp1[1] * cp2[0]
        n2 = s[0] * e[1] - s[1] * e[0]
        n3 = 1.0 / (dc[0] * dp[1] - dc[1] * dp[0])
        return [(n1 * dp[0] - n2 * dc[0]) * n3, (n1 * dp[1] - n2 * dc[1]) * n3]

    outputList = subjectPolygon
    cp1 = clipPolygon[-1]

    for clipVertex in clipPolygon:
        cp2 = clipVertex
        inputList = outputList
        outputList = []
        s = inputList[-1]

        for subjectVertex in inputList:
            e = subjectVertex
            if inside(e):
                if not inside(s): outputList.append(computeIntersection())
                outputList.append(e)
            elif inside(s):
                outputList.append(computeIntersection())
            s = e
        cp1 = cp2
        if len(outputList) == 0: return None
    return (outputList)
