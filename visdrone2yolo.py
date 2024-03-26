"""
将visdrone数据集格式转化为yolo格式数据集
这里主要实现的是标签格式的转换
转化的形式如下：
684,8,273,116,0,0,0,0 -->> 0 0.436842 0.387755 0.129825 0.306122
yolo格式的第一列为目标种类，而visdrone的第六列为目标的种类
yolo的格式为：
DataSet:
|--images:
|--|--train
|--|--val
|--labels:
|--|--train
|--|--val

只保留行人和车辆两个类别的数据
图片可以手动复制进去
"""
import os
from pathlib import Path
from PIL import Image
import csv

def convert(size, box):
    # 分别为 [x_center, y_center, width, height]
    dw = 1. / size[0]
    dh = 1. / size[1]
    x = (box[0] + box[2] / 2) * dw # x_center
    y = (box[1] + box[3] / 2) * dh # y_center
    w = box[2] * dw
    h = box[3] * dh
    return (x, y, w, h)

wd = os.getcwd()
# print(wd)   # 获取当前路径

path1=r'VisDrone2019-DET-train\annotations'   # visdrone训练集标签所在的位置
path2=r'VisDrone2019-DET-val\annotations'     # visdrone验证集标签所在的位置
path3=r'VisDrone2019-DET-test-dev\annotations'# visdrone测试集标签所在的位置

# 图片所在位置
picPath1=r'VisDrone2019-DET-train\images'
picPath2=r'VisDrone2019-DET-val\images'
picPath3=r'VisDrone2019-DET-test-dev\images'

# 转换为yolo标签的位置
train_path=r'visdrone_Yolo\labels\train'
val_path=r'visdrone_Yolo\labels\val'

test_path=r'test'

doc_list = os.listdir(path3)

# print(anns)

for doc in doc_list:
    result=''
    outpath=test_path+'\\'+doc
    # print(outpath)
    with Image.open(picPath3+'\\'+doc[:-3]+'jpg') as Img:
        imgSize=Img.size # 得到图片形状大小
        # print(imgSize)
    
    with open(path3+'\\'+doc,newline='') as f:
        fc=csv.reader(f)  # 转换为csv格式
        for row in fc:
            # print(row)
            if row[4] == '0':  #为0时忽略这个区域
                continue
            if row[5] == '1' or row[5] == '2': # 判断为人和行人
                # print(row[:5])
                bbox=convert(imgSize,box=tuple(map(int,row[:4])))
                # print(bbox)
                result = result + '0' + ' ' + ' '.join(str(box) for box in bbox) + '\n'
            
            elif row[5] == '4' or row[5] == '5' or row[5] == '6': # 判断为汽车、面包车、卡车
                bbox=convert(imgSize,box=tuple(map(int,row[:4])))
                # print(bbox)
                result = result + '1' + ' ' + ' '.join(str(box) for box in bbox) + '\n'
    
    # print(result)
    with open(outpath,mode='w') as outfile:
        outfile.write(result)


    # break