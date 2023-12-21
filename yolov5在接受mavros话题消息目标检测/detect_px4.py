import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import argparse
import time
import torch
# import torch.backends.cudnn as cudnn
from numpy import random

from models.experimental import attempt_load
from utils.datasets import letterbox
from utils.general import check_img_size, check_requirements,  non_max_suppression, \
    scale_coords, xyxy2xywh, set_logging
from utils.plots import plot_one_box
from utils.torch_utils import select_device, time_synchronized

def detect(img0):
    """
    img:cv2类型的图像 ，bgr
    在此处进行yolov5对图片的检测，发布检测结果
    需不需要封装一个yolo的检测模型？
    """
    img=letterbox(img0,imgsz,stride=stride)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img = np.ascontiguousarray(img)

    # 单一图片推理
    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    t1 = time_synchronized()
    pred = model(img, augment=opt.augment)[0]

    # Apply NMS
    pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
    t2 = time_synchronized()

    # Process detections
    for i, det in enumerate(pred):  # detections per image
        s, im0 = '', img0
        s += '%gx%g ' % img.shape[2:]  # print string
        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

            # Write results
            for *xyxy, conf, cls in reversed(det):
                if save_txt:  # Write to file
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    line = (cls, *xywh, conf) if opt.save_conf else (cls, *xywh)  # label format

                label = f'{names[int(cls)]} {conf:.2f}'
                plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)

        # Print time (inference + NMS)
        print(f'{s}Done. ({t2 - t1:.3f}s)')

    fps=round(1/(t2-t1),2)
    
    cv2.putText(im0,f'FPS:{fps}',(10,30),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2,cv2.LINE_AA)
    cv2.imshow("Image",im0)
    key =cv2.waitKey(1) & 0xFF
    if key==ord('q'):
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        rospy.signal_shutdown("-----推出图像检测模式----")        

    print(f'Done. ({time.time() - t0:.3f}s)')
    return im0

# 订阅ros发布的图像，然后进行检测，再发布出去
def img_cb(msg):
    bridge=CvBridge()
    # 转化为cv2的图像
    img=bridge.imgmsg_to_cv2(msg,'bgr8')
    # 检测
    result_img=detect(img)
    img_msg=bridge.cv2_to_imgmsg(result_img,encoding='bgr8')
    # 发布消息
    img_pub.publish(img_msg)

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    # 存储权重的文件
    parser.add_argument('--weights', nargs='+', type=str, default='car.pt', help='model.pt path(s)')
    # 检测图像的来源,使用 0 (摄像头) 或者 使用 1.mp4 (视频) 或者 xxx.jpg
    # parser.add_argument('--source', type=str, default='0', help='source')  # file/folder, 0 for webcam
    # 图片的大小
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    # 使用第几号cuda,gpu
    parser.add_argument('--device', default='cpu', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    # 存储检测图像的结果的位置
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    # 存储结果的文件名称
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    opt = parser.parse_args()
    print(opt)
    check_requirements(exclude=('pycocotools', 'thop'))

    weights, view_img, save_txt, imgsz = opt.weights, opt.view_img, opt.save_txt, opt.img_size
    # save_img = not opt.nosave and not source.endswith('.txt')  # save inference images

    # ----初始化
    set_logging()
    device = select_device(opt.device)  #设置cuda
    half = device.type != 'cpu'
    # ----加载模型 
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size
    if half:
        model.half()  # to FP16
        
    # 载入名字和相应的颜色
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

    # 开始推理
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
    t0 = time.time()

    # 初始化节点
    rospy.init_node("Yolov5_detection",anonymous=True)
    # 创建一个发布者
    img_pub=rospy.Publisher("yolov5/detect",Image,queue_size=10)
    # 创建一个订阅者
    rospy.Subscriber("/iris/camera/rgb/image_raw",Image,img_cb)

    rospy.spin()

"""
优化以后检测的速度变快了，检测的帧率更高
重构一下代码
"""
