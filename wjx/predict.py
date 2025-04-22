
from ultralytics import YOLO
# 加载训练好的模型，改为自己的路径
model = YOLO('/home/dianrobot/PycharmProjects/dianrobot/wjx/runs/detect/train2/weights/best.pt')  #修改为训练好的路径
source = '/home/dianrobot/PycharmProjects/dianrobot/21.jpg' #修改为自己的图片路径及文件名
# 运行推理，并附加参数
model.predict(source, save=False)