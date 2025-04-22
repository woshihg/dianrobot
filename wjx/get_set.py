from ultralytics import YOLO

model = YOLO('/home/dianrobot/PycharmProjects/dianrobot/detect.pt')

jpg_path = '/home/dianrobot/PycharmProjects/dianrobot/wjx/JPEGImages/00.jpg'

result = model(jpg_path, show=True, save=False)

print(type(result))

print(type(result[0]))

print(result[0].boxes.xyxy[0])  # 预测框坐标
print(result[0].boxes.conf[0])  # 置信度
print(result[0].boxes.cls[0])  # 类别
