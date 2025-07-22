import cv2
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
# yolo v8n 가중치를 사용한다.
# git 코드를 분석해보니 YOLO라는 함수안에서 yolov8n.pt가 default이고,
# string으로 넘겨준 모델이 없으면 git에서 download하여 사용한다.
# 잘못된 글자라면 에러가 출력됨

cap = cv2.VideoCapture(0)
# cv2에서 웹캠을 사용하겠다.

while cap.isOpened():
    success, frame = cap.read()
    # cv2에서 웹캠을 read한다.
    # cap.read()함수는 튜플로 2개의 인자를 리턴하는데,
    # 첫번째 인자는 frame을 가져왔는지의 bool 변수 (true, false)이고
    # 두번째 인자는 웹캠프레임 1장의 이미지를 가지고 온다.

    if success:
        results= model(frame)
        # frame이미지를 모델에 넣어 결과를 가지고 온다.
        # 리턴하는값은 리스트이다.
        annotated_frame = results[0].plot()
        # 이걸 plot()하게 되면 numpy 어레이 좌표가 나온다.
        # 이 좌표값들은 검출된 박스를 그려주는 좌표다.
        cv2.imshow("YOLOv8 Inference", annotated_frame)
        # 이 좌표값들이 표시된걸 inshow에 넣으면 박스가 그려져서 출력된다.

        if cv2.waitKey(1)&0xFF == ord("q"): # 키값이 들어왔는데 q라면 꺼진다.
            break
    else:
        print("error")
        break

cap.release() # 윕캠을 끈다.
cv2.destroyAllWindows() # cv2로 열어진 모든 창을 종료시킨다.




# import cv2
# import supervision as sv
# from ultralytics import YOLO

# model = YOLO('yolov8n.pt')
# result = model.predict(cv2.imread('temp.jpg'))[0]
# detections = sv.Detections.from_yolov8(result)
# labels = [[f"{model.model.names[class_id]}", confidence] for _, _, confidence, class_id, _ in detections]
# labels
# '''
# 0: 640x480 5 persons, 1 tv, 140.1ms
# Speed: 5.0ms preprocess, 140.1ms inference, 10.9ms postprocess per image at shape (1, 3, 640, 480)
# [['tv', 0.9154752],
#  ['person', 0.860976],
#  ['person', 0.84003013],
#  ['person', 0.75352204],
#  ['person', 0.40899122],
#  ['person', 0.3967925]]
# '''
