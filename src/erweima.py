import cv2 as cv
import numpy as np

# 打开视频流（这里以打开摄像头为例，若要打开视频文件，替换括号内参数为视频文件路径即可）
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("无法打开视频流")
    exit()

while True:
    # 读取视频帧
    ret, frame = cap.read()
    if not ret:
        print("视频流读取完毕，退出...")
        break

    # 实例化
    qrcoder = cv.QRCodeDetector()
    # qr检测并解码
    codeinfo, points, straight_qrcode = qrcoder.detectAndDecode(frame)
    if points is not None:
        # 绘制qr的检测结果
        cv.drawContours(frame, [np.int32(points)], 0, (0, 0, 255), 2)
        print(points)
        # 打印解码结果
        print("qrcode :", codeinfo)

    # 显示处理后的帧
    cv.imshow("result", frame)
    # 等待按键，按'q'键退出循环
    if cv.waitKey(1) == ord('q'):
        break

# 释放视频流资源
cap.release()
cv.destroyAllWindows()