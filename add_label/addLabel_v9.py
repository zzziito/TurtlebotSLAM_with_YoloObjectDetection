import cv2
import numpy as np

###########################################################################
###############  Simple BlobDetector로 full_map blob 추출   ###############
###########################################################################

img = cv2.imread("map.pgm")
height3, width3 = img.shape[:2]
img = cv2.resize(img, (int(width3 * 5), int(height3*5)), None, 0, 0, cv2.INTER_CUBIC)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

params = cv2.SimpleBlobDetector_Params()

params.minThreshold = 70
params.maxThreshold = 100
params.thresholdStep = 5
params.filterByArea = True
params.minArea = 50

params.filterByColor = False
params.filterByConvexity = False
params.filterByInertia = False
params.filterByCircularity = False 

detector = cv2.SimpleBlobDetector_create(params)
keypoints = detector.detect(gray)

keyDatas = []
keyImgs = []
for i in range(0, len(keypoints)):
    x = int(keypoints[i].pt[0])
    y = int(keypoints[i].pt[1])
    s = int(keypoints[i].size)
    keyDatas.append([x, y, s])

    img_i = gray[x-s*2:x+s, y-s:y+s*2]

    keyImgs.append(img_i)

img_draw = cv2.drawKeypoints(img, keypoints, None, None,\
                     cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

#############################################################################
###############  feature matching을 위해서 full_map slicing   ###############
#############################################################################

image = cv2.imread('map.pgm')
height, width = image.shape[:2]
image_gray = cv2.imread('map.pgm', cv2.IMREAD_GRAYSCALE)


image = cv2.resize(image, (int(width * 5), int(height*5)), None, 0, 0, cv2.INTER_CUBIC)
image_gray = cv2.resize(image_gray, (int(width * 5), int(height*5)), None, 0, 0, cv2.INTER_CUBIC)


blur = cv2.GaussianBlur(image_gray, ksize = (3, 3), sigmaX=0)
ret, thresh1 = cv2.threshold(blur, 127, 225, cv2.THRESH_BINARY)

edged = cv2.Canny(blur, 10, 250)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7, 7))
closed = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)

contours, _ = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
total = 0

contours_xy = np.array(contours)
contours_xy.shape

x_min, x_max = 0, 0
value = list()
for i in range(len(contours_xy)):
    for j in range(len(contours_xy[i])):
        value.append(contours_xy[i][j][0][0])
        x_min = min(value)
        x_max = max(value)

y_min, y_max = 0, 0
value = list()
for i in range(len(contours_xy)):
    for j in range(len(contours_xy[i])):
        value.append(contours_xy[i][j][0][1])
        y_min = min(value)
        y_max = max(value)

inner_map_gray = image_gray[y_min+40:y_max-30, x_min+40:x_max-30]
inner_map = image[y_min+40:y_max-30, x_min+40:x_max-30]


############################################################################################
###############  장애물 map 읽어와서 feature matching에 사용하기 위해 보정   ###############
############################################################################################

fileList = ['sports ball.pgm']


image2 = cv2.imread(fileList[0], cv2.IMREAD_COLOR)
height2, width2 = image2.shape[:2]
for x in range(0, width2):
    for y in range(0, height2):
        if image2[y, x][0] == 205 and image2[y, x][1] == 205 and image2[y, x][2] == 205: 
            image2[y, x] = (255, 255, 255)

image2_gray = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
image2 = cv2.resize(image2, (int(width * 5), int(height*5)), None, 0, 0, cv2.INTER_CUBIC)
image2_gray = cv2.resize(image2_gray, (int(width * 5), int(height*5)), None, 0, 0, cv2.INTER_CUBIC)

blur = cv2.GaussianBlur(image2_gray, ksize = (3, 3), sigmaX=0)
ret, thresh1 = cv2.threshold(blur, 127, 225, cv2.THRESH_BINARY)

edged = cv2.Canny(blur, 10, 250)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7, 7))
closed = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)

contours, _ = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
total = 0

contours_xy = np.array(contours)
contours_xy.shape

x_min, x_max = 0, 0
value = list()
for i in range(len(contours_xy)):
    for j in range(len(contours_xy[i])):
        value.append(contours_xy[i][j][0][0])
        x_min = min(value)
        x_max = max(value)

y_min, y_max = 0, 0
value = list()
for i in range(len(contours_xy)):
    for j in range(len(contours_xy[i])):
        value.append(contours_xy[i][j][0][1])
        y_min = min(value)
        y_max = max(value)

inner_map2_gray = image2_gray[y_min-30:y_max+30, x_min+30:x_max+30]
inner_map2 = image2[y_min-30:y_max+30, x_min+30:x_max+30]


#############################################################################
###############  SIFT, FlannBasedMatcher로 feature matching   ###############
#############################################################################

detector = cv2.xfeatures2d.SIFT_create()

kp1, desc1 = detector.detectAndCompute(inner_map_gray, None)
kp2, desc2 = detector.detectAndCompute(inner_map2_gray, None)

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)

matcher = cv2.FlannBasedMatcher(index_params, search_params)

matches = matcher.match(desc1, desc2)

matches = sorted(matches, key=lambda x:x.distance)
min_dist, max_dist = matches[0].distance, matches[-1].distance

ratio = 0.4

good_thresh = (max_dist - min_dist) * ratio + min_dist

good_matches = [m for m in matches if m.distance < good_thresh]

######################################################################################################
###############  두 장애물 근처 영역에 위치한 keypoints 수를 비교해 장애물 class 매칭  ###############
######################################################################################################

num_kp = []

for data in keyDatas:
    count = 0
    for mat in good_matches:
        x = kp1[mat.queryIdx].pt[0] + x_min + 40
        y = kp1[mat.queryIdx].pt[1] + y_min + 40
        if x in range(data[0] - data[2] *2, data[0] + data[2]) and y in range(data[1] - data[2], data[1] + data[2] * 2):
            count = count + 1
    num_kp.append(count)


img_draw = cv2.putText(img_draw, fileList[0][:-4], (keyDatas[0][1], keyDatas[0][0] + 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 1, cv2.LINE_AA)

cv2.imshow('final', img_draw)
cv2.waitKey()
cv2.destroyAllWindows()

