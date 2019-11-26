#!/usr/bin/env python
import sys, time
import numpy as np

# Ros
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from PIL import Image as PILimage
from image_geometry import PinholeCameraModel
import message_filters
# Pytorch
import torch
# Our library
import network
import utils

VERBOSE = True
publish_image = True

global count
def to_tensor(image):
    # normalize
    mean = [118, 117, 117]
    std = [57, 58, 60]
    image = image.astype(np.float32)
    image = (image - mean) / std
    # swap color axis because
    # numpy image: H x W x C
    # torch image: C X H X W
    image = image.transpose((2, 0, 1))
    image = np.expand_dims(image, axis=0)
    image = torch.from_numpy(image).float()
    return image


class barbie_detector:
    def __init__(self):
        rospy.init_node('barbie_detector')
        # topics where we publish
        self.image_pub = rospy.Publisher("/barbie_detections", Image, queue_size=1)
        self.point_pub = rospy.Publisher("/barbie_point", PointStamped, queue_size=1)
        # subscribed topics
        self.image_subscriber = message_filters.Subscriber("/camera/rgb/image_raw", Image)
        self.cam_info_subscriber = message_filters.Subscriber("/camera/rgb/camera_info", CameraInfo)
        self.depth_subscriber = message_filters.Subscriber("/camera/depth_registered/sw_registered/image_rect", Image)
        # time synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_subscriber,  self.depth_subscriber, self.cam_info_subscriber], 5, 0.5)

        # network
        self.model = network.Net()
        weights_path = rospy.get_param('~weights_path', 'trained_weights')
        self.model.load_state_dict(torch.load(weights_path, map_location='cpu'))
        rospy.loginfo("Weights loaded from path: %s", weights_path)
        # callback registration
        self.ts.registerCallback(self.callback)
        print ("Detector initialized")
	
        self.pbl = rospy.Publisher("framed",PointStamped, queue_size = 10)



    def callback(self, image_data, depth_data, cam_info):
        global count
        det = False

        # direct conversion to numpy
        np_data = np.fromstring(image_data.data, np.uint8)
        in_image = np_data.reshape(image_data.height, image_data.width,3)

        time1 = time.time()
        # evaluate network
        out_heat = {}
        det_treshhold = 4.0
        scales = [1, 1.5, 2, 3]
        s_i = 0
        for scale in scales:
            # resize image
            im = PILimage.fromarray(in_image)
            image_r = im.resize((int(im.size[0] / scale), int(im.size[1] / scale)))
            image_r = np.array(image_r)
            # transform numpy to tensor
            image_r = to_tensor(image_r)
            # evaluate model
            output = self.model(image_r)

            out_heat[s_i] = output[0, 0, :, :].detach().cpu().numpy()
            s_i += 1

        # maximum output of all scales
        max_idx = np.argmax([out_heat[0].max(), out_heat[1].max(), out_heat[2].max(), out_heat[3].max()])
        max_val = np.max([out_heat[0].max(), out_heat[1].max(), out_heat[2].max(), out_heat[3].max()])
	print("MAX_VAL", max_val)
	
        tmp = PointStamped()
        self.pbl.publish(tmp)
        if max_val> det_treshhold:
        #   count +=1
        #if count > 2:
            det = True
            out_max = utils.max_filter(out_heat[max_idx], size=500)
            # get bbox of detection
            bbox = utils.bbox_in_image(
                np.zeros([int(in_image.shape[0] / scales[max_idx]), int(in_image.shape[1] / scales[max_idx])]), out_max,
                [32, 24], det_treshhold)


        time2 = time.time()
        # print ('TIME',time2-time1)
        # print ('DETECTION COUNTER', count)
        if det:
        #    count = 0
            # get depth
            np_data = np.fromstring(depth_data.data, np.float32)
            in_depth = np_data.reshape(depth_data.height, depth_data.width)
            in_depth[np.isnan(in_depth)] = 0
            # convert depth to PIL image and resize
            depth_image = PILimage.fromarray(in_depth)
            depth_image = depth_image.resize((in_image.shape[1], in_image.shape[0]))
            # convert depth to numpy back
            depth = np.array(depth_image)

            # create mask for detected bbox
            mask = np.zeros(in_image.shape[0:2]).astype(np.uint8)
            mask[int(bbox[0, 1] * image_data.height):int(bbox[0, 3] * image_data.height),
            int(bbox[0, 0] * image_data.width):int(bbox[0, 2] * image_data.width)] = 1

            # estimate the depth
            depths = depth[mask == 1]
            if (len(depths)>0) & (sum(depths)>0):
                d = np.percentile(depths[depths != 0], 50)
            else:
                # if no depth information known -> skip
                return
            # (u,v) coordinates of the filter
            u = np.argmax(out_max, axis=1).max()*8*scales[max_idx]
            v = np.argmax(out_max, axis=0).max()*8*scales[max_idx]

            # project points to X,Y,Z
            K = np.matrix([[cam_info.K[0], cam_info.K[1], cam_info.K[2]], [cam_info.K[3], cam_info.K[4], cam_info.K[5]],
                           [cam_info.K[6], cam_info.K[7], cam_info.K[8]]])

            Kinv = np.linalg.inv(K)
            U = np.matrix([[u], [v], [1]])
            X = d * Kinv * U

            # project points to X,Y,Z
            cam_model = PinholeCameraModel()
            cam_model.fromCameraInfo(cam_info)
            X = cam_model.projectPixelTo3dRay((u,v))
            X = np.array(X)
            X = X/X[2]
            X = X*d


            # create a 3D point
            barbie_point = PointStamped()
            barbie_point.header = image_data.header
            barbie_point.point.x = X[0]
            barbie_point.point.y = X[1]
            barbie_point.point.z = X[2]
            self.point_pub.publish(barbie_point)

        if publish_image:
            # this will draw bbox in image
            if det:
                in_image[int(bbox[0, 1] * image_data.height):int(bbox[0, 3] * image_data.height),
                int(bbox[0, 0] * image_data.width):int(bbox[0, 0] * image_data.width) + 2, 1] = 255
                in_image[int(bbox[0, 1] * image_data.height):int(bbox[0, 3] * image_data.height),
                int(bbox[0, 2] * image_data.width) - 3:int(bbox[0, 2] * image_data.width) - 1, 1] = 255
                in_image[int(bbox[0, 1] * image_data.height):int(bbox[0, 1] * image_data.height) + 2,
                int(bbox[0, 0] * image_data.width):int(bbox[0, 2] * image_data.width), 1] = 255
                in_image[int(bbox[0, 3] * image_data.height) - 3:int(bbox[0, 3] * image_data.height) - 1,
                int(bbox[0, 0] * image_data.width):int(bbox[0, 2] * image_data.width), 1] = 255

            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.data = in_image.tostring()
            msg.height = image_data.height
            msg.width = image_data.width
            msg.step = 1920
            msg.encoding = 'rgb8'
            msg.is_bigendian = 0
            self.image_pub.publish(msg)


if __name__ == '__main__':
    print("hello")
    count = 0
    ic = barbie_detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
