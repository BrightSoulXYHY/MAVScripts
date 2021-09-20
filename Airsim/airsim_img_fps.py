import cv2
import numpy as np
import airsim
import sys

import time


client = airsim.MultirotorClient("192.168.1.192")


def get_asimg(instense):
    vehicle_name = 'PX4_{:02d}'.format(instense)
    responses = client.simGetImages([
        airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)
        ],
        vehicle_name = vehicle_name
    )
    response = responses[0]
    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) 
    img_rgb = img1d.reshape(response.height, response.width, 3)
    return img_rgb


if __name__ == '__main__':
    instense = 0
    if len(sys.argv) > 1:
        instense = sys.argv[1]
    while True:
        start_time = time.time()
        img = get_asimg(instense)
        print("fps={:.2f}".format(1/(time.time()-start_time)))