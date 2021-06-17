#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters

import cv2
import sm
import aslam_cv as acv
import aslam_cv_backend as acvb
import aslam_cameras_april as acv_april
import kalibr_common as kc

import os
import time
import numpy as np
import pylab as pl
import argparse
import sys
import getopt
import igraph
import re
import yaml

input_folder_path = '/home/jrebello/projects/calibration_data/ijrr/jun15/'
output_folder_path = '/home/jrebello/projects/AC-DCC/data/realworld1/train/'
encoder_filename = 'encoder_deg.txt'
target_filename = 'target.yaml'
camera_names = ['gimbal','static1','static2']
num_images = 120
start_idx = 0

# make numpy print prettier
np.set_printoptions(suppress=True)

def atoi(text):
    return int(text) if text.isdigit() else text.lower()

def natural_keys(text):
    return [ atoi(c) for c in re.split('(\d+)', text) ]

# check if directory exists; if not, create it
def create_directory (directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

class CalibrationTargetDetector(object):

    def __init__(self, targetConfig):
        targetParams = targetConfig.getTargetParams()
        targetType = targetConfig.getTargetType()

        #set up target
        if( targetType == 'checkerboard' ):
            self.grid = acv.GridCalibrationTargetCheckerboard(targetParams['targetRows'],
                                                         targetParams['targetCols'],
                                                         targetParams['rowSpacingMeters'],
                                                         targetParams['colSpacingMeters'])

        elif( targetType == 'aprilgrid' ):
            self.grid = acv_april.GridCalibrationTargetAprilgrid(targetParams['tagRows'],
                                                            targetParams['tagCols'],
                                                            targetParams['tagSize'],
                                                            targetParams['tagSpacing'])
        else:
            raise RuntimeError( "Unknown calibration target." )

        #setup detector
        self.options = acv.GridDetectorOptions()
        self.options.filterCornerOutliers = True

class MeasurementGenerator(object):

    def __init__(self):
        self.loadTragetParams()
        self.loadCameraParams()
        self.readEncoderAngles()
        self.computeMeasurements()

    def loadTragetParams(self):
        targetConfig = kc.ConfigReader.CalibrationTargetParameters(input_folder_path + target_filename)
        print("Initializing calibration target:".format(targetConfig.getTargetType()))
        targetConfig.printDetails()
        self.target = CalibrationTargetDetector( targetConfig)
        print("=====================================")
    
    def loadCameraParams(self):
        self.cameras = []
        for camera_name in camera_names:
            print('\nLoading Camera: ' + camera_name)
            camera_chain = kc.ConfigReader.CameraChainParameters(input_folder_path + camera_name +'.yaml')
            camera_chain.printDetails()
            camera_config = camera_chain.getCameraParameters(0)
            self.cameras.append(kc.ConfigReader.AslamCamera.fromParameters(camera_config))
            print("=====================================")
    
    def readEncoderAngles(self):
        print("Reading IMU angles:")
        self.imu_angles_deg = np.loadtxt(input_folder_path + encoder_filename, delimiter=' ', usecols=(0,1,2))
        print("=====================================")

    def computeMeasurements(self):
        for camera_num in range(len(camera_names)):
            camera = self.cameras[camera_num]
            camera_name = camera_names[camera_num]
            detector = acv.GridDetector(camera.geometry, self.target.grid, self.target.options)
            for image_num in range(1, num_images+1):
                image_path = input_folder_path + str(image_num) + '_' + camera_name   + '.jpeg'
                print(image_path)
                image = cv2.imread(image_path, 0)
                np_image = np.array(image)
                success, observation = detector.findTarget(acv.Time(0, 0), np_image)
                observation.clearImage()

                if success:
                    print("Writing image" + ' ' + str(image_num))
                    curr_imu_angle = self.imu_angles_deg[image_num-1]
                    [image_corners, reproj_corners, meanReprojectionError] = self.generateMonoview(camera_num, np_image, observation, success)
                    
                    target_corners = observation.getCornersTargetFrame()
                    T_t_c = observation.T_t_c()
                    T_c_t = np.linalg.inv(T_t_c.T())

                    output_file_name = str(image_num + start_idx) + "_" + camera_name + str(".txt")

                    # write the gridpoints to file
                    if not os.path.exists(output_folder_path):
                        create_directory(output_folder_path)
                    write_path = os.path.join(output_folder_path, output_file_name)

                    outfile = open(write_path,"w")

                    outfile.write("T_CW:\n" )
                    tmatrix = np.array(T_c_t)
                    tmatrix_str = '\n'.join(" ".join('%0.10f' %x for x in y) for y in tmatrix)
                    outfile.write(tmatrix_str)
                    outfile.write("\n\n")

                    outfile.write("T_CW_cov:\n" )
                    outfile.write("\n")
                    
                    outfile.write("target_points_pix:\n" )

                    for corner, pixel in zip(target_corners,image_corners):
                        line = "{:.5f}".format(corner[0])  + " " + "{:.5f}".format(corner[1]) + " " + "{:.5f}".format(corner[2]) + " " + "{:.5f}".format(pixel[0]) + " "+  "{:.5f}".format(pixel[1]) + "\n"
                        outfile.write(line)
                    
                    outfile.write("\ngimbalangles:\n")
                    for i in range(len(curr_imu_angle)):
                        outfile.write(str(np.radians(curr_imu_angle[i])) + ' ')
                    outfile.write("\n\n")
                    
                    outfile.write("end:")
                    outfile.close()
    
    def generateMonoview(self, camera_num, np_image, observation, obs_valid):
        np_image = cv2.cvtColor(np_image, cv2.COLOR_GRAY2BGR)
        camera = self.cameras[camera_num]
        window_name = camera_names[camera_num]
        if obs_valid:
            #calculate the reprojection error statistics
            cornersImage = observation.getCornersImageFrame()
            cornersReproj = observation.getCornerReprojection(camera.geometry)

            reprojectionErrors2 = cornersImage-cornersReproj
            reprojectionErrors = np.sum(np.abs(reprojectionErrors2)**2, axis=-1)**(1./2)

            outputList = [ ( "mean_x:  ", np.mean(reprojectionErrors2[:,0]) ),
                           ( "std_x:   ", np.std(reprojectionErrors2[:,0]) ),
                           ( "max_y:   ", np.max(reprojectionErrors2[:,0]) ),
                           ( "min_x:   ", np.min(reprojectionErrors2[:,0]) ),
                           ( "", 0),
                           ( "mean_y:  ", np.mean(reprojectionErrors2[:,1]) ),
                           ( "std_y:   ", np.std(reprojectionErrors2[:,1]) ),
                           ( "max_y:   ", np.max(reprojectionErrors2[:,1]) ),
                           ( "min_y:   ", np.min(reprojectionErrors2[:,1]) ),
                           ( "", 0),
                           ( "mean_L2: ", np.mean(reprojectionErrors) ),
                           ( "std_L2:  ", np.std(reprojectionErrors) ),
                           ( "max_L2:  ", np.max(reprojectionErrors) ),
                           ( "min_L2:  ", np.min(reprojectionErrors) )      ]

            meanReprojectionError = np.mean(reprojectionErrors)

            #print the text
            y = 20; x = 20
            for err_txt, err_val in outputList:
                fontScale = 0.75
                y += int(42*fontScale)

                if err_txt == "":
                    continue

                cv2.putText(np_image, err_txt, (x,y), cv2.FONT_HERSHEY_SIMPLEX, fontScale=fontScale, color=(0, 0, 255), thickness=2)
                cv2.putText(np_image, "{0: .4f}".format(err_val), (x+100,y), cv2.FONT_HERSHEY_SIMPLEX, fontScale=fontScale, color=(0, 0, 255), thickness=2)

            #draw reprojected corners
            for px, py in zip(cornersReproj[:,0],cornersReproj[:,1]):
                #convert pixel to fixed point (opencv subpixel rendering...)
                shift = 4; radius = 0.5; thickness = 1
                px_fix =  int(px * 2**shift)
                py_fix =  int(py * 2**shift)
                radius_fix = int(radius * 2**shift)
                cv2.circle(np_image, (px_fix, py_fix), radius=radius_fix, color=(255,255,0), thickness=thickness, shift=shift, lineType=cv2.LINE_AA)

        else:
            cv2.putText(np_image, "Detection failed...", (np_image.shape[0]/2,np_image.shape[1]/5), cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5, color=(0, 0, 255), thickness=2)

        cv2.imshow(window_name, np_image)
        cv2.waitKey(1)

        if obs_valid:
            return [cornersImage, cornersReproj, meanReprojectionError]
                    

if __name__ == "__main__":
    MeasurementGenerator()
    print("All Done !!!!")
    cv2.destroyAllWindows()