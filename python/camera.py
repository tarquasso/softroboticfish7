#!/usr/bin/python
#This is the camera class
import picamera
import time
import os

class FishCamera:
    def __init__(self, outputDir = '.'):
	#init camera
        # Open the camera device.
	self.camera = picamera.PiCamera()
	
        #set default camera values
        self.camera.sharpness = 0
	self.camera.contrast = 0
	self.camera.brightness = 50
	self.camera.saturation = 0
	self.camera.ISO = 0
	self.camera.video_stabilization = False
	self.camera.exposure_compensation = 0
	self.camera.exposure_mode = 'auto'
	self.camera.meter_mode = 'average'
	self.camera.awb_mode = 'auto'
	self.camera.image_effect = 'none'
	self.camera.color_effects = None
	self.camera.rotation = 0
	self.camera.hflip = False
	self.camera.vflip = False
	self.camera.crop = (0.0, 0.0, 1.0, 1.0)

        self.video_record=False #true if we're currently recording
        self.f=None #the file to be saved
        self.video_trigger=True #controller hysteresis variable
        self.picture_trigger=True
        self.outputDir = outputDir
    def __enter__(self):
        return self

    def start_preview(self):
        self.camera.start_preview()

    def take_video(self):
	self.camera.resolution = (640,480)
        f = os.path.join(self.outputDir, str(time.time()).replace('.','_')+'.h264')
        self.camera.start_recording(f)
        # start timeout timer

        #self.camera.wait_recording(10)
        #self.camera.stop_recording()

    def stop_video(self):
        self.camera.stop_recording()

    def take_picture(self, append=''):
	self.camera.resolution = (640,480)
        f = os.path.join(self.outputDir, str(time.time()).replace('.','_')+append+".jpg")
        self.camera.capture(f)
    def __str__(self):
        return 'MainController Status:'
    def cleanup(self):
        self.camera.close()
    def __exit__(self,type,value,traceback):
        self.cleanup()
if (__name__=="__main__"): # for debugging purposes when running just this file
	print 'Code is running'
        with FishCamera() as fishCam:
            print 'taking video'
            #fishCam.start_preview()
            fishCam.take_video()
            #fishCam.take_picture()
            time.sleep(60)
            fishCam.stop_video()
            print 'stopped taking video'
            

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
