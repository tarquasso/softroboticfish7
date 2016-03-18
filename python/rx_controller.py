
print '\n\n*** LAUNCHING RX_CONTROLLER ***'

try:
    print 'Trying it with the camera'
    import rx_controller_camera
except:
    print 'No camera, just launching battery monitor'
    import rx_controller_noCamera
