# Camera configuration for ThinkGrasp
# Update these parameters to match your camera's intrinsic parameters

class CameraConfig:
    """Camera configuration parameters"""
    WIDTH = 640
    HEIGHT = 480
    FX = 382.8567  # Focal length x
    FY = 382.4391  # Focal length y
    CX = 331.3490  # Principal point x
    CY = 247.1126  # Principal point y
    SCALE = 1000.0  # Depth scale factor

    # Image processing region - UPDATE FOR YOUR CAMERA RESOLUTION
    XMIN = 0
    YMIN = 0
    XMAX = 480
    YMAX = 640
    # Camera intrinsic parameters - iPhone camera configuration
    # WIDTH = 720
    # HEIGHT = 960
    # FX = 1386.71533203125  # Focal length x
    # FY = 1386.71533203125  # Focal length y
    # CX = 718.263916015625  # Principal point x
    # CY = 954.7833862304688  # Principal point y
    # SCALE = 1000.0  # Depth scale factor
    #
    # # Image processing region - iPhone camera resolution
    # XMIN = 0
    # YMIN = 0
    # XMAX = 960
    # YMAX = 720
    
    @classmethod
    def get_camera_info(cls):
        """Returns camera parameters in CameraInfo format"""
        from models.FGC_graspnet.utils.data_utils import CameraInfo
        return CameraInfo(
            width=cls.WIDTH,
            height=cls.HEIGHT,
            fx=cls.FX,
            fy=cls.FY,
            cx=cls.CX,
            cy=cls.CY,
            scale=cls.SCALE
        )
    
    @classmethod
    def get_processing_region(cls):
        """Returns image processing region coordinates"""
        return cls.XMIN, cls.YMIN, cls.XMAX, cls.YMAX