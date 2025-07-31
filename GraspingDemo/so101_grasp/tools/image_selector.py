import numpy as np
from typing import Optional
import matplotlib
import matplotlib.pyplot as plt



class ImgClick:
    def __init__(self, img: np.ndarray, os: str = "MAC"):
        """
        Initializes the PointcloudCropPipeline.

        Args:
            img (np.array): The image to view/ click on
        """
        self.x: Optional[int] = None
        self.y: Optional[int] = None
        self.img = img
        if os == "MAC":
            matplotlib.use("MacOSX")
        elif os == "LINUX":
            matplotlib.use("TkAgg")
        else:
            raise ValueError(f"Invalid OS: {os}")

    def on_click(self, event: matplotlib.backend_bases.MouseEvent) -> None:
        """
        Handle mouse click events on the image and save the x,y coordinates.

        Args:
            event: The mouse click event containing coordinates
        """
        if event.xdata is not None and event.ydata is not None:
            self.x, self.y = int(event.xdata), int(event.ydata)
            print(f"Pixel Position (x,y): ({self.x}, {self.y})")

    def get_point_from_img(self) -> None:
        """
        Displays the image and allows the user to click on the image to get the x,y coordinates.
        Runs callback to save x,y coordinates.
        """
        # Handle different image formats
        if len(self.img.shape) == 3:
            # Image is already in HxWxC format
            img = self.img
        else:
            # Try to reshape - detect dimensions automatically
            total_pixels = self.img.size
            if total_pixels == 480 * 640 * 3:
                img = self.img.reshape(480, 640, 3)
            elif total_pixels == 720 * 1280 * 3:
                img = self.img.reshape(720, 1280, 3)
            else:
                # Fallback: assume it's the total size divided by 3 channels
                height_width = int(np.sqrt(total_pixels // 3))
                if height_width * height_width * 3 == total_pixels:
                    img = self.img.reshape(height_width, height_width, 3)
                else:
                    raise ValueError(f"Cannot determine image dimensions from size {total_pixels}")
        
        # Ensure image is in correct format for display
        if img.dtype != np.uint8:
            if img.max() <= 1.0:
                # Normalized image (0-1 range)
                img = (img * 255).astype(np.uint8)
            else:
                # Already in 0-255 range
                img = img.astype(np.uint8)
        
        fig, ax = plt.subplots()
        ax.imshow(img)
        ax.set_title("Click on the marker center, then close the window")
        fig.canvas.mpl_connect(
            "button_press_event",
            lambda event: self.on_click(
                event,
            ),
        )
        plt.show()

    def run(self):
        """
        Returns a cropped pointcloud based on the object the user clicks on.

        Returns:
            x (int): x coordinate of click on img
            y (int): y coordinate of click on img
        """
        self.get_point_from_img()
        return self.x, self.y