from setuptools import setup, find_packages

setup(
    name="cube_stacking",
    version="0.0.1",
    description="MuJoCo cube-stacking environment",
    packages=find_packages(),
    install_requires=[
        "gymnasium>=0.29",
        "mujoco>=3.0.2",          # physics engine
        "numpy>=1.24",            # math / array ops
        "pillow>=10.0",           # PNG/JPEG save  (Image.fromarray)
        "imageio[ffmpeg]>=2.34",  # MP4 writer + bundled FFmpeg
        "opencv-python>=4.8",     # cv2 (only if you actually use it)
    ],
    package_data={
        "cube_stacking": ["trs_so_arm100/*"],   # ship XML, meshes, textures
    },
    python_requires=">=3.8",
)
