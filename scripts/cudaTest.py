import cv2
import numpy as np

# Create a dummy image
img = np.random.randint(0, 255, (1000, 1000), dtype=np.uint8)

# Create CUDA matrix
gpu_mat = cv2.cuda_GpuMat()
gpu_mat.upload(img)

# Try a CUDA operation (like Gaussian blur)
gpu_blur = cv2.cuda.createGaussianFilter(cv2.CV_8UC1, cv2.CV_8UC1, (7, 7), 0)
gpu_result = gpu_blur.apply(gpu_mat)

# If no errors occur, CUDA is working
print("CUDA test successful!")
