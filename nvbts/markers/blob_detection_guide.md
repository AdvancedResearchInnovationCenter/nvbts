replace the path in blob.py with the path to your image and run blob.py. Play around with the parameters until you're satisfied. If not, try the following tips

1. try blob_tuner on your image. if it does not work, try the following steps.
2. decrease params.minDistBetweenBlobs
3. increase the contrast of the image by the createCLAHE method
4. turn off circularity and intertia filtering
5. morphological operators of various kernel sizes
