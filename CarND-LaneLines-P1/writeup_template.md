# **Finding Lane Lines on the Road** 

---

**The goal of this project is to make a pipeline that finds lane lines on the road**

---

### 1. Pipeline description.

The pipeline is implemented in process_image() function and consists of the following steps:
1. Grayscale transform.
2. Gaussian smoothing.
3. Canny edge detection.
4. Image masking (mask region with a road).
5. Hough transform to detect possible lane lines.
6. Detect and draw lane lines from the results of Hough transform.

Step 6 above is implemented in draw_lines() function and consists of the following sub-steps:
1. Sort lines detected by Hough transform to 2 lists (left and right) depending on an angle.
2. Fit 1st order polynomials for these left and right lists of lines.
3. Construct left and right lane lines from these fits.
4. Draw detected lane lines.

<img src="./test_images_input/solidWhiteCurve.jpg" width="280">

### 2. Potential shortcomings and possible improvements with the current pipeline

There are few shortcomings with the currect approach:
1. Detected lane lines are always straight, which is obviously wrong when road turns left or right. These could be probably improved by fitting higher order polynomials in step 6 of the pipeline.
2. Depeding on quality of an individual image, detected lane lines could be altered by image artifacts of noise. These however could be improved when image processing is done continously (i.e. video stream) by applying some filtering based on previos frames.
