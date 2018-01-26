# CornerCluster
This project detects harris corner of the pattern, and cluster these corners based on the density，and then process these clustered pattern to identify which pattern it is and the object is find the feature points of these pattern and using pop to calculate the extrinsic parameters of the camera. 


Recently I am working on calculating the transformation matrix of one camera relative to the other by using an easy-to-identitfied pattern(with complex corner points) attached to the target camera. Recognizing the  pattern is easy but when it comes to detect object which is far from the camera, the task becomes harder. Because the complex environment interferes with the detection and both feature-based and deep learning based algorithm perform not well when detecting object which is far, my purpose is to design a workflow to solve this problem.

To solve this problem I have tried different ways both to detect the pattern’s key points and to calculate the transformation.

I first tried to use the QR code as an object to detect, because the pattern of QR code is well organized and we can also embed information into it, but after implementing it I found that the algorithm performs not good enough even in the distance more than one meter away. And using a perspective model to calculate the transformation needs to constrain the sides of QR code must be perpendicular to the ground, which seems impossible for the 3-D environment. The following link shows the process:

https://v.qq.com/x/page/h0561m4qjh2.html



Then I tried to use chessboard to solve my problem, because the pattern of chessboard is also well organized and I can use the camera calibration to determine the intrinsic and extrinsic parameters of the camera. Next I located 4 points in a specific pattern, because once I find 4 points’ coordinates of the pattern in an 2-D image, I can associate these 2-D coordinates with 3-D coordinates we measured and camera parameters to solve a Perspective 4 Points problem to find the rotated vector and transformation matrix. Unfortunately this method can not solve the distance struggle. The following link shows the process:

1. Using camera calibration to determine the camera intrinsic parameter matrix. And store them.

2. Locating 4 points in the image and calculating the rotation vector and transformation    
matrix.

 The following link shows how it works:

https://v.qq.com/x/page/g0561v0ikec.html

https://v.qq.com/x/page/l0561kxt26m.html



Moreover, I come up with an idea that we have to use the most easily found features that we can detect at a far distance, so I decided to use corner points, the workflow is showing here.
1.  I used the opencv library function, but the effect is not very good, so I implemented my own corner point detection by applying the differential mask in x and y directions and the gaussian mask which is slightly different from Harris method.

2. The next step is to use Density-based clustering algorithm to cluster these points, because our pattern has the dense corner points, so it is easy to cluster them into one cluster.

3. We scan the clusters and drop those that do not meet our criteria with regard to the shape of the pattern. 

4. We chose the 4 Points to do the same job as the last method and calculate the relative position and posture of another camera. 

 The following link shows the how it works:
(The green points represent the noise detected by my algorithm, and the points in different color represent different cluster)

https://v.qq.com/x/page/o0561prp6eg.html

I found that the complexity of our pattern has relationship with the detection which is that if it is too complicated the corner points are harder to detect at a far distance, but if it is not complicated enough we can easily be confused with the environment noise, so I want to still focus on this topic and look for a good way to generate a pattern to be easily recognized. This process is just the opposite of the detection process, we think of the problem in a reverse thinking which is novel and there is a great application.
