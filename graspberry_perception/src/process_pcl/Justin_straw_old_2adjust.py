# pip install open3d-python==0.7.0.0

# adjust the code to include self in the methods of the class generate_cloud()
import sys
import rospy
import lib_cloud_conversion_between_Open3D_and_ROS as cld2ros


class KineticImportsFix:
    def __init__(self, kinetic_dist_packages="/opt/ros/kinetic/lib/python2.7/dist-packages"):
        self.kinetic_dist_packages = kinetic_dist_packages

    def __enter__(self):
        sys.path.remove(self.kinetic_dist_packages)

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.path.append(self.kinetic_dist_packages)


with KineticImportsFix(): 
    import open3d as opend
    import numpy as np
    import png
    import matplotlib.pyplot as plt
    import cv2
    from PIL import Image
    import struct


class generate_cloud():

    def vectorized_pixe_to_point(depth_raw,color_raw,annotation,color_principal_point,color_focal_length):  # get point cloud from rgb and depth image, "color_principal_point,color_focal_length" are camera intrinsic parameters, 'annotation' is label2.png
        depth_raw = depth_raw/1000  # /1000 to get the depth of each pixel in meters, depth_raw shape is (720,1280)
        depth_raw2 = np.expand_dims(depth_raw,2)  # np.expand_dims(a,axis), 2 is along axis z, dim will be 1 along 3rd axis, so (720,1280,1)
        depth_raw3 = np.concatenate([depth_raw2,depth_raw2],2) # 2 copies of depth_raw2 are concatenated one after the other along the 3rd dim , so shape is (720,1280,2)
        x =  np.expand_dims(np.repeat(np.expand_dims(np.arange(depth_raw.shape[1]),0),depth_raw.shape[0],0),2) # np.repeat(array, nb of rep, axis),  np.arange(depth_raw.shape[1]) returns : array([0,1,2...1279]), expand along axis 0 gives shape of ( 1,1280) so array([[0,1,2....1279]]) then will be copied 720times along axis 0, x.shape gives (720,1280,1), it assigns the column nb to each pixel ... means x values
        y = np.arange(depth_raw.shape[0])  # array([0,1,2...719])
        y = np.expand_dims(y,1) # array([[0], [1], ...[719]]) , y.shape is (720,1)
        y = np.expand_dims(np.repeat(y,depth_raw.shape[1],1),2) #y.shape is (720,1280,1), assigns row nb to each pixel , means y value
        z = np.concatenate([x,y],2) # z.shape (720,1280,2)
        points = (z - color_principal_point) * depth_raw3 / color_focal_length  # multiply x and y values of each pixel by the depth value, considering intrinsic params, points.shape(720,1280,2)
        points = np.concatenate([points, depth_raw2],2) # add depth array as 3rd layer in points 3D array, points.shape (720,1280,3), 1st layer is x, 2nd is y and 3rd is depth
        points =  np.reshape(points,(points.shape[0]*points.shape[1],points.shape[2])) # got 3 columns , in 1st we have all x values aligned in 1 col, in 2nd we have y values, and 3rd we have depth values
        points = points * np.array([1,-1.0,-1.0])  # np.array([1,-1.0,-1.0]) ?
        colors = np.reshape(color_raw,(color_raw.shape[0]*color_raw.shape[1],color_raw.shape[2])) # 720*1280 ... will get 1 colomn of rgb value for each pixel
        annotation = np.reshape(annotation,(annotation.shape[0]*annotation.shape[1]))
        annotation = annotation[np.nonzero(points[:,2])] # indices of pixels with nonzero depth
        colors = colors [np.nonzero(points[:,2])]
        points = points[np.nonzero(points[:,2])]
        # points2 = points[(points[:,2]>-0.50).nonzero()]
        # colors = colors[(points[:,2]>-0.50).nonzero()]
        # annotation = annotation[(points[:,2]>-0.50).nonzero()]
        return (points,colors,annotation )


    def display_pc(annotation,pcd,objects):
        print("Displaying")
        annotation =  np.expand_dims(annotation,1) # shape is (720*1280 , 1) means array([[], /newline [], \newline [].....])
        annotation = np.concatenate([annotation,annotation,annotation],1)  # same size as points (720*1280,3)
        pcd2 =  opend.PointCloud() # create a PointCloud object 
        pcd2.points = pcd.points  # after loading the pointcloud_color.ply
        pcd2.colors = opend.Vector3dVector(annotation)  # from np array to open3d.Vector3dVector (pcd2 will have annotated colors)
        pcd4 =  opend.PointCloud()
        pcd4.points = opend.Vector3dVector(np.asarray(pcd.points) )
        pcd4.colors = pcd.colors # after loading the pointcloud_color.ply (pcd4 wil have rgb colors)
        objects.append(pcd2)
        objects.append(pcd4)
        opend.draw_geometries(objects)


    # def get_boxe_from_p(boxes): # each box is defined by 2 3D points
    #     vertices = []
    #     lines = np.array([[0,0]])
    #     i = 0
    #     for b in boxes:  # for each box
    #         first_p = b[1]
    #         second_p = b[0]
    #         width = first_p[0] - second_p[0]
    #         height = first_p[1] - second_p[1]
    #         depth = first_p[2] - second_p[2]
    #         vertices.append(first_p) # top front right
    #         vertices.append(first_p-[width,0,0]) # top front left
    #         vertices.append(first_p-[width,height,0]) # bottom front left
    #         vertices.append(first_p-[0,height,0]) # botton front right
    #         vertices.append(second_p) # bottom back left
    #         vertices.append(first_p-[width,0,depth]) # top back left
    #         vertices.append(first_p-[0,height,depth]) # bottom back right
    #         vertices.append(first_p-[0,0,depth]) # top back right
    #         edges = [[0+(i*8),1+(i*8)],[1+(i*8),2+(i*8)],[2+(i*8),3+(i*8)],[3+(i*8),0+(i*8)]
    #                 ,[4+(i*8),5+(i*8)],[4+(i*8),6+(i*8)],[6+(i*8),7+(i*8)],[7+(i*8),5+(i*8)]
    #                 ,[0+(i*8),7+(i*8)],[1+(i*8),5+(i*8)],[4+(i*8),2+(i*8)],[3+(i*8),6+(i*8)]]
    #         lines = np.concatenate([lines,edges],axis = 0)
    #         i = i+1 # next box
    #     line_set = opend.geometry.LineSet()
    #     line_set.points = opend.utility.Vector3dVector(vertices) # 3D doubles
    #     line_set.lines = opend.utility.Vector2iVector(lines[1:]) # integers
    #     line_set.colors = opend.utility.Vector3dVector([[1, 0, 0] for i in range(lines[1:].shape[0])])
    #         # i = i + 1
    #     return line_set


    def get_boxe_from_p(boxes, i): # each box is defined by 2 3D points
        vertices = []
        lines = np.array([[0,0]])
        b = boxes
        #for b in boxes:  # for each box
        first_p = b[1]
        second_p = b[0]
        width = first_p[0] - second_p[0]
        height = first_p[1] - second_p[1]
        depth = first_p[2] - second_p[2]
        vertices.append(first_p) # top front right
        vertices.append(first_p-[width,0,0]) # top front left
        vertices.append(first_p-[width,height,0]) # bottom front left
        vertices.append(first_p-[0,height,0]) # botton front right
        vertices.append(second_p) # bottom back left
        vertices.append(first_p-[width,0,depth]) # top back left
        vertices.append(first_p-[0,height,depth]) # bottom back right
        vertices.append(first_p-[0,0,depth]) # top back right
        edges = [[0+(i*8),1+(i*8)],[1+(i*8),2+(i*8)],[2+(i*8),3+(i*8)],[3+(i*8),0+(i*8)]
                ,[4+(i*8),5+(i*8)],[4+(i*8),6+(i*8)],[6+(i*8),7+(i*8)],[7+(i*8),5+(i*8)]
                ,[0+(i*8),7+(i*8)],[1+(i*8),5+(i*8)],[4+(i*8),2+(i*8)],[3+(i*8),6+(i*8)]]
        lines = np.concatenate([lines,edges],axis = 0)
            #i = i+1 # next box
        line_set = opend.geometry.LineSet()
        line_set.points = opend.utility.Vector3dVector(vertices) # 3D doubles
        line_set.lines = opend.utility.Vector2iVector(lines[1:]) # integers
        line_set.colors = opend.utility.Vector3dVector([[1, 0, 0] for i in range(lines[1:].shape[0])])
            # i = i + 1
        return line_set

    #################################################################################################################
    def __init__(self,):    
        colorpp =  np.array([6.39151982e+02,3.89859506e+02])
        colorff =  np.array([1.05139286e+03,1.05053887e+03])
        depth_raw = png.Reader("./annotated_straw/depth.png")
        _,_,pngData,_=depth_raw.asDirect() # Like the read() method this method returns a 4-tuple:(width, height, pixel values, meta)
        depth_raw = np.vstack(list(pngData))
        # plt.imshow(depth_raw)
        # plt.show()
        color = np.asarray(cv2.imread("./annotated_straw/rgb.png"))/255
        # plt.imshow(color)
        # plt.show()
        annotation = np.asarray(Image.open("./annotated_straw/label2.png"))
        # plt.imshow(annotation)
        # plt.show()
        points,colors,annotation = vectorized_pixe_to_point(depth_raw,color,annotation,colorpp,colorff)
        classa = {} # a dictionary, to store how many classes are in the image annotated
        for k,a in enumerate(annotation): #k is the element index and a is the element value in annotation array
            if(a!=0): # everything different than the background
                if(not a in classa.keys()):
                    classa[a] = [k] # a is exactly a key stored in the dict , to which we assign values k: the indices of array annotation that have value a, without []
                else:
                    val  = classa[a] # val will b the value existing in the key a 
                    val.append(k)  # append a value to the key
                    classa[a] = val
        bb_list_int = []
        centers = []
        objects = []## list of objects to display
        objects_annot = []
        print('keys=',classa.keys())

        i = 0
        ## we compute and process the bounding boxes of for the point cloud
        for k in classa.keys(): # classa.keys() = nb of straw, k is the key 
            centers.append(points[classa[k]].mean(axis=0))  # classa[k] return the value of the key k .. which contains the indices of array 'points' that have key k as annotation
            min = points[classa[k]].min(axis=0,keepdims=True) #returns [minx miny minz]
            # if k == 2:
            # 	print('minimum = ', min)
            # 	print('minimum shape= ', min.shape)
            max = points[classa[k]].max(axis=0,keepdims=True) #returns [maxx maxy maxz]
            median = np.median(points[classa[k]],axis=0,keepdims=True)
            ##BECAUSE ON  HOW THE DATA IS ANNOTATED WE NEED TO FIX SOME BOUNDING BOXES with excessive size (for pre processint creation of mask)
            if(min[0][2]<median[0][2]-0.03):  # min[0][2] is minz
                min = [np.array([min[0][0],min[0][1],median[0][2]-0.03])]
            if(max[0][2]>median[0][2]+0.03):
                max = [np.array([max[0][0],max[0][1],median[0][2]+0.03])]
            print('min = ', min[0])
            bb_list_int.append([min[0],max[0]])
            objects.append(get_boxe_from_p([min[0],max[0]],i))
            objects_annot.append(k)
            i = i + 1
        #print('bb_list_int shape = ',bb_list_int)

        #objects.append(get_boxe_from_p(bb_list_int))
        ## lets add the bounding boxes to the list of stuff to display


        for c in centers:
            mesh_sphere = opend.create_mesh_sphere(radius=0.002)
            mesh_sphere.compute_vertex_normals()
            mesh_sphere.paint_uniform_color([0.1, 0.1, 0.7])
            print('c', c)
            mesh_sphere.translate(c)
            objects.append(mesh_sphere)

        for l in range(0, len(classa.keys())):
        	objects_annot.append(objects_annot[l])


        print('obj annot=', objects_annot)
        #print(objects_annot.index(1)) # returns only first found index
        indices = [i for i, x in enumerate(objects_annot) if x == 1]
        print(indices)

        # we transform the annotation from (N,C) to (N,3*C) to downsample and keep them
        annotation =  np.expand_dims(annotation,1)
        annotation = np.concatenate([annotation,annotation,annotation],1)

        ## we downsample once to keep the colors
        pcd = opend.PointCloud()
        pcd.points = opend.Vector3dVector(points)
        pcd.colors = opend.Vector3dVector(colors)  # from np array to opend.Vector3dVector form 
        pcd,_ = opend.voxel_down_sample_and_trace(pcd, voxel_size = 0.0001
                                                    ,min_bound= points.min(axis=0,keepdims=True)[0]  # xmin 
                                                    ,max_bound=points.max(axis=0,keepdims=True)[0]  # x max
                                                    ,approximate_class=False )
        ## we downsample a second time to keep the annotation
        pcdb = opend.PointCloud()
        pcdb.points = opend.Vector3dVector(points)
        pcdb.colors = opend.Vector3dVector(annotation)
        pcd3,_ = opend.voxel_down_sample_and_trace(pcdb, voxel_size = 0.0001
                                                     ,min_bound= points.min(axis=0,keepdims=True)[0]
                                                     ,max_bound=points.max(axis=0,keepdims=True)[0]
                                                     ,approximate_class=True )
        ##transforming the annotations to the class we computed previously
        annotation = np.asarray(pcd3.colors)[:,0]  # [:,1] and [:,2] are copies od label in [:,0]
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)



        for lab in range(0,len(classa.keys())+1): # len(classa.keys()) = 19 without bkgd so lab = 0, ... 18
            # annotation[ np.where(annotation == float(list(classa.keys()) [lab])) ] = lab+1
            str7p = points[annotation==lab] # plot each downsampled straw

            # Find outliers in the subsampled point cloud straw
            if lab != 0:
        	    min_ss = str7p.min(axis=0,keepdims=True)
        	    print('m',min_ss[0][2])
        	    max_ss = str7p.max(axis=0,keepdims=True)
        	    median_ss = np.median(str7p,axis=0,keepdims=True)
        	    bottom_ss = str7p[str7p[:,2] == max_ss[0][2]]
        	    if (min_ss[0][2]<median_ss[0][2]-0.03):
        	    	min_ss = [np.array([min_ss[0][0],min_ss[0][1],median_ss[0][2]-0.03])]
        	    	#bottom_ss = np.array([bottom_ss[0][0], bottom_ss[0][1], median_ss[0][2]-0.02])
        	    if (max_ss[0][2]>median_ss[0][2]+0.03):
        	    	max_ss = [np.array([max_ss[0][0],max_ss[0][1],median_ss[0][2]+0.03])]
        	    	bottom_ss = np.array([bottom_ss[0][0], bottom_ss[0][1], median_ss[0][2]+0.03])


            print('shape',str7p[:,2].shape)
            str7c = colors[annotation==lab]
            pcdb = opend.PointCloud()
            pcdb.points = opend.Vector3dVector(str7p)
            pcdb.colors = opend.Vector3dVector(str7c)
            ind = [i for i, x in enumerate(objects_annot) if x == lab]
            print('ind=', ind)
            if lab != 0:
            	# retrieve bottom point of straw
            	peak_str_ind = np.where(str7p[:,2] == np.amin(str7p[:,2]))
            	peak_str_ind = np.squeeze(peak_str_ind)
            	#print('peak_ind=', peak_str_ind)
            	#print('pek', str7p[peak_str_ind])
            	#print('shape peak',str7p[peak_str_ind])
            	sh = str7p[peak_str_ind].shape
            	sh_ss = bottom_ss.shape
            	l = len(sh)
            	l_ss = len(sh_ss)
            	str7p_r = str7p[peak_str_ind].shape[0]
            	if (l == 1):
            		bottom_mean = str7p[peak_str_ind]
            	else:
            		bottom_mean = np.mean(str7p[peak_str_ind], axis= 0)
            	if (l_ss == 1):
            		bottom_mean_ss = bottom_ss
            	else:
            		bottom_mean_ss = np.mean(bottom_ss, axis= 0)
            	print('bottom_mean', bottom_mean)
            	sphere_peak_str = opend.create_mesh_sphere(radius=0.002)
            	sphere_peak_str.compute_vertex_normals()
            	sphere_peak_str.paint_uniform_color([0.1, 0.7, 0.1])
            	sphere_peak_str.translate(bottom_mean_ss)
            	opend.draw_geometries([pcdb, objects[ind[0]], objects[ind[1]], sphere_peak_str], window_name='annotation = {}'.format(lab))
            else:
            	opend.draw_geometries([pcdb], window_name='annotation = {}'.format(lab))




## Calculate bottom point of each straw
## Calculate the distance between the COM and bottom and get orientation



## Insert a point cloud of type PointCloud2 in gazebo:
# Method 1: use octomap_server. It takes point clouds with tf and builds an octomap for you that you can request 
# Method 2: use OctoMap API, insertPointCloud() is the correct function. You only need to convert the point clouds ...
# ...into the OctoMap native type by using conversions.h in the package octomap_ros.


# publish the point cloud 
def cloud_publisher():
    rospy.loginfo("Started publishing one scene point clouds")
    cloud = generate_cloud()
    msg = cld2ros.convertCloudFromOpen3dToRos(cloud)
    cloud_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('cloud advertiser', anonymous=True)
    rate = rospy.Rate(50)
    cloud_pub = rospy.Publisher('/camera1_graspberry/PointCloud', PointCloud2, queue_size=10)
    cloud_publisher()
    while not rospy.is_shutdown():
        rate.sleep()



