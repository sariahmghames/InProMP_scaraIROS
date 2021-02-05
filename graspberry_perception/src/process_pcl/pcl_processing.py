import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from PIL import Image
#from bbox.utils import nms
#from bbox import BBox3D

if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud("./annotated_straw/pointcloud_annotated.ply")
    print(pcd)
    print((np.asarray(pcd.colors)[:,0]*19).astype(np.uint8))
    points  = np.asarray(pcd.points)  # Open3D.o3d.geometry.PointCloud to numpy array
    colorss = (np.asarray(pcd.colors)[:,0]*19).astype(np.uint8)  # each point has a color value represented as 1x3 where 2nd and 3rd element are copies of 1st and first correspond to straw nb
    print('colorss=',colorss[1000:2000])
    #colors = np.asarray(pcd.colors)
    o3d.visualization.draw_geometries([pcd])

    o3d.io.write_point_cloud("copy_pointcloud_annotated.pcd", pcd)

    print("Testing IO for images ...")
    img = o3d.io.read_image("./annotated_straw/rgb.png")
    img_labeled = Image.open('./annotated_straw/label2.png')
    plt.imshow(img_labeled)
    plt.show()

    ## Get all strawberries from the labeled point clouds
    str1_p = points[colorss==1]
    print('str1_p=',str1_p)
    str1_colors = colorss[colorss==1]
    #pcd.points = o3d.utility.Vector3dVector(str1_p) # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
    #pcd.colors = o3d.utility.Vector3dVector(str1_colors)

    str2_p = points[colorss==2.0]
    str2_colors = colorss[colorss==2.0]

    str3_p = points[colorss==3.0]
    str3_colors = colorss[colorss==3.0]

    str4_p = points[colorss==4.0]
    str4_colors = colorss[colorss==4.0]

    str5_p = points[colorss==5.0]
    str5_colors = colorss[colorss==5.0]

    str6_p = points[colorss==6.0]
    str6_colors = colorss[colorss==6.0]


    str7_p = points[colorss==7.0]
    str7_colors = colorss[colorss==7.0]

    str8_p = points[colorss==8.0]
    str8_colors = colorss[colorss==8.0]

    str9_p = points[colorss==9.0]
    str9_colors = colorss[colorss==9.0]


    str10_p = points[colorss==10.0]
    str10_colors = colorss[colorss==10.0]


    str11_p = points[colorss==11.0]
    str11_colors = colorss[colorss==11.0]


    str12_p = points[colorss==12.0]
    str12_colors = colorss[colorss==12.0]


    str13_p = points[colorss==13.0]
    str13_colors = colorss[colorss==13.0]


    str14_p = points[colorss==14.0]
    str14_colors = colorss[colorss==14.0]


    str15_p = points[colorss==15.0]
    str15_colors = colorss[colorss==15.0]


    str16_p = points[colorss==16.0]
    str16_colors = colorss[colorss==16.0]


    str17_p = points[colorss==17.0]
    str17_colors = colorss[colorss==17.0]


    str18_p = points[colorss==18.0]
    str18_colors = colorss[colorss==18.0]

    str19_p = points[colorss==19.0]
    str19_colors = colorss[colorss==19.0]


    ## bbox for 3D strawberries
    # straw1

    # str1_minx = np.amin(str1_p[:,0])
    # str1_maxx = np.amax(str1_p[:,0])
    # str1_miny = np.amin(str1_p[:,1])
    # str1_maxy = np.amax(str1_p[:,1])
    # str1_minz = np.amin(str1_p[:,2])
    # str1_maxz = np.amax(str1_p[:,2])
    # str1_pt_minx = str1_p(str1_p[:,0]== str1_minx)
    # str1_pt_maxx = str1_p(str1_p[:,0]== str1_maxx)
    # str1_pt_miny = str1_p(str1_p[:,1]== str1_miny)
    # str1_pt_maxy = str1_p(str1_p[:,1]== str1_maxy)
    # str1_pt_miny = str1_p(str1_p[:,2]== str1_minz)
    # str1_pt_maxy = str1_p(str1_p[:,2]== str1_maxz)
    # box_str1 = BBox3D([str1_pt_minx, str1_pt_max, ....])



































    ## Tutorial open3D

    # print("Testing IO for meshes ...")
    # mesh = o3d.io.read_triangle_mesh("../../TestData/knot.ply")
    # print(mesh)
    # o3d.io.write_triangle_mesh("copy_of_knot.ply", mesh)


    # print("Downsample the point cloud with a voxel of 0.05")
    # downpcd = pcd.voxel_down_sample(voxel_size=0.05)
    # o3d.visualization.draw_geometries([downpcd])

    # print("Recompute the normal of the downsampled point cloud")
    # downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
    #     radius=0.1, max_nn=30))
    # o3d.visualization.draw_geometries([downpcd])

    # print("Print a normal vector of the 0th point")
    # print(pcd.normals[0])
    # print("Print the normal vectors of the first 10 points")
    # print(np.asarray(pcd.normals)[:10, :])
    # print("Finish")

    # print("Load a polygon volume and use it to crop the original point cloud")
    # vol = o3d.visualization.read_selection_polygon_volume(
    #     "../../TestData/Crop/cropped.json")
    # chair = vol.crop_point_cloud(pcd)
    # o3d.visualization.draw_geometries([chair])
    # print("")

    # print("Paint chair")
    # chair.paint_uniform_color([1, 0.706, 0])
    # o3d.visualization.draw_geometries([chair])
    # print("")

