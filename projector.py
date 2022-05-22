import numpy as np
from enum import Enum

class Projector(object):
    """
    """
    class Coloring(Enum):
        DEPTH = 1
        INTENSITY = 2

    def __init__(self):
        pass

    @staticmethod
    def cart2hom(pts_3d):
        """ Input: nx3 points in Cartesian
            Oupput: nx4 points in Homogeneous by pending 1
        """
        n = pts_3d.shape[0]
        pts_3d_hom = np.hstack((pts_3d, np.ones((n, 1))))
        return pts_3d_hom
    
    @staticmethod
    def inverse_rigid_trans(Tr):
        """ Inverse a rigid body transform matrix (3x4 as [R|t])
            [R'|-R't; 0|1]
        """
        inv_Tr = np.zeros_like(Tr)  # 3x4
        inv_Tr[0:3, 0:3] = np.transpose(Tr[0:3, 0:3])
        inv_Tr[0:3, 3] = np.dot(-np.transpose(Tr[0:3, 0:3]), Tr[0:3, 3])
        return inv_Tr
    
    @staticmethod
    def view_points(points: np.ndarray, view: np.ndarray, normalize: bool) -> np.ndarray:
        """
        This is a helper class that maps 3d points to a 2d plane. It can be used to implement both perspective and
        orthographic projections. It first applies the dot product between the points and the view. By convention,
        the view should be such that the data is projected onto the first 2 axis. It then optionally applies a
        normalization along the third dimension.

        For a perspective projection the view should be a 3x3 camera matrix, and normalize=True
        For an orthographic projection with translation the view is a 3x4 matrix and normalize=False
        For an orthographic projection without translation the view is a 3x3 matrix (optionally 3x4 with last columns
        all zeros) and normalize=False

        :param points: <np.float32: 3, n> Matrix of points, where each point (x, y, z) is along each column.
        :param view: <np.float32: n, n>. Defines an arbitrary projection (n <= 4).
            The projection should be such that the corners are projected onto the first 2 axis.
        :param normalize: Whether to normalize the remaining coordinate (along the third axis).
        :return: <np.float32: 3, n>. Mapped point. If normalize=False, the third coordinate is the height.
        """

        assert view.shape[0] <= 4
        assert view.shape[1] <= 4
        assert points.shape[0] == 3

        viewpad = np.eye(4)
        viewpad[:view.shape[0], :view.shape[1]] = view

        nbr_points = points.shape[1]

        # Do operation in homogenous coordinates.
        points = np.concatenate((points, np.ones((1, nbr_points))))
        points = np.dot(viewpad, points)
        points = points[:3, :]

        if normalize:
            points = points / points[2:3, :].repeat(3, 0).reshape(3, nbr_points)
            # points[:2] /= points[2,:]

        return points

    @staticmethod
    def map_pointcloud_to_image(img, pc_points, extrinsic, intrinsic, min_dist=0.0, coloring=Coloring.DEPTH):
        """
            im: PIL image object
            points: (?, 4), fields: x,y,z,intensity
                   or (?, 3), fields: x,y,z
        """
        if pc_points.shape[1] == 3:
            coloring=Projector.Coloring.DEPTH

        pcd_arr = Projector.cart2hom(pc_points[:, :3]) # n,4
        pts_in_cam = np.matmul(pcd_arr, extrinsic)
        depths = pts_in_cam[:, 2]
        
        # Take the actual picture (matrix multiplication with camera-matrix + renormalization).
        points = Projector.view_points(pts_in_cam.T[:3, :], intrinsic, normalize=True)
        mask = np.ones(depths.shape[0], dtype=bool)
        mask = np.logical_and(mask, depths > min_dist)
        mask = np.logical_and(mask, points[0, :] > 1)
        mask = np.logical_and(mask, points[0, :] < img.size[0] - 1)
        mask = np.logical_and(mask, points[1, :] > 1)
        mask = np.logical_and(mask, points[1, :] < img.size[1] - 1)
        points = points[:, mask]
        
        if coloring == Projector.Coloring.DEPTH:
            coloring = depths[mask]
        else:
            # Retrieve the color from the intensities.
            # Performs arbitary scaling to achieve more visually pleasing results.
            intensities = pc_points[:, 3] # intensity field
            intensities = (intensities - np.min(intensities)) / (np.max(intensities) - np.min(intensities))
            intensities = intensities ** 0.1
            intensities = np.maximum(0, intensities - 0.5)
            coloring = intensities[mask]

        return points, coloring