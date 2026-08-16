import copy
import numpy as np
import open3d as o3d
import time
import sys
from pathlib import Path

folder = Path(sys.argv[1])

SOURCE_FILE = folder / "0000000000.txt"
TARGET_FILE = folder / "0000000001.txt"

print("SOURCE:", SOURCE_FILE)
print("TARGET:", TARGET_FILE)

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)

    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])

    source_temp.transform(transformation)

    o3d.visualization.draw_geometries(
        [
            source_temp,
            target_temp
        ],
        zoom=0.4559,
        front=[0.6452, -0.3036, -0.7011],
        lookat=[1.9892, 2.0208, 1.8945],
        up=[-0.2779, -0.9482, 0.1556]
    )

def load_point_cloud(filename):
    data = np.loadtxt(filename)
    points = data[:, :3]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd

def preprocess_point_cloud(pcd, voxel_size):
    print(
        ":: Downsample with a voxel size %.3f."
        % voxel_size
    )
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    print(
        ":: Estimate normal with search radius %.3f."
        % radius_normal
    )
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_normal,
            max_nn=30
        )
    )
    radius_feature = voxel_size * 5
    print(
        ":: Compute FPFH feature with search radius %.3f."
        % radius_feature
    )
    pcd_fpfh = (
        o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=radius_feature,
                max_nn=100
            )
        )
    )
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size):
    print(":: Load two point clouds.")
    source = load_point_cloud(SOURCE_FILE)
    target = load_point_cloud(TARGET_FILE)
    print(
        "SOURCE points:",
        len(source.points)
    )
    print(
        "TARGET points:",
        len(target.points)
    )
    source_down, source_fpfh = preprocess_point_cloud(
        source,
        voxel_size
    )
    target_down, target_fpfh = preprocess_point_cloud(
        target,
        voxel_size
    )
    return (
        source,
        target,
        source_down,
        target_down,
        source_fpfh,
        target_fpfh
    )

def execute_global_registration(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        voxel_size):

    distance_threshold = voxel_size * 1.5
    print(
        ":: RANSAC registration on downsampled point clouds."
    )
    print(
        "   Since the downsampling voxel size is %.3f,"
        % voxel_size
    )
    print(
        "   we use a liberal distance threshold %.3f."
        % distance_threshold
    )
    result = (
        o3d.pipelines.registration
        .registration_ransac_based_on_feature_matching(

            source_down,
            target_down,

            source_fpfh,
            target_fpfh,

            mutual_filter=False,

            max_correspondence_distance=
                distance_threshold,

            estimation_method=
                o3d.pipelines.registration
                .TransformationEstimationPointToPoint(
                    False
                ),

            ransac_n=4,

            checkers=[

                o3d.pipelines.registration
                .CorrespondenceCheckerBasedOnEdgeLength(
                    0.9
                ),

                o3d.pipelines.registration
                .CorrespondenceCheckerBasedOnDistance(
                    distance_threshold
                )
            ],

            criteria=
                o3d.pipelines.registration
                .RANSACConvergenceCriteria(
                    4000000,
                    500
                )
        )
    )

    return result

def refine_registration(
        source,
        target,
        result_ransac,
        voxel_size):

    distance_threshold = voxel_size * 0.4
    print(
        ":: Point-to-plane ICP registration is applied "
        "on original point clouds."
    )
    print(
        "   distance threshold %.3f."
        % distance_threshold
    )
    source.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 2,
            max_nn=30
        )
    )
    target.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 2,
            max_nn=30
        )
    )
    result = (
        o3d.pipelines.registration
        .registration_icp(

            source,

            target,

            distance_threshold,

            result_ransac.transformation,

            o3d.pipelines.registration
            .TransformationEstimationPointToPlane()
        )
    )

    return result

def execute_fast_global_registration(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        voxel_size):

    distance_threshold = voxel_size * 0.5
    print(
        ":: Apply fast global registration "
        "with distance threshold %.3f"
        % distance_threshold
    )
    result = (
        o3d.pipelines.registration
        .registration_fgr_based_on_feature_matching(

            source_down,

            target_down,

            source_fpfh,

            target_fpfh,

            o3d.pipelines.registration
            .FastGlobalRegistrationOption(
                maximum_correspondence_distance=
                    distance_threshold
            )
        )
    )
    return result

voxel_size = 0.05

source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size)

print()
print("========================================")
print("RANSAC GLOBAL REGISTRATION")
print("========================================")

start = time.time()

result_ransac = execute_global_registration(
    source_down,
    target_down,
    source_fpfh,
    target_fpfh,
    voxel_size
)

ransac_time = time.time() - start

print()
print(result_ransac)

print(
    "RANSAC Fitness: %.10f"
    % result_ransac.fitness
)

print(
    "RANSAC Inlier RMSE: %.10f"
    % result_ransac.inlier_rmse
)

print(
    "RANSAC Correspondences: %d"
    % len(result_ransac.correspondence_set)
)

print(
    "RANSAC Time: %.6f sec"
    % ransac_time
)

print()
print("========================================")
print("LOCAL REFINEMENT / ICP")
print("========================================")

start = time.time()

result_icp = refine_registration(
    source,
    target,
    result_ransac,
    voxel_size
)

icp_time = time.time() - start


print()
print(result_icp)

print(
    "ICP Fitness: %.10f"
    % result_icp.fitness
)

print(
    "ICP Inlier RMSE: %.10f"
    % result_icp.inlier_rmse
)

print(
    "ICP Correspondences: %d"
    % len(result_icp.correspondence_set)
)

print(
    "ICP Time: %.6f sec"
    % icp_time
)

print()
print("========================================")
print("FAST GLOBAL REGISTRATION")
print("========================================")

start = time.time()

result_fast = execute_fast_global_registration(
    source_down,
    target_down,
    source_fpfh,
    target_fpfh,
    voxel_size
)

fast_time = time.time() - start

print()
print(result_fast)

print(
    "FAST Fitness: %.10f"
    % result_fast.fitness
)

print(
    "FAST Inlier RMSE: %.10f"
    % result_fast.inlier_rmse
)

print(
    "FAST Correspondences: %d"
    % len(result_fast.correspondence_set)
)

print(
    "FAST Time: %.6f sec"
    % fast_time
)

print()
print("========================================")
print("FINAL COMPARISON")
print("========================================")

print(
    "RANSAC Fitness : %.10f"
    % result_ransac.fitness
)

print(
    "ICP Fitness    : %.10f"
    % result_icp.fitness
)

print(
    "FAST Fitness   : %.10f"
    % result_fast.fitness
)

print()

print(
    "RANSAC Time    : %.6f sec"
    % ransac_time
)

print(
    "ICP Time       : %.6f sec"
    % icp_time
)

print(
    "FAST Time      : %.6f sec"
    % fast_time
)

print()
print("========================================")
print("VISUALIZATION")
print("========================================")

print()
print("RANSAC visualization...")
draw_registration_result(
    source_down,
    target_down,
    result_ransac.transformation
)

print()
print("ICP visualization...")
draw_registration_result(
    source,
    target,
    result_icp.transformation
)

print()
print("FAST GLOBAL REGISTRATION visualization...")
draw_registration_result(
    source_down,
    target_down,
    result_fast.transformation
)