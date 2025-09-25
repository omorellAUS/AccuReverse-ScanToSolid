import pcl
import sys

def compute_hausdorff(original, simplified):
    orig = pcl.PointCloud.from_file(original)
    simp = pcl.PointCloud.from_file(simplified)
    kd = pcl.KdTreeFLANN(orig)
    distances = [kd.nearest_k_search_for_point(pt, 1)[1][0] for pt in simp.points]
    return max(distances)

if __name__ == "__main__":
    max_dev = compute_hausdorff(sys.argv[1], sys.argv[2])
    print(f"Max deviation: {max_dev}mm")
    if max_dev > 0.02:
        sys.exit(1)