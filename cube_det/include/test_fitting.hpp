
#ifndef CURVE_FIT_3D_HPP
#define CURVE_FIT_3D_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <algorithm>
#include <limits>

class CurveFitter3D {
public:
    struct Parameters {
        int fitted_points = 300;       // 最终拟合曲线的总点数
        float voxel_size = 0.02f;      // 体素网格降采样尺寸
        float search_radius = 0.5f;    // 搜索最近点的半径
        
        Parameters() : fitted_points(300), voxel_size(0.02f), search_radius(0.5f) {}
    };

    CurveFitter3D(const Parameters& params = Parameters()) : params_(params) {}

    // 使用体素网格降采样并排序点
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleAndSortPoints(
        const std::vector<pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>& curves) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        
        // 1. 合并所有点
        for (const auto& curve : curves) {
            *merged += *curve;
        }
        
        // 2. 使用体素网格降采样
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(merged);
        voxel_filter.setLeafSize(params_.voxel_size, params_.voxel_size, params_.voxel_size);
        voxel_filter.filter(*downsampled);
        
        // 3. 使用改进的曲线排序算法
        return sortPointsAlongCurve(downsampled);
    }

    // 整体拟合为一条完整曲线
    pcl::PointCloud<pcl::PointXYZ>::Ptr fitSingleCurve(
        const std::vector<pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>& input_curves) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
        if (input_curves.empty()) return result;
        
        // 1. 降采样并排序点
        auto sorted_points = downsampleAndSortPoints(input_curves);
        if (sorted_points->size() < 3) return sorted_points;
        
        // 2. 准备插值参数
        std::vector<float> t(sorted_points->size());
        std::iota(t.begin(), t.end(), 0);
        float max_t = static_cast<float>(t.back());
        for (auto& val : t) val /= max_t;
        
        // 3. 提取坐标数据
        std::vector<float> x(sorted_points->size()), y(sorted_points->size()), z(sorted_points->size());
        for (size_t i = 0; i < sorted_points->size(); ++i) {
            x[i] = sorted_points->points[i].x;
            y[i] = sorted_points->points[i].y;
            z[i] = sorted_points->points[i].z;
        }
        
        // 4. 创建三次样条
        CubicSpline spline_x(t, x);
        CubicSpline spline_y(t, y);
        CubicSpline spline_z(t, z);
        
        // 5. 生成拟合曲线
        for (int i = 0; i < params_.fitted_points; ++i) {
            float t_new = static_cast<float>(i) / (params_.fitted_points - 1);
            
            pcl::PointXYZ pt;
            pt.x = spline_x.evaluate(t_new);
            pt.y = spline_y.evaluate(t_new);
            pt.z = spline_z.evaluate(t_new);
            
            result->push_back(pt);
        }
        
        return result;
    }

private:
    Parameters params_;

    // 改进的曲线点排序算法(考虑曲线延伸方向)
    pcl::PointCloud<pcl::PointXYZ>::Ptr sortPointsAlongCurve(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr sorted(new pcl::PointCloud<pcl::PointXYZ>);
        if (cloud->empty()) return sorted;
        
        // 1. 找到最左下角的起点
        pcl::PointXYZ start_point = findStartPoint(cloud);
        sorted->push_back(start_point);
        
        // 2. 创建KD树用于最近邻搜索
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        
        // 3. 逐步构建曲线
        pcl::PointXYZ current_point = start_point;
        std::vector<bool> processed(cloud->size(), false);
        
        // 标记起点为已处理
        for (size_t i = 0; i < cloud->size(); ++i) {
            if (pcl::euclideanDistance(cloud->points[i], start_point) < 1e-6f) {
                processed[i] = true;
                break;
            }
        }
        
        // 4. 基于曲线延伸方向寻找下一个点
        Eigen::Vector3f prev_direction(0, 0, 0);
        for (size_t i = 1; i < cloud->size(); ++i) {
            std::vector<int> point_idx;
            std::vector<float> point_sqr_dist;
            
            // 搜索半径内的候选点
            kdtree.radiusSearch(current_point, params_.search_radius, point_idx, point_sqr_dist);
            
            // 过滤掉已处理的点
            std::vector<int> candidates;
            for (size_t j = 0; j < point_idx.size(); ++j) {
                if (!processed[point_idx[j]]) {
                    candidates.push_back(point_idx[j]);
                }
            }
            
            // 如果没有候选点，扩大搜索半径
            if (candidates.empty()) {
                kdtree.nearestKSearch(current_point, 1, point_idx, point_sqr_dist);
                if (!processed[point_idx[0]]) {
                    candidates.push_back(point_idx[0]);
                } else {
                    break; // 没有更多点可处理
                }
            }
            
            // 选择最符合曲线延伸方向的点
            int best_idx = selectBestNextPoint(cloud, current_point, candidates, prev_direction);
            if (best_idx == -1) break;
            
            // 添加到结果并更新状态
            sorted->push_back(cloud->points[best_idx]);
            processed[best_idx] = true;
            
            // 更新延伸方向
            prev_direction = Eigen::Vector3f(
                cloud->points[best_idx].x - current_point.x,
                cloud->points[best_idx].y - current_point.y,
                cloud->points[best_idx].z - current_point.z).normalized();
            
            current_point = cloud->points[best_idx];
        }
        
        return sorted;
    }

    // 找到最左下角的起点
    pcl::PointXYZ findStartPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        pcl::PointXYZ start = cloud->points[0];
        for (const auto& p : *cloud) {
            if (p.x < start.x || (p.x == start.x && p.y < start.y)) {
                start = p;
            }
        }
        return start;
    }

    // 选择最佳的下一个点(基于曲线延伸方向)
    int selectBestNextPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          const pcl::PointXYZ& current_point,
                          const std::vector<int>& candidates,
                          const Eigen::Vector3f& prev_direction) {
        if (candidates.empty()) return -1;
        
        // 如果没有先前的方向信息，选择最近的点
        if (prev_direction.norm() < 0.1f) {
            float min_dist = std::numeric_limits<float>::max();
            int best_idx = candidates[0];
            
            for (int idx : candidates) {
                float dist = pcl::euclideanDistance(current_point, cloud->points[idx]);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_idx = idx;
                }
            }
            return best_idx;
        }
        
        // 否则选择最符合延伸方向的点
        float best_score = -std::numeric_limits<float>::max();
        int best_idx = candidates[0];
        
        for (int idx : candidates) {
            Eigen::Vector3f dir(
                cloud->points[idx].x - current_point.x,
                cloud->points[idx].y - current_point.y,
                cloud->points[idx].z - current_point.z);
            
            float dist = dir.norm();
            dir.normalize();
            
            // 评分标准: 方向一致性和距离的平衡
            float direction_score = dir.dot(prev_direction);
            float distance_score = 1.0f / (dist + 0.001f); // 避免除以零
            float total_score = direction_score * 0.7f + distance_score * 0.3f;
            
            if (total_score > best_score) {
                best_score = total_score;
                best_idx = idx;
            }
        }
        
        return best_idx;
    }

    // 三次样条实现
    class CubicSpline {
    public:
        CubicSpline(const std::vector<float>& x, const std::vector<float>& y) : x_(x), y_(y) {
            computeCoefficients();
        }

        float evaluate(float t) {
            auto it = std::upper_bound(x_.begin(), x_.end(), t);
            int idx = std::distance(x_.begin(), it) - 1;
            idx = std::max(0, std::min(static_cast<int>(x_.size()) - 2, idx));
            
            float h = t - x_[idx];
            return a_[idx] + b_[idx] * h + c_[idx] * h * h + d_[idx] * h * h * h;
        }

    private:
        std::vector<float> x_, y_;
        std::vector<float> a_, b_, c_, d_;

        void computeCoefficients() {
            int n = x_.size() - 1;
            std::vector<float> h(n), alpha(n), l(n+1), mu(n+1), z(n+1);
            
            for (int i = 0; i < n; ++i) 
                h[i] = x_[i+1] - x_[i];

            for (int i = 1; i < n; ++i)
                alpha[i] = 3.0f * ((y_[i+1] - y_[i]) / h[i] - (y_[i] - y_[i-1]) / h[i-1]);

            l[0] = 1; mu[0] = z[0] = 0;

            for (int i = 1; i < n; ++i) {
                l[i] = 2 * (x_[i+1] - x_[i-1]) - h[i-1] * mu[i-1];
                mu[i] = h[i] / l[i];
                z[i] = (alpha[i] - h[i-1] * z[i-1]) / l[i];
            }

            l[n] = 1; z[n] = 0;
            std::vector<float> c(n+1), b(n), d(n);
            c[n] = 0;

            for (int j = n-1; j >= 0; --j) {
                c[j] = z[j] - mu[j] * c[j+1];
                b[j] = (y_[j+1] - y_[j]) / h[j] - h[j] * (c[j+1] + 2 * c[j]) / 3;
                d[j] = (c[j+1] - c[j]) / (3 * h[j]);
            }

            a_ = y_;
            b_.resize(n); c_.resize(n); d_.resize(n);
            for (int i = 0; i < n; ++i) {
                b_[i] = b[i]; c_[i] = c[i]; d_[i] = d[i];
            }
        }
    };
};

#endif // CURVE_FIT_3D_HPP