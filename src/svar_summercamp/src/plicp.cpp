#include <ceres/ceres.h>
#include <GSLAM/core/GSLAM.h>
#include "factors.h"
#include "plicp.h"

using namespace GSLAM;

GImage transform(GImage map_cloud, GSLAM::SE3 trans){
    GImage ret = map_cloud.clone();
    for(int y=0;y<ret.rows;y++){
        float*  ptr = & ret.at<float>(y*ret.cols);
        Point3d pt = trans * Point3d(ptr[0], ptr[1], ptr[2]);
        ptr[0] = pt[0]; ptr[1] = pt[1]; ptr[2] = pt[2];
    }
    return ret;
}

double plicp_score(std::shared_ptr<KDTreeGImage> kdtree, GImage map_cloud,GImage cur_cloud,GSLAM::SE3 current_pose, double line_search_radius_sqr){
    GImage cloud_aligned = transform(cur_cloud, current_pose);

    std::vector<size_t> tmp_indices(1);
    std::vector<float>  tmp_dists(1);
    GSLAM::SE3 pos;
    std::vector<double> distances;
    for(auto y = 0; y < cloud_aligned.rows; y++){
        float* c = &cloud_aligned.at<float>(cloud_aligned.cols*y);
        kdtree->knnSearch(c, 1u, &tmp_indices[0], &tmp_dists[0]);
        if(tmp_dists[1] > line_search_radius_sqr)
            continue;// abort

        float* a = &map_cloud.at<float>(cloud_aligned.cols * tmp_indices[0]);

        Point3d pc(c[0], c[1], c[2]), pa(a[0],a[1],a[2]);
        distances.push_back((pa-pc).norm());
    }

    if((int)distances.size() < std::max(20, cloud_aligned.rows /3))
        return 1e9;

    std::sort(distances.begin(), distances.end());

    return distances[cloud_aligned.rows /2];
}

// -PI ~ PI
float angle_diff(float diff){
    if(diff > M_PI)
        diff -= 2 * M_PI;
    else if(diff < -M_PI)
        diff += 2 * M_PI;
    LOG_IF(FATAL,diff < -M_PI || diff > M_PI)<<"diff:"<<diff
                                            <<", should not happen.";
    return fabs(diff);
}

sv::Svar plicp_kdtree(std::shared_ptr<KDTreeGImage> kdtree, GSLAM::GImage cur_cloud, 
                      GSLAM::SE3 prior_pose, sv::Svar config, double &fitness_score){
    int max_iteration = config.get("plicp_iterations",10);
    int max_num_iterations = config.get("max_num_iterations", 3);
    int dim = config.get("dim",2);
    int min_residuals = config.get("min_residuals", 20);
    double huber = config.get("huber", 0.1);
    double line_search_radius = config.get("line_search_radius", 1.0);
    double prior_weight = config.get("prior_weight", 0); // prior weight
    double progress_threshold = config.get("progress_threshold", 0.01); // converaged
    bool   const_rotation = config.get("const_rotation", false);
    double icp_score_threshold = config.get("icp_score_threshold",0.05);
    bool   cloud_angle = config.get("cloud_angle",false);
    double min_line_length = config.get("min_line_length", 0.01);

    double line_search_radius_sqr = line_search_radius * line_search_radius;

    GImage map_cloud = kdtree->ad->cloud;
    std::vector<size_t> tmp_indices(2);
    std::vector<float>  tmp_dists(2);

    GSLAM::SE3 current_pose = prior_pose;

    for(auto i = 0; i < max_iteration; i++){
        GImage cloud_aligned = transform(cur_cloud, current_pose);

        // prior_pose = progress * current_pose;

        ceres::Problem problem;
        ceres::LossFunction *lossfunction = new ceres::HuberLoss(huber);

        GSLAM::SE3 progress;

        ceres::LocalParameterization* quaterion_parameterization = new ceres::QuaternionParameterization();
        problem.AddParameterBlock(&progress.get_translation().x,3);
        problem.AddParameterBlock(&progress.get_rotation().x,4,quaterion_parameterization);
        if(const_rotation)
            problem.SetParameterBlockConstant(&progress.get_rotation().x);

        for(auto y = 0; y < cloud_aligned.rows; y++){
            float* c = &cloud_aligned.at<float>(cloud_aligned.cols*y);
            kdtree->knnSearch(c, 2u, &tmp_indices[0], &tmp_dists[0]);
            if(tmp_dists[1] > line_search_radius) continue;// abort

            float* a = &map_cloud.at<float>(cloud_aligned.cols * tmp_indices[0]);
            float* b = &map_cloud.at<float>(cloud_aligned.cols * tmp_indices[1]);

            if(cloud_angle && cur_cloud.cols >= 5){
                float a_angle = a[4];
                float b_angle = b[4];
                float c_angle = c[4];

                if(angle_diff(a_angle - c_angle) > M_PI ||
                   angle_diff(b_angle - c_angle) > M_PI) continue;
            }

            Point3d pc(c[0], c[1], c[2]), pa(a[0],a[1],a[2]), pb(b[0], b[1], b[2]);

            if((pa-pb).norm() < min_line_length)
            {
                ceres::CostFunction* cost = ceres::PointDistanceFactor::Create(pc, pa);
                problem.AddResidualBlock(cost, lossfunction, &progress.get_rotation().x, &progress.get_translation().x);
            }
            else
            {
                ceres::CostFunction* cost = ceres::PointToLineDistanceFactor::Create(20.,pc, pa, pb);
                problem.AddResidualBlock(cost, lossfunction, &progress.get_rotation().x, &progress.get_translation().x);
            }
        }

        if(prior_weight > 0){
            ceres::CostFunction* cost = ceres::PosePriorFactor::Create(prior_pose * current_pose.inverse(), std::vector<double>(6,prior_weight));
            problem.AddResidualBlock(cost, lossfunction, &progress.get_translation().x, &progress.get_rotation().x);
        }

        if(problem.NumResidualBlocks() < min_residuals){
            // LOG(WARNING) << "residuals not enough.";
            return Svar();// failed
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.max_num_iterations = max_num_iterations;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
//        LOG(INFO)<<summary.BriefReport();

        current_pose = progress * current_pose;

        if(progress.get_translation().norm() + progress.get_rotation().log().norm() < progress_threshold)
            break;
    }

    fitness_score = plicp_score(kdtree,map_cloud,cur_cloud,current_pose,line_search_radius_sqr);
    if(line_search_radius > 1){
        LOG(INFO) << "score:" << fitness_score;
    }
    
    if(fitness_score > icp_score_threshold){
        LOG(INFO)<<"fitness_score "<<fitness_score<<" too large.";
        return Svar();
    }

    return {current_pose, fitness_score};
}

std::shared_ptr<KDTreeGImage> return_plicp_kdtree(GSLAM::GImage map_cloud, sv::Svar config){
    int dim = config.get("dim",2);
    std::shared_ptr<GImageAdaptor> adaptor= std::make_shared<GImageAdaptor>(map_cloud);
    std::shared_ptr<KDTreeGImage> kdtree = std::make_shared<KDTreeGImage>(dim, adaptor);

    kdtree->buildIndex();
    return kdtree;
}

sv::Svar plicp(GImage map_cloud, GImage cur_cloud, GSLAM::SE3 prior_pose, sv::Svar config){
    int dim = config.get("dim",2);
    std::shared_ptr<GImageAdaptor> adaptor= std::make_shared<GImageAdaptor>(map_cloud);
    std::shared_ptr<KDTreeGImage> kdtree = std::make_shared<KDTreeGImage>(dim, adaptor);

    kdtree->buildIndex();
    double fitness_score = 0.0; 
    return plicp_kdtree(kdtree, cur_cloud, prior_pose, config, fitness_score);
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d mat;
    mat(0, 0) = 0.0;
    mat(0, 1) = -v(2);
    mat(0, 2) = v(1);
    mat(1, 0) = v(2);
    mat(1, 1) = 0.0;
    mat(1, 2) = -v(0);
    mat(2, 0) = -v(1);
    mat(2, 1) = v(0);
    mat(2, 2) = 0.0;
    return mat;
}

std::vector<double> calculate_cloud_hessian(GImage input_cloud){
    GImage new_input_cloud = input_cloud;

    int dim = 2;
    KDTreeGImage kdtree(dim, std::make_shared<GImageAdaptor>(new_input_cloud));
    kdtree.buildIndex();
    std::vector<size_t> tmp_indices(5);
    std::vector<float>  tmp_dists(5);
    Eigen::Matrix<double, 6, 6> Hessian = Eigen::Matrix<double, 6, 6>::Zero();

    int corner_num= 0;
    for(auto y = 0; y < new_input_cloud.rows; y++){
        float* c = &new_input_cloud.at<float>(input_cloud.cols*y);
        kdtree.knnSearch(c, 5u, &tmp_indices[0], &tmp_dists[0]);
        if(tmp_dists[4] < 1.0){
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp((&new_input_cloud.at<float>(new_input_cloud.cols * tmp_indices[j]))[0],
                                    (&new_input_cloud.at<float>(new_input_cloud.cols * tmp_indices[j]))[1],
                                    0);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            // if is indeed line feature
            // note Eigen library sort eigenvalues in increasing order
            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d lp(c[0], c[1], 0);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            {
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d last_point_a, last_point_b;
                last_point_a = 0.1 * unit_direction + point_on_line;
                last_point_b = -0.1 * unit_direction + point_on_line;

                Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
                Eigen::Vector3d de = last_point_a - last_point_b;
                double de_norm = de.norm();
                if (de.norm() == 0 || nu.norm() == 0)
                {
                    continue;
                }

                Eigen::Matrix3d skew_lp = skew(lp);
                Eigen::Matrix<double, 3, 6> dp_by_se3;
                dp_by_se3.block<3, 3>(0, 0) = -skew_lp;
                (dp_by_se3.block<3, 3>(0, 3)).setIdentity();
                Eigen::Matrix<double, 1, 6, Eigen::RowMajor> J_se3;
                J_se3.setZero();
                Eigen::Matrix3d skew_de =skew(de);
                J_se3.block<1, 6>(0, 0) =
                    -nu.transpose() / nu.norm() * skew_de * dp_by_se3 / de_norm;
                corner_num++;
                Hessian += J_se3.transpose() * J_se3;
            }
        }
    }

    Eigen::Matrix<double, 6, 6> Hessian_tmp = Hessian / corner_num * 1000.0;
    return { Hessian_tmp(0, 0), Hessian_tmp(1, 1), Hessian_tmp(2, 2),
             Hessian_tmp(3, 3), Hessian_tmp(4, 4), Hessian_tmp(5, 5)};
}

int test_plicp_sim(Svar config){
    std::vector<Point3f> points, tranformed;

    for(int i=1;i<100;i++){
        points.push_back(Point3f(i*0.1, 0, 0));
        points.push_back(Point3f(0, i*0.1, 0));
    }

    GImage map(points.size(), 3, GImageType<float,1>::Type, (uchar*)&points.data()->x);
    SE3    trans (SO3::exp(Point3d(0,0,0.1)), Point3d(0.05, 0.05, 0));
    GImage cur = transform(map, trans.inverse());

    Svar ret = plicp(map, cur, SE3(), Svar());

    LOG(INFO)<<ret.as<SE3>()<<", ground:"<<trans;
    return 0;
}

GImage resample(GImage cloud, double distance){
    if(distance <= 0)
        return cloud;

    std::unordered_map<uint64_t, float*> grid;

    int step = cloud.cols * cloud.elemSize();
    float distance_inv = 1.f/ distance;

    for(int i=0;i < cloud.rows;i++){
        float*   ptr = (float*)(cloud.data + i * step);
        Point2i  gid (ptr[0] * distance_inv, ptr[1] * distance_inv);
        uint64_t idx = *(uint64_t*)&gid;
        grid[idx] = ptr;
    }

    GImage ret(grid.size(), cloud.cols, cloud.type());

    int offset = 0;
    for(std::pair<uint64_t, float*> it : grid){
        memcpy(ret.data + offset, it.second, step);
        offset += step;
    }

    return ret;
}

GImage concat(std::vector<GImage> clouds){
    if(clouds.size() == 1)
        return clouds.front();

    int num = 0;
    for(auto& c:clouds) num+=c.rows;

    GImage ret(num, clouds.front().cols, clouds.front().type());

    int offset=0;
    for(auto& c:clouds)
    {
        int sz = c.total() * c.elemSize();
        memcpy(ret.data + offset, c.data, sz);
        offset += sz;
    }
    return ret;
}

REGISTER_SVAR_MODULE(plicp){
    svar["apps"]["test_plicp_sim"]={test_plicp_sim,"test plicp with simulate"};
    svar["plicp"] = plicp;
    svar["transform"] = transform;
    svar["resample"] = resample;
    svar["concat"] = concat;
    svar["calculate_cloud_hessian"] = calculate_cloud_hessian;
}
