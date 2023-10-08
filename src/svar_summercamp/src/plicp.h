#pragma once
#include "gimage_adaptor.h"

GSLAM::GImage transform(GSLAM::GImage map_cloud, GSLAM::SE3 trans);
double plicp_score(std::shared_ptr<KDTreeGImage> kdtree, GSLAM::GImage map_cloud,
                   GSLAM::GImage cur_cloud,GSLAM::SE3 current_pose, double line_search_radius_sqr);
std::shared_ptr<KDTreeGImage> return_plicp_kdtree(GSLAM::GImage map_cloud, sv::Svar config);

std::vector<double> calculate_cloud_hessian(GSLAM::GImage input_cloud);

sv::Svar plicp_kdtree(std::shared_ptr<KDTreeGImage> kdtree, GSLAM::GImage cur_cloud, 
                      GSLAM::SE3 prior_pose, sv::Svar config, double &fitness_score);
sv::Svar plicp(GSLAM::GImage map_cloud, GSLAM::GImage cur_cloud, GSLAM::SE3 prior_pose, sv::Svar config);


