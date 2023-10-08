#pragma once
#include "GSLAM/core/GSLAM.h"
#include "nanoflann.h"

struct GImageAdaptor {
    GImageAdaptor(GSLAM::GImage cloud)
        : cloud(cloud){}

    inline size_t kdtree_get_point_count() const {
        return cloud.rows;
    }

    inline float kdtree_get_pt(const size_t idx, int dim) const {
        return cloud.at<float>(dim, idx);
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /*bb*/) const {
        return false;
    }

    inline float kdtree_distance(const float* a, const size_t idx, size_t dim) const
    {
        float* b = cloud.ptr<float>(idx);
        float sum = 0;
        for(size_t i = 0; i < dim; i++)
            sum += (a[i] - b[i]) * (a[i] - b[i]);
        
        return sum;
    }

    mutable GSLAM::GImage cloud;
};

class KDTreeGImage: public nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, GImageAdaptor>, GImageAdaptor, -1>{
public:
    KDTreeGImage(int dim, std::shared_ptr<GImageAdaptor> ad)
        :nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, GImageAdaptor>, GImageAdaptor, -1>(dim,*ad), ad(ad){}

    std::shared_ptr<GImageAdaptor> ad;
};
