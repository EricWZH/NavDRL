#pragma once
#include <GSLAM/core/GSLAM.h>

class TinyTF{
public:
    TinyTF(sv::Svar config);

    // support both /odom and /fusion_pose
    void append_msg(sv::Svar msg);
    void append(double tm, GSLAM::SE3 pose);

    sv::Svar get_once(double tm);
    sv::Svar get(double tm);

    static double msg_time(sv::Svar msg);
    static GSLAM::SE3 msg_pose(const sv::Svar& msg);

    static sv::Svar from_pose(const GSLAM::SE3& p){
        auto t = p.get_translation();
        auto r = p.get_rotation();
        sv::Svar position = {{"x",t.x},{"y",t.y},{"z",t.z}};
        sv::Svar orientation = {{"x",r.x},{"y",r.y},{"z",r.z},{"w",r.w}};
        return {{"position",position},{"orientation",orientation}};
    }

    static void pub_tf(GSLAM::Publisher pub, double tm, GSLAM::SE3 pos,
                       std::string frame_id, std::string child_frame_id){
        int sec     = tm;
        int nanosec = (tm - sec)*1e9;
        sv::Svar stamp = {{"sec",sec},{"nanosec",nanosec}};
        sv::Svar tf;
        tf["header"] = {{"frame_id", frame_id}, {"stamp", stamp}};
        tf["child_frame_id"] = child_frame_id;

        sv::Svar pose = TinyTF::from_pose(pos);
        sv::Svar r = pose["orientation"];
        sv::Svar t = pose["position"];
        tf["transform"] = {{"rotation",r}, {"translation",t}};
        sv::Svar msg = {{"transforms",{tf}}};

        DLOG(INFO) << msg;
        pub.publish(msg);
    }

public:
    GSLAM::SE3 interpolate(std::pair<double,GSLAM::SE3> l, std::pair<double,GSLAM::SE3> r, double t);

    double                 duration, predict_period;
    std::pair<double, GSLAM::SE3> latest;
    std::map<double, GSLAM::SE3>  lut;
    std::mutex             mtx;
    std::condition_variable    condition;
    bool                       should_stop = false;
};
