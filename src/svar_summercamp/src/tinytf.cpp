#include "tinytf.h"

using namespace GSLAM;
using namespace sv;

TinyTF::TinyTF(Svar config){
    duration = config.get("duration",20.); // 10 seconds
    predict_period = config.get("predict_period", 0.02); // allow predict
}

// support both /odom and /fusion_pose
void TinyTF::append_msg(Svar msg){
    double tm   = msg_time(msg);
    SE3    pose = msg_pose(msg["pose"]["pose"]);
    append(tm, pose);
}

void TinyTF::append(double tm, SE3 pose){
    std::lock_guard<std::mutex> lock(mtx);
    if(tm < latest.first) // prevent clock go backward
        lut.clear();

    lut[tm] = pose;
    latest  = std::make_pair(tm, pose);

    auto it = lut.begin();
    for(; it != lut.end(); it++){
        if(tm - it->first < duration)
            break;
    }

    lut.erase(lut.begin(), it);
    condition.notify_one();
}

Svar TinyTF::get_once(double tm){
    std::lock_guard<std::mutex> lock(mtx);
    if(lut.empty())
    {
        return Svar();
    } 

    if(tm <= 0){
        return latest.second;// return the latest
    }
       

    if(tm > latest.first + predict_period)
        return Svar();

    if(tm > latest.first && lut.size() > 2){// use predict
        auto it = lut.end();
        auto right = *(--it);
        auto left  = *(--it);
        return interpolate(left, right, tm);
    }

    auto it = lut.lower_bound(tm);
    if(it == lut.end())
        return Svar(); // failed

    if(it == lut.begin()){
        LOG(WARNING)<<"TF exceed duration:"<< Svar(tm)
                   << "not in "<<Svar({lut.begin()->first, latest.first}).dump_json();
        throw SvarExeption("TF exceed duration.");
    }

    auto right = *it;
    auto left  = *(--it);

    return interpolate(left, right, tm);
}

Svar TinyTF::get(double tm){
    try {
        while(!should_stop){
            // LOG(WARNING)<<"TF:"<< Svar(tm)
            //            << " tring "<<Svar({lut.begin()->first, latest.first}).dump_json();
            auto ret = get_once(tm);
            if(!ret.isUndefined())
                return ret;

            {
                std::unique_lock<std::mutex> lock(mtx);
                condition.wait(lock);
            }
        }
        return Svar();
    } catch (std::exception& e) {
        return Svar();
    }
}

SE3 TinyTF::interpolate(std::pair<double,SE3> l, std::pair<double,SE3> r, double t){
    // TODO: this is a linear implementation, maybe a xyr model is better?
    SE3 r2l = l.second.inverse() * r.second;
    double factor = (t - l.first) / (r.first - l.first);
    r2l.get_translation() = r2l.get_translation() * factor;
    r2l.get_rotation() = SO3::exp(r2l.get_rotation().log() * factor);
    return l.second * r2l;
}

double TinyTF::msg_time(sv::Svar msg)
{
    sv::Svar stamp = msg["header"]["stamp"];
    return double(stamp["sec"].as<int>()) + stamp["nanosec"].as<int>() * 1e-9;
}

SE3 TinyTF::msg_pose(const Svar& msg){
    Svar p = msg["position"];
    Svar r = msg["orientation"];
    Point3d t (p["x"].castAs<double>(),p["y"].castAs<double>(),p["z"].castAs<double>());
    SO3 R (r["x"].castAs<double>(),r["y"].castAs<double>(),r["z"].castAs<double>(),r["w"].castAs<double>());
    return SE3(R,t);
}

REGISTER_SVAR_MODULE(tinytf){
    Class<TinyTF>("TinyTF")
            .construct<Svar>()
            .def("append_msg",&TinyTF::append_msg)
            .def("append",&TinyTF::append)
            .def("get_once",&TinyTF::get_once)
            .def("get",&TinyTF::get)
            .def_static("msg_pose",&TinyTF::msg_pose)
            .def_static("msg_time",&TinyTF::msg_time);

    Class<SE3>("SE3")
            .construct<>()
            .def("__init__", [](std::vector<double> v){
        return SE3(v[0],v[1],v[2],v[3],v[4],v[5],v[6]);
    })
    .def("from_buf", [](SE3& self, SvarBuffer buf){
        memcpy(&self, buf.ptr(), buf.size());
    })
    .def("to_buf",[](const SE3& self){
        return SvarBuffer((double*)&self.get_rotation().x, std::vector<ssize_t>({7})).clone();
    });
}
