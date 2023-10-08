#include "GSLAM/core/GSLAM.h"
#include "opencv2/opencv.hpp"
#include "tinytf.h"

using namespace sv;
using namespace GSLAM;

typedef Matrix<double,6,6> Matrix6d;

// this is to accelarate publishing by prepared memory
class PreparedOdom{
public:
    int  sec = 0;
    int  nanosec = 0;

    Svar msg;
    SE3  pos, vel;
    Point3d angular_vel;
    Matrix6d cov_pos, cov_vel;

    PreparedOdom(Svar config){
        std::string odom_frame_id = config.get<std::string>("odom_frame_id","map");
        std::string odom_child_frame_id = config.get<std::string>("odom_child_frame_id","odom");

        Svar stamp  = {{"sec", &sec},{"nanosec", &nanosec}};
        Svar header = {{"frame_id",odom_frame_id},{"stamp",stamp}};
        Svar pose   = {{"pose", from_pose(pos)},{"covariance", covbuf(cov_pos)}};
        Svar twist  = {{"twist", from_vel(vel)}, {"covariance", covbuf(cov_vel)}};

        msg = {{"header", header},
               {"child_frame_id", odom_child_frame_id},
               {"pose", pose},
               {"twist", twist}
              };
    }

    Svar from_pose(SE3& p){
        auto& t = p.get_translation();
        auto& r = p.get_rotation();
        Svar position = {{"x",&t.x},{"y",&t.y},{"z",&t.z}};
        Svar orientation = {{"x",&r.x},{"y",&r.y},{"z",&r.z},{"w",&r.w}};
        return {{"position",position},{"orientation",orientation}};
    }

    Svar from_vel(SE3& p){
        auto& t = p.get_translation();
        auto& r = angular_vel;
        Svar linear  = {{"x",&t.x},{"y",&t.y},{"z",&t.z}};
        Svar angular = {{"x",&r.x},{"y",&r.y},{"z",&r.z}};
        return {{"linear",linear},{"angular",angular}};
    }

    Svar from_pose_point(SE3& p){
        auto& t = p.get_translation();
        Svar position = {{"x",&t.x},{"y",&t.y},{"z",&t.z}};
        return position;
    }

    SvarBuffer covbuf(Matrix6d& m){
        return SvarBuffer(m.data(), {6,6}).clone();
    }
};

/// 对机器人本体的运动进行仿真
class Simulator{
public:
    Simulator(Svar config):odom(config){
        max_vx = config.get("max_vx",0.4);
        max_vrz= config.get("max_vrz",1.0);
        max_ax = config.get("max_ax",1.0);
        max_arz = config.get("max_arz",1.0);
        sim_t_factor= config.get("sim_t_factor",1.0);
        cmd_timeout = config.get("cmd_timeout", 0.2);
        freq = config.get("freq",100.0);
        mesh = config.get<std::string>("mesh","C3.stl");
        std::string odom = config.get<std::string>("odom","/odom");
        std::string cmd_topic = config.get<std::string>("cmd_topic","/vel_mcu");

        if(config["init_pose"].isArray()){
            auto vec = config["init_pose"].castAs<std::vector<double>>();
            if(vec.size() >= 12)
                pose.fromMatrix(vec.data());
            else if(vec.size() == 7)
                pose = SE3(vec[0],vec[1],vec[2],vec[3],vec[4],vec[5],vec[6]);
        }

        pub_odom   = messenger.advertise(odom, 0);
        pub_fusion_pose = messenger.advertise("/ground_pose");
        pub_tf     = messenger.advertise("/tf");

        sub_velcmd = messenger.subscribe(cmd_topic,0,&Simulator::cbk_vel_mcu,this);

        if(config.get("thread",false))
            work_thread = std::thread(&Simulator::work,this);
    }

    ~Simulator(){
        should_stop = true;
        if(work_thread.joinable())
            work_thread.join();
    }

    void cbk_vel_mcu(const Svar& msg){
        double vx = msg["linear"]["x"].castAs<double>();
        double vrz = msg["angular"]["z"].castAs<double>();

        set_target_vel(SE3(SO3::exp(Point3d(0,0,vrz)),Point3d(vx,0,0)));

        DLOG(INFO)<<get_target_vel();
    }

    void work(){
        Rate rate(freq * sim_t_factor);

        should_stop = false;
        start_sim_t = TicToc::timestamp();

        last_t      = start_sim_t;

        while(!should_stop){
            double sim_t   = get_sim_t();
            double delta_t = sim_t - last_t;
            last_t = sim_t;

            update_pose(delta_t);
            publish_odom(sim_t, pose, vel);
            publish_pose(sim_t, pose);

            rate.sleep();
        }
    }

    Svar step(double delta_t, Svar action){
        cbk_vel_mcu(action);
        last_t += delta_t;

        while(delta_t > 0.01){
            delta_t -= 0.01;
            update_pose(0.01);
        }

        update_pose(delta_t);
        publish_odom(last_t, pose, vel);
        publish_pose(last_t, pose);

        return {pose, vel};
    }

    Svar reset(SE3 pose){
        this->pose = pose;
        vel = target_vel = SE3();
//        last_t = 0;
        publish_odom(last_t, pose, vel);
        publish_pose(last_t, pose);

        return {pose, vel};
    }

    void update_pose(double dt){
        // update velocity
        SE3 t_v = get_target_vel();

        // vx
        double& vx = vel.get_translation().x;

        double delta_vx = t_v.get_translation().x - vx;

        if(fabs(delta_vx) < max_ax * dt)
            vx = t_v.get_translation().x;
        else if(delta_vx > 0)
            vx += max_ax * dt;
        else
            vx -= max_ax * dt;

        if(fabs(vx) > max_vx)
            vx *= max_vx/fabs(vx);

        // vrz
        double vrz = vel.get_rotation().log().z;
        double delta_vrz = t_v.get_rotation().log().z - vrz;
        if(fabs(delta_vrz) < max_arz * dt)
            vrz = t_v.get_rotation().log().z;
        else  if(delta_vrz > 0)
            vrz += max_arz * dt;
        else
            vrz -= max_arz * dt;

        if(fabs(vrz) > max_vrz)
            vrz *= max_vrz/fabs(vrz);

        vel.get_rotation() = SO3::exp(Point3d(0,0,vrz));
        vel.get_translation() = Point3d(vx, 0, 0);

        // update pose
        double dx = dt * vx;
        SO3 delta_rx = SO3::exp(Point3d(0,0,vrz) * dt);
        pose.get_translation() = pose * Point3d(dx, 0, 0);
        pose.get_rotation() = pose.get_rotation() * delta_rx;

        DLOG(INFO)<<"t_v:"<<t_v<<",vel:"<<vel<<",pose:"<<pose;

        // TODO: simulate odom drift
    }

    void publish_odom(double tm, SE3 pos, SE3 vel,
                      const Matrix6d& cov_pos = Matrix6d::zeros(),
                      const Matrix6d& cov_vel = Matrix6d::zeros()){
        if(tm - last_publish_time >= odom_publish_gap) {// 50hz
            odom.sec = tm;
            odom.nanosec = (tm - odom.sec) * 1e9;
            odom.pos = pos;
            odom.vel = vel;
            odom.angular_vel = vel.get_rotation().log();
            odom.cov_pos = cov_pos;
            odom.cov_vel = cov_vel;

            pub_odom.publish(odom.msg);
            last_publish_time = tm;
        }
    }

    void publish_pose(double sim_t, SE3 pos){
        pub_fusion_pose.publish(Svar({sim_t, pose}));

        TinyTF::pub_tf(pub_tf, sim_t, pos, "map", "base_link_ground");

        if(sim_t - last_publish_robot > 5)
        {
            last_publish_robot = sim_t;
            visualize_robot();
        }
    }

    void set_target_vel(SE3 s){
        std::unique_lock<std::mutex> lock(m_target_vel);
        target_vel = s;
        last_cmd_t = get_sim_t();
    }

    SE3 get_target_vel(){
        std::unique_lock<std::mutex> lock(m_target_vel);
        if(get_sim_t() - last_cmd_t > cmd_timeout)
            target_vel = SE3();
        return target_vel;
    }

    double get_sim_t(){
        double t       = TicToc::timestamp();
        double sim_t   = start_sim_t + sim_t_factor * (t - start_sim_t);
        return sim_t;
    }

    void visualize_robot(int type = 10){
        double M[16] = {0,0,-1,0.255,
                        -1,0,0,0.255,
                        0,1,0,0,
                        0,0,0,1};
        SE3 T;
        T.fromMatrix(M);

        auto r = T.get_rotation();
        auto t = T.get_translation();


        double s = 0.05;
        if(type == 10){
            s = 0.001;
        }
        else
            t.z += 0.5;

        Svar pose= {{"position",{{"x",t.x},{"y",t.y},{"z",t.z}}},
                    {"orientation",{{"x",r.x},{"y",r.y},{"z",r.z},{"w",r.w}}}};

        double tm = TicToc::timestamp();
        int sec = tm;
        int nanosec = (tm - sec)*1e9;

        Svar header = {{"frame_id", "base_link_ground"},
                       {"stamp", {{"sec",sec},{"nanosec",nanosec}}}};

        Svar color = {{"a",1.0},{"r",1.0},
                      {"g",1.0},{"b",1.0}};
        Svar scale = {{"x",s},
                      {"y",s},
                      {"z",s}};

        std::string mesh_resource = "file://" + mesh;
        int action = 0;
        Svar marker={{"header", header},
                     {"ns", "robot"},
                     {"id", type},
                     {"action", action},
                     {"type", type},
                     {"color", color},
                     {"scale", scale},
                     {"pose", pose},
                     {"frame_locked", true},
                     {"mesh_resource", mesh_resource},
                     {"text", "robot"},
                     {"lifetime", {{"sec", 0}}}};
        Svar markers = {{"markers",{marker}}};

        messenger.publish("/markers", markers);
    }

    Subscriber sub_velcmd;
    Publisher pub_odom, pub_fusion_pose, pub_tf;
    SE3    pose, vel, target_vel, odom2map;
    std::mutex m_target_vel;

    PreparedOdom odom;

    std::thread work_thread;
    std::string mesh;


    bool   should_stop;
    double max_vx, max_vrz, max_ax, max_arz;
    double sim_t_factor, start_sim_t, last_t = 0, freq, last_cmd_t = 0, cmd_timeout = 0.2,
    last_publish_time=0, odom_publish_gap=0, last_publish_robot = 0;
};

class Enviroment{
public:
    Enviroment(Svar config)
        : sim(config){
        config["ground_pose_queue"] = 0;
        world = svar["World"](config);

        sub_scan = messenger.subscribe("/scan",0,&Enviroment::cbk_scan,this);
    }

    void cbk_scan(const Svar& msg){
        scan = msg;
    }

    Svar step(double dt, Svar action){
        Svar pos_vel = sim.step(dt, action);
        pos_vel.push_back(scan);
        return pos_vel;
    }

    Svar reset(){
        Svar pos_vel = sim.reset(random_free_pose());
        pos_vel.push_back(scan);
        return pos_vel;
    }

    SE3 random_free_pose(){
        return world.call("random_free_pose").as<SE3>();
    }

    Subscriber sub_scan;
    Simulator sim;
    Svar      world, scan;
};

REGISTER_SVAR_MODULE(simulater){
    Class<Simulator>("Simulator")
            .construct<Svar>();

    Class<Enviroment>("Enviroment")
            .construct<Svar>()
            .def("step",&Enviroment::step)
            .def("reset",&Enviroment::reset)
            .def("random_free_pose",&Enviroment::random_free_pose);
}

EXPORT_SVAR_INSTANCE
