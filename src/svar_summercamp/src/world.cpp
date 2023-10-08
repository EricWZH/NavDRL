#include "GSLAM/core/GSLAM.h"
#include "opencv2/opencv.hpp"
#include "tinytf.h"


using namespace sv;
using namespace GSLAM;

typedef Matrix<double,6,6> Matrix6d;

/// 对机器人的传感器和碰撞信息进行仿真
class World{
public:
    World(Svar config){
        std::string map_file = config.get<std::string>("map_file","");
        ranges_num = config.get("ranges_num", 360); // 一圈激光的个数
        int ground_pose_queue = config.get("ground_pose_queue",10);
        robot_radius = config.get("robot_radius",0.3);
        min_scan_time_diff = config.get("min_scan_time_diff",0.1);

        scan2baselink = SE3(SO3(), Point3d(0.08, 0, 0.25));

        if(config["scan2baselink"].isArray()){
            auto vec = config["scan2baselink"].castAs<std::vector<double>>();
            if(vec.size() >= 12)
                scan2baselink.fromMatrix(vec.data());
            else if(vec.size() == 7)
                scan2baselink = SE3(vec[0],vec[1],vec[2],vec[3],vec[4],vec[5],vec[6]);
        }

        if(!load_map(map_file))
            LOG(FATAL)<<"Failed to load map "<<map_file;
        else
            LOG(INFO)<<"Loaded map "<<map_file;

        pub_map  = messenger.advertise("/map");
        pub_map_cloud = messenger.advertise("/map_cloud");
        pub_scan = messenger.advertise("/scan");
        pub_tf   = messenger.advertise("/tf");
        sub_pose = messenger.subscribe("/ground_pose", ground_pose_queue, &World::cbk_pose, this);
    }

    bool load_map(std::string map_file){
        std::ifstream in(map_file + ".yaml",std::ios::in|std::ios::binary);
        std::string body( (std::istreambuf_iterator<char>(in)) , std::istreambuf_iterator<char>() );
        Svar content = svar.import("svar_yaml")["parse_yaml"](body);

        auto o = content["origin"].castAs<std::vector<double>>();
        origin = SE3(SO3(),Point3d(o[0],o[1],o[2]));
        resolution = content["resolution"].as<double>();


        cv::Mat map = cv::imread(map_file + ".pgm", cv::IMREAD_UNCHANGED);

        cv::flip(map, map, 0);
        map_image = map;

        publish_map();

        // prepare map_cloud
        {
            map_cloud.clear();

            uchar* data = map_image.data;
            for(int y=0;y<map_image.rows;y++)
                for(int x=0;x<map_image.cols;x++,data++){
                    if(*data) continue;
                    Point3d pt   = origin * (resolution * Point3d(x + 0.5,y+ 0.5,0));
                    for(int z = 0; z < 3; z++)
                    {
                        pt.z = resolution * (z + 0.5);
                        map_cloud.push_back(pt);
                    }
                }
        }

        // prepare map_lines
        {
            map_lines.clear();

            uchar* data = map_image.data;
            for(int y=0;y<map_image.rows;y++)
                for(int x=0;x<map_image.cols;x++,data++){
                    if(*data) continue;
                    Point3d p0   = origin * (resolution * Point3d(x ,y, 0));
                    Point3d p1   = origin * (resolution * Point3d(x+1 ,y, 0));
                    Point3d p2   = origin * (resolution * Point3d(x+1 ,y+1, 0));
                    Point3d p3   = origin * (resolution * Point3d(x ,y+1, 0));

                    map_lines.push_back(std::make_pair(p0, p1));
                    map_lines.push_back(std::make_pair(p1, p2));
                    map_lines.push_back(std::make_pair(p2, p3));
                    map_lines.push_back(std::make_pair(p3, p0));
                }
        }

        return !map_image.empty();
    }

    SE3 random_free_pose(){
        cv::Mat grids;
        int r = ceil(robot_radius / resolution) * 2 + 1;
        cv::erode(map_image, grids, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(r,r)));

        while(1){
            int x = rand()%map_image.cols;
            int y = rand()%map_image.rows;
            if(map_image.at<uchar>(y,x) != 255)
                continue;

            double yaw = drand48();

            return SE3(SO3::fromPitchYawRoll(0,yaw,0), origin * (resolution * Point3d(x ,y, 0)));
        }
    }

    void publish_map(){
        Svar info = {{"resolution",resolution},
                    {"width",map_image.cols},
                    {"height",map_image.rows},
                    {"origin",from_pose(origin)}};
        Svar msg = {{"header",{{"frame_id","map"}}},
                    {"info",info},
                    {"data", SvarBuffer(map_image.data, map_image.elemSize()*map_image.total(), map_image)}};

        pub_map.publish(msg);
    }

    void publish_map_cloud(Svar stamp){
        Svar header= {{"frame_id","map"},{"stamp",stamp}};
        sv::Svar cloud2={{"header",header},
                         {"height",int(map_cloud.size())},{"width",1},
                         {"is_bigendian",false},{"point_step",12},
                         {"row_step",12},
                         {"is_dense",true}};
        cloud2["data"] = sv::SvarBuffer(map_cloud.data(),map_cloud.size() * 12).clone(); // no copy
        cloud2["fields"]={{{"name","x"},{"offset",0},{"datatype",7},{"count",1}},
                          {{"name","y"},{"offset",4},{"datatype",7},{"count",1}},
                          {{"name","z"},{"offset",8},{"datatype",7},{"count",1}}};

        pub_map_cloud.publish(cloud2);
    }


    Svar from_pose(SE3& p){
        auto& t = p.get_translation();
        auto& r = p.get_rotation();
        Svar position = {{"x",&t.x},{"y",&t.y},{"z",&t.z}};
        Svar orientation = {{"x",&r.x},{"y",&r.y},{"z",&r.z},{"w",&r.w}};
        return {{"position",position},{"orientation",orientation}};
    }

    void cbk_pose(const Svar& msg){
        double sim_t = msg[0].as<double>();
        SE3    pose  = msg[1].as<SE3>();

        Svar tm = stamp(sim_t);

        if(sim_t - last_scan_t > min_scan_time_diff){
            last_scan_t = sim_t;

            pub_scan.publish(line_scan(pose * scan2baselink, tm));

            TinyTF::pub_tf(pub_tf, sim_t, scan2baselink,
                           "base_link_ground", "laser");
            DLOG(INFO)<<"scan:"<<tm;
        }

        if(sim_t - last_pub_t > 5){
            last_pub_t = sim_t;

            publish_map();
            publish_map_cloud(tm);
            DLOG(INFO)<<"map:"<<sim_t;
        }
    }

    Svar scan(SE3 scan2map, Svar stamp){// topic /scan
        std::vector<float> ranges(ranges_num, 100);
        double angle_min = -M_PI;
        double angle_increment = (M_PI*2)/ranges.size();
        double angle_max = angle_min + angle_increment *(ranges.size() - 1 );
        double time_increment = 0;
        double scan_time = 0;
        double range_min = 0.05;
        double range_max = 10;

        double band_width_inv = ranges.size() / (M_PI*2);

        SE3 pic2scan = scan2map.inverse() * origin;

        uchar* data = map_image.data;
        for(int y=0;y<map_image.rows;y++)
            for(int x=0;x<map_image.cols;x++,data++){
                if(*data) continue;
                Point3d pt   = pic2scan * (resolution * Point3d(x,y,0));
                double  rad  = atan2(pt.y,pt.x) - angle_min;
                int     band = rad * band_width_inv;
                float   depth = Point2d(pt.x,pt.y).norm();

                if(depth < ranges[band])
                    ranges[band] = depth;
            }

        Svar header= {{"frame_id","laser"},{"stamp",stamp}};
        Svar msg = {{"header", header},
                    {"ranges", SvarBuffer(ranges.data(), {1,(int)ranges.size()}).clone()},
                    {"angle_min", angle_min},{"angle_max", angle_max},
                    {"angle_increment",angle_increment},
                    {"time_increment",time_increment},
                    {"scan_time",scan_time},
                    {"range_min",range_min},{"range_max",range_max}};

        return msg;
    }

    Svar line_scan(SE3 scan2map, Svar stamp){// topic /scan
        std::vector<float> ranges(ranges_num, 100);
        double angle_min = -M_PI;
        double angle_increment = (M_PI*2)/ranges.size();
        double angle_max = angle_min + angle_increment *(ranges.size() - 1 );
        double time_increment = 0;
        double scan_time = 0;
        double range_min = 0.05;
        double range_max = 10;

        double band_width_inv = ranges.size() / (M_PI*2);

        SE3 map2scan = scan2map.inverse();
        for(auto& line:map_lines){
            auto p0 = map2scan * line.first;
            auto p1 = map2scan * line.second;

            auto r0 = atan2(p0.y,p0.x);
            auto r1 = atan2(p1.y,p1.x);

            if(fabs(r1 - r0) > M_PI){
                if(r1 > r0){
                    std::swap(p0,p1);
                    std::swap(r0,r1);
                }
            }
            else if(r1 < r0)
            {
                std::swap(p0,p1);
                std::swap(r0,r1);
            }

            if(r1 < r0)
            {
                r1 += 2* M_PI;
            }


            int band_min = (r0 + M_PI) * band_width_inv;
            int band_max = (r1 + M_PI) * band_width_inv;

            double d0 = p0.norm();
            double d1 = p1.norm();
            double dd = 0;

            if(band_max != band_min)
                dd = (d1 - d0)/(band_max - band_min);

            // TODO: 下面这个是近似解，只有当线段比较短时才成立
            for(int b = band_min;b<=band_max;b++){
                int real_b = b%ranges.size();
                double r = real_b * angle_increment + angle_min;
                double d = d0 + dd * (b-band_min);
                if(d < ranges[real_b])
                    ranges[real_b] = d;
            }
        }

        Svar header= {{"frame_id","laser"},{"stamp",stamp}};
        Svar msg = {{"header", header},
                    {"ranges", SvarBuffer(ranges.data(), {1,(int)ranges.size()}).clone()},
                    {"angle_min", angle_min},{"angle_max", angle_max},
                    {"angle_increment",angle_increment},
                    {"time_increment",time_increment},
                    {"scan_time",scan_time},
                    {"range_min",range_min},{"range_max",range_max}};

        return msg;
    }

    Svar stamp(double t){
        int sec = t;
        int nanosec = (t-sec)*1e9;
        return {{"sec",sec},{"nanosec",nanosec}};
    }

    Svar crash(SE3 pose){// 模拟前档碰撞


        return false;
    }

    cv::Mat   map_image;
    std::vector<Point3f>                    map_cloud;
    std::vector<std::pair<Point3d,Point3d>> map_lines;

    SE3       origin, scan2baselink;
    double    resolution, last_scan_t = 0, last_pub_t = 0, robot_radius, min_scan_time_diff;
    int       ranges_num;

    Subscriber sub_pose;
    Publisher  pub_map, pub_map_cloud, pub_scan, pub_tf;
};

REGISTER_SVAR_MODULE(world){
    Class<World>("World")
            .construct<Svar>()
            .def("random_free_pose",&World::random_free_pose);
}
