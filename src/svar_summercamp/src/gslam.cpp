#include "GSLAM/core/GSLAM.h"
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace sv;

namespace GSLAM{

template <typename T>
void def_matrix(std::string name){
    sv::Class<T>(name)
//            .construct<>()
            .def("__init__",[](SvarBuffer buf){
        T ret;
        memcpy(ret.data(),buf.ptr(),buf.size());
        return ret;
    })
            .def("__buffer__",[](T& self){
        return SvarBuffer(self.data(),{self.rows(),self.cols()});
    })
            .def("__repr__",[](T& self){
        std::stringstream sst;
        sst<<self;
        return sst.str();
    });
}

SO3 rotation_from_abc(double a,double b,double c){
    auto A=SO3::fromPitchYawRollAngle(0.,0.,a);
    auto B=SO3::fromPitchYawRollAngle(b,0.,0.);
    auto C=SO3::fromPitchYawRollAngle(0.,c,0.);
    return C*B*A;
}

std::vector<double> abc_from_rotation(SO3 rotation){
    Matrix3d m = rotation.getMatrix();
    double b = asin( - m(2,0)),a,c;
    double to_degree = 180./ M_PI;

    if ( abs( m(2,0) ) < 0.99999 ) {

        a = atan2( m(2,1), m(2,2) );
        c = atan2( m(1,0), m(0,0) );

    } else {

        a = 0;
        c = atan2( - m(0,1), m(1,1) );
    }

    return {a*to_degree,b*to_degree,c*to_degree};
}

REGISTER_SVAR_MODULE(gslam_builtin) {
    svar["__doc__"]="This is the python APIs for GSLAM "+string(GSLAM_VERSION_STR)
            +"(https://github.com/zdzhaoyong/GSLAM)";

    def_matrix<Matrix2d>("Matrix2d");
    def_matrix<Matrix3d>("Matrix3d");
    def_matrix<Matrix4d>("Matrix4d");

    svar["rotation_from_abc"] = rotation_from_abc;
    svar["abc_from_rotation"] = abc_from_rotation;

    Class<Point2d>("Point2d")
            .construct<>()
            .construct<double,double>()
            .def("dot",&Point2d::dot)
            .def("norm",&Point2d::norm)
            .def("at",&Point2d::at)
            .def("__repr__",&Point2d::toString)
            .def("__str__",&Point2d::toString)
            .def("__add__",&Point2d::add)
            .def("__sub__",&Point2d::sub)
            .def("__mul__",&Point2d::mul)
            .def("__div__",&Point2d::div)
            .def_readwrite("x", &Point2d::x)
            .def_readwrite("y", &Point2d::y)
        ;

    Class<Point3d>("Point3d")
            .construct<>()
            .construct<double,double,double>()
            .def("dot",&Point3d::dot)
            .def("norm",&Point3d::norm)
            .def("at",&Point3d::at)
            .def("__repr__",&Point3d::toString)
            .def("__str__",&Point3d::toString)
            .def("__add__",&Point3d::add)
            .def("__sub__",&Point3d::sub)
            .def("__mul__",&Point3d::mul)
            .def("__div__",&Point3d::div)
            .def_readwrite("x", &Point3d::x)
            .def_readwrite("y", &Point3d::y)
            .def_readwrite("z", &Point3d::z);

    Class<Point3ub>("Point3ub")
            .construct<>()
            .construct<uchar,uchar,uchar>()
            .def("dot",&Point3ub::dot)
            .def("norm",&Point3ub::norm)
            .def("at",&Point3ub::at)
            .def("__repr__",&Point3ub::toString)
            .def("__str__",&Point3ub::toString)
            .def("__add__",&Point3ub::add)
            .def("__sub__",&Point3ub::sub)
            .def("__mul__",&Point3ub::mul)
            .def("__div__",&Point3ub::div)
            .def_readwrite("x", &Point3ub::x)
            .def_readwrite("y", &Point3ub::y)
            .def_readwrite("z", &Point3ub::z);

    Class<Camera>("Camera")
            .construct<>()
            .construct<std::vector<double> >()
            .def("CameraType",&Camera::CameraType)
            .def("info",&Camera::info)
            .def("isValid",&Camera::isValid)
            .def("width",&Camera::width)
            .def("height",&Camera::height)
            .def("getParameters",&Camera::getParameters)
            .def("estimatePinHoleCamera",&Camera::estimatePinHoleCamera)
            .def("Project",(Point2d (Camera::*)(const Point3d&)const) &Camera::Project,
                 "Project a point from camera coordinate to image coordinate")
            .def("UnProject",(Point3d (Camera::*)(const Point2d&)const) &Camera::UnProject,"");

    Class<SO3>("SO3")
            .construct<>()
            .construct<double,double,double,double>()
            .construct<const Point3d&,double>()
            .construct<const double*>()
            .def("log",&SO3::log)
            .def_static("exp",&SO3::exp<double>)
            .def_static("fromPitchYawRollAngle",&SO3::fromPitchYawRollAngle)
            .def("normalise",&SO3::normalise)
            .def("getMatrix",(Matrix<double,3,3> (SO3::*)()const)&SO3::getMatrix)
            .def("__mul__",&SO3::mul)
            .def("trans",&SO3::trans)
            .def("inverse",&SO3::inverse)
            .def("__repr__",&SO3::toString)
            .def_property("yaw",&SO3::getYaw)
            .def_property("pitch",&SO3::getPitch)
            .def_property("roll",&SO3::getRoll)
            .def_readwrite("x", &SO3::x)
            .def_readwrite("y", &SO3::y)
            .def_readwrite("z", &SO3::z)
            .def_readwrite("w", &SO3::w)
            ;

    Class<SE3>("SE3")
            .construct<>()
            .construct<const SO3&,const Point3d&>()
            .def("__init__",[](std::vector<double> v){
        if(v.size()!=7)
            LOG(FATAL)<<"SE3 need 7 parameters";
        return SE3(v[0],v[1],v[2],v[3],v[4],v[5],v[6]);
    })
       .def("__init__",[](SvarBuffer buf){
            SE3 ret;
            ret.fromMatrix(buf.ptr<double>());
            return ret;
    })
            .def("log",&SE3::log)
            .def("inverse",&SE3::inverse)
            .def_static("exp",&SE3::exp<double>)
            .def("__mul__",&SE3::mul)
            .def("trans",&SE3::trans)
            .def("toString",&SE3::toString)
            .def("__repr__",&SE3::toString)
            .def_property("translation", &SE3::getTranslation, &SE3::setTranslation)
            .def_property("rotation", &SE3::getRotation, &SE3::setRotation)
            .def("fromMatrix",[](SE3* self, Matrix4d m){
        self->fromMatrix(m.data());
    })
            .def("getMatrix",[](SE3& self){
        Matrix4d m;
        m.identity();
        self.getMatrix(m.data());
        return m;
    })
            ;

    Class<SIM3>("SIM3")
            .construct<>()
            .construct<const SE3&,const double&>()
            .def_property("translation", &SE3::getTranslation, &SE3::setTranslation)
            .def_property("rotation", &SE3::getRotation, &SE3::setRotation)
            ;

    Class<GImage>("GImage")
            .construct<>()
            .construct<int,int,int,uchar*,bool>()
            .def("empty",&GImage::empty)
            .def_readonly("data",&GImage::data)
            .def("elemSize",&GImage::elemSize)
            .def("elemSize1",&GImage::elemSize1)
            .def("channels",&GImage::channels)
            .def("type",&GImage::type)
            .def("total",&GImage::total)
            .def("clone",&GImage::clone)
            .def("row",&GImage::row)
            .def("width",&GImage::getWidth)
            .def("height",&GImage::getHeight)
            .def("reshape",[](GImage self,int width,int height){
        assert(self.total()==width*height);
        self.cols=width;
        self.rows=height;
        return self;
})
            .def("__repr__",[](const GImage& img){
                return to_string(img.cols)+"x"
                        +to_string(img.rows)+"x"+to_string(img.channels());})
            .def("__init__",[](SvarBuffer buffer){
        if(buffer._holder.is<GImage>())
            return buffer._holder.as<GImage>();
        std::string format=buffer._format;
        std::vector<ssize_t> shape=buffer.shape;
        int channels=1;
        if(shape.size()==2)
            channels=1;
        else if(shape.size()==3)
            channels=shape[2];
        else
            LOG(FATAL)<<"Shape should be 2 or 3 demisson";

        int rows=shape[0];
        int cols=shape[1];

        static int lut[256]={0};
        lut['b']=0;lut['B']=1;
        lut['h']=2;lut['H']=3;
        lut['i']=4;
        lut['f']=5;lut['d']=6;
        int type=lut[format.front()]+((channels-1)<<3);
        return GImage(rows,cols,type,(uchar*)buffer.ptr(),true);
    })
    .def("__buffer__",[](GImage& self){
        std::string formats[]={"b","B","h","H","i","f","d"};
        std::string format   =formats[(self.type()&0x7)];
        return SvarBuffer(self.data,self.elemSize1(),format,{self.rows,self.cols,self.channels()},{},self.holder);
    })
    .def("save",[](GImage& self,std::string filepath){
        cv::imwrite(filepath,(cv::Mat)self);
    })
    .def_static("imread",[](std::string filepath,int flag){
        return (GImage)cv::imread(filepath,flag);
    });

    Class<TicToc>("TicToc")
            .construct<>()
            .def("timestamp",&TicToc::timestamp)
            .def("tic",&TicToc::Tic)
            .def("toc",&TicToc::Toc)
            ;

    Class<Timer>("Timer")
//            .construct<>()
            .def("__init__",[](){return std::make_shared<Timer>();})
            .def("enter",&Timer::enter)
            .def("leave",&Timer::leave)
            .def("disable",&Timer::disable)
            .def("enable",&Timer::enable)
            .def("getMeanTime",&Timer::getMeanTime)
            .def("getStatsAsText",&Timer::getStatsAsText)
            .def("dumpAllStats",&Timer::dumpAllStats)
            ;

        Class<MapPoint>("MapPoint")
            .def("id",&MapPoint::id)
            .def("getPose",&MapPoint::getPose)
            .def("setPose",&MapPoint::setPose)
            .def("getNormal",&MapPoint::getNormal)
            .def("setNormal",&MapPoint::setNormal)
            .def("getColor",&MapPoint::setColor)
            .def("getDescriptor",&MapPoint::getDescriptor)
            .def("isPoseRelative",&MapPoint::isPoseRelative)
            .def("refKeyframeID",&MapPoint::refKeyframeID)
            .def("refKeyframe",&MapPoint::refKeyframe)
            .def("observationNum",&MapPoint::observationNum)
            .def("getObservations",(MapPointObsVec (MapPoint::*)()const) &MapPoint::getObservations)
            .def("eraseObservation",&MapPoint::eraseObservation)
            .def("clearObservation",&MapPoint::clearObservation)
            .def_property("pose", &MapPoint::getPose, &MapPoint::setPose)
            .def_property("normal", &MapPoint::getNormal, &MapPoint::setNormal)
            .def_property("color", &MapPoint::getColor, &MapPoint::setColor)
            .def_property("descriptor", &MapPoint::getDescriptor, &MapPoint::setDescriptor)
            ;

    Class<FrameConnection>("FrameConnection")
            .def("matchesNum",&FrameConnection::matchesNum)
            .def("getInformation",&FrameConnection::getInformation)
            .def("setMatches",&FrameConnection::setMatches)
            .def("setInformation",&FrameConnection::setInformation)
            .def("getMatches",(std::vector<std::pair<int,int> >(FrameConnection::*)())&FrameConnection::getMatches)
            ;

    Class<MapFrame>("MapFrame")
            .def("id",&MapFrame::id)
            .def("timestamp",&MapFrame::timestamp)
            .def("setPose",(void (MapFrame::*)(const SE3&))&MapFrame::setPose)
            .def("setPoseSim3",(void (MapFrame::*)(const SIM3&))&MapFrame::setPose)
            .def("getPose",(SE3(MapFrame::*)()const)&MapFrame::getPose)
            .def("getPoseScale",&MapFrame::getPoseScale)
            .def("cameraNum",&MapFrame::cameraNum)
            .def("getCameraPose",&MapFrame::getCameraPose)
            .def("imageChannels",&MapFrame::imageChannels)
            .def("getCamera",&MapFrame::getCamera)
            .def("getImage",&MapFrame::getImage)
            .def("setImage",&MapFrame::setImage)
            .def("setCamera",&MapFrame::setCamera)
            .def("getIMUNum",&MapFrame::getIMUNum)
            .def("getIMUPose",&MapFrame::getIMUPose)
            .def("getAcceleration",&MapFrame::getAcceleration)
            .def("getAngularVelocity",&MapFrame::getAngularVelocity)
            .def("getMagnetic",&MapFrame::getMagnetic)
            .def("getAccelerationNoise",&MapFrame::getAccelerationNoise)
            .def("getAngularVNoise",&MapFrame::getAngularVNoise)
            .def("getPitchYawRoll",&MapFrame::getPitchYawRoll)
            .def("getPYRSigma",&MapFrame::getPYRSigma)
            .def("getGPSNum",&MapFrame::getGPSNum)
            .def("getGPSPose",&MapFrame::getGPSPose)
            .def("getGPSLLA",&MapFrame::getGPSLLA)
            .def("getGPSLLASigma",&MapFrame::getGPSLLASigma)
            .def("getGPSECEF",&MapFrame::getGPSECEF)
            .def("getHeight2Ground",&MapFrame::getHeight2Ground)
            .def("getGPSECEF",&MapFrame::getGPSECEF)
            .def("getHeight2Ground",&MapFrame::getHeight2Ground)
            .def("keyPointNum",&MapFrame::keyPointNum)
            .def("setKeyPoints",&MapFrame::setKeyPoints)
            .def("getKeyPoints",(std::vector<KeyPoint>(MapFrame::*)() const)&MapFrame::getKeyPoints)
            .def("getKeyPointColor",&MapFrame::getKeyPointColor)
            .def("getKeyPointIDepthInfo",&MapFrame::getKeyPointIDepthInfo)
            .def("getKeyPointObserve",&MapFrame::getKeyPointObserve)
            .def("getDescriptor",&MapFrame::getDescriptor)
            .def("getBoWVector",(BowVector (MapFrame::*)()const)&MapFrame::getBoWVector)
            .def("getFeatureVector",(FeatureVector (MapFrame::*)()const)&MapFrame::getFeatureVector)
            .def("getFeaturesInArea",&MapFrame::getFeaturesInArea)
            .def("observationNum",&MapFrame::observationNum)
            .def("getObservations",(std::map<GSLAM::PointID,size_t>(MapFrame::*)()const)&MapFrame::getObservations)
            .def("addObservation",&MapFrame::addObservation)
            .def("eraseObservation",&MapFrame::eraseObservation)
            .def("clearObservations",&MapFrame::clearObservations)
            .def("getParent",&MapFrame::getParent)
            .def("getChild",&MapFrame::getChild)
            .def("getParents",(FrameConnectionMap (MapFrame::*)()const)&MapFrame::getParents)
            .def("getChildren",(FrameConnectionMap (MapFrame::*)()const)&MapFrame::getChildren)
            .def("addParent",&MapFrame::addParent)
            .def("addChildren",&MapFrame::addChildren)
            .def("eraseParent",&MapFrame::eraseParent)
            .def("eraseChild",&MapFrame::eraseChild)
            .def("clearParents",&MapFrame::clearParents)
            .def("clearChildren",&MapFrame::clearChildren)
            .def("getMedianDepth",&MapFrame::getMedianDepth)
            .def("channelTypeString",&MapFrame::channelTypeString)
            .def("channelString",&MapFrame::channelString)
            ;

   Class<Dataset>("Dataset")
            .construct<>()
            .construct<const std::string&>()
            .def("name",&Dataset::name)
            .def("type",&Dataset::type)
            .def("isOpened",&Dataset::isOpened)
            .def("grabFrame",&Dataset::grabFrame)
            .def("open",&Dataset::open)
            ;

    Class<Map>("Map")
            .def("insertMapPoint",&Map::insertMapPoint)
            .def("insertMapFrame",&Map::insertMapFrame)
            .def("eraseMapPoint",&Map::eraseMapPoint)
            .def("eraseMapFrame",&Map::eraseMapFrame)
            .def("clear",&Map::clear)
            .def("frameNum",&Map::frameNum)
            .def("pointNum",&Map::pointNum)
            .def("getFrame",&Map::getFrame)
            .def("getPoint",&Map::getPoint)
            .def("getFrames",(FrameArray(Map::*)()const)&Map::getFrames)
            .def("getPoints",(PointArray(Map::*)()const)&Map::getPoints)
            .def("setLoopDetector",&Map::setLoopDetector)
            .def("getLoopDetector",&Map::getLoopDetector)
            .def("obtainCandidates",(LoopCandidates(Map::*)(const FramePtr&))&Map::obtainCandidates)
            .def("save",&Map::save)
            .def("load",&Map::load)
            .def("getPid",&Map::getPid)
            .def("getFid",&Map::getFid)
            ;

//    py::class_<HashMap,Map,SPtr<HashMap> >(m,"HashMap")
//            .construct<>())
//            ;

    Class<Vocabulary>("Vocabulary")
            .construct<const std::string &>()
            .def_static("create",&Vocabulary::create)
            .def("save",&Vocabulary::save)
            .def("load",(bool(Vocabulary::*)(const std::string &))&Vocabulary::load)
            .def("size",&Vocabulary::size)
            .def("empty",&Vocabulary::empty)
            .def("clear",&Vocabulary::clear)
            .def("transformImage",(void (Vocabulary::*)(const TinyMat&,
                 BowVector &, FeatureVector &, int)const)&Vocabulary::transform)
            .def("transformFeature",(void (Vocabulary::*)(const TinyMat &,
                 WordId &, WordValue &, NodeId*, int) const)&Vocabulary::transform)
            .def("getBranchingFactor",&Vocabulary::getBranchingFactor)
            .def("getDepthLevels",&Vocabulary::getDepthLevels)
            .def("getWord",&Vocabulary::getWord)
            .def("getWordWeight",&Vocabulary::getWordWeight)
            .def("getWeightingType",&Vocabulary::getWeightingType)
            .def("getScoringType",&Vocabulary::getScoringType)
            .def("setWeightingType",&Vocabulary::setWeightingType)
            .def("getDescritorSize",&Vocabulary::getDescritorSize)
            .def("getDescritorType",&Vocabulary::getDescritorType)
            .def_static("meanValue",&Vocabulary::meanValue)
            .def_static("distance",&Vocabulary::distance)
            ;

    Class<FileResource>("FileResource")
            .def_static("toHex",&FileResource::toHex)
            .def_static("exportResourceFile",&FileResource::exportResourceFile)
            .def_static("Register",&FileResource::Register)
            .def_static("saveResource2File",&FileResource::saveResource2File)
            ;

    Class<Undistorter>("Undistorter")
            .construct<Camera, Camera>()
            .def("undistort",(GImage (Undistorter::*)(const GImage& image))&Undistorter::undistort)
            .def("undistortFast",(GImage (Undistorter::*)(const GImage& image))&Undistorter::undistortFast)
            .def("cameraIn",&Undistorter::cameraIn)
            .def("cameraOut",&Undistorter::cameraOut)
            .def("prepareReMap",&Undistorter::prepareReMap)
            .def("valid",&Undistorter::valid)
            ;

    Class<Messenger>("Messenger")
            .construct<>()
            .def_static("instance",&Messenger::instance)
            .def("getPublishers",&Messenger::getPublishers)
            .def("getSubscribers",&Messenger::getSubscribers)
            .def("introduction",&Messenger::introduction)
            .def("advertise",[](Messenger msg,const std::string& topic,int queue_size){
        return msg.advertise<sv::Svar>(topic,queue_size);
    })
    .def("subscribe",[](Messenger msger,
         const std::string& topic, int queue_size,
         const SvarFunction& callback){
        return msger.subscribe(topic,queue_size,callback);
    })
    .def("publish",[](Messenger* msger,std::string topic,sv::Svar msg){return msger->publish(topic,msg);});

    Class<Publisher>("Publisher")
            .def("shutdown",&Publisher::shutdown)
            .def("getTopic",&Publisher::getTopic)
            .def("getTypeName",&Publisher::getTypeName)
            .def("getNumSubscribers",&Publisher::getNumSubscribers)
            .def("publish",[](Publisher* pubptr,sv::Svar msg){return pubptr->publish(msg);});

    Class<Subscriber>("Subscriber")
            .def("shutdown",&Subscriber::shutdown)
            .def("getTopic",&Subscriber::getTopic)
            .def("getTypeName",&Subscriber::getTypeName)
            .def("getNumPublishers",&Subscriber::getNumPublishers);

    svar["messenger"]=Messenger::instance();

}


}
