#ifndef GIMAGE
#define GIMAGE
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <atomic>
#include <memory>
#include <vector>
#include "Svar.h"

#if defined(HAS_OPENCV) || defined(HAS_OPENCV3)
#include <opencv2/core/core.hpp>
#else
typedef unsigned char uchar;
#endif

namespace GSLAM{


enum GElementType{
    GElementType_8U =0,
    GElementType_8S =1,
    GElementType_16U=2,
    GElementType_16S=3,
    GElementType_32S=4,
    GElementType_32F=5,
    GElementType_64F=6,
    GElementType_UserType=7
};

template <typename C>
class GElement
{
public:
    enum{Type=GElementType_UserType};
};

template <>
class GElement<uint8_t>
{
public:
    enum{Type=GElementType_8U};
};

template <>
class GElement<char>
{
public:
    enum{Type=GElementType_8S};
};

template <>
class GElement<int16_t>
{
public:
    enum{Type=GElementType_16S};
};

template <>
class GElement<uint16_t>
{
public:
    enum{Type=GElementType_16U};
};

template <>
class GElement<int32_t>
{
public:
    enum{Type=GElementType_32S};
};

template <>
class GElement<float>
{
public:
    enum{Type=GElementType_32F};
};

template <>
class GElement<double>
{
public:
    enum{Type=GElementType_64F};
};

template <typename EleType=uint8_t,int channelSize=1>
struct GImageType
{
    enum{Type=((GElement<EleType>::Type&0x7)+((channelSize-1)<<3))};
};

/**
 * @brief The GImage class is a tiny implementation of image for removing dependency of opencv.
 * Most APIs are corrosponding to "cv::Mat".
 */
class GImage
{
public:
    GImage()
        :cols(0),rows(0),flags(0),data(NULL)
    {

    }

    GImage(int rows_,int cols_,int type=GImageType<>::Type,uchar* src=NULL,bool copy=false, int align=16)
        :cols(cols_),rows(rows_),flags(type),data(NULL)
    {
        if(src&&!copy)
        {
            data=src;
            return;
        }

        int byteNum=total()*elemSize();
        if(byteNum<=0) return;
        std::shared_ptr<DefaultHolder> h(new DefaultHolder(byteNum,align));
        holder=h;
        data=h->data;
        if(!data)
        {
            cols=0;rows=0;return ;
        }
        if(src)
            memcpy(data,src,byteNum);
    }

    GImage(const GImage& ref)
        : cols(ref.cols),rows(ref.rows),flags(ref.flags),
          data(ref.data),holder(ref.holder)
    {}

    static GImage create(int rows,int cols,int type=GImageType<>::Type,uchar* src=NULL,bool copy=false)
    {
        return GImage(rows,cols,type,src,copy);
    }

    static GImage zeros(int rows,int cols,int type=GImageType<>::Type,uchar* src=NULL,bool copy=false)
    {
        GImage result(rows,cols,type,src,copy);
        memset(result.data,0,result.total()*result.elemSize());
        return result;
    }

    bool empty()const{return !data;}
    int  elemSize()const{return channels()*elemSize1();}
    int  elemSize1()const{return (1<<((type()&0x7)>>1));}

    int  channels()const{return (type()>>3)+1;}
    int  type()const{return flags;}
    int  total()const{return cols*rows;}

    GImage clone()const
    {
        return GImage(rows,cols,flags,data,true);
    }

    void release(){
        *this=GImage();
    }

    template <typename C>
    C& at(int idx){return ((C*)data)[idx];}

    template <typename C>
    C& at(int ix,int iy){return ((C*)data)[iy*cols+ix];}


#if defined(HAS_OPENCV)
    inline operator cv::Mat()const
    {
        if(holder.is<cv::Mat>())
            return holder.as<cv::Mat>();
        if(empty()) return cv::Mat();
        cv::Mat result(rows,cols,type(),data);
        return result;
    }

    GImage(const cv::Mat& mat)
        : cols(mat.cols),rows(mat.rows),flags(mat.type()),
          data(mat.data)
    {
        holder=mat;
    }
#endif

    template<typename _Tp> _Tp* ptr(int i0=0){return (_Tp*)(data+i0*cols*elemSize());}

    template<typename _Tp> const _Tp* ptr(int i0=0) const{return (_Tp*)(data+i0*cols*elemSize());}

    const GImage row(int idx=0)const{return GImage(1,cols,type(),data+elemSize()*cols*idx);}

    int getWidth()const{return cols;}
    int getHeight()const{return rows;}

private:

    struct DefaultHolder{
      DefaultHolder(size_t sz, int align):data((uchar*)fastMalloc(sz,align)){}

      ~DefaultHolder(){        fastFree(data);      }

      template<typename _Tp> static inline _Tp* alignPtr(_Tp* ptr, int n=(int)sizeof(_Tp))
      {
        return (_Tp*)(((size_t)ptr + n-1) & -n);
      }

      static void* fastMalloc( size_t size , int align)
      {
        uchar* udata = (uchar*)malloc(size + sizeof(void*) + align);
        if(!udata)
          return NULL;
        uchar** adata = alignPtr((uchar**)udata + 1, align);
        adata[-1] = udata;
        return adata;
      }

      static void fastFree(void* ptr)
      {
        if(ptr)
        {
          uchar* udata = ((uchar**)ptr)[-1];
          free(udata);
        }
      }

      static inline size_t alignSize(size_t sz, int n)
      {
        assert((n & (n - 1)) == 0); // n is a power of 2
        return (sz + n-1) & -n;
      }
      uchar*          data;
    };

public:
    int  cols,rows,flags;
    uchar*          data;
    sv::Svar        holder;
};

}
#endif // GIMAGE
