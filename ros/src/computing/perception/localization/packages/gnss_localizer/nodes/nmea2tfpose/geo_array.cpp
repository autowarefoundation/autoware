#include "nmea2tfpose_core_RTK.h"

namespace gnss_localizer
{

geo_array::geo_array(unsigned int arraySize)
{
    listSize=arraySize;
    counter=0;
    list=new geo_pos_conv[listSize];
    yaw_list=new double[listSize];
    lat_std=new double[listSize];
    lon_std=new double[listSize];
    alt_std=new double[listSize];
    surface_speed=new double[listSize];
    for(unsigned int i=0;i<listSize;i++)
    {
        yaw_list[i]=0; lat_std[i]=0; lon_std[0]=0; alt_std[0]=0; surface_speed[i]=0;
    }
}

geo_array::~geo_array()
{
    delete[] list;
    delete[] yaw_list;
    delete[] lat_std;
    delete[] lon_std;
    delete[] alt_std;
    delete[] surface_speed;
}

void geo_array::push_back(const geo_pos_conv &gpc, double latstd, double lonstd, double altstd, double surfacespeed)
{
    for(unsigned int i=listSize-1;i>0;i--)
    {
        list[i]=list[i-1];
        yaw_list[i]=yaw_list[i-1];
        lat_std[i]=lat_std[i-1];
        lon_std[i]=lon_std[i-1];
        alt_std[i]=alt_std[i-1];
        surface_speed[i]=surface_speed[i-1];
    }
    list[0]=gpc;
    lat_std[0]=latstd;
    lon_std[0]=lonstd;
    alt_std[0]=altstd;
    surface_speed[0]=surfacespeed;
    if(counter<listSize) counter++;
}

double geo_array::returnAngle_movingAverage(unsigned int range)
{
    if(counter<=range*2+1) return 0;

    double sumx=0,sumy=0;
    for(unsigned int i=range;i<counter-range;i++)
    {
        double ax=0,ay=0;
        for(unsigned int j=i-range;j<=i+range;j++)
        {
            ax+=list[j].x(); ay+=list[j].y();
        }
        sumx+=ax/(range*2+1); sumy+=ay/(range*2+1);
    }
    sumx/=counter-range*2; sumy/=counter-range*2;
    yaw_list[0]=atan2(list[0].x() - sumx, list[0].y() - sumy);

    double sumyaw=0;
    for(unsigned int i=range;i<counter-range;i++)
    {
        double ay=0;
        for(unsigned int j=i-range;j<=i+range;j++)
        {
            ay+=yaw_list[j];
        }
        sumyaw+=ay/(range*2+1);
    }
    return sumyaw/(counter-range*2);
}

geo_pos_conv geo_array::operator[](unsigned int index)
{
    return list[index];
}

}
