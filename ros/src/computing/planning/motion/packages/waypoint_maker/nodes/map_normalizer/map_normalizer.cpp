#include <ros/ros.h>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sys/types.h>
#include <dirent.h>
#include <unordered_map>
#define _USE_MATH_DEFINES
#include <math.h>//円周率用

struct Limit_Deviation
{
    double lat,lon,alt;
};

struct WayPoint
{
    struct
    {
        double np_x,np_y,np_z,nl_x,nl_y,nl_z;
        double np_yaw,velocity,change_flag;
    } ndt;
    struct
    {
        double rp_x,rp_y,rp_z,rl_x,rl_y,rl_z;
        double std_lat,std_lon,std_alt;
    } rtk;
};

class MapNormalizer
{
private:
    pcl::PCLPointCloud2 pcd;
    std::vector<WayPoint> waypoint, allWaypoint;
    std::string pcdPath;

    std::vector<std::string> split(const std::string &s, char delim) {
        std::vector<std::string> elems;
        std::stringstream ss(s);
        std::string item;
        while (getline(ss, item, delim)) {
        if (!item.empty()) {
                elems.push_back(item);
            }
        }
        return elems;
    }

    std::vector<double> split_double(const std::string &s, char delim) {
        std::vector<double> elems;
        std::stringstream ss(s);
        std::string item;
        while (getline(ss, item, delim)) {
        if (!item.empty()) {
                double d = std::stod(item.c_str());
                elems.push_back(d);
            }
        }
        return elems;
    }

    std::string pdcRead(std::string path_list)
    {
        std::vector<std::string> pathArray = split(path_list,',');
        int path_cou;
        for(path_cou = 0; path_cou < pathArray.size(); path_cou++)
        {
            std::string path = pathArray[path_cou]; std::cout<<path<<std::endl;
            if(path_cou==0)
            {
                if (pcl::io::loadPCDFile(path, pcd) == -1) {
                    std::stringstream error;
                    error << "error : Can not load [" << path << "]" << std::endl;
                    return error.str();
                }
            }
            else
            {
                pcl::PCLPointCloud2 part;
                if (pcl::io::loadPCDFile(path, part) == -1) {
                    std::stringstream error;
                    error << "error : Can not load [" << path << "]" << std::endl;
                    return error.str();
                }
                pcd.width += part.width;
                pcd.row_step += part.row_step;
                pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
            }
        }
        if(path_cou == 0)
        {
            std::string error = "There is no PCD data.";
            return error;
        }
        return std::string("OK");
    }

    void csvRead(std::string path_list, Limit_Deviation limit_deviation)
    {
        std::vector<std::string> pathArray = split(path_list,',');
        int path_cou;
        for(path_cou = 0; path_cou < pathArray.size(); path_cou++)
        {
            std::string path = pathArray[path_cou]; std::cout<<path<<std::endl;

            std::ifstream ifs(path);
            if(!ifs.is_open())
            {
                std::string str = "error : no path : " + path;
                throw str;
            }

            std::string line;
            std::getline(ifs, line);
            std::vector<std::string> field_line = split(line, ',');
            for(int data_cou = 0; !ifs.eof(); data_cou++)
            {
                std::getline(ifs, line);
                std::vector<double> data_line = split_double(line, ',');
                if(data_line.size() != field_line.size()) continue;

                std::unordered_map<std::string, double> map;
                for(int i=0; i<field_line.size();i++)
                    map[field_line.at(i)] = data_line.at(i);

                WayPoint wp;
                if(map.find("np_x") != map.end()) wp.ndt.np_x = map["np_x"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [np_x] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("np_y") != map.end()) wp.ndt.np_y = map["np_y"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [np_y] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("np_z") != map.end()) wp.ndt.np_z = map["np_z"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [np_z] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("np_yaw") != map.end()) wp.ndt.np_yaw = map["np_yaw"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [np_yaw] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("velocity") != map.end()) wp.ndt.velocity = map["velocity"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [velocity] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("change_flag") != map.end()) wp.ndt.change_flag = map["change_flag"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [change_flag] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("nl_x") != map.end()) wp.ndt.nl_x = map["nl_x"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [nl_x] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("nl_y") != map.end()) wp.ndt.nl_y = map["nl_y"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [nl_y] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("nl_z") != map.end()) wp.ndt.nl_z = map["nl_z"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [nl_z] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("rp_x") != map.end()) wp.rtk.rp_x = map["rp_x"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [rp_x] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("rp_y") != map.end()) wp.rtk.rp_y = map["rp_y"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [rp_y] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("rp_z") != map.end()) wp.rtk.rp_z = map["rp_z"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [rp_z] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("rl_x") != map.end()) wp.rtk.rl_x = map["rl_x"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [rl_x] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("rl_y") != map.end()) wp.rtk.rl_y = map["rl_y"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [rl_y] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("rl_z") != map.end()) wp.rtk.rl_z = map["rl_z"];
                else
                {
                    std::stringstream error; ifs.close();
                    error << "error : no field [rl_z] : line " << data_cou;
                    throw error.str();
                }
                if(map.find("std_lat") != map.end()) wp.rtk.std_lat = map["std_lat"];
                else { std::cout << "warning : There is no std_lat.  set 5 : " << data_cou ; wp.rtk.std_lat = 5;}
                if(map.find("std_lon") != map.end()) wp.rtk.std_lon = map["std_lon"];
                else { std::cout << "warning : There is no std_lon.  set 5 : " << data_cou ; wp.rtk.std_lon = 5;}
                if(map.find("std_alt") != map.end()) wp.rtk.std_alt = map["std_alt"];
                else { std::cout << "warning : There is no std_alt.  set 5 : " << data_cou ; wp.rtk.std_alt = 5;}

                allWaypoint.push_back(wp);
                if(wp.rtk.std_lat <= limit_deviation.lat &&
                   wp.rtk.std_lon <= limit_deviation.lon &&
                   wp.rtk.std_alt <= limit_deviation.alt)
                {
                    std::cout<<data_cou<<" : OK"<<std::endl;
                    waypoint.push_back(wp);
                }
                else
                {
                    std::cout<<data_cou<<" : Limit daviation over"<<std::endl;
                    continue;
                }
            }

            if(waypoint.size() == 0)
            {
                std::string str = "There is no waypoint selected.";
                throw str;
            }
            ifs.close();
        }
        if(path_cou == 0)
        {
            throw std::string("error : no csv data");
        }
    }
public:
    MapNormalizer(std::string load_map_name, std::string load_csv_name, Limit_Deviation limit_deviation)
    {
        try{
            csvRead(load_csv_name, limit_deviation);
            pcdPath = load_map_name;
            //pdcRead(load_map_name);
        }
        catch(std::string str) { throw str; }
    }

    std::string transform(std::string save_map_name, std::string save_csv_name)
    {
        int rows = waypoint.size()*2;
        Eigen::MatrixXd map(rows,3),gnss(rows,3);
        Eigen::VectorXd avemap(3),avegnss(3);
        avemap.setZero(); avegnss.setZero();
        for(int i=0; i<rows/2; i++)
        {
            std::vector<std::string> xyz={"x:","y:","z:"};
            map(i,0) = waypoint[i].ndt.np_x; map(i,1) = waypoint[i].ndt.np_y; map(i,2) = waypoint[i].ndt.np_z;
            map(i+rows/2,0) = waypoint[i].ndt.nl_x; map(i+rows/2,1) = waypoint[i].ndt.nl_y; map(i+rows/2,2) = waypoint[i].ndt.nl_z;
            gnss(i,0) = waypoint[i].rtk.rp_x; gnss(i,1) = waypoint[i].rtk.rp_y; gnss(i,2) = waypoint[i].rtk.rp_z;
            gnss(i+rows/2,0) = waypoint[i].rtk.rl_x; gnss(i+rows/2,1) = waypoint[i].rtk.rl_y; gnss(i+rows/2,2) = waypoint[i].rtk.rl_z;
            for(int j=0; j<3; j++)
            {
                std::cout<<i<<" : "<<std::setprecision(16)<<xyz[j]<<map(i,j)<<",";
                std::cout<<std::setprecision(16)<<xyz[j]<<gnss(i,j)<<std::endl;
                std::cout<<i<<" : "<<std::setprecision(16)<<xyz[j]<<map(i+rows/2,j)<<",";
                std::cout<<std::setprecision(16)<<xyz[j]<<gnss(i+rows/2,j)<<std::endl;
                avemap(j)+=map(i,j)+map(i+rows/2,j);
                avegnss(j)+=gnss(i,j)+gnss(i+rows/2,j);
            }
            std::cout<<std::endl;
        }
        for(int i=0;i<3;i++) {avemap(i)/=rows; avegnss(i)/=rows;}
        std::cout<<"avemap:"<<avemap(0)<<","<<avemap(1)<<","<<avemap(2)<<std::endl;
        std::cout<<"avegnss:"<<avegnss(0)<<","<<avegnss(1)<<","<<avegnss(2)<<std::endl<<std::endl;

        Eigen::MatrixXd map_ma(rows,3),gnss_ma(rows,3);
        Eigen::VectorXd plusmap(3),plusgnss(3);
        for(int i=0;i<rows;i++)
        {
            std::vector<std::string> xyz={"x:","y:","z:"};
            for(int j=0;j<3;j++)
            {
                map_ma(i,j)=map(i,j)-avemap(j); std::cout<<xyz[j]<<map_ma(i,j)<<",";
                gnss_ma(i,j)=gnss(i,j)-avegnss(j);std::cout<<xyz[j]<<gnss_ma(i,j)<<",";
                plusmap(j)+=map_ma(i,j); plusgnss(j)+=gnss_ma(i,j);
                std::cout<<std::endl;
            }
        }
        std::cout<<std::endl;

        std::cout<<"plusmap:"<<plusmap(0)<<","<<plusmap(1)<<","<<plusmap(2)<<std::endl;
        std::cout<<"plusgnss:"<<plusgnss(0)<<","<<plusgnss(1)<<","<<plusgnss(2)<<std::endl<<std::endl;

        Eigen::MatrixXd &d=gnss_ma,&m=map_ma;
        Eigen::VectorXd &d_ave=avegnss,&m_ave=avemap;
        Eigen::MatrixXd H=m.transpose()*d;
        for(int i=0;i<H.rows();i++)
        {
            for(int j=0;j<H.cols();j++) std::cout<<H(i,j)<<",";
            std::cout<<std::endl;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H,Eigen::ComputeFullU | Eigen::ComputeFullV);
        std::cout<<"svd:OK"<<std::endl<<std::endl;
        Eigen::MatrixXd U=svd.matrixU(),V=svd.matrixV();
        std::cout << "singular values" << std::endl
                  << svd.singularValues() << std::endl;
        std::cout << "matrix U" << std::endl << U << std::endl;
        std::cout << "matrix V" << std::endl << V << std::endl<<std::endl;

        Eigen::MatrixXd R=V*U.transpose();
        std::cout << "matrix R" << std::endl << R << std::endl<<std::endl;

        Eigen::VectorXd T=d_ave-R*m_ave;
        std::cout << "matrix T" << std::endl << T << std::endl<<std::endl;

        std::ofstream ofs_csv(save_csv_name);
        ofs_csv << "x,y,z,yaw,velocity,change_flag" << std::endl;
        for(int i=0;i<allWaypoint.size();i++)
        {
            Eigen::VectorXd mi=m.row(i);
            for(int j=0;j<3;j++) mi(j)+=m_ave(j);
            //Eigen::VectorXd g=gnss.row(i);
            Eigen::VectorXd g(3);
            g(0) = allWaypoint[i].rtk.rp_x; g(1) = allWaypoint[i].rtk.rp_y; g(2) = allWaypoint[i].rtk.rp_z;
            Eigen::VectorXd rmi=R*mi;
            Eigen::VectorXd di=rmi+T;
            std::vector<std::string> xyz={"x:","y:","z:"};
            for(int j=0;j<3;j++)
            {
                ofs_csv << std::setprecision(16) << di(j) << ",";
            }
            ofs_csv << allWaypoint[i].ndt.np_yaw << "," << allWaypoint[i].ndt.velocity << "," << allWaypoint[i].ndt.change_flag << std::endl;
        }

        ofs_csv.close();

        std::string result = pdcRead(pcdPath);
        if(result != "OK") return result;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromPCLPointCloud2 (pcd, *cloud);
        std::cout << "pcd OK" << std::endl;
        for(int i=0; i<cloud->points.size(); i++)
        {
            Eigen::VectorXd mi(3);
            mi(0) = cloud->points[i].x;
            mi(1) = cloud->points[i].y;
            mi(2) = cloud->points[i].z;
            Eigen::VectorXd rmi=R*mi;
            Eigen::VectorXd di=rmi+T;
            cloud->points[i].x = di(0);
            cloud->points[i].y = di(1);
            cloud->points[i].z = di(2);
        }
        pcl::io::savePCDFileBinary<pcl::PointXYZI>(save_map_name,*cloud);

        return std::string("transform OK");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_normalizer");
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_("~");

    std::string load_map_name, load_csv_name, save_map_name, save_csv_name;
    private_nh_.param<std::string>("load_map_name", load_map_name, std::string(""));
    private_nh_.param<std::string>("load_csv_name", load_csv_name, std::string(""));
    private_nh_.param<std::string>("save_map_name", save_map_name, std::string("/tmp/normalize_map.pcd"));
    private_nh_.param<std::string>("save_csv_name", save_csv_name, std::string("/tmp/normalize_path.csv"));

    Limit_Deviation limit_dev;
    private_nh_.param<double>("limit_deviation_lat", limit_dev.lat, 0);
    private_nh_.param<double>("limit_deviation_lon", limit_dev.lon, 0);
    private_nh_.param<double>("limit_deviation_alt", limit_dev.alt, 0);
    std::cout << "Limit_Deviationes" << std::endl;
    std::cout << "    lat : " << limit_dev.lat << std::endl;
    std::cout << "    lon : " << limit_dev.lon << std::endl;
    std::cout << "    alt : " << limit_dev.alt << std::endl;

    try{
        MapNormalizer norm(load_map_name, load_csv_name, limit_dev);
        std::string result = norm.transform(save_map_name, save_csv_name);
        if(result == "transform OK") std::cout << result << std::endl;
        else std::cerr << result << std::endl;
    }
    catch(std::string error)
    {
        std::cerr << error << std::flush << std::endl;
        exit(-1);
    }


    return 0;
}
