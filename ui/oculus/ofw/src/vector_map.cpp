//
//  vector_map.cpp
//  ORR_PointCloud
//
//  Created by Kenjiro on 6/6/15.
//
//

#include "vector_map.h"

typedef std::vector<std::vector<std::string> > File_contents;

static inline File_contents read_csv(const char* filename){
    File_contents fileContents;
    fileContents.clear();

    std::ifstream ifs(filename);
    if (ifs == 0){
        ifs.close();
        return fileContents;
    }
    std::string line_contents;
    std::getline(ifs, line_contents); // skip first header line

    while (std::getline(ifs, line_contents))
    {
        std::istringstream ss(line_contents);

        std::vector<std::string> column;
        std::string element;
        while (std::getline(ss, element, ',')) {
            column.push_back(element);
        }
        fileContents.push_back(column);
    }
    ifs.close();
    return fileContents;
}

int VectorMap::read_roadedge(const char* filename){
        File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        RoadEdge tmp_l;
        tmp_l.id     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.lid    = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.linkid = atoi((const char*)(fileContents[i][2].c_str()));

        roadedges.insert( std::map<int, RoadEdge>::value_type(tmp_l.id, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_gutter(const char* filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        Gutter tmp_l;
        tmp_l.id     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.aid    = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.type   = atoi((const char*)(fileContents[i][2].c_str()));
        tmp_l.linkid = atoi((const char*)(fileContents[i][3].c_str()));

        gutters.insert( std::map<int, Gutter>::value_type(tmp_l.id, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_curb(const char* filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        Curb tmp_l;
        tmp_l.id      = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.lid     = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.height  = atof((const char*)(fileContents[i][2].c_str()));
        tmp_l.width   = atof((const char*)(fileContents[i][3].c_str()));
        tmp_l.dir     = atoi((const char*)(fileContents[i][4].c_str()));
        tmp_l.linkid  = atoi((const char*)(fileContents[i][5].c_str()));

        curbs.insert( std::map<int, Curb>::value_type(tmp_l.id, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_whiteline(const char* filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        WhiteLine tmp_l;
        tmp_l.id     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.lid    = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.width  = atof((const char*)(fileContents[i][2].c_str()));
        tmp_l.color  = fileContents[i][3].c_str()[0];
        tmp_l.type   = atoi((const char*)(fileContents[i][4].c_str()));
        tmp_l.linkid = atoi((const char*)(fileContents[i][5].c_str()));

        //clines[tmp_l.id] = tmp_l;
        whitelines.insert( std::map<int, WhiteLine>::value_type(tmp_l.id, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_stopline(const char* filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        StopLine tmp_l;
        tmp_l.id     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.lid    = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.tlid  = atoi((const char*)(fileContents[i][2].c_str()));
        tmp_l.signid = atoi((const char*)(fileContents[i][3].c_str()));
        tmp_l.linkid = atoi((const char*)(fileContents[i][4].c_str()));

        stoplines.insert( std::map<int, StopLine>::value_type(tmp_l.id, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_zebrazone(const char* filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        ZebraZone tmp_l;
        tmp_l.id     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.aid    = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.linkid = atoi((const char*)(fileContents[i][2].c_str()));

        zebrazones.insert( std::map<int, ZebraZone>::value_type(tmp_l.id, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_crosswalk(const char* filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        CrossWalk tmp_l;
        tmp_l.id     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.aid    = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.type   = atoi((const char*)(fileContents[i][2].c_str()));
        tmp_l.bdid   = atoi((const char*)(fileContents[i][3].c_str()));
        tmp_l.linkid = atoi((const char*)(fileContents[i][4].c_str()));

        crosswalks.insert( std::map<int, CrossWalk>::value_type(tmp_l.id, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_roadsurfacemark(const char* filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        RoadSurfaceMark tmp_l;
        tmp_l.id     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.aid    = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.type   = fileContents[i][2].c_str();
        tmp_l.linkid = atoi((const char*)(fileContents[i][3].c_str()));

        roadsurfacemarks.insert( std::map<int, RoadSurfaceMark>::value_type(tmp_l.id, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_poledata(const char* filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        PoleData tmp_l;
        tmp_l.id     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.plid   = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.linkid = atoi((const char*)(fileContents[i][2].c_str()));

        poledatas.insert( std::map<int, PoleData>::value_type(tmp_l.id, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_roadsign(const char *filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        SignalData tmp_s;
        tmp_s.id     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_s.vid    = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_s.plid   = atoi((const char*)(fileContents[i][2].c_str()));
        tmp_s.type   = atoi((const char*)(fileContents[i][3].c_str()));
        tmp_s.linkid = atoi((const char*)(fileContents[i][4].c_str()));

        roadsigns.insert( std::map<int, SignalData>::value_type(tmp_s.id, tmp_s) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_signaldata(const char *filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        SignalData tmp_s;
        tmp_s.id     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_s.vid    = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_s.plid   = atoi((const char*)(fileContents[i][2].c_str()));
        tmp_s.type   = atoi((const char*)(fileContents[i][3].c_str()));
        tmp_s.linkid = atoi((const char*)(fileContents[i][4].c_str()));

        signaldatas.insert( std::map<int, SignalData>::value_type(tmp_s.id, tmp_s) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_streetlight(const char *filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        StreetLight tmp_l;
        tmp_l.id     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.lid    = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.plid   = atoi((const char*)(fileContents[i][2].c_str()));
        tmp_l.linkid = atoi((const char*)(fileContents[i][3].c_str()));

        streetlights.insert( std::map<int, StreetLight>::value_type(tmp_l.id, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_utilitypole(const char* filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        PoleData tmp_l;
        tmp_l.id     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.plid   = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.linkid = atoi((const char*)(fileContents[i][2].c_str()));

        utilitypoles.insert( std::map<int, PoleData>::value_type(tmp_l.id, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_pointclass(const char *filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        PointClass tmp_p;
        tmp_p.pid    = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_p.b      = atof((const char*)(fileContents[i][1].c_str()));
        tmp_p.l      = atof((const char*)(fileContents[i][2].c_str()));
        tmp_p.h      = atof((const char*)(fileContents[i][3].c_str()));
        tmp_p.ly     = atof((const char*)(fileContents[i][4].c_str()));
        tmp_p.bx     = atof((const char*)(fileContents[i][5].c_str()));
        tmp_p.ref    = atoi((const char*)(fileContents[i][6].c_str()));
        tmp_p.mcode1 = atoi((const char*)(fileContents[i][7].c_str()));
        tmp_p.mcode2 = atoi((const char*)(fileContents[i][8].c_str()));
        tmp_p.mcode3 = atoi((const char*)(fileContents[i][9].c_str()));

        points.insert( std::map<int, PointClass>::value_type(tmp_p.pid, tmp_p) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_vectorclass(const char *filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        VectorClass tmp_v;
        tmp_v.vid  = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_v.pid  = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_v.hang = atof((const char*)(fileContents[i][2].c_str()));
        tmp_v.vang = atof((const char*)(fileContents[i][3].c_str()));

        vectors.insert( std::map<int, VectorClass>::value_type(tmp_v.vid, tmp_v) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_lineclass(const char *filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        LineClass tmp_l;
        tmp_l.lid  = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.bpid = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.fpid = atoi((const char*)(fileContents[i][2].c_str()));
        tmp_l.blid = atoi((const char*)(fileContents[i][3].c_str()));
        tmp_l.flid = atoi((const char*)(fileContents[i][4].c_str()));

        lines.insert( std::map<int, LineClass>::value_type(tmp_l.lid, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_areaclass(const char *filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        AreaClass tmp_l;
        tmp_l.aid     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.slid    = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.elid    = atoi((const char*)(fileContents[i][2].c_str()));

        areas.insert( std::map<int, AreaClass>::value_type(tmp_l.aid, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_poleclass(const char *filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        PoleClass tmp_l;
        tmp_l.plid     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.vid      = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.length   = atof((const char*)(fileContents[i][2].c_str()));
        tmp_l.dim      = atof((const char*)(fileContents[i][3].c_str()));

        poles.insert( std::map<int, PoleClass>::value_type(tmp_l.plid, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_boxclass(const char *filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        BoxClass tmp_l;
        tmp_l.bid     = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.pid1    = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.pid2    = atoi((const char*)(fileContents[i][2].c_str()));
        tmp_l.pid3    = atoi((const char*)(fileContents[i][3].c_str()));
        tmp_l.pid4    = atoi((const char*)(fileContents[i][4].c_str()));
        tmp_l.height  = atof((const char*)(fileContents[i][4].c_str()));

        boxes.insert( std::map<int, BoxClass>::value_type(tmp_l.bid, tmp_l) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_dtlane(const char *filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        DTLane tmp_a;
        tmp_a.did   = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_a.dist  = atof((const char*)(fileContents[i][1].c_str()));
        tmp_a.pid   = atoi((const char*)(fileContents[i][2].c_str()));
        tmp_a.dir   = atof((const char*)(fileContents[i][3].c_str()));
        tmp_a.apara = atof((const char*)(fileContents[i][4].c_str()));
        tmp_a.r     = atof((const char*)(fileContents[i][5].c_str()));
        tmp_a.slope = atof((const char*)(fileContents[i][6].c_str()));
        tmp_a.cant  = atof((const char*)(fileContents[i][7].c_str()));
        tmp_a.lw    = atof((const char*)(fileContents[i][8].c_str()));
        tmp_a.rw    = atof((const char*)(fileContents[i][9].c_str()));

        dtlanes.insert( std::map<int, DTLane>::value_type(tmp_a.did, tmp_a) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_nodedata(const char *filename){
        File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        NodeData tmp_a;
        tmp_a.nid   = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_a.pid   = atoi((const char*)(fileContents[i][1].c_str()));

        nodedatas.insert( std::map<int, NodeData>::value_type(tmp_a.nid, tmp_a) );
    }

    return EXIT_SUCCESS;
}
int VectorMap::read_lanedata(const char *filename){
    File_contents fileContents = read_csv(filename);
    if (fileContents.empty())
        return EXIT_FAILURE;

    size_t line_num = fileContents.size();
    for (int i=0; i<line_num; i++)
    {
        LaneData tmp_l;
        tmp_l.lnid    = atoi((const char*)(fileContents[i][0].c_str()));
        tmp_l.did     = atoi((const char*)(fileContents[i][1].c_str()));
        tmp_l.blid    = atoi((const char*)(fileContents[i][2].c_str()));
        tmp_l.flid    = atoi((const char*)(fileContents[i][3].c_str()));
        tmp_l.bnid    = atoi((const char*)(fileContents[i][4].c_str()));
        tmp_l.fnid    = atoi((const char*)(fileContents[i][5].c_str()));
        tmp_l.jct     = atoi((const char*)(fileContents[i][6].c_str()));
        tmp_l.blid2   = atoi((const char*)(fileContents[i][7].c_str()));
        tmp_l.blid3   = atoi((const char*)(fileContents[i][8].c_str()));
        tmp_l.blid4   = atoi((const char*)(fileContents[i][9].c_str()));
        tmp_l.flid2   = atoi((const char*)(fileContents[i][10].c_str()));
        tmp_l.flid3   = atoi((const char*)(fileContents[i][11].c_str()));
        tmp_l.flid4   = atoi((const char*)(fileContents[i][12].c_str()));
        tmp_l.clossid = atoi((const char*)(fileContents[i][13].c_str()));
        tmp_l.span    = atof((const char*)(fileContents[i][14].c_str()));
        tmp_l.lcnt    = atoi((const char*)(fileContents[i][15].c_str()));
        tmp_l.lno     = atoi((const char*)(fileContents[i][16].c_str()));

        lanedatas.insert( std::map<int, LaneData>::value_type(tmp_l.lnid, tmp_l) );
    }

    return EXIT_SUCCESS;
}

void VectorMap::loadAll (const std::string &dirname)
{
    string rename = dirname + "/roadedge.csv";
    if (read_roadedge((char*)rename.c_str()) == EXIT_FAILURE) {
        std::cerr << rename << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << rename << "\t load complete." << std::endl;

    string guttername = dirname + "/gutter.csv";
    if (read_gutter((char*)guttername.c_str()) == EXIT_FAILURE) {
        std::cerr << guttername << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << guttername << "\t load complete." << std::endl;

    string curbname = dirname + "/curb.csv";
    if (read_curb((char*)curbname.c_str()) == EXIT_FAILURE) {
        std::cerr << curbname << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << curbname << "\t load complete." << std::endl;

    //string clinename = dirname + "/cline.csv";
    string clinename = dirname + "/whiteline.csv";
    if (read_whiteline ((char*)clinename.c_str()) == EXIT_FAILURE) {
        std::cerr << clinename << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << clinename << "\t load complete." << std::endl;

    string slinename = dirname + "/stopline.csv";
    if (read_stopline ((char*)slinename.c_str()) == EXIT_FAILURE) {
        std::cerr << slinename << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << slinename << "\t load complete." << std::endl;

    string zebraname = dirname + "/zebrazone.csv";
    if (read_zebrazone ((char*)zebraname.c_str()) == EXIT_FAILURE) {
        std::cerr << zebraname << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << zebraname << "\t load complete." << std::endl;

    string crossname = dirname + "/crosswalk.csv";
    if (read_crosswalk ((char*)crossname.c_str()) == EXIT_FAILURE) {
        std::cerr << crossname << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << crossname << "\t load complete." << std::endl;

    string roadsurfname = dirname + "/road_surface_mark.csv";
    if (read_roadsurfacemark ((char*)roadsurfname.c_str()) == EXIT_FAILURE) {
        std::cerr << roadsurfname << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << roadsurfname << "\t load complete." << std::endl;

    string poledataname = dirname + "/poledata.csv";
    if (read_poledata ((char*)poledataname.c_str()) == EXIT_FAILURE) {
        std::cerr << poledataname << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << poledataname << "\t load complete." << std::endl;

    string roadsignname = dirname + "/roadsign.csv";
    if (read_roadsign ((char*)roadsignname.c_str()) == EXIT_FAILURE) {
        std::cerr << roadsignname << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << roadsignname << "\t load complete." << std::endl;

    //string signalname = dirname + "/signal.csv";
    string signalname = dirname + "/signaldata.csv";
    if (read_signaldata ((char*)signalname.c_str()) == EXIT_FAILURE) {
        std::cerr << signalname << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << signalname << "\t load complete." << std::endl;

    string lightname = dirname + "/streetlight.csv";
    if (read_streetlight ((char*)lightname.c_str()) == EXIT_FAILURE) {
        std::cerr << lightname << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << lightname << "\t load complete." << std::endl;

    string utilitypolename = dirname + "/utilitypole.csv";
    if (read_utilitypole ((char*)utilitypolename.c_str()) == EXIT_FAILURE) {
        std::cerr << utilitypolename << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << utilitypolename << "\t load complete." << std::endl;


    string ptname = dirname + "/point.csv";
    if (read_pointclass((char*)ptname.c_str()) == EXIT_FAILURE) {
        std::cerr << ptname << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << ptname << "\t load complete." << std::endl;

    string vectorname = dirname + "/vector.csv";
    if (read_vectorclass ((char*)vectorname.c_str()) == EXIT_FAILURE) {
        std::cerr << vectorname << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << vectorname << "\t load complete." << std::endl;

    string linename = dirname + "/line.csv";
    if (read_lineclass((char*)linename.c_str()) == EXIT_FAILURE) {
        std::cerr << linename << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << linename << "\t load complete." << std::endl;

    string areaname = dirname + "/area.csv";
    if (read_areaclass ((char*)areaname.c_str()) == EXIT_FAILURE) {
        std::cerr << areaname << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << areaname << "\t load complete." << std::endl;

    string polename = dirname + "/pole.csv";
    if (read_poleclass ((char*)polename.c_str()) == EXIT_FAILURE) {
        std::cerr << polename << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << polename << "\t load complete." << std::endl;

//    string boxname = dirname + "/box.csv";
//    if (read_boxclass ((char*)boxname.c_str()) == EXIT_FAILURE) {
//        std::cerr << boxname << "\t load fail." << std::endl;
//        exit(EXIT_FAILURE);
//    }
//    std::cout << boxname << "\t load complete." << std::endl;


    string dtlanename = dirname + "/dtlane.csv";
    if (read_dtlane((char*)dtlanename.c_str()) == EXIT_FAILURE) {
        std::cerr << dtlanename << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << dtlanename << "\t load complete." << std::endl;

    string nodename = dirname + "/node.csv";
    if (read_vectorclass ((char*)vectorname.c_str()) == EXIT_FAILURE) {
        std::cerr << nodename << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << nodename << "\t load complete." << std::endl;


    string lanename = dirname + "/lane.csv";
    if (read_lanedata ((char*)lanename.c_str()) == EXIT_FAILURE) {
        std::cerr << lanename << "\t load fail." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << lanename << "\t load complete." << std::endl;


    std::cout << "all vecter maps loaded." << std::endl;

}

ofPoint VectorMap::get_point(int pid){
    ofPoint point;
    point.x = points[pid].bx - (min_x + max_x) / 2.0;    // b or bx?
    point.z = (points[pid].ly - (min_y + max_y) / 2.0) * (-1);    // l or ly?
    point.y = points[pid].h;    // Y-up or Z-up?

    return point;
}

int VectorMap::set_roadedge(){
    for(int i =0;i < roadedges.size();i++){
        if (roadedges[i].id <= 0) continue;
        int lid = roadedges[i].lid;
        if(lines[lid].blid == 0) {
            re_mesh.push_back(set_lineclass(OF_PRIMITIVE_LINE_STRIP, ofColor::white, 3,  lid));
        }
    }
    roadedges.clear();
    return EXIT_SUCCESS;

}
int VectorMap::set_gutter(){
        for(int i =0;i < gutters.size();i++){
            if (gutters[i].id <= 0) continue;
            int aid = gutters[i].aid;
                gutter_mesh.push_back(set_areaclass(OF_PRIMITIVE_LINE_STRIP, ofColor::gray,  aid));
            }
    gutters.clear();
    return EXIT_SUCCESS;
}
int VectorMap::set_curb(){
    for(int i =0;i < curbs.size();i++){
        if (curbs[i].id <= 0) continue;
        int lid = curbs[i].lid;
        if(lines[lid].blid == 0) {
            curb_mesh.push_back(set_lineclass(OF_PRIMITIVE_LINE_STRIP, ofColor::white, 3,  lid));
        }
    }
    curbs.clear();
    return EXIT_SUCCESS;
}
int VectorMap::set_whiteline(){
    for(int i =0;i < whitelines.size();i++){
        if (whitelines[i].id <= 0) continue;
        int lid = whitelines[i].lid;
        if(lines[lid].blid == 0) {
            if(whitelines[i].color == 'W')wline_mesh.push_back(set_lineclass(OF_PRIMITIVE_LINE_STRIP, ofColor::white, 3,  lid));
            if(whitelines[i].color == 'Y')wline_mesh.push_back(set_lineclass(OF_PRIMITIVE_LINE_STRIP, ofFloatColor(1.0, 0.5, 0.0), 3,  lid));
        }
    }
    whitelines.clear();
    return EXIT_SUCCESS;
}
int VectorMap::set_stopline(){
    for(int i =0;i < stoplines.size();i++){
        if (stoplines[i].id <= 0) continue;
        int lid = stoplines[i].lid;
        if(lines[lid].blid == 0) {
            sline_mesh.push_back(set_lineclass(OF_PRIMITIVE_LINE_STRIP, ofColor::white, 3,  lid));
        }
    }
    stoplines.clear();
    return EXIT_SUCCESS;
}
int VectorMap::set_zebrazone(){
    for(int i =0;i < zebrazones.size();i++){
        if (zebrazones[i].id <= 0) continue;
        int aid = zebrazones[i].aid;
        zz_mesh.push_back(set_areaclass(OF_PRIMITIVE_LINE_LOOP, ofColor::white, aid));
        }
    zebrazones.clear();
    return EXIT_SUCCESS;
}
int VectorMap::set_crosswalk(){
    for(int i =0;i < crosswalks.size();i++){
        if (crosswalks[i].id <= 0) continue;
        int aid = crosswalks[i].aid;
        cw_mesh.push_back(set_areaclass(OF_PRIMITIVE_LINE_LOOP, ofColor::white, aid));
        }
    crosswalks.clear();
    return EXIT_SUCCESS;
}
int VectorMap::set_roadsurfacemark(){
    for(int i =0;i < roadsurfacemarks.size();i++){
        if (roadsurfacemarks[i].id <= 0) continue;
        int aid = roadsurfacemarks[i].aid;
        rsm_mesh.push_back(set_areaclass(OF_PRIMITIVE_LINE_LOOP, ofColor::white, aid));
        }
    roadsurfacemarks.clear();
        return EXIT_SUCCESS;
}
int VectorMap::set_poledata(){
    for(int i =0;i < poledatas.size();i++){
        if (poledatas[i].id <= 0) continue;
        int plid = poledatas[i].plid;
        pd_mesh.push_back(set_poleclass(ofColor::gray, plid));
        }
    poledatas.clear();
    return EXIT_SUCCESS;
}
int VectorMap::set_roadsign(){
    for(int i =0;i < roadsigns.size();i++){
        if (roadsigns[i].id <= 0) continue;
        int plid = roadsigns[i].plid;
        rs_mesh.push_back(set_poleclass(ofColor::white, plid));
        }
    roadsigns.clear();
    return EXIT_SUCCESS;
}
int VectorMap::set_signaldata(){
    for(int i =0;i < signaldatas.size();i++){
        if (signaldatas[i].id <= 0) continue;
        int plid = signaldatas[i].plid;
        sd_mesh.push_back(set_poleclass(ofColor::gray, plid));
        }
    signaldatas.clear();
    return EXIT_SUCCESS;
}
int VectorMap::set_streetlight(){
    for(int i =0;i < streetlights.size();i++){
        if (streetlights[i].id <= 0) continue;
        int lid = streetlights[i].lid;
        int plid = streetlights[i].plid;
        sl_line_mesh.push_back(set_lineclass(OF_PRIMITIVE_LINE_LOOP, ofColor::yellow, 3, lid));
        sl_pole_mesh.push_back(set_poleclass(ofColor::gray, plid));
        }
    streetlights.clear();
    return EXIT_SUCCESS;
}
int VectorMap::set_utilitypole(){
    for(int i =0;i < utilitypoles.size();i++){
        if (utilitypoles[i].id <= 0) continue;
        int plid = utilitypoles[i].plid;
        up_mesh.push_back(set_poleclass(ofColor::gray, plid));
        }
    utilitypoles.clear();
    return EXIT_SUCCESS;
}

//int VectorMap::draw_pointclass(){}
//int VectorMap::draw_vectorclass(){}
ofVboMesh VectorMap::set_lineclass(ofPrimitiveMode mode, ofColor color, float width,  int lineid){
    int lid = lineid;
    ofVboMesh tmp;
    ofVec3f pos;
    tmp.setMode(mode);
    pos.set(get_point(lines[lid].bpid));
    tmp.addVertex(pos);
    tmp.addColor(color);
    if(lines[lid].flid==0){
        pos.set(get_point(lines[lid].fpid));
        tmp.addVertex(pos);
        tmp.addColor(color);
    }
    while(lines[lid].flid!=0){
        pos.set(get_point(lines[lid].fpid));
        tmp.addVertex(pos);
        tmp.addColor(color);
        lid = lines[lid].flid;
    }
    return tmp;
}
ofVboMesh VectorMap::set_areaclass(ofPrimitiveMode mode, ofColor color, int aid){
    int lid = areas[aid].slid;
    ofVboMesh tmp;
    ofVec3f pos;
    tmp.setMode(mode);
    pos.set(get_point(lines[lid].bpid));
    tmp.addVertex(pos);
    tmp.addColor(color);
    while(lid != areas[aid].elid){
        pos.set(get_point(lines[lid].fpid));
        tmp.addVertex(pos);
        tmp.addColor(color);
        lid = lines[lid].flid;
        }
    return tmp;
}
ofVboMesh VectorMap::set_poleclass(ofColor color, int plid){
    int vid = poles[plid].vid;
    ofVboMesh tmp = ofVboMesh::cylinder(poles[plid].dim/2.0, poles[plid].length);
    ofVec3f point;
    ofVec3f pos = get_point(vectors[vid].pid);

//  RotateX 90
    for(int i = 0; i < tmp.getNumVertices(); i++){
        point = tmp.getVertex(i);
        tmp.setVertex(i, ofVec3f(point.x, point.y + poles[plid].length / 2.0, point.z));
    }

    for(int i = 0; i < tmp.getNumVertices(); i++){
        point = tmp.getVertex(i);
        point += pos;
        tmp.setVertex(i, point);
        tmp.addColor(color);
    }
    return tmp;
    }
//int VectorMap::draw_boxclass(){}

//int VectorMap::draw_dtlane(){}
//int VectorMap::draw_nodedata(){}
//int VectorMap::draw_lanedata(){}

void VectorMap::set_all (){
    set_roadedge();
    set_gutter();
    set_curb();
    set_whiteline();
    set_stopline();
    set_zebrazone();
    set_crosswalk();
    set_roadsurfacemark();
    set_poledata();
    set_roadsign();
    set_signaldata();
    set_streetlight();
    set_utilitypole();
}


void VectorMap::draw (){
    glDisable(GL_DEPTH_TEST);
    for(int i = 0; i < re_mesh.size(); i++)     re_mesh[i].draw();
    for(int i = 0; i < gutter_mesh.size(); i++) gutter_mesh[i].draw();
    for(int i = 0; i < curb_mesh.size(); i++)   curb_mesh[i].draw();
    for(int i = 0; i < wline_mesh.size(); i++)  wline_mesh[i].draw();
    for(int i = 0; i < sline_mesh.size(); i++)  sline_mesh[i].draw();
    for(int i = 0; i < zz_mesh.size(); i++)     zz_mesh[i].draw();
    for(int i = 0; i < cw_mesh.size(); i++)     cw_mesh[i].draw();
    for(int i = 0; i < rsm_mesh.size(); i++)    rsm_mesh[i].draw();
    glEnable(GL_DEPTH_TEST);
    for(int i = 0; i < pd_mesh.size(); i++)     pd_mesh[i].draw();
    for(int i = 0; i < sd_mesh.size(); i++)     sd_mesh[i].draw();
    for(int i = 0; i < sl_line_mesh.size(); i++)sl_line_mesh[i].draw();
    for(int i = 0; i < sl_pole_mesh.size(); i++)sl_pole_mesh[i].draw();
    for(int i = 0; i < up_mesh.size(); i++)     up_mesh[i].draw();
}

void VectorMap::drawWireframe (){
    glDisable(GL_DEPTH_TEST);
    for(int i = 0; i < re_mesh.size(); i++)     re_mesh[i].drawWireframe();
    for(int i = 0; i < gutter_mesh.size(); i++) gutter_mesh[i].drawWireframe();
    for(int i = 0; i < curb_mesh.size(); i++)   curb_mesh[i].drawWireframe();
    for(int i = 0; i < wline_mesh.size(); i++)  wline_mesh[i].drawWireframe();
    for(int i = 0; i < sline_mesh.size(); i++)  sline_mesh[i].drawWireframe();
    for(int i = 0; i < zz_mesh.size(); i++)     zz_mesh[i].drawWireframe();
    for(int i = 0; i < cw_mesh.size(); i++)     cw_mesh[i].drawWireframe();
    for(int i = 0; i < rsm_mesh.size(); i++)    rsm_mesh[i].drawWireframe();
    glEnable(GL_DEPTH_TEST);
    for(int i = 0; i < pd_mesh.size(); i++)     pd_mesh[i].drawWireframe();
    for(int i = 0; i < sd_mesh.size(); i++)     sd_mesh[i].drawWireframe();
    for(int i = 0; i < sl_line_mesh.size(); i++)sl_line_mesh[i].drawWireframe();
    for(int i = 0; i < sl_pole_mesh.size(); i++)sl_pole_mesh[i].drawWireframe();
    for(int i = 0; i < up_mesh.size(); i++)     up_mesh[i].drawWireframe();
}

