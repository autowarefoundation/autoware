#include "pointcloud_map.h"

void PointcloudMap::searcharea(){
    
    char fpath[sizeof(PCD_DIR)+sizeof("/Laserxxxxxx_yyyyyy.pcd")];
    
    while(max_x==0||max_y==0){usleep(100);}
    int prev_x = x - 2;
    int prev_y = y - 2;
    int center_x = 1;
    int center_y = 1;
    // we're going to load a ton of points into an ofMesh
    for(int i=0;i<9;i++)mesh[i].setMode(OF_PRIMITIVE_POINTS);
    printf("%s %06d %06d \n",PCD_DIR,prev_x, prev_y);
    sleep(5);
    
    sload();
    
    while(1){
        if((int)x - prev_x == 1){
            printf("----------------------------------\n");
            printf("[%d] x++\n",(int)x);
            prev_x = x;
            for(int i = 0; i <= 2; i++){
                sprintf(fpath,"%s/Laser%06d_%06d.pcd", PCD_DIR , (int)prev_x+1, (int)prev_y+i-1);
                printf("%s ",fpath);
                if(ofFile::doesFileExist(fpath)){
                    printf("exist.\n Load: mesh[%d]\n",(center_x+2)%3+((center_y+i+2)%3)*3);
                    load(fpath,(center_x+2)%3+((center_y+i+2)%3)*3);
                }else {
                    printf("\n Clear: mesh[%d]\n",(center_x+2)%3+((center_y+i+2)%3)*3);
                    mesh[(center_x+2)%3+((center_y+i+2)%3)*3].clear();
                }
            }
            center_x = (center_x + 1) % 3;
        }
        if((int)x - prev_x == -1){
            printf("----------------------------------\n");
            printf("[%d] x--\n",(int)x);
            prev_x = x;
            for(int i = 0; i <= 2; i++){
                sprintf(fpath,"%s/Laser%06d_%06d.pcd", PCD_DIR , prev_x-1, prev_y+i-1);
                printf("%s ",fpath);
                if(ofFile::doesFileExist(fpath)){
                    printf("exist.\n Load: mesh[%d]\n",(center_x+1)%3+((center_y+i+2)%3)*3);
                    load(fpath,(center_x+1)%3+((center_y+i+2)%3)*3);
                }else {
                    printf("\n Clear: mesh[%d]\n",(center_x+1)%3+((center_y+i+2)%3)*3);
                    mesh[(center_x+1)%3+((center_y+i+2)%3)*3].clear();
                }
            }
            center_x = (center_x + 2) % 3;
        }
        if((int)y - prev_y == 1){
            printf("----------------------------------\n");
            printf("[%d] y++\n",(int)y);
            prev_y = y;
            for(int j = 0; j <= 2; j++){
                sprintf(fpath,"%s/Laser%06d_%06d.pcd", PCD_DIR , (int)prev_x+j-1, (int)prev_y+1);
                printf("%s ",fpath);
                if(ofFile::doesFileExist(fpath)){
                    printf("exist.\n Load: mesh[%d]\n",(center_x+j+2)%3+((center_y+2)%3)*3);
                    load(fpath,(center_x+j+2)%3+((center_y+2)%3)*3);
                }else {
                    printf("\n Clear: mesh[%d]\n",(center_x+j+2)%3+((center_y+2)%3)*3);
                    mesh[(center_x+j+2)%3+((center_y+2)%3)*3].clear();
                }
            }
            center_y = (center_y + 1) % 3;
        }
        if((int)y - prev_y == -1){
            printf("----------------------------------\n");
            printf("[%d] y--\n",(int)y);
            prev_y = y;
            for(int j = 0; j <= 2; j++){
                sprintf(fpath,"%s/Laser%06d_%06d.pcd", PCD_DIR , (int)prev_x+j-1, (int)prev_y-1);
                printf("%s ",fpath);
                if(ofFile::doesFileExist(fpath)){
                    printf("exist.\n Load: mesh[%d]\n",(center_x+j+2)%3+((center_y+1)%3)*3);
                    load(fpath,(center_x+j+2)%3+((center_y+1)%3)*3);
                }else {
                    printf("\n Clear: mesh[%d]\n",(center_x+j+2)%3+((center_y+1)%3)*3);
                    mesh[(center_x+j+2)%3+((center_y+1)%3)*3].clear();
                }
            }
            center_y = (center_y + 2) % 3;
        }
        if(abs(x-prev_x)>1||abs(y-prev_y)>1){
            printf("[%d %d]\n",(int)x,(int)y);
            prev_x = x;
            prev_y = y;
            for(int i = 0; i <= 2; i++){
                for(int j = 0; j <= 2; j++){
                    sprintf(fpath,"%s/Laser%06d_%06d.pcd", PCD_DIR , (int)prev_x+j-1, (int)prev_y+i-1);
                    printf("%s ",fpath);
                    
                    if(ofFile::doesFileExist(fpath)){
                        printf("exist.\n Load: mesh[%d]\n",j+i*3);
                        load(fpath,j+i*3);
                    }else{
                        printf("\n Clear: mesh[%d]\n",j+i*3);
                        mesh[j+i*3].clear();
                    }
                }
            }
            center_x = 1;
            center_y = 1;
        }
    }
}

void PointcloudMap::load(char* path, int num){
    loading=num;
    mesh[num].clear();
    points.clear();
    
    totalPoints = 0;
    int startPoint = totalPoints;
    fin.open(ofToDataPath(path, true).c_str());
    
    if(fin.fail()) {
        cerr << "File do not exist.\n";
    }else{
        printf("Reading... ");
    }
    while(getline(fin, line)){
        lines=(ofSplitString(line, " "));
        if(lines.size() == 4){
            float x = ofToFloat(lines[0]);
            float y = ofToFloat(lines[1]);
            float z = ofToFloat(lines[2]);
            float c = ofToFloat(lines[3]);
            //printf("%f %f %f %f\n",x,y,z,c);
            points.push_back(PCDPoint(x,y,z,c));
            
            // Global MINMAX
            if(totalPoints == 0){
                printf("update:GlobalPoints\n");
                //                    max_x = x; min_x = x; max_y = y; min_y = y; max_z = z; min_z = z;
            }else{
                //                    if(x > max_x) max_x = x;
                //                    if(x < min_x) min_x = x;
                //                    if(y > max_y) max_y = y;
                //                    if(y < min_y) min_y = y;
                //                    if(z > max_z) max_z = z;
                //                    if(z < min_z) min_z = z;
            }
            // Local MINMAX
            if(localPoints == 0){
                printf("update:localPoints\n");
                ma_x = x; mi_x = x; ma_y = y; mi_y = y; ma_z = z; mi_z = z;
            }else{
                if(x > ma_x) ma_x = x;
                if(x < mi_x) mi_x = x;
                if(y > ma_y) ma_y = y;
                if(y < mi_y) mi_y = y;
                if(z > ma_z) ma_z = z;
                if(z < mi_z) mi_z = z;
            }
            
            localPoints++;
            totalPoints++;
        }
    }
    fin.close();
    
    printf("%d\n  X [%f : %f]\n  Y [%f : %f]\n  Z [%f : %f]\n\n", localPoints,mi_x, ma_x,  mi_y, ma_y,  mi_z, ma_z);
    PCDfile file = (PCDfile(num,startPoint,totalPoints,ma_x, ma_y, ma_z, mi_x, mi_y, mi_z, path));
    localPoints = 0;
    
    
    
    ofSetVerticalSync(true);
    uint8_t r, g, b;
    
    printf("Creating mesh ...  %d to %d\n",file.start,file.end);
    for(int j = file.start; j<file.end; j++){
#ifdef COLOR
        r = (*(int32_t*)(&points[j].c))>>16;
        g = (*(int32_t*)(&points[j].c))>>8 & 0b11111111;
        b = (*(int32_t*)(&points[j].c)) & 0b11111111;
        mesh[num].addColor(ofFloatColor((double)r/255.0,(double)g/255.0,(double)b/255.0));
#else
        mesh[num].addColor(ofFloatColor(0.2, 1, 0.2, 0.5));
#endif // COLOR
        ofVec3f pos(points[j].x-(min_x+max_x)/2.0, points[j].z, -(points[j].y-(min_y+max_y)/2.0));
        mesh[num].addVertex(pos);
        
    }
    loading=-1;
}

void PointcloudMap::sload()
{
    string path = STATIC_DIR;
    ofDirectory dir(path);
    dir.allowExt("pcd");
    dir.listDir();
    readingFileNum = dir.numFiles();
    start.setMode(OF_PRIMITIVE_POINTS);
    for(int i = 0; i < readingFileNum; i++){
        int startPoint;
        startPoint = totalPoints;
        fin.open(ofToDataPath(dir.getPath(i), true).c_str());
        
        if(fin.fail()) {
            cerr << "File do not exist.\n";
        }else{
            printf("Reading... ");
        }
        while(getline(fin, line)){
            lines=(ofSplitString(line, " "));
            if(lines.size() == 4){
                float x = ofToFloat(lines[0]);
                float y = ofToFloat(lines[1]);
                float z = ofToFloat(lines[2]);
                float c = ofToFloat(lines[3]);
                points.push_back(PCDPoint(x,y,z,c));
                
                // Global MINMAX
                if(totalPoints == 0){
                    printf("update:GlobalPoints\n");
                    //                    max_x = x; min_x = x; max_y = y; min_y = y; max_z = z; min_z = z;
                }else{
                    //                    if(x > max_x) max_x = x;
                    //                    if(x < min_x) min_x = x;
                    //                    if(y > max_y) max_y = y;
                    //                    if(y < min_y) min_y = y;
                    //                    if(z > max_z) max_z = z;
                    //                    if(z < min_z) min_z = z;
                }
                // Local MINMAX
                if(localPoints == 0){
                    printf("update:localPoints\n");
                    ma_x = x; mi_x = x; ma_y = y; mi_y = y; ma_z = z; mi_z = z;
                }else{
                    if(x > ma_x) ma_x = x;
                    if(x < mi_x) mi_x = x;
                    if(y > ma_y) ma_y = y;
                    if(y < mi_y) mi_y = y;
                    if(z > ma_z) ma_z = z;
                    if(z < mi_z) mi_z = z;
                }
                
                localPoints++;
                totalPoints++;
            }
        }
        fin.close();
        
        printf("DONE[ %d / %d] %d\n  X [%f : %f]  Y [%f : %f]  Z [%f : %f]\n\n", i, readingFileNum, localPoints,mi_x, ma_x,  mi_y, ma_y,  mi_z, ma_z);
        files.push_back(PCDfile(i,startPoint,totalPoints,ma_x, ma_y, ma_z, mi_x, mi_y, mi_z, dir.getPath(i)));
        localPoints = 0;
    }
    
    printf("TOTAL: %d  X [%f : %f]  Y [%f : %f]  Z [%f : %f]\n",totalPoints, min_x, max_x,  min_y, max_y,  min_z, max_z);
    
    ofSetVerticalSync(true);
    uint8_t r, g, b;
    
    // we're going to load a ton of points into an ofMesh
    start.setMode(OF_PRIMITIVE_POINTS);
    
    // loop through the image in the x and y axes
    int skip = 1; // load a subset of the points
    for(int i = 0; i< readingFileNum; i++){
        printf("Creating mesh ... %d  %d to %d\n",i,files[i].start,files[i].end);
        for(int j = files[i].start; j<files[i].end; j++){
#ifdef COLOR
            r = (*(int32_t*)(&points[j].c))>>16;
            g = (*(int32_t*)(&points[j].c))>>8 & 0b11111111;
            b = (*(int32_t*)(&points[j].c)) & 0b11111111;
            start.addColor(ofFloatColor((double)r/255.0,(double)g/255.0,(double)b/255.0));
#else
#ifdef RAINBOW
            float level = (11 + 3) / 7.0;
            if(          - 3 <= points[j].z && points[j].z < level * 1 - 3)start.addColor(ofColor::red);
            if(level * 1 - 3 <= points[j].z && points[j].z < level * 2 - 3)start.addColor(ofColor::orange);
            if(level * 2 - 3 <= points[j].z && points[j].z < level * 3 - 3)start.addColor(ofColor::yellow);
            if(level * 3 - 3 <= points[j].z && points[j].z < level * 4 - 3)start.addColor(ofColor::green);
            if(level * 4 - 3 <= points[j].z && points[j].z < level * 5 - 3)start.addColor(ofColor::blue);
            if(level * 5 - 3 <= points[j].z && points[j].z < level * 6 - 3)start.addColor(ofColor::navy);
            if(level * 6 - 3 <= points[j].z && points[j].z <=level * 7 - 3)start.addColor(ofColor::purple);
            
#else
            start.addColor(ofFloatColor(0.1, 1.0, 0.1,0.3));
#endif
#endif
            ofVec3f pos(points[j].x-(min_x+max_x)/2.0, points[j].z, -(points[j].y-(min_y+max_y)/2.0));
            start.addVertex(pos);
            
        }
    }
}
