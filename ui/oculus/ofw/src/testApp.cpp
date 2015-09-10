#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup()
{
    ofEnableDepthTest();
    ofEnableAlphaBlending();
    ofEnableSmoothing();

    while(pm.max_x==0||pm.max_y==0){
        pm.max_x = udp.car_x;
        pm.max_y = udp.car_y;
        pm.min_x = udp.car_x;
        pm.min_y = udp.car_y;
        pm.x = udp.car_x < 0 ? udp.car_x / 100.0 - 1 : udp.car_x / 100.0;
        pm.y = udp.car_y < 0 ? udp.car_y / 100.0 - 1 : udp.car_y / 100.0;
    }
    printf("%f %f %d %d\n", pm.max_x, pm.max_y, pm.x, pm.y);

#ifdef VECTOR
    vm.setTF(pm.min_x, pm.max_x, pm.min_y, pm.max_y);
    vm.loadAll(VECTOR);
    vm.set_all();
#endif

    vs.setMode(OF_PRIMITIVE_LINES);

    glPointSize(1); // make the points bigger
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_POINT_SMOOTH); // use circular points instead of square points
    glColorMaterial(GL_FRONT,GL_AMBIENT_AND_DIFFUSE);
    glShadeModel(GL_SMOOTH);

    float fogColor[4]= {0.1, 0.1, 0.1, 0.3};
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glFogfv(GL_FOG_COLOR, fogColor);
    glFogf(GL_FOG_DENSITY, 0.35);
    glHint(GL_FOG_HINT, GL_NICEST);
    glFogf(GL_FOG_START, 50.0);
    glFogf(GL_FOG_END, 100.0);
    glEnable(GL_FOG);

    ofEnableBlendMode(OF_BLENDMODE_SUBTRACT);

    ofBackground(0);
    ofSetVerticalSync( true );

    showOverlay = false;
    predictive = true;

    ofHideCursor();

    oculusRift.baseCamera = &cam;
    oculusRift.setup();
    printf("%f %f\n",(fclip=oculusRift.baseCamera->getFarClip()),oculusRift.baseCamera->getNearClip());
    fclip = 50;
    oculusRift.baseCamera->setNearClip(0.1);
    oculusRift.baseCamera->setFarClip(fclip);
    cam.setNearClip(0.1);
    cam.setFarClip(70);
    cam1.setNearClip(0.1);
    cam1.setFarClip(70);
    cam2.setNearClip(0.1);
    cam2.setFarClip(70);
    cam3.setNearClip(0.1);
    cam3.setFarClip(70);

    //enable mouse;
    cam.begin();
    cam.end();

    // JUMP
    initial_velocity = 2 * (float)MAX_OFFSET / (float)JUMP_TIME;
    accel = initial_velocity / (float)JUMP_TIME;

    // Car Model
    model.loadModel("prius_model.dae",true);
    model.setScale(0.004, 0.004, 0.004);

    ofToggleFullscreen();
    pm.f=false;

    sun.setDiffuseColor(ofColor::white);
    sun.setSpecularColor(ofColor::black);
    sun.setDirectional();
    sun.setOrientation(ofVec3f(90,0,45));
    carlight.setDiffuseColor(ofColor::white);
    carlight.setSpecularColor(ofColor::black);
    carlight.setSpotlight();
}

//--------------------------------------------------------------
void testApp::update()
{
    car.x = udp.car_x - (pm.min_x + pm.max_x) / 2.0;
    car.y = (udp.car_y - (pm.min_y + pm.max_y) / 2.0) * (-1);
    car.z = udp.car_z;

    pm.x = udp.car_x < 0 ? udp.car_x / 100.0 - 1 : udp.car_x / 100.0;
    pm.y = udp.car_y < 0 ? udp.car_y / 100.0 - 1 : udp.car_y / 100.0;

    // Stop vibration
    euler.x = (udp.roll < 5) || (udp.roll > -5) ? 0 : udp.roll;
    euler.y = udp.yaw - 90;
    euler.z = 0;

    // JUMP
    if((jump==0 && OculusRPY.x > -100 && OculusRPY.x < -50) || (jump==1 && OculusRPY.x>=-30))count++;
    else count = 0;
    if(count > 500){
        count = 0;
        jump = 1 - jump;
    }
    fps = ofGetFrameRate();
    if(jump == 1 && offset < MAX_OFFSET){
        offset += velocity / fps;
        velocity -= accel / fps;
    }
    else if(jump == 0 && offset > 0){
        offset -= velocity / fps;
        velocity -= accel / fps;
    }
    if(offset >= MAX_OFFSET){
        offset = MAX_OFFSET;
        velocity = initial_velocity;
    }
    if(offset <= 0){
        offset = 0;
        velocity = initial_velocity;
    }
        cam1.setGlobalPosition(car.x, car.z + offset, car.y);
        cam1.setOrientation(euler);
        cam2 = cam1;
        cam3 = cam1;
        cam1.truck(0.3);
        cam1.dolly(0.27);
        cam1.boom(-0.93);
        cam2.dolly(7);
        cam2.boom(2);
        cam3.boom(-1.8);
        cam3.dolly(-1.2);
        cam3.truck(-0.9);
        cam3.pan(-15);
        carlight.setGlobalPosition(car.x, car.z, car.y);
        carlight.setOrientation(euler);
        carlight.dolly(1);
    //Driver's view
    switch(mode){
    case 0 :
        oculusRift.baseCamera->setGlobalPosition(camPos);
        break;
    case 1 :
        cam = cam1;
        break;
    case 2 :
        cam = cam2;
        break;
    case 3 :
        cam = cam3;
        break;
    }
    if(udp.vscan.size()==vs.getNumVertices()){
        vs.clear();
        for(int i = 0;i < udp.vscan.size(); i++){
            vs.addVertex(ofVec3f(udp.vscan[i].x-(pm.min_x + pm.max_x) / 2.0, udp.vscan[i].z, -(udp.vscan[i].y-(pm.min_y + pm.max_y) / 2.0)));
        }
    }
}

//--------------------------------------------------------------
void testApp::draw()
{
    if(disp){
    if(oculusRift.isSetup()){

        ofSetColor(100, 100, 250);
        glEnable(GL_DEPTH_TEST);
        camera = oculusRift.baseCamera->getGlobalPosition();
        OculusQuaternion = oculusRift.getOrientationQuat();
        OculusRPY = OculusQuaternion.getEuler();
        oculusRift.beginLeftEye();
        drawScene();
        oculusRift.endLeftEye();

        oculusRift.beginRightEye();
        drawScene();
        oculusRift.endRightEye();

        oculusRift.draw();

        glDisable(GL_DEPTH_TEST);
        ofSetColor(255);
        ofDrawBitmapString(ofToString(ofGetFrameRate())+"fps",10,15);
    }
    else{
        cam.begin();

        drawScene();

        cam.end();
    }
    }else{
                ofDrawBitmapString(ofToString(ofGetFrameRate())+"fps",10,15);
        cam.begin();
        drawScene();
        cam.end();
    }
            if(op){

			ofSetColor(255,255);
			ofFill();

            ofDrawBitmapString("x     :\t"+ofToString(udp.car_x),300,100);
            ofDrawBitmapString("y     :\t"+ofToString(udp.car_y),300,110);
            ofDrawBitmapString("z     :\t"+ofToString(udp.car_z),300,120);
            ofDrawBitmapString("CAM   :\t"+ofToString(camPos.x),500,100);
            ofDrawBitmapString("CAM   :\t"+ofToString(camPos.y),500,110);
            ofDrawBitmapString("CAM   :\t"+ofToString(camPos.z),500,120);
            ofDrawBitmapString("roll  :\t"+ofToString(udp.roll),300,150);
            ofDrawBitmapString("pitch :\t"+ofToString(udp.pitch),300,160);
            ofDrawBitmapString("yaw   :\t"+ofToString(udp.yaw),300,170);
            ofDrawBitmapString("CAM   :\t"+ofToString(oculusRift.baseCamera->getRoll()),500,150);
            ofDrawBitmapString("CAM   :\t"+ofToString(oculusRift.baseCamera->getPitch()),500,160);
            ofDrawBitmapString("CAM   :\t"+ofToString(oculusRift.baseCamera->getHeading()),500,170);
            ofDrawBitmapString("UNIX  :\t"+ofToString(ofGetUnixTime()),300,200);
            ofDrawBitmapString("time  :\t"+ofToString(ofGetElapsedTimef()),300,210);
                    }

}

//--------------------------------------------------------------
void testApp::drawScene()
{
//    ofEnableSeparateSpecularLight();
        if(mout){
            ofSetColor(ofColor::silver);
//            sun.enable();

            ofPushMatrix();
            ofTranslate(car.x, car.z-2.0, car.y);
            ofRotateX(90);
            ofRotateZ(-90-euler.y);
            if(fill) model.drawFaces();
            else model.drawWireframe();
            ofPopMatrix();
//        sun.disable();
        }
        // Change BackGround Color
        ofPushMatrix();
        ofPushStyle();
        ofTranslate(camera);
        ofSetColor(ofColor::black);
        ofSphere(100);
        ofPopStyle();
        ofPopMatrix();
//        carlight.enable();
        pm.start.draw();
        for(int i=0; i < 9; i++)
            if(i != pm.loading )
            pm.mesh[i].draw();

#ifdef CAR_LOC
        ofPushStyle();
        ofNoFill();
        ofSetColor(ofColor::blue);
        for(int i = 0;i < udp.oth.size(); i++){
            ofPushMatrix();
            ofTranslate(udp.oth[i].x- (pm.min_x + pm.max_x) / 2.0,udp.car_z-1.5,(udp.oth[i].y - (pm.min_y + pm.max_y) / 2.0) * (-1));
            ofRotateX(90);
            ofRotateZ(-90-euler.y);
            ofDrawBox(0,0,0,3,1.5,1.5);
            ofPopMatrix();
        }
        ofSetColor(ofColor::red);
        for(int i = 0;i < udp.ped.size(); i++){
            ofPushMatrix();
            ofTranslate(udp.oth[i].x- (pm.min_x + pm.max_x) / 2.0,udp.car_z-1.5,(udp.oth[i].y - (pm.min_y + pm.max_y) / 2.0) * (-1));
//            ofRotateY();
            ofDrawCylinder(0,0,0,0.3,1.5);
            ofPopMatrix();
        }
        ofSetColor(ofColor::red);
        for(int i = 0;i < udp.obj.size(); i++){
            ofDrawBox(udp.obj[i].x- (pm.min_x + pm.max_x) / 2.0,udp.obj[i].z - 1,(udp.obj[i].y - (pm.min_y + pm.max_y) / 2.0) * (-1),1,1,1);
        }
        ofFill();
        ofPopStyle();
#endif
        vs.draw();
//        udp.vscan.draw();
#ifdef VECTOR
//        vm.draw();
        vm.drawWireframe();
#endif
//            carlight.disable();
}

//--------------------------------------------------------------
void testApp::keyPressed(int key)
{
    switch (key) {
        case 'w':
            rotAng = (oculusRift.baseCamera->getOrientationEuler()).y;
            camPos.x -= sin(rotAng/180.0*PI)*10;
            camPos.z -= cos(rotAng/180.0*PI)*10;
            break;
        case 's':
            rotAng = (oculusRift.baseCamera->getOrientationEuler()).y;
            camPos.x += sin(rotAng/180.0*PI)*10;
            camPos.z += cos(rotAng/180.0*PI)*10;
            break;
        case 'a':
            camPos.z +=10;
            break;
        case 'd':
            camPos.z -=10;
            break;
        case 'r':
            camPos.y +=10;
            break;
        case 'f':
            camPos.y -=10;
            break;

        case 'W':
            rotAng = (oculusRift.baseCamera->getOrientationEuler()).y;
            camPos.x -= sin(rotAng/180.0*PI);
            camPos.z -= cos(rotAng/180.0*PI);
            break;
        case 'S':
            rotAng = (oculusRift.baseCamera->getOrientationEuler()).y;
            camPos.x += sin(rotAng/180.0*PI);
            camPos.z += cos(rotAng/180.0*PI);
            break;
        case 'A':
            camPos.z +=10;
            break;
        case 'D':
            camPos.z -=10;
            break;
        case 'R':
            camPos.y +=10;
            break;
        case 'F':
            camPos.y -=10;
            break;
        case 'e':
            oculusRift.baseCamera->rotate(-10,0,1,0);
            break;
        case 'q':
            oculusRift.baseCamera->rotate(+10,0,1,0);
            break;
        case 'E':
            oculusRift.baseCamera->rotate(-1,0,1,0);
            break;
        case 'Q':
            oculusRift.baseCamera->rotate(+1,0,1,0);
            break;
        case '0':
            ofToggleFullscreen();
            break;
        case '1':
            mode = 1;
            break;
        case '2':
            mode = 2;
            break;
        case '3':
            mode = 3;
            break;
        case ' ':
            mode = 0;
            break;
        case 'j':
            if(offset == MAX_OFFSET || offset == 0){
                jump = 1 - jump;
            }
            break;
        case 'i':
            op = !op;
            break;
        case '.':
            udp.pose = !udp.pose;
            break;
        case 'm':
            mout = !mout;
            break;
        case 'n':
            fill = !fill;
            break;
        case '@':
            disp = !disp;
            break;
        default:
            break;
    }

}

//--------------------------------------------------------------
void testApp::keyReleased(int key)
{

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y)
{
    //   cursor2D.set(x, y, cursor2D.z);
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{
    //    cursor2D.set(x, y, cursor2D.z);
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg)
{

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo)
{

}

int basetime = 0;

void *udpThread(void *arg)
{
    udparg *n;
    n = (udparg *)arg;
    char buf[BUFSIZE];
    float tm;
#ifdef LOCAL
    std::ifstream fs(LOCAL);
    if (fs.fail()) {
        cerr << "Log File do not exist.";
        exit(1);
    }
    n->pose=false;

    while(1){
        while(fs.getline(buf, BUFSIZE)){
            while(n->pose){usleep(100);}
            string message = buf;
            if(message != ""){
                vector<string> udp = ofSplitString(message, " ");
                if(udp.size() == 9){
                    if(strcmp(udp[1].c_str(),"CAR")==0){
                        n->car_x = atof(udp[3].c_str());
                        n->car_y = atof(udp[4].c_str());
                        n->car_z = atof(udp[5].c_str());
                        n->roll = atof(udp[6].c_str());
                        n->pitch = atof(udp[7].c_str());
                        n->yaw = atof(udp[8].c_str());
                    }
                    if(strcmp(udp[1].c_str(),"OTH")==0){
                        n->oth.push_back(ofVec3f(atof(udp[3].c_str()),atof(udp[4].c_str()),atof(udp[5].c_str())));
                    }
                }
                if(basetime == 0)basetime = string2unixtime(udp[0].c_str());
                //                while((float)(string2unixtime(udp[0].c_str()) - basetime) / 1000.0 >= ofGetElapsedTimef()){
                //                }
                usleep(80009);
                n->oth.clear();
            }
        }
        fs.clear();
        fs.seekg(0, ios_base::beg);
    }
#else
    //UDPReceiver
    ofxUDPManager udpNDT;
    udpNDT.Create();
    udpNDT.Bind(PORT);
    udpNDT.SetNonBlocking(true);

    while(1){
        udpNDT.Receive(buf, BUFSIZE);
        string message = buf;
        if(message != ""){
            vector<string> udp = ofSplitString(message, " ");
            if(udp.size() == 9){
                if(strcmp(udp[1].c_str(),"CAR")==0){
                    n->car_x = atof(udp[3].c_str());
                    n->car_y = atof(udp[4].c_str());
                    n->car_z = atof(udp[5].c_str());
                    n->roll = atof(udp[6].c_str());
                    n->pitch = atof(udp[7].c_str());
                    n->yaw = atof(udp[8].c_str());
                }
                else if(strcmp(udp[1].c_str(),"OTH")==0){
                    n->oth.push_back(ofVec3f(atof(udp[3].c_str()),atof(udp[4].c_str()),atof(udp[5].c_str())));
                }
                else if(strcmp(udp[1].c_str(),"PED")==0){
                    n->ped.push_back(ofVec3f(atof(udp[3].c_str()),atof(udp[4].c_str()),atof(udp[5].c_str())));
                }
                else if(strcmp(udp[1].c_str(),"OBJ")==0){
                    n->obj.push_back(ofVec3f(atof(udp[3].c_str()),atof(udp[4].c_str()),atof(udp[5].c_str())));
                }
                else if(strcmp(udp[1].c_str(),"VSC")==0){
                    n->vscan.push_back(ofVec3f(atof(udp[3].c_str()),atof(udp[4].c_str()),atof(udp[5].c_str())));
                    n->vscan.push_back(ofVec3f(atof(udp[6].c_str()),atof(udp[7].c_str()),atof(udp[8].c_str())));
                }
            }
            if(ofGetElapsedTimef() - tm > 0.5){
                tm = ofGetElapsedTimef();
                n->oth.clear();
                n->ped.clear();
                n->obj.clear();
                n->vscan.clear();
            }
        }
    }
#endif
}

void *pcdThread(void *arg)
{
    PointcloudMap *p = reinterpret_cast<PointcloudMap *>(arg);
    p->searcharea();
}


int string2unixtime(string str){
    int rec=0;
    for(int i = 0; i < 13; i++){
        rec *= 10;
        rec += str[i] - '0';
    }
    return rec;
}
