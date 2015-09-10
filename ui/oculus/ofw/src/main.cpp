#include "testApp.h"

//--------------------------------------------------------------
int main(){
	// set width, height, mode (OF_WINDOW or OF_FULLSCREEN)
	ofSetupOpenGL(960*2, 1080, OF_WINDOW);

    testApp *tA = new testApp();

    pthread_t udp_t;
    pthread_create(&udp_t, NULL, udpThread, (void *) &tA->udp);
    pthread_t pcd_t;
    pthread_create(&pcd_t, NULL, pcdThread,(void *) &tA->pm);

    ofRunApp(tA); // start the app

    pthread_cancel(udp_t);
    pthread_cancel(pcd_t);

}
