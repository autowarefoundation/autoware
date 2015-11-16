/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "trans.h"

Two_dimensional_vector trans_scan2scan_image(Three_dimensional_vector* scan, CvMat* v_g2l, CvMat* v_g2c, CvMat* m_rotation, CvMat* m_intrinsic)
{
    float global_x;
    float global_y;
    float global_z;
    float camera_x;
    float camera_y;
    float camera_z;
    Two_dimensional_vector scan_image;
    scan_image.x.resize(scan->x.size());
    scan_image.y.resize(scan->y.size());

    for(int i = 0; i < (int)scan->x.size(); i++) {
        /*
         * Coordinate transformation. Change from laser range finder coordinate to global coordinate
         */
        global_y = -1.0 * scan->x.at(i) * 1000 + (cvmGet(v_g2l, 0, 0));
        global_x = scan->y.at(i) * 1000 + (cvmGet(v_g2l, 0, 1));
        global_z = -1.0*scan->z.at(i) * 1000 + (cvmGet(v_g2l, 0, 2));

        /*
         * Coordinate transformation. Change from global coordinate to camera coordinate
         */
        /* Rotation matrix */
        camera_x = cvmGet(m_rotation, 0, 0) * global_x + cvmGet(m_rotation, 0, 1) * global_y + cvmGet(m_rotation, 0, 2) * global_z;
        camera_y = cvmGet(m_rotation, 1, 0) * global_x + cvmGet(m_rotation, 1, 1) * global_y + cvmGet(m_rotation, 1, 2) * global_z;
        camera_z = cvmGet(m_rotation, 2, 0) * global_x + cvmGet(m_rotation, 2, 1) * global_y + cvmGet(m_rotation, 2, 2) * global_z;

        /* Transration vector */
        camera_x = camera_x + cvmGet(v_g2c, 0, 0);
        camera_y = camera_y + cvmGet(v_g2c, 0, 1);
        camera_z = camera_z + cvmGet(v_g2c, 0, 2);

        /*
         * Projection transformation. Change from camera coordinate to image coordinate
         */
        if (camera_z >= 0.0) {
            scan_image.x.push_back(((camera_x * cvmGet(m_intrinsic, 0, 0)) / camera_z) + cvmGet(m_intrinsic, 0, 2));
            scan_image.y.push_back(((camera_y * cvmGet(m_intrinsic, 1, 1)) / camera_z) + cvmGet(m_intrinsic, 1, 2));
        } else {
            scan_image.x.push_back(-1);
            scan_image.y.push_back(-1);
        }
    }

    return scan_image;
}
