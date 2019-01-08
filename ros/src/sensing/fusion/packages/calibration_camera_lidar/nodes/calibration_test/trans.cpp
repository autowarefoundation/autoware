/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
