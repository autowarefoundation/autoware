/***************************************************************************
 *   Copyright (C) 2005 by TAKEUCHI Eijiro,,,   *
 *   etake@bird5   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef __ALGEBRA__
#define __ALGEBRA__

#define E_ERROR 0.001

int mux_matrix2d(double s1[2][2], double s2[2][2],double dst[2][2]);
int mux_matrix3d(double s1[3][3], double s2[3][3],double dst[3][3]);
int mux_matrix(double *s1,double *s2,double *dst,int in, int kn, int 
jn);

int add_matrix2d(double s1[2][2], double s2[2][2],double dst[2][2]);
int add_matrix3d(double s1[3][3], double s2[3][3],double dst[3][3]);
int add_matrix6d(double s1[6][6], double s2[6][6],double dst[6][6]);

int sub_matrix2d(double s1[2][2], double s2[2][2],double dst[2][2]);
int sub_matrix3d(double s1[3][3], double s2[3][3],double dst[3][3]);

int identity_matrix2d(double dst[2][2]); 
int identity_matrix3d(double dst[3][3]); 
int identity_matrix6d(double dst[6][6]); 

int zero_matrix2d(double dst[2][2]); 
int zero_matrix3d(double dst[3][3]); 
int zero_matrix6d(double dst[6][6]); 

int transpose_matrix2d(double s1[2][2],double dst[2][2]);
int transpose_matrix3d(double s1[3][3],double dst[3][3]);

double determinant_matrix2d(double mat[2][2]);
double determinant_matrix3d(double mat[3][3]);

int inverse_matrix2d(double mat[2][2],double dst[2][2]);
int inverse_matrix3d(double mat[3][3], double dst[3][3]);

int eigenvalue_matrix2d(double mat[2][2],double *l1, double *l2);
int eigenvalue_matrix3d(double mat[3][3],double l1[2], double l2[2],double l3[2]);

int eigenvecter_matrix2d(double mat[2][2],double *v1,double *v2,double *l1,double *l2);
int eigenvecter_matrix3d(double mat[3][3],double v[3][3],double *l);

int matrix2d_eigen(double *v1,double *v2,double l1,double l2,double dst[2][2]);
int matrix3d_eigen(double v[3][3],double l1,double l2,double l3,double dst[3][3]);
int round_matrix3d(double mat[3][3],double dst[3][3] );
int ginverse_matrix3d(double mat[3][3], double dst[3][3]);
int ginverse_matrix6d(double mat[6][6], double dst[6][6]);
#endif
