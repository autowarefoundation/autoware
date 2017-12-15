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
#include "algebra.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int jacobi_matrix3d(int ct, double eps, double A[3][3], double A1[3][3], double X1[3][3]);
int kai_3(double a, double b, double c, double x1[2], double x2[2], double x3[2]);

int mux_matrix2d(double s1[2][2], double s2[2][2], double dst[2][2])
{
  dst[0][0] = s1[0][0] * s2[0][0] + s1[0][1] * s2[1][0];
  dst[0][1] = s1[0][0] * s2[0][1] + s1[0][1] * s2[1][1];
  dst[1][0] = s1[1][0] * s2[0][0] + s1[1][1] * s2[1][0];
  dst[1][1] = s1[1][0] * s2[0][1] + s1[1][1] * s2[1][1];
  return 1;
}

int mux_matrix3d(double s1[3][3], double s2[3][3], double dst[3][3])
{
  int i, j, k;
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      dst[i][j] = 0;
      for (k = 0; k < 3; k++)
      {
        dst[i][j] += s1[i][k] * s2[k][j];
      }
    }
  }
  return 1;
}

int mux_matrix(double *s1, double *s2, double *dst, int in, int kn, int jn)
{
  int i, j, k;

  for (i = 0; i < in; i++)
  {
    for (j = 0; j < jn; j++)
    {
      dst[i * jn + j] = 0;
      for (k = 0; k < kn; k++)
      {
        dst[i * jn + j] += s1[i * kn + k] * s2[k * jn + j];
      }
    }
  }
  return 1;
}


int add_matrix2d(double s1[2][2], double s2[2][2], double dst[2][2])
{
  dst[0][0] = s1[0][0] + s2[0][0];
  dst[0][1] = s1[0][1] + s2[0][1];
  dst[1][0] = s1[1][0] + s2[1][0];
  dst[1][1] = s1[1][1] + s2[1][1];
  return 1;
}

int add_matrix3d(double s1[3][3], double s2[3][3], double dst[3][3])
{
  int i, j;
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      dst[i][j] = s1[i][j] + s2[i][j];
    }
  }
  return 1;
}
int add_matrix6d(double s1[6][6], double s2[6][6], double dst[6][6])
{
  int i, j;
  for (i = 0; i < 6; i++)
  {
    for (j = 0; j < 6; j++)
    {
      dst[i][j] = s1[i][j] + s2[i][j];
    }
  }
  return 1;
}

/**/
int sub_matrix2d(double s1[2][2], double s2[2][2], double dst[2][2])
{
  dst[0][0] = s1[0][0] - s2[0][0];
  dst[0][1] = s1[0][1] - s2[0][1];
  dst[1][0] = s1[1][0] - s2[1][0];
  dst[1][1] = s1[1][1] - s2[1][1];
  return 1;
}

int sub_matrix3d(double s1[3][3], double s2[3][3], double dst[3][3])
{
  int i, j;
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      dst[i][j] = s1[i][j] - s2[i][j];
    }
  }
  return 1;
}

int identity_matrix2d(double dst[2][2])
{
  dst[0][0] = 1;
  dst[0][1] = 0;
  dst[1][0] = 0;
  dst[1][1] = 1;
  return 1;
}

int identity_matrix3d(double dst[3][3])
{
  int i, j;
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      if (i == j)
        dst[i][j] = 1;
      else
        dst[i][j] = 0;
    }
  }
  return 1;
}

int identity_matrix6d(double dst[6][6])
{
  int i, j;
  for (i = 0; i < 6; i++)
  {
    for (j = 0; j < 6; j++)
    {
      if (i == j)
        dst[i][j] = 1;
      else
        dst[i][j] = 0;
    }
  }
  return 1;
}

int zero_matrix2d(double dst[2][2])
{
  dst[0][0] = 0;
  dst[0][1] = 0;
  dst[1][0] = 0;
  dst[1][1] = 0;
  return 1;
}

int zero_matrix3d(double dst[3][3])
{
  int i, j;
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      dst[i][j] = 0;
    }
  }
  return 1;
}

int zero_matrix6d(double dst[6][6])
{
  int i, j;
  for (i = 0; i < 6; i++)
  {
    for (j = 0; j < 6; j++)
    {
      dst[i][j] = 0;
    }
  }
  return 1;
}

int transpose_matrix2d(double s1[2][2], double dst[2][2])
{
  dst[0][0] = s1[0][0];
  dst[0][1] = s1[1][0];
  dst[1][0] = s1[0][1];
  dst[1][1] = s1[1][1];
  return 1;
}

int transpose_matrix3d(double s1[3][3], double dst[3][3])
{
  int i, j;
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      dst[i][j] = s1[j][i];
    }
  }
  return 1;
}

double determinant_matrix2d(double mat[2][2])
{
  return (mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1]);
}

double determinant_matrix3d(double mat[3][3])
{
  return (mat[0][0] * mat[1][1] * mat[2][2] + mat[0][1] * mat[1][2] * mat[2][0] + mat[0][2] * mat[1][0] * mat[2][1] -
          mat[2][0] * mat[1][1] * mat[0][2] - mat[2][1] * mat[1][2] * mat[0][0] - mat[2][2] * mat[1][0] * mat[0][1]);
}

/*inverse matrix*/
int inverse_matrix2d(double mat[2][2], double dst[2][2])
{
  double inv;

  inv = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
  if (fabs(inv) == 0.0)
  {
    fprintf(stderr, "matrix is not regular matrix.\n");
    return 0;
  }

  dst[0][0] = mat[1][1] / inv;
  dst[0][1] = -mat[0][1] / inv;
  dst[1][0] = -mat[1][0] / inv;
  dst[1][1] = mat[0][0] / inv;

  return 1; /*regular matrix*/
}

int inverse_matrix3d(double mat[3][3], double dst[3][3])
{
  double d;
  d = determinant_matrix3d(mat);

  if (fabs(d) < E_ERROR)
    return 0;

  dst[0][0] = (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) / d;
  dst[1][0] = -(mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) / d;
  dst[2][0] = (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]) / d;

  dst[0][1] = -(mat[0][1] * mat[2][2] - mat[0][2] * mat[2][1]) / d;
  dst[1][1] = (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) / d;
  dst[2][1] = -(mat[0][0] * mat[2][1] - mat[0][1] * mat[2][0]) / d;

  dst[0][2] = (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) / d;
  dst[1][2] = -(mat[0][0] * mat[1][2] - mat[0][2] * mat[1][0]) / d;
  dst[2][2] = (mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]) / d;
  return 1;
}

/*eigen value*/
int eigenvalue_matrix2d(double mat[2][2], double *l1, double *l2)
{
  double a, b, c, x;

  a = 1.0;
  b = -mat[0][0] - mat[1][1];
  c = mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1];

  //	if(fabs(a) < E_ERROR)return(0);

  x = sqrt(b * b - 4 * a * c);
  *l1 = (-b + x) / 2 * a;
  *l2 = (-b - x) / 2 * a;
  if (fabs(*l1) < fabs(*l2))
  {
    x = *l1;
    *l1 = *l2;
    *l2 = x;
  }
  return 1;
}

int kai_3(double a, double b, double c, double x1[2], double x2[2], double x3[2])
{
  double p, q, m, n, w1[2], w2[2];
  p = b - a * a / 3.0;
  q = 2.0 * a * a * a / 27.0 - a * b / 3.0 + c;
  w1[0] = -1.0 / 2.0;
  w1[1] = sqrt(3) / 2.0;
  w2[0] = -1.0 / 2.0;
  w2[1] = -sqrt(3) / 2.0;

  m = pow((-q / 2.0 + sqrt(q * q / 4.0 + p * p * p / 27.0)), 1.0 / 3.0);
  n = pow((-q / 2.0 - sqrt(q * q / 4.0 + p * p * p / 27.0)), 1.0 / 3.0);

  x1[0] = m + n - a / 3.0;
  x1[1] = 0.0;
  x2[0] = w1[0] * m + w2[0] * n - a / 3.0;
  x2[1] = w1[1] * m + w2[1] * n;
  x3[0] = w2[0] * m + w1[0] * n - a / 3.0;
  x3[1] = w2[1] * m + w1[1] * n;
  return 0;
}

/*eigen value*/
int eigenvalue_matrix3d(double mat[3][3], double l1[2], double l2[2], double l3[2])
{
  double a, b, c;

  a = -mat[0][0] - mat[1][1] - mat[2][2];
  b = mat[0][0] * mat[1][1] + mat[1][1] * mat[2][2] + mat[2][2] * mat[0][0] -
      (mat[0][1] * mat[0][1] + mat[0][2] * mat[0][2] + mat[1][2] * mat[1][2]);
  c = -mat[0][0] * mat[1][1] * mat[2][2] - 2 * mat[0][1] * mat[0][2] * mat[1][2] + mat[0][0] * mat[1][2] * mat[1][2] +
      mat[1][1] * mat[0][2] * mat[0][2] + mat[2][2] * mat[0][1] * mat[0][1];
  kai_3(a, b, c, l1, l2, l3);

  //	if(fabs(a) < E_ERROR)return(0);
  return 0;
}
/*****************************************************************/
/*     ���оι���θ�ͭ�͡���ͭ�٥��ȥ�ʥ䥳��ˡ��              */
/*          n : ����                                             */
/*          ct : ���緫���֤����                                */
/*          eps : ��«Ƚ����                                   */
/*          A : �оݤȤ������                                   */
/*          A1, A2 : ��Ȱ��nxn�ι���ˡ�A1���г����Ǥ���ͭ��   */
/*          X1, X2 : ��Ȱ��nxn�ι���ˡ�X1�γ��󤬸�ͭ�٥��ȥ� */
/*          return : =0 : ����                                   */
/*                   =1 : ��«����                               */
/*          coded by Y.Suganuma                                  */
/*****************************************************************/

int jacobi_matrix3d(int ct, double eps, double A[3][3], double A1[3][3], double X1[3][3])
{
  double A2[3][3], X2[3][3];
  double max, s, t, v, sn, cs;
  int i1, i2, k = 0, ind = 1, p = 0, q = 0, n = 3;
  // �������
  for (i1 = 0; i1 < n; i1++)
  {
    for (i2 = 0; i2 < n; i2++)
    {
      A1[i1][i2] = A[i1][i2];
      X1[i1][i2] = 0.0;
    }
    X1[i1][i1] = 1.0;
  }
  // �׻�
  while (ind > 0 && k < ct)
  {
    // �������Ǥ�õ��
    max = 0.0;
    for (i1 = 0; i1 < n; i1++)
    {
      for (i2 = 0; i2 < n; i2++)
      {
        if (i2 != i1)
        {
          if (fabs(A1[i1][i2]) > max)
          {
            max = fabs(A1[i1][i2]);
            p = i1;
            q = i2;
          }
        }
      }
    }
    // ��«Ƚ��
    // ��«����
    if (max < eps)
      ind = 0;
    // ��«���ʤ�
    else
    {
      // ����
      s = -A1[p][q];
      t = 0.5 * (A1[p][p] - A1[q][q]);
      v = fabs(t) / sqrt(s * s + t * t);
      sn = sqrt(0.5 * (1.0 - v));
      if (s * t < 0.0)
        sn = -sn;
      cs = sqrt(1.0 - sn * sn);
      // Ak�η׻�
      for (i1 = 0; i1 < n; i1++)
      {
        if (i1 == p)
        {
          for (i2 = 0; i2 < n; i2++)
          {
            if (i2 == p)
              A2[p][p] = A1[p][p] * cs * cs + A1[q][q] * sn * sn - 2.0 * A1[p][q] * sn * cs;
            else if (i2 == q)
              A2[p][q] = 0.0;
            else
              A2[p][i2] = A1[p][i2] * cs - A1[q][i2] * sn;
          }
        }
        else if (i1 == q)
        {
          for (i2 = 0; i2 < n; i2++)
          {
            if (i2 == q)
              A2[q][q] = A1[p][p] * sn * sn + A1[q][q] * cs * cs + 2.0 * A1[p][q] * sn * cs;
            else if (i2 == p)
              A2[q][p] = 0.0;
            else
              A2[q][i2] = A1[q][i2] * cs + A1[p][i2] * sn;
          }
        }
        else
        {
          for (i2 = 0; i2 < n; i2++)
          {
            if (i2 == p)
              A2[i1][p] = A1[i1][p] * cs - A1[i1][q] * sn;
            else if (i2 == q)
              A2[i1][q] = A1[i1][q] * cs + A1[i1][p] * sn;
            else
              A2[i1][i2] = A1[i1][i2];
          }
        }
      }
      // Xk�η׻�
      for (i1 = 0; i1 < n; i1++)
      {
        for (i2 = 0; i2 < n; i2++)
        {
          if (i2 == p)
            X2[i1][p] = X1[i1][p] * cs - X1[i1][q] * sn;
          else if (i2 == q)
            X2[i1][q] = X1[i1][q] * cs + X1[i1][p] * sn;
          else
            X2[i1][i2] = X1[i1][i2];
        }
      }
      // ���Υ��ƥåפ�
      k++;
      for (i1 = 0; i1 < n; i1++)
      {
        for (i2 = 0; i2 < n; i2++)
        {
          A1[i1][i2] = A2[i1][i2];
          X1[i1][i2] = X2[i1][i2];
        }
      }
    }
  }

  if (ind)
    k = -1;
  return k;
}

/*eigen vecter*/
int eigenvecter_matrix2d(double mat[2][2], double *v1, double *v2, double *l1, double *l2)
{
  double a;

  if (!eigenvalue_matrix2d(mat, l1, l2))
    return 0;

  v1[0] = mat[0][1];
  v1[1] = -(mat[0][0] - *l1);
  a = sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
  v1[0] = v1[0] / a;
  v1[1] = v1[1] / a;

  v2[0] = mat[1][1] - *l2;
  v2[1] = -mat[1][0];
  a = sqrt(v2[0] * v2[0] + v2[1] * v2[1]);
  v2[0] = v2[0] / a;
  v2[1] = v2[1] / a;

  return 1;
}

/*eigen vecter*/
int eigenvecter_matrix3d(double mat[3][3], double v[3][3], double *l)
{
  double L[3][3], V[3][3];
  int i;

  if ((i = jacobi_matrix3d(10000, 0.0000000001, mat, L, V)) < 0)
    return -1;

  /*sort*/
  if (fabs(L[0][0]) > fabs(L[1][1]))
  {
    if (fabs(L[0][0]) > fabs(L[2][2]))
    {
      l[0] = L[0][0];
      v[0][0] = V[0][0];
      v[1][0] = V[1][0];
      v[2][0] = V[2][0];
      if (fabs(L[1][1]) > fabs(L[2][2]))
      {
        l[1] = L[1][1];
        v[0][1] = V[0][1];
        v[1][1] = V[1][1];
        v[2][1] = V[2][1];
        l[2] = L[2][2];
        v[0][2] = V[0][2];
        v[1][2] = V[1][2];
        v[2][2] = V[2][2];
      }
      else
      {
        l[2] = L[1][1];
        v[0][2] = V[0][1];
        v[1][2] = V[1][1];
        v[2][2] = V[2][1];
        l[1] = L[2][2];
        v[0][1] = V[0][2];
        v[1][1] = V[1][2];
        v[2][1] = V[2][2];
      }
    }
    else
    {
      l[0] = L[2][2];
      v[0][0] = V[0][2];
      v[1][0] = V[1][2];
      v[2][0] = V[2][2];
      l[1] = L[0][0];
      v[0][1] = V[0][0];
      v[1][1] = V[1][0];
      v[2][1] = V[2][0];
      l[2] = L[1][1];
      v[0][2] = V[0][1];
      v[1][2] = V[1][1];
      v[2][2] = V[2][1];
    }
  }
  else
  {
    if (fabs(L[0][0]) < fabs(L[2][2]))
    {
      l[2] = L[0][0];
      v[0][2] = V[0][0];
      v[1][2] = V[1][0];
      v[2][2] = V[2][0];
      if (fabs(L[1][1]) > fabs(L[2][2]))
      {
        l[0] = L[1][1];
        v[0][0] = V[0][1];
        v[1][0] = V[1][1];
        v[2][0] = V[2][1];
        l[1] = L[2][2];
        v[0][1] = V[0][2];
        v[1][1] = V[1][2];
        v[2][1] = V[2][2];
      }
      else
      {
        l[0] = L[2][2];
        v[0][0] = V[0][2];
        v[1][0] = V[1][2];
        v[2][0] = V[2][2];
        l[1] = L[1][1];
        v[0][1] = V[0][1];
        v[1][1] = V[1][1];
        v[2][1] = V[2][1];
      }
    }
    else
    {
      l[0] = L[1][1];
      v[0][0] = V[0][1];
      v[1][0] = V[1][1];
      v[2][0] = V[2][1];
      l[1] = L[0][0];
      v[0][1] = V[0][0];
      v[1][1] = V[1][0];
      v[2][1] = V[2][0];
      l[2] = L[2][2];
      v[0][2] = V[0][2];
      v[1][2] = V[1][2];
      v[2][2] = V[2][2];
    }
  }
  return i;
}

int matrix2d_eigen(double *v1, double *v2, double l1, double l2, double dst[2][2])
{
  double V[2][2], IV[2][2], A[2][2], B[2][2];

  V[0][0] = v1[0];
  V[1][0] = v1[1];
  V[0][1] = v2[0];
  V[1][1] = v2[1];

  A[0][0] = l1;
  A[0][1] = 0;
  A[1][0] = 0;
  A[1][1] = l2;

  if (!inverse_matrix2d(V, IV))
    return 0;

  mux_matrix2d(V, A, B);
  mux_matrix2d(B, IV, dst);
  return 1;
}

int matrix3d_eigen(double v[3][3], double l1, double l2, double l3, double dst[3][3])
{
  double IV[3][3], A[3][3], B[3][3];

  A[0][0] = l1;
  A[0][1] = 0;
  A[0][2] = 0;

  A[1][0] = 0;
  A[1][1] = l2;
  A[1][2] = 0;

  A[2][0] = 0;
  A[2][1] = 0;
  A[2][2] = l3;

  if (!inverse_matrix3d(v, IV))
    return 0;

  mux_matrix3d(v, A, B);
  mux_matrix3d(B, IV, dst);
  return 1;
}

int round_matrix3d(double mat[3][3], double dst[3][3])
{
  double v[3][3], l[3], a;

  eigenvecter_matrix3d(mat, v, l);
  //  print_matrix3d(v);
  printf("l1 = %f,l2 = %f,l3 = %f  (%f)\n", l[0], l[1], l[2],
         v[0][0] * v[0][0] + v[1][0] * v[1][0] + v[2][0] * v[2][0]);

  if (v[0][0] * v[1][0] + v[0][1] * v[1][1] + v[0][2] * v[1][2] != 0)
    return 0;
  if (v[1][0] * v[2][0] + v[1][1] * v[2][1] + v[1][2] * v[2][2] != 0)
    return 0;
  if (v[2][0] * v[0][0] + v[2][1] * v[0][1] + v[2][2] * v[0][2] != 0)
    return 0;

  a = fabs(l[1] / l[0]);
  if (a < 0.0001)
  {
    if (l[1] > 0)
      l[1] = fabs(l[0]) / 1000.0;
    else
      l[1] = -fabs(l[0]) / 1000.0;

    a = fabs(l[2] / l[0]);
    if (a < -0.0001)
    {
      if (l[2] > 0)
        l[2] = fabs(l[0]) / 1000.0;
      else
        l[2] = -fabs(l[0]) / 1000.0;
    }
    printf("r");
    matrix3d_eigen(v, l[0], l[1], l[2], mat);
  }
  return 1;
}

int ginverse_matrix3d(double mat[3][3], double inv[3][3])
{
  double p, d, max, dumy, A[3][3];
  int i, j, k, s;

  identity_matrix3d(inv);
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      A[i][j] = mat[i][j];
    }
  }

  for (k = 0; k < 3; k++)
  {
    max = -100000000;
    s = k;
    for (j = k; j < 3; j++)
    {
      if (fabs(A[j][k]) > max)
      {
        max = fabs(A[j][k]);
        s = j;
      }
    }
    if (max == -100000000)
      return 0;

    for (j = 0; j < 3; j++)
    {
      dumy = A[k][j];
      A[k][j] = A[s][j];
      A[s][j] = dumy;

      dumy = inv[k][j];
      inv[k][j] = inv[s][j];
      inv[s][j] = dumy;
    }

    p = A[k][k];
    for (j = k; j < 3; j++)
    {
      A[k][j] = A[k][j] / p;
    }
    for (j = 0; j < 3; j++)
    {
      inv[k][j] = inv[k][j] / p;
    }

    for (i = 0; i < 3; i++)
    {
      if (i != k)
      {
        d = A[i][k];
        for (j = 0; j < 3; j++)
        {
          inv[i][j] = inv[i][j] - d * inv[k][j];
        }
        for (j = k; j < 3; j++)
        {
          A[i][j] = A[i][j] - d * A[k][j];
        }
      }
    }
  }
  return 1;
}

int ginverse_matrix6d(double mat[6][6], double inv[6][6])
{
  double p, d, max, dumy, A[6][6];
  int i, j, k, s;

  identity_matrix6d(inv);
  for (i = 0; i < 6; i++)
  {
    for (j = 0; j < 6; j++)
    {
      A[i][j] = mat[i][j];
    }
  }

  for (k = 0; k < 6; k++)
  {
    max = -100000000;
    s = k;
    for (j = k; j < 6; j++)
    {
      if (fabs(A[j][k]) > max)
      {
        max = fabs(A[j][k]);
        s = j;
      }
    }
    if (max == 100000000)
      return 0;

    for (j = 0; j < 6; j++)
    {
      dumy = A[k][j];
      A[k][j] = A[s][j];
      A[s][j] = dumy;

      dumy = inv[k][j];
      inv[k][j] = inv[s][j];
      inv[s][j] = dumy;
    }

    p = A[k][k];
    for (j = k; j < 6; j++)
    {
      A[k][j] = A[k][j] / p;
    }
    for (j = 0; j < 6; j++)
    {
      inv[k][j] = inv[k][j] / p;
    }

    for (i = 0; i < 6; i++)
    {
      if (i != k)
      {
        d = A[i][k];
        for (j = 0; j < 6; j++)
        {
          inv[i][j] = inv[i][j] - d * inv[k][j];
        }
        for (j = k; j < 6; j++)
        {
          A[i][j] = A[i][j] - d * A[k][j];
        }
      }
    }
  }
  return 1;
}
