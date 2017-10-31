#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "ndt.h"
#include "algebra.h"

#define E_THETA 0.0001

//#define WEIGHTED_SELECT 1
#define WEIGHTED_SELECT 0

extern int point_num;
extern NDMapPtr NDmap;
extern int layer_select;

extern double scan_points_weight[];
extern double scan_points_totalweight;

extern int _downsampler_num;

double qdd[3][3][2];
double qd[3][2];
double qd3[6][3];
double qdd3[6][6][3];

void set_sincos(double a, double b, double g, double sc_d[3][3][3]);
void set_sincos2(double a, double b, double g, double sc[3][3]);
int check_Hessian(double H[3][3]);
void save_data(PointPtr scan, int num, PosturePtr pose);
void depth(PointPtr scan, int num, PosturePtr pose);

/*
�����̤���뤿��η׻��ʰ���ʬ��
�إå�����η׻��⤦�����ڤǤ��뤫�⡣
�켡��ʬ�����ο������׻����ơ��إå�����Ϥ���κ�ʬ������롣*/
double calc_summand3d(PointPtr p, NDPtr nd, PosturePtr pose, double *g, double H[6][6], double qd3_d[6][3], double dist)
{
  double a[3];
  double e;
  double q[3];
  double qda[6][3], *qda_p;  //,*qdd_p;
  int i, j;

  /*q�η׻�*/
  q[0] = p->x - nd->mean.x;
  q[1] = p->y - nd->mean.y;
  q[2] = p->z - nd->mean.z;

  /*exp�η׻�*/
  //  e = probability_on_ND(nd, p->x, p->y, p->z);
  e = probability_on_ND(nd, q[0], q[1], q[2]) * dist;

  if (e < 0.000000001)
  {
    for (i = 0; i < 6; i++)
    {
      g[i] = 0;
      for (j = 0; j < 6; j++)
      {
        H[i][j] = 0;
      }
    }
    return 0;
  }
  /*  */
  a[0] = q[0] * nd->inv_covariance[0][0] + q[1] * nd->inv_covariance[1][0] + q[2] * nd->inv_covariance[2][0];
  a[1] = q[0] * nd->inv_covariance[0][1] + q[1] * nd->inv_covariance[1][1] + q[2] * nd->inv_covariance[2][1];
  a[2] = q[0] * nd->inv_covariance[0][2] + q[1] * nd->inv_covariance[1][2] + q[2] * nd->inv_covariance[2][2];

  g[0] = (a[0] * qd3_d[0][0] + a[1] * qd3_d[0][1] + a[2] * qd3_d[0][2]);
  g[1] = (a[0] * qd3_d[1][0] + a[1] * qd3_d[1][1] + a[2] * qd3_d[1][2]);
  g[2] = (a[0] * qd3_d[2][0] + a[1] * qd3_d[2][1] + a[2] * qd3_d[2][2]);
  g[3] = (a[0] * qd3_d[3][0] + a[1] * qd3_d[3][1] + a[2] * qd3_d[3][2]);
  g[4] = (a[0] * qd3_d[4][0] + a[1] * qd3_d[4][1] + a[2] * qd3_d[4][2]);
  g[5] = (a[0] * qd3_d[5][0] + a[1] * qd3_d[5][1] + a[2] * qd3_d[5][2]);

  for (j = 0; j < 6; j++)
  {
    qda[j][0] = qd3[j][0] * nd->inv_covariance[0][0] + qd3[j][1] * nd->inv_covariance[1][0] +
                qd3[j][2] * nd->inv_covariance[2][0];
    qda_p++;
    qda[j][1] = qd3[j][0] * nd->inv_covariance[0][1] + qd3[j][1] * nd->inv_covariance[1][1] +
                qd3[j][2] * nd->inv_covariance[2][1];
    qda_p++;
    qda[j][2] = qd3[j][0] * nd->inv_covariance[0][2] + qd3[j][1] * nd->inv_covariance[1][2] +
                qd3[j][2] * nd->inv_covariance[2][2];
    qda_p++;
  }

  for (i = 0; i < 6; i++)
  {
    for (j = 0; j < 6; j++)
    {
      H[i][j] = -e * ((-g[i]) * (g[j]) - (a[0] * qdd3[i][j][0] + a[1] * qdd3[i][j][1] + a[2] * qdd3[i][j][2]) -
                      (qda[j][0] * qd3[i][0] + qda[j][1] * qd3[i][1] + qda[j][2] * qd3[i][2]));
    }
  }

  for (i = 0; i < 6; i++)
    g[i] = g[i] * e;

  return e;
}

/*�ȤäƤʤ���*/
int check_Hessian(double H[3][3])
{
  int i, j;
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      if (H[i][j] < -0.0001)
        return 0;
    }
  }
  return 1;
}

/*�ǡ�������¸*/
void save_data(PointPtr scan, int num, PosturePtr pose)
{
  double sc[3][3], x, y, z;
  int i;
  FILE *scanfile;
  Point p;
  scanfile = fopen("scan", "w");

  set_sincos2(pose->theta, pose->theta2, pose->theta3, sc);
  for (i = 0; i < num; i++)
  {
    x = scan[i].x;
    y = scan[i].y;
    z = scan[i].z;

    p.x = x * sc[0][0] + y * sc[0][1] + z * sc[0][2] + pose->x;
    p.y = x * sc[1][0] + y * sc[1][1] + z * sc[1][2] + pose->y;
    p.z = x * sc[2][0] + y * sc[2][1] + z * sc[2][2] + pose->z;

    fprintf(scanfile, "%f %f %f \n", p.x, p.y, p.z);
  }
  fclose(scanfile);
}

void scan_transrate(PointPtr src, PointPtr dst, PosturePtr pose, int num)
{
  double sc[3][3], x, y, z;
  int i;

  PointPtr p, q;

  p = src;
  q = dst;

  set_sincos2(pose->theta, pose->theta2, pose->theta3, sc);
  for (i = 0; i < num; i++)
  {
    x = p->x;
    y = p->y;
    z = p->z;

    q->x = x * sc[0][0] + y * sc[0][1] + z * sc[0][2] + pose->x;
    q->y = x * sc[1][0] + y * sc[1][1] + z * sc[1][2] + pose->y;
    q->z = x * sc[2][0] + y * sc[2][1] + z * sc[2][2] + pose->z;

    p++;
    q++;
  }
}

/*��*/
void depth(PointPtr scan, int num, PosturePtr pose)
{
  double sc[3][3], x, y, z;
  int i;

  Point p;

  set_sincos2(pose->theta, pose->theta2, pose->theta3, sc);
  for (i = 0; i < num; i++)
  {
    x = scan[i].x;
    y = scan[i].y;
    z = scan[i].z;

    p.x = x * sc[0][0] + y * sc[0][1] + z * sc[0][2] + pose->x;
    p.y = x * sc[1][0] + y * sc[1][1] + z * sc[1][2] + pose->y;
    p.z = x * sc[2][0] + y * sc[2][1] + z * sc[2][2] + pose->z;
  }
}

/*���ʬ�ν���*/
double adjust3d(PointPtr scan, int num, PosturePtr initial, int target)
{
  // double gsum[6], Hsum[6][6],Hsumh[6][6],Hinv[6][6],g[6],gd[6],ge[6][6],H[6][6],hH[6][6];
  double gsum[6], Hsum[6][6], Hsumh[6][6], Hinv[6][6], g[6], H[6][6], hH[6][6];
  // double sc[3][3],sc_d[3][3][3],sc_dd[3][3][3][3],sce[3][3][3];
  double sc[3][3], sc_d[3][3][3], sc_dd[3][3][3][3];
  // double *work,*work2,*work3;
  double *work;
  double esum = 0, gnum = 0;
  NDPtr nd[8];
  NDMapPtr nd_map;
  int i, j, n, m, k, layer;
  double x, y, z;  //,sa,ca,sb,cb,sg,cg;
  PosturePtr pose;
  // Point p,pe[6],pd;
  Point p;
  PointPtr scanptr;
  // int inc,count;
  int inc;
  int ndmode;
  double dist, weight_total, weight_sum, weight_next;

  /*initialize*/
  gsum[0] = 0;
  gsum[1] = 0;
  gsum[2] = 0;
  gsum[3] = 0;
  gsum[4] = 0;
  gsum[5] = 0;
  j = 0;
  zero_matrix6d(Hsum);
  zero_matrix6d(Hsumh);
  pose = initial;

  /*�Ѵ������1����ʬʬ��ޤ�ˤβ�žʬ��׻�*/
  set_sincos(pose->theta, pose->theta2, pose->theta3, sc_d);
  set_sincos(pose->theta + E_THETA, pose->theta2, pose->theta3, sc_dd[0]);
  set_sincos(pose->theta, pose->theta2 + E_THETA, pose->theta3, sc_dd[1]);
  set_sincos(pose->theta, pose->theta2, pose->theta3 + E_THETA, sc_dd[2]);

  /*��ɸ�Ѵ���*/
  set_sincos2(pose->theta, pose->theta2, pose->theta3, sc);

  /*�켡��ʬ������Ѳ����ʤ���ʬ�η׻�*/
  qd3[0][0] = 1;
  qd3[0][1] = 0;
  qd3[0][2] = 0;

  qd3[1][0] = 0;
  qd3[1][1] = 1;
  qd3[1][2] = 0;

  qd3[2][0] = 0;
  qd3[2][1] = 0;
  qd3[2][2] = 1;
  for (n = 0; n < 6; n++)
  {
    for (m = 0; m < 6; m++)
    {
      for (k = 0; k < 3; k++)
      {
        qdd3[n][m][k] = 0;
      }
    }
  }

  //#if WEIGHTED_SELECT
  if (_downsampler_num == 0)
  {
    /*�ǡ��������Ф�����1=��ĤŤġ�*/
    switch (target)
    {
      case 3:
        inc = 1;
        ndmode = 0;
        break;
      case 2:
        inc = 500;
        ndmode = 1;
        break;
      case 1:
        inc = 5000;
        ndmode = 0;
        break;
      default:
        inc = 5000;
        ndmode = 0;
        break;
    }
  }
  //#else
  if (_downsampler_num == 1)
  {
    /*�ǡ��������Ф�����1=��ĤŤġ�*/
    switch (target)
    {
      case 3:
        inc = 1;
        ndmode = 0;
        break;
      case 2:
        inc = 1;
        ndmode = 1;
        break;
      case 1:
        inc = 1;
        ndmode = 0;
        break;
      default:
        inc = 1;
        ndmode = 0;
        break;
    }
  }
  //#endif

  scanptr = scan;

  /*����ˤĤ��Ʒ����֤��׻�*/

  //#if WEIGHTED_SELECT
  if (_downsampler_num == 0)
  {
    weight_total = scan_points_totalweight;
    ;
    weight_next = 0;
    weight_sum = 0;

    //  FILE *point_fp;
    // point_fp=fopen("/tmp/range","w");
    for (i = 0; i < num; i++)
    {
      weight_sum += scan_points_weight[i];
      if (weight_sum < weight_next)
      {
        scanptr++;
        continue;
      }

      /*���κ�ɸ�Ѵ��׻�*/
      x = scanptr->x;
      y = scanptr->y;
      z = scanptr->z;
      //    fprintf(point_fp,"%f %f %f \n",x,y,z);

      scanptr++;
      weight_next += weight_total / (double)inc;  // 1000;
      dist = 1;

      p.x = x * sc[0][0] + y * sc[0][1] + z * sc[0][2] + pose->x;
      p.y = x * sc[1][0] + y * sc[1][1] + z * sc[1][2] + pose->y;
      p.z = x * sc[2][0] + y * sc[2][1] + z * sc[2][2] + pose->z;

      /*���ϥ������ˤ�����������*/
      if (ndmode == 1)
        layer = 1;  // layer_select;
      if (ndmode == 0)
        layer = 0;  // layer_select;
      nd_map = NDmap;

      while (layer > 0)
      {
        if (nd_map->next)
          nd_map = nd_map->next;
        layer--;
      }

      /*�����б�����ND�ܥ�����������Ʊ���˼�������ND�ܥ�����򹹿���
        �٤����Τ�Ĥ������Ӥ���Ĥ�Ĥ�����*/

      if (!get_ND(nd_map, &p, nd, target))
        continue;

      /*q�ΰ켡��ʬ(�Ѳ�������Τ�)*/
      work = (double *)sc_d;
      for (m = 0; m < 3; m++)
      {
        for (k = 0; k < 3; k++)
        {
          // qd3[txtytzabg][xyz]
          qd3[m + 3][k] = x * (*work) + y * (*(work + 1)) + z * (*(work + 2));
          // x*sc_d[m][k][0] + y*sc_d[m][k][1] + z*sc_d[m][k][2];
          work += 3;
        }
      }

      /*q������ʬ���Ѳ�������Τߡ�*/
      work = (double *)sc_dd;
      for (n = 0; n < 3; n++)
      {
        for (m = 0; m < 3; m++)
        {
          for (k = 0; k < 3; k++)
          {
            qdd3[n + 3][m + 3][k] = (*work * x + *(work + 1) * y + *(work + 2) * z - qd3[m + 3][k]) / E_THETA;
            work += 3;
          }
        }
      }

      /*�����̷׻�*/
      if (nd[j])
      {
        if (nd[j]->num > 10 && nd[j]->sign == 1)
        {
          //	double e;
          esum += calc_summand3d(&p, nd[j], pose, g, hH, qd3, dist);
          add_matrix6d(Hsumh, hH, Hsumh);

          //	  dist =1;
          gsum[0] += g[0];                //*nd[j]->w;
          gsum[1] += g[1];                //*nd[j]->w;
          gsum[2] += g[2] + pose->z * 0;  //*nd[j]->w;
          gsum[3] += g[3];                //*nd[j]->w;
          gsum[4] += g[4];                //+(pose->theta2-(0.0))*1;//*nd[j]->w;
          gsum[5] += g[5];                //*nd[j]->w;
          gnum += 1;  // nd[j]->w;
        }
      }
    }
  }

  //#else
  if (_downsampler_num == 1)
  {
    for (i = 0; i < num; i += inc)
    {
      //    dist = (x*x+y*y+z*z);
      // dist *= (1.2-exp(-1*(-1 - z)*(-1 - z)/4.0));
      //    if(dist>2500)dist=2500;
      /*���κ�ɸ�Ѵ��׻�*/
      x = scanptr->x;
      y = scanptr->y;
      z = scanptr->z;
      dist = 1;
      scanptr += inc;

      p.x = x * sc[0][0] + y * sc[0][1] + z * sc[0][2] + pose->x;
      p.y = x * sc[1][0] + y * sc[1][1] + z * sc[1][2] + pose->y;
      p.z = x * sc[2][0] + y * sc[2][1] + z * sc[2][2] + pose->z;

      /*���ϥ������ˤ�����������*/
      if (ndmode == 1)
        layer = 1;  // layer_select;
      if (ndmode == 0)
        layer = 0;  // layer_select;
      nd_map = NDmap;

      while (layer > 0)
      {
        if (nd_map->next)
          nd_map = nd_map->next;
        layer--;
      }

      /*�����б�����ND�ܥ�����������Ʊ���˼�������ND�ܥ�����򹹿���
        �٤����Τ�Ĥ������Ӥ���Ĥ�Ĥ�����*/

      if (!get_ND(nd_map, &p, nd, target))
        continue;

      /*q�ΰ켡��ʬ(�Ѳ�������Τ�)*/
      work = (double *)sc_d;
      for (m = 0; m < 3; m++)
      {
        for (k = 0; k < 3; k++)
        {
          // qd3[txtytzabg][xyz]
          qd3[m + 3][k] = x * (*work) + y * (*(work + 1)) + z * (*(work + 2));
          // x*sc_d[m][k][0] + y*sc_d[m][k][1] + z*sc_d[m][k][2];
          work += 3;
        }
      }

      /*q������ʬ���Ѳ�������Τߡ�*/
      work = (double *)sc_dd;
      for (n = 0; n < 3; n++)
      {
        for (m = 0; m < 3; m++)
        {
          for (k = 0; k < 3; k++)
          {
            qdd3[n + 3][m + 3][k] = (*work * x + *(work + 1) * y + *(work + 2) * z - qd3[m + 3][k]) / E_THETA;
            work += 3;
          }
        }
      }

      /*�����̷׻�*/
      if (nd[j])
      {
        if (nd[j]->num > 10 && nd[j]->sign == 1)
        {
          //	double e;
          esum += calc_summand3d(&p, nd[j], pose, g, hH, qd3, dist);
          add_matrix6d(Hsumh, hH, Hsumh);

          //	  dist =1;
          gsum[0] += g[0];                //*nd[j]->w;
          gsum[1] += g[1];                //*nd[j]->w;
          gsum[2] += g[2] + pose->z * 0;  //*nd[j]->w;
          gsum[3] += g[3];                //*nd[j]->w;
          gsum[4] += g[4];                //+(pose->theta2-(0.0))*1;//*nd[j]->w;
          gsum[5] += g[5];                //*nd[j]->w;
          gnum += 1;  // nd[j]->w;
        }
      }
    }
  }
  //#endif

  if (gnum > 1)
  {
    //  printf("gnum=%lf\n",gnum);
    //    fclose(point_fp);
    identity_matrix6d(H);
    H[0][0] = H[0][0] / (gnum * gnum * 1000.001);
    H[1][1] = H[1][1] / (gnum * gnum * 1000.001);
    H[2][2] = H[2][2] / (gnum * gnum * 1000.001);
    H[3][3] = H[3][3] / (gnum * gnum * 0.001);
    H[4][4] = H[4][4] / (gnum * gnum * 0.001);
    H[5][5] = H[5][5] / (gnum * gnum * 0.001);

    add_matrix6d(Hsumh, H, Hsumh);

    ginverse_matrix6d(Hsumh, Hinv);

    /*----------------����------------------------*/
    pose->x -= (Hinv[0][0] * gsum[0] + Hinv[0][1] * gsum[1] + Hinv[0][2] * gsum[2] + Hinv[0][3] * gsum[3] +
                Hinv[0][4] * gsum[4] + Hinv[0][5] * gsum[5]);
    pose->y -= (Hinv[1][0] * gsum[0] + Hinv[1][1] * gsum[1] + Hinv[1][2] * gsum[2] + Hinv[1][3] * gsum[3] +
                Hinv[1][4] * gsum[4] + Hinv[1][5] * gsum[5]);
    pose->z -= (Hinv[2][0] * gsum[0] + Hinv[2][1] * gsum[1] + Hinv[2][2] * gsum[2] + Hinv[2][3] * gsum[3] +
                Hinv[2][4] * gsum[4] + Hinv[2][5] * gsum[5]);
    pose->theta -= (Hinv[3][0] * gsum[0] + Hinv[3][1] * gsum[1] + Hinv[3][2] * gsum[2] + Hinv[3][3] * gsum[3] +
                    Hinv[3][4] * gsum[4] + Hinv[3][5] * gsum[5]);
    pose->theta2 -= (Hinv[4][0] * gsum[0] + Hinv[4][1] * gsum[1] + Hinv[4][2] * gsum[2] + Hinv[4][3] * gsum[3] +
                     Hinv[4][4] * gsum[4] + Hinv[4][5] * gsum[5]);
    pose->theta3 -= (Hinv[5][0] * gsum[0] + Hinv[5][1] * gsum[1] + Hinv[5][2] * gsum[2] + Hinv[5][3] * gsum[3] +
                     Hinv[5][4] * gsum[4] + Hinv[5][5] * gsum[5]);
  }
  return esum;
}

void set_sincos2(double a, double b, double g, double sc[3][3])
{
  double sa, ca, sb, cb, sg, cg;
  double Rx[3][3], Ry[3][3], Rz[3][3], R[3][3], W[3][3];
  sa = sin(a);
  ca = cos(a);
  sb = sin(b);
  cb = cos(b);
  sg = sin(g);
  cg = cos(g);

  Rx[0][0] = 1;
  Rx[0][1] = 0;
  Rx[0][2] = 0;
  Rx[1][0] = 0;
  Rx[1][1] = ca;
  Rx[1][2] = -sa;
  Rx[2][0] = 0;
  Rx[2][1] = sa;
  Rx[2][2] = ca;

  Ry[0][0] = cb;
  Ry[0][1] = 0;
  Ry[0][2] = sb;
  Ry[1][0] = 0;
  Ry[1][1] = 1;
  Ry[1][2] = 0;
  Ry[2][0] = -sb;
  Ry[2][1] = 0;
  Ry[2][2] = cb;

  Rz[0][0] = cg;
  Rz[0][1] = -sg;
  Rz[0][2] = 0;
  Rz[1][0] = sg;
  Rz[1][1] = cg;
  Rz[1][2] = 0;
  Rz[2][0] = 0;
  Rz[2][1] = 0;
  Rz[2][2] = 1;

  identity_matrix3d(W);
  mux_matrix3d(W, Rz, R);
  mux_matrix3d(R, Ry, W);
  mux_matrix3d(W, Rx, sc);

  /*
  sc[0][0] =  ca*cb*cg-sa*sg;
  sc[0][1] = -ca*cb*sg -sa*cg;
  sc[0][2] =  ca*sb;
  sc[1][0] =  sa*cb*cg+ca*sg;
  sc[1][1] = -sa*cb*sg+ca*cg;
  sc[1][2] =  sa*sb;
  sc[2][0] = -sb*cg;
  sc[2][1] =  sb*sg;
  sc[2][2] =  cb;
  */

  /*
  sc[0][0] =  ca*cb*cg-sa*sg;
  sc[0][1] = -ca*cb*sg -sa*cg;
  sc[0][2] =  ca*sb;
  sc[1][0] =  sa*cb*cg+ca*sg;
  sc[1][1] = -sa*cb*sg+ca*cg;
  sc[1][2] =  sa*sb;
  sc[2][0] = -sb*cg;
  sc[2][1] =  sb*sg;
  sc[2][2] =  cb;
  */
}

void set_sincos(double a, double b, double g, double sc[3][3][3])
{
  //  double sa,ca,sb,cb,sg,cg;
  double dd[3][3][3], d[3][3];
  int i, j, k;

  set_sincos2(a, b, g, d);
  set_sincos2(a + 0.0001, b, g, dd[0]);
  set_sincos2(a, b + 0.0001, g, dd[1]);
  set_sincos2(a, b, g + 0.0001, dd[2]);

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      for (k = 0; k < 3; k++)
      {
        sc[i][j][k] = (dd[i][j][k] - d[j][k]) / 0.0001;
      }
    }
  }

  /*
  sa = sin(a);
  ca = cos(a);
  sb = sin(b);
  cb = cos(b);
  sg = sin(g);
  cg = cos(g);

  //sc[tx ty tz a b g][x y z][x+y+z ]
  sc[0][0][0] = -sa*cb*cg-ca*sg;
  sc[0][0][1] =  sa*cb*sg-ca*cg;
  sc[0][0][2] = -sa*sb;
  sc[0][1][0] =  ca*cb*cg-sa*sg;
  sc[0][1][1] = -ca*cb*sg-sa*cg;
  sc[0][1][2] =  ca*sb;
  sc[0][2][0] =  0;
  sc[0][2][1] =  0;
  sc[0][2][2] =  0;

  sc[1][0][0] = -ca*sb*cg;
  sc[1][0][1] =  ca*sb*sg;
  sc[1][0][2] =  ca*cb;
  sc[1][1][0] = -sa*sb*cg;
  sc[1][1][1] =  sa*sb*sg;
  sc[1][1][2] =  sa*cb;
  sc[1][2][0] = -cb*cg;
  sc[1][2][1] =  cb*sg;
  sc[1][2][2] = -sb;


  sc[2][0][0] = -ca*cb*sg-sa*cg;
  sc[2][0][1] = -ca*cb*cg+sa*sg;
  sc[2][0][2] =  0;
  sc[2][1][0] = -sa*cb*sg+ca*cg;
  sc[2][1][1] = -sa*cb*cg-ca*sg;
  sc[2][1][2] =  0;
  sc[2][2][0] =  sb*sg;
  sc[2][2][1] =  sb*cg;
  sc[2][2][2] =  0;
  */
}
/*
void set_sincos(double a,double b,double g,double sc_d[3][3][3]){
  double sa,ca,sb,cb,sg,cg;

  sa = sin(a);
  ca = cos(a);
  sb = sin(b);
  cb = cos(b);
  sg = sin(g);
  cg = cos(g);

  sc_d[0][0][0] = -sa*cb*cg - ca*sg;
  sc_d[0][0][1] =  ca*sb*sg - sa*cg;
  sc_d[0][0][2] = -sa*sb;

  sc_d[1][0][0] =  ca*sb*cg;
  sc_d[1][0][1] =  ca*sb*sg;
  sc_d[1][0][2] =  ca*cb;

  sc_d[2][0][0] =  -ca*cb*sg-sa*cg;
  sc_d[2][0][1] =  -ca*cb*cg+sa*sg;
  sc_d[2][0][2] =  0;


  sc_d[0][1][0] =  ca*cb*cg - sa*sg;
  sc_d[0][1][1] = -ca*cb*sg - sa*cg;
  sc_d[0][1][2] =  ca*sb;

  sc_d[1][1][0] =  -sa*sb*cg;
  sc_d[1][1][1] =  sa*sb*sg;
  sc_d[1][1][2] =  sa*cb;

  sc_d[2][1][0] =  -sa*cb*sg+ca*cg;
  sc_d[2][1][1] =  -sa*cb*cg-ca*sg;
  sc_d[2][1][2] =  0;

  sc_d[0][2][0] =  0;
  sc_d[0][2][1] =  0;
  sc_d[0][2][2] =  0;

  sc_d[1][2][0] =  -cb*cg;
  sc_d[1][2][1] =  cb*sg;
  sc_d[1][2][2] =  -sb;

  sc_d[2][2][0] =  sb*sg;
  sc_d[2][2][1] =  sb*cg;
  sc_d[2][2][2] =  0;

}

*/
