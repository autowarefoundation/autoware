#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "algebra.h"
#include "ndt.h"


NDMapPtr NDmap;
NDPtr NDs;
int NDs_num;

int g_map_x, g_map_y, g_map_z;
double g_map_cellsize;

/*add point to ndcell */
int add_point_covariance(NDPtr nd, PointPtr p)
{
  /*add data num*/
  nd->num++;
  nd->flag = 0; /*need to update*/

  /*calcurate means*/
  nd->m_x += p->x;
  nd->m_y += p->y;
  nd->m_z += p->z;

  /*calcurate covariances*/
  nd->c_xx += p->x * p->x;
  nd->c_yy += p->y * p->y;
  nd->c_zz += p->z * p->z;

  nd->c_xy += p->x * p->y;
  nd->c_yz += p->y * p->z;
  nd->c_zx += p->z * p->x;

  return 1;
}

int inv_check(double inv[3][3])
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (isnan(inv[i][j]))
        return 0;
      if (fabs(inv[i][j]) > 1000)
        return 0;
    }
  }
  return 1;
}

/*calcurate covariances*/
int update_covariance(NDPtr nd)
{
  double a, b, c; /*for calcurate*/
  if (!nd->flag)
  { /*need calcurate?*/
    /*means*/
    nd->mean.x = a = nd->m_x / nd->num;
    nd->mean.y = b = nd->m_y / nd->num;
    nd->mean.z = c = nd->m_z / nd->num;

    /*covariances*/
    nd->covariance[0][0] = (nd->c_xx - 2 * a * nd->m_x) / nd->num + a * a;
    nd->covariance[1][1] = (nd->c_yy - 2 * b * nd->m_y) / nd->num + b * b;
    nd->covariance[2][2] = (nd->c_zz - 2 * c * nd->m_z) / nd->num + c * c;
    nd->covariance[0][1] = nd->covariance[1][0] = (nd->c_xy - nd->m_x * b - nd->m_y * a) / nd->num + a * b;
    nd->covariance[1][2] = nd->covariance[2][1] = (nd->c_yz - nd->m_y * c - nd->m_z * b) / nd->num + b * c;
    nd->covariance[2][0] = nd->covariance[0][2] = (nd->c_zx - nd->m_z * a - nd->m_x * c) / nd->num + c * a;
    nd->sign = 0;
    nd->flag = 1; /*this ND updated*/
    if (nd->num >= 5)
    {
      if (ginverse_matrix3d(nd->covariance, nd->inv_covariance))
        if (inv_check(nd->inv_covariance))
          nd->sign = 1;
    }
  }

  return 1;
}

/*add point to ndmap*/
int add_point_map(NDMapPtr ndmap, PointPtr point)
{
  double x, y, z;
  NDPtr *ndp[8];

  /*mapping*/
  x = (point->x / ndmap->size) + ndmap->x / 2;
  y = (point->y / ndmap->size) + ndmap->y / 2;
  z = (point->z / ndmap->size) + ndmap->z / 2;

  /*clipping*/
  if (x < 1 || x >= ndmap->x)
    return 0;
  if (y < 1 || y >= ndmap->y)
    return 0;
  if (z < 1 || z >= ndmap->z)
    return 0;

  /*select root ND*/
  ndp[0] = ndmap->nd + (int)x * ndmap->to_x + (int)y * ndmap->to_y + (int)z;
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1;
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;

  /*add  point to map */
  for (int i = 0; i < 8; i++)
  {
    if ((*ndp[i]) == 0)
      *ndp[i] = add_ND();
    if ((*ndp[i]) != 0)
      add_point_covariance(*ndp[i], point);
  }

  if (ndmap->next)
  {
    add_point_map(ndmap->next, point);
  }

  return 0;
}

/*get nd cell at point*/
int get_ND(NDMapPtr ndmap, PointPtr point, NDPtr *nd, int ndmode)
{
  double x, y, z;
  int i;
  NDPtr *ndp[8];

  /*mapping*/
  if (ndmode < 3)
  {
    x = (point->x / ndmap->size) + ndmap->x / 2 - 0.5;
    y = (point->y / ndmap->size) + ndmap->y / 2 - 0.5;
    z = (point->z / ndmap->size) + ndmap->z / 2 - 0.5;
  }
  else
  {
    x = (point->x / ndmap->size) + ndmap->x / 2;
    y = (point->y / ndmap->size) + ndmap->y / 2;
    z = (point->z / ndmap->size) + ndmap->z / 2;
  }

  /*clipping*/
  if (x < 1 || x >= ndmap->x)
    return 0;
  if (y < 1 || y >= ndmap->y)
    return 0;
  if (z < 1 || z >= ndmap->z)
    return 0;

  /*select root ND*/
  ndp[0] = ndmap->nd + (int)x * ndmap->to_x + (int)y * ndmap->to_y + (int)z;
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1;
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;

  for (i = 0; i < 8; i++)
  {
    if (*ndp[i] != 0)
    {
      if (!(*ndp[i])->flag)
        update_covariance(*ndp[i]);
      nd[i] = *ndp[i];
    }
    else
    {
      nd[i] = NDs;
      // return 0;
    }
  }

  return 1;
}

NDPtr add_ND(void)
{
  NDPtr ndp;
  // int m;

  if (NDs_num >= MAX_ND_NUM)
  {
    printf("over flow\n");
    return 0;
  }

  ndp = NDs + NDs_num;
  NDs_num++;

  ndp->flag = 0;
  ndp->sign = 0;
  ndp->num = 0;
  ndp->m_x = 0;
  ndp->m_y = 0;
  ndp->m_z = 0;
  ndp->c_xx = 0;
  ndp->c_yy = 0;
  ndp->c_zz = 0;
  ndp->c_xy = 0;
  ndp->c_yz = 0;
  ndp->c_zx = 0;
  ndp->w = 1;
  ndp->is_source = 0;

  return ndp;
}

NDMapPtr initialize_NDmap_layer(int layer, NDMapPtr child)
{
  // int i,j,k,i2,i3,m;
  int i, j, k;
  int x, y, z;
  NDPtr *nd, *ndp;
  NDMapPtr ndmap;

  x = (g_map_x >> layer) + 1;
  y = (g_map_y >> layer) + 1;
  z = (g_map_z >> layer) + 1;

  nd = (NDPtr *)malloc(x * y * z * sizeof(NDPtr));
  ndmap = (NDMapPtr)malloc(sizeof(NDMap));

  ndmap->x = x;
  ndmap->y = y;
  ndmap->z = z;
  ndmap->to_x = y * z;
  ndmap->to_y = z;
  ndmap->layer = layer;
  ndmap->nd = nd;
  ndmap->next = child;
  ndmap->size = g_map_cellsize * ((int)1 << layer);

  ndp = nd;

  for (i = 0; i < x; i++)
  {
    for (j = 0; j < y; j++)
    {
      for (k = 0; k < z; k++)
      {
        *ndp = 0;
        ndp++;
      }
    }
  }

  return ndmap;
}

NDMapPtr initialize_NDmap(void)
{
  int i;
  NDMapPtr ndmap;
  NDPtr null_nd;

  printf("Initialize NDmap\n");
  ndmap = 0;

  // init NDs
  NDs = (NDPtr)malloc(sizeof(NormalDistribution) * MAX_ND_NUM);
  NDs_num = 0;

  null_nd = add_ND();
  if (null_nd == 0)
  {
    return 0;
  }

  for (i = LAYER_NUM - 1; i >= 0; i--)
  {
    ndmap = initialize_NDmap_layer(i, ndmap);
  }

  return ndmap;
}

int round_covariance(NDPtr nd)
{
  double v[3][3], a;

  eigenvecter_matrix3d(nd->covariance, v, nd->l);
  //  print_matrix3d(v);
  if (fabs(v[0][0] * v[0][0] + v[1][0] * v[1][0] + v[2][0] * v[2][0] - 1) > 0.1)
    printf("!1");
  if (fabs(v[0][0] * v[0][1] + v[1][0] * v[1][1] + v[2][0] * v[2][1]) > 0.01)
    printf("!01");
  if (fabs(v[0][1] * v[0][2] + v[1][1] * v[1][2] + v[2][1] * v[2][2]) > 0.01)
    printf("!02");
  if (fabs(v[0][2] * v[0][0] + v[1][2] * v[1][0] + v[2][2] * v[2][0]) > 0.01)
    printf("!03");

  a = fabs(nd->l[1] / nd->l[0]);
  if (a < 0.001)
  {
    return 0;
  }
  return 1;
}

double probability_on_ND(NDPtr nd, double xp, double yp, double zp)
{
  //  double xp,yp,zp;
  double e;

  if (nd->num < 5)
    return 0;

  e = exp((xp * xp * nd->inv_covariance[0][0] + yp * yp * nd->inv_covariance[1][1] +
           zp * zp * nd->inv_covariance[2][2] + 2.0 * xp * yp * nd->inv_covariance[0][1] +
           2.0 * yp * zp * nd->inv_covariance[1][2] + 2.0 * zp * xp * nd->inv_covariance[2][0]) /
          -2.0);

  if (e > 1)
    return 1;
  if (e < 0)
    return 0;
  return (e);
}
