/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: kd-tree functions
 * Author: Andrew Howard
 * Date: 18 Dec 2002
 * CVS: $Id: pf_kdtree.c 7057 2008-10-02 00:44:06Z gbiggs $
 * chq:
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>


#include "amcl/pf/pf_vector.h"
#include "amcl/pf/pf_kdtree.h"


// Compare keys to see if they are equal
static int pf_kdtree_equal(pf_kdtree_t *self, int key_a[], int key_b[]);

// Insert a node into the tree
static pf_kdtree_node_t *pf_kdtree_insert_node(pf_kdtree_t *self, pf_kdtree_node_t *parent,
                                               pf_kdtree_node_t *node, int key[], double value);

// Recursive node search
static pf_kdtree_node_t *pf_kdtree_find_node(pf_kdtree_t *self, pf_kdtree_node_t *node, int key[]);

// Recursively label nodes in this cluster
static void pf_kdtree_cluster_node(pf_kdtree_t *self, pf_kdtree_node_t *node, int depth);

// Recursive node printing
//static void pf_kdtree_print_node(pf_kdtree_t *self, pf_kdtree_node_t *node);


#ifdef INCLUDE_RTKGUI

// Recursively draw nodes
static void pf_kdtree_draw_node(pf_kdtree_t *self, pf_kdtree_node_t *node, rtk_fig_t *fig);

#endif



////////////////////////////////////////////////////////////////////////////////
// Create a tree
pf_kdtree_t *pf_kdtree_alloc(int max_size)
{
  pf_kdtree_t *self;

  self = calloc(1, sizeof(pf_kdtree_t));

  self->size[0] = 0.50;//cluster xy 栅格化尺寸0.5米
  self->size[1] = 0.50;
  self->size[2] = (10 * M_PI / 180);//cluster angle 栅格化尺寸10 deg

  self->root = NULL;

  self->node_count = 0;
  self->node_max_count = max_size;
  self->nodes = calloc(self->node_max_count, sizeof(pf_kdtree_node_t));

  self->leaf_count = 0;

  return self;
}


////////////////////////////////////////////////////////////////////////////////
// Destroy a tree
// 释放树
void pf_kdtree_free(pf_kdtree_t *self)
{
  free(self->nodes);
  free(self);
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Clear all entries from the tree
// 清除所用结点
void pf_kdtree_clear(pf_kdtree_t *self)
{
  self->root = NULL;
  self->leaf_count = 0;
  self->node_count = 0;

  return;
}


////////////////////////////////////////////////////////////////////////////////
// Insert a pose into the tree.
// 插入一个位姿到树中
void pf_kdtree_insert(pf_kdtree_t *self, pf_vector_t pose, double value)
{
  int key[3];

  key[0] = floor(pose.v[0] / self->size[0]);
  key[1] = floor(pose.v[1] / self->size[1]);
  key[2] = floor(pose.v[2] / self->size[2]);

  self->root = pf_kdtree_insert_node(self, NULL, self->root, key, value);

  // Test code
  /*
  printf("find %d %d %d\n", key[0], key[1], key[2]);
  assert(pf_kdtree_find_node(self, self->root, key) != NULL);

  pf_kdtree_print_node(self, self->root);

  printf("\n");

  for (i = 0; i < self->node_count; i++)
  {
    node = self->nodes + i;
    if (node->leaf)
    {
      printf("find %d %d %d\n", node->key[0], node->key[1], node->key[2]);
      assert(pf_kdtree_find_node(self, self->root, node->key) == node);
    }
  }
  printf("\n\n");
  */

  return;
}


////////////////////////////////////////////////////////////////////////////////
// Determine the probability estimate for the given pose. TODO: this
// should do a kernel density estimate rather than a simple histogram.
// 找到给定位姿的概率或权重
// node的value值存储的就是权重
double pf_kdtree_get_prob(pf_kdtree_t *self, pf_vector_t pose)
{
  int key[3];
  pf_kdtree_node_t *node;

  key[0] = floor(pose.v[0] / self->size[0]);
  key[1] = floor(pose.v[1] / self->size[1]);
  key[2] = floor(pose.v[2] / self->size[2]);

  node = pf_kdtree_find_node(self, self->root, key);// 在树中搜索结点
  if (node == NULL)
    return 0.0;
  return node->value;
}


////////////////////////////////////////////////////////////////////////////////
// Determine the cluster label for the given pose
int pf_kdtree_get_cluster(pf_kdtree_t *self, pf_vector_t pose)
{
  int key[3];
  pf_kdtree_node_t *node;
 // key值就是整数化后的坐标值,size代表整数化精度（或误差）
  key[0] = floor(pose.v[0] / self->size[0]);
  key[1] = floor(pose.v[1] / self->size[1]);
  key[2] = floor(pose.v[2] / self->size[2]);

  node = pf_kdtree_find_node(self, self->root, key);
  if (node == NULL)
    return -1;
  return node->cluster;
}


////////////////////////////////////////////////////////////////////////////////
// Compare keys to see if they are equal
// 判断key值是否相等
int pf_kdtree_equal(pf_kdtree_t *self, int key_a[], int key_b[])
{
  //double a, b;

  if (key_a[0] != key_b[0])
    return 0;
  if (key_a[1] != key_b[1])
    return 0;

  if (key_a[2] != key_b[2])
    return 0;

  /* TODO: make this work (pivot selection needs fixing, too)
  // Normalize angles
  a = key_a[2] * self->size[2];
  a = atan2(sin(a), cos(a)) / self->size[2];
  b = key_b[2] * self->size[2];
  b = atan2(sin(b), cos(b)) / self->size[2];

 if ((int) a != (int) b)
    return 0;
  */

  return 1;
}


////////////////////////////////////////////////////////////////////////////////
// Insert a node into the tree
// 插入一个结点到树中
// 插入方法是看传入节点的类型：
// 1,为空，插入作为新叶子
// 2,为叶子，与新传入的key做比较，旧的,更新node的value,新的，作为node的新叶子
// 3,为中间节点，在node分割维度上，node　key值与新传入的key做比较，小于更新为左子树value,否则为右value
pf_kdtree_node_t *pf_kdtree_insert_node(pf_kdtree_t *self, pf_kdtree_node_t *parent,
                                        pf_kdtree_node_t *node, int key[], double value)
{
  int i;
  int split, max_split;

  // If the node doesnt exist yet...
  // <1>如果要插入的节点为空，则将该节点作为kdtree的新的一个叶子节点。
  // 若传入的父也为空，则新插入节点深度为０，否则深度为父深度加１
  // 节点的值为传入的value，kdtree的叶子数加１
  if (node == NULL)
  {
    assert(self->node_count < self->node_max_count);
    node = self->nodes + self->node_count++;
    memset(node, 0, sizeof(pf_kdtree_node_t));

    node->leaf = 1;

    if (parent == NULL)
      node->depth = 0;
    else
      node->depth = parent->depth + 1;

    for (i = 0; i < 3; i++)
      node->key[i] = key[i];

    node->value = value;
    self->leaf_count += 1;
  }

  // <2>If the node exists, and it is a leaf node...
  // 如果给定的节点是叶子节点，判定其key值与传入的新key值是否相等:
  //  1,如果相等，对叶子节点进行更新：
  //    该叶子节点只对value值作累计处理
  //  2,如果不相等，插入子节点到该叶节点：
  //    对比传入的node的key值与传入的key值，找到最大分割维度，并在该维度上求取分割中值
  //    如果在该维度上，传入的key值小于分割中值，新传入的key作为一个node的左节点
  else if (node->leaf)
  {
    // If the keys are equal, increment the value
    if (pf_kdtree_equal(self, key, node->key))
    {
      node->value += value;///如果是相同节点，则value（权重）累加
    }

    // The keys are not equal, so split this node
    // 否则 新建子节点
    else
    {
      // Find the dimension with the largest variance and do a mean
      // split
      max_split = 0;
      node->pivot_dim = -1;//pivot_dim 分割维度　 pivot_value: 分割value
      for (i = 0; i < 3; i++)
      {
        split = abs(key[i] - node->key[i]);
        if (split > max_split)
        {
          max_split = split;
          node->pivot_dim = i;// 找到差值最大的那一维
        }
      }
      assert(node->pivot_dim >= 0);
      //新的分割值等于原分割值与新的插入数据点（在该维度上）的均值　－－即新的中值
      node->pivot_value = (key[node->pivot_dim] + node->key[node->pivot_dim]) / 2.0;
      //
      if (key[node->pivot_dim] < node->pivot_value)
      {
        node->children[0] = pf_kdtree_insert_node(self, node, NULL, key, value);
        node->children[1] = pf_kdtree_insert_node(self, node, NULL, node->key, node->value);
      }
      else
      {
        node->children[0] = pf_kdtree_insert_node(self, node, NULL, node->key, node->value);
        node->children[1] = pf_kdtree_insert_node(self, node, NULL, key, value);
      }

      node->leaf = 0;
      self->leaf_count -= 1;
    }
  }

  // <3>If the node exists, and it has children...
  // 如果节点是中间节点　即　有左右子
  //将node对应的的分割值，与新传入的key在分割维度上的值比较
  // 如果小于，更新node左子树的节点值为新传入的key值
  // 否则更新右子树
  else
  {
    assert(node->children[0] != NULL);
    assert(node->children[1] != NULL);

    if (key[node->pivot_dim] < node->pivot_value)
      pf_kdtree_insert_node(self, node, node->children[0], key, value);
    else
      pf_kdtree_insert_node(self, node, node->children[1], key, value);
  }

  return node;
}


////////////////////////////////////////////////////////////////////////////////
// Recursive node search
// 树中查找某结点
pf_kdtree_node_t *pf_kdtree_find_node(pf_kdtree_t *self, pf_kdtree_node_t *node, int key[])
{
  if (node->leaf)
  {
    //printf("find  : leaf %p %d %d %d\n", node, node->key[0], node->key[1], node->key[2]);

    // If the keys are the same...
    // 如果节点已经是叶子节点在则将节点key值与输入的key值做比较，不同则返回空，否则返回当前node
    if (pf_kdtree_equal(self, key, node->key))
      return node;
    else
      return NULL;
  }
  else
  {
    //printf("find  : brch %p %d %f\n", node, node->pivot_dim, node->pivot_value);

    assert(node->children[0] != NULL);
    assert(node->children[1] != NULL);

    // If the keys are different...
    //真实的节点数据都是在叶子上，所以如果当前节点是中间节点，则递归按照比较方式向下搜寻，直到找到最接近的叶子节点
    if (key[node->pivot_dim] < node->pivot_value) //
      return pf_kdtree_find_node(self, node->children[0], key);
    else
      return pf_kdtree_find_node(self, node->children[1], key);
  }

  return NULL;
}


////////////////////////////////////////////////////////////////////////////////
// Recursive node printing
/*
void pf_kdtree_print_node(pf_kdtree_t *self, pf_kdtree_node_t *node)
{
  if (node->leaf)
  {
    printf("(%+02d %+02d %+02d)\n", node->key[0], node->key[1], node->key[2]);
    printf("%*s", node->depth * 11, "");
  }
  else
  {
    printf("(%+02d %+02d %+02d) ", node->key[0], node->key[1], node->key[2]);
    pf_kdtree_print_node(self, node->children[0]);
    pf_kdtree_print_node(self, node->children[1]);
  }
  return;
}
*/


////////////////////////////////////////////////////////////////////////////////
// Cluster the leaves in the tree
void pf_kdtree_cluster(pf_kdtree_t *self)
{
  int i;
  int queue_count, cluster_count;
  pf_kdtree_node_t **queue, *node;

  queue_count = 0;
  queue = calloc(self->node_count, sizeof(queue[0]));

  // Put all the leaves in a queue
  for (i = 0; i < self->node_count; i++)
  {
    node = self->nodes + i;
    if (node->leaf)
    {
      node->cluster = -1;
      assert(queue_count < self->node_count);
      queue[queue_count++] = node;

      // TESTING; remove
      // 确保node　key值有效性（一一对应）
      assert(node == pf_kdtree_find_node(self, self->root, node->key));
    }
  }

  cluster_count = 0;

  // Do connected components for each node
  while (queue_count > 0)
  {
    node = queue[--queue_count];

    // If this node has already been labelled, skip it
    if (node->cluster >= 0)
      continue;

    // Assign a label to this cluster
    node->cluster = cluster_count++;

    // Recursively label nodes in this cluster
    pf_kdtree_cluster_node(self, node, 0);
  }

  free(queue);
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Recursively label nodes in this cluster
void pf_kdtree_cluster_node(pf_kdtree_t *self, pf_kdtree_node_t *node, int depth)
{
  //three dimension included:x y angle:
  //chq
  //The first two dimensions are assumed to be nine palace lattices, including middle points and other eight surrounding points.
  //The third dimension, angle direction, left and right sides of each one, plus itself, a total of three.
  //So In all three dimensions, a total of 3 x 9 equals 27 data as a cluster.
  // example:if size_x = 0.5, 0.5*3 =1.5 is a same cluster
  int i;
  int nkey[3];
  pf_kdtree_node_t *nnode;
  int num = 3 * 3 * 3;
  int a[num],b[num],c[num];
  for (i = 0; i < 3 * 3 * 3; i++)
  {
    a[i] = (i / 9) - 1;
    b[i] = ((i % 9) / 3) - 1;
    c[i] = ((i % 9) % 3) - 1;

    nkey[0] = node->key[0] + (i / 9) - 1;
    nkey[1] = node->key[1] + ((i % 9) / 3) - 1;
    nkey[2] = node->key[2] + ((i % 9) % 3) - 1;

    nnode = pf_kdtree_find_node(self, self->root, nkey);
    if (nnode == NULL)
      continue;

    assert(nnode->leaf);

    // This node already has a label; skip it.  The label should be
    // consistent, however.
    if (nnode->cluster >= 0)
    {
      assert(nnode->cluster == node->cluster);
      continue;
    }

    // Label this node and recurse
    nnode->cluster = node->cluster;

    pf_kdtree_cluster_node(self, nnode, depth + 1);
  }
  return;
}



#ifdef INCLUDE_RTKGUI

////////////////////////////////////////////////////////////////////////////////
// Draw the tree
void pf_kdtree_draw(pf_kdtree_t *self, rtk_fig_t *fig)
{
  if (self->root != NULL)
    pf_kdtree_draw_node(self, self->root, fig);
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Recursively draw nodes
void pf_kdtree_draw_node(pf_kdtree_t *self, pf_kdtree_node_t *node, rtk_fig_t *fig)
{
  double ox, oy;
  char text[64];

  if (node->leaf)
  {
    ox = (node->key[0] + 0.5) * self->size[0];
    oy = (node->key[1] + 0.5) * self->size[1];

    rtk_fig_rectangle(fig, ox, oy, 0.0, self->size[0], self->size[1], 0);

    //snprintf(text, sizeof(text), "%0.3f", node->value);
    //rtk_fig_text(fig, ox, oy, 0.0, text);

    snprintf(text, sizeof(text), "%d", node->cluster);
    rtk_fig_text(fig, ox, oy, 0.0, text);
  }
  else
  {
    assert(node->children[0] != NULL);
    assert(node->children[1] != NULL);
    pf_kdtree_draw_node(self, node->children[0], fig);
    pf_kdtree_draw_node(self, node->children[1], fig);
  }

  return;
}

#endif
