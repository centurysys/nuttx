/****************************************************************************
 * fs/driver/fs_registerblockdriver.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/partition.h>

#include "inode/inode.h"

#ifndef CONFIG_DISABLE_MOUNTPOINT

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: partition_handler
 ****************************************************************************/

#if defined(CONFIG_MBR_PARTITION) || defined(CONFIG_GPT_PARTITION)
static void partition_handler(struct partition_s *part, void *arg)
{
  const char *parent = (const char *) arg;
  char path[NAME_MAX + 1];

  sprintf(path, "%sp%d", parent, part->index + 1);
  register_blockpartition(path, 0666, parent, part->firstblock, part->nblocks);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: register_blockdriver
 *
 * Description:
 *   Register a block driver inode the pseudo file system.
 *
 * Input Parameters:
 *   path - The path to the inode to create
 *   bops - The block driver operations structure
 *   mode - inmode privileges
 *   priv - Private, user data that will be associated with the inode.
 *
 * Returned Value:
 *   Zero on success (with the inode point in 'inode'); A negated errno
 *   value is returned on a failure (all error values returned by
 *   inode_reserve):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int register_blockdriver(FAR const char *path,
                         FAR const struct block_operations *bops,
                         mode_t mode, FAR void *priv)
{
  FAR struct inode *node;
  int ret;

  /* Insert an inode for the device driver -- we need to hold the inode
   * semaphore to prevent access to the tree while we this.  This is because
   * we will have a momentarily bad true until we populate the inode with
   * valid data.
   */

  ret = inode_lock();
  if (ret < 0)
    {
      _err("! inode_lock() failed, ret = %d\n", ret);
      return ret;
    }

  ret = inode_reserve(path, mode, &node);
  if (ret >= 0)
    {
      /* We have it, now populate it with block driver specific information.
       * NOTE that the initial reference count on the new inode is zero.
       */

      INODE_SET_BLOCK(node);

      node->u.i_bops  = bops;
      node->i_private = priv;
      ret             = OK;

      _info("%s registered.\n", path);
    }

  inode_unlock();
#if defined(CONFIG_MBR_PARTITION) || defined(CONFIG_GPT_PARTITION)
  if (ret >= 0)
    {
      parse_block_partition(path, partition_handler, (void *) path);
    }
#endif
  return ret;
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT */
