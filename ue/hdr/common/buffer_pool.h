/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of the srsUE library.
 *
 * srsUE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsUE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#ifndef BUFFER_POOL_H
#define BUFFER_POOL_H

#include <pthread.h>

/*******************************************************************************
                              INCLUDES
*******************************************************************************/

#include "common/common.h"

namespace srslte {

/******************************************************************************
 * Buffer pool
 *
 * Preallocates a large number of srsue_byte_buffer_t and provides allocate and
 * deallocate functions. Provides quick object creation and deletion as well
 * as object reuse. Uses a linked list to keep track of available buffers.
 * Singleton class of byte_buffer_t (but other pools of different type can be created)
 *****************************************************************************/

template <class buffer_t>
class buffer_pool{
public:
  
  // non-static methods
  buffer_pool()
  {
    pthread_mutex_init(&mutex, NULL);
    pool = new buffer_t[POOL_SIZE];
    first_available = &pool[0];
    for(int i=0;i<POOL_SIZE-1;i++)
    {
      pool[i].set_next(&pool[i+1]);
    }
    pool[POOL_SIZE-1].set_next(NULL);
    allocated = 0;
  }

  buffer_pool(uint32_t buffer_size)
  {
    buffer_pool();
    for(int i=0;i<POOL_SIZE-1;i++)
    {
      pool[i].set_size(buffer_size);
    }
  }
  ~buffer_pool() { 
    delete [] pool; 
    
  }
  
  
  buffer_t* allocate()
  {
    pthread_mutex_lock(&mutex);

    if(first_available == NULL)
    {
      printf("Error - buffer pool is empty\n");
      pthread_mutex_unlock(&mutex);
      return NULL;
    }

    // Remove from available list
    buffer_t* b = first_available;
    first_available = b->get_next();
    allocated++;
    
    //printf("%s\tallocated=%d\n", now_time().c_str(), allocated);

    pthread_mutex_unlock(&mutex);
    return b;
  }
  
  void deallocate(buffer_t *b)
  {
    pthread_mutex_lock(&mutex);

    // Add to front of available list
    b->reset();
    b->set_next(first_available);
    first_available = b;
    allocated--;
    
    pthread_mutex_unlock(&mutex);
  }

  
private:  
  static const int      POOL_SIZE = 2048;
  buffer_t              *pool;
  buffer_t              *first_available;
  pthread_mutex_t       mutex;  
  int                   allocated;
};


class byte_buffer_pool {
public: 
  // Singleton static methods
  static byte_buffer_pool   *instance;  
  static byte_buffer_pool*   get_instance(void);
  static void                cleanup(void); 
  byte_buffer_pool() {
    pool = new buffer_pool<byte_buffer_t>;
  }
  ~byte_buffer_pool() {
    delete pool; 
  }
  byte_buffer_t* allocate() {
    return pool->allocate();
  }
  void deallocate(byte_buffer_t *b) {
    pool->deallocate(b);
  }
private:
  buffer_pool<byte_buffer_t> *pool; 
};


} // namespace srsue

#endif // BUFFER_POOL_H
