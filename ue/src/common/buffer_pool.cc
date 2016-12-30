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


#include <pthread.h>
#include "common/buffer_pool.h"
#include <stdio.h>
#include <string>

namespace srslte{

buffer_pool* buffer_pool::instance = NULL;
pthread_mutex_t instance_mutex = PTHREAD_MUTEX_INITIALIZER;

buffer_pool* buffer_pool::get_instance(void)
{
  pthread_mutex_lock(&instance_mutex);
  if(NULL == instance)
    instance = new buffer_pool();
  pthread_mutex_unlock(&instance_mutex);
  return instance;
}

void buffer_pool::cleanup(void)
{
  pthread_mutex_lock(&instance_mutex);
  if(NULL != instance)
  {
    delete instance;
    instance = NULL;
  }
  pthread_mutex_unlock(&instance_mutex);
}

buffer_pool::buffer_pool()
{
  pthread_mutex_init(&mutex, NULL);
  pool = new byte_buffer_t[POOL_SIZE];
  first_available = &pool[0];
  for(int i=0;i<POOL_SIZE-1;i++)
  {
    pool[i].set_next(&pool[i+1]);
  }
  pool[POOL_SIZE-1].set_next(NULL);
  allocated = 0;
}

std::string now_time()
{
  struct timeval rawtime;
  struct tm * timeinfo;
  char buffer[64];
  char us[16];
  
  gettimeofday(&rawtime, NULL);
  timeinfo = localtime(&rawtime.tv_sec);
  
  strftime(buffer,64,"%H:%M:%S",timeinfo);
  strcat(buffer,".");
  snprintf(us,16,"%06ld",rawtime.tv_usec);
  strcat(buffer,us);
  
  return std::string(buffer);
}


byte_buffer_t* buffer_pool::allocate()
{
  pthread_mutex_lock(&mutex);

  if(first_available == NULL)
  {
    printf("Error - buffer pool is empty\n");
    pthread_mutex_unlock(&mutex);
    return NULL;
  }

  // Remove from available list
  byte_buffer_t* b = first_available;
  first_available = b->get_next();
  allocated++;
  
  //printf("%s\tallocated=%d\n", now_time().c_str(), allocated);

  pthread_mutex_unlock(&mutex);
  return b;
}

void buffer_pool::deallocate(byte_buffer_t *b)
{
  pthread_mutex_lock(&mutex);

  // Add to front of available list
  b->reset();
  b->set_next(first_available);
  first_available = b;
  allocated--;
  
  pthread_mutex_unlock(&mutex);
}



} // namespace srsue
