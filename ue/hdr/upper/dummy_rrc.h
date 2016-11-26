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

#ifndef DUMMY_RRC_H
#define DUMMY_RRC_H

#include "common/buffer_pool.h"
#include "common/log.h"
#include "common/common.h"
#include "common/interfaces.h"
#include "common/threads.h"
#include "srslte/io/netsink.h"
#include "srslte/io/netsource.h"

using srslte::byte_buffer_t;

namespace srsue {

typedef struct{
  uint16_t snkport;
  uint16_t srcport;
}dummy_rrc_args_t;


class dummy_gw
    :public gw_interface_nas
{
public:
  dummy_gw()
  {}
  srslte::error_t setup_if_addr(uint32_t ip_addr, char *err_str)
  {
    return srslte::ERROR_NONE;
  }
};


class dummy_rrc
    :public rrc_interface_nas
    ,public thread
{
public:
  dummy_rrc();
  bool init(dummy_rrc_args_t      *args_,
            nas_interface_rrc     *nas_,
            srslte::log           *rrc_log_);
  void stop();
  void trigger();
  
private:
  static const int THREAD_PRIO = 7;

  srslte::buffer_pool  *pool;
  dummy_rrc_args_t      args;
  srslte::log          *rrc_log;
  nas_interface_rrc    *nas;

  srslte_netsink_t      snk;
  srslte_netsource_t    src;

  bool running;

  // NAS interface
  void write_sdu(uint32_t lcid, byte_buffer_t *sdu);
  uint16_t get_mcc();
  uint16_t get_mnc();
  void enable_capabilities();

  // Threading
  void run_thread();
  
};

} // namespace srsue


#endif // DUMMY_RRC_H
