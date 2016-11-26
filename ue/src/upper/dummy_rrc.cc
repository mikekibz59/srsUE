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


#include "upper/dummy_rrc.h"

using namespace srslte;

namespace srsue{

dummy_rrc::dummy_rrc()
{}

bool dummy_rrc::init(dummy_rrc_args_t      *args_,
                     nas_interface_rrc     *nas_,
                     srslte::log           *rrc_log_)
{ 
  pool    = buffer_pool::get_instance();
  args    = *args_;
  nas     = nas_;
  rrc_log = rrc_log_;

  if(0 != srslte_netsource_init(&src, "127.0.0.1", args.srcport, SRSLTE_NETSOURCE_UDP)) {
    rrc_log->error("Failed to create source socket on 127.0.0.1:%d", args.srcport);
    return false;
  }
  if(0 != srslte_netsink_init(&snk, "127.0.0.1", args.snkport, SRSLTE_NETSINK_UDP)) {
    rrc_log->error("Failed to create sink socket on 127.0.0.1:%d", args.snkport);
    return false;
  }

  // Setup a thread to receive packets from the src socket
  start(THREAD_PRIO);

  return true;
}

void dummy_rrc::stop()
{
  if(running) {
    running = false;
    thread_cancel();
    wait_thread_finish();
  }

  srslte_netsink_free(&snk);
  srslte_netsource_free(&src);
}

void dummy_rrc::trigger()
{
  nas->notify_connection_setup();
}


/*******************************************************************************
  NAS interface
*******************************************************************************/

void dummy_rrc::write_sdu(uint32_t lcid, byte_buffer_t *sdu)
{
  rrc_log->info_hex(sdu->msg, sdu->N_bytes, "RX %s SDU", rb_id_text[lcid]);
  srslte_netsink_write(&snk, sdu->msg, sdu->N_bytes);
}

uint16_t dummy_rrc::get_mcc()
{
  return 1;
}

uint16_t dummy_rrc::get_mnc()
{
  return 1;
}

void dummy_rrc::enable_capabilities()
{
  //printf("Not enabling 64QAM\n");
  //phy->set_config_64qam_en(true);
}

/*******************************************************************************
  Socket interface
*******************************************************************************/

void dummy_rrc::run_thread()
{
  uint32_t lcid = 1;
  byte_buffer_t *pdu = pool->allocate();
  running = true;

  while(running) {
    pdu->N_bytes = srslte_netsource_read(&src, pdu->msg, SRSUE_MAX_BUFFER_SIZE_BYTES - SRSUE_BUFFER_HEADER_OFFSET);
    rrc_log->info_hex(pdu->msg, pdu->N_bytes, "RX %s PDU", rb_id_text[lcid]);
    nas->write_pdu(lcid, pdu);
    pdu = pool->allocate();
  }
}

} // namespace srsue
