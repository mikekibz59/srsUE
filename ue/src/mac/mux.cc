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

#define Error(fmt, ...)   log_h->error_line(__FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define Warning(fmt, ...) log_h->warning_line(__FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define Info(fmt, ...)    log_h->info_line(__FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define Debug(fmt, ...)   log_h->debug_line(__FILE__, __LINE__, fmt, ##__VA_ARGS__)

#include "mac/mux.h"
#include "mac/mac.h"

#include <algorithm>

namespace srsue {

mux::mux() : pdu_msg(MAX_NOF_SUBHEADERS)
{
  msg3_buff.init(1, MSG3_BUFF_SZ);

  pthread_mutex_init(&mutex, NULL);
  msg3_has_been_transmitted = false; 
  
  pending_crnti_ce = 0;
}

void mux::init(rlc_interface_mac *rlc_, srslte::log *log_h_, bsr_proc *bsr_procedure_, phr_proc *phr_procedure_)
{
  log_h      = log_h_;
  rlc        = rlc_;
  bsr_procedure = bsr_procedure_;
  phr_procedure = phr_procedure_;
  reset();
}

void mux::reset()
{
  lch.clear();
  pending_crnti_ce = 0;
}

bool mux::is_pending_any_sdu()
{
  for (int i=0;i<lch.size();i++) {
    if (rlc->get_buffer_state(lch[i].id)) {
      return true; 
    }
  }
  return false; 
}

bool mux::is_pending_sdu(uint32_t lch_id) {
  return rlc->get_buffer_state(lch_id)>0;  
}

int mux::find_lchid(uint32_t lcid) 
{
  for (int i=0;i<lch.size();i++) {
    if(lch[i].id == lcid) {
      return i;
    }
  }
  return -1; 
}

bool sortPriority(lchid_t u1, lchid_t u2) {
  return u1.priority < u2.priority; 
}

void mux::clear_lch(uint32_t lch_id)
{
  int pos = find_lchid(lch_id);
  if (pos >= 0) {
    lch.erase(lch.begin()+pos);
  } else {
    Error("Deleting logical channel id %d. Does not exist\n", lch_id);
  }
}

void mux::set_priority(uint32_t lch_id, uint32_t new_priority, int set_PBR, uint32_t set_BSD)
{
  int pos = find_lchid(lch_id);
    
  // Create new channel if it does not exist
  if (pos < 0) {
    lchid_t ch; 
    ch.id       = lch_id; 
    ch.priority = new_priority; 
    ch.BSD      = set_BSD; 
    ch.PBR      = set_PBR; 
    ch.Bj       = 0; 
    lch.push_back(ch);
  } else {
    lch[pos].priority = new_priority; 
    lch[pos].PBR      = set_PBR; 
    lch[pos].BSD      = set_BSD;     
  }
  
  // sort according to priority (increasing is lower priority)
  std::sort(lch.begin(), lch.end(), sortPriority); 
}

srslte::sch_subh::cetype bsr_format_convert(bsr_proc::bsr_format_t format) {
  switch(format) {
    case bsr_proc::LONG_BSR: 
      return srslte::sch_subh::LONG_BSR;
    case bsr_proc::SHORT_BSR: 
      return srslte::sch_subh::SHORT_BSR;
    case bsr_proc::TRUNC_BSR: 
      return srslte::sch_subh::TRUNC_BSR;   
  }
}

void mux::pusch_retx(uint32_t tx_tti, uint32_t pid)
{
  if (pid_has_bsr[pid%MAX_HARQ_PROC]) {
    bsr_procedure->set_tx_tti(tx_tti);
  }
}

// Multiplexing and logical channel priorization as defined in Section 5.4.3
uint8_t* mux::pdu_get(uint8_t *payload, uint32_t pdu_sz, uint32_t tx_tti, uint32_t pid)
{
  
  pthread_mutex_lock(&mutex);
    
  // Update Bj
  for (int i=0;i<lch.size();i++) {    
    // Add PRB unless it's infinity 
    if (lch[i].PBR >= 0) {
      lch[i].Bj += lch[i].PBR;
    }
    if (lch[i].Bj >= lch[i].BSD) {
      lch[i].Bj = lch[i].BSD; 
    }    
  }
  
// Logical Channel Procedure

  pdu_msg.init_tx(payload, pdu_sz, true);

  // MAC control element for C-RNTI or data from UL-CCCH
  if (!allocate_sdu(0, &pdu_msg, -1, NULL)) {
    if (pending_crnti_ce) {
      if (pdu_msg.new_subh()) {
        if (!pdu_msg.get()->set_c_rnti(pending_crnti_ce)) {
          Warning("Pending C-RNTI CE could not be inserted in MAC PDU\n");
        }
      }
    }
  }
  pending_crnti_ce = 0; 
  
  bsr_proc::bsr_t bsr; 
  bool regular_bsr = bsr_procedure->need_to_send_bsr_on_ul_grant(pdu_msg.rem_size(), &bsr);
  bool bsr_is_inserted = false; 
  
  // MAC control element for BSR, with exception of BSR included for padding;
  if (regular_bsr) {
    if (pdu_msg.new_subh()) {
      pdu_msg.get()->set_bsr(bsr.buff_size, bsr_format_convert(bsr.format));    
      bsr_is_inserted  = true; 
    }
  }
  // MAC control element for PHR
  float phr_value; 
  if (phr_procedure->generate_phr_on_ul_grant(&phr_value)) {
    if (pdu_msg.new_subh()) {
      pdu_msg.get()->set_phr(phr_value);
    }
  }
  
  sched_lch.clear();
  int sdu_space = pdu_msg.get_sdu_space(); 
  
  // data from any Logical Channel, except data from UL-CCCH;  
  // first only those with positive Bj
  uint32_t sdu_sz   = 0; 
  for (int i=0;i<lch.size();i++) {
    if (lch[i].id != 0) {
      bool res = true; 
      while ((lch[i].Bj > 0 || lch[i].PBR < 0) && res) {
        res = sched_sdu(&lch[i], &sdu_space, (lch[i].PBR<0)?-1:lch[i].Bj, &sdu_sz);
        if (res && lch[i].PBR >= 0) {
          lch[i].Bj -= sdu_sz;         
        }
      }
    }
  }

  // If resources remain, allocate regardless of their Bj value
  for (int i=0;i<lch.size();i++) {
    while (sched_sdu(&lch[i], &sdu_space, -1, NULL));   
  }
  
  // Merge allocated SDUs to prevent fragmentation 
  merge_sdus();
  
  // Now allocate the SDUs from the RLC 
  for (int i=0;i<sched_lch.size();i++) {
    allocate_sdu(sched_lch[i]->id, &pdu_msg, sched_lch[i]->sched_len, NULL);    
  }

  if (!regular_bsr) {
    // Insert Padding BSR if not inserted Regular/Periodic BSR 
    if (bsr_procedure->generate_padding_bsr(pdu_msg.rem_size(), &bsr)) {
      if (pdu_msg.new_subh()) {
        pdu_msg.get()->set_bsr(bsr.buff_size, bsr_format_convert(bsr.format));
        bsr_is_inserted  = true; 
      }    
    }
  }
  
  Debug("Assembled MAC PDU msg size %d/%d bytes\n", pdu_msg.get_pdu_len()-pdu_msg.rem_size(), pdu_sz);

  /* Generate MAC PDU and save to buffer */
  uint8_t *ret = pdu_msg.write_packet(log_h);   

  pid_has_bsr[pid%MAX_HARQ_PROC] = bsr_is_inserted; 
  if (bsr_is_inserted) {
    bsr_procedure->set_tx_tti(tx_tti);
  }
  
  pthread_mutex_unlock(&mutex);
  

  return ret; 
}

void mux::append_crnti_ce_next_tx(uint16_t crnti) {
  pending_crnti_ce = crnti; 
}

void mux::merge_sdus()
{
  // If a single SDU is scheduled, maximize the grant utilization
  if (sched_lch.size() == 1) {
    sched_lch[0]->sched_len = -1; 
  } else {
    for(std::vector<lchid_t*>::iterator iter=sched_lch.begin(); iter!=sched_lch.end(); ++iter) {
      lchid_t* ch = *iter; 
      int j=0;
      while(j<sched_lch.size() && sched_lch[j]->id != ch->id) {
        j++;
      }
      if (j<sched_lch.size()) {
        ch->sched_len += sched_lch[j]->sched_len; 
        sched_lch.erase(sched_lch.begin()+j);
      }
    }
  }
}

bool mux::sched_sdu(lchid_t *ch, int *sdu_space, int max_sdu_sz, uint32_t* sdu_sz) 
{
 
  // Get n-th pending SDU pointer and length
  int sdu_len = rlc->get_buffer_state(ch->id); 
  
  if (sdu_len > 0) { // there is pending SDU to allocate
    int buffer_state = sdu_len; 
    if (sdu_len > max_sdu_sz && max_sdu_sz >= 0) {
      sdu_len = max_sdu_sz;
    }
    if (sdu_len > *sdu_space) {
      sdu_len = *sdu_space;
    }        
    
    *sdu_space -= sdu_len; 
    *sdu_sz = sdu_len; 

    ch->sched_len = sdu_len; 
    sched_lch.push_back(ch);
    Info("SDU: scheduled lcid=%d, rlc_buffer=%d, allocated=%d/%d\n", 
            ch->id, buffer_state, sdu_len, sdu_space);
  }
  return false; 
}

bool mux::allocate_sdu(uint32_t lcid, srslte::sch_pdu* pdu_msg, int max_sdu_sz, uint32_t* sdu_sz) 
{
 
  // Get n-th pending SDU pointer and length
  int sdu_len = rlc->get_buffer_state(lcid); 
  
  if (sdu_len > 0) { // there is pending SDU to allocate
    int buffer_state = sdu_len; 
    if (sdu_len > max_sdu_sz && max_sdu_sz >= 0) {
      sdu_len = max_sdu_sz;
    }
    int sdu_space = pdu_msg->get_sdu_space();
    if (sdu_len > sdu_space) {
      sdu_len = sdu_space;
    }        
    if (sdu_len > MIN_RLC_SDU_LEN) {
      if (pdu_msg->new_subh()) { // there is space for a new subheader
        int sdu_len2 = sdu_len; 
        sdu_len = pdu_msg->get()->set_sdu(lcid, sdu_len, rlc);
        if (sdu_len > 0) { // new SDU could be added
          if (sdu_sz) {
            *sdu_sz = sdu_len; 
          }
          
          Info("SDU: lcid=%d, rlc_buffer=%d, allocated=%d/%d, max_sdu_sz=%d, remaining=%d\n", 
                 lcid, buffer_state, sdu_len, sdu_space, max_sdu_sz, pdu_msg->rem_size());
          return true;               
        } else {
          Warning("SDU: rlc_buffer=%d, allocated=%d/%d, remaining=%d\n", 
               buffer_state, sdu_len, sdu_space, pdu_msg->rem_size());
          pdu_msg->del_subh();
        }
      } 
    }
  }
  return false; 
}

void mux::msg3_flush()
{
  Debug("Msg3 buffer flushed\n");
  msg3_buff.flush();
  msg3_has_been_transmitted = false; 
}

bool mux::msg3_is_transmitted()
{
  return msg3_has_been_transmitted; 
}


bool mux::pdu_move_to_msg3(uint32_t pdu_sz)
{
  uint8_t *msg3_start = (uint8_t*) msg3_buff.request();
  if (msg3_start) {
    uint8_t *msg3_pdu = pdu_get(msg3_start, pdu_sz, 0, 0); 
    if (msg3_pdu) {
      memmove(msg3_start, msg3_pdu, pdu_sz*sizeof(uint8_t));
      msg3_buff.push(pdu_sz);
      return true;       
    } else {
      Error("Assembling PDU\n");
    }    
  } else {
    Error("Generating PDU: PDU pending in buffer for transmission\n");
  }  
  return false; 
}

/* Returns a pointer to the Msg3 buffer */
uint8_t* mux::msg3_get(uint8_t *payload, uint32_t pdu_sz)
{
  if (msg3_buff.isempty()) {
    Debug("Moving PDU from Mux unit to Msg3 buffer\n");
    if (!pdu_move_to_msg3(pdu_sz)) {
      Error("Moving PDU from Mux unit to Msg3 buffer\n");
      return NULL;
    }    
  }
  uint8_t *msg3 = (uint8_t*) msg3_buff.pop();
  if (msg3) {
    memcpy(payload, msg3, sizeof(uint8_t)*pdu_sz);
    msg3_has_been_transmitted = true; 
    return payload; 
  } else {
    Error("Generating Msg3\n");
  }
  return NULL; 
}

  
}
