/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/******************************************************************************
 *
 *  This file contains EATT utility functions
 *
 *  When EATT is found to be supported on both local and remote devices,
 *  these utility functions will be used to process GATT operations on
 *  EATT and ATT channels.
 *
 ******************************************************************************/
#include "bt_target.h"
#include "bt_utils.h"
#include "osi/include/osi.h"

#include <string.h>
#include "bt_common.h"
#include "stdio.h"

#include "btm_int.h"
#include "connection_manager.h"
#include "gatt_api.h"
#include "gatt_int.h"
#include "gattdefs.h"
#include "l2cdefs.h"
#include "sdp_api.h"
#include "stack/gatt/connection_manager.h"
#include "stack/gatt/eatt_int.h"
#include "stack/gatt/gatt_int.h"

#include <vector>
#include <algorithm>

using base::StringPrintf;
using bluetooth::Uuid;

/*******************************************************************************
 *
 * Function         gatt_eatt_bcb_alloc
 *
 * Description      The function allocates a EATT bearer control block
 *
 * Returns          NULL if not found. Otherwise pointer to the EATT bearer control
 *                  block.
 *
 ******************************************************************************/
tGATT_EBCB* gatt_eatt_bcb_alloc(tGATT_TCB* p_tcb, uint16_t lcid, bool create_in_prg,
                                bool is_remote_initiated) {
  uint8_t i = 0;
  tGATT_EBCB* p_eatt_bcb = NULL;
  std::string conf_timer = "gatt.conf_timer";
  std::string ind_ack_timer = "gatt.ind_ack_timer";

  p_eatt_bcb = gatt_find_eatt_bcb_by_cid(lcid);
  if (p_eatt_bcb) {
    VLOG(1) << __func__ << " p_eatt_bcb already available for lcid: " << +lcid;
    return p_eatt_bcb;
  }

  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (!gatt_cb.eatt_bcb[i].in_use) {
      p_eatt_bcb = &gatt_cb.eatt_bcb[i];

      *p_eatt_bcb = tGATT_EBCB();

      p_eatt_bcb->in_use = true;
      p_eatt_bcb->p_tcb = p_tcb;
      p_eatt_bcb->cid = lcid;

      p_eatt_bcb->create_in_prg = create_in_prg;

      if (lcid != L2CAP_ATT_CID) {
        p_eatt_bcb->pending_ind_q = fixed_queue_new(SIZE_MAX);

        conf_timer += std::to_string(lcid);
        ind_ack_timer += std::to_string(lcid);
        p_eatt_bcb->payload_size = p_tcb->payload_size;
        p_eatt_bcb->conf_timer = alarm_new(conf_timer.c_str());
        p_eatt_bcb->ind_ack_timer = alarm_new(ind_ack_timer.c_str());
        p_eatt_bcb->is_remote_initiated = is_remote_initiated;
      }
      else {
        p_eatt_bcb->payload_size = p_tcb->payload_size;
        p_eatt_bcb->ind_ack_timer = p_tcb->ind_ack_timer;
        p_eatt_bcb->conf_timer = p_tcb->conf_timer;
        p_eatt_bcb->cl_cmd_q = p_tcb->cl_cmd_q;
        p_eatt_bcb->pending_ind_q = p_tcb->pending_ind_q;
        p_eatt_bcb->indicate_handle = p_tcb->indicate_handle;
        p_eatt_bcb->sr_cmd = p_tcb->sr_cmd;
        p_eatt_bcb->sr_cmd.multi_rsp_q = p_tcb->sr_cmd.multi_rsp_q;
      }
      break;
    }
  }

  return p_eatt_bcb;
}

/*******************************************************************************
 *
 * Function         gatt_eatt_bcb_dealloc
 *
 * Description      The function deallocates EATT bearer control block
 *
 * Returns          true if EATT bearer control block is deallocated.
 *
 ******************************************************************************/
bool gatt_eatt_bcb_dealloc(uint16_t lcid) {
  uint16_t i = 0;
  tGATT_EBCB* p_eatt_bcb = NULL;
  bool ret = false;

  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (gatt_cb.eatt_bcb[i].in_use &&
       (gatt_cb.eatt_bcb[i].cid == lcid)) {
      p_eatt_bcb = &gatt_cb.eatt_bcb[i];

      gatt_remove_conns_by_cid(p_eatt_bcb->cid);

      if (lcid != L2CAP_ATT_CID) {
        alarm_free(p_eatt_bcb->ind_ack_timer);
        p_eatt_bcb->ind_ack_timer = NULL;
        alarm_free(p_eatt_bcb->conf_timer);
        p_eatt_bcb->conf_timer = NULL;
        gatt_free_pending_ind(p_eatt_bcb->p_tcb, lcid);
        fixed_queue_free(p_eatt_bcb->sr_cmd.multi_rsp_q, NULL);
        p_eatt_bcb->sr_cmd.multi_rsp_q = NULL;
      }

      p_eatt_bcb->in_use = false;
      p_eatt_bcb->p_tcb = NULL;
      p_eatt_bcb->cid = -1;

      p_eatt_bcb->create_in_prg = false;
      p_eatt_bcb->is_remote_initiated = false;
      p_eatt_bcb->apps.clear();
      p_eatt_bcb->opportunistic_apps.clear();

      *p_eatt_bcb = tGATT_EBCB();
      ret = true;
    }
  }
  return ret;
}

/*******************************************************************************
 *
 * Function         gatt_eatt_bcb_in_progress_dealloc
 *
 * Description      The function deallocates EATT bearer control blocks for
 *                  channels whose creation is in progress for a remote
 *                  device.
 *
 * Returns          Number of EATT bearer control blocks deallocated.
 *
 ******************************************************************************/
uint8_t gatt_eatt_bcb_in_progress_dealloc(RawAddress& bda) {
  uint16_t i = 0;
  tGATT_EBCB* p_eatt_bcb = NULL;
  uint8_t num_dealloc = 0;

  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (gatt_cb.eatt_bcb[i].in_use && (gatt_cb.eatt_bcb[i].create_in_prg) &&
       (gatt_cb.eatt_bcb[i].p_tcb->peer_bda == bda)) {
      p_eatt_bcb = &gatt_cb.eatt_bcb[i];

      *p_eatt_bcb = tGATT_EBCB();

      p_eatt_bcb->in_use = false;
      p_eatt_bcb->p_tcb = NULL;
      p_eatt_bcb->cid = -1;

      p_eatt_bcb->create_in_prg = false;
      p_eatt_bcb->apps.clear();
      num_dealloc++;
    }
  }

  return num_dealloc;
}

/*******************************************************************************
 *
 * Function         gatt_find_tcb_by_eatt_cid
 *
 * Description      The function searches for the tcb entry
 *                  based on EATT bearer CID
 *
 * Returns          NULL if not found. Otherwise pointer to the tcb.
 *
 ******************************************************************************/
tGATT_TCB* gatt_find_tcb_by_eatt_cid(uint16_t lcid) {
  uint16_t i = 0;
  tGATT_TCB* p_tcb = NULL;

  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (gatt_cb.eatt_bcb[i].in_use && (gatt_cb.eatt_bcb[i].cid == lcid)
        && (!gatt_cb.eatt_bcb[i].create_in_prg)) {
      p_tcb = gatt_cb.eatt_bcb[i].p_tcb;
      break;
    }
  }
  return p_tcb;
}

/*******************************************************************************
 *
 * Function         gatt_find_eatt_bcb_by_cid
 *
 * Description      The function searches for the eatt_bcb entry
 *                  based on channel id
 *
 * Returns          NULL if not found. Otherwise pointer to the eatt_bcb.
 *
 ******************************************************************************/
tGATT_EBCB* gatt_find_eatt_bcb_by_cid(uint16_t lcid) {
  uint16_t i = 0;
  tGATT_EBCB* p_eatt_bcb = NULL;

  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (gatt_cb.eatt_bcb[i].in_use && gatt_cb.eatt_bcb[i].cid == lcid
        && (!gatt_cb.eatt_bcb[i].create_in_prg)) {
      p_eatt_bcb = &gatt_cb.eatt_bcb[i];
      break;
    }
  }
  return p_eatt_bcb;
}

/*******************************************************************************
 *
 * Function         gatt_find_eatt_bcb_by_srv_trans_id
 *
 * Description      The function searches for the eatt_bcb entry based on
 *                  server transaction id.
 *
 *                  When response is received by stack from server app for an
 *                  incoming GATT client transaction request, server transaction
 *                  id is used to retrieve the appropriate l2cap channel to send
 *                  the GATT server response.
 *
 *
 * Returns          NULL if not found. Otherwise pointer to the eatt_bcb.
 *
 ******************************************************************************/
tGATT_EBCB* gatt_find_eatt_bcb_by_srv_trans_id(uint32_t trans_id, const RawAddress& bda) {
  uint16_t i = 0;
  tGATT_EBCB* p_eatt_bcb = NULL;

  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (gatt_cb.eatt_bcb[i].in_use && (gatt_cb.eatt_bcb[i].sr_cmd.trans_id == trans_id)
       && (gatt_cb.eatt_bcb[i].p_tcb->peer_bda == bda) && (!gatt_cb.eatt_bcb[i].create_in_prg)) {
      p_eatt_bcb = &gatt_cb.eatt_bcb[i];
      break;
    }
  }
  return p_eatt_bcb;
}

/*******************************************************************************
 *
 * Function         gatt_find_eatt_bcb_by_cl_trans_id
 *
 * Description      The function searches for the eatt_bcb entry
 *                  based on client transcation id.
 *
 *                  When confirmation response needs to be sent for a
 *                  received GATT server indication, client transaction id is
 *                  used to retrieve the appropriate l2cap channel to send the
 *                  confirmation PDU.
 *
 *
 * Returns          NULL if not found. Otherwise pointer to the eatt_bcb.
 *
 ******************************************************************************/
tGATT_EBCB* gatt_find_eatt_bcb_by_cl_trans_id(uint32_t trans_id, const RawAddress& bda) {
  uint16_t i = 0;
  tGATT_EBCB* p_eatt_bcb = NULL;

  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (gatt_cb.eatt_bcb[i].in_use && (gatt_cb.eatt_bcb[i].cl_trans_id == trans_id)
        && (gatt_cb.eatt_bcb[i].p_tcb->peer_bda == bda) && (!gatt_cb.eatt_bcb[i].create_in_prg)) {
      p_eatt_bcb = &gatt_cb.eatt_bcb[i];
      break;
    }
  }
  return p_eatt_bcb;
}

/*******************************************************************************
 *
 * Function         gatt_find_eatt_bcb_by_gatt_if
 *
 * Description      The function searches for the eatt_bcb entry
 *                  based on gatt if
 *
 * Returns          NULL if not found. Otherwise pointer to the eatt_bcb.
 *
 ******************************************************************************/
tGATT_EBCB* gatt_find_eatt_bcb_by_gatt_if(uint8_t gatt_if, const RawAddress& bda) {
  uint16_t i = 0;
  tGATT_EBCB* p_eatt_bcb = NULL;
  std::vector<tGATT_IF> apps;

  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (gatt_cb.eatt_bcb[i].in_use && (gatt_cb.eatt_bcb[i].p_tcb->peer_bda == bda)
        && (!gatt_cb.eatt_bcb[i].create_in_prg)) {
      apps = gatt_cb.eatt_bcb[i].apps;
      if(std::find(apps.begin(), apps.end(), gatt_if) != apps.end()) {
        p_eatt_bcb = &gatt_cb.eatt_bcb[i];
        break;
      }
    }
  }
  return p_eatt_bcb;
}

/*******************************************************************************
 *
 * Function         gatt_find_eatt_bcb_by_cid_in_progress
 *
 * Description      The function searches for the eatt_bcb entry
 *                  based on channel id which are in progress.
 *
 * Returns          NULL if not found. Otherwise pointer to the eatt_bcb.
 *
 ******************************************************************************/
tGATT_EBCB* gatt_find_eatt_bcb_by_cid_in_progress(RawAddress bda, uint16_t lcid) {
  uint16_t i = 0;
  tGATT_EBCB* p_eatt_bcb = NULL;

  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (gatt_cb.eatt_bcb[i].in_use
        && (gatt_cb.eatt_bcb[i].p_tcb->peer_bda == bda)
        && (gatt_cb.eatt_bcb[i].cid == lcid)
        && (gatt_cb.eatt_bcb[i].create_in_prg)) {
      p_eatt_bcb = &gatt_cb.eatt_bcb[i];
      break;
    }
  }
  return p_eatt_bcb;
}

bool compare_num_apps_in_ebcb_list(tGATT_APPS_EBCB a, tGATT_APPS_EBCB b) {
  return(a.num_apps < b.num_apps);
}

/*******************************************************************************
 *
 * Function         gatt_find_best_eatt_bcb
 *
 * Description      The function searches for the least burdened EATT bearer.
 *                  If no suitable EATT bearers are available, ATT channel will
 *                  be selected.
 *
 * Returns          pointer to the selected eatt_bcb.
 *
 ******************************************************************************/
tGATT_EBCB* gatt_find_best_eatt_bcb(tGATT_TCB* p_tcb, tGATT_IF gatt_if, uint16_t old_cid, bool opportunistic) {
  uint16_t i = 0;
  tGATT_EBCB* p_eatt_bcb = NULL;
  std::vector<tGATT_APPS_EBCB> apps_in_ebcb_list;
  tGATT_APPS_EBCB apps_in_ebcb;
  uint16_t conn_id = 0;
  tGATT_EBCB* p_eatt_bcb_old = gatt_find_eatt_bcb_by_cid(old_cid);
  tGATT_REG* p_reg = gatt_get_regcb(gatt_if);

  //Find all EATT channels with suitable MTU size
  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (gatt_cb.eatt_bcb[i].in_use && (gatt_cb.eatt_bcb[i].p_tcb->peer_bda == p_tcb->peer_bda)
        && (!gatt_cb.eatt_bcb[i].create_in_prg) && (!gatt_cb.eatt_bcb[i].disconn_in_prg)
        && (gatt_cb.eatt_bcb[i].cid != L2CAP_ATT_CID) && (p_reg->eatt_support)
        && (!gatt_cb.eatt_bcb[i].is_remote_initiated)) {
      if (old_cid > 0) {
        if ((p_eatt_bcb_old) && (gatt_cb.eatt_bcb[i].payload_size < p_eatt_bcb_old->payload_size)) {
          continue;
        }
      }
      p_eatt_bcb = &gatt_cb.eatt_bcb[i];

      if (p_eatt_bcb) {
        apps_in_ebcb.lcid = p_eatt_bcb->cid;
        apps_in_ebcb.num_apps = p_eatt_bcb->apps.size();
        apps_in_ebcb_list.push_back(apps_in_ebcb);
      }
    }
  }

  //EATT channels available to select
  if(apps_in_ebcb_list.size() > 0) {
    //Select the least burdened EATT channel
    std::sort(apps_in_ebcb_list.begin(), apps_in_ebcb_list.end(), compare_num_apps_in_ebcb_list);
    p_eatt_bcb = gatt_find_eatt_bcb_by_cid(apps_in_ebcb_list[0].lcid);
  }
  // Unable to find any suitable EATT channel, use ATT
  else {
    p_eatt_bcb = gatt_find_eatt_bcb_by_cid(L2CAP_ATT_CID);
    if ((old_cid > 0) && p_eatt_bcb && p_eatt_bcb_old &&
        (p_eatt_bcb->payload_size < p_eatt_bcb_old->payload_size)) {
      VLOG(1) << __func__ << " cid:" << +p_eatt_bcb->cid << " does not have required MTU";
      p_eatt_bcb = NULL;
    }
  }

  if (p_eatt_bcb) {
    if (opportunistic)
      p_eatt_bcb->opportunistic_apps.push_back(gatt_if);
    else
      p_eatt_bcb->apps.push_back(gatt_if);

    VLOG(1) << __func__ << " Least burdened channel selected:" << +p_eatt_bcb->cid;
    //Add mapping of conn_id and lcid
    conn_id = GATT_CREATE_CONN_ID(p_tcb->tcb_idx, gatt_if);
    gatt_add_conn(conn_id, p_eatt_bcb->cid);
  }
  else {
    uint16_t conn_handle = BTM_GetHCIConnHandle(p_tcb->peer_bda, BT_TRANSPORT_LE);
    btm_sec_disconnect(conn_handle, GATT_CONN_TERMINATE_LOCAL_HOST);
  }

  return p_eatt_bcb;
}

/*******************************************************************************
 *
 * Function         gatt_num_eatt_bcbs_in_progress
 *
 * Description      The function finds number of eatt_bcb entries(EATT l2cap conn reqs)
 *                  in progress
 *
 * Returns          NULL if not found. Otherwise pointer to the eatt_bcb.
 *
 ******************************************************************************/
uint8_t gatt_num_eatt_bcbs_in_progress(tGATT_TCB* p_tcb) {
  uint16_t i = 0;
  uint8_t num_eatt_bcbs = 0;

  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (gatt_cb.eatt_bcb[i].in_use && (gatt_cb.eatt_bcb[i].p_tcb->peer_bda == p_tcb->peer_bda)
        && (gatt_cb.eatt_bcb[i].create_in_prg)) {
      num_eatt_bcbs++;
    }
  }
  return num_eatt_bcbs;
}

/*******************************************************************************
 *
 * Function         gatt_num_eatt_bcbs
 *
 * Description      The function finds number of EATT channels for a bda
 *
 *
 * Returns          number of EATT channels for a bd address.
 *
 ******************************************************************************/
uint8_t gatt_num_eatt_bcbs(tGATT_TCB* p_tcb) {
  uint16_t i = 0;
  uint8_t num_eatt_bcbs = 0;

  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (gatt_cb.eatt_bcb[i].in_use && (gatt_cb.eatt_bcb[i].p_tcb->peer_bda == p_tcb->peer_bda)
        && (!gatt_cb.eatt_bcb[i].create_in_prg)
        && (gatt_cb.eatt_bcb[i].cid != L2CAP_ATT_CID)) {
      num_eatt_bcbs++;
    }
  }
  VLOG(1) << __func__ << " num_eatt_bcbs:" << +num_eatt_bcbs;
  return num_eatt_bcbs;
}

/*******************************************************************************
 *
 * Function         gatt_get_payload_size
 *
 * Description      The function gets the payload size for the channel
 *
 * Returns          payload_size of cid.
 *
 ******************************************************************************/
uint16_t gatt_get_payload_size(tGATT_TCB* p_tcb, uint16_t lcid) {
  uint16_t i = 0;
  tGATT_EBCB* p_eatt_bcb = NULL;
  uint16_t payload_size = 0;

  if (!p_tcb->is_eatt_supported || (lcid == p_tcb->att_lcid)) {
    payload_size = p_tcb->payload_size;
  }
  else if (p_tcb->is_eatt_supported) {
    for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
      if (gatt_cb.eatt_bcb[i].in_use && gatt_cb.eatt_bcb[i].cid == lcid
          && (!gatt_cb.eatt_bcb[i].create_in_prg)) {
        p_eatt_bcb = &gatt_cb.eatt_bcb[i];
        if (p_eatt_bcb) {
          payload_size = p_eatt_bcb->payload_size;
        }
        return payload_size;
      }
    }
  }
  return payload_size;
}

struct find_gatt_conn
{
  uint16_t conn_id;
  uint16_t lcid;
  find_gatt_conn(uint16_t conn_id, uint16_t lcid) : conn_id(conn_id), lcid(lcid) {}
  bool operator () (const tGATT_CONN& gatt_conn) const {
    return ((gatt_conn.conn_id == conn_id) && (gatt_conn.lcid == lcid));
  }
};

struct find_gatt_cid
{
  uint16_t conn_id;
  find_gatt_cid(uint16_t conn_id) : conn_id(conn_id) {}
  bool operator () (const tGATT_CONN& gatt_conn) const {
    return (gatt_conn.conn_id == conn_id);
  }
};

struct is_gatt_cid_found
{
  uint16_t cid;
  is_gatt_cid_found(uint16_t cid) : cid(cid) {}
  bool operator () (const tGATT_CONN& gatt_conn) const {
    return (gatt_conn.lcid == cid);
  }
};

/*******************************************************************************
 *
 * Function         gatt_remove_conn
 *
 * Description      The function deletes gatt_conn entry
 *
 * Returns          void.
 *
 ******************************************************************************/
void gatt_remove_conn(uint16_t conn_id, uint16_t lcid) {
  std::vector<tGATT_CONN>::iterator it
      = std::find_if(gatt_cb.gatt_conn_list.begin(), gatt_cb.gatt_conn_list.end(), find_gatt_conn(conn_id, lcid));
  VLOG(1) << __func__;
  if (it != gatt_cb.gatt_conn_list.end()) {
    gatt_cb.gatt_conn_list.erase(it);
  }
}

/*******************************************************************************
 *
 * Function         gatt_remove_conns_by_cid
 *
 * Description      The function deletes all gatt_conn entries for a lcid
 *
 * Returns          void.
 *
 ******************************************************************************/
void gatt_remove_conns_by_cid(uint16_t lcid) {
  if (gatt_cb.gatt_conn_list.empty()) {
    VLOG(1) << __func__ << " empty list:";
    return;
  }

  std::vector<tGATT_CONN>::iterator it
        = std::remove_if(gatt_cb.gatt_conn_list.begin(), gatt_cb.gatt_conn_list.end(), is_gatt_cid_found(lcid));
  if (it != gatt_cb.gatt_conn_list.end()) {
    gatt_cb.gatt_conn_list.erase(it, gatt_cb.gatt_conn_list.end());
  }
}

/*******************************************************************************
 *
 * Function         is_gatt_conn_id_found
 *
 * Description      The function checks whether conn id is found
 *
 * Returns          true if entry found.
 *
 ******************************************************************************/
bool is_gatt_conn_id_found(uint16_t conn_id) {
  bool ret = false;
  std::vector<tGATT_CONN>::iterator it
      = std::find_if(gatt_cb.gatt_conn_list.begin(), gatt_cb.gatt_conn_list.end(), find_gatt_cid(conn_id));

  if (it != gatt_cb.gatt_conn_list.end()) {
    ret = true;
  }
  return ret;
}

/*******************************************************************************
 *
 * Function         is_gatt_conn_found
 *
 * Description      The function checks whether gatt_conn entry is found
 *
 * Returns          true if entry found.
 *
 ******************************************************************************/
bool is_gatt_conn_found(uint16_t conn_id, uint16_t lcid) {
  bool ret = false;
  std::vector<tGATT_CONN>::iterator it
      = std::find_if(gatt_cb.gatt_conn_list.begin(), gatt_cb.gatt_conn_list.end(), find_gatt_conn(conn_id, lcid));

  if (it != gatt_cb.gatt_conn_list.end()) {
    ret = true;
  }
  return ret;
}

/*******************************************************************************
 *
 * Function         gatt_add_conn
 *
 * Description      The function adds a gatt_conn entry
 *
 * Returns          true if entry found.
 *
 ******************************************************************************/
void gatt_add_conn(uint16_t conn_id, uint16_t lcid) {
  //check whether an entry already exists, dont add if it exists
  if (is_gatt_conn_found(conn_id, lcid)) {
    VLOG(1) << __func__ << " conn entry already available";
    return;
  }

  tGATT_CONN gatt_conn = {
    .conn_id = conn_id,
    .lcid = lcid
  };
  gatt_cb.gatt_conn_list.push_back(gatt_conn);
}

/*******************************************************************************
 *
 * Function         gatt_get_cid_by_conn_id
 *
 * Description      The function gets cid for a given conn_id
 *
 * Returns          lcid associated with conn_id.
 *
 ******************************************************************************/
uint16_t gatt_get_cid_by_conn_id(uint16_t conn_id) {
  uint16_t lcid = L2CAP_ATT_CID;
  tGATT_CONN gatt_conn;

  std::vector<tGATT_CONN>::iterator it
      = std::find_if(gatt_cb.gatt_conn_list.begin(), gatt_cb.gatt_conn_list.end(), find_gatt_cid(conn_id));

  if (it != gatt_cb.gatt_conn_list.end()) {
    gatt_conn = *it;
    lcid = gatt_conn.lcid;
  }
  return lcid;
}

/*******************************************************************************
 *
 * Function         gatt_upgrade_conn
 *
 * Description      The function upgrades apps from ATT to EATT channel
 *
 * Returns          void.
 *
 ******************************************************************************/
void gatt_upgrade_conn(tGATT_TCB* p_tcb) {
  uint8_t num_apps_hold_link = 0;
  tGATT_REG* p_reg;

  if (p_tcb->is_eatt_upgrade_done) {
    VLOG(1) << __func__ << " EATT upgrade already done";
    return;
  }

  auto& holders = p_tcb->app_hold_link;
  num_apps_hold_link = holders.size();
  VLOG(1) << __func__ << " num_apps_hold_link: " << +num_apps_hold_link;
  for (auto it = holders.begin(); it != holders.end(); ++it) {
    p_reg = gatt_get_regcb(*it);
    if (p_reg && p_reg->eatt_support) {
      std::deque<tGATT_IF>::iterator iter =
          std::find(p_tcb->apps_needing_eatt.begin(), p_tcb->apps_needing_eatt.end(), *it);
      if (iter == p_tcb->apps_needing_eatt.end()) {
        p_tcb->apps_needing_eatt.push_back(*it);
      }
    }
  }

  if (!p_tcb->apps_needing_eatt.empty()) {
    gatt_establish_eatt_connect(p_tcb, 1);
  }
  p_tcb->is_eatt_upgrade_done = true;
}

/*******************************************************************************
 *
 * Function         gatt_send_pending_ind
 *
 * Description      This function check any pending indication needs to be sent
 *                  if there is a pending indication then sent the indication
 *
 * Returns          void
 *
 ******************************************************************************/
void gatt_send_pending_ind(tGATT_TCB& tcb, uint16_t lcid) {
  tGATT_EBCB* p_eatt_bcb = NULL;
  fixed_queue_t** pending_ind_q = &(tcb.pending_ind_q);
  tGATT_VALUE* p_buf =
      (tGATT_VALUE*)fixed_queue_try_peek_first(tcb.pending_ind_q);

  if (tcb.is_eatt_supported) {
    p_eatt_bcb = gatt_find_eatt_bcb_by_cid(lcid);
    if (p_eatt_bcb) {
      VLOG(1) << __func__ << " Known EATT bearer";
      pending_ind_q = &(p_eatt_bcb->pending_ind_q);
      p_buf = (tGATT_VALUE*)fixed_queue_try_peek_first(*pending_ind_q);
    }
  }

  p_buf = (tGATT_VALUE*)fixed_queue_try_peek_first(*pending_ind_q);
  if (p_buf != NULL) {
    GATTS_HandleValueIndication(p_buf->conn_id, p_buf->handle, p_buf->len,
                                p_buf->value);
    osi_free(fixed_queue_try_remove_from_queue(*pending_ind_q, p_buf));
  }
}

/*******************************************************************************
 *
 * Function         gatt_move_apps
 *
 * Description      The function moves apps(and their client/server operations)
 *                  from the disconnected channel to another suitable channel.
 *
 * Returns          void
 *
 ******************************************************************************/
void gatt_move_apps(uint16_t cid) {
  tGATT_EBCB* p_eatt_bcb_old = gatt_find_eatt_bcb_by_cid(cid);
  tGATT_TCB* p_tcb = NULL;
  tGATT_EBCB* p_eatt_bcb = NULL;
  tGATT_IF gatt_if;
  uint16_t cl_conn_id = 0;
  tGATT_CLCB* p_clcb_current = NULL;
  uint16_t sr_conn_id = 0;
  uint16_t sr_handle = 0;

  VLOG(1) << __func__ << " cid:" << +cid;

  if (p_eatt_bcb_old) {
    p_tcb = p_eatt_bcb_old->p_tcb;

    if (!p_tcb->is_eatt_supported || (cid == L2CAP_ATT_CID)) {
      VLOG(1) << __func__ << " Invalid scenario for moving apps";
      return;
    }

    if (p_eatt_bcb_old->cl_cmd_q.size() > 0) {
      tGATT_CMD_Q& cmd = p_eatt_bcb_old->cl_cmd_q.front();
      if (!cmd.to_send) {
        cl_conn_id = cmd.p_clcb->conn_id;
        p_clcb_current = cmd.p_clcb;
      }
    }

    sr_handle = p_eatt_bcb_old->indicate_handle;
    if (GATT_HANDLE_IS_VALID(sr_handle)) {
      for (auto& el : *gatt_cb.srv_list_info) {
        if (el.s_hdl <= sr_handle && el.e_hdl >= sr_handle) {
          sr_conn_id = GATT_CREATE_CONN_ID(p_tcb->tcb_idx, el.gatt_if);
          break;
        }
      }
    }
  }

  if ((cid != L2CAP_ATT_CID) && p_tcb->is_eatt_supported &&
      (p_tcb->transport == BT_TRANSPORT_LE) && p_eatt_bcb_old) {
    std::vector<tGATT_APP_INFO> apps;
    std::vector<tGATT_APPS_Q> apps_q;
    uint8_t i = 0;
    uint8_t xx = 0;
    uint16_t conn_id;

    for (xx=0; xx < p_eatt_bcb_old->apps.size(); xx++) {
      tGATT_APP_INFO app_info = {.gatt_if = p_eatt_bcb_old->apps[xx],
                                 .opportunistic = false};
      apps.push_back(app_info);
    }
    for (xx=0; xx < p_eatt_bcb_old->opportunistic_apps.size(); xx++) {
      tGATT_APP_INFO app_info = {.gatt_if = p_eatt_bcb_old->opportunistic_apps[xx],
                                 .opportunistic = true};
      apps.push_back(app_info);
    }

    //Iterate for all the apps which are on this disconnected cid
    for (i=0; i<apps.size(); i++) {
      tGATT_APP_INFO app_info = apps[i];
      gatt_if = app_info.gatt_if;
      conn_id = GATT_CREATE_CONN_ID(p_tcb->tcb_idx, gatt_if);

      //Remove the conn_id from previous lcid
      gatt_remove_conn(conn_id, cid);
      p_eatt_bcb_old->disconn_in_prg = true;

      //Find least burdened channel for the gatt_if
      p_eatt_bcb = gatt_find_best_eatt_bcb(p_tcb, gatt_if, cid, app_info.opportunistic);

      if (p_eatt_bcb) {
        VLOG(1) << __func__ << " selected lcid:" << +p_eatt_bcb->cid;
        tGATT_APPS_Q app = {.conn_id = conn_id,
                            .lcid = p_eatt_bcb->cid,
                            .cl_to_send = false,
                            .sr_to_send = false};

       if (p_eatt_bcb->cl_cmd_q.empty()) {
          app.cl_to_send = true;
        }
        if (!GATT_HANDLE_IS_VALID(p_eatt_bcb->indicate_handle)) {
          app.sr_to_send = true;
        }
        apps_q.push_back(app);
      }
    }

    //Move client operations to selected cid
    while (!p_eatt_bcb_old->cl_cmd_q.empty()) {
      tGATT_CMD_Q& cmd = p_eatt_bcb_old->cl_cmd_q.front();

      for (uint8_t i=0; i<apps_q.size(); i++) {
        tGATT_APPS_Q app = apps_q[i];
        if ((app.conn_id == cmd.p_clcb->conn_id) && (app.conn_id != cl_conn_id) &&
            (cmd.p_cmd != NULL)) {
          p_eatt_bcb = gatt_find_eatt_bcb_by_cid(app.lcid);
          p_eatt_bcb->cl_cmd_q.push(cmd);
        }
      }
      p_eatt_bcb_old->cl_cmd_q.pop();
    }

    //Move server indications to selected cid
    while (!fixed_queue_is_empty(p_eatt_bcb_old->pending_ind_q)) {
      tGATT_VALUE* p_buf = (tGATT_VALUE*)fixed_queue_try_peek_first(p_eatt_bcb_old->pending_ind_q);

      if (p_buf != NULL) {
        for (uint8_t i=0; i<apps_q.size(); i++) {
          tGATT_APPS_Q app = apps_q[i];
          if ((app.conn_id == p_buf->conn_id) && (app.conn_id != sr_conn_id)) {
            p_eatt_bcb = gatt_find_eatt_bcb_by_cid(app.lcid);
            gatt_add_pending_ind(p_tcb, p_eatt_bcb->cid, p_buf);
          }
        }
        osi_free(fixed_queue_try_remove_from_queue(p_eatt_bcb_old->pending_ind_q, p_buf));
      }
    }

    for (i=0; i<apps_q.size(); i++) {
      if (apps_q[i].cl_to_send) {
        gatt_cl_send_next_cmd_inq(*(p_tcb), apps_q[i].lcid);
      }

      if (apps_q[i].sr_to_send) {
        gatt_send_pending_ind(*(p_tcb), apps_q[i].lcid);
      }
    }

    if (p_clcb_current) {
      alarm_cancel(p_clcb_current->gatt_rsp_timer_ent);
      gatt_end_operation(p_clcb_current, GATT_ERROR, NULL);
    }
  }
}

/*******************************************************************************
 *
 * Function         gatt_add_eatt_device
 *
 * Description      The function adds eatt device to db.
 *
 * Returns          void
 *
 ******************************************************************************/
void gatt_add_eatt_device(RawAddress bda) {
  gatt_cb.eatt_devices_list.push_back(bda);
}

/*******************************************************************************
 *
 * Function         is_eatt_device
 *
 * Description      The function checks whether bda is an eatt supported device.
 *
 * Returns          true if bda supports EATT, false otherwise
 *
 ******************************************************************************/
bool is_eatt_device(RawAddress bda) {
  std::vector<RawAddress>::iterator it;
  bool ret = false;

  it = std::find(gatt_cb.eatt_devices_list.begin(), gatt_cb.eatt_devices_list.end(), bda);
  if (it != gatt_cb.eatt_devices_list.end()) {
    ret = true;
    VLOG(1) << __func__ << " bda:" << +bda.ToString().c_str() << " is EATT supported device";
  }
  return ret;
}

/*******************************************************************************
 *
 * Function         gatt_remove_app_on_lcid
 *
 * Description      The function removes app on lcid.
 *
 * Returns          number of apps still on lcid
 *
 ******************************************************************************/
uint8_t gatt_remove_app_on_lcid(uint16_t lcid, tGATT_IF gatt_if) {
  uint8_t num_apps = 0;
  uint16_t i = 0;
  tGATT_EBCB* p_eatt_bcb = NULL;
  uint16_t conn_id = 0;

  VLOG(1) << __func__ << " lcid:" << +lcid;
  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (gatt_cb.eatt_bcb[i].in_use && (gatt_cb.eatt_bcb[i].cid == lcid)
        && (!gatt_cb.eatt_bcb[i].create_in_prg) && (!gatt_cb.eatt_bcb[i].disconn_in_prg)
        && (gatt_cb.eatt_bcb[i].cid != L2CAP_ATT_CID)) {
      p_eatt_bcb = &gatt_cb.eatt_bcb[i];
      break;
    }
  }

  if (p_eatt_bcb) {
    conn_id = GATT_CREATE_CONN_ID(p_eatt_bcb->p_tcb->tcb_idx, gatt_if);
    std::vector<tGATT_IF>::iterator pos =
        std::find(p_eatt_bcb->opportunistic_apps.begin(), p_eatt_bcb->opportunistic_apps.end(), gatt_if);
    if (pos != p_eatt_bcb->opportunistic_apps.end()) {
      p_eatt_bcb->opportunistic_apps.erase(pos);
      gatt_remove_conn(conn_id, lcid);
    }
    else {
      pos = std::find(p_eatt_bcb->apps.begin(), p_eatt_bcb->apps.end(), gatt_if);
      if (pos != p_eatt_bcb->apps.end()) {
        p_eatt_bcb->apps.erase(pos);
        gatt_remove_conn(conn_id, lcid);
      }
    }

    num_apps = p_eatt_bcb->apps.size();
  }

  return num_apps;
}

/*******************************************************************************
 *
 * Function         eatt_cleanup_upon_disc
 *
 * Description      The function cleans up EATT channels when LE link
 *                  is disconnected.
 *
 * Returns          void
 *
 ******************************************************************************/
void eatt_cleanup_upon_disc(const RawAddress& bda) {
  uint16_t i = 0;
  tGATT_EBCB* p_eatt_bcb = NULL;

  VLOG(1) << __func__;
  for (i = 0; i < GATT_MAX_EATT_CHANNELS; i++) {
    if (gatt_cb.eatt_bcb[i].in_use && (gatt_cb.eatt_bcb[i].p_tcb->peer_bda == bda)
        && (!gatt_cb.eatt_bcb[i].create_in_prg)) {
      p_eatt_bcb = &gatt_cb.eatt_bcb[i];
      gatt_eatt_bcb_dealloc(p_eatt_bcb->cid);
    }
  }
}