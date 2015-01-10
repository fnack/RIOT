/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     nhdp
 * @{
 *
 * @file
 * @brief       Reader implementation for message processing in NHDP
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 *
 * @}
 */

#include "mutex.h"
#include "utlist.h"

#include "rfc5444/rfc5444.h"
#include "rfc5444/rfc5444_iana.h"
#include "rfc5444/rfc5444_reader.h"

#include "lib_table.h"
#include "nib_table.h"
#include "iib_table.h"
#include "nhdp.h"
#include "nhdp_address.h"
#include "nhdp_reader.h"

/* Internal variables */
struct rfc5444_reader reader;
static mutex_t mtx_packet_handler = MUTEX_INIT;

static nhdp_addr_entry_t *send_addr_list_head;
static nhdp_addr_entry_t *nb_addr_list_head;
static nhdp_addr_entry_t *th_sym_addr_list_head;
static nhdp_addr_entry_t *th_rem_addr_list_head;
static nhdp_addr_entry_t *rem_addr_list_head;

static iib_link_set_entry_t *originator_link_tuple;
static nhdp_addr_metr_t *addr_metric_head;

static kernel_pid_t if_pid;
static uint64_t val_time;
static uint64_t int_time;
static uint8_t sym = 0;
static uint8_t lost = 0;

static uint32_t lt_metric_val = NHDP_METRIC_UNKNOWN;

/* Internal function prototypes */
static enum rfc5444_result _nhdp_pkt_end_cb(struct rfc5444_reader_tlvblock_context *context,
        bool dropped);
static enum rfc5444_result _nhdp_blocktlv_msg_cb(struct rfc5444_reader_tlvblock_context *cont);
static enum rfc5444_result _nhdp_blocktlv_address_cb(struct rfc5444_reader_tlvblock_context *cont);
static enum rfc5444_result _nhdp_msg_end_cb(struct rfc5444_reader_tlvblock_context *cont,
        bool dropped);
static enum rfc5444_result check_msg_validity(struct rfc5444_reader_tlvblock_context *cont);
static enum rfc5444_result check_addr_validity(nhdp_addr_t *addr);
static nhdp_addr_t* get_nhdp_db_addr(uint8_t *addr, uint8_t prefix);
static void process_temp_tables(void);
static void cleanup_temp_addr_lists(void);

/* Array containing the processable message TLVs for HELLO messages */
static struct rfc5444_reader_tlvblock_consumer_entry _nhdp_msg_tlvs[] = {
    [RFC5444_MSGTLV_INTERVAL_TIME] = { .type = RFC5444_MSGTLV_INTERVAL_TIME, .mandatory = false },
    [RFC5444_MSGTLV_VALIDITY_TIME] = { .type = RFC5444_MSGTLV_VALIDITY_TIME, .mandatory = true },
};

/* Array containing the processable address TLVs for HELLO message address blocks */
static struct rfc5444_reader_tlvblock_consumer_entry _nhdp_addr_tlvs[] = {
    [RFC5444_ADDRTLV_LOCAL_IF] = { .type = RFC5444_ADDRTLV_LOCAL_IF },
    [RFC5444_ADDRTLV_LINK_STATUS] = { .type = RFC5444_ADDRTLV_LINK_STATUS },
    [RFC5444_ADDRTLV_OTHER_NEIGHB] = { .type = RFC5444_ADDRTLV_OTHER_NEIGHB },
    [RFC5444_ADDRTLV_LINK_METRIC] = { .type = RFC5444_ADDRTLV_LINK_METRIC, .type_ext = NHDP_METRIC,
            .match_type_ext = true, .min_length = 0x02, .max_length = 0x02, .match_length = true },
};

/* oonf_api packet consumer used for RFC5444 packet consumption */
static struct rfc5444_reader_tlvblock_consumer _nhdp_packet_consumer = {
  .end_callback = _nhdp_pkt_end_cb,
};

/* oonf_api message consumer used for HELLO message consumption */
static struct rfc5444_reader_tlvblock_consumer _nhdp_msg_consumer = {
    .msg_id = RFC5444_MSGTYPE_HELLO,
    .block_callback = _nhdp_blocktlv_msg_cb,
    .end_callback = _nhdp_msg_end_cb,
};

/* oonf_api message consumer user for HELLO message address block consumption */
static struct rfc5444_reader_tlvblock_consumer _nhdp_address_consumer = {
    .msg_id = RFC5444_MSGTYPE_HELLO,
    .addrblock_consumer = true,
    .block_callback = _nhdp_blocktlv_address_cb,
};


/*---------------------------------------------------------------------------*
 *                             NHDP Reader API                               *
 *---------------------------------------------------------------------------*/

void nhdp_reader_init(void)
{
    /* Reset locally created address lists */
    send_addr_list_head = NULL;
    nb_addr_list_head = NULL;
    th_sym_addr_list_head = NULL;
    th_rem_addr_list_head = NULL;
    rem_addr_list_head = NULL;

    /* Reset addr_metric container */
    addr_metric_head = NULL;

    /* Reset originator */
    originator_link_tuple = NULL;

    /* Initialize reader */
    rfc5444_reader_init(&reader);

    /* Register packet consumer for sequence number processing */
    rfc5444_reader_add_packet_consumer(&reader, &_nhdp_packet_consumer, NULL, 0);

    /* Register HELLO message consumer */
    rfc5444_reader_add_message_consumer(&reader, &_nhdp_msg_consumer,
            _nhdp_msg_tlvs, ARRAYSIZE(_nhdp_msg_tlvs));
    rfc5444_reader_add_message_consumer(&reader, &_nhdp_address_consumer,
            _nhdp_addr_tlvs, ARRAYSIZE(_nhdp_addr_tlvs));
}

int nhdp_reader_handle_packet(kernel_pid_t rcvg_if_pid, void *buffer, size_t length)
{
    int result;
    
    mutex_lock(&mtx_packet_handler);

    /* Store PID of interface this packet was received on */
    if_pid = rcvg_if_pid;
    /* Parse packet with reader */
    result = rfc5444_reader_handle_packet(&reader, buffer, length);

    mutex_unlock(&mtx_packet_handler);

    return result;
}

void nhdp_reader_cleanup(void)
{
    cleanup_temp_addr_lists();
    rfc5444_reader_cleanup(&reader);
}


/*------------------------------------------------------------------------------------*/
/*                                Internal functions                                  */
/*------------------------------------------------------------------------------------*/

/**
 * Process metric steps for packet with packet sequence number
 * Called by oonf_api after the whole packet was processed
 */
static enum rfc5444_result _nhdp_pkt_end_cb(struct rfc5444_reader_tlvblock_context *context,
        bool dropped __attribute__((unused)))
{
    if ((originator_link_tuple != NULL) && (context->has_pktseqno)) {
        iib_process_metric_pckt(originator_link_tuple, lt_metric_val, context->pkt_seqno);
    }
    originator_link_tuple = NULL;
    lt_metric_val = NHDP_METRIC_UNKNOWN;
    return RFC5444_OKAY;
}

/**
 * Handle one address and its corresponding TLVs
 * Called by oonf_api for every included address to allow parsing
 */
static enum rfc5444_result
_nhdp_blocktlv_address_cb(struct rfc5444_reader_tlvblock_context *cont)
{
    uint8_t tmp_result;
    uint8_t append_metric_val = 0;
    uint16_t metric_enc;
    nhdp_addr_entry_t *sec_container;
    nhdp_addr_metr_t *metric_container;
    /* Create address list entry to add it later to one of the temp address lists */
    nhdp_addr_entry_t *current_addr = (nhdp_addr_entry_t*) malloc(sizeof(nhdp_addr_entry_t));
    if (!current_addr) {
        /* Insufficient memory */
        return RFC5444_DROP_MESSAGE;
    }

    /* Get NHDP address for the current netaddr */
    current_addr->address = get_nhdp_db_addr(&cont->addr._addr[0], cont->addr._prefix_len);
    if (!current_addr->address) {
        /* Insufficient memory */
        free(current_addr);
        return RFC5444_DROP_MESSAGE;
    }

    /* Check validity of address tlvs */
    if (check_addr_validity(current_addr->address) != RFC5444_OKAY) {
        nhdp_free_addr_entry(current_addr);
        return RFC5444_DROP_MESSAGE;
    }

    /* Handle address and add it to proper temporary list */
    if (_nhdp_addr_tlvs[RFC5444_ADDRTLV_LOCAL_IF].tlv) {
        switch (*_nhdp_addr_tlvs[RFC5444_ADDRTLV_LOCAL_IF].tlv->single_value) {
            case RFC5444_LOCALIF_THIS_IF:
                LL_PREPEND(send_addr_list_head, current_addr);
                /* Local IF marked addresses have to be added to two temp lists */
                sec_container = (nhdp_addr_entry_t*) malloc(sizeof(nhdp_addr_entry_t));
                if (!sec_container) {
                    return RFC5444_DROP_MESSAGE;
                }
                nhdp_increment_addr_usage(current_addr->address);
                sec_container->address = current_addr->address;
                LL_PREPEND(nb_addr_list_head, sec_container);
                break;
            case RFC5444_LOCALIF_OTHER_IF:
                LL_PREPEND(nb_addr_list_head, current_addr);
                break;
            default:
                /* Wrong value, drop message */
                nhdp_free_addr_entry(current_addr);
                return RFC5444_DROP_MESSAGE;
        }
    }
    else if ((tmp_result = lib_is_reg_addr(if_pid, current_addr->address))) {
        /* The address is one of our local addresses (do not add it for processing) */
        if ((!sym) && (tmp_result == 1) && _nhdp_addr_tlvs[RFC5444_ADDRTLV_LINK_STATUS].tlv) {
            /* If address is a local address of the receiving interface, check */
            /* whether we can derive a status for this link (symmetry or lost) */
            switch (*_nhdp_addr_tlvs[RFC5444_ADDRTLV_LINK_STATUS].tlv->single_value) {
                case RFC5444_LINKSTATUS_SYMMETRIC:
                    /* Fall - through */
                case RFC5444_LINKSTATUS_HEARD:
                    sym = 1;
                    break;
                case RFC5444_LINKSTATUS_LOST:
                    lost = 1;
                    break;
                default:
                    /* Wrong value, drop message */
                    nhdp_free_addr_entry(current_addr);
                    return RFC5444_DROP_MESSAGE;
            }
        }

        if (lt_metric_val == NHDP_METRIC_UNKNOWN) {
            if (_nhdp_addr_tlvs[RFC5444_ADDRTLV_LINK_METRIC].tlv) {
                metric_enc = *((uint16_t*)_nhdp_addr_tlvs[RFC5444_ADDRTLV_LINK_METRIC]
                                                          .tlv->single_value);
                if (metric_enc & NHDP_KD_LM_INC) {
                    lt_metric_val = rfc5444_metric_decode(metric_enc);
                }
            }
        }

        /* Address is one of our own addresses, ignore it */
        nhdp_free_addr_entry(current_addr);
    }
    else if (_nhdp_addr_tlvs[RFC5444_ADDRTLV_LINK_STATUS].tlv) {
        switch (*_nhdp_addr_tlvs[RFC5444_ADDRTLV_LINK_STATUS].tlv->single_value) {
            case RFC5444_LINKSTATUS_SYMMETRIC:
                append_metric_val = 1;
                LL_PREPEND(th_sym_addr_list_head, current_addr);
                break;
            case RFC5444_LINKSTATUS_HEARD:
                /* Fall-through */
            case RFC5444_LINKSTATUS_LOST:
                if (_nhdp_addr_tlvs[RFC5444_ADDRTLV_OTHER_NEIGHB].tlv
                        && *_nhdp_addr_tlvs[RFC5444_ADDRTLV_OTHER_NEIGHB].tlv->single_value
                            == RFC5444_OTHERNEIGHB_SYMMETRIC) {
                    /* Symmetric has higher priority */
                    append_metric_val = 1;
                    LL_PREPEND(th_sym_addr_list_head, current_addr);
                }
                else {
                    LL_PREPEND(th_rem_addr_list_head, current_addr);
                }
                break;
            default:
                /* Wrong value, drop message */
                nhdp_free_addr_entry(current_addr);
                return RFC5444_DROP_MESSAGE;
        }
    }
    else if (_nhdp_addr_tlvs[RFC5444_ADDRTLV_OTHER_NEIGHB].tlv) {
        switch (*_nhdp_addr_tlvs[RFC5444_ADDRTLV_OTHER_NEIGHB].tlv->single_value) {
            case RFC5444_OTHERNEIGHB_SYMMETRIC:
                append_metric_val = 1;
                LL_PREPEND(th_sym_addr_list_head, current_addr);
                break;
            case RFC5444_OTHERNEIGHB_LOST:
                LL_PREPEND(th_rem_addr_list_head, current_addr);
                break;
            default:
                /* Wrong value, drop message */
                nhdp_free_addr_entry(current_addr);
                return RFC5444_DROP_MESSAGE;
        }
    }
    else {
        /* Addresses without expected TLV are ignored */
        nhdp_free_addr_entry(current_addr);
        return RFC5444_DROP_ADDRESS;
    }

    if (append_metric_val) {
        if (_nhdp_addr_tlvs[RFC5444_ADDRTLV_LINK_METRIC].tlv) {
            metric_enc = *((uint16_t*)_nhdp_addr_tlvs[RFC5444_ADDRTLV_LINK_METRIC]
                                                      .tlv->single_value);
            if (metric_enc & (NHDP_KD_LM_INC | NHDP_KD_NM_INC)) {
                metric_container = (nhdp_addr_metr_t*) malloc(sizeof(nhdp_addr_metr_t));
                if (metric_container) {
                    metric_container->address = current_addr;
                    metric_container->metric_val = rfc5444_metric_decode(metric_enc);
                    LL_PREPEND(addr_metric_head, metric_container);
                }
            }
        }
    }

    return RFC5444_OKAY;
}

/**
 * Handle message TLVs of received HELLO
 * Called by oonf_api to allow message TLV parsing
 */
static enum rfc5444_result
_nhdp_blocktlv_msg_cb(struct rfc5444_reader_tlvblock_context *cont)
{
    /* Check whether specified message TLVs are correctly included */
    if (check_msg_validity(cont) != RFC5444_OKAY) {
        return RFC5444_DROP_MESSAGE;
    }

    /* Validity time must be included as message tlv */
    val_time = rfc5444_timetlv_decode(
            *_nhdp_msg_tlvs[RFC5444_MSGTLV_VALIDITY_TIME].tlv->single_value);
    /* Interval time is not mandatory as message tlv */
    if (_nhdp_msg_tlvs[RFC5444_MSGTLV_INTERVAL_TIME].tlv) {
        int_time = rfc5444_timetlv_decode(
                *_nhdp_msg_tlvs[RFC5444_MSGTLV_INTERVAL_TIME].tlv->single_value);
    }

    return RFC5444_OKAY;
}

/**
 * Process received addresses and clean up temporary stuff
 * Called by oonf_api after message was parsed
 */
static enum rfc5444_result
_nhdp_msg_end_cb(struct rfc5444_reader_tlvblock_context *cont __attribute__((unused)),
        bool dropped)
{
    nhdp_addr_metr_t *metr_elt, *metr_tmp;

    if (!dropped) {
        /* Only process the received addresses if message was valid */
        process_temp_tables();
    }

    /* Clean all temporary stuff */
    val_time = 0ULL;
    int_time = 0ULL;
    sym = 0;
    lost = 0;
    cleanup_temp_addr_lists();

    /* Cleanup metric container */
    LL_FOREACH_SAFE(addr_metric_head, metr_elt, metr_tmp) {
        free(metr_elt);
    }
    addr_metric_head = NULL;

    if (dropped) {
        return RFC5444_DROP_MESSAGE;
    }
    return RFC5444_OKAY;
}

/**
 * Check validity of HELLO message header and message TLVs
 */
static enum rfc5444_result check_msg_validity(struct rfc5444_reader_tlvblock_context *cont)
{
    if (cont->has_hoplimit && cont->hoplimit != 1) {
        /* Hop Limit other than 1 */
        return RFC5444_DROP_MESSAGE;
    }

    if (cont->has_hopcount && cont->hopcount != 0) {
        /* Hop Count other than zero */
        return RFC5444_DROP_MESSAGE;
    }

    if (!(_nhdp_msg_tlvs[RFC5444_MSGTLV_VALIDITY_TIME].tlv)) {
        /* No validity time tlv */
        return RFC5444_DROP_MESSAGE;
    }
    else if (_nhdp_msg_tlvs[RFC5444_MSGTLV_VALIDITY_TIME].tlv->next_entry) {
        /* Multiple validity time tlvs */
        return RFC5444_DROP_MESSAGE;
    }

    if (_nhdp_msg_tlvs[RFC5444_MSGTLV_INTERVAL_TIME].tlv
            && _nhdp_msg_tlvs[RFC5444_MSGTLV_INTERVAL_TIME].tlv->next_entry) {
        /* Multiple interval time tlvs */
        return RFC5444_DROP_MESSAGE;
    }

    return RFC5444_OKAY;
}

/**
 * Check validity of address block TLVs
 */
static enum rfc5444_result check_addr_validity(nhdp_addr_t *addr)
{
    if (_nhdp_addr_tlvs[RFC5444_ADDRTLV_LOCAL_IF].tlv) {
        if (_nhdp_addr_tlvs[RFC5444_ADDRTLV_LINK_STATUS].tlv
                || _nhdp_addr_tlvs[RFC5444_ADDRTLV_OTHER_NEIGHB].tlv) {
            /* Conflicting tlv types for the address */
            return RFC5444_DROP_MESSAGE;
        }
        else if (_nhdp_addr_tlvs[RFC5444_ADDRTLV_LOCAL_IF].tlv->next_entry) {
            /* Multiple tlvs of the same type are not allowed */
            return RFC5444_DROP_MESSAGE;
        }
        else if (lib_is_reg_addr(if_pid, addr)) {
            /* Address of one of neighbor's IFs equals one of ours */
            return RFC5444_DROP_MESSAGE;
        }
    }
    else if (_nhdp_addr_tlvs[RFC5444_ADDRTLV_LINK_STATUS].tlv
            && _nhdp_addr_tlvs[RFC5444_ADDRTLV_LINK_STATUS].tlv->next_entry) {
        /* Multiple tlvs of the same type are not allowed */
        return RFC5444_DROP_MESSAGE;
    }
    else if (_nhdp_addr_tlvs[RFC5444_ADDRTLV_OTHER_NEIGHB].tlv
            && _nhdp_addr_tlvs[RFC5444_ADDRTLV_OTHER_NEIGHB].tlv->next_entry) {
        /* Multiple tlvs of the same type are not allowed */
        return RFC5444_DROP_MESSAGE;
    }

    return RFC5444_OKAY;
}

/**
 * Get a new or existing NHDP address entry from the centralized address storage
 * for the given address data
 */
static nhdp_addr_t* get_nhdp_db_addr(uint8_t *addr, uint8_t prefix)
{
    switch (prefix) {
        case 8:
            return nhdp_addr_db_get_address(addr, 1, AF_CC110X);
        case 32:
            return nhdp_addr_db_get_address(addr, 4, AF_INET);
        default:
            if (prefix < 32) {
                return nhdp_addr_db_get_address(addr, 4, AF_INET);
            }
            else {
                return nhdp_addr_db_get_address(addr, 16, AF_INET6);
            }
    }
}

/**
 * Process address lists from the HELLO msg in the information bases
 */
static void process_temp_tables(void)
{
    nib_entry_t *nib_elt;
    iib_link_set_entry_t *ls_elt;
    timex_t now;

    vtimer_now(&now);
    iib_update_lt_status(&now);

    nib_elt = nib_process_hello(nb_addr_list_head, &rem_addr_list_head);

    if (nib_elt) {
        ls_elt = iib_process_hello_lt(if_pid, nib_elt, send_addr_list_head,
                rem_addr_list_head, val_time, sym, lost);

        originator_link_tuple = ls_elt;

        if (ls_elt) {
            /* First process changes for two hop tuples */
            iib_process_hello_th(if_pid, val_time, ls_elt,
                    th_sym_addr_list_head, th_rem_addr_list_head, addr_metric_head);
            /* Afterwards process necessary metric mechanisms */
            iib_process_metric_msg(ls_elt, int_time != 0 ? int_time : val_time);
        }
    }
}

/**
 * Free all allocated space for the temporary address lists
 */
static void cleanup_temp_addr_lists(void)
{
    nhdp_free_addr_list(send_addr_list_head);
    nhdp_free_addr_list(nb_addr_list_head);
    nhdp_free_addr_list(th_sym_addr_list_head);
    nhdp_free_addr_list(th_rem_addr_list_head);
    nhdp_free_addr_list(rem_addr_list_head);
    send_addr_list_head = NULL;
    nb_addr_list_head = NULL;
    th_sym_addr_list_head = NULL;
    th_rem_addr_list_head = NULL;
    rem_addr_list_head = NULL;
}