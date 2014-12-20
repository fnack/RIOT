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
 * @brief       Implementation of NHDP's core functionality
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 *
 * @}
 */

#include "msg.h"
#include "netapi.h"
#include "thread.h"
#include "utlist.h"

#include "rfc5444/rfc5444_writer.h"

#include "lib_table.h"
#include "iib_table.h"
#include "nib_table.h"
#include "nhdp.h"
#include "nhdp_address.h"
#include "nhdp_writer.h"
#include "nhdp_reader.h"

char nhdp_stack[NHDP_STACK_SIZE];

/* Internal variables */
static kernel_pid_t nhdp_pid = KERNEL_PID_UNDEF;
static nhdp_if_entry_t *nhdp_if_entry_head = NULL;

static struct autobuf _hexbuf;
char buf[256];
size_t leng;

/* Internal function prototypes */
static void *_nhdp_runner(void *arg __attribute__((unused)));
static void write_packet(struct rfc5444_writer *wr __attribute__((unused)),
                         struct rfc5444_writer_target *iface __attribute__((unused)),
                         void *buffer, size_t length);
static int reg_nhdp_as_recipient(kernel_pid_t if_pid);


/*---------------------------------------------------------------------------*
 *                            NHDP Core API                                  *
 *---------------------------------------------------------------------------*/

void nhdp_init(void)
{
    if (nhdp_pid != KERNEL_PID_UNDEF) {
        /* do not initialize twice */
        return;
    }

    /* Initialize reader and writer */
    nhdp_writer_init();
    nhdp_reader_init();
}

kernel_pid_t nhdp_start(void)
{
    /* Start the NHDP thread */
    nhdp_pid = thread_create(nhdp_stack, sizeof(nhdp_stack), PRIORITY_MAIN - 1,
            CREATE_STACKTEST, _nhdp_runner, NULL, "NHDP");

    return nhdp_pid;
}

int nhdp_register_if_default(kernel_pid_t if_pid, uint8_t *addr, size_t addr_size,
        uint8_t addr_type, uint16_t max_pl_size)
{
    return nhdp_register_if(if_pid, addr, addr_size, addr_type, max_pl_size,
            NHDP_DEFAULT_HELLO_INT_MS, NHDP_DEFAULT_HOLD_TIME_MS);
}

int nhdp_register_if(kernel_pid_t if_pid, uint8_t *addr, size_t addr_size,uint8_t addr_type,
        uint16_t max_pl_size, uint16_t hello_int_ms, uint16_t val_time_ms)
{
    nhdp_if_entry_t *if_entry;
    nhdp_addr_t *nhdp_addr;
    msg_t signal_msg;

    if_entry = (nhdp_if_entry_t*) malloc(sizeof(nhdp_if_entry_t));
    if (!if_entry) {
        /* Insufficient memory */
        return -1;
    }

    /* Create an interface writer targer for the nhdp_writer */
    if_entry->wr_target = (struct rfc5444_writer_target*)
            calloc(1, sizeof(struct rfc5444_writer_target));
    if (!if_entry->wr_target) {
        /* Insufficient memory */
        free(if_entry);
        return -1;
    }
    if_entry->wr_target->packet_buffer = (uint8_t*) calloc(max_pl_size, sizeof(uint8_t));
    if (!if_entry->wr_target->packet_buffer) {
        /* Insufficient memory */
        free(if_entry->wr_target);
        free(if_entry);
        return -1;
    }
    if_entry->wr_target->packet_size = max_pl_size;
    if_entry->wr_target->sendPacket = write_packet;

    /* Get NHDP address entry for the given address */
    nhdp_addr = nhdp_addr_db_get_address(addr, addr_size, addr_type);
    if (!nhdp_addr) {
        /* Insufficient memory */
        free(if_entry->wr_target->packet_buffer);
        free(if_entry->wr_target);
        free(if_entry);
        return -1;
    }

    /* Set Interface's PID */
    if_entry->if_pid = if_pid;
    /* Set HELLO_INTERVAL and H_HOLD_TIME (validity time) */
    if_entry->hello_interval.seconds = 0;
    if_entry->hello_interval.microseconds = MS_IN_USEC * hello_int_ms;
    if_entry->validity_time.seconds = 0;
    if_entry->validity_time.microseconds = MS_IN_USEC * val_time_ms;
    timex_normalize(&if_entry->hello_interval);
    timex_normalize(&if_entry->validity_time);

    /* Add the interface to the LIB */
    if (lib_add_if_addr(if_entry->if_pid, nhdp_addr) != 0) {
        free(if_entry->wr_target->packet_buffer);
        free(if_entry->wr_target);
        free(if_entry);
        nhdp_decrement_addr_usage(nhdp_addr);
        return -1;
    }

    /* Create new IIB for the interface */
    if (iib_register_if(if_pid) != 0) {
        /* TODO: Cleanup lib entry */
        free(if_entry->wr_target->packet_buffer);
        free(if_entry->wr_target);
        free(if_entry);
        nhdp_decrement_addr_usage(nhdp_addr);
        return -1;
    }

    /* Everything went well */
    nhdp_decrement_addr_usage(nhdp_addr);
    reg_nhdp_as_recipient(if_entry->if_pid);
    nhdp_writer_register_if(if_entry->wr_target);
    LL_PREPEND(nhdp_if_entry_head, if_entry);

    /* Start sending periodic HELLO */
    signal_msg.type = MSG_TIMER;
    signal_msg.content.ptr = (char *) if_entry;
    /* TODO: msg_send or msg_try_send? */
    msg_try_send(&signal_msg, nhdp_pid);

    return 0;
}

int nhdp_register_non_manet_if(kernel_pid_t if_pid, uint8_t *addr, size_t addr_size,
        uint8_t addr_type)
{
    return nhdp_add_address(if_pid, addr, addr_size, addr_type);
}

int nhdp_add_address(kernel_pid_t if_pid, uint8_t *addr, size_t addr_size, uint8_t addr_type)
{
    int result;

    /* Get NHDP address entry for the given address */
    nhdp_addr_t *nhdp_addr = nhdp_addr_db_get_address(addr, addr_size, addr_type);
    if (!nhdp_addr) {
        /* Insufficient memory */
        return -1;
    }

    result = lib_add_if_addr(if_pid, nhdp_addr);
    nhdp_decrement_addr_usage(nhdp_addr);

    return result;
}

/*------------------------------------------------------------------------------------*/
/*                                Internal functions                                  */
/*------------------------------------------------------------------------------------*/

/**
 * Function executed by NHDP thread receiving messages in an endless loop
 */
static void *_nhdp_runner(void *arg)
{
    nhdp_if_entry_t *if_entry;
    netapi_cmd_t *cmd;
    netapi_rcv_pkt_t *rcvd_pkt;
    netapi_ack_t *ack;
    msg_t msg_rcvd, msg_ack, msg_queue[NHDP_MSG_QUEUE_SIZE];

    (void)arg;
    msg_init_queue(msg_queue, NHDP_MSG_QUEUE_SIZE);

    while (1) {
        msg_receive(&msg_rcvd);

        switch (msg_rcvd.type) {
            case MSG_TIMER:
                if_entry = (nhdp_if_entry_t*) msg_rcvd.content.ptr;

                nhdp_writer_send_hello(if_entry);

                /* TODO: Add jitter */
                
                /* Schedule next sending */
                vtimer_set_msg(&if_entry->if_timer, if_entry->hello_interval,
                        thread_getpid(), MSG_TIMER, (void*) if_entry);
                break;
            case NETAPI_MSG_TYPE:
                /* Received msg from lower layer */
                cmd = (netapi_cmd_t*) (msg_rcvd.content.ptr);
                ack = cmd->ack;
                ack->result = -ENOTSUP;
                ack->orig = cmd->type;
                ack->type = NETAPI_CMD_ACK;
                msg_ack.content.ptr = (char *) ack;
                msg_ack.type = NETAPI_MSG_TYPE;

                if (cmd->type == NETAPI_CMD_RCV) {
                    /* Received a packet from lower layer */
                    rcvd_pkt = (netapi_rcv_pkt_t*) cmd;
                    nhdp_reader_handle_packet(msg_rcvd.sender_pid,
                            rcvd_pkt->data, rcvd_pkt->data_len);
                    ack->result = (int) rcvd_pkt->data_len;
                }
                msg_reply(&msg_rcvd, &msg_ack);
                break;
            default:
                break;
        }
    }
    return 0;
}

/**
 * Send packet over a registered interface using netapi
 * Called by oonf_api to send packet (hand it to lower layer)
 */
static void write_packet(struct rfc5444_writer *wr __attribute__((unused)),
                         struct rfc5444_writer_target *iface __attribute__((unused)),
                         void *buffer, size_t length)
{
    nhdp_if_entry_t *if_elt;
    msg_t msg_pkt, msg_ack;
    netapi_snd_pkt_t packet;
    netapi_ack_t ack_mem;
    /* TODO: Introduce target address for interfaces */
    uint8_t address = 0;

    memcpy(buf, buffer, length);
    leng = length;

    LL_FOREACH(nhdp_if_entry_head, if_elt) {
        if (if_elt->wr_target == iface) {
            /* Use netapi to forward packet to lower layer */
            packet.type = NETAPI_CMD_SND;
            packet.ulh = NULL;
            packet.ack = &ack_mem;

            /* TODO: Introduce target address for interfaces */
            packet.dest = &address;
            packet.dest_len = 1;

            packet.data = buffer;
            packet.data_len = length;
            msg_pkt.type = NETAPI_MSG_TYPE;
            msg_pkt.content.ptr = (char *)(&packet);

            msg_send_receive(&msg_pkt, &msg_ack, if_elt->if_pid);
            break;
        }
    }
}

/**
 * Send netapi message to register NHDP as upper layer recipient for the given interface
 */
static int reg_nhdp_as_recipient(kernel_pid_t if_pid)
{
    msg_t reg_msg, ack_msg;
    netapi_reg_t reg_cmd;
    netapi_ack_t reg_ack;

    /* Register NHDP as recipient for lower layer using netapi */
    reg_msg.type = NETAPI_MSG_TYPE;
    reg_cmd.reg_pid = nhdp_pid;
    reg_cmd.ack = &reg_ack;
    reg_cmd.type = NETAPI_CMD_REG;
    reg_msg.content.ptr = (char*) (&reg_cmd);

    msg_send_receive(&reg_msg, &ack_msg, if_pid);
    if ((reg_ack.type == NETAPI_CMD_ACK) && (reg_ack.orig = NETAPI_CMD_REG)) {
        return 0;
    }
    return -1;
}

void print_packet(void)
{
    /* Generate hexdump of packet */
    abuf_hexdump(&_hexbuf, "\t", buf, leng);
    rfc5444_print_direct(&_hexbuf, buf, leng);
    /* Print hexdump */
    printf("Packet size: %" PRIu16 "\n", leng);
    printf("%s", abuf_getptr(&_hexbuf));
    abuf_free(&_hexbuf);
}
