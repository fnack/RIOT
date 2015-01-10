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
#include "mutex.h"

#include "rfc5444/rfc5444_writer.h"

#include "lib_table.h"
#include "iib_table.h"
#include "nib_table.h"
#include "nhdp.h"
#include "nhdp_address.h"
#include "nhdp_writer.h"
#include "nhdp_reader.h"

char nhdp_stack[NHDP_STACK_SIZE];
char nhdp_rcv_stack[NHDP_STACK_SIZE];

/* Internal variables */
static kernel_pid_t nhdp_pid = KERNEL_PID_UNDEF;
static nhdp_if_entry_t *nhdp_if_entry_head = NULL;

#if (NHDP_METRIC != NHDP_LMT_HOP_COUNT)
static vtimer_t metric_timer;
static timex_t metric_interval;
#endif

static kernel_pid_t nhdp_rcv_pid = KERNEL_PID_UNDEF;
static kernel_pid_t helper_pid = KERNEL_PID_UNDEF;
static ipv6_addr_t _v6_addr_local;
static mutex_t send_rcv_mutex = MUTEX_INIT;
static sockaddr6_t sa_bcast;
static int sock_rcv;

static struct autobuf _hexbuf;
char buf[256];
size_t leng;

/* Internal function prototypes */
static void *_nhdp_runner(void *arg __attribute__((unused)));
static void *_nhdp_receiver_thread(void *arg __attribute__((unused)));
static void write_packet(struct rfc5444_writer *wr __attribute__((unused)),
                         struct rfc5444_writer_target *iface __attribute__((unused)),
                         void *buffer, size_t length);
static void add_seqno(struct rfc5444_writer *writer, struct rfc5444_writer_target *rfc5444_target);


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
    /* init multicast address: set to to a link-local all nodes multicast address */
    sa_bcast.sin6_family = AF_INET6;
    sa_bcast.sin6_port = HTONS(269);
    ipv6_addr_set_all_nodes_addr(&sa_bcast.sin6_addr);
    /* get best IP for sending */
    ipv6_net_if_get_best_src_addr(&_v6_addr_local, &sa_bcast.sin6_addr);

    sock_rcv = socket_base_socket(PF_INET6, SOCK_DGRAM, IPPROTO_UDP);

    /* Start the NHDP thread */
    nhdp_pid = thread_create(nhdp_stack, NHDP_STACK_SIZE, PRIORITY_MAIN - 1,
            CREATE_STACKTEST, _nhdp_runner, NULL, "NHDP");

#if (NHDP_METRIC != NHDP_LMT_HOP_COUNT)
    if (nhdp_pid != KERNEL_PID_UNDEF) {
        metric_interval = timex_from_uint64(DAT_REFRESH_INTERVAL * SEC_IN_USEC);
        vtimer_set_msg(&metric_timer, metric_interval, nhdp_pid, NHDP_METRIC_TIMER, NULL);
    }
#endif

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
    if_entry->wr_target->addPacketHeader = add_seqno;

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
    if_entry->hello_interval.microseconds = MS_IN_US * hello_int_ms;
    if_entry->validity_time.seconds = 0;
    if_entry->validity_time.microseconds = MS_IN_US * val_time_ms;
    timex_normalize(&if_entry->hello_interval);
    timex_normalize(&if_entry->validity_time);

    if_entry->seq_no = 0;

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
    nhdp_writer_register_if(if_entry->wr_target);
    LL_PREPEND(nhdp_if_entry_head, if_entry);

    nhdp_rcv_pid = thread_create(nhdp_rcv_stack, sizeof(nhdp_rcv_stack), PRIORITY_MAIN - 1,
            CREATE_STACKTEST, _nhdp_receiver_thread, NULL, "nhdp_rcv_thread");

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
                mutex_lock(&send_rcv_mutex);
                if_entry = (nhdp_if_entry_t*) msg_rcvd.content.ptr;

                nhdp_writer_send_hello(if_entry);

                /* TODO: Add jitter */
                
                /* Schedule next sending */
                vtimer_set_msg(&if_entry->if_timer, if_entry->hello_interval,
                        thread_getpid(), MSG_TIMER, (void*) if_entry);
                mutex_unlock(&send_rcv_mutex);
                break;
            case NETAPI_MSG_TYPE:
                /* Received msg from lower layer */
                cmd = (netapi_cmd_t*) (msg_rcvd.content.ptr);
                ack = cmd->ack;
                ack->result = -ENOTSUP;
                ack->orig = cmd->type;
                msg_ack.content.ptr = (char *) ack;

                if (cmd->type == NETAPI_CMD_RCV) {
                    /* Received a packet from lower layer */
                    rcvd_pkt = (netapi_rcv_pkt_t*) cmd;
                    nhdp_reader_handle_packet(msg_rcvd.sender_pid,
                            rcvd_pkt->data, rcvd_pkt->data_len);
                    ack->result = (int) rcvd_pkt->data_len;
                }
                msg_reply(&msg_rcvd, &msg_ack);
                break;
            case NHDP_METRIC_TIMER:
                mutex_lock(&send_rcv_mutex);
#if (NHDP_METRIC == NHDP_LMT_DAT)
                /* Process necessary metric computations */
                iib_process_dat_refresh();
                /* Schedule next sending */
                vtimer_set_msg(&metric_timer, metric_interval,
                        thread_getpid(), NHDP_METRIC_TIMER, NULL);
#endif
#if (NHDP_METRIC == NHDP_LMT_ETX)
                /* Process necessary metric computations */
                iib_process_etx_refresh();
                /* Schedule next sending */
                vtimer_set_msg(&metric_timer, metric_interval,
                        thread_getpid(), NHDP_METRIC_TIMER, NULL);
#endif
                mutex_unlock(&send_rcv_mutex);
                break;
            default:
                break;
        }
    }
    return 0;
}

/* Receive HELLOs and handle them */
static void *_nhdp_receiver_thread(void *arg __attribute__((unused)))
{
    uint32_t fromlen;
    char nhdp_rcv_buf[128];
    msg_t msg_q[NHDP_MSG_QUEUE_SIZE];

    msg_init_queue(msg_q, NHDP_MSG_QUEUE_SIZE);

    sockaddr6_t sa_rcv = { .sin6_family = AF_INET6,
                           .sin6_port = HTONS(269) // MANET_PORT
                         };

    if (-1 == socket_base_bind(sock_rcv, &sa_rcv, sizeof(sa_rcv))) {
        socket_base_close(sock_rcv);
    }

    while (1) {
        int32_t rcv_size = socket_base_recvfrom(sock_rcv, (void *)nhdp_rcv_buf, 128, 0,
                                        &sa_rcv, &fromlen);

        if (rcv_size > 0) {
            mutex_lock(&send_rcv_mutex);
            nhdp_reader_handle_packet(helper_pid, (void *) nhdp_rcv_buf, rcv_size);
            mutex_unlock(&send_rcv_mutex);
        }
    }

    socket_base_close(sock_rcv);

    return NULL;
}

/**
 * Send packet over a registered interface using netapi
 * Called by oonf_api to send packet (hand it to lower layer)
 */
static void write_packet(struct rfc5444_writer *wr __attribute__((unused)),
                         struct rfc5444_writer_target *iface __attribute__((unused)),
                         void *buffer, size_t length)
{
    memcpy(buf, buffer, length);
    leng = length;
    socket_base_sendto(sock_rcv, buffer, length, 0, &sa_bcast, sizeof(sa_bcast));
}

static void add_seqno(struct rfc5444_writer *writer, struct rfc5444_writer_target *rfc5444_target)
{
    nhdp_if_entry_t *if_entry;
    LL_FOREACH(nhdp_if_entry_head, if_entry) {
        if (if_entry->wr_target == rfc5444_target) {
            rfc5444_writer_set_pkt_header(writer, rfc5444_target, true);
            rfc5444_writer_set_pkt_seqno(writer, rfc5444_target, ++if_entry->seq_no);
            return;
        }
    }
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
