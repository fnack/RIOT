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
 * @brief       Centralized address storage implementation for NHDP
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 *
 * @}
 */

#include "mutex.h"
#include "utlist.h"

#include "nhdp.h"
#include "nhdp_address.h"

/* Internal variables */
static mutex_t mtx_addr_access = MUTEX_INIT;
static nhdp_addr_t *nhdp_addr_db_head = NULL;


/*---------------------------------------------------------------------------*
 *                      Centralized Address Storage API                      *
 *---------------------------------------------------------------------------*/

nhdp_addr_t* nhdp_addr_db_get_address(uint8_t *addr, size_t addr_size, uint8_t addr_type)
{
    nhdp_addr_t *addr_elt;

    mutex_lock(&mtx_addr_access);

    LL_FOREACH(nhdp_addr_db_head, addr_elt) {
        if ((addr_elt->addr_size == addr_size) && (addr_elt->addr_type == addr_type)) {
            if (memcmp(addr_elt->addr, addr, addr_size) == 0) {
                /* Found a matching entry */
                break;
            }
        }
    }

    if (!addr_elt) {
        /* No matching entry, create a new one */
        addr_elt = (nhdp_addr_t*) malloc(sizeof(nhdp_addr_t));
        if (!addr_elt) {
            /* Insufficient memory */
            return NULL;
        }

        /* Allocate space for the address */
        addr_elt->addr = (uint8_t*) malloc(addr_size * sizeof(uint8_t));
        if (!addr_elt->addr) {
            /* Insufficient memory */
            free(addr_elt);
            return NULL;
        }

        memcpy(addr_elt->addr, addr, addr_size);
        addr_elt->addr_size = addr_size;
        addr_elt->addr_type = addr_type;
        addr_elt->usg_count = 0;
        addr_elt->in_tmp_table = NHDP_ADDR_TMP_NONE;
        LL_PREPEND(nhdp_addr_db_head, addr_elt);
    }

    nhdp_increment_addr_usage(addr_elt);

    mutex_unlock(&mtx_addr_access);
    
    return addr_elt;
}

void nhdp_increment_addr_usage(nhdp_addr_t *addr)
{
    addr->usg_count += 1;
}

void nhdp_decrement_addr_usage(nhdp_addr_t *addr)
{
    mutex_lock(&mtx_addr_access);

    /* Decrement usage count and delete address if no longer used */
    if (addr) {
        addr->usg_count -= 1;
        if (addr->usg_count <= 0) {
            /* Free address space if address is no longer used */
            LL_DELETE(nhdp_addr_db_head, addr);
            free(addr->addr);
            free(addr);
        }
    }

    mutex_unlock(&mtx_addr_access);
}

void nhdp_free_addr_list(nhdp_addr_entry_t *list_head)
{
    nhdp_addr_entry_t *list_elt, *list_tmp;

    LL_FOREACH_SAFE(list_head, list_elt, list_tmp) {
        nhdp_free_addr_entry(list_elt);
    }
}

void nhdp_free_addr_entry(nhdp_addr_entry_t *addr_entry)
{
    nhdp_decrement_addr_usage(addr_entry->address);
    free(addr_entry);
}

nhdp_addr_entry_t* nhdp_generate_new_addr_list(nhdp_addr_entry_t *orig_list)
{
    nhdp_addr_entry_t *new_list_head, *new_entry, *addr_elt;

    new_list_head = NULL;
    LL_FOREACH(orig_list, addr_elt) {
        new_entry = (nhdp_addr_entry_t*) malloc(sizeof(nhdp_addr_entry_t));
        if (!new_entry) {
            /* Insufficient memory, free all previously allocated memory */
            nhdp_free_addr_list(new_list_head);
            return NULL;
        }
        new_entry->address = addr_elt->address;
        nhdp_increment_addr_usage(addr_elt->address);
        LL_PREPEND(new_list_head, new_entry);
    }

    return new_list_head;
}

void nhdp_reset_addresses_tmp_usg(void)
{
    nhdp_addr_t *addr_elt;

    LL_FOREACH(nhdp_addr_db_head, addr_elt) {
        addr_elt->in_tmp_table = NHDP_ADDR_TMP_NONE;
    }
}