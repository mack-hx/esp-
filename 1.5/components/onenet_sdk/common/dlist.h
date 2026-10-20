/**
 * Copyright (c), 2012~2018 iot.10086.cn All Rights Reserved
 * @file         dlist.h
 * @brief         Double direction linked list
 */

#ifndef __DLIST_H__
#define __DLIST_H__

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* External Definition（Constant and Macro )                                 */
/*****************************************************************************/

/*****************************************************************************/
/* External Structures, Enum and Typedefs                                    */
/*****************************************************************************/
/**
 * @brief Doubly linked list node definition，You need to include this node in a custom structure，Implement the linked list operation of custom structures.
 *
 */
struct dlist_node_t
{
    /** Point to the previous data node，When the current node is the first node, it isNULL*/
    struct dlist_node_t *prev;
    /** Point to the next data node，When the current node is the last node, it isNULL*/
    struct dlist_node_t *next;
};

/**
 * @brief Doubly linked list header definition.
 *
 */
struct dlist_head_t
{
    /** Points to the first node of the linked list，When the linked list is emptyNULL*/
    struct dlist_node_t *head;
    /** Points to the last node of the linked list，When the linked list is emptyNULL*/
    struct dlist_node_t *tail;
    /** Number of nodes in the current linked list*/
    uint32_t             cnt;
};
/*****************************************************************************/
/* External Function Prototypes                                              */
/*****************************************************************************/
/**
 * Initialize a doubly linked list header
 * @param head Doubly linked list header structure
 * @retval 0 Succeed
 * @retval -1 Failed
 */
int32_t dlist_init(struct dlist_head_t *head);

/**
 * Query whether the specified linked list is empty
 * @param head Doubly linked list header structure
 * @retval -1 - Error
 * @retval  0 - There are nodes in the linked list
 * @retval  1 - There are no nodes in the linked list
 */
int32_t dlist_is_empty(struct dlist_head_t *head);

/**
 * Insert a new node after the specified node
 * @param head Doubly linked list header
 * @param current Specify where nodes need to be inserted
 * @param node Specify the new nodes to be inserted
 * @retval   0 - Insertion successful
 * @retval  -1 - Insertion failed
 */
int32_t dlist_insert_after(struct dlist_head_t *head, struct dlist_node_t *current, struct dlist_node_t *node);

/**
 * Insert a new node in the linked list header
 * @param head Doubly linked list header
 * @param node New nodes that need to be inserted
 * @retval   0 - Insertion successful
 * @retval  -1 - Insertion failed
 */
int32_t dlist_insert_head(struct dlist_head_t *head, struct dlist_node_t *node);

/**
 * Insert a new node at the end of the linked list
 * @param head Doubly linked list header
 * @param node New nodes that need to be inserted
 * @retval   0 - Insertion successful
 * @retval  -1 - Insertion failed
 */
int32_t dlist_insert_tail(struct dlist_head_t *head, struct dlist_node_t *node);

/**
 * Delete the specified node
 * @param head Doubly linked list header
 * @param node Nodes that need to be deleted
 * @retval   0 - Delete successfully
 * @retval  -1 - Delete failed
 */
int32_t dlist_remove_node(struct dlist_head_t *head, struct dlist_node_t *node);

/**
 * Get the next node of the current node
 * @param current Specify the current node
 * @return Non-NULL - Next node pointer
 */
struct dlist_node_t *dlist_get_next(struct dlist_node_t *current);

/**
 * Get the header node of a doubly linked list
 * @param head Doubly linked list header
 * @return Non-NULL - Head node pointer
 */
struct dlist_node_t *dlist_get_head(struct dlist_head_t *head);

/**
 * Get the tail node of a doubly linked list
 * @param head Doubly linked list header
 * @return Non-NULL - Tail node pointer
 */
struct dlist_node_t *dlist_get_tail(struct dlist_head_t *head);

/**
 * @brief Linked list polling handler function definition，Onlydlist_eachUse of
 * @param node Current node of polling process
 * @param arg dlist_eachParameters passed inarg
 * @retval    0 - Continue Polling Processing
 * @retval  Non-0 - Terminate polling.
 *
 */
typedef int (*dlist_cb)(struct dlist_node_t * /*node*/, void * /*arg*/);

/**
 * Poll all nodes in the linked list and call the same function，When the function returns not0Exit polling when
 * @param head Doubly linked list header
 * @param handle Function callbacks that need to be called（It is safe to delete the current node within the callback function）
 * @param arg Parameters for function execution
 * @return Non-NULL - The node at which you exit the polling
 */
struct dlist_node_t *dlist_each(struct dlist_head_t *head, dlist_cb handle, void *arg);

#ifdef __cplusplus
}
#endif

#endif /* __DLIST_H__ */
