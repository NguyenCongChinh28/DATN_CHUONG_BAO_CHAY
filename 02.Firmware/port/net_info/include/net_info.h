#ifndef NETWORK_INFO_H
#define NETWORK_INFO_H

#include <stdint.h>
#include <stdbool.h>

#define NET_INFO_MAX_NODE_ALLOWED                         16
#define NET_INFO_ID_INVALID                              (-1)

typedef struct
{
    uint8_t mac[6];
    int8_t id;      // -1 mean invalid
} __attribute__ ((packed)) net_info_node_id_t;

typedef struct
{
    uint16_t network_id;
    net_info_node_id_t node_id[NET_INFO_MAX_NODE_ALLOWED];
} net_info_t;

/**
 * @brief           Load network information in flash
 * @param[out]      net_info Network information
 * @retval          TRUE : Network information stored
 *                  FALSE : No network information
 */
bool net_info_load(net_info_t *net_info);

/**
 * @brief           Load network information in flash
 * @param[in]      net_info Network information
 * @retval          TRUE : Network information stored
 *                  FALSE : No network information
 */
bool net_info_store_default(net_info_t *net_info);

/**
 * @brief           Insert node into network storage
 * @param[in]       net_info Network information to insert
 * @param[in]       mac 6 bytes node mac address
 * @param[out]      offer_id Assigned node id
 * @retval          TRUE : Operation success
 *                  FALSE : Operation failed
 */
bool net_info_insert_node(net_info_t *net_info, uint8_t *mac, uint8_t *offer_id);

/**
 * @brief           Override node in network storage
 * @param[in]       net_info Network information to insert
 * @param[in]       mac 6 bytes node mac address
 * @param[out]      offer_id Assigned node id
 * @retval          TRUE : Operation success
 *                  FALSE : Operation failed
 */
bool net_info_override_node(net_info_t *net_info, uint8_t *mac, uint8_t offer_id);

/**
 * @brief           Get node index in network storage
 * @param[in]       net_info Network information to search
 * @param[in]       mac 6 bytes node mac address
 * @retval          Node index, -1 on error
 */
int8_t net_info_get_node_id(net_info_t *net_info, uint8_t *mac);


/**
 * @brief           Store current network information
 * @param[in]       net_info Network information
 * @retval          TRUE : Operation success
 *                  FALSE : Operation failed
 */
bool net_info_store_current_info(net_info_t *net_info);

/**
 * @brief           Check node id exist in memory
 * @param[in]       net_info Network information
 * @param[in]       node_id Node id
 * @retval          TRUE : Valid node id
 *                  FALSE : Invalid node id
 */
bool net_info_node_id_exist(net_info_t *net_info, int8_t node_id);

/**
 * @brief           Remove node
 * @param[in]       net_info Network information
 * @param[in]       mac Node mac addr
 */
void net_info_remove_node(net_info_t *net_info, uint8_t *mac);

/**
 * @brief           Get number of device paired in network
 * @retval          Number of nodes
 */
uint32_t net_info_get_number_of_nodes_paired(void);

/**
 * @brief           Delete all configuration in flash
 */
void net_info_delele_all(void);

#endif /* NETWORK_INFO_H */
