/**
 * demo
 */

#include <stdio.h>

#include <rte_memory.h>
#include <rte_memzone.h>
#include <rte_launch.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_lcore.h>
#include <rte_debug.h>
#include <rte_ethdev.h>
#include <rte_cycles.h>
#include <rte_mbuf.h>
#include <rte_acl.h>
#include <rte_mempool.h>
#include <rte_malloc.h>
#include <rte_ring.h>
#include <rte_ip.h>
#include <rte_hash.h>
#include <rte_rcu_qsbr.h>
#include <sys/times.h>
#include  <rte_mbuf.h>

#define NUM_QUEUE 1
#define NUM_DESC 1024
#define NUM_MBUFS 8191
#define MBUF_CACHE_SIZE 250
#define NUM_BURST 32
#define QUEUE_MAX 32
#define THREAD_MAX 32

typedef struct ST_PMD_CFG
{
    uint32_t core_num;
    uint16_t port_num;
    uint16_t queue_num;
    uint32_t mbuf_num;
    uint32_t desc_num;
    uint32_t brust_num;
    uint32_t queue_all;

} ST_PMD_CFG;

typedef struct ST_TH_PARAM
{
    uint16_t p_id;
    uint16_t q_id;
    rte_atomic64_t pps;
    rte_atomic64_t bps;
} ST_TH_PARAM, *PST_TH_PARAM;

ST_PMD_CFG g_cfg;
ST_TH_PARAM g_th_param[QUEUE_MAX];

void port_init(void)
{
    int retval;
    uint16_t portid;
    RTE_ETH_FOREACH_DEV(portid)
    {
        if(rte_eth_dev_is_valid_port(portid) == 0)
        {
            rte_exit(EXIT_FAILURE, "Cannot init (port %hu)\n", portid);
        }
        else
        {
            printf("start to init port:%hu\n", portid);
        }

        struct rte_eth_dev_info dev_info;
        retval = rte_eth_dev_info_get(portid, &dev_info);
        if(retval != 0)
        {
            rte_exit(EXIT_FAILURE, "Cannot getting device (port %hu) info: %s\n", portid, strerror(-retval));
        }

        struct rte_eth_conf port_conf;
        memset(&port_conf, 0, sizeof(struct rte_eth_conf));
        //if (dev_info.tx_offload_capa & RTE_ETH_TX_OFFLOAD_MBUF_FAST_FREE)
        //{
        //    port_conf.txmode.offloads |= RTE_ETH_TX_OFFLOAD_MBUF_FAST_FREE;
        //}

        retval = rte_eth_dev_configure(portid, g_cfg.queue_num, 1, &port_conf);
        if(retval != 0)
        {
            rte_exit(EXIT_FAILURE, "Cannot config (port %hu)\n", portid);
        }

        uint16_t nb_rxd = g_cfg.desc_num;
        uint16_t nb_txd = g_cfg.desc_num;
        retval = rte_eth_dev_adjust_nb_rx_tx_desc(portid, &nb_rxd, &nb_txd);
        if(retval != 0)
        {
            rte_exit(EXIT_FAILURE, "Cannot set desc (port %hu)\n", portid);
        }
        uint16_t q;
        for(q = 0; q < g_cfg.queue_num; q++)
        {
            struct rte_mempool *mbuf_pool;
            char buf_name[64];
            sprintf(buf_name, "buf_pool_%hu_%hu", portid, q);
            mbuf_pool = rte_pktmbuf_pool_create(buf_name, g_cfg.mbuf_num,
                                                MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE,
                                                rte_eth_dev_socket_id(portid));

            retval = rte_eth_rx_queue_setup(portid, q, nb_rxd, rte_eth_dev_socket_id(portid), NULL, mbuf_pool);
            if(retval < 0)
            {
                rte_exit(EXIT_FAILURE, "Cannot set pool (port %hu)\n", portid);
            }
        }

        rte_eth_tx_queue_setup(portid, 0, nb_txd, rte_eth_dev_socket_id(portid), NULL);

        /* Starting Ethernet port. 8< */
        retval = rte_eth_dev_start(portid); // 启动网口
        /* >8 End of starting of ethernet port. */
        if(retval < 0)
        {
            rte_exit(EXIT_FAILURE, "Cannot start (port %hu)\n", portid);
        }

        retval = rte_eth_promiscuous_enable(portid);
        if(retval != 0)
        {
            rte_exit(EXIT_FAILURE, "Cannot set promiscuous (port %hu)\n", portid);
        }
    }
}

int32_t lcore_rcv(void *arg)
{
    PST_TH_PARAM param = (PST_TH_PARAM)arg;
    unsigned lcore_id;
    lcore_id = rte_lcore_id();
    printf("hello from core %u, port %hu queue %hu\n", lcore_id, param->p_id, param->q_id);

    /* Get burst of RX packets, from first port of pair. */
    struct rte_mbuf *bufs[128];
    uint16_t nb_rx;
    while(1)
    {
        nb_rx = rte_eth_rx_burst(param->p_id, param->q_id, bufs, g_cfg.brust_num);

        if(unlikely(nb_rx > 0))
        {
            rte_atomic64_add(&param->pps, nb_rx);
            int i;
            for(i = 0; i < nb_rx; i++)
            {
                rte_atomic64_add(&param->bps, bufs[i]->data_len);
                uint8_t *data = (uint8_t *)(bufs[i]->buf_addr) + bufs[i]->data_off;
                uint16_t mid = bufs[i]->data_len / 2;
                data[0] = 0x12;
                data[mid] = 0x34;
                data[bufs[i]->data_len - 1] = 0x56;
                rte_pktmbuf_free(bufs[i]);
            }
        }
    }
    return 0;
}

void show(void)
{
#define TIMES 10 //默认打印记录10次
    uint64_t g_stat[QUEUE_MAX][TIMES][2] = {0};
    uint32_t i = 0;
    uint32_t t = 0;

    printf(" core  num: %u\n", g_cfg.core_num);
    printf(" port  num: %u\n", g_cfg.port_num);
    printf(" queue num: %u\n", g_cfg.queue_num);
    printf(" mbuf  num: %u\n", g_cfg.mbuf_num);
    printf(" desc  num: %u\n", g_cfg.desc_num);
    printf(" burst num: %u\n", g_cfg.brust_num);
    printf(" queue all: %u\n", g_cfg.queue_all);


    while(1)
    {
        rte_delay_ms(1000);
        for(i = 0; i < g_cfg.queue_all; i++)
        {
            g_stat[i][t][0] = rte_atomic64_read(&g_th_param[i].pps);
            rte_atomic64_clear(&g_th_param[i].pps);
            g_stat[i][t][1] = rte_atomic64_read(&g_th_param[i].bps);
            rte_atomic64_clear(&g_th_param[i].bps);
        }

        for(i = 0; i < g_cfg.queue_all; i++)
        {
            printf("%15lu %15lu\n", g_stat[i][t][0], g_stat[0][t][1]);
        }
        t = (t + 1) % TIMES;
    }
}

void usage(void)
{
    printf("usage:\n");
}

int main(int argc, char **argv)
{
    int ret = rte_eal_init(argc, argv);
    if(ret < 0)
        rte_panic("Cannot init EAL\n");
    else
        printf("init eal\n");
    argc -= ret;
    argv += ret;
    // uint32_t socket_id = rte_socket_id();

    g_cfg.core_num = rte_lcore_count();
    if(g_cfg.core_num < 2)
    {
        rte_exit(EXIT_FAILURE, "need 2 core\n");
    }

    g_cfg.port_num = rte_eth_dev_count_avail();
    if(g_cfg.port_num == 0)
    {
        rte_exit(EXIT_FAILURE, "no port available\n");
    }

    if(argc == 1)
    {
        g_cfg.queue_num = NUM_QUEUE;
        g_cfg.mbuf_num = NUM_MBUFS;
        g_cfg.desc_num = NUM_DESC;
        g_cfg.brust_num = NUM_BURST;
        g_cfg.queue_all = 0;
    }
    else if(argc == 5)
    {
        g_cfg.queue_num = atoi(argv[1]);
        g_cfg.mbuf_num = atoi(argv[2]);
        g_cfg.desc_num = atoi(argv[3]);
        g_cfg.brust_num = atoi(argv[4]);
        g_cfg.queue_all = 0;
    }
    else
    {
        usage();
        goto end;
    }

    port_init();

    uint32_t lcore_id;
    uint16_t p_id = 0;
    uint16_t q_id = 0;
    uint32_t i = 0;
    uint32_t flag = 0;
    RTE_LCORE_FOREACH(lcore_id)
    {
        if(flag == 0)
        {
            flag = 1;
            continue;
        }
        g_th_param[i].p_id = p_id;
        g_th_param[i].q_id = q_id++;
        rte_atomic64_clear(&g_th_param[i].pps);
        rte_atomic64_clear(&g_th_param[i].bps);
        rte_eal_remote_launch(lcore_rcv, (void *)(g_th_param + i), lcore_id);
        if(q_id == g_cfg.queue_num)
        {
            p_id += 1;
            q_id = 0;
        }
        g_cfg.queue_all += 1;
        i++;
    }

    show();

    rte_eal_mp_wait_lcore();
end:
    rte_eal_cleanup(); // 清理eal
    return 0;
}