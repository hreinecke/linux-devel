/*
 * dm-ssdcache.c
 *
 * Copyright (c) 2011 Hannes Reinecke, SUSE Linux Products GmbH
 *
 * This file is released under the GPL.
 */

#include "dm.h"
#include <linux/module.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <linux/slab.h>
#include <linux/hash.h>
#include <linux/device-mapper.h>
#include <linux/dm-io.h>
#include <linux/dm-kcopyd.h>

#define DM_MSG_PREFIX "ssdcache: "

#define SSD_DEBUG
#define SSD_LOG

#ifdef SSD_LOG
#define DPRINTK( s, arg... ) printk(DM_MSG_PREFIX s "\n", ##arg)
#define WPRINTK( w, s, arg... ) printk(DM_MSG_PREFIX "%lu: %s (cte %lx:%lx): "\
				       s "\n", (w)->nr, __FUNCTION__, \
				       (w)->cmd->hash, \
				       (w)->cte_idx, ##arg)
#else
#define DPRINTK( s, arg... )
#define WPRINTK( w, s, arg... )
#endif

#define SSDCACHE_COPY_PAGES 1024
#define MIN_SIO_ITEMS 1024
#define MIN_CTE_NUM 512
#define MIN_CMD_NUM 64
#define DEFAULT_CTE_NUM 4096

#define DEFAULT_BLOCKSIZE	4096
#define DEFAULT_ASSOCIATIVITY	8

/* Caching modes */
enum ssdcache_mode_t {
	CACHE_IS_WRITETHROUGH,
	CACHE_IS_WRITEBACK,
};

/* Caching strategies */
enum ssdcache_strategy_t {
	CACHE_LRU,
	CACHE_LFU,
};

enum ssdcache_algorithm_t {
	CACHE_ALG_HASH64,
	CACHE_ALG_WRAP,
};

static enum ssdcache_mode_t default_cache_mode = CACHE_IS_WRITETHROUGH;
static enum ssdcache_strategy_t default_cache_strategy = CACHE_LFU;
static enum ssdcache_algorithm_t default_cache_algorithm = CACHE_ALG_HASH64;

struct ssdcache_md;
struct ssdcache_io;

struct ssdcache_te {
	unsigned long index;	/* Offset within table entry block */
	unsigned long atime;	/* Timestamp of the block's last access */
	unsigned long count;    /* Number of accesses */
	unsigned long state;    /* State bitmap */
	sector_t sector;	/* Sector number on target device */
	struct ssdcache_md *md; /* Backlink to metadirectory */
	struct bio_list writeback;
	struct rcu_head rcu;
};

struct ssdcache_md {
	spinlock_t lock;	/* Lock to protect operations on the bio list */
	unsigned long hash;	/* Hash number */
	unsigned int num_cte;	/* Number of table entries */
	unsigned long atime;
	struct ssdcache_te *te[DEFAULT_ASSOCIATIVITY];	/* RCU Table entries */
};

struct ssdcache_c {
	struct dm_dev *target_dev;
	struct dm_dev *cache_dev;
	struct dm_io_client *iocp;
	struct radix_tree_root md_tree;
	spinlock_t cmd_lock;
	unsigned long hash_bits;
	unsigned long hash_shift;
	unsigned long block_size;
	unsigned long block_mask;
	sector_t data_offset;
	unsigned long nr_sio;
	enum ssdcache_mode_t cache_mode;
	enum ssdcache_strategy_t cache_strategy;
	unsigned long cache_misses;
	unsigned long cache_hits;
	unsigned long cache_bypassed;
	unsigned long cache_evictions;
};

struct ssdcache_io {
	struct list_head list;
	struct kref kref;
	unsigned long nr;
	struct ssdcache_c *sc;
	struct ssdcache_md *cmd;
	long cte_idx;
	struct bio *bio;
	unsigned long bio_mask;
};

static DEFINE_SPINLOCK(_work_lock);
static struct workqueue_struct *_ssdcached_wq;
static struct work_struct _ssdcached_work;
static LIST_HEAD(_cte_work);
static LIST_HEAD(_io_work);

static struct kmem_cache *_sio_cache;
static struct kmem_cache *_cmd_cache;
static struct kmem_cache *_cte_cache;
static mempool_t *_sio_pool;
static mempool_t *_cmd_pool;
static mempool_t *_cte_pool;

/* Cache metadirectory states */
enum cmd_state {
	CMD_STATE_UNMAPPED,
	CMD_STATE_MAPPED,
	CMD_STATE_RESERVED,
};

/* Cache table entry states */
enum cte_state {
	CTE_SETUP,
	/* Permanent states */
	CTE_INVALID,	/* Sector not valid */
	CTE_CLEAN,	/* Sector valid, Cache and target date identical */
	CTE_DIRTY,	/* Sector valid, Cache and target data differ */
	/* Transient states */
	CTE_PREFETCH,	/* Sector valid, Read target data */
	CTE_RESERVED,	/* Sector valid, Transfer target data into cache */
	CTE_UPDATE,	/* Sector valid, Write cache data */
	CTE_WRITEBACK,	/* Sector valid, Transfer cache data to target */
};

/* Cache table entry state encoding */
#define CTE_STATE_SHIFT 48
#define CTE_SECTOR_MASK ~(1ULL << CTE_STATE_SHIFT)
#define CTE_STATE_MASK (0xFFULL << CTE_STATE_SHIFT)

static int do_io(struct ssdcache_io *sio);

/*
 * Slab pools
 */

static int pool_init(void)
{
	_sio_cache = kmem_cache_create("ssdcache-sio",
					sizeof(struct ssdcache_io),
					__alignof__(struct ssdcache_io),
					0, NULL);
	if (!_sio_cache)
		return -ENOMEM;

	_cmd_cache = kmem_cache_create("ssdcache-cmd",
				       sizeof(struct ssdcache_md),
				       __alignof__(struct ssdcache_md),
				       0, NULL);

	if (!_cmd_cache) {
		kmem_cache_destroy(_sio_cache);
		return -ENOMEM;
	}

	_cte_cache = kmem_cache_create("ssdcache-cte",
				       sizeof(struct ssdcache_te),
				       __alignof__(struct ssdcache_te),
				       0, NULL);

	if (!_cte_cache) {
		kmem_cache_destroy(_cmd_cache);
		kmem_cache_destroy(_sio_cache);
		return -ENOMEM;
	}

	_sio_pool = mempool_create(MIN_SIO_ITEMS, mempool_alloc_slab,
				    mempool_free_slab, _sio_cache);
	if (!_sio_pool) {
		kmem_cache_destroy(_cte_cache);
		kmem_cache_destroy(_cmd_cache);
		kmem_cache_destroy(_sio_cache);
		return -ENOMEM;
	}

	_cmd_pool = mempool_create(MIN_CMD_NUM, mempool_alloc_slab,
				   mempool_free_slab, _cmd_cache);
	if (!_cmd_pool) {
		mempool_destroy(_sio_pool);
		kmem_cache_destroy(_cte_cache);
		kmem_cache_destroy(_cmd_cache);
		kmem_cache_destroy(_sio_cache);
	}

	_cte_pool = mempool_create(MIN_CTE_NUM, mempool_alloc_slab,
				   mempool_free_slab, _cte_cache);
	if (!_cte_pool) {
		mempool_destroy(_cmd_pool);
		mempool_destroy(_sio_pool);
		kmem_cache_destroy(_cte_cache);
		kmem_cache_destroy(_cmd_cache);
		kmem_cache_destroy(_sio_cache);
	}

	return 0;
}

static void pool_exit(void)
{
	mempool_destroy(_cte_pool);
	mempool_destroy(_cmd_pool);
	mempool_destroy(_sio_pool);
	kmem_cache_destroy(_cte_cache);
	kmem_cache_destroy(_cmd_cache);
	kmem_cache_destroy(_sio_cache);
}

/*
 * cache metadirectory handling
 */

static inline struct ssdcache_md *cmd_lookup(struct ssdcache_c *sc,
					     unsigned long hash_number)
{
	struct ssdcache_md *cmd;

	rcu_read_lock();
	cmd = radix_tree_lookup(&sc->md_tree, hash_number);
	rcu_read_unlock();
	return cmd;
}

static inline struct ssdcache_md *cmd_insert(struct ssdcache_c *sc,
					     unsigned long hash_number)
{
	struct ssdcache_md *cmd;
	unsigned long flags;

	cmd = cmd_lookup(sc, hash_number);
	if (cmd)
		return cmd;

	cmd = mempool_alloc(_cmd_pool, GFP_NOIO | __GFP_ZERO);
	if (!cmd)
		return NULL;
	cmd->hash = hash_number;
	cmd->num_cte = (1UL << sc->hash_shift) * DEFAULT_ASSOCIATIVITY;
	spin_lock_init(&cmd->lock);
	cmd->atime = jiffies;

	if (radix_tree_preload(GFP_NOIO)) {
		mempool_free(cmd, _cmd_pool);
		return NULL;
	}

	spin_lock_irqsave(&sc->cmd_lock, flags);
	if (radix_tree_insert(&sc->md_tree, hash_number, cmd)) {
		mempool_free(cmd, _cmd_pool);
		cmd = radix_tree_lookup(&sc->md_tree, hash_number);
		BUG_ON(!cmd);
		BUG_ON(cmd->hash != hash_number);
	}
	spin_unlock_irqrestore(&sc->cmd_lock, flags);

	radix_tree_preload_end();
	return cmd;
}

/*
 * Macros for accessing table entry data
 */
static const struct {
	enum cte_state value;
	char *name;
} cte_state_string[] = {
	{ CTE_SETUP, "CTE_SETUP" },
	{ CTE_INVALID, "CTE_INVALID"},
	{ CTE_CLEAN, "CTE_CLEAN" },
	{ CTE_DIRTY, "CTE_DIRTY" },
	{ CTE_PREFETCH, "CTE_PREFETCH" },
	{ CTE_RESERVED, "CTE_RESERVED" },
	{ CTE_UPDATE, "CTE_UPDATE" },
	{ CTE_WRITEBACK, "CTE_WRITEBACK" },
};

const char *cte_state_name(enum cte_state state)
{
	int i;
	char *name = NULL;

	for (i = 0; i < ARRAY_SIZE(cte_state_string); i++) {
		if (cte_state_string[i].value == state) {
			name = cte_state_string[i].name;
			break;
		}
	}
	return name;
}

#define cte_bio_align(s,b) ((b)->bi_sector & ~s->block_mask)
#define cte_bio_offset(s,b) ((b)->bi_sector & s->block_mask)

static unsigned long cte_bio_mask(struct ssdcache_c *sc, struct bio *bio)
{
	unsigned long mask = 0;
	int i;

	BUG_ON(!bio);

	for (i = 0; i < to_sector(sc->block_size); i++) {
		if (i >= cte_bio_offset(sc, bio) &&
		    i < cte_bio_offset(sc, bio) + to_sector(bio->bi_size)) {
			mask |= (0xFUL << (i * 4));
		}
	}

	return mask;
}

struct ssdcache_te * cte_new(struct ssdcache_c *sc, struct ssdcache_md *cmd,
			     unsigned int index)
{
	struct ssdcache_te *newcte;
	unsigned long newstate = 0;
	int i;

	newcte = mempool_alloc(_cte_pool, GFP_NOWAIT | __GFP_ZERO);
	if (!newcte)
		return NULL;

	newcte->index = index;
	newcte->atime = jiffies;
	newcte->count = 1;
	for (i = 0; i < to_sector(sc->block_size); i++)
		newstate |= (unsigned long)CTE_INVALID << (i * 4);
	newcte->state = newstate;
	newcte->md = cmd;
	bio_list_init(&newcte->writeback);

	return newcte;
}

static void cte_reset(struct rcu_head *rp)
{
	struct ssdcache_te *cte = container_of(rp, struct ssdcache_te, rcu);

	mempool_free(cte, _cte_pool);
}

static void * cte_insert(struct ssdcache_c *sc, struct ssdcache_md *cmd,
			 unsigned int index)
{
	struct ssdcache_te *newcte, *oldcte;
	unsigned long flags;

	rcu_read_lock();
	newcte = rcu_dereference(cmd->te[index]);
	rcu_read_unlock();
	if (newcte)
		return newcte;

	newcte = cte_new(sc, cmd, index);
	if (!newcte)
		return NULL;

	spin_lock_irqsave(&cmd->lock, flags);
	oldcte = cmd->te[index];
	cmd->atime = jiffies;
	rcu_assign_pointer(cmd->te[index], newcte);
	spin_unlock_irqrestore(&cmd->lock, flags);
	if (oldcte)
		call_rcu(&oldcte->rcu, cte_reset);

	return newcte;
}

static inline int cte_is_state(struct ssdcache_c *sc, struct ssdcache_te *cte,
			       struct bio *bio, enum cte_state state)
{
	unsigned long oldstate;
	enum cte_state tmpstate;
	int i, offset, match = 0;

	if (!cte)
		return 0;

	rcu_read_lock();
	oldstate = rcu_dereference(cte)->state;
	rcu_read_unlock();

	offset = cte_bio_offset(sc, bio);
	for (i = 0; i < to_sector(bio->bi_size); i++) {
		tmpstate = (oldstate >> ((offset + i) * 4)) & 0xF;
		match += (tmpstate == state);
	}
	return (match == to_sector(bio->bi_size));
}

static inline int cte_check_state_transition(enum cte_state oldstate,
					     enum cte_state newstate)
{
	int illegal_transition = 0;

	if (oldstate == newstate)
		return 0;
	/* Check state machine transitions */
	switch (newstate) {
	case CTE_SETUP:
		illegal_transition++;
		break;
	case CTE_INVALID:
		switch (oldstate) {
		case CTE_SETUP:
		case CTE_CLEAN:
		case CTE_PREFETCH:
		case CTE_RESERVED:
		case CTE_UPDATE:
			break;
		default:
			illegal_transition++;
			break;
		}
		break;
	case CTE_CLEAN:
		switch (oldstate) {
		case CTE_WRITEBACK:
		case CTE_RESERVED:
			break;
		default:
			illegal_transition++;
			break;
		}
		break;
	case CTE_DIRTY:
		switch (oldstate) {
		case CTE_WRITEBACK:
		case CTE_UPDATE:
			break;
		default:
			illegal_transition++;
			break;
		}
		break;
	case CTE_PREFETCH:
		switch (oldstate) {
		case CTE_INVALID:
			break;
		default:
			illegal_transition++;
		}
		break;
	case CTE_RESERVED:
		switch (oldstate) {
		case CTE_PREFETCH:
		case CTE_UPDATE:
			break;
		default:
			illegal_transition++;
		}
		break;
	case CTE_UPDATE:
		switch (oldstate) {
		case CTE_INVALID:
		case CTE_CLEAN:
		case CTE_DIRTY:
			break;
		default:
			illegal_transition++;
			break;
		}
		break;
	case CTE_WRITEBACK:
		switch (oldstate) {
		case CTE_DIRTY:
		case CTE_UPDATE:
			break;
		default:
			illegal_transition++;
		}
		break;
	}
	return illegal_transition;
}

static unsigned long sio_update_state(struct ssdcache_io *sio,
				      unsigned long oldstate,
				      enum cte_state state)
{
	unsigned long newstate = 0, state_shift, tmpmask;
	int i;
	enum cte_state tmpstate;

	for (i = 0; i < to_sector(sio->sc->block_size); i++) {
		state_shift = (i * 4);
		tmpmask = (sio->bio_mask >> state_shift) & 0xF;
		if (tmpmask == 0xF) {
			tmpstate = (oldstate >> state_shift) & 0xF;
			if (cte_check_state_transition(tmpstate, state)) {
				DPRINTK("Illegal state transition %s -> %s",
					cte_state_name(tmpstate),
					cte_state_name(state));
			}
		}
		newstate |= (unsigned long)state << state_shift;
	}

	return (oldstate & ~(sio->bio_mask)) | (newstate & sio->bio_mask);
}

static unsigned long sio_get_state(struct ssdcache_io *sio)
{
	unsigned long state;
	struct ssdcache_te *cte;

	memset(&state, 0x11, sizeof(unsigned long));
	rcu_read_lock();
	if (sio && sio->cte_idx >= 0) {
		cte = rcu_dereference(sio->cmd->te[sio->cte_idx]);
		BUG_ON(!cte);
		state = cte->state;
	}
	rcu_read_unlock();

	return state;
}

static inline bool sio_is_state(struct ssdcache_io *sio, enum cte_state state)
{
	unsigned long oldstate, newstate;
	unsigned char tmpstate;

	if (!sio)
		return false;

	oldstate = sio_get_state(sio);
	tmpstate = state | (state << 4);
	memset(&newstate, tmpstate, sizeof(unsigned long));

	return ((oldstate & sio->bio_mask) == (newstate & sio->bio_mask));
}

static void sio_set_state(struct ssdcache_io *sio, enum cte_state state)
{
	struct ssdcache_te *oldcte, *newcte = NULL;
	unsigned long newstate, oldstate, tmpstate, flags;

	if (!sio || sio->cte_idx < 0)
		return;

	oldstate = sio_get_state(sio);
	newstate = sio_update_state(sio, oldstate, state);

	/* Check if we should drop the old cte */
	memset(&tmpstate, 0x11, sizeof(unsigned long));
	tmpstate &= sio->sc->block_mask;

	if ((newstate & sio->sc->block_mask) != tmpstate ) {
		/* cte still valid */
		newcte = mempool_alloc(_cte_pool, GFP_NOWAIT | __GFP_ZERO);
		if (!newcte)
			return;
	}
	spin_lock_irqsave(&sio->cmd->lock, flags);
	oldcte = sio->cmd->te[sio->cte_idx];
	if (newcte) {
		*newcte = *oldcte;
		newcte->state = newstate;
		newcte->atime = jiffies;
		newcte->count++;
	}
	rcu_assign_pointer(sio->cmd->te[oldcte->index], newcte);
	spin_unlock_irqrestore(&sio->cmd->lock, flags);
	call_rcu(&oldcte->rcu, cte_reset);
}

static bool cte_is_busy(struct ssdcache_c *sc, struct ssdcache_te * cte,
			struct bio * bio)
{
	unsigned long oldstate, offset, num_sectors;
	enum cte_state tmpstate;
	int match = 0, i;

	rcu_read_lock();
	oldstate = rcu_dereference(cte)->state;
	rcu_read_unlock();

	if (bio) {
		num_sectors = to_sector(bio->bi_size);
		offset= cte_bio_offset(sc, bio);
	} else {
		num_sectors = to_sector(sc->block_size);
		offset = 0;
	}
	for (i = 0; i < num_sectors; i++) {
		tmpstate = (oldstate >> ((offset + i) * 4)) & 0xF;
		match += ((tmpstate == CTE_UPDATE) ||
			  (tmpstate == CTE_PREFETCH) ||
			  (tmpstate == CTE_RESERVED));
		if (!bio || bio_data_dir(bio) == WRITE)
			match += (tmpstate == CTE_WRITEBACK);
	}
	return (match > 0);
}

#define sio_cte_is_busy(s) cte_is_busy((s)->sc,(s)->cmd->te[(s)->cte_idx],(s)->bio)

/*
 * cte_start_sequence
 *
 * Setup sio to start a new sequence starting with @state.
 */
static void cte_start_sequence(struct ssdcache_io *sio, enum cte_state state)
{
	struct ssdcache_te *oldcte, *newcte;
	unsigned long oldstate, newstate, flags;

	BUG_ON(!sio->bio);
	BUG_ON(sio->cte_idx < 0);
	oldstate = sio_get_state(sio);

	newcte = mempool_alloc(_cte_pool, GFP_NOWAIT | __GFP_ZERO);
	if (!newcte)
		return;

	sio->bio_mask = cte_bio_mask(sio->sc, sio->bio);
	newstate = sio_update_state(sio, oldstate, state);

	spin_lock_irqsave(&sio->cmd->lock, flags);
	oldcte = sio->cmd->te[sio->cte_idx];
	*newcte = *oldcte;
	newcte->sector = cte_bio_align(sio->sc, sio->bio);
	newcte->state = newstate;
	newcte->atime = jiffies;
	newcte->count++;
	rcu_assign_pointer(sio->cmd->te[sio->cte_idx], newcte);
	spin_unlock_irqrestore(&sio->cmd->lock, flags);
	call_rcu(&oldcte->rcu, cte_reset);
}

/*
 * cte_restart_sequence
 *
 * Setup a sio to start a sequence with no state change.
 * Only valid for read requests in CTE_DIRTY or CTE_CLEAN.
 */
static void cte_restart_sequence(struct ssdcache_io *sio)
{
	struct ssdcache_te *oldcte, *newcte;
	sector_t sector;
	unsigned long flags;

	BUG_ON(!sio->bio);
	BUG_ON(sio->cte_idx < 0);

	rcu_read_lock();
	sector = rcu_dereference(sio->cmd->te[sio->cte_idx])->sector;
	rcu_read_unlock();

	if (sector != cte_bio_align(sio->sc, sio->bio)) {
		WPRINTK(sio, "access to non sector %llx/%llx",
			(unsigned long long)cte_bio_align(sio->sc, sio->bio),
			(unsigned long long)sector);
	}

	newcte = mempool_alloc(_cte_pool, GFP_NOWAIT | __GFP_ZERO);
	if (!newcte)
		return;

	spin_lock_irqsave(&sio->cmd->lock, flags);
	oldcte = sio->cmd->te[sio->cte_idx];
	*newcte = *oldcte;
	newcte->atime = jiffies;
	newcte->count++;
	rcu_assign_pointer(sio->cmd->te[sio->cte_idx], newcte);
	spin_unlock_irqrestore(&sio->cmd->lock, flags);
	call_rcu(&oldcte->rcu, cte_reset);
}

static inline bool cte_match_sector(struct ssdcache_c *sc,
				    struct ssdcache_te *cte,
				    struct bio *bio)
{
	bool match = false;
	unsigned long oldstate, tmpstate, bio_mask;
	sector_t sector;

	if (!cte)
		return match;

	memset(&tmpstate, 0x11, sizeof(unsigned long));
	bio_mask = cte_bio_mask(sc, bio);
	rcu_read_lock();
	sector = rcu_dereference(cte)->sector;
	oldstate = rcu_dereference(cte)->state;
	rcu_read_unlock();
	if ((oldstate & bio_mask) != (tmpstate & bio_mask))
		match = (cte_bio_align(sc, bio) == sector);

	return match;
}

/*
 * Workqueue handling
 */

static struct ssdcache_io *ssdcache_create_sio(struct ssdcache_c *sc)
{
	struct ssdcache_io *sio;

	sio = mempool_alloc(_sio_pool, GFP_NOIO);
	if (!sio)
		return NULL;
	memset(sio, 0, sizeof(struct ssdcache_io));
	sio->sc = sc;
	sio->cte_idx = -1;
	sio->nr = ++sc->nr_sio;
	kref_init(&sio->kref);
	INIT_LIST_HEAD(&sio->list);
	return sio;
}

static void ssdcache_destroy_sio(struct kref *kref)
{
	struct ssdcache_io *sio = container_of(kref, struct ssdcache_io, kref);

	BUG_ON(!list_empty(&sio->list));

	mempool_free(sio, _sio_pool);
}

static void ssdcache_get_sio(struct ssdcache_io *sio)
{
	kref_get(&sio->kref);
}

static int ssdcache_put_sio(struct ssdcache_io *sio)
{
	return kref_put(&sio->kref, ssdcache_destroy_sio);
}

static inline void push_sio(struct list_head *q, struct ssdcache_io *w)
{
	unsigned long flags;

	spin_lock_irqsave(&_work_lock, flags);
	list_add_tail(&w->list, q);
	spin_unlock_irqrestore(&_work_lock, flags);
}

static void ssdcache_schedule_sio(struct ssdcache_io *sio)
{
	push_sio(&_io_work, sio);
	queue_work(_ssdcached_wq, &_ssdcached_work);
}

static int process_sio(struct list_head *q,
			 int (*fn) (struct ssdcache_io *))
{
	LIST_HEAD(tmp);
	LIST_HEAD(defer);
	unsigned long flags;
	struct ssdcache_io *sio, *next;
	int r, dequeued = 0;

	spin_lock_irqsave(&_work_lock, flags);
	list_splice_init(q, &tmp);
	spin_unlock_irqrestore(&_work_lock, flags);
	list_for_each_entry_safe(sio, next, &tmp, list) {
		list_del_init(&sio->list);
		r = fn(sio);
		if (r < 0) {
			DMERR("process_sio: Job processing error");
		} else if (r > 0) {
			list_add_tail(&sio->list, &defer);
		} else {
			dequeued++;
		}
	}
	spin_lock_irqsave(&_work_lock, flags);
	list_splice(&defer, q);
	list_splice(&tmp, q);
	spin_unlock_irqrestore(&_work_lock, flags);
	return dequeued;
}

static void do_sio(struct work_struct *ignored)
{
	int items = 1, empty = 0;

	while (items > 0) {
		empty = 0;
		items = process_sio(&_io_work, do_io);
		if (list_empty(&_io_work))
			empty++;
	}
}

static struct bio * sio_fetch_writeback_bio(struct ssdcache_io *sio)
{
	struct ssdcache_te *cte;
	unsigned long flags;
	struct bio *bio;

	if (!sio || !sio->cmd || sio->cte_idx < 0)
		return NULL;

	spin_lock_irqsave(&sio->cmd->lock, flags);
	cte = sio->cmd->te[sio->cte_idx];
	bio = bio_list_pop(&cte->writeback);
	spin_unlock_irqrestore(&sio->cmd->lock, flags);

	return bio;
}

/*
 * finish_reserved
 *
 * Finish I/O for cte in RESERVED
 * Data has been written to cache device.
 * Finish I/O and complete sio item.
 */
static int finish_reserved(struct ssdcache_io *sio, unsigned long error)
{
	int i;
	struct bio_vec *bvec;

	if (error) {
		WPRINTK(sio, "error, mark cte invalid");
		sio_set_state(sio, CTE_INVALID);
	} else {
		sio_set_state(sio, CTE_CLEAN);
	}

	if (sio->bio) {
		/* Release bio */
		bio_for_each_segment(bvec, sio->bio, i)
			put_page(bvec->bv_page);
		bio_put(sio->bio);
		sio->bio = NULL;
	}

	ssdcache_put_sio(sio);
	return 0;
}

static int finish_update(struct ssdcache_io *sio, unsigned long error)
{
	if (error) {
		/* Move back to INVALID */
		WPRINTK(sio, "error, mark cte invalid");
		sio_set_state(sio, CTE_INVALID);
	} else {
		sio_set_state(sio, CTE_WRITEBACK);
	}
	if (sio->bio) {
		bio_put(sio->bio);
		sio->bio = NULL;
	}
	ssdcache_put_sio(sio);
	return 0;
}

static int finish_writeback(struct ssdcache_io *sio, unsigned long error)
{
	if (error) {
		WPRINTK(sio, "error, mark cte invalid");
		sio_set_state(sio, CTE_INVALID);
	} else {
		sio_set_state(sio, CTE_CLEAN);
	}
	if (sio->bio) {
		bio_put(sio->bio);
		sio->bio = NULL;
	}
	ssdcache_put_sio(sio);
	return 0;
}

static void io_callback(unsigned long error, void *context)
{
	struct ssdcache_io *sio = context;
	int requeue = 0;

	if (error)
		WPRINTK(sio, "finished with %lu", error);
	BUG_ON(!sio->bio);
	if (sio_is_state(sio, CTE_RESERVED)) {
		requeue = finish_reserved(sio, error);
	} else if (sio_is_state(sio, CTE_UPDATE)) {
		requeue = finish_update(sio, error);
	} else if (sio_is_state(sio, CTE_WRITEBACK)) {
		requeue = finish_writeback(sio, error);
	} else if (sio_is_state(sio, CTE_INVALID)) {
		if (sio->bio)
			bio_put(sio->bio);
		ssdcache_put_sio(sio);
	} else {
		WPRINTK(sio, "unhandled state %08lx",
			sio_get_state(sio));
		if (sio->bio) {
			bio_put(sio->bio);
			sio->bio = NULL;
		}
		ssdcache_put_sio(sio);
	}
	if (requeue) {
		ssdcache_schedule_sio(sio);
	} else {
		queue_work(_ssdcached_wq, &_ssdcached_work);
	}
}

static inline sector_t to_cache_sector(struct ssdcache_io *sio,
				       sector_t data_sector)
{
	sector_t sector_offset, cte_offset, cmd_offset, cache_sector;

	sector_offset = data_sector & sio->sc->block_mask;

	cte_offset = to_sector(sio->cte_idx * sio->sc->block_size);
	cmd_offset = to_sector(sio->cmd->hash * sio->cmd->num_cte * sio->sc->block_size);
	cache_sector = sio->sc->data_offset + cmd_offset + cte_offset + sector_offset;

	return cache_sector;
}

static void write_to_cache(struct ssdcache_io *sio)
{
	struct dm_io_region cache;
	struct dm_io_request iorq;
	struct bio *bio = sio->bio;

	cache.bdev = sio->sc->cache_dev->bdev;
	cache.sector = to_cache_sector(sio, bio->bi_sector);
	cache.count = to_sector(sio->bio->bi_size);

	iorq.bi_rw = WRITE;
	iorq.mem.type = DM_IO_BVEC;
	iorq.mem.ptr.bvec = bio->bi_io_vec + bio->bi_idx;
	iorq.notify.fn = io_callback;
	iorq.notify.context = sio;
	iorq.client = sio->sc->iocp;

	dm_io(&iorq, 1, &cache, NULL);
}

static void write_to_target(struct ssdcache_io *sio)
{
	struct dm_io_region target;
	struct dm_io_request iorq;

	target.bdev = sio->sc->target_dev->bdev;
	target.sector = sio->bio->bi_sector;
	target.count = to_sector(sio->bio->bi_size);

	iorq.bi_rw = WRITE;
	iorq.mem.type = DM_IO_BVEC;
	iorq.mem.ptr.bvec = sio->bio->bi_io_vec + sio->bio->bi_idx;
	iorq.notify.fn = io_callback;
	iorq.notify.context = sio;
	iorq.client = sio->sc->iocp;

	dm_io(&iorq, 1, &target, NULL);
}

static int do_io(struct ssdcache_io *sio)
{
	BUG_ON(sio->cte_idx < 0);
	if (!sio->bio) {
		/* Fetch first writeback bio */
		sio->bio = sio_fetch_writeback_bio(sio);
		if (!sio->bio) {
			WPRINTK(sio, "no bio");
			ssdcache_put_sio(sio);
			return 0;
		} else {
			WPRINTK(sio, "fetch writeback bio");
			cte_start_sequence(sio, CTE_WRITEBACK);
		}
	}
	BUG_ON(bio_data_dir(sio->bio) == READ);
	if (sio_is_state(sio, CTE_RESERVED)) {
		write_to_cache(sio);
		return 0;
	}
	if (sio_is_state(sio, CTE_UPDATE)) {
		write_to_cache(sio);
		return 0;
	}
	if (sio_is_state(sio, CTE_WRITEBACK)) {
		write_to_target(sio);
		return 0;
	}
	WPRINTK(sio, "unhandled state %08lx", sio_get_state(sio));
	return EINVAL;
}

/*
 * Cache lookup and I/O handling
 */

static unsigned long ssdcache_hash_64(struct ssdcache_c *sc, sector_t sector)
{
	unsigned long value, hash_number, offset, hash_mask, sector_shift;

	sector_shift = fls(sc->block_size / 512) - 1;
	hash_mask = (1UL << sc->hash_shift) - 1;
	value = sector >> (sc->hash_shift + sector_shift);
	offset = sector & hash_mask;
	hash_number = (unsigned long)hash_64(value, sc->hash_bits - sc->hash_shift);

	return (hash_number << sc->hash_shift) | offset;
}

static unsigned long ssdcache_hash_wrap(struct ssdcache_c *sc, sector_t sector)
{
	unsigned long sector_shift, value, hash_mask;

	sector_shift = fls(sc->block_size/ 512) - 1;
	value = sector >> (sc->hash_shift + sector_shift);
	hash_mask = (1UL << (sc->hash_bits - sc->hash_shift)) - 1;

	return value & hash_mask;
}

static unsigned long hash_block(struct ssdcache_c *sc, sector_t sector)
{
	if (default_cache_algorithm == CACHE_ALG_HASH64)
		return ssdcache_hash_64(sc, sector);
	else
		return ssdcache_hash_wrap(sc, sector);
}

static int cte_lookup(struct ssdcache_c *sc,
		     struct ssdcache_md *cmd,
		     struct bio *bio)
{
	sector_t data_sector;
	struct ssdcache_te *cte;
	int i, invalid = -1;

	data_sector = cte_bio_align(sc, bio);
	for (i = 0; i < cmd->num_cte; i++) {
		cte = cmd->te[i];
		if (!cte || cte_is_state(sc, cte, bio, CTE_INVALID)) {
			if (invalid < 0)
				invalid = i;
			continue;
		} else if (cte_match_sector(sc, cte, bio)) {
			return i;
		}
		/* Break out if we have found an invalid entry */
		if (invalid != -1)
			break;
	}

	return invalid;
}

static int cte_match(struct ssdcache_c *sc,
		     struct ssdcache_md *cmd,
		     struct bio *bio)
{
	sector_t data_sector;
	struct ssdcache_te *cte;
	unsigned long cte_atime, clean_atime, oldest_atime, flags;
	unsigned long cte_count, clean_count, oldest_count;
	int invalid = -1, clean, oldest, i, busy = 0;

	data_sector = cte_bio_align(sc, bio);
	clean_atime = oldest_atime = jiffies;
	clean_count = oldest_count = -1;
	clean = oldest = -1;

	for (i = 0; i < cmd->num_cte; i++) {
		cte = cmd->te[i];
		if (!cte || cte_is_state(sc, cte, bio, CTE_INVALID)) {
			if (invalid < 0)
				invalid = i;
			continue;
		} else if (cte_match_sector(sc, cte, bio)) {
			return i;
		}
		/* Break out if we have found an invalid entry */
		if (invalid != -1)
			break;
		/* Ignore transient states */
		if (cte_is_busy(sc, cte, bio)) {
			busy++;
			continue;
		}

		if (sc->cache_strategy == CACHE_LRU) {
			/* Select the oldest clean entry */
			rcu_read_lock();
			cte_atime = rcu_dereference(cte)->atime;
			rcu_read_unlock();
			if (cte_is_state(sc, cte, bio, CTE_CLEAN) &&
			    time_before_eq(cte_atime, clean_atime)) {
				clean_atime = cte_atime;
				clean = i;
			} else if (time_before_eq(cte_atime, oldest_atime)) {
				oldest_atime = cte_atime;
				oldest = i;
			}
		} else {
			/* Select the lowest access count */
			rcu_read_lock();
			cte_count = rcu_dereference(cte)->count;
			rcu_read_unlock();
			if (cte_is_state(sc, cte, bio, CTE_CLEAN) &&
			    cte_count <= clean_count) {
				clean_count = cte_count;
				clean = i;
			} else if (cte_count <= oldest_count) {
				oldest_count = cte_count;
				oldest = i;
			}
		}
	}

	if (invalid != -1)
		return invalid;

	if (clean != -1) {
#ifdef SSD_DEBUG
		DPRINTK("%s (cte %lx:%x): drop clean cte", __FUNCTION__,
			cmd->hash, clean);

		for (i = 0; i < cmd->num_cte; i++) {
			sector_t cte_sector;
			unsigned long cte_state;

			cte = cmd->te[i];
			if (cte) {
				rcu_read_lock();
				cte_count = rcu_dereference(cte)->count;
				cte_sector = rcu_dereference(cte)->sector;
				cte_state = rcu_dereference(cte)->state;
				rcu_read_unlock();
				DPRINTK("%s (cte %lx:%x): sector %lx "
					"state %08lx count %ld",
					__FUNCTION__, cmd->hash, i,
					cte_sector, cte_state, cte_count);
			} else {
				DPRINTK("%s (cte %lx:%x): unassigned",
					__FUNCTION__, cmd->hash, i);
			}
		}
#endif
		spin_lock_irqsave(&cmd->lock, flags);
		cte = cmd->te[clean];
		rcu_assign_pointer(cmd->te[clean], NULL);
		spin_unlock_irqrestore(&cmd->lock, flags);
		call_rcu(&cte->rcu, cte_reset);
		sc->cache_evictions++;
		return clean;
	}

	if (oldest != -1) {
#ifdef SSD_DEBUG
		DPRINTK("%s (cte %lx:%x): drop oldest cte, %d/%d ctes busy",
			__FUNCTION__, cmd->hash, oldest, busy, i);
#endif
		spin_lock_irqsave(&cmd->lock, flags);
		cte = cmd->te[oldest];
		rcu_assign_pointer(cmd->te[oldest], NULL);
		spin_unlock_irqrestore(&cmd->lock, flags);
		call_rcu(&cte->rcu, cte_reset);
		sc->cache_evictions++;
		return oldest;
	}

	DPRINTK("%s (cte %lx:%x): %d ctes busy", __FUNCTION__,
		cmd->hash, -1, busy);
	return -1;
}

static int ssdcache_map(struct dm_target *ti, struct bio *bio,
		      union map_info *map_context)
{
	struct ssdcache_c *sc = ti->private;
	sector_t offset, data_sector;
	unsigned long hash_number, state = 0;
	struct ssdcache_te *cte;
	struct ssdcache_io *sio;
	int res = DM_MAPIO_SUBMITTED, assoc = 0;

	if (bio->bi_rw & REQ_FLUSH) {
		bio->bi_bdev = sc->target_dev->bdev;
		return DM_MAPIO_REMAPPED;
	}

	data_sector = cte_bio_align(sc, bio);
	offset = cte_bio_offset(sc, bio);
	hash_number = hash_block(sc, data_sector);

	/* splitting bios is not yet implemented */
	WARN_ON(bio->bi_size > sc->block_size);

	if (bio->bi_size == 0) {
		DPRINTK("zero-sized bio (flags %lx)", bio->bi_flags);
		bio->bi_bdev = sc->target_dev->bdev;
		return DM_MAPIO_REMAPPED;
	}

	sio = ssdcache_create_sio(sc);
	if (!sio) {
		DPRINTK("sio creation failure");
		sc->cache_bypassed++;
		bio->bi_bdev = sc->target_dev->bdev;
		ssdcache_put_sio(sio);
		return DM_MAPIO_REMAPPED;
	}

	/* Lookup cmd */
retry:
	sio->cmd = cmd_insert(sc, hash_number);
	if (!sio->cmd) {
		DPRINTK("cache insertion failure");
		sc->cache_bypassed++;
		bio->bi_bdev = sc->target_dev->bdev;
		map_context->ptr = NULL;
		ssdcache_put_sio(sio);
		return DM_MAPIO_REMAPPED;
	}

	sio->cte_idx = cte_lookup(sc, sio->cmd, bio);
	if (sio->cte_idx < 0) {
		if (assoc < DEFAULT_ASSOCIATIVITY) {
			assoc++;
			DPRINTK("(cte %04lx:ff): retry with assoc %d",
				sio->cmd->hash, assoc);
			hash_number = rol16(hash_number, 2);
			goto retry;
		}
		sio->cte_idx = cte_match(sc, sio->cmd, bio);
		if (sio->cte_idx < 0) {
			DPRINTK("bypass cte sio");
			sc->cache_bypassed++;
			bio->bi_bdev = sc->target_dev->bdev;
			map_context->ptr = NULL;
			ssdcache_put_sio(sio);
			return DM_MAPIO_REMAPPED;
		}
	}

	cte = cte_insert(sc, sio->cmd, sio->cte_idx);
	BUG_ON(!cte);
	state = sio_get_state(sio);
	sio->bio = bio;

	if (cte_match_sector(sc, cte, bio)) {
		/* Cache hit */
#ifdef SSD_DEBUG
		WPRINTK(sio, "%s hit %llu state %08lx/%08lx",
			bio_data_dir(bio) == READ ? "read" : "write",
			(unsigned long long)data_sector, state,
			cte_bio_mask(sc, bio));
#endif

		if (cte_is_busy(sc, cte, bio)) {
			/* Bypass cache */
			WPRINTK(sio, "bypass busy cte");
			sc->cache_bypassed++;
			bio->bi_bdev = sc->target_dev->bdev;
			ssdcache_put_sio(sio);
			map_context->ptr = NULL;
			return DM_MAPIO_REMAPPED;
		}
		if (bio_data_dir(bio) == READ) {
			sc->cache_hits++;
			cte_restart_sequence(sio);
			bio->bi_bdev = sc->cache_dev->bdev;
			bio->bi_sector = to_cache_sector(sio, bio->bi_sector);
			ssdcache_put_sio(sio);
			map_context->ptr = NULL;
			return DM_MAPIO_REMAPPED;
		}
		cte_start_sequence(sio, CTE_UPDATE);
		map_context->ptr = sio;
		ssdcache_get_sio(sio);
		bio_get(bio);
		if (sc->cache_mode == CACHE_IS_WRITETHROUGH) {
			bio->bi_bdev = sc->target_dev->bdev;
			write_to_cache(sio);
		} else {
			bio->bi_bdev = sc->cache_dev->bdev;
			bio->bi_sector = to_cache_sector(sio, bio->bi_sector);
			write_to_target(sio);
		}
		res = DM_MAPIO_REMAPPED;
	} else {
#ifdef SSD_DEBUG
		WPRINTK(sio, "%s: miss %llu state %08lx/%08lx",
			bio_data_dir(bio) == READ ? "read" : "write",
			(unsigned long long)data_sector, state,
			cte_bio_mask(sc, bio));
#endif
		bio_get(bio);
		map_context->ptr = sio;
		if (bio_data_dir(bio) == READ) {
			sc->cache_misses++;
			cte_start_sequence(sio, CTE_PREFETCH);
			bio->bi_bdev = sc->target_dev->bdev;
			return DM_MAPIO_REMAPPED;
		}
		ssdcache_get_sio(sio);
		cte_start_sequence(sio, CTE_UPDATE);
		if (sc->cache_mode == CACHE_IS_WRITETHROUGH) {
			bio->bi_bdev = sc->target_dev->bdev;
			write_to_cache(sio);
		} else {
			bio->bi_bdev = sc->cache_dev->bdev;
			bio->bi_sector = to_cache_sector(sio, bio->bi_sector);
			write_to_target(sio);
		}
		res = DM_MAPIO_REMAPPED;
	}
	return res;
}

static int ssdcache_endio(struct dm_target *ti, struct bio *bio,
			  int error, union map_info *map_context)
{
	struct ssdcache_io *sio = map_context->ptr;
	struct bio *clone = NULL;
	struct bio_vec *bvec;
	int i;

	if (!sio)
		return 0;

	if (error) {
		WPRINTK(sio, "finished with %u", error);
		sio_set_state(sio, CTE_INVALID);
		goto finish;
	}

	if (sio_is_state(sio, CTE_PREFETCH)) {
		/* Move to RESERVED */
		sio_set_state(sio, CTE_RESERVED);
		/* Setup clone for writing to cache device */
		clone = bio_clone(sio->bio, GFP_NOWAIT);
		clone->bi_rw |= WRITE;
		bio_for_each_segment(bvec, sio->bio, i)
			get_page(bvec->bv_page);
		bio_put(sio->bio);
		sio->bio = clone;
		ssdcache_schedule_sio(sio);
		return 0;
	} else if (sio_is_state(sio, CTE_UPDATE)) {
		/* Direct write completed, indirect write still in flight */
		sio_set_state(sio, CTE_WRITEBACK);
	} else if (sio_is_state(sio, CTE_WRITEBACK)) {
		/* Direct and indirect write completed */
		sio_set_state(sio, CTE_CLEAN);
	} else if (!sio_is_state(sio, CTE_CLEAN)) {
		WPRINTK(sio, "unhandled state %08lx",
			sio_get_state(sio));
	}

finish:
	ssdcache_put_sio(sio);

	return 0;
}


/*
 * Construct a ssdcache mapping: <target_dev_path> <cache_dev_path>
 */
static int ssdcache_ctr(struct dm_target *ti, unsigned int argc, char **argv)
{
	struct ssdcache_c *sc;
	unsigned long num_cte;
	unsigned long cdev_size;
	unsigned long long tdev_size;
	int r = 0;

	if (argc < 2 || argc > 6) {
		ti->error = "Invalid argument count";
		return -EINVAL;
	}

	sc = kzalloc(sizeof(*sc), GFP_KERNEL);
	if (sc == NULL) {
		ti->error = "dm-ssdcache: Cannot allocate ssdcache context";
		return -ENOMEM;
	}

	if (dm_get_device(ti, argv[0], dm_table_get_mode(ti->table),
			  &sc->target_dev)) {
		ti->error = "dm-ssdcache: Target device lookup failed";
		r = -EINVAL;
		goto bad;
	}

	if (dm_get_device(ti, argv[1], dm_table_get_mode(ti->table),
			  &sc->cache_dev)) {
		ti->error = "dm-ssdcache: Cache device lookup failed";
		dm_put_device(ti, sc->target_dev);
		r = -EINVAL;
		goto bad;
	}

	if (argc >= 3) {
		if (sscanf(argv[2], "%lu", &sc->block_size) != 1) {
			ti->error = "dm-ssdcache: Invalid blocksize";
			sc->block_size = DEFAULT_BLOCKSIZE;
		}
		if (sc->block_size < 512) {
			ti->error = "dm-ssdcache: blocksize too small";
			sc->block_size = DEFAULT_BLOCKSIZE;
		}
	} else {
		sc->block_size = DEFAULT_BLOCKSIZE;
	}

	if (argc >= 4) {
		if (!strncmp(argv[3], "lru", 3)) {
			sc->cache_strategy = CACHE_LRU;
		} else if (!strncmp(argv[3], "lfu", 3)) {
			sc->cache_strategy = CACHE_LFU;
		} else {
			ti->error = "dm-ssdcache: invalid strategy";
			sc->cache_strategy = default_cache_strategy;
		}
	} else {
		sc->cache_strategy = default_cache_strategy;
	}

	if (argc >= 5) {
		if (!strncmp(argv[4], "wb", 2)) {
			sc->cache_mode = CACHE_IS_WRITEBACK;
		} else if (!strncmp(argv[4], "wt", 2)) {
			sc->cache_mode = CACHE_IS_WRITETHROUGH;
		} else {
			ti->error = "dm-ssdcache: invalid cache mode";
			sc->cache_mode = default_cache_mode;
		}
	} else {
		sc->cache_mode = default_cache_mode;
	}

	sc->iocp = dm_io_client_create();
	if (IS_ERR(sc->iocp)) {
		r = PTR_ERR(sc->iocp);
		ti->error = "Failed to create io client\n";
		goto bad_io_client;
	}

	spin_lock_init(&sc->cmd_lock);

	cdev_size = i_size_read(sc->cache_dev->bdev->bd_inode);
	tdev_size = i_size_read(sc->target_dev->bdev->bd_inode);

	num_cte = cdev_size / sc->block_size / DEFAULT_ASSOCIATIVITY;

	/*
	 * Hash bit calculation might return a lower number
	 * for the possible number of ctes, so adjust that
	 * as well.
	 */
	sc->hash_bits = fls(num_cte) - 1;
	num_cte = (1UL << sc->hash_bits);
	sc->hash_shift = 0;

	DPRINTK("block size %lu, hash bits %lu/%lu, num cte %lu",
		sc->block_size, sc->hash_bits, sc->hash_shift, num_cte);

	INIT_RADIX_TREE(&sc->md_tree, GFP_ATOMIC);

	sc->data_offset = 0;
	sc->block_mask = to_sector(sc->block_size) - 1;
	sc->nr_sio = 0;
	ti->num_flush_requests = 1;
	ti->num_discard_requests = 1;
	ti->private = sc;
	return 0;

bad_io_client:
	dm_put_device(ti, sc->target_dev);
	dm_put_device(ti, sc->cache_dev);
bad:
	kfree(sc);
	return r;
}

static void ssdcache_dtr(struct dm_target *ti)
{
	struct ssdcache_c *sc = (struct ssdcache_c *) ti->private;
	struct ssdcache_te *cte;
	unsigned long pos = 0, nr_cmds, flags;
	struct ssdcache_md *cmds[MIN_CMD_NUM], *cmd;
	int i, j;

	do {
		nr_cmds = radix_tree_gang_lookup(&sc->md_tree,
						 (void **)cmds, pos,
						 MIN_CMD_NUM);
		for (i = 0; i < nr_cmds; i++) {
			pos = cmds[i]->hash;
			cmd = radix_tree_delete(&sc->md_tree, pos);
			spin_lock_irqsave(&cmd->lock, flags);
			for (j = 0; j < cmd->num_cte; j++) {
				if (cmd->te[j]) {
					cte = cmd->te[j];
					rcu_assign_pointer(cmd->te[j], NULL);
					spin_unlock_irqrestore(&cmd->lock, flags);
					synchronize_rcu();
					mempool_free(cte, _cte_pool);
					spin_lock_irqsave(&cmd->lock, flags);
				}
			}
			spin_unlock_irqrestore(&cmd->lock, flags);
			mempool_free(cmd, _cmd_pool);
		}
		pos++;
	} while (nr_cmds == MIN_CMD_NUM);

	dm_io_client_destroy(sc->iocp);
	dm_put_device(ti, sc->target_dev);
	dm_put_device(ti, sc->cache_dev);
	kfree(sc);
}

static int ssdcache_status(struct dm_target *ti, status_type_t type,
			 char *result, unsigned int maxlen)
{
	struct ssdcache_c *sc = (struct ssdcache_c *) ti->private;
	struct ssdcache_md *cmds[MIN_CMD_NUM], *cmd;
	unsigned long nr_elems, nr_cmds = 0, nr_ctes = 0, pos = 0;
	int i, j;

	rcu_read_lock();
	do {
		nr_elems = radix_tree_gang_lookup(&sc->md_tree,
						 (void **)cmds, pos,
						 MIN_CMD_NUM);
		for (i = 0; i < nr_elems; i++) {
			cmd = cmds[i];
			pos = cmd->hash;
			nr_cmds++;
			for (j = 0; j < cmd->num_cte; j++) {
				if (cmd->te[j])
					nr_ctes++;
			}
		}
		pos++;
	} while (nr_elems == MIN_CMD_NUM);
	rcu_read_unlock();
	switch (type) {
	case STATUSTYPE_INFO:
		snprintf(result, maxlen, "cmd %lu/%lu cte %lu/%lu "
			 "cache %lu/%lu/%lu/%lu",
			 nr_cmds, (1UL << sc->hash_bits), nr_ctes,
			 (1UL << sc->hash_bits) * DEFAULT_ASSOCIATIVITY,
			 sc->cache_misses, sc->cache_hits,
			 sc->cache_bypassed, sc->cache_evictions);
		break;

	case STATUSTYPE_TABLE:
		snprintf(result, maxlen, "%s %s %lu %s %s",
			 sc->target_dev->name, sc->cache_dev->name,
			 sc->block_size,
			 sc->cache_strategy == CACHE_LRU ? "lru" : "lfu",
			 sc->cache_mode == CACHE_IS_WRITETHROUGH ?
			 "wt" : "wb" );
		break;
	}
	return 0;
}

static int ssdcache_merge(struct dm_target *ti, struct bvec_merge_data *bvm,
			struct bio_vec *biovec, int max_size)
{
	struct ssdcache_c *sc = ti->private;
	struct request_queue *q = bdev_get_queue(sc->target_dev->bdev);

	if (!q->merge_bvec_fn)
		return max_size;

	bvm->bi_bdev = sc->target_dev->bdev;
	bvm->bi_sector = dm_target_offset(ti, bvm->bi_sector);

	return min(max_size, q->merge_bvec_fn(q, bvm, biovec));
}

static int ssdcache_iterate_devices(struct dm_target *ti,
				  iterate_devices_callout_fn fn, void *data)
{
	struct ssdcache_c *sc = ti->private;

	return fn(ti, sc->target_dev, 0, ti->len, data);
}

static struct target_type ssdcache_target = {
	.name   = "ssdcache",
	.version = {1, 1, 0},
	.module = THIS_MODULE,
	.ctr    = ssdcache_ctr,
	.dtr    = ssdcache_dtr,
	.map    = ssdcache_map,
	.end_io = ssdcache_endio,
	.status = ssdcache_status,
	.merge  = ssdcache_merge,
	.iterate_devices = ssdcache_iterate_devices,
};

int __init dm_ssdcache_init(void)
{
	int r;

	r = pool_init();
	if (r < 0) {
		DMERR("kmempool allocation failed: %d", r);
		return r;
	}
	_ssdcached_wq = create_singlethread_workqueue("kssdcached");
	if (!_ssdcached_wq) {
		DMERR("failed to start kssdcached");
		pool_exit();
		return -ENOMEM;
	}
	INIT_WORK(&_ssdcached_work, do_sio);

	r = dm_register_target(&ssdcache_target);
	if (r < 0) {
		DMERR("register failed %d", r);
		destroy_workqueue(_ssdcached_wq);
		pool_exit();
	}

	return r;
}

void dm_ssdcache_exit(void)
{
	pool_exit();
	destroy_workqueue(_ssdcached_wq);
	dm_unregister_target(&ssdcache_target);
}

module_init(dm_ssdcache_init);
module_exit(dm_ssdcache_exit);

MODULE_DESCRIPTION(DM_NAME " cache target");
MODULE_AUTHOR("Hannes Reinecke <hare@suse.de>");
MODULE_LICENSE("GPL");
