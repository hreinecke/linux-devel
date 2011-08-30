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

#ifdef SSD_DEBUG
#define DPRINTK( s, arg... ) printk(DM_MSG_PREFIX s "\n", ##arg)
#define WPRINTK( w, s, arg... ) printk(DM_MSG_PREFIX "%lu: %s (cte %lx:%lx): "\
				       s "\n", (w)->nr, __FUNCTION__, \
				       (w)->cmd->index, \
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

static enum ssdcache_mode_t default_cache_mode = CACHE_IS_WRITETHROUGH;

struct ssdcache_md;
struct ssdcache_io;

struct ssdcache_te {
	unsigned long index;	/* Offset within table entry block */
	unsigned long atime;	/* Timestamp of the block's last access */
	unsigned long state;    /* State bitmap */
	sector_t sector;	/* Sector number on target device */
	struct ssdcache_md *md; /* Backlink to metadirectory */
	struct bio_list writeback;
	struct rcu_head rcu;
};

struct ssdcache_md {
	spinlock_t lock;	/* Lock to protect operations on the bio list */
	unsigned long index;	/* Hash number */
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
	unsigned int assoc;
	unsigned long nr_sio;
	enum ssdcache_mode_t cache_mode;
	unsigned long cache_hits;
	unsigned long cache_bypassed;
	unsigned long cache_requeue;
};

struct ssdcache_io {
	struct list_head list;
	unsigned long nr;
	struct ssdcache_c *sc;
	struct ssdcache_md *cmd;
	long cte_idx;
	struct bio *bio;
	unsigned long bio_mask;
	struct bio_vec *bvec;
	sector_t bvec_sector;
	unsigned long bvec_count;
	struct page *pg_head, *pg_tail;
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
	_sio_cache = kmem_cache_create("ssdcache-work",
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
	cmd->index = hash_number;
	cmd->num_cte = (1UL << sc->hash_shift) * sc->assoc;
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
		BUG_ON(cmd->index != hash_number);
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

	if (!bio)
		return 0;

	for (i = 0; i < to_sector(sc->block_size); i++) {
		if (i >= cte_bio_offset(sc, bio) &&
		    i < cte_bio_offset(sc, bio) + to_sector(bio->bi_size)) {
			mask |= (0xFUL << (i * 4));
		}
	}

	return mask;
}

static unsigned long cte_block_mask(struct ssdcache_c *sc)
{
	unsigned long mask = 0;
	int i;

	for (i = 0; i < to_sector(sc->block_size); i++) {
		mask |= 0xFUL << (i * 4);
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
		case CTE_CLEAN:
			break;
		default:
			illegal_transition++;
		}
		break;
	case CTE_RESERVED:
		if (oldstate != CTE_PREFETCH)
			illegal_transition++;
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
		if (oldstate != CTE_DIRTY)
			illegal_transition++;
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
	unsigned long state = 0x11111111UL;
	struct ssdcache_te *cte;

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
	unsigned long oldstate, offset;
	enum cte_state tmpstate = CTE_INVALID;
	int match = 0, i;

	if (!sio || !sio->bio || sio->cte_idx < 0)
		return false;

	oldstate = sio_get_state(sio);
	offset= cte_bio_offset(sio->sc, sio->bio);
	for (i = 0; i < to_sector(sio->bio->bi_size); i++) {
		tmpstate = (oldstate >> ((offset + i) * 4)) & 0xF;
		match += (tmpstate == state);
	}
	return (match == to_sector(sio->bio->bi_size));
}

static void sio_set_state(struct ssdcache_io *sio, enum cte_state state)
{
	struct ssdcache_te *oldcte, *newcte = NULL;
	unsigned long newstate, oldstate, flags, state_shift;
	int i, match = 0;

	if (!sio || sio->cte_idx < 0)
		return;

	oldstate = sio_get_state(sio);
	newstate = sio_update_state(sio, oldstate, state);

	/* Check if we should drop the old cte */
	for (i = 0; i < to_sector(sio->sc->block_size); i++) {
		state_shift = (i * 4);
		if (((newstate >> state_shift) & 0xF) == CTE_INVALID)
			match++;
	}
	if (match < to_sector(sio->sc->block_size)) {
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
		match += ((tmpstate == CTE_PREFETCH) ||
			  (tmpstate == CTE_RESERVED) ||
			  (tmpstate == CTE_UPDATE));
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

	if (state == CTE_PREFETCH)
		sio->bio_mask = cte_block_mask(sio->sc);
	else
		sio->bio_mask = cte_bio_mask(sio->sc, sio->bio);

	newstate = sio_update_state(sio, oldstate, state);
	WPRINTK(sio, "state %08lx:%08lx", oldstate, newstate);

	spin_lock_irqsave(&sio->cmd->lock, flags);
	oldcte = sio->cmd->te[sio->cte_idx];
	*newcte = *oldcte;
	newcte->sector = cte_bio_align(sio->sc, sio->bio);
	newcte->state = newstate;
	newcte->atime = jiffies;
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
	rcu_assign_pointer(sio->cmd->te[sio->cte_idx], newcte);
	spin_unlock_irqrestore(&sio->cmd->lock, flags);
	call_rcu(&oldcte->rcu, cte_reset);
}

static inline bool cte_match_sector(struct ssdcache_c *sc,
				    struct ssdcache_te *cte,
				    struct bio *bio)
{
	bool match = false;
	sector_t sector;

	if (!cte)
		return match;

	rcu_read_lock();
	sector = rcu_dereference(cte)->sector;
	rcu_read_unlock();
	if (!cte_is_state(sc, cte, bio, CTE_INVALID)) {
		match = (cte_bio_align(sc, bio) == sector);
	}

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
	INIT_LIST_HEAD(&sio->list);
	return sio;
}

static void ssdcache_destroy_sio(struct ssdcache_io *sio)
{
	BUG_ON(!list_empty(&sio->list));

	if (sio->bvec)
		kfree(sio->bvec);
	if (sio->pg_head)
		__free_page(sio->pg_head);
	if (sio->pg_tail)
		__free_page(sio->pg_tail);

	mempool_free(sio, _sio_pool);
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
		DPRINTK("do_sio: %d items dequeued, %d queues empty",
			items, empty);
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
 * finish_prefetch
 *
 * Finish I/O for cte in state PREFETCH
 * Data has been read from the target device.
 * Return I/O to upper layers and setup I/O
 * to write to cache device.
 * CTE has been locked on entry.
 */
static int finish_prefetch(struct ssdcache_io *sio, unsigned long error)
{
	struct bio *clone;
	struct bio_vec *bvec;
	int i;

	if (error) {
		WPRINTK(sio, "error, mark cte invalid");
		sio_set_state(sio, CTE_INVALID);
		bio_endio(sio->bio, -EIO);
		sio->bio = NULL;
		sio->bio_mask = 0;
		ssdcache_destroy_sio(sio);
		return 0;
	}
	/* Move to RESERVED */
	WPRINTK(sio, "mark cte reserved");
	sio_set_state(sio, CTE_RESERVED);
	/* Setup clone for writing to cache device */
	WPRINTK(sio, "write to cache device");
	clone = bio_clone(sio->bio, GFP_NOWAIT);
	clone->bi_rw |= WRITE;
	bio_for_each_segment(bvec, sio->bio, i)
		get_page(bvec->bv_page);
	bio_endio(sio->bio, 0);
	sio->bio = clone;
	return 1;
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
	struct ssdcache_te *oldcte, *newcte;
	struct bio_vec *bvec;
	enum cte_state state;
	unsigned long oldstate, newstate, flags;

	oldstate = sio_get_state(sio);

	newcte = mempool_alloc(_cte_pool, GFP_NOWAIT | __GFP_ZERO);
	if (!newcte)
		return 1;

	if (error) {
		WPRINTK(sio, "error, mark cte invalid");
		state = CTE_INVALID;
	} else {
		WPRINTK(sio, "mark cte clean");
		state = CTE_CLEAN;
	}

	newstate = sio_update_state(sio, oldstate, state);
	/* Release bio */
	bio_for_each_segment(bvec, sio->bio, i)
		put_page(bvec->bv_page);
	bio_put(sio->bio);

	spin_lock_irqsave(&sio->cmd->lock, flags);
	oldcte = sio->cmd->te[sio->cte_idx];

	*newcte = *oldcte;
	newcte->state = newstate;
	newcte->atime = jiffies;
	rcu_assign_pointer(sio->cmd->te[sio->cte_idx], newcte);
	spin_unlock_irqrestore(&sio->cmd->lock, flags);
	call_rcu(&oldcte->rcu, cte_reset);

	ssdcache_destroy_sio(sio);
	return 0;
}

static int finish_update(struct ssdcache_io *sio, unsigned long error)
{
	int i;
	struct ssdcache_te *oldcte, *newcte;
	enum cte_state state;
	struct bio *clone = NULL;
	struct bio_vec *bvec;
	unsigned long oldstate, newstate, flags;

	oldstate = sio_get_state(sio);

	newcte = mempool_alloc(_cte_pool, GFP_NOWAIT | __GFP_ZERO);
	if (!newcte)
		return 1;

	if (error || sio->sc->cache_mode == CACHE_IS_WRITEBACK) {
		if (error) {
			/* Move back to INVALID */
			WPRINTK(sio, "error, mark cte invalid");
			state = CTE_INVALID;
		} else {
			/* Move to DIRTY */
			WPRINTK(sio, "mark cte dirty");
			state = CTE_DIRTY;
		}
		newstate = sio_update_state(sio, oldstate, state);
	} else {
		WPRINTK(sio, "mark cte dirty");
		state = CTE_DIRTY;

		newstate = sio_update_state(sio, oldstate, state);

		/* Setup bio clone for writing to target device */
		clone = bio_clone(sio->bio, GFP_NOWAIT);
		bio_for_each_segment(bvec, sio->bio, i)
			get_page(bvec->bv_page);

	}
	bio_endio(sio->bio, error?-EIO:0);
	sio->bio = NULL;

	spin_lock_irqsave(&sio->cmd->lock, flags);
	oldcte = sio->cmd->te[sio->cte_idx];

	*newcte = *oldcte;
	newcte->state = newstate;
	newcte->atime = jiffies;
	if (clone)
		bio_list_add(&newcte->writeback, clone);
	rcu_assign_pointer(sio->cmd->te[sio->cte_idx], newcte);
	spin_unlock_irqrestore(&sio->cmd->lock, flags);
	call_rcu(&oldcte->rcu, cte_reset);

	if (!clone) {
		ssdcache_destroy_sio(sio);
		sio = NULL;
	}
	return sio ? 1 : 0;
}

static int finish_nochange(struct ssdcache_io *sio, unsigned long error)
{
	if (error) {
		WPRINTK(sio, "error, mark cte invalid");
		sio_set_state(sio, CTE_INVALID);
	}
	/* Finish I/O and complete sio item */
	bio_endio(sio->bio, error ? -EIO : 0);
	sio->bio = NULL;
	/* Complete sio item */
	ssdcache_destroy_sio(sio);
	return 0;
}

static int finish_writeback(struct ssdcache_io *sio, unsigned long error)
{
	int i;
	struct ssdcache_te *oldcte, *newcte;
	struct bio_vec *bvec;
	enum cte_state state;
	unsigned long oldstate, newstate, flags;

	oldstate = sio_get_state(sio);

	newcte = mempool_alloc(_cte_pool, GFP_NOWAIT | __GFP_ZERO);
	if (!newcte)
		return 1;

	if (error) {
		WPRINTK(sio, "error, mark cte dirty");
		state = CTE_DIRTY;
	} else {
		WPRINTK(sio, "mark cte clean");
		state = CTE_CLEAN;
	}

	newstate = sio_update_state(sio, oldstate, state);
	/* Release bio */
	bio_for_each_segment(bvec, sio->bio, i)
		put_page(bvec->bv_page);
	bio_put(sio->bio);

	spin_lock_irqsave(&sio->cmd->lock, flags);
	oldcte = sio->cmd->te[sio->cte_idx];
	*newcte = *oldcte;
	newcte->state = newstate;
	newcte->atime = jiffies;
	rcu_assign_pointer(sio->cmd->te[sio->cte_idx], newcte);
	spin_unlock_irqrestore(&sio->cmd->lock, flags);
	call_rcu(&oldcte->rcu, cte_reset);

	ssdcache_destroy_sio(sio);
	return 0;
}

static void io_callback(unsigned long error, void *context)
{
	struct ssdcache_io *sio = context;
	int requeue = 0;

	if (error)
		WPRINTK(sio, "finished with %lu", error);
	BUG_ON(!sio->bio);
	if (sio_is_state(sio, CTE_PREFETCH)) {
		requeue = finish_prefetch(sio, error);
	} else if (sio_is_state(sio, CTE_RESERVED)) {
		requeue = finish_reserved(sio, error);
	} else if (sio_is_state(sio, CTE_UPDATE)) {
		requeue = finish_update(sio, error);
	} else if (sio_is_state(sio, CTE_WRITEBACK)) {
		requeue = finish_writeback(sio, error);
	} else if (sio_is_state(sio, CTE_CLEAN) ||
		   sio_is_state(sio, CTE_DIRTY)) {
		requeue = finish_nochange(sio, error);
	} else {
		WPRINTK(sio, "unhandled state %08lx",
			sio_get_state(sio));
		if (sio->bio) {
			bio_endio(sio->bio, error?-EIO:0);
			sio->bio = NULL;
		}
		ssdcache_destroy_sio(sio);
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
	cmd_offset = to_sector(sio->cmd->index * sio->cmd->num_cte * sio->sc->block_size);
	cache_sector = sio->sc->data_offset + cmd_offset + cte_offset + sector_offset;

	return cache_sector;
}

static int write_to_cache(struct ssdcache_io *sio)
{
	struct dm_io_region cache;
	struct dm_io_request iorq;
	struct bio_vec *bvec;
	sector_t sector;

	cache.bdev = sio->sc->cache_dev->bdev;
	if (sio->bvec) {
		bvec = sio->bvec;
		sector = sio->bvec_sector;
		cache.count = sio->bvec_count;
	} else {
		bvec = sio->bio->bi_io_vec + sio->bio->bi_idx;
		sector = sio->bio->bi_sector;
		cache.count = to_sector(sio->bio->bi_size);
	}
	cache.sector = to_cache_sector(sio, sector);

	iorq.bi_rw = WRITE;
	iorq.mem.type = DM_IO_BVEC;
	iorq.mem.ptr.bvec = bvec;
	iorq.notify.fn = io_callback;
	iorq.notify.context = sio;
	iorq.client = sio->sc->iocp;

	return dm_io(&iorq, 1, &cache, NULL);
}

static int read_from_cache(struct ssdcache_io *sio)
{
	struct dm_io_region cache;
	struct dm_io_request iorq;
	struct bio *bio = sio->bio;

	cache.bdev = sio->sc->cache_dev->bdev;
	cache.sector = to_cache_sector(sio, bio->bi_sector);
	cache.count = to_sector(bio->bi_size);

	sio->bvec = NULL;
	sio->pg_head = sio->pg_tail = NULL;
	sio->bvec_count = 0;
	sio->bvec_sector = -1;

	iorq.bi_rw = READ;
	iorq.mem.type = DM_IO_BVEC;
	iorq.mem.ptr.bvec = bio->bi_io_vec + bio->bi_idx;
	iorq.notify.fn = io_callback;
	iorq.notify.context = sio;
	iorq.client = sio->sc->iocp;

	return dm_io(&iorq, 1, &cache, NULL);
}

static int write_to_target(struct ssdcache_io *sio)
{
	struct dm_io_region target;
	struct dm_io_request iorq;

	target.bdev = sio->sc->target_dev->bdev;
	target.sector = sio->bio->bi_sector;
	target.count = to_sector(sio->bio->bi_size);

	sio->bvec = NULL;
	sio->pg_head = sio->pg_tail = NULL;
	sio->bvec_count = target.count;
	sio->bvec_sector = target.sector;

	iorq.bi_rw = WRITE;
	iorq.mem.type = DM_IO_BVEC;
	iorq.mem.ptr.bvec = sio->bio->bi_io_vec + sio->bio->bi_idx;
	iorq.notify.fn = io_callback;
	iorq.notify.context = sio;
	iorq.client = sio->sc->iocp;

	return dm_io(&iorq, 1, &target, NULL);
}

static int read_from_target(struct ssdcache_io *sio, int pad)
{
	struct dm_io_region target;
	struct dm_io_request iorq;
	struct bio *bio = sio->bio;
	struct bio_vec *bvec;
	unsigned long offset, head, tail;
	loff_t size;
	int i, j, length, nr_vecs;

	if (pad) {
		/*
		 * Pad I/O to cache blocksize
		 */
		offset = cte_bio_offset(sio->sc, bio);
		head = to_bytes(offset);
		tail = (bio->bi_size + head) % sio->sc->block_size;
		size = i_size_read(sio->sc->target_dev->bdev->bd_inode);

		if (to_bytes(bio->bi_sector) + bio->bi_size + tail > size) {
			/* Padding beyond end of device */
			tail = size - to_bytes(bio->bi_sector) - bio->bi_size;
		}
	} else {
		head = tail = 0;
	}
	target.bdev = sio->sc->target_dev->bdev;
	target.sector = bio->bi_sector - to_sector(head);
	target.count = to_sector(bio->bi_size + head + tail);

	nr_vecs = bio->bi_vcnt - bio->bi_idx;
	sio->bvec = NULL;
	sio->pg_head = sio->pg_tail = NULL;
	if (head) {
		sio->pg_head = alloc_page(GFP_KERNEL);
		if (!sio->pg_head) {
			DMERR("read_from_target: no memory");
			return ENOMEM;
		}
		nr_vecs++;
	}
	if (tail) {
		sio->pg_tail = alloc_page(GFP_KERNEL);
		if (!sio->pg_tail) {
			DMERR("read_from_target: no memory");
			if (sio->pg_head) {
				kfree(sio->pg_head);
				sio->pg_head = NULL;
			}
			return ENOMEM;
		}
		nr_vecs++;
	}
	sio->bvec_count = target.count;
	sio->bvec_sector = target.sector;
	if (head || tail) {
		bvec = kmalloc(nr_vecs * sizeof(*bvec), GFP_NOIO);
		if (!bvec) {
			DMERR("read_from_target: no memory for bvec");
			return ENOMEM;
		}
		i = 0;
		if (head) {
			bvec[i].bv_page = sio->pg_head;
			bvec[i].bv_len = head;
			bvec[i].bv_offset = 0;
			i++;
		}
		length = bio->bi_size;
		j = bio->bi_idx;
		while (length) {
			bvec[i] = bio->bi_io_vec[j];
			length -= bvec[i].bv_len;
			i++; j++;
		}
		if (tail) {
			bvec[i].bv_page = sio->pg_tail;
			bvec[i].bv_len = tail;
			bvec[i].bv_offset = 0;
		}
		sio->bvec = bvec;
	} else {
		bvec = bio->bi_io_vec + bio->bi_idx;
	}
	iorq.bi_rw = READ;
	iorq.mem.type = DM_IO_BVEC;
	iorq.mem.ptr.bvec = bvec;
	iorq.notify.fn = io_callback;
	iorq.notify.context = sio;
	iorq.client = sio->sc->iocp;

	return dm_io(&iorq, 1, &target, NULL);
}

static int copy_bvec_to_bio(struct ssdcache_io *sio, struct bio *bio)
{
	struct bio_vec *bvec;
	int i, ret = 0;

	for (i = 0; i < sio->bvec_count; i++) {
		bvec = &sio->bvec[i];
		get_page(bvec->bv_page);
		ret = bio_add_page(bio, bvec->bv_page, bvec->bv_len,
				   bvec->bv_offset);
		if (ret)
			break;
	}

	return ret;
}

static int do_io(struct ssdcache_io *sio)
{
	BUG_ON(sio->cte_idx < 0);
	if (!sio->bio) {
		/* Fetch first writeback bio */
		sio->bio = sio_fetch_writeback_bio(sio);
		if (!sio->bio) {
			WPRINTK(sio, "no bio");
			ssdcache_destroy_sio(sio);
			return 0;
		} else {
			WPRINTK(sio, "fetch writeback bio");
			cte_start_sequence(sio, CTE_WRITEBACK);
		}
	}
	BUG_ON(bio_data_dir(sio->bio) == READ);
	if (sio_is_state(sio, CTE_RESERVED)) {
		WPRINTK(sio, "start reserved");
		return write_to_cache(sio);
	}
	if (sio_is_state(sio, CTE_UPDATE)) {
		WPRINTK(sio, "start update");
		sio->bvec = NULL;
		return write_to_cache(sio);
	}
	if (sio_is_state(sio, CTE_WRITEBACK)) {
		WPRINTK(sio, "start writeback");
		return write_to_target(sio);
	}
	WPRINTK(sio, "unhandled state %08lx", sio_get_state(sio));
	return EINVAL;
}

/*
 * Cache lookup and I/O handling
 */
static unsigned long hash_block(struct ssdcache_c *sc, sector_t sector)
{
	unsigned long value, hash_number, offset, hash_mask;

	hash_mask = (1UL << sc->hash_shift) - 1;
	value = sector >> sc->hash_shift;
	offset = sector & hash_mask;
	hash_number = (unsigned long)hash_64(value, sc->hash_bits - sc->hash_shift);

	return (hash_number << sc->hash_shift) | offset;
}

static int cte_match(struct ssdcache_c *sc,
		     struct ssdcache_md *cmd,
		     struct bio *bio)
{
	sector_t data_sector;
	struct ssdcache_te *cte;
	unsigned long cte_atime, clean_atime, oldest_atime;
	int invalid = -1, clean, oldest, i, busy = 0;

	data_sector = cte_bio_align(sc, bio);
	clean_atime = oldest_atime = jiffies;
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
	}

	if (invalid != -1)
		return invalid;

	if (clean != -1) {
		DPRINTK("%s (cte %lx:%x): Use clean", __FUNCTION__,
			cmd->index, clean);
		return clean;
	}

	if (oldest != -1) {
		DPRINTK("%s (cte %lx:%x): Use oldest, %d/%d ctes busy",
			__FUNCTION__, cmd->index, oldest, busy, i);
		return oldest;
	}

	DPRINTK("%s (cte %lx:%x): %d ctes busy", __FUNCTION__,
		cmd->index, -1, busy);
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
	int res = DM_MAPIO_SUBMITTED;

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
		ssdcache_destroy_sio(sio);
		return DM_MAPIO_REMAPPED;
	}

	/* Lookup cmd */
	sio->cmd = cmd_insert(sc, hash_number);
	if (!sio->cmd) {
		DPRINTK("cache insertion failure");
		sc->cache_bypassed++;
		bio->bi_bdev = sc->target_dev->bdev;
		ssdcache_destroy_sio(sio);
		return DM_MAPIO_REMAPPED;
	}

	sio->cte_idx = cte_match(sc, sio->cmd, bio);
	if (sio->cte_idx < 0) {
		DPRINTK("bypass cte sio");
		sc->cache_bypassed++;
		bio->bi_bdev = sc->target_dev->bdev;
		ssdcache_destroy_sio(sio);
		return DM_MAPIO_REMAPPED;
	}

	cte = cte_insert(sc, sio->cmd, sio->cte_idx);
	BUG_ON(!cte);
	state = sio_get_state(sio);
	sio->bio = bio;

	if (cte_match_sector(sc, cte, bio)) {
		/* Cache hit */
		sc->cache_hits++;
		if (cte_is_busy(sc, cte, bio)) {
			if (bio_data_dir(bio) == READ) {
				/* Bypass cache */
				DPRINTK("%s: (cte %lx:%lx): bypass busy cte",
					__FUNCTION__, sio->cmd->index,
					sio->cte_idx);
				sc->cache_bypassed++;
				bio->bi_bdev = sc->target_dev->bdev;
				ssdcache_destroy_sio(sio);
				return DM_MAPIO_REMAPPED;
			} else {
				/* Transaction in progress, delay */
				DPRINTK("%s: (cte %lx:%lx): delay cache hit",
					__FUNCTION__, sio->cmd->index,
					sio->cte_idx);
				sc->cache_requeue++;
				ssdcache_destroy_sio(sio);
				return DM_MAPIO_REQUEUE;
			}
		}
		sio->nr = ++sc->nr_sio;
		WPRINTK(sio, "%s hit %llu state %08lx/%08lx",
			bio_data_dir(bio) == READ ? "read" : "write",
			(unsigned long long)data_sector, state,
			cte_bio_mask(sc, bio));
		if (bio_data_dir(bio) == READ) {
			cte_restart_sequence(sio);
			res = read_from_cache(sio);
		} else if (sc->cache_mode == CACHE_IS_WRITETHROUGH) {
			sio_set_state(sio, CTE_INVALID);
			bio->bi_bdev = sc->target_dev->bdev;
			ssdcache_destroy_sio(sio);
			res = DM_MAPIO_REMAPPED;
		} else {
			cte_start_sequence(sio, CTE_UPDATE);
			res = write_to_cache(sio);
		}
	} else {
		if (sio_is_state(sio, CTE_DIRTY)) {
			DPRINTK("%s: (cte %lx:%lx): wait for writeback",
				__FUNCTION__, sio->cmd->index, sio->cte_idx);
			sc->cache_requeue++;
			ssdcache_destroy_sio(sio);
			return DM_MAPIO_REQUEUE;
		}
		sio->nr = ++sc->nr_sio;
		WPRINTK(sio, "%s: miss %llu state %08lx/%08lx",
			bio_data_dir(bio) == READ ? "read" : "write",
			(unsigned long long)data_sector, state,
			cte_bio_mask(sc, bio));
		if (bio_data_dir(bio) == READ) {
			cte_start_sequence(sio, CTE_PREFETCH);
			res = read_from_target(sio, 1);
		} else if (sc->cache_mode == CACHE_IS_WRITETHROUGH) {
			bio->bi_bdev = sc->target_dev->bdev;
			ssdcache_destroy_sio(sio);
			return DM_MAPIO_REMAPPED;
		} else {
			cte_start_sequence(sio, CTE_UPDATE);
			res = write_to_cache(sio);
		}
	}
	return res;
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
		if (sscanf(argv[3], "%u", &sc->assoc) != 1) {
			ti->error = "dm-ssdcache: Invalid associativity";
			sc->block_size = DEFAULT_BLOCKSIZE;
		}
	} else {
		sc->assoc = DEFAULT_ASSOCIATIVITY;
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

	num_cte = cdev_size / sc->block_size / sc->assoc;

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
			pos = cmds[i]->index;
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
			pos = cmd->index;
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
		snprintf(result, maxlen, "cmd %lu/%lu cte %lu/%lu cache %lu/%lu/%lu/%lu",
			 nr_cmds, (1UL << sc->hash_bits), nr_ctes,
			 (1UL << sc->hash_bits) * sc->assoc,
			 sc->nr_sio, sc->cache_hits,
			 sc->cache_requeue, sc->cache_bypassed);
		break;

	case STATUSTYPE_TABLE:
		snprintf(result, maxlen, "%s %s %lu %u %s",
			 sc->target_dev->name, sc->cache_dev->name,
			 sc->block_size, sc->assoc,
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
