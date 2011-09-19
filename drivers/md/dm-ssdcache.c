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
#define WPRINTK( w, s, arg... ) printk(DM_MSG_PREFIX "%lu: %s (cte %lx:%02lx): "\
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

#define DEFAULT_BLOCKSIZE	8
#define DEFAULT_ASSOCIATIVITY	64

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
	DECLARE_BITMAP(clean, DEFAULT_BLOCKSIZE);
	DECLARE_BITMAP(target_busy, DEFAULT_BLOCKSIZE);
	DECLARE_BITMAP(cache_busy, DEFAULT_BLOCKSIZE);
	sector_t sector;	/* Sector number on target device */
	struct ssdcache_md *md; /* Backlink to metadirectory */
	struct rcu_head rcu;
};

struct ssdcache_md {
	spinlock_t lock;	/* Lock to protect operations on the bio list */
	unsigned long hash;	/* Hash number */
	unsigned int num_cte;	/* Number of table entries */
	unsigned long atime;
	struct ssdcache_ctx *sc;
	struct ssdcache_te *te[DEFAULT_ASSOCIATIVITY];	/* RCU Table entries */
};

struct ssdcache_ctx {
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
	unsigned long sio_active;
	unsigned long cte_active;
	enum ssdcache_mode_t cache_mode;
	enum ssdcache_strategy_t cache_strategy;
	unsigned long cache_misses;
	unsigned long cache_hits;
	unsigned long cache_busy;
	unsigned long cache_bypassed;
	unsigned long cache_overruns;
	unsigned long cache_evictions;
	unsigned long cache_failures;
};

struct ssdcache_io {
	struct list_head list;
	spinlock_t lock;
	struct kref kref;
	unsigned long nr;
	struct ssdcache_ctx *sc;
	struct ssdcache_md *cmd;
	long cte_idx;
	struct bio *bio;
	struct bio *writeback_bio;
	unsigned long bio_sector;
	DECLARE_BITMAP(bio_mask, DEFAULT_BLOCKSIZE);
	unsigned long error;
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
	/* Permanent states */
	CTE_INVALID,	/* Sector not valid */
	CTE_CLEAN,	/* Sector valid, Cache and target date identical */
	/* Transient states */
	CTE_PREFETCH,	/* Sector valid, Read target data */
	CTE_UPDATE,	/* Sector valid, Write cache data */
};

enum cte_match_t {
	CTE_READ_CLEAN,
	CTE_READ_BUSY,
	CTE_READ_INVALID,
	CTE_READ_MISS,
	CTE_WRITE_CLEAN,
	CTE_WRITE_BUSY,
	CTE_WRITE_INVALID,
	CTE_WRITE_MISS,
	CTE_WRITE_DONE,
	CTE_WRITE_ERROR,
	CTE_LOOKUP_FAILED,
};

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

static inline struct ssdcache_md *cmd_lookup(struct ssdcache_ctx *sc,
					     unsigned long hash_number)
{
	struct ssdcache_md *cmd;

	rcu_read_lock();
	cmd = radix_tree_lookup(&sc->md_tree, hash_number);
	rcu_read_unlock();
	return cmd;
}

static inline struct ssdcache_md *cmd_insert(struct ssdcache_ctx *sc,
					     unsigned long hash_number)
{
	struct ssdcache_md *cmd;

	cmd = mempool_alloc(_cmd_pool, GFP_NOIO | __GFP_ZERO);
	if (!cmd)
		return NULL;
	cmd->hash = hash_number;
	cmd->num_cte = (1UL << sc->hash_shift) * DEFAULT_ASSOCIATIVITY;
	spin_lock_init(&cmd->lock);
	cmd->atime = jiffies;
	cmd->sc = sc;

	if (radix_tree_preload(GFP_NOIO)) {
		mempool_free(cmd, _cmd_pool);
		return NULL;
	}

	spin_lock(&sc->cmd_lock);
	if (radix_tree_insert(&sc->md_tree, hash_number, cmd)) {
		mempool_free(cmd, _cmd_pool);
		cmd = radix_tree_lookup(&sc->md_tree, hash_number);
		BUG_ON(!cmd);
		BUG_ON(cmd->hash != hash_number);
	}
	spin_unlock(&sc->cmd_lock);

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
	{ CTE_INVALID, "CTE_INVALID"},
	{ CTE_CLEAN, "CTE_CLEAN" },
	{ CTE_PREFETCH, "CTE_PREFETCH" },
	{ CTE_UPDATE, "CTE_UPDATE" },
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

#define cte_bio_align(s,b) ((b)->bi_sector & ~(s)->block_mask)
#define cte_bio_offset(s,b) ((b)->bi_sector & (s)->block_mask)

void sio_bio_mask(struct ssdcache_io *sio, struct bio *bio)
{
	unsigned long offset = cte_bio_offset(sio->sc, bio);
	int i;

	for (i = 0; i < bio_sectors(bio); i++) {
		set_bit(offset + i, sio->bio_mask);
	}
}

struct ssdcache_te * cte_new(struct ssdcache_ctx *sc, struct ssdcache_md *cmd,
			     unsigned int index)
{
	struct ssdcache_te *newcte;

	newcte = mempool_alloc(_cte_pool, GFP_NOWAIT | __GFP_ZERO);
	if (!newcte)
		return NULL;

	newcte->index = index;
	newcte->atime = jiffies;
	newcte->count = 1;
	newcte->md = cmd;
	sc->cte_active++;
	return newcte;
}

static void cte_reset(struct rcu_head *rp)
{
	struct ssdcache_te *cte = container_of(rp, struct ssdcache_te, rcu);

	cte->md->sc->cte_active--;
	mempool_free(cte, _cte_pool);
}

static bool cte_is_clean(struct ssdcache_te *cte, unsigned long *mask)
{
	DECLARE_BITMAP(tmpmask, DEFAULT_BLOCKSIZE);

	bitmap_and(tmpmask, cte->clean, mask, DEFAULT_BLOCKSIZE);
	return bitmap_equal(tmpmask, mask, DEFAULT_BLOCKSIZE);
}

static bool cte_cache_is_busy(struct ssdcache_te *cte, unsigned long *mask)
{
	DECLARE_BITMAP(tmpmask, DEFAULT_BLOCKSIZE);

	bitmap_and(tmpmask, cte->cache_busy, mask, DEFAULT_BLOCKSIZE);
	return !bitmap_empty(tmpmask, DEFAULT_BLOCKSIZE);
}

static bool cte_target_is_busy(struct ssdcache_te *cte, unsigned long *mask)
{
	DECLARE_BITMAP(tmpmask, DEFAULT_BLOCKSIZE);

	bitmap_and(tmpmask, cte->target_busy, mask, DEFAULT_BLOCKSIZE);
	return !bitmap_empty(tmpmask, DEFAULT_BLOCKSIZE);
}

static bool sio_cache_is_busy(struct ssdcache_io *sio)
{
	struct ssdcache_te *cte;
	bool match = false;

	if (!sio->cmd)
		return false;
	BUG_ON(sio->cte_idx == -1);

	rcu_read_lock();
	cte = rcu_dereference(sio->cmd->te[sio->cte_idx]);
	if (cte)
		match = cte_cache_is_busy(cte, sio->bio_mask);
	rcu_read_unlock();

	return match;
}

static bool sio_target_is_busy(struct ssdcache_io *sio)
{
	struct ssdcache_te *cte;
	bool match = false;

	if (!sio->cmd || sio->cte_idx == -1)
		return false;

	rcu_read_lock();
	cte = rcu_dereference(sio->cmd->te[sio->cte_idx]);
	if (cte)
		match = cte_target_is_busy(cte, sio->bio_mask);
	rcu_read_unlock();

	return match;
}

static bool sio_match_sector(struct ssdcache_io *sio)
{
	struct ssdcache_te *cte;
	sector_t cte_sector;

	if (!sio || !sio->cmd || sio->cte_idx == -1)
		return false;
	rcu_read_lock();
	cte = rcu_dereference(sio->cmd->te[sio->cte_idx]);
	if (cte)
		cte_sector = cte->sector;
	else
		cte_sector = sio->bio_sector - 1;
	rcu_read_unlock();

	return (sio->bio_sector == cte_sector);
}

static void sio_cleanup_cte(struct ssdcache_io *sio)
{
	struct ssdcache_te *newcte, *oldcte;
	bool is_invalid = false;

	BUG_ON(!sio);
	BUG_ON(!sio->cmd);
	BUG_ON(sio->cte_idx == -1);

	rcu_read_lock();
	oldcte = rcu_dereference(sio->cmd->te[sio->cte_idx]);
	if (oldcte &&
	    bitmap_empty(oldcte->clean, DEFAULT_BLOCKSIZE) &&
	    bitmap_empty(oldcte->target_busy, DEFAULT_BLOCKSIZE) &&
	    bitmap_empty(oldcte->cache_busy, DEFAULT_BLOCKSIZE))
		is_invalid = true;
	rcu_read_unlock();

	if (!is_invalid) {
		newcte = cte_new(sio->sc, sio->cmd, sio->cte_idx);
		if (!newcte)
			return;
	} else {
		WPRINTK(sio, "drop invalid cte");
		newcte = NULL;
	}

	spin_lock_irq(&sio->cmd->lock);
	if (newcte) {
		oldcte = sio->cmd->te[sio->cte_idx];
		if (oldcte)
			*newcte = *oldcte;

		newcte->atime = jiffies;
		newcte->count++;
	}
	rcu_assign_pointer(sio->cmd->te[sio->cte_idx], newcte);
	spin_unlock_irq(&sio->cmd->lock);
	if (oldcte)
		call_rcu(&oldcte->rcu, cte_reset);
}

static void sio_update_cte(struct ssdcache_io *sio, bool is_cache)
{
	struct ssdcache_te *newcte, *oldcte;

	BUG_ON(!sio);

	BUG_ON(!sio->cmd);
	BUG_ON(sio->cte_idx == -1);

	/* Check if we should drop the old cte */
	newcte = cte_new(sio->sc, sio->cmd, sio->cte_idx);
	if (!newcte)
		return;

	spin_lock_irq(&sio->cmd->lock);
	oldcte = sio->cmd->te[sio->cte_idx];
	if (oldcte)
		*newcte = *oldcte;

	newcte->atime = jiffies;
	newcte->count++;
	/* Reset busy bitmaps */
	if (is_cache)
		bitmap_andnot(newcte->cache_busy, newcte->cache_busy,
			      sio->bio_mask, DEFAULT_BLOCKSIZE);
	else
		bitmap_andnot(newcte->target_busy, newcte->target_busy,
			      sio->bio_mask, DEFAULT_BLOCKSIZE);

	/* Upon error always reset the clean bitmap */
	if (sio->error)
		bitmap_andnot(newcte->clean, newcte->clean, sio->bio_mask,
			      DEFAULT_BLOCKSIZE);
	/* And update the clean bitmap if it was a cache write */
	else if (is_cache) {
		bitmap_or(newcte->clean, newcte->clean, sio->bio_mask,
			  DEFAULT_BLOCKSIZE);
	}

	rcu_assign_pointer(sio->cmd->te[sio->cte_idx], newcte);
	spin_unlock_irq(&sio->cmd->lock);
	if (oldcte)
		call_rcu(&oldcte->rcu, cte_reset);
}

static void sio_invalidate(struct ssdcache_io *sio)
{
	struct ssdcache_te *oldcte;

	BUG_ON(!sio || sio->cte_idx < 0);

	spin_lock_irq(&sio->cmd->lock);
	oldcte = sio->cmd->te[sio->cte_idx];
	rcu_assign_pointer(sio->cmd->te[sio->cte_idx], NULL);
	spin_unlock_irq(&sio->cmd->lock);
	if (oldcte)
		call_rcu(&oldcte->rcu, cte_reset);
}

static void sio_start_cache_write(struct ssdcache_io *sio)
{
	struct ssdcache_te *newcte, *oldcte;

	if (!sio || !sio->cmd || sio->cte_idx == -1)
		return;

	/* Check if we should drop the old cte */
	newcte = cte_new(sio->sc, sio->cmd, sio->cte_idx);
	if (!newcte)
		return;

	spin_lock_irq(&sio->cmd->lock);
	oldcte = sio->cmd->te[sio->cte_idx];
	BUG_ON(!oldcte);
	*newcte = *oldcte;

	newcte->count++;
	bitmap_andnot(newcte->clean, oldcte->clean, sio->bio_mask,
		      DEFAULT_BLOCKSIZE);
	bitmap_or(newcte->cache_busy, oldcte->cache_busy, sio->bio_mask,
		   DEFAULT_BLOCKSIZE);
	if (sio->error == -EUCLEAN) {
		sio->error = 0;
	} else {
		bitmap_or(newcte->target_busy, oldcte->target_busy,
			  sio->bio_mask, DEFAULT_BLOCKSIZE);
	}
	newcte->atime = jiffies;

	rcu_assign_pointer(sio->cmd->te[sio->cte_idx], newcte);
	spin_unlock_irq(&sio->cmd->lock);
	call_rcu(&oldcte->rcu, cte_reset);
}

static void sio_start_writeback(struct ssdcache_io *sio)
{
	struct ssdcache_te *newcte, *oldcte;

	if (!sio || !sio->cmd || sio->cte_idx == -1)
		return;

	/* Check if we should drop the old cte */
	newcte = cte_new(sio->sc, sio->cmd, sio->cte_idx);
	if (!newcte)
		return;

	spin_lock_irq(&sio->cmd->lock);
	oldcte = sio->cmd->te[sio->cte_idx];
	BUG_ON(!oldcte);
	*newcte = *oldcte;

	newcte->count++;
	bitmap_andnot(newcte->clean, oldcte->clean, sio->bio_mask,
		      DEFAULT_BLOCKSIZE);
	bitmap_or(newcte->cache_busy, oldcte->cache_busy, sio->bio_mask,
		   DEFAULT_BLOCKSIZE);
	newcte->atime = jiffies;

	rcu_assign_pointer(sio->cmd->te[sio->cte_idx], newcte);
	spin_unlock_irq(&sio->cmd->lock);
	call_rcu(&oldcte->rcu, cte_reset);
}

static void sio_start_target_write(struct ssdcache_io *sio)
{
	struct ssdcache_te *newcte, *oldcte;

	if (!sio || !sio->cmd || sio->cte_idx == -1)
		return;

	/* Check if we should drop the old cte */
	newcte = cte_new(sio->sc, sio->cmd, sio->cte_idx);
	if (!newcte)
		return;

	spin_lock_irq(&sio->cmd->lock);
	oldcte = sio->cmd->te[sio->cte_idx];
	BUG_ON(!oldcte);
	*newcte = *oldcte;

	newcte->count++;
	bitmap_or(newcte->target_busy, oldcte->target_busy, sio->bio_mask,
		  DEFAULT_BLOCKSIZE);
	newcte->atime = jiffies;

	rcu_assign_pointer(sio->cmd->te[sio->cte_idx], newcte);
	spin_unlock_irq(&sio->cmd->lock);
	call_rcu(&oldcte->rcu, cte_reset);
}

/*
 * Workqueue handling
 */

static struct ssdcache_io *ssdcache_create_sio(struct ssdcache_ctx *sc)
{
	struct ssdcache_io *sio;

	sio = mempool_alloc(_sio_pool, GFP_NOIO);
	if (!sio)
		return NULL;
	memset(sio, 0, sizeof(struct ssdcache_io));
	sio->sc = sc;
	sio->cte_idx = -1;
	sio->nr = ++sc->nr_sio;
	sc->sio_active++;
	kref_init(&sio->kref);
	INIT_LIST_HEAD(&sio->list);
	spin_lock_init(&sio->lock);
	return sio;
}

static void ssdcache_destroy_sio(struct kref *kref)
{
	struct ssdcache_io *sio = container_of(kref, struct ssdcache_io, kref);

	BUG_ON(!list_empty(&sio->list));

	if (sio->bio) {
		struct bio_vec *bvec;
		int i;

		bio_for_each_segment(bvec, sio->bio, i)
			put_page(bvec->bv_page);
		bio_put(sio->bio);
		sio->bio = NULL;
	}
	if (sio_match_sector(sio) &&
	    !sio_cache_is_busy(sio) &&
	    !sio_target_is_busy(sio))
		sio_cleanup_cte(sio);
	sio->sc->sio_active--;
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

static inline void push_sio(struct list_head *q, struct ssdcache_io *sio)
{
	unsigned long flags;

	ssdcache_get_sio(sio);
	spin_lock_irqsave(&_work_lock, flags);
	list_add_tail(&sio->list, q);
	spin_unlock_irqrestore(&_work_lock, flags);
}

static void ssdcache_schedule_sio(struct ssdcache_io *sio)
{
	push_sio(&_io_work, sio);
	queue_work(_ssdcached_wq, &_ssdcached_work);
}

static void unmap_writeback_bio(struct ssdcache_io *sio)
{
	int i;
	struct bio_vec *bvec;

	if (sio->writeback_bio) {
		/* Release bio */
		bio_for_each_segment(bvec, sio->writeback_bio, i)
			put_page(bvec->bv_page);
		bio_put(sio->writeback_bio);
		sio->writeback_bio = NULL;
	}
}

static void cache_io_callback(unsigned long error, void *context)
{
	struct ssdcache_io *sio = context;

	if (!sio_match_sector(sio)) {
		WPRINTK(sio, "cte overrun, not updating state");
	} else if (!sio_cache_is_busy(sio)) {
		WPRINTK(sio, "cte not busy, not updating state");
	} else {
		if (error || sio->error) {
			WPRINTK(sio, "finished with %lu", error);
			sio->error = error;
		}
		sio_update_cte(sio, true);
	}
	unmap_writeback_bio(sio);

	ssdcache_put_sio(sio);
	queue_work(_ssdcached_wq, &_ssdcached_work);
}

static void target_io_callback(unsigned long error, void *context)
{
	struct ssdcache_io *sio = context;

	if (!sio->cmd) {
#ifdef SSD_DEBUG
		DPRINTK("%lu: %s: cte lookup not finished",
			sio->nr, __FUNCTION__);
#endif
		if (!sio->error)
			sio->error = -EUCLEAN;
		sio->sc->cache_overruns++;
	} else if (!sio_match_sector(sio)) {
		WPRINTK(sio, "cte overrun, not updating state");
	} else if (!sio_target_is_busy(sio)) {
		WPRINTK(sio, "cte not busy, not updating state");
	} else {
		if (error || sio->error) {
			WPRINTK(sio, "finished with %lu", error);
			sio->error = error;
		}
		sio_update_cte(sio, false);
	}

	ssdcache_put_sio(sio);
	queue_work(_ssdcached_wq, &_ssdcached_work);
}

static inline sector_t to_cache_sector(struct ssdcache_io *sio,
				       sector_t data_sector)
{
	sector_t sector_offset, cte_offset, cmd_offset, cache_sector;

	sector_offset = data_sector & sio->sc->block_mask;

	BUG_ON(sio->cte_idx < 0);
	cte_offset = to_sector(sio->cte_idx) * sio->sc->block_size;
	cmd_offset = to_sector(sio->cmd->hash * sio->cmd->num_cte) * sio->sc->block_size;
	cache_sector = sio->sc->data_offset + cmd_offset + cte_offset + sector_offset;

	if (cache_sector > i_size_read(sio->sc->cache_dev->bdev->bd_inode)) {
		WPRINTK(sio, "access beyond end of device");
		BUG();
	}
	return cache_sector;
}

static void write_to_cache(struct ssdcache_io *sio, struct bio *bio)
{
	struct dm_io_region cache;
	struct dm_io_request iorq;

	cache.bdev = sio->sc->cache_dev->bdev;
	cache.sector = to_cache_sector(sio, bio->bi_sector);
	cache.count = bio_sectors(bio);

	iorq.bi_rw = WRITE;
	iorq.mem.type = DM_IO_BVEC;
	iorq.mem.ptr.bvec = bio_iovec(bio);
	iorq.notify.fn = cache_io_callback;
	iorq.notify.context = sio;
	iorq.client = sio->sc->iocp;

	dm_io(&iorq, 1, &cache, NULL);
}

static void write_to_target(struct ssdcache_io *sio, struct bio *bio)
{
	struct dm_io_region target;
	struct dm_io_request iorq;

	target.bdev = sio->sc->target_dev->bdev;
	target.sector = bio->bi_sector;
	target.count = bio_sectors(bio);

	iorq.bi_rw = WRITE;
	iorq.mem.type = DM_IO_BVEC;
	iorq.mem.ptr.bvec = bio_iovec(bio);
	iorq.notify.fn = target_io_callback;
	iorq.notify.context = sio;
	iorq.client = sio->sc->iocp;

	dm_io(&iorq, 1, &target, NULL);
}

static void sio_start_prefetch(struct ssdcache_io *sio, struct bio *bio)
{
	struct bio_vec *bvec;
	int i;

	/* Setup clone for writing to cache device */
	sio->writeback_bio = bio_clone(bio, GFP_NOWAIT);
	sio->writeback_bio->bi_rw |= WRITE;
	bio_for_each_segment(bvec, sio->writeback_bio, i)
		get_page(bvec->bv_page);
	sio->sc->cache_misses++;
	bio->bi_bdev = sio->sc->target_dev->bdev;
}

static void sio_start_read_hit(struct ssdcache_io *sio, struct bio *bio)
{
	sio->sc->cache_hits++;
	bio->bi_bdev = sio->sc->cache_dev->bdev;
	bio->bi_sector = to_cache_sector(sio, bio->bi_sector);
}

static void sio_start_busy(struct ssdcache_io *sio, struct bio *bio)
{
	sio->sc->cache_busy++;
	bio->bi_bdev = sio->sc->target_dev->bdev;
}

static void sio_start_write_miss(struct ssdcache_io *sio, struct bio *bio)
{
	bio->bi_bdev = sio->sc->cache_dev->bdev;
	bio->bi_sector = to_cache_sector(sio, bio->bi_sector);
}

static bool sio_check_writeback(struct ssdcache_io *sio)
{
	struct ssdcache_te *cte;

	BUG_ON(!sio);
	BUG_ON(!sio->cmd);
	BUG_ON(sio->cte_idx == -1);

	rcu_read_lock();
	cte = rcu_dereference(sio->cmd->te[sio->cte_idx]);
	rcu_read_unlock();
	if (!cte) {
		WPRINTK(sio, "invalid cte");
		return false;
	}
	if (cte->sector != sio->bio_sector) {
		WPRINTK(sio, "wrong sector %llx %llx",
			(unsigned long long)cte->sector,
			(unsigned long long)sio->bio_sector);
		return false;
	}

	/* Check if there is an outstanding cache write */
	if (cte_cache_is_busy(cte, sio->bio_mask)) {
		WPRINTK(sio, "cache sector busy");
		return false;
	}
	/* Check if there is an outstanding target write */
	/*
	 * Danger Will Robinson
	 * There is no guarantee that two writes to the same
	 * block are handled separate.
	 * Need to check the block layer details here.
	 */
	if (cte_target_is_busy(cte, sio->bio_mask)) {
		WPRINTK(sio, "target sector busy");
		return false;
	}

	return true;
}
/*
 * Cache lookup and I/O handling
 */

static unsigned long ssdcache_hash_64(struct ssdcache_ctx *sc, sector_t sector)
{
	unsigned long value, hash_number, offset, hash_mask, sector_shift;

	sector_shift = fls(sc->block_size) - 1;
	hash_mask = (1UL << sc->hash_shift) - 1;
	value = sector >> (sc->hash_shift + sector_shift);
	offset = sector & hash_mask;
	hash_number = (unsigned long)hash_64(value, sc->hash_bits - sc->hash_shift);

	return (hash_number << sc->hash_shift) | offset;
}

static unsigned long ssdcache_hash_wrap(struct ssdcache_ctx *sc, sector_t sector)
{
	unsigned long sector_shift, value, hash_mask;

	sector_shift = fls(sc->block_size) - 1;
	value = sector >> (sc->hash_shift + sector_shift);
	hash_mask = (1UL << (sc->hash_bits - sc->hash_shift)) - 1;

	return value & hash_mask;
}

static unsigned long hash_block(struct ssdcache_ctx *sc, sector_t sector)
{
	if (default_cache_algorithm == CACHE_ALG_HASH64)
		return ssdcache_hash_64(sc, sector);
	else
		return ssdcache_hash_wrap(sc, sector);
}

static unsigned long rotate(struct ssdcache_ctx *sc, unsigned long value)
{
	unsigned long result, hash_mask;

	hash_mask = (1UL << (sc->hash_bits - sc->hash_shift)) - 1;
	result = (value << 2);
	result |= (value >> ((sc->hash_bits - sc->hash_shift) - 2));
	if (value == (result & hash_mask))
		result++;
	return result & hash_mask;
}

static enum cte_match_t cte_match(struct ssdcache_io *sio, int rw)
{
	unsigned long hash_number;
	unsigned long cte_atime, oldest_atime;
	unsigned long cte_count, oldest_count;
	int invalid, oldest, i, index, busy = 0, assoc = 1;
	enum cte_match_t retval = CTE_LOOKUP_FAILED;

	hash_number = hash_block(sio->sc, sio->bio_sector);

retry:
	oldest_atime = jiffies;
	oldest_count = -1;
	oldest = -1;
	invalid = -1;
	index = -1;

	/* Lookup cmd */
	sio->cmd = cmd_lookup(sio->sc, hash_number);
	if (!sio->cmd) {
		if (sio->error == -EUCLEAN) {
			/* Target write already completed */
			return CTE_WRITE_DONE;
		}
		sio->cmd = cmd_insert(sio->sc, hash_number);
		if (!sio->cmd) {
			DPRINTK("%lu: %s: cmd insertion failure",
				sio->nr, __FUNCTION__);
			sio->sc->cache_failures++;
			retval = CTE_LOOKUP_FAILED;
			goto out;
		} else {
#ifdef SSD_DEBUG
			DPRINTK("%lu: %s (cte %lx:0): use first clean entry",
				sio->nr, __FUNCTION__, sio->cmd->hash);
#endif
			/* Clean cmd, first entry is useable */
			invalid = 0;
			/* Skip cte lookup */
			goto found;
		}
	}

	for (i = 0; i < sio->cmd->num_cte; i++) {
		struct ssdcache_te *cte;

		rcu_read_lock();
		cte = rcu_dereference(sio->cmd->te[i]);
		rcu_read_unlock();
		if (!cte) {
			if (invalid == -1)
				invalid = i;
			continue;
		}
		if (cte->sector == sio->bio_sector) {
			sio->cte_idx = i;
			if (cte_is_clean(cte, sio->bio_mask)) {
				/* Clean cte */
				if (rw == WRITE) {
					sio_start_cache_write(sio);
					retval = CTE_WRITE_CLEAN;
				} else {
					retval = CTE_READ_CLEAN;
				}
			} else if (cte_target_is_busy(cte, sio->bio_mask) ||
				   cte_cache_is_busy(cte, sio->bio_mask)) {
				/* Busy cte */
				if (rw == WRITE) {
					retval = CTE_WRITE_BUSY;
				} else {
					retval = CTE_READ_BUSY;
				}
			} else {
				/* Invalid cte sector */
				if (rw == WRITE) {
					sio_start_cache_write(sio);
					retval = CTE_WRITE_INVALID;
				} else {
					retval = CTE_READ_INVALID;
				}
			}
			goto out;
		}
		/* Break out if we have found an invalid entry */
		if (invalid != -1 || sio->error == -EUCLEAN)
			break;
		/* Can only eject CLEAN entries */
		if (!cte_is_clean(cte, sio->bio_mask) ||
		    cte_target_is_busy(cte, sio->bio_mask) ||
		    cte_cache_is_busy(cte, sio->bio_mask)) {
#ifdef SSD_DEBUG
			DPRINTK("%lu: %s (cte %lx:%x): skip not-clean cte",
				sio->nr, __FUNCTION__, sio->cmd->hash, i);
#endif
			busy++;
			continue;
		}
		if (sio->sc->cache_strategy == CACHE_LRU) {
			/* Select the oldest clean entry */
			rcu_read_lock();
			cte_atime = rcu_dereference(cte)->atime;
			rcu_read_unlock();
			if (time_before_eq(cte_atime, oldest_atime)) {
				oldest_atime = cte_atime;
				oldest = i;
			}
		} else {
			/* Select the lowest access count */
			rcu_read_lock();
			cte_count = rcu_dereference(cte)->count;
			rcu_read_unlock();
			if (cte_count <= oldest_count) {
				oldest_count = cte_count;
				oldest = i;
			}
		}
	}
	if (invalid == -1 && assoc > 0) {
		hash_number = rotate(sio->sc, hash_number);
		assoc--;
#ifdef SSD_DEBUG
		DPRINTK("%s (cte %lx:ff): retry with assoc %d", __FUNCTION__,
			sio->cmd->hash, assoc);
#endif
		goto retry;
	}
found:
	if (invalid != -1) {
		index = invalid;
	} else if (oldest != -1) {
#ifdef SSD_DEBUG
		DPRINTK("%lu: %s (cte %lx:%x): drop oldest cte", sio->nr,
			__FUNCTION__, sio->cmd->hash, oldest);
#endif
		sio->sc->cache_evictions++;
		index = oldest;
	} else {
		index = -1;
		sio->error - 0;
	}

	if (index != -1) {
		struct ssdcache_te *oldcte, *newcte;

		newcte = cte_new(sio->sc, sio->cmd, index);
		if (!newcte) {
			/* Ouch */
			sio->cte_idx = -1;
			DPRINTK("%lu: %s (cte %lx:ff): oom", sio->nr,
				__FUNCTION__, sio->cmd->hash);
			return retval;
		}
		spin_lock_irq(&sio->cmd->lock);
		oldcte = sio->cmd->te[index];
		sio->cte_idx = index;
		if (rw == WRITE) {
			bitmap_or(newcte->cache_busy, newcte->cache_busy,
				  sio->bio_mask, DEFAULT_BLOCKSIZE);
			bitmap_or(newcte->target_busy, newcte->target_busy,
				  sio->bio_mask, DEFAULT_BLOCKSIZE);
			retval = CTE_WRITE_MISS;
		} else {
			retval = CTE_READ_MISS;
		}
		newcte->sector = sio->bio_sector;
		rcu_assign_pointer(sio->cmd->te[index], newcte);
		spin_unlock_irq(&sio->cmd->lock);
		if (oldcte)
			call_rcu(&oldcte->rcu, cte_reset);
	} else {
#ifdef SSD_DEBUG
		DPRINTK("%lu: %s (cte %lx:ff): %d ctes busy", sio->nr,
			__FUNCTION__, sio->cmd->hash, busy);
#endif
		sio->cte_idx = -1;
	}
out:
	return retval;
}

static void process_sio(struct work_struct *ignored)
{
	LIST_HEAD(tmp);
	LIST_HEAD(defer);
	unsigned long flags;
	struct ssdcache_io *sio, *next;
	enum cte_match_t ret = CTE_LOOKUP_FAILED;

	spin_lock_irqsave(&_work_lock, flags);
	list_splice_init(&_io_work, &tmp);
	spin_unlock_irqrestore(&_work_lock, flags);
	list_for_each_entry_safe(sio, next, &tmp, list) {
		list_del_init(&sio->list);
		if (sio->bio) {
			/* secondary write */
			if (sio->sc->cache_mode == CACHE_IS_WRITETHROUGH) {
				ret = CTE_LOOKUP_FAILED;
				if (!sio->cmd || sio->cte_idx == -1)
					ret = cte_match(sio, WRITE);
				switch (ret) {
				case CTE_WRITE_INVALID:
				case CTE_WRITE_CLEAN:
				case CTE_WRITE_MISS:
					ssdcache_get_sio(sio);
					write_to_cache(sio, sio->bio);
					break;
				case CTE_WRITE_BUSY:
					WPRINTK(sio, "cte busy for write");
					sio->error = -EBUSY;
					sio->sc->cache_busy++;
					break;
				case CTE_WRITE_DONE:
					DPRINTK("%lu: %s: cte already done",
						sio->nr, __FUNCTION__);
					break;
				case CTE_WRITE_ERROR:
					WPRINTK(sio, "cte target write fail");
					break;
				default:
					WPRINTK(sio, "invalid cte lookup %d",
						ret);
					sio->error = -ENOENT;
					sio->sc->cache_overruns++;
				}
			} else {
				ssdcache_get_sio(sio);
				sio_start_target_write(sio);
				write_to_target(sio, sio->bio);
			}
		} else if (sio->writeback_bio) {
			if (!sio_check_writeback(sio)) {
				WPRINTK(sio, "cancel writeback");
				unmap_writeback_bio(sio);
				sio->error = -ENOENT;
				sio->sc->cache_overruns++;
			} else {
				/* Start writing to cache device */
				ssdcache_get_sio(sio);
				sio_start_writeback(sio);
				write_to_cache(sio, sio->writeback_bio);
			}
		} else {
			WPRINTK(sio, "invalid workqueue state");
		}
		ssdcache_put_sio(sio);
	}
	spin_lock_irqsave(&_work_lock, flags);
	list_splice(&tmp, &_io_work);
	spin_unlock_irqrestore(&_work_lock, flags);
}

static int ssdcache_map(struct dm_target *ti, struct bio *bio,
		      union map_info *map_context)
{
	struct ssdcache_ctx *sc = ti->private;
	struct ssdcache_io *sio;

	if (bio->bi_rw & REQ_FLUSH) {
		bio->bi_bdev = sc->target_dev->bdev;
		map_context->ptr = NULL;
		return DM_MAPIO_REMAPPED;
	}

	if (bio_cur_bytes(bio) > to_bytes(sc->block_size)) {
		DPRINTK("bio size %u larger than block size",
			bio_cur_bytes(bio));
		sc->cache_bypassed++;
		bio->bi_bdev = sc->target_dev->bdev;
		map_context->ptr = NULL;
		return DM_MAPIO_REMAPPED;
	}

	if (bio_cur_bytes(bio) == 0) {
		DPRINTK("zero-sized bio (flags %lx)", bio->bi_flags);
		sc->cache_bypassed++;
		bio->bi_bdev = sc->target_dev->bdev;
		map_context->ptr = NULL;
		return DM_MAPIO_REMAPPED;
	}

	sio = ssdcache_create_sio(sc);
	if (!sio) {
		DPRINTK("sio creation failure");
		sio->sc->cache_failures++;
		bio->bi_bdev = sc->target_dev->bdev;
		ssdcache_put_sio(sio);
		return DM_MAPIO_REMAPPED;
	}
	sio->bio_sector = cte_bio_align(sc, bio);
	sio_bio_mask(sio, bio);
	map_context->ptr = sio;
	if (bio_data_dir(bio) == WRITE) {
		struct bio_vec *bvec;
		int i;

		/* Kick off secondary writes */
		sio->bio = bio_clone(bio, GFP_NOWAIT);
		BUG_ON(!sio->bio);
		bio_for_each_segment(bvec, sio->bio, i)
			get_page(bvec->bv_page);
		ssdcache_schedule_sio(sio);
		if (sc->cache_mode == CACHE_IS_WRITETHROUGH) {
			bio->bi_bdev = sc->target_dev->bdev;
			return DM_MAPIO_REMAPPED;
		}
	}
	switch (cte_match(sio, bio_data_dir(bio))) {
	case CTE_READ_CLEAN:
		/* Cache hit, cte clean */
#ifdef SSD_DEBUG
		WPRINTK(sio, "read hit clean %llx %u",
			(unsigned long long)bio->bi_sector,
			bio_cur_bytes(bio));
#endif
		sio_start_read_hit(sio, bio);
		break;
	case CTE_READ_BUSY:
		WPRINTK(sio, "read hit busy %llx %u",
			(unsigned long long)bio->bi_sector,
			bio_cur_bytes(bio));
		/* Do not start prefetching here, sector is already busy */
		sc->cache_busy++;
		bio->bi_bdev = sc->target_dev->bdev;
		map_context->ptr = NULL;
		ssdcache_put_sio(sio);
		break;
	case CTE_READ_INVALID:
#ifdef SSD_DEBUG
		WPRINTK(sio, "read invalid %llx %u",
			(unsigned long long)bio->bi_sector,
			bio_cur_bytes(bio));
#endif
		sio_start_prefetch(sio, bio);
		break;
	case CTE_READ_MISS:
#ifdef SSD_DEBUG
		WPRINTK(sio, "read miss %llx %u",
			(unsigned long long)bio->bi_sector,
			bio_cur_bytes(bio));
#endif
		sio_start_prefetch(sio, bio);
		break;
	case CTE_LOOKUP_FAILED:
		WPRINTK(sio, "lookup failure %llx %u",
			(unsigned long long)bio->bi_sector,
			bio_cur_bytes(bio));
		sc->cache_bypassed++;
		bio->bi_bdev = sc->target_dev->bdev;
		map_context->ptr = NULL;
		ssdcache_put_sio(sio);
		break;
	case CTE_WRITE_DONE:
		/* Write to target already completed */
		DPRINTK("%lu: %s: write target done", sio->nr, __FUNCTION__);
		map_context->ptr = NULL;
		bio_endio(bio, sio->error);
		ssdcache_put_sio(sio);
		return DM_MAPIO_SUBMITTED;
		break;
	case CTE_WRITE_ERROR:
		WPRINTK(sio, "write target failure");
		ssdcache_put_sio(sio);
		return sio->error;
		break;
	case CTE_WRITE_BUSY:
		WPRINTK(sio, "write hit busy %llx %u",
			(unsigned long long)bio->bi_sector,
			bio_cur_bytes(bio));
		sio_start_busy(sio, bio);
		break;
	case CTE_WRITE_INVALID:
#ifdef SSD_DEBUG
		WPRINTK(sio, "write hit invalid %llx %u",
			(unsigned long long)bio->bi_sector,
			bio_cur_bytes(bio));
#endif
		sio_start_write_miss(sio, bio);
		break;
	case CTE_WRITE_CLEAN:
#ifdef SSD_DEBUG
		WPRINTK(sio, "write hit clean %llx %u",
			(unsigned long long)bio->bi_sector,
			bio_cur_bytes(bio));
#endif
		sio_start_write_miss(sio, bio);
		break;
	case CTE_WRITE_MISS:
#ifdef SSD_DEBUG
		WPRINTK(sio, "write miss %llx %u",
			(unsigned long long)bio->bi_sector,
			bio_cur_bytes(bio));
#endif
		sio_start_write_miss(sio, bio);
		break;
	}
	return DM_MAPIO_REMAPPED;
}

static int ssdcache_endio(struct dm_target *ti, struct bio *bio,
			  int error, union map_info *map_context)
{
	struct ssdcache_io *sio = map_context->ptr;

	if (!sio)
		return error;

	if (!sio->cmd) {
#ifdef SSD_DEBUG
		DPRINTK("%lu: %s: cte lookup not finished",
			sio->nr, __FUNCTION__);
#endif
		sio->sc->cache_overruns++;
		sio->error = -EUCLEAN;
		goto out;
	}
	if (error || sio->error) {
		WPRINTK(sio, "finished with %u", error);
		sio->error = error;
		unmap_writeback_bio(sio);
	}

	if (bio_data_dir(bio) == WRITE) {
		bool to_cache = bio->bi_bdev == sio->sc->cache_dev->bdev;
		if (!sio_match_sector(sio)) {
			WPRINTK(sio, "cte overrun, not updating state");
		} else if (to_cache) {
			if (!sio_cache_is_busy(sio)) {
				WPRINTK(sio, "cte not busy, not updating");
			} else {
				sio_update_cte(sio, to_cache);
			}
		} else {
			if (!sio_target_is_busy(sio)) {
				WPRINTK(sio, "cte not busy, not updating");
			} else {
				sio_update_cte(sio, to_cache);
			}
		}
	} else if (sio->writeback_bio) {
		/* Kick off writeback */
		ssdcache_schedule_sio(sio);
	}
out:
	ssdcache_put_sio(sio);

	return error;
}


/*
 * Construct a ssdcache mapping: <target_dev_path> <cache_dev_path>
 */
static int ssdcache_ctr(struct dm_target *ti, unsigned int argc, char **argv)
{
	struct ssdcache_ctx *sc;
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
		if (sc->block_size < 1) {
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

	num_cte = cdev_size / to_bytes(sc->block_size) / DEFAULT_ASSOCIATIVITY;

	/*
	 * Hash bit calculation might return a lower number
	 * for the possible number of ctes, so adjust that
	 * as well.
	 */
	sc->hash_bits = fls(num_cte) - 1;
	num_cte = (1UL << sc->hash_bits);
	sc->hash_shift = 0;

	DPRINTK("block size %lu, hash bits %lu/%lu, num cte %lu",
		to_bytes(sc->block_size), sc->hash_bits, sc->hash_shift,
		num_cte);

	INIT_RADIX_TREE(&sc->md_tree, GFP_ATOMIC);

	sc->data_offset = 0;
	sc->block_mask = sc->block_size - 1;
	sc->nr_sio = 0;
	ti->num_flush_requests = 1;
	ti->num_discard_requests = 1;
	ti->private = sc;
	ti->split_io = sc->block_size;
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
	struct ssdcache_ctx *sc = (struct ssdcache_ctx *) ti->private;
	struct ssdcache_te *cte;
	unsigned long pos = 0, nr_cmds;
	struct ssdcache_md *cmds[MIN_CMD_NUM], *cmd;
	int i, j;

	do {
		nr_cmds = radix_tree_gang_lookup(&sc->md_tree,
						 (void **)cmds, pos,
						 MIN_CMD_NUM);
		for (i = 0; i < nr_cmds; i++) {
			pos = cmds[i]->hash;
			cmd = radix_tree_delete(&sc->md_tree, pos);
			spin_lock_irq(&cmd->lock);
			for (j = 0; j < cmd->num_cte; j++) {
				if (cmd->te[j]) {
					cte = cmd->te[j];
					rcu_assign_pointer(cmd->te[j], NULL);
					spin_unlock_irq(&cmd->lock);
					synchronize_rcu();
					mempool_free(cte, _cte_pool);
					spin_lock_irq(&cmd->lock);
				}
			}
			spin_unlock_irq(&cmd->lock);
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
	struct ssdcache_ctx *sc = (struct ssdcache_ctx *) ti->private;
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
			 "cache misses %lu hits %lu busy %lu overruns %lu "
			 "bypassed %lu evicts %lu failures %lu sio %lu cte %lu",
			 nr_cmds, (1UL << sc->hash_bits), nr_ctes,
			 (1UL << sc->hash_bits) * DEFAULT_ASSOCIATIVITY,
			 sc->cache_misses, sc->cache_hits, sc->cache_busy,
			 sc->cache_overruns, sc->cache_bypassed,
			 sc->cache_evictions, sc->cache_failures,
			 sc->sio_active, sc->cte_active - nr_ctes);
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

static int ssdcache_iterate_devices(struct dm_target *ti,
				  iterate_devices_callout_fn fn, void *data)
{
	struct ssdcache_ctx *sc = ti->private;

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
	INIT_WORK(&_ssdcached_work, process_sio);

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
